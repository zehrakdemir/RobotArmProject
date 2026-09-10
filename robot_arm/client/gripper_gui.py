#!/usr/bin/env python3
"""gripper_gui.py - Jodell EPG40-050 gripper icin basit Tkinter arayuzu.

Slider'larla pozisyon/hiz/kuvvet ayarlayip "Uygula" ile gripper'i hareket
ettirir. (Slider'i surukledikce CANLI hareket ettirme denendi - donanim
[RS485 hatti] onceki hareketi fiziksel olarak bitirmeden gelen yeni hedefi
duzgun isleyemedigi icin "az hareket + uzun donme" seklinde takiliyordu,
hicbir zamanlama/kilit ayari bunu duzeltemedi; bu yuzden Uygula-tuslu
tasarima geri donuldu.) movegripper (gripper_interactive.py) ile ayni
GripperController sinifini kullanir, sadece arayuzu farkli.

Kullanim (bagimsiz, kendi RPC baglantisini acar):
    grippergui                      (alias)
    python3 gripper_gui.py

Kullanim (movecobot gibi baska bir scriptten, MEVCUT baglantiyi paylasarak -
ayni process icinde IKINCI bir Robot.RPC() acmak Fairino SDK'da sorunlu
oldugu icin bu onemli, bkz. gripper_controller.py'deki GripperController
docstring'i). movecobot artik bunu KENDISI, kendi arka plan thread'inde,
AYNI gripper (ve dolayisiyla ayni sdk + ayni kilit) ile otomatik aciyor:
    from gripper_gui import run_gripper_gui
    run_gripper_gui(existing_gripper_controller, standalone=True)  # kendi Tk() koku, arka plan thread'inde

UYARI: Bu GUI'yi movecobot ya da movegripper ile AYNI ANDA, AYRI birer
terminal/process olarak calistirma - ayni robota iki ayri baglantidan
komut gonderilmesi (Fairino RPC'nin gercek zamanli tek-tuketicili soketi
yuzunden) bagliliklari bozabilir. (movecobot kendi baslattigi GUI icin bu
sorunu yasamaz - AYNI GripperController nesnesini, AYNI kilitle paylasir.)
"""
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

from gripper_controller import GripperController, GripperError

MOTION_STATUS_TEXT = {
    0: "hareket devam ediyor",
    1: "bitti - nesne yok",
    2: "bitti - nesne var (kavradi)",
}


def run_gripper_gui(gripper, standalone=True, busy_event=None):
    """gripper: baglanti kurulmus bir GripperController. standalone=True ise
    pencere kapaninca process de sonlanir (mainloop() cagirir ve doner);
    False ise Toplevel olarak acilir (baska bir Tk uygulamasindan cagirmak
    icin, bu script su an bunu kullanmiyor ama gelecekte movecobot'a
    gomulmek istenirse hazir).

    busy_event: verilirse (movecobot.py'nin CobotArm.executing'i), arka
    plan durum sorgulama thread'i bu event set edilmisken KENDI SDK
    cagrisini atlar - kol GERCEK bir hareketi calistirirken (execute())
    movecobot'un kendi baglantisindan ES ZAMANLI baska bir istek gitmesin
    diye. Canli testte, aktif calistirma sirasinda robotun gercek-zamanli
    komut kanalinin iki farkli baglantidan gelen es zamanli trafigi
    kaldiramadigi (surekli ServoJ err=14) gozlemlendi - bu, o riski
    movecobot'un KENDI kontrol edebildigi taraftan (gripper polling)
    azaltmaya calisir."""
    root = tk.Tk() if standalone else tk.Toplevel()
    root.title(f"Gripper Kontrol - {gripper.robot_ip or 'paylasilan baglanti'}")
    root.geometry("380x500")

    status_queue = queue.Queue()
    stop_polling = threading.Event()

    main = ttk.Frame(root, padding=12)
    main.pack(fill="both", expand=True)

    # --- Durum paneli -------------------------------------------------
    status_frame = ttk.LabelFrame(main, text="Durum", padding=8)
    status_frame.pack(fill="x", pady=(0, 10))

    active_var = tk.StringVar(value="?")
    position_var = tk.StringVar(value="?")
    motion_var = tk.StringVar(value="?")
    fault_var = tk.StringVar(value="?")

    for label, var in [("Aktif mi:", active_var), ("Pozisyon:", position_var),
                        ("Hareket:", motion_var), ("Hata kodu:", fault_var)]:
        row = ttk.Frame(status_frame)
        row.pack(fill="x")
        ttk.Label(row, text=label, width=10).pack(side="left")
        ttk.Label(row, textvariable=var).pack(side="left")

    # --- Slider'lar -----------------------------------------------------
    pos_var = tk.IntVar(value=0)
    speed_var = tk.IntVar(value=50)
    force_var = tk.IntVar(value=50)

    def slider_row(parent, text, var, default, with_increment=False):
        frame = ttk.Frame(parent)
        frame.pack(fill="x", pady=4)
        ttk.Label(frame, text=text, width=10).pack(side="left")

        entry_var = tk.StringVar(value=str(default))

        def clamp(v):
            return max(0, min(100, v))

        def set_value(v):
            v = clamp(v)
            var.set(v)
            entry_var.set(str(v))

        def commit_entry(event=None):
            # Entry'ye yazilan deger Enter'a basinca ya da odak baska yere
            # gecince (FocusOut) uygulanir - gecersiz/araligin disindaki
            # deger sessizce en yakin gecerli degere cekilir.
            try:
                v = int(entry_var.get().strip())
            except ValueError:
                entry_var.set(str(var.get()))
                return
            set_value(v)

        def on_slider_move(value):
            entry_var.set(str(int(float(value))))

        if with_increment:
            ttk.Button(frame, text="▼", width=2, command=lambda: set_value(var.get() - 1)).pack(side="right", padx=(2, 0))
            ttk.Button(frame, text="▲", width=2, command=lambda: set_value(var.get() + 1)).pack(side="right", padx=(2, 0))

        entry = ttk.Entry(frame, textvariable=entry_var, width=4)
        entry.pack(side="right", padx=(4, 0))
        entry.bind("<Return>", commit_entry)
        entry.bind("<FocusOut>", commit_entry)

        scale = ttk.Scale(
            frame, from_=0, to=100, orient="horizontal", variable=var,
            command=on_slider_move,
        )
        scale.pack(side="left", fill="x", expand=True, padx=6)
        return scale, set_value

    sliders_frame = ttk.LabelFrame(main, text="Hedef (0-100, yazarak ya da surukleyerek degistir)", padding=8)
    sliders_frame.pack(fill="x", pady=(0, 10))
    # set_pos_value: slider'in HEDEF degerini disaridan (do_read_position'dan)
    # da guncelleyebilmek icin - gripper acilista/talep uzerine nerede
    # oldugunu okumadan slider hep 0'dan basliyordu, yani "Uygula"ya
    # dokunmadan basilirsa bile gripper beklenmedik sekilde 0'a (tam acik)
    # komut alabiliyordu. Bkz. do_read_position.
    _, set_pos_value = slider_row(sliders_frame, "Pozisyon", pos_var, 0, with_increment=True)
    slider_row(sliders_frame, "Hiz", speed_var, 50)
    slider_row(sliders_frame, "Kuvvet", force_var, 50)
    ttk.Label(sliders_frame, text="Pozisyon: 0=tam acik, 100=tam kapali",
              foreground="#666").pack(anchor="w", pady=(4, 0))

    # --- Aksiyon durumu ---------------------------------------------------
    action_status_var = tk.StringVar(value="Hazir.")
    ttk.Label(main, textvariable=action_status_var, foreground="#0a5").pack(fill="x", pady=(0, 6))

    def set_buttons_enabled(enabled):
        state = "normal" if enabled else "disabled"
        for b in action_buttons:
            b.configure(state=state)

    def run_in_background(label, fn):
        set_buttons_enabled(False)
        action_status_var.set(f"{label}...")

        def worker():
            try:
                custom_msg = fn()
                msg = custom_msg if custom_msg else f"{label}: tamam."
            except GripperError as e:
                msg = f"{label}: HATA - {e}"
            except Exception as e:
                msg = f"{label}: HATA - {type(e).__name__}: {e}"
            root.after(0, lambda: (action_status_var.set(msg), set_buttons_enabled(True)))

        threading.Thread(target=worker, daemon=True).start()

    def do_read_position():
        def action():
            st = gripper.status()
            root.after(0, lambda: set_pos_value(st["position_pct"]))
            return f"Poz. oku: gripperin gercek pozisyonu {st['position_pct']} - hedef buna esitlendi."
        run_in_background("Poz. oku", action)

    def do_apply():
        run_in_background(
            "Uygula",
            lambda: gripper.move(position=pos_var.get(), speed=speed_var.get(), force=force_var.get()),
        )

    def do_open():
        pos_var.set(0)
        run_in_background("Ac", lambda: gripper.move(position=0, speed=speed_var.get(), force=force_var.get()))

    def do_close():
        pos_var.set(100)
        run_in_background("Kapa", lambda: gripper.move(position=100, speed=speed_var.get(), force=force_var.get()))

    def do_grip():
        def action():
            result = gripper.grip(speed=speed_var.get(), force=force_var.get())
            root.after(0, lambda: pos_var.set(result["position_pct"]))
            if result["gripped"]:
                return f"Kavra: nesne yakalandi (pozisyon={result['position_pct']})."
            return f"Kavra: kapandi ama nesne algilanmadi (pozisyon={result['position_pct']})."
        run_in_background("Kavra", action)

    def do_activate():
        run_in_background("Aktive et", lambda: gripper.activate(force=True))

    def do_deactivate():
        run_in_background("Serbest birak", gripper.deactivate)

    # --- Butonlar --------------------------------------------------------
    btns = ttk.Frame(main)
    btns.pack(fill="x", pady=(0, 10))
    b_open = ttk.Button(btns, text="Ac (0)", command=do_open)
    b_close = ttk.Button(btns, text="Kapa (100)", command=do_close)
    b_apply = ttk.Button(btns, text="Uygula", command=do_apply)
    for b in (b_open, b_close, b_apply):
        b.pack(side="left", expand=True, fill="x", padx=3)

    btns_read = ttk.Frame(main)
    btns_read.pack(fill="x", pady=(0, 10))
    b_read_pos = ttk.Button(btns_read, text="Pozisyonu Robottan Oku (hedefi gercege esitle)", command=do_read_position)
    b_read_pos.pack(fill="x")

    btns_grip = ttk.Frame(main)
    btns_grip.pack(fill="x", pady=(0, 10))
    b_grip = ttk.Button(btns_grip, text="Kavra (nesneyi tut)", command=do_grip)
    b_grip.pack(fill="x")

    btns2 = ttk.Frame(main)
    btns2.pack(fill="x")
    b_activate = ttk.Button(btns2, text="Aktive Et", command=do_activate)
    b_deactivate = ttk.Button(btns2, text="Serbest Birak (elle acmak icin)", command=do_deactivate)
    for b in (b_activate, b_deactivate):
        b.pack(side="left", expand=True, fill="x", padx=3)

    action_buttons = [b_open, b_close, b_apply, b_read_pos, b_grip, b_activate, b_deactivate]

    # --- Arka planda durum sorgulama --------------------------------------
    def poll_status():
        while not stop_polling.is_set():
            if busy_event is not None and busy_event.is_set():
                # Kol su an GERCEK bir hareketi calistiriyor - bkz. run_gripper_gui
                # docstring'i. Bu dongude SDK'ya hic dokunmadan kisa kisa
                # bekleyip tekrar kontrol ediyoruz (busy_event kapaninca
                # normal 1sn'lik sorgulamaya geri donulur).
                stop_polling.wait(timeout=0.2)
                continue
            try:
                active = gripper.is_active()
                status = gripper.status()
                status_queue.put({"active": active, **status})
            except GripperError as e:
                status_queue.put({"error": str(e)})
            except Exception as e:
                # movecobot manuel/drag-teach moduna girerken sdk baglantisini
                # kapatip yeniden aciyor (bkz. movecobot.py enter_drag_teach) -
                # bu gecis sirasinda GripperError DISINDA bir hata (baglanti
                # kapali/degisti) gelebilir. Thread'i OLDURMEDEN (crash etmeden)
                # bir sonraki dongude tekrar dene - kilit (sdk_lock) zaten
                # gecisi movecobot'un ana thread'iyle sirali hale getiriyor.
                status_queue.put({"error": f"{type(e).__name__}: {e}"})
            time.sleep(1.0)

    def drain_status_queue():
        try:
            while True:
                s = status_queue.get_nowait()
                if "error" in s:
                    fault_var.set(f"okuma hatasi: {s['error']}")
                else:
                    active_var.set("evet" if s["active"] else "hayir")
                    position_var.set(f"{s['position_pct']}")
                    motion_var.set(MOTION_STATUS_TEXT.get(s["motion_status"], str(s["motion_status"])))
                    fault_var.set(str(s["fault"]))
        except queue.Empty:
            pass
        root.after(300, drain_status_queue)

    threading.Thread(target=poll_status, daemon=True).start()
    root.after(300, drain_status_queue)
    # Acilista slider'i (hedefi) gripperin GERCEK pozisyonuyla bir kez
    # senkronize et - aksi halde hedef hep 0'dan basliyordu (bkz. yukaridaki
    # set_pos_value yorumu).
    root.after(500, do_read_position)

    def on_close():
        stop_polling.set()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    if standalone:
        root.mainloop()
    return root


def main():
    print("Robot araniyor ve kavrayici hazirlaniyor...")
    try:
        gripper = GripperController()
    except Exception as e:
        print(f"HATA: baglanti kurulamadi: {e}")
        sys.exit(1)

    if not gripper.is_active():
        print("Aktive ediliyor...")
        try:
            gripper.activate()
        except GripperError as e:
            print(f"HATA: aktivasyon basarisiz: {e}")
            sys.exit(1)

    print("Hazir, pencere aciliyor...")
    run_gripper_gui(gripper, standalone=True)


if __name__ == "__main__":
    main()
