#!/usr/bin/env python3
"""Jodell EPG40-050 kavrayici kontrolu. movepy'nin (moveit_py_interactive.py)
kullandigi ayni ~/detect_robot.py mekanizmasiyla robotu bulur - IP elle
girilmiyor, robot hangi IP'deyse onu bulur.

ON KOSUL: WebApp'te Peripherals -> Gripper -> Custom protocol -> End_Jodell_V1.0
protokolu uygulanmis ve enable edilmis olmali; Device enable altinda Device ID=1
ve Initialization/Gripping trigger/Position/Speed/Torque + ilgili Status query'ler
isaretlenip Configure edilmis olmali (WebApp uzerinden birlikte yaptigimiz ayar).

Kullanim (modul olarak):
    from gripper_controller import GripperController

    with GripperController() as gripper:
        gripper.activate()
        gripper.open(speed=40, force=40)
        gripper.close(speed=40, force=40)
        print(gripper.status())

Kullanim (dogrudan test):
    python3 gripper_controller.py
"""
import re
import subprocess
import sys
import threading
import time

from fairino_cobot import paths

# SDK'nin yeri makineye gore degisiyor - tek yerden cozuluyor (paths.py).
paths.ensure_sdk_on_path()

from fairino import Robot

GRIPPER_INDEX = 2  # WebApp Custom Protocol > Device enable > Device ID ile ayni olmali


def detect_robot_ip():
    """movepy'nin kullandigi ayni ~/detect_robot.py scriptini calistirip
    ROBOT_IP'yi doner. Boylece IP hicbir yerde sabit kodlanmiyor - simulator
    (127.0.0.1) calisiyor olsa bile detect_robot.py gercek LAN robotunu
    onceliklendirip onu doner (bkz. discover_candidates() oncelik siralamasi)."""
    script_path = str(paths.detect_robot_script())

    result = subprocess.run(
        [sys.executable, script_path],
        stdout=subprocess.PIPE, stderr=None, text=True, timeout=120,
    )
    if result.returncode != 0 or not result.stdout.strip():
        raise RuntimeError(
            f"Robot algilanamadi (returncode={result.returncode}, stdout={result.stdout!r})"
        )

    info = {}
    for line in result.stdout.splitlines():
        m = re.match(r"^([A-Z_][A-Z0-9_]*)=(.*)$", line)
        if m:
            info[m.group(1)] = m.group(2)

    ip = info.get("ROBOT_IP")
    if not ip:
        raise RuntimeError(f"detect_robot.py ciktisinda ROBOT_IP yok: {result.stdout!r}")
    return ip


class GripperError(RuntimeError):
    pass


class GripperController:
    """Jodell EPG40-050 icin ince bir sarmalayici. fairino SDK'nin
    ActGripper/MoveGripper/GetGripperXXX cagrilarini kullanir; bunlar WebApp'te
    Custom Protocol olarak uygulanan End_Jodell_V1.0 Lua'sinin arkasindaki
    cihaza yonlendirilir (index = WebApp'teki Device ID)."""

    def __init__(self, index=GRIPPER_INDEX, robot_ip=None, sdk=None, lock=None):
        """sdk verilirse (baska bir yerde zaten acilmis bir Robot.RPC(...)
        nesnesi), YENI bir RPC baglantisi acmak yerine onu kullanir. Fairino
        SDK'nin RPC sinifi 20004 portunda tek-tuketicili bir realtime soket
        aciyor ve bazi durum degiskenlerini (is_conect, SDK_state ...) sinif
        seviyesinde (instance'lar arasi paylasimli) tutuyor; ayni process
        icinde IKINCI bir Robot.RPC(ip) acmak birinci baglantiyi (ör. kol
        kontrolunun kullandigi) bozabiliyor. Kol ve gripper'i ayni process'te
        birlikte kullanan kod (movecobot.py) bu yuzden tek bir sdk'yi
        paylasmali.

        lock verilirse (ör. movecobot'un CobotArm.sdk_lock'u), BUTUN
        self.robot.X(...) cagrilari bu kilitle korunur - xmlrpc.client.
        ServerProxy AYNI ANDA birden fazla thread'den cagrilmaya thread-safe
        DEGIL (ayni HTTP baglantisi uzerinden yaris durumu / cevaplarin
        karismasi riski var). movecobot artik gripper GUI'yi arka plan
        thread'inde ayni sdk'yla calistirdigi icin (canli slider hareketi +
        1sn'de bir durum sorgulama) bu, konsoldan yapilan islemlerle AYNI ANDA
        tetiklenebilir - kilit olmadan bu gercek bir risk. RLock kullaniliyor
        (Lock degil) cunku activate() -> is_active(), move() -> activate()
        gibi AYNI thread icinde ic ice cagrilar var; sade Lock bunda kilitlenir
        (deadlock)."""
        self.index = index
        self._lock = lock if lock is not None else threading.RLock()
        # Gripper iletisimi tamamen olduyse (bkz. _sdk) True olur; sonraki
        # cagrilar ANINDA hata verir, bir daha asili thread birakilmaz.
        self._dead = False
        if sdk is not None:
            self.robot_ip = robot_ip
            self.robot = sdk
        else:
            self.robot_ip = robot_ip or detect_robot_ip()
            self.robot = Robot.RPC(self.robot_ip)

    def _sdk(self, fn, *args, timeout_s=8.0, what="gripper komutu"):
        """Bir SDK cagrisini ZAMAN ASIMIYLA calistirir.

        NEDEN: gripper komutlari (ActGripper, MoveGripper, GetGripperMotionDone)
        robota XML-RPC ile gider, robot da bunlari RS485/Modbus uzerinden
        gripper'a iletir. Gripper cevap vermezse (guc yok, kablo cikmis,
        yanlis Device ID...) robot da XML-RPC yanitini HIC dondurmuyor ve
        xmlrpc istemcisinin varsayilan zaman asimi olmadigi icin cagri
        SONSUZA KADAR asili kaliyor. Canli testte tam bu yasandi: gripper
        cevapsizken movecobot acilista "Aktive ediliyor..." satirinda
        kilitlendi ve KOL DA kullanilamaz hale geldi - oysa kol tarafi
        gayet saglikliydi.

        movecobot'un baslangic kodu gripper hatalarini zaten yakalayip
        "sadece kol kontrolu" moduna geciyor; sorun cagrinin hata VERMEMESI,
        asili kalmasiydi. Bu sarmalayici cagriyi ayri bir daemon thread'de
        calistirip sureyi sinirlar, boylece hata firlatilabilir hale gelir.

        Zaman asiminda thread arka planda asili kalir (daemon, process ile
        birlikte olur) ve ayni ServerProxy'yi kullanmaya devam ettigi icin
        sonraki cagrilar guvenli degildir - bu yuzden self._dead isaretlenir
        ve sonraki cagrilar hic denenmeden hata verir."""
        if self._dead:
            raise GripperError(
                "gripper iletisimi kopuk (onceki bir komut yanitsiz kaldi) - "
                "gucu/RS485 kablosunu ve WebApp'teki Device ID'yi kontrol edip "
                "movecobot'u yeniden baslatin")
        box = {}

        def _run():
            try:
                box["v"] = fn(*args)
            except BaseException as e:  # noqa: BLE001 - thread'e ozel, aynen tasiniyor
                box["e"] = e

        t = threading.Thread(target=_run, daemon=True)
        t.start()
        t.join(timeout_s)
        if t.is_alive():
            self._dead = True
            raise GripperError(
                f"{what} {timeout_s:.0f} saniyede yanit vermedi - gripper robota "
                f"cevap vermiyor (guc, RS485 kablosu veya Device ID={self.index} "
                f"kontrol edilmeli)")
        if "e" in box:
            raise box["e"]
        return box["v"]

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass

    def is_active(self):
        with self._lock:
            err, fault, active = self.robot.GetGripperActivateStatus()
        if err != 0:
            raise GripperError(f"GetGripperActivateStatus basarisiz: err={err}")
        # DIKKAT: activate_status bitmap 0-index'li ama gripper index'i
        # 1-index'li (index=2 -> bit1, index=1 -> bit0). bit = index-1.
        return bool(active & (1 << (self.index - 1)))

    def activate(self, force=False, timeout_s=5.0):
        """Kavrayiciyi etkinlestirir. force=True ise zaten aktifse bile
        reset+activate dongusunu tekrar calistirir - Jodell manueline gore
        rACT bitini 0->1 gecirmek tum hata bayraklarini temizler."""
        if self.is_active() and not force:
            return
        with self._lock:
            # DIKKAT: burada eskiden self.robot.ResetAllError() cagriliyordu.
            # KALDIRILDI. ResetAllError GLOBAL bir robot resetidir ve robotun
            # SERVO OTURUMUNU (ros2_control'un ServoMoveStart ile kurdugu
            # akisi) sonlandiriyor. Sonucu: movecobot acilirken gripper
            # aktive edildigi anda surucunun ServoJ akisi oluyor ve ondan
            # sonra HER ServoJ ERR_SOCKET_COM_FAILED (-2) donuyor - yani
            # robot komut almiyor, tek care fairino_launch'i yeniden
            # baslatmak. "Kendiliginden err=-2 basladi" olarak gozlenen
            # sorunun kaynagi buydu (zamanlama birebir uyusuyor: movecobot
            # baslatildiktan ~15-60sn sonra, yani MoveItPy yuklenip gripper
            # aktivasyonuna gelindigi an). Ayrica canli testte ayni cagri
            # disaridan elle gonderilerek bu etki dogrudan dogrulandi.
            #
            # Zaten opsiyoneldi (try/except pass icindeydi); gripper'in kendi
            # hata bayraklarini temizleyen asil islem asagidaki rACT 0->1
            # gecisidir (ActGripper(index,0) -> ActGripper(index,1)).
            err = self._sdk(self.robot.ActGripper, self.index, 0,
                            what="Gripper reset (ActGripper 0)")
            if err != 0:
                raise GripperError(f"ActGripper(reset) basarisiz: err={err}")
        time.sleep(0.5)
        with self._lock:
            err = self._sdk(self.robot.ActGripper, self.index, 1,
                            what="Gripper aktivasyonu (ActGripper 1)")
            if err != 0:
                raise GripperError(f"ActGripper(activate) basarisiz: err={err}")

        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.is_active():
                return
            time.sleep(0.2)
        raise GripperError(f"Aktivasyon {timeout_s}s icinde tamamlanmadi.")

    def deactivate(self):
        """Gripper'i resetler (ActGripper(index, 0)) - activate()'in ilk yarisi
        ama YENIDEN aktive etmeden birakir. Cogu endustriyel gripper'da reset
        tutma torkunu keser, boylece ceneler elle acilip kapatilabilir hale
        gelir (movecobot'un manuel/drag-teach modu bunu kullanir). Jodell
        EPG40-050'nin bunu gercekten destekleyip desteklemedigi elle denenerek
        dogrulanmali - desteklemiyorsa gripper sadece mevcut pozisyonunda
        kalir, ama pozisyon okumasi (status()) yine de calisir."""
        with self._lock:
            err = self._sdk(self.robot.ActGripper, self.index, 0,
                            what="Gripper serbest birakma (ActGripper 0)")
        if err != 0:
            raise GripperError(f"ActGripper(reset) basarisiz: err={err}")

    def move(self, position, speed=50, force=50, max_time_ms=10000, blocking=True, retries=2):
        """position/speed/force: 0-100 yuzde. position=0 tam acik, 100 tam
        kapali. blocking=True ise MoveGripper cagrisi hareket bitene/timeout'a
        kadar donmez. RS485 hatti zaman zaman kararsiz oldugu icin (blue LED
        ara ara yanip sonuyor - iletisim kesintisi), basarisiz olursa
        'retries' kadar tekrar dener."""
        block_flag = 0 if blocking else 1
        last_err = None
        for attempt in range(1, retries + 2):
            with self._lock:
                # blocking=True iken MoveGripper hareket bitene kadar donmez,
                # yani mesru olarak max_time_ms kadar surebilir - zaman asimi
                # onun UZERINDE olmali, yoksa normal hareketi hata sayariz.
                err = self._sdk(
                    self.robot.MoveGripper,
                    self.index, position, speed, force, max_time_ms, block_flag,
                    0, 0, 0, 0,  # type=0 (paralel), rotNum/rotVel/rotTorque kullanilmiyor
                    timeout_s=(max_time_ms / 1000.0) + 5.0,
                    what="Gripper hareketi (MoveGripper)",
                )
            if err == 0:
                return
            last_err = err
            if attempt <= retries:
                # KRITIK: err=73 ("Gripper movement error") olustugunda
                # robotun KENDI hata kuyrugunda da (main_code=8, sub_code=1
                # - "Gripper motion timeout error") bir fault birikiyor.
                # Bu, GetGripperActivateStatus/fault register'indan tamamen
                # AYRI bir sistem - temizlenmeden birakilirsa, kavrayicinin
                # kendisi tamamen saglikli olsa bile robot SONRAKI TUM
                # MoveGripper cagrilarini reddediyor (bu da rastgele/aralikli
                # bir ariza gibi goruntu veriyordu). ResetAllError() bunu
                # temizler.
                # YAN ETKI (bilincli olarak kabul ediliyor): ResetAllError
                # GLOBAL bir resettir ve kolun SERVO OTURUMUNU da dusurur -
                # ros2_control'un ServoJ akisi bundan sonra -2 doner ve
                # surucu "SERVO AKISI KOPTU" hatasi basar (bkz. activate()
                # icindeki ayrintili yorum; orada bu cagri tam da bu yuzden
                # kaldirildi). Burada KALIYOR cunku alternatifi yok: bu fault
                # temizlenmezse gripper tamamen kullanilamaz hale geliyor.
                # Sadece gripper hareketi BASARISIZ oldugunda (yeniden deneme
                # yolunda) calisir, normal akista degil.
                print("UYARI: gripper hatasi temizleniyor (ResetAllError). Bu, kolun "
                      "servo akisini da dusurur - hareket komutlari calismazsa "
                      "'clearfault' ile kontrolu geri alin.")
                with self._lock:
                    try:
                        self.robot.ResetAllError()
                    except Exception:
                        pass
                try:
                    self.activate(force=True, timeout_s=3.0)
                except GripperError:
                    pass
                time.sleep(0.5)
        raise GripperError(f"MoveGripper basarisiz ({retries + 1} denemede de): err={last_err}")

    def open(self, speed=50, force=50, **kw):
        self.move(position=0, speed=speed, force=force, **kw)

    def close(self, speed=50, force=50, **kw):
        self.move(position=100, speed=speed, force=force, **kw)

    def grip(self, speed=50, force=50, max_time_ms=10000, retries=2):
        """'Gerektigi kadar kapan ve nesneyi tut': MoveGripper'i tam kapanma
        (position=100) hedefiyle cagirir, ama Jodell'in kendi firmware'i
        harekat sirasinda bir nesneyle karsilasirsa (direnc artisi) ORADA
        durup verilen force ile tutar - tam kapanmaya devam etmez. Yani
        hedef pozisyon nesnenin buyuklugune gore degisir, bu yuzden sabit bir
        'position' degeri olarak degil ayri bir eylem olarak ele alinmali.
        status()'un motion_status alanina gore (0=devam,1=bitti-nesne yok,
        2=bitti-nesne var) gercekten bir seyi kavrayip kavramadigini doner.

        DIKKAT: gripper PASIF (deactivate edilmis) durumdaysa MoveGripper
        yine de err=0 (basarili) donuyor ama gripper fiilen HIC HAREKET
        ETMIYOR - sessizce "kapandi ama nesne yok, pozisyon=0" gibi yanlis
        bir sonuc cikiyor (drag-teach modunda - orada gripper bilerek elle
        acilip kapanabilsin diye pasiflestiriliyor - bu sekilde yasandi).
        Bu yuzden kavramadan once gripper PASIFSE otomatik yeniden aktive
        ediliyor."""
        if not self.is_active():
            self.activate(force=True)
        self.move(position=100, speed=speed, force=force, max_time_ms=max_time_ms, retries=retries)
        # MoveGripper() blocking=True ile "hareket bitti" dese bile,
        # GetGripperCurPosition()'un iç durumu bir sonraki okumaya kadar
        # bayat kalabiliyor (bugün SDK'nin baska yerlerinde de gorulen bir
        # kalip) - "kapandi ama pozisyon=0 gosterdi" seklinde canli
        # gozlemlendi. Kisa bir bekleme ile bu riski azaltiyoruz.
        time.sleep(0.3)
        st = self.status()
        return {
            "gripped": st["motion_status"] == 2,
            "position_pct": st["position_pct"],
            "motion_status": st["motion_status"],
        }

    def status(self):
        """Debug/log icin okunabilir bir durum ozeti doner."""
        with self._lock:
            _, motion_pair = self._sdk(self.robot.GetGripperMotionDone,
                                       what="Gripper durumu (GetGripperMotionDone)")
            fault, motion_status = motion_pair
            _, _, position = self.robot.GetGripperCurPosition()
            _, _, speed = self.robot.GetGripperCurSpeed()
        return {
            "fault": fault,
            "motion_status": motion_status,  # 0=devam ediyor,1=bitti-nesne yok,2=bitti-nesne var
            "position_pct": position,
            "speed_pct": speed,
        }


def _demo():
    print("Robot aranıyor (detect_robot.py)...")
    ip = detect_robot_ip()
    print(f"Bulundu: {ip}")

    gripper = GripperController(robot_ip=ip)

    print("Aktivasyon durumu kontrol ediliyor...")
    if gripper.is_active():
        print("  zaten aktif.")
    else:
        print("  aktive ediliyor...")
        gripper.activate()
        print("  aktivasyon tamamlandi.")

    print("\n--- AÇ (position=0) ---")
    gripper.open(speed=40, force=40)
    print("  durum:", gripper.status())

    time.sleep(1)

    print("\n--- KAPA (position=100) ---")
    gripper.close(speed=40, force=40)
    print("  durum:", gripper.status())


if __name__ == "__main__":
    _demo()
