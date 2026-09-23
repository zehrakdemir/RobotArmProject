#!/usr/bin/env python3
"""
UCTAN UCA AL-YERLESTIR: goruntu olcumu (bu repo, src/cell.py) + ROS2/MoveIt kol
kontrolu (movecobot-web API, Huseyin'in 'cobot' konteynerinde, port 8088).

Bu iki ayri proje/surucu (zehra: gorme + eski Fairino JOG surucusu; huseyin:
ROS2/MoveIt) BIRLIKTE kullanildigi icin bir donusum katmani gerekiyor:
  - Konum (x,y,z): src/grasp.py'nin ciktisi dogrudan kullanilabilir (bu
    oturumda defalarca dogrulandi, robot gercekten hesaplanan noktaya gidiyor).
  - Aci (rz): src/grasp.py'nin ciktisi (Fairino JOG surucusu icin hesaplanmis)
    ROS2/MoveIt API'sine DOGRUDAN verilirse ~45 derece hatali cikiyor (2026-09-18
    oturumunda iki bagimsiz olcumle dogrulandi: -83.9 vs gercek -132.28,
    -86.03 vs gercek -130.39 -> fark ~45-48 derece, ayni yonde). KOKU HENUZ
    COZULMEDI (muhtemelen nesne tam dik degil, egik durunca yukari-eksen
    varsayimi izdusumu donduruyor). Simdilik YAW_CORRECTION_DEG ile deneysel
    duzeltme uygulaniyor - farkli bir nesne/aci ile YENIDEN DOGRULANMADAN
    tam otomatik/gozetimsiz calistirilmamali.

GUVENLIK (2026-09-18 oturumunda ogrenilenler, hepsi burada uygulaniyor):
  - Gripper PASIF durumdaysa "open"/"close"/"move" sessizce hicbir sey yapmiyor
    (err=0 donuyor ama gripper hareket etmiyor) - sadece "grip" otomatik
    reaktive ediyor. Bu yuzden her calismada once bir "grip" gonderilip
    reaktivasyon zorlaniyor.
  - Nesneye yakin (yaklasma yuksekligi -> kavrama yuksekligi) inis/kalkis TEK
    BUYUK PTP adimiyla YAPILMAZ: PTP eklem-uzayinda enterpole eder, baslangic
    ve bitis yonelimi ayni olsa bile ARADA aletin egilmesine (gripper capraz
    inmesine) yol acabiliyor - bu oturumda 3 gercek carpismaya sebep oldu.
    Onun yerine /api/arm/tcp_base ile KUCUK (<=30mm) goreceli jog'lar
    kullaniliyor - bunlar mevcut eklem "seklini" koruyor, yonelim sapmiyor.
  - Her hareketten sonra fault kontrol ediliyor; fault olursa DUR - otomatik
    temizleme YOK (ResetAllError riskli, robotun kendi WebApp'inde elle
    Clear + docker restart cobot gerekiyor, bkz. movecobot.py).
  - IK basarisiz olursa (0 cozum / tekillik) ayni yonelimin 180 derece
    donmus esdegeri denenir (gripper simetrik, kapanma yonu ayni kalir).
  - Raf/birakma noktasi olarak varsayilan olarak nesnenin ORIJINAL x,y'sinden
    farkli, ayni guvenli yukseklik/yonelimde bir nokta kullanilir (kullanicinin
    kaydettigi orijinal yuksek raf poz'u -- (-194.9,-84.9,646,90.5,-36.4,-95.4)
    -- bu API'nin tekillik kontrolunden geciyor, joint-uzayinda jog'lanarak
    kaydedilmisti; --shelf ile override edilebilir ama once --dry-run'da
    tekillik kontrolu yapin).

KULLANIM (konteyner icinde, host.docker.internal ile hem kamera hem ROS2
webapp'e ulasir):
    docker compose run --rm valiz-olcum python -m tools.pick_and_place \\
        --shelf-x -300 --shelf-y 100 --shelf-z 150 \\
        --shelf-rx 180 --shelf-ry 0 --shelf-rz 0

    --dry-run   : sadece olc + planla + ROS2 preview yap, HICBIR HAREKET
                  KOMUTU YURUTME (robota tamamen zararsiz, ilk deneme icin
                  ONERILIR).
    --step      : her adimdan once Enter'a basilmasini bekler (yari-otomatik,
                  bugunku gibi supervised calisma icin).
"""
from __future__ import annotations

import argparse
import sys
import time

import cv2
import numpy as np
import requests

from src import config
from src.cell import run_cell

ROS2_API = "http://host.docker.internal:8088"
YAW_CORRECTION_DEG = -46.37  # DENEYSEL - bkz. modul docstring'i, kok neden cozulmedi

JOG_STEP_MM = 30.0           # inis/kalkista tek jog'un en fazla boyu
SPEED_PCT_EXPECTED = 25.0    # /api/settings ile ayarlanir; burada sadece belge amacli
SETTLE_SEC = 0.5             # her hareket sonrasi robotun FIILEN oturmasi icin
                              # bekleme - "is bitti" (job.running=False) dendigi
                              # anda TF/pozisyon okumasi hala gecis halinde
                              # olabiliyor (gripper GetGripperCurPosition() ile
                              # ayni bayatlik sorunu). Adim adim tiklarken
                              # aradaki insan gecikmesi bunu gizliyordu; run_all
                              # adimlari bosluksuz zincirleyince 2026-09-18'de
                              # descend GOAL_STATE_INVALID hatasi verdi - kok
                              # neden buydu.

# Poz-tabanli adimlarda (yaklas/birakma noktasi) "movecobot'tan onayla"
# modunda (auto_run=False) robotun gercekten hedefe ulasmasini beklerken
# kullanilir - kullanici movecobot'un kendi arayuzunde "Run this path"e
# basana kadar (2026-09-23, kullanici istegi: run_all TEK akis kalsin).
MANUAL_CONFIRM_TIMEOUT_S = 300.0
MANUAL_CONFIRM_POLL_S = 1.0
MANUAL_CONFIRM_TOL_DEG = 3.0

_STEP_MODE = False


def _log(msg: str) -> None:
    print(f"[pick_and_place] {msg}", flush=True)


def _confirm_step(label: str) -> None:
    if _STEP_MODE:
        input(f"    -> devam icin Enter: {label}")


# --------------------------------------------------------------------------
# ROS2 webapp (movecobot-web) HTTP yardimcilari
# --------------------------------------------------------------------------
def _get(path: str) -> dict:
    r = requests.get(f"{ROS2_API}{path}", timeout=15)
    r.raise_for_status()
    return r.json()


def _post(path: str, body: dict, timeout: float = 20.0) -> dict:
    r = requests.post(f"{ROS2_API}{path}", json=body, timeout=timeout)
    r.raise_for_status()
    return r.json()


def _wait_job(timeout: float = 40.0, poll: float = 1.0) -> dict:
    t0 = time.time()
    while time.time() - t0 < timeout:
        st = _get("/api/state")
        job = st.get("job")
        if job is not None and not job.get("running", False):
            return st
        time.sleep(poll)
    raise RuntimeError(f"is {timeout}s icinde bitmedi")


def _check_fault(st: dict) -> None:
    f = st.get("fault") or {}
    if f.get("main_code", 0) != 0:
        raise RuntimeError(
            f"ROBOT FAULT: main_code={f.get('main_code')} sub_code={f.get('sub_code')}. "
            "DURDURULDU - robotun kendi WebApp'inde 'Clear' basip 'docker restart cobot' "
            "calistirin, sonra bu script'i tekrar baslatin."
        )


def ensure_ready() -> dict:
    st = _get("/api/state")
    if not st.get("ready"):
        raise RuntimeError(f"ROS2 arayuzu hazir degil: {st.get('startup_error')}")
    _check_fault(st)
    return st


def ensure_gripper_active(dry_run: bool) -> None:
    """'open' pasif gripper'da sessizce hicbir sey yapmiyor - once 'grip' ile
    zorla reaktive et (bkz. gripper_controller.py .grip() docstring'i)."""
    _log("gripper reaktive ediliyor (grip, sonra open)...")
    if dry_run:
        _log("  [dry-run] atlandi")
        return
    _post("/api/gripper", {"action": "grip", "speed": 30, "force": 20})
    st = _wait_job()
    _check_fault(st)
    _post("/api/gripper", {"action": "open", "speed": 40, "force": 30})
    st = _wait_job()
    _check_fault(st)
    # GetGripperCurPosition() "hareket bitti" dendikten sonra bile bir sure
    # BAYAT kalabiliyor (gripper_controller.py .grip() docstring'inde de
    # belirtiliyor, ayni sebeple orada 0.3s bekleniyor) - 2026-09-18 canli
    # testte gercekten acikken %12 VE %48 gibi yanlis okumalar goruldu.
    # Kisa bekleme + birkac deneme ile bu bayatligi tolere ediyoruz.
    gs = {}
    for attempt in range(5):
        time.sleep(0.4)
        gs = requests.get(f"{ROS2_API}/api/gripper/status", timeout=10).json()
        if gs.get("position_pct", 100) <= 25:
            break
    else:
        raise RuntimeError(f"Gripper acilmadi (position_pct={gs.get('position_pct')}). "
                            "Elle kontrol edin.")
    _log(f"  gripper acik dogrulandi (position_pct={gs.get('position_pct')})")


def move_joints(joints_deg: list[float], dry_run: bool) -> None:
    _log(f"eklem hedefi: {joints_deg}")
    if dry_run:
        return
    _post("/api/arm/joints", {"joints_deg": joints_deg})
    st = _wait_job()
    _check_fault(st)
    if not st["job"]["ok"]:
        raise RuntimeError(f"Eklem hareketi basarisiz: {st['job'].get('detail')}")
    time.sleep(SETTLE_SEC)


def _max_joint_travel(cur_joints: list[float], goal_joints: list[float]) -> float:
    return max(abs(a - b) for a, b in zip(cur_joints, goal_joints))


def _try_preview(x, y, z, rx, ry, rz, prefer_lin: bool, cur_joints: list[float]
                  ) -> tuple[int, int, float, list[float]] | None:
    """Basarili (en az eklem hareketi gerektiren) secenegi bulur - suresi
    kisa olan degil, cunku kisa sure bazen "duz" ama BUYUK bir taban donusu
    (J1) anlamina gelebiliyor (2026-09-18: PTP boyle bir donuste kolun
    nesnenin uzerinden genis bir yay cizmesine sebep oldu). Basarili ise
    (preview_id, option_index, max_joint_travel_deg, goal_joints) doner,
    yoksa None."""
    _post("/api/preview", {"kind": "pose", "values": [x, y, z, rx, ry, rz],
                           "prefer_lin": prefer_lin})
    st = _wait_job(timeout=25)
    prev = _get("/api/preview")
    ok_opts = [o for o in prev.get("options", []) if o.get("ok") and o.get("goal_joints")]
    if not ok_opts:
        return None
    scored = [(o, _max_joint_travel(cur_joints, o["goal_joints"])) for o in ok_opts]
    best_o, best_travel = min(scored, key=lambda t: t[1])
    return prev["id"], best_o["index"], best_travel, best_o["goal_joints"]


def move_pose(x: float, y: float, z: float, rx: float, ry: float, rz: float,
              dry_run: bool, prefer_lin: bool = False, label: str = "",
              auto_run: bool = True) -> bool:
    """Hem verilen yonelimi hem de (gripper simetrik oldugu icin islevsel
    olarak ayni olan) 180 derece donmus esdegerini dener, HANGISI DAHA AZ
    EKLEM HAREKETI GEREKTIRIYORSA onu kullanir - byle boylece kol nesnenin
    veya baska bir seyin uzerinden genis bir yayla donmek yerine en dogrudan
    yoldan gider (bkz. _try_preview docstring'i)."""
    _log(f"pose hedefi {label}: x={x:.1f} y={y:.1f} z={z:.1f} rz={rz:.1f}")
    cur_joints = _get("/api/state")["joints_deg"]
    alt_rz = rz + 180.0 if rz <= 0 else rz - 180.0
    a = _try_preview(x, y, z, rx, ry, rz, prefer_lin, cur_joints)
    b = _try_preview(x, y, z, rx, ry, alt_rz, prefer_lin, cur_joints)
    candidates = [(rz, a), (alt_rz, b)]
    candidates = [(r, f) for r, f in candidates if f is not None]
    if not candidates:
        _log("  HICBIR yonelim planlanamadi (tekillik/IK) - bu adim atlaniyor")
        return False
    used_rz, (preview_id, option, travel, _goal_joints) = min(candidates, key=lambda t: t[1][2])
    _log(f"  secilen yonelim rz={used_rz:.1f} (en buyuk eklem hareketi {travel:.1f} derece)")
    if travel > 90.0:
        _log(f"  UYARI: {travel:.1f} derecelik eklem hareketi hala buyuk - "
             "kolun gecis yolunu gozle kontrol edin.")
    _log(f"  planlandi (rz={used_rz:.1f}, secenek {option})")
    _confirm_step(f"{label} calistirilsin mi?")
    if dry_run:
        _log("  [dry-run] yurutulmedi")
        return True
    # Iki yonelim de denendigi icin sunucudaki TEK onizleme yuvasi son
    # denemeyle (alt_rz) ezilmis olabilir - kazanan rz KAZANANI ise, onu
    # calistirmadan once TAZELE (409 "onizleme eskimis" hatasini onler,
    # 2026-09-18'de canli tespit edildi).
    fresh = _try_preview(x, y, z, rx, ry, used_rz, prefer_lin, cur_joints)
    if fresh is None:
        _log("  yeniden planlama basarisiz - bu adim atlaniyor")
        return False
    preview_id, option, _, goal_joints = fresh
    if not auto_run:
        # Kullanici istegi (2026-09-23): sadece onizleme yapilsin, robot
        # movecobot'un KENDI arayuzunden (ayni sunucu-tarafi onizleme, ayni
        # anda orada da goruntuleniyor) "Run this path" ile elle onaylanip
        # calistirilsin - pickcobot otomatik calistirmasin. "Tum akisi
        # calistir" TEK bir akis olarak kalsin diye, burada robotun gercekten
        # hedefe ulasmasini BEKLIYORUZ (poll) - kullanici movecobot'ta
        # onaylayip calistirinca akis KENDILIGINDEN devam ediyor, ayrica
        # "devam et" butonuna basmaya gerek yok.
        _log("  onizleme hazir - movecobot'un kendi arayuzunde "
             "(http://localhost:8088) inceleyip 'Run this path' ile onaylayin. "
             f"Onaylanana kadar bekleniyor (en fazla {MANUAL_CONFIRM_TIMEOUT_S:.0f}s)...")
        deadline = time.time() + MANUAL_CONFIRM_TIMEOUT_S
        while time.time() < deadline:
            time.sleep(MANUAL_CONFIRM_POLL_S)
            st = _get("/api/state")
            if (st.get("job") or {}).get("running"):
                continue
            now_joints = st.get("joints_deg")
            if now_joints and _max_joint_travel(now_joints, goal_joints) <= MANUAL_CONFIRM_TOL_DEG:
                _check_fault(st)
                _log("  movecobot'ta onaylandi - devam ediliyor.")
                time.sleep(SETTLE_SEC)
                return True
        _log(f"  {MANUAL_CONFIRM_TIMEOUT_S:.0f}s icinde movecobot'tan onay gelmedi - zaman asimi.")
        return False
    _post("/api/preview/run", {"id": preview_id, "option": option})
    st = _wait_job(timeout=40)
    _check_fault(st)
    if not st["job"]["ok"]:
        raise RuntimeError(f"Hareket basarisiz: {st['job'].get('detail')}")
    time.sleep(SETTLE_SEC)
    return True


def jog_dz(dz_mm: float, dry_run: bool, label: str = "", max_step: float = JOG_STEP_MM) -> None:
    """Kucuk goreceli dikey jog - buyuk PTP yerine (bkz. modul docstring'i).
    max_step kucultulerek nesneye yakin inislerde daha temkinli ilerlenebilir
    (2026-09-18: nesneye ~40mm kalmisken tek 30mm'lik adim temasa sebep oldu)."""
    remaining = dz_mm
    while abs(remaining) > 1e-6:
        step = max(-max_step, min(max_step, remaining))
        _log(f"jog dz={step:+.1f}mm {label}")
        _confirm_step(f"jog dz={step:+.1f}mm")
        if not dry_run:
            _post("/api/arm/tcp_base", {"dx_mm": 0, "dy_mm": 0, "dz_mm": step})
            st = _wait_job(timeout=20)
            _check_fault(st)
            if not st["job"]["ok"]:
                raise RuntimeError(f"Jog basarisiz: {st['job'].get('detail')}")
            time.sleep(SETTLE_SEC)
        remaining -= step


def gripper_grip(dry_run: bool) -> dict:
    _log("gripper kapatiliyor (adaptif 'grip' - nesneye gore durur, sikistirmaz)")
    _confirm_step("gripper kapat (grip)")
    if dry_run:
        return {"gripped": None}
    _post("/api/gripper", {"action": "grip", "speed": 20, "force": 15})
    st = _wait_job()
    _check_fault(st)
    time.sleep(SETTLE_SEC)
    gs = requests.get(f"{ROS2_API}/api/gripper/status", timeout=10).json()
    gripped = gs.get("motion_status") == 2
    _log(f"  motion_status={gs.get('motion_status')} (2=nesne yakalandi) "
         f"position_pct={gs.get('position_pct')}")
    if not gripped:
        _log("  UYARI: nesne yakalanmamis gibi gorunuyor (motion_status != 2)")
    return {"gripped": gripped}


def gripper_release(dry_run: bool) -> None:
    _log("gripper aciliyor (birakma)")
    _confirm_step("gripper ac (birak)")
    if dry_run:
        return
    _post("/api/gripper", {"action": "open", "speed": 30, "force": 20})
    st = _wait_job()
    _check_fault(st)
    time.sleep(SETTLE_SEC)


# --------------------------------------------------------------------------
# Gorme: olc + kavrama plani (+ deneysel aci duzeltmesi)
# --------------------------------------------------------------------------
def capture_pair() -> tuple[str, str]:
    r = requests.post(f"{config.CAM_WEB_URL}/snap", timeout=15).json()
    if not r.get("ok"):
        raise RuntimeError(f"Kamera cekimi basarisiz: {r.get('err')}")
    # cam_web.py Windows'ta calisiyor, donen yollar ters slash icerebilir -
    # konteyner (Linux) icin normalize et.
    return r["left"].replace("\\", "/"), r["right"].replace("\\", "/")


def measure_and_plan():
    left, right = capture_pair()
    _log(f"cift alindi: {left}, {right}")
    imgL, imgR = cv2.imread(left), cv2.imread(right)
    if imgL is None or imgR is None:
        raise RuntimeError("goruntu okunamadi")

    # Once serbest olcum (grasp_z_mm zorlanmadan) - nesne yuksekligini ogren,
    # sonra parmak boyuna gore guvenli kavrama yuksekligini hesapla.
    probe = run_cell(imgL, imgR, posture_override="standing_on_wheels")
    if probe.obb is None:
        raise RuntimeError(f"Olcum basarisiz: {probe.message}")
    height_mm = probe.obb.height_mm
    # 2026-09-18 canli testte height-30mm (ortadan asagi) capisma yaptirdi;
    # height-10mm (tepeye yakin) sorunsuz calisti - dar/dik duran nesnelerde
    # tepeye yakin kavramak daha guvenli (bkz. gunluk oturum notlari).
    safe_grasp_z = max(10.0, height_mm - 10.0)
    _log(f"nesne yuksekligi={height_mm:.1f}mm -> guvenli kavrama_z={safe_grasp_z:.1f}mm "
         f"(tepeye yakin - deneysel)")

    res = run_cell(imgL, imgR, posture_override="standing_on_wheels",
                    grasp_z_mm=safe_grasp_z)
    if res.grasp is None or not res.grasp.feasible:
        raise RuntimeError(f"Kavrama plani yok: {res.message}")
    g = res.grasp
    corrected_rz = g.yaw_deg + YAW_CORRECTION_DEG
    _log(f"olcum: {res.message}")
    _log(f"ham tool_yaw={g.yaw_deg:.1f} -> duzeltilmis rz={corrected_rz:.1f} "
         f"(YAW_CORRECTION_DEG={YAW_CORRECTION_DEG})")
    return {
        "grasp": g.grasp_point.tolist(),
        "approach": g.approach_point.tolist(),
        "lift": g.lift_point.tolist(),
        "rz": corrected_rz,
        "opening_mm": g.opening_mm,
    }


# --------------------------------------------------------------------------
def main() -> int:
    global _STEP_MODE
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true",
                     help="sadece olc+planla+preview, HICBIR HAREKETI YURUTME")
    ap.add_argument("--step", action="store_true",
                     help="her adimdan once Enter bekle (yari-otomatik)")
    ap.add_argument("--shelf-x", type=float, default=-300.0)
    ap.add_argument("--shelf-y", type=float, default=100.0)
    ap.add_argument("--shelf-z", type=float, default=150.0)
    ap.add_argument("--shelf-rx", type=float, default=180.0)
    ap.add_argument("--shelf-ry", type=float, default=0.0)
    ap.add_argument("--shelf-rz", type=float, default=0.0)
    ap.add_argument("--place-drop-mm", type=float, default=60.0,
                     help="raf noktasindan ne kadar asagi inilecek (masaya/rafa deginceye kadar)")
    args = ap.parse_args()
    _STEP_MODE = args.step
    dry = args.dry_run

    st = ensure_ready()
    _log(f"ROS2 hazir. joints={st.get('joints_deg')}")

    plan = measure_and_plan()
    ax, ay, az = plan["approach"]
    gx, gy, gz = plan["grasp"]
    lx, ly, lz = plan["lift"]
    rz = plan["rz"]

    ensure_gripper_active(dry)

    _log("=== nesneye yaklas ===")
    # Bazi x,y konumlarinda yuksek Z (varsayilan yaklasma) erisilemiyor
    # (is alani/tekillik siniri, 2026-09-18'de gozlemlendi) - basarisiz
    # olursa kademeli olarak daha alcak bir yaklasma yuksekligi dene (yine de
    # kavrama noktasindan en az 30mm yukarida kalarak).
    used_az = None
    for candidate_az in np.arange(az, gz + 29.0, -20.0):
        candidate_az = float(candidate_az)
        ok = move_pose(ax, ay, candidate_az, 180.0, 0.0, rz, dry, prefer_lin=False,
                        label=f"yaklasma (z={candidate_az:.0f})")
        if ok:
            used_az = candidate_az
            break
        _log(f"  z={candidate_az:.0f} basarisiz, daha alcak deneniyor...")
    if used_az is None:
        _log("Yaklasma noktasina HICBIR yukseklikte ulasilamadi - DURULDU. "
             "Kavrama noktasini/yonelimi elle kontrol edin.")
        return 1

    _log("=== kucuk adimlarla in ===")
    jog_dz(gz - used_az, dry, label="kavramaya inis")

    gripper_grip(dry)

    _log("=== kucuk adimlarla kaldir ===")
    jog_dz(lz - gz, dry, label="kaldirma")

    _log("=== rafa/birakma noktasina tasi ===")
    ok = move_pose(args.shelf_x, args.shelf_y, args.shelf_z,
                    args.shelf_rx, args.shelf_ry, args.shelf_rz, dry,
                    prefer_lin=False, label="raf")
    if not ok:
        _log("Raf noktasina ulasilamadi (tekillik/IK) - nesne hala kavrada, "
             "DURULDU. --shelf-x/y/z/rx/ry/rz ile farkli bir nokta deneyin.")
        return 1

    _log("=== birakma icin in ===")
    jog_dz(-args.place_drop_mm, dry, label="birakmaya inis")

    gripper_release(dry)

    _log("=== geri cekil ===")
    jog_dz(args.place_drop_mm, dry, label="geri cekilme")

    _log("=== notre don ===")
    move_joints([0.0, -90.0, 90.0, -90.0, -90.0, 0.0], dry)

    _log("TAMAMLANDI.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
