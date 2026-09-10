#!/usr/bin/env python3
"""
HAND-EYE yakalama: gripper'a ArUco yapistir, robotu MEVCUT pozdan kucuk
aci ofsetleriyle ~10 poza otomatik goturur; her pozda cam_web'den L/R cift +
FR3 TCP pozu kaydeder.  Sonra tools.handeye_solve calistir.

ONCE: robotu elle (pendant) oyle bir poza getir ki marker HER IKI kamerada da
net gorunsun, kadraj ortasinda olsun. Sonra:

    docker compose run --rm valiz-olcum python -m tools.handeye_capture
    docker compose run --rm valiz-olcum python -m tools.handeye_solve \
        --shots snaps/handeye --marker-mm <YAZDIRDIGIN KARE mm> --aruco DICT_4X4_50 \
        --out src/calibration/base_T_cam.yaml
"""
from __future__ import annotations

import json
import os
import time
import urllib.request

import numpy as np

from src import config
from src.robot import FairinoFR3

OUT = os.path.join(config.SNAP_DIR, "handeye")
# mevcut pozdan (derece) eklem ofsetleri - marker gorus alaninda kalacak kadar kucuk
OFFSETS = [
    (0, 0, 0, 0, 0, 0),
    (10, 0, 0, 0, 0, 0), (-10, 0, 0, 0, 0, 0),
    (0, 0, 0, 10, 0, 0), (0, 0, 0, -10, 0, 0),
    (0, 0, 0, 0, 12, 0), (0, 0, 0, 0, -12, 0),
    (0, 0, 0, 0, 0, 20), (0, 0, 0, 0, 0, -20),
    (8, 0, 0, -8, 10, 0), (-8, 0, 0, 8, -10, 0),
    (6, 0, 0, 6, 0, 15),
]


def _snap(idx: int):
    req = urllib.request.Request(config.CAM_WEB_URL.rstrip("/") + "/snap", method="POST")
    with urllib.request.urlopen(req, timeout=25) as r:
        js = json.loads(r.read().decode("utf-8", "replace"))
    if not js.get("ok"):
        raise RuntimeError(f"cam_web: {js}")
    import shutil
    for side in ("left", "right"):
        src = os.path.join(config.SNAP_DIR, os.path.basename(js[side].replace("\\", "/")))
        for _ in range(20):
            if os.path.exists(src):
                break
            time.sleep(0.3)
        shutil.copyfile(src, os.path.join(OUT, f"{side}_{idx:02d}.png"))


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    fr3 = FairinoFR3(enabled=True)
    if not fr3.connect():
        print("FR3 BAGLANAMADI"); return 1
    prep = fr3.prepare()
    if not prep.get("prepared"):
        print("HAZIRLIK BASARISIZ:", prep); return 1

    center = fr3._joints_now()
    print("merkez eklemler:", [round(v, 2) for v in center])
    fr3._call("SetSpeed", 12, timeout=6)

    n_ok = 0
    for i, off in enumerate(OFFSETS):
        tgt = [center[k] + off[k] for k in range(6)]
        bad = fr3._jlim_bad(tgt)
        if bad:
            print(f"  poz {i}: eklem limiti ({bad}) - atlandi"); continue
        try:
            print(f"  poz {i}: JOG -> {[round(v,1) for v in tgt]}")
            fr3._movej(tgt, 12)
            time.sleep(0.8)                       # sarsinti dursun
            tcp = fr3._call("GetForwardKin", fr3._joints_now(), timeout=8)
            if not (isinstance(tcp, (list, tuple)) and int(tcp[0]) == 0):
                print("    TCP okunamadi - atlandi"); continue
            _snap(i)
            np.savetxt(os.path.join(OUT, f"pose_{i:02d}.txt"),
                       np.array([float(v) for v in tcp[1:7]]))
            n_ok += 1
            print(f"    kaydedildi  TCP={[round(v,1) for v in tcp[1:7]]}")
        except Exception as exc:  # noqa: BLE001
            print(f"    HATA: {exc}")
            fr3._recover(); fr3.prepare()

    print(f"\nmerkeze donuluyor...")
    try:
        fr3._movej(center, 12)
    except Exception:  # noqa: BLE001
        pass
    print(f"BITTI: {n_ok} poz kaydedildi -> {OUT}")
    print("Simdi: docker compose run --rm valiz-olcum python -m tools.handeye_solve "
          f"--shots snaps/handeye --marker-mm <mm> --aruco DICT_4X4_50 "
          "--out src/calibration/base_T_cam.yaml")
    return 0 if n_ok >= 6 else 1


if __name__ == "__main__":
    raise SystemExit(main())
