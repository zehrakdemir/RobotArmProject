#!/usr/bin/env python3
"""
OTOMATIK OLCEK KALIBRASYONU - bilinen boyutlu bir kutu goster, olcegi kilitle.

Kalibrasyondan sonra kameralar kaydiysa disparite->derinlik olcegi kayar; tum
olculer sabit bir carpanla sisik/kucuk cikar. Bu arac o carpani bulup .env'e yazar.

    # masaya SADECE bilinen kutuyu koy, cam_web'den cift kaydet, sonra:
    docker compose run --rm valiz-olcum python -m tools.calib_scale --dims 167,107,46
    # dims = gercek boy,en,derinlik (mm) - buyukten kucuge sirali

Yazar: .env  MEASURE_SCALE=<carpan>
"""
from __future__ import annotations

import argparse
import glob
import os
import re


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dims", required=True, help="gercek boy,en,derinlik (mm), buyukten kucuge")
    ap.add_argument("--left"); ap.add_argument("--right")
    ap.add_argument("--use-thickness", action="store_true",
                    help="carpani 3 eksenin medyaniyla hesapla (varsayilan: sadece ayak izi)")
    ap.add_argument("--env", default=".env")
    args = ap.parse_args()

    import cv2
    import numpy as np
    from src import config
    from src.multi import detect_objects

    true_mm = sorted([float(x) for x in args.dims.split(",")], reverse=True)
    if len(true_mm) != 3:
        print("HATA: --dims 3 sayi olmali"); return 2

    if args.left and args.right:
        lp, rp = args.left, args.right
    else:
        ls = sorted(glob.glob(os.path.join(config.SNAP_DIR, "left_*.png")))
        if not ls:
            print("HATA: snaps/ bos"); return 2
        lp, rp = ls[-1], ls[-1].replace("left_", "right_")
    print(f"cift: {os.path.basename(lp)}")
    L, R = cv2.imread(lp), cv2.imread(rp)

    # mevcut MEASURE_SCALE'i etkisiz kabul et: olcumu ham carpanla geri al
    cur = float(getattr(config, "MEASURE_SCALE", 1.0)) or 1.0
    objs = detect_objects(L, R)
    if not objs:
        print("HATA: nesne bulunamadi. Masada sadece kutu var mi? Isik/doku?"); return 1
    o = max(objs, key=lambda z: z.volume_l)
    meas_mm = sorted([d * 10.0 for d in o.dims_cm], reverse=True)
    meas_raw = [m / cur for m in meas_mm]                 # mevcut carpani cikar
    print(f"olculen (ham): {[round(v) for v in meas_raw]} mm")
    print(f"gercek       : {[round(v) for v in true_mm]} mm")

    ratios = [t / m for t, m in zip(true_mm, meas_raw)]
    print(f"eksen oranlari (gercek/olculen): {[round(r, 3) for r in ratios]}")
    scale = float(np.median(ratios)) if args.use_thickness else float(np.mean(ratios[:2]))
    scale = round(scale, 4)
    print(f"\n==> MEASURE_SCALE = {scale}")

    # .env guncelle
    path = args.env
    lines = []
    if os.path.exists(path):
        lines = open(path, encoding="utf-8").read().splitlines()
    lines = [ln for ln in lines if not re.match(r"\s*MEASURE_SCALE\s*=", ln)]
    lines.append(f"MEASURE_SCALE={scale}")
    open(path, "w", encoding="utf-8").write("\n".join(lines) + "\n")
    print(f"yazildi -> {path}   (konteyneri yeniden baslat: docker compose up -d valiz-olcum)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
