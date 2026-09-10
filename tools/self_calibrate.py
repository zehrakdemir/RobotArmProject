#!/usr/bin/env python3
"""
CEVRIMICI STEREO OZ-KALIBRASYON - CLI sarmalayici (satranc tahtasi YOK).

Gercek is `src/selfcalib.py` icinde. Kameralar oturumlar arasi kaydiginda
(rektifiye dikey hata buyur, gecerli disparite duser) dokulu masadan kameralar-
arasi POZ'u (R,T) yeniden kestirir; LENS parametrelerini korur.

    # dokulu masa gorunur sekilde bir cift kaydet, sonra:
    docker compose exec -T valiz-olcum python -m tools.self_calibrate
      [--left L.png --right R.png] [--no-write]

Yaml bind-mount -> konteyneri yeniden baslatmaya gerek YOK.
detect_objects zaten gecerli disparite dususunce bunu OTOMATIK cagirir; bu arac
elle tetiklemek / dogrulamak icindir.
"""
from __future__ import annotations

import argparse
import glob
import os

import cv2

from src import config
from src.selfcalib import refresh_stereo_extrinsics


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--left")
    ap.add_argument("--right")
    ap.add_argument("--no-write", action="store_true", help="sadece raporla, yaml yazma")
    args = ap.parse_args()

    if args.left and args.right:
        lp, rp = args.left, args.right
    else:
        ls = sorted(glob.glob(os.path.join(config.SNAP_DIR, "left_*.png")))
        if not ls:
            print("HATA: snaps/ bos"); return 2
        lp, rp = ls[-1], ls[-1].replace("left_", "right_")
    print("cift:", os.path.basename(lp))
    L, R = cv2.imread(lp), cv2.imread(rp)
    if L is None or R is None:
        print("HATA: cift okunamadi"); return 2

    info = refresh_stereo_extrinsics(L, R, write=not args.no_write)
    for k, v in info.items():
        print(f"  {k}: {v}")
    err = info.get("rektifiye_dikey_hata_px", {})
    if isinstance(err, dict) and err.get("sonra", 9) > 1.5:
        print("UYARI: dikey hata hala > 1.5 px - sahnede yeterli doku yok olabilir.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
