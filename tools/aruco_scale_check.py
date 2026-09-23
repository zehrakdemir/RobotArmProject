#!/usr/bin/env python3
"""
ArUco isaretiyle canli olcek kontrolu (DENEYSEL) - bkz. src/aruco_scale.py.

Bir stereo cift verildiginde: isareti sol goruntude bulur, MUTLAK (monoküler,
solvePnP) mesafesini hesaplar, ayni noktanin stereo/disparite mesafesiyle
karsilastirir, onerilen MEASURE_SCALE oranini yazdirir.

KULLANIM (konteyner icinde):
    docker compose run --rm valiz-olcum python -m tools.aruco_scale_check \\
        --left snaps/left_x.png --right snaps/right_x.png \\
        --marker-id 7 --marker-size-mm 50
"""
from __future__ import annotations

import argparse


def main() -> int:
    import cv2

    from src.aruco_scale import estimate_scale_from_marker
    from src.calib import StereoRig
    from src.cell import _epipolar_correct
    from src.stereo import compute_disparity_adaptive

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--left", required=True)
    ap.add_argument("--right", required=True)
    ap.add_argument("--marker-id", type=int, default=None,
                     help="belirtilmezse goruntudeki ILK isaret kullanilir")
    ap.add_argument("--marker-size-mm", type=float, required=True,
                     help="isaretin GERCEK (kumpasla olculmus) kenar uzunlugu")
    args = ap.parse_args()

    imgL = cv2.imread(args.left)
    imgR = cv2.imread(args.right)
    if imgL is None or imgR is None:
        print(f"HATA: goruntu okunamadi ({args.left} / {args.right})")
        return 1

    rig = StereoRig.load()
    lr, rr = rig.rectify(imgL, imgR)
    rr, _epi_dy, _epi_ok = _epipolar_correct(lr, rr)
    gray = lambda im: cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    disp = compute_disparity_adaptive(gray(lr), gray(rr), roi_mask=None)
    points_mm = rig.reproject(disp.disparity)

    K1 = rig.P1[:3, :3]
    result = estimate_scale_from_marker(
        lr, K1, args.marker_size_mm, args.marker_id, points_mm, None)

    print("--- ArUco olcek kontrolu ---")
    for k, v in result.items():
        print(f"  {k}: {v}")

    if result.get("onerilen_olcek"):
        print()
        print(f"Mevcut MEASURE_SCALE ile karsilastirin (.env). Bu oran '1.0'a "
              f"yakinsa kalibrasyon zaten dogru; uzaksa .env'deki MEASURE_SCALE'i "
              f"bu degerle guncellemeyi dusunun.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
