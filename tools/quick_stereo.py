#!/usr/bin/env python3
"""
HIZLI STEREO KONTROL (CLI) - kalibrasyon YOK, tespit YOK.

    docker compose run --rm valiz-olcum python -m tools.quick_stereo \
        --left L.png --right R.png --out qs_out --focal-px 1150 [--ply qs.ply]

Cekirdek: src/quickstereo.py  (arayuz de ayni fonksiyonu kullanir:
pages/2_Hizli_Stereo.py). Cikti: qs_out/disparity.png, depth.png, rect_*.png.
Olcek YOK (baseline birimi); goreli yapinin dogrulugu gorunur.
"""
from __future__ import annotations

import argparse
import os


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--left", required=True)
    ap.add_argument("--right", required=True)
    ap.add_argument("--out", default="qs_out")
    ap.add_argument("--ply", default="")
    ap.add_argument("--focal-px", type=float, default=0.0,
                    help="piksel odak (C310 @720p ~1150). 0=sezgisel (genellikle KOTU)")
    ap.add_argument("--center-roi", type=float, default=0.6)
    args = ap.parse_args()

    import cv2
    from src.quickstereo import run_quick_stereo

    imgL, imgR = cv2.imread(args.left), cv2.imread(args.right)
    if imgL is None or imgR is None:
        print("HATA: goruntu okunamadi")
        return 2

    res = run_quick_stereo(imgL, imgR, focal_px=args.focal_px, center_roi=args.center_roi)
    print("\n=== HIZLI STEREO ===")
    print(" ", res.message)
    for k, v in res.stats.items():
        print(f"  {k}: {v}")
    if not res.ok and res.rect_left is None:
        return 1

    os.makedirs(args.out, exist_ok=True)
    cv2.imwrite(f"{args.out}/rect_left.png", res.rect_left)
    cv2.imwrite(f"{args.out}/rect_right.png", res.rect_right)
    cv2.imwrite(f"{args.out}/disparity.png", res.disparity_color)
    cv2.imwrite(f"{args.out}/depth.png", res.depth_color)
    print(f"  cikti -> {args.out}/")

    if args.ply:
        pc = res.pointcloud()
        if pc is not None and len(pc):
            open(args.ply, "wb").write(pc.to_ply_bytes())
            print(f"  {len(pc)} nokta -> {args.ply}")
    return 0 if res.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
