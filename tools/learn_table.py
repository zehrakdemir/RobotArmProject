#!/usr/bin/env python3
"""
BOS MASAYI OGREN - sabit kameralar icin otomatik nesne segmentasyonu referansi.

    # en son snaps/ ciftini kullan (masada HICBIR sey olmasin):
    docker compose run --rm valiz-olcum python -m tools.learn_table
    # ya da belirli cift:
    docker compose run --rm valiz-olcum python -m tools.learn_table \
        --left snaps/left_XXXX.png --right snaps/right_XXXX.png

Kaydeder: src/calibration/table_ref.npz  (masa duzlemi + goruntu bolgesi).
Sonrasinda Hucre Modu, YOLO bir sey bulamazsa nesneyi otomatik ayirir - ROI GEREKMEZ.
Cekirdek mantik src/cell.py::learn_table icinde (UI butonu da onu cagirir).
"""
from __future__ import annotations

import argparse
import glob
import os


def _latest_pair(d="snaps"):
    ls = sorted(glob.glob(os.path.join(d, "left_*.png")))
    if not ls:
        raise SystemExit(f"{d}/left_*.png yok")
    return ls[-1], ls[-1].replace("left_", "right_")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--left"); ap.add_argument("--right")
    ap.add_argument("--out", default=None)
    ap.add_argument("--above-mm", type=float, default=12.0)
    args = ap.parse_args()

    import cv2
    from src.cell import learn_table

    lp, rp = (args.left, args.right) if args.left and args.right else _latest_pair()
    print(f"bos masa cifti: {os.path.basename(lp)}")
    imgL, imgR = cv2.imread(lp), cv2.imread(rp)
    if imgL is None or imgR is None:
        raise SystemExit("goruntu okunamadi")

    info = learn_table(imgL, imgR, out_path=args.out, above_mm=args.above_mm)
    for k, v in info.items():
        print(f"  {k}: {v}")
    if info["duzlem_inlier_%"] < 35:
        print("UYARI: masa duzlemi zayif (<%35). Masada nesne var mi? Isik yeterli mi?")
    print("Artik Hucre Modu'nda YOLO bulamazsa nesne otomatik ayrilir (ROI gerekmez).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
