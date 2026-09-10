#!/usr/bin/env python3
"""
Iki USB kameradan (Ubuntu /dev/videoN) es-zamanli L/R kare cifti kaydeder.

Kullanim (konteyner icinde):
    docker compose run --rm valiz-olcum python -m tools.capture_pairs \
        --out calib_shots --count 20 --interval 1.5

  --fr3  verilirse her karede FR3'un guncel TCP pozunu da yazar
         (hand-eye ornekleri icin: samples/NNNN_pose.txt).

GUI yok; belirli aralikla otomatik cekim. Satranç tahtasini kameralara
farkli acilardan gosterin.
"""
from __future__ import annotations

import argparse
import os
import time

import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="calib_shots")
    ap.add_argument("--left", type=int, default=None)
    ap.add_argument("--right", type=int, default=None)
    ap.add_argument("--width", type=int, default=None)
    ap.add_argument("--height", type=int, default=None)
    ap.add_argument("--count", type=int, default=20)
    ap.add_argument("--interval", type=float, default=1.5)
    ap.add_argument("--warmup", type=int, default=5)
    ap.add_argument("--fr3", action="store_true", help="her karede FR3 TCP pozunu da yaz")
    args = ap.parse_args()

    import cv2
    from src import config

    li = args.left if args.left is not None else config.CAM_LEFT_INDEX
    ri = args.right if args.right is not None else config.CAM_RIGHT_INDEX
    w = args.width or config.CAM_WIDTH
    h = args.height or config.CAM_HEIGHT

    capL, capR = cv2.VideoCapture(li), cv2.VideoCapture(ri)
    for c in (capL, capR):
        c.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        c.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    if not capL.isOpened() or not capR.isOpened():
        print(f"HATA: kamera acilamadi (L=/dev/video{li}, R=/dev/video{ri})")
        return 2

    os.makedirs(args.out, exist_ok=True)
    fr3 = None
    if args.fr3:
        from src.robot import FairinoFR3
        fr3 = FairinoFR3(enabled=True)
        fr3.connect()

    for _ in range(args.warmup):
        capL.read(); capR.read(); time.sleep(0.05)

    n = 0
    while n < args.count:
        time.sleep(args.interval)
        okL, fL = capL.read()
        okR, fR = capR.read()
        if not (okL and okR):
            print("kare alinamadi, tekrar...")
            continue
        cv2.imwrite(os.path.join(args.out, f"left_{n:04d}.png"), fL)
        cv2.imwrite(os.path.join(args.out, f"right_{n:04d}.png"), fR)
        line = f"[{n+1}/{args.count}] kaydedildi"
        if fr3 is not None and fr3._robot is not None:
            try:
                pose = fr3._robot.GetActualTCPPose()[1]  # [x,y,z,rx,ry,rz] mm/deg
                np.savetxt(os.path.join(args.out, f"pose_{n:04d}.txt"), np.asarray(pose))
                line += f"  TCP={np.round(pose, 1).tolist()}"
            except Exception as exc:  # noqa: BLE001
                line += f"  (TCP alinamadi: {exc})"
        print(line)
        n += 1

    capL.release(); capR.release()
    print(f"\nBitti -> {args.out}/  ({n} cift)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
