#!/usr/bin/env python3
"""
Bagli kameralari listeler + her birinden bir kare yakalayip kucuk onizleme
kaydeder. Hangi /dev/video* SOL, hangisi SAG karar vermek icin.

    docker compose run --rm valiz-olcum python -m tools.list_cameras --out cam_probe

Cikti: cam_probe/videoN.png  +  ekranda cozunurluk/FPS. Onizlemelere bakip
.env'ye  CAM_LEFT_DEV=/dev/videoN  CAM_RIGHT_DEV=/dev/videoM  yazin
(veya docker-compose.cell.yml kullaniyorsaniz zaten sabit 0/1 olur).
"""
from __future__ import annotations

import argparse
import glob
import os


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="cam_probe")
    ap.add_argument("--max-index", type=int, default=10)
    args = ap.parse_args()

    import cv2

    os.makedirs(args.out, exist_ok=True)
    devs = sorted(glob.glob("/dev/video*")) or [f"/dev/video{i}" for i in range(args.max_index)]
    print(f"Denenecek: {devs}\n")

    found = 0
    for d in devs:
        idx = int("".join(ch for ch in os.path.basename(d) if ch.isdigit()) or -1)
        if idx < 0:
            continue
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            cap.release()
            continue
        for _ in range(5):
            cap.read()
        ok, frame = cap.read()
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        cap.release()
        if not ok or frame is None:
            print(f"  {d}: acildi ama kare yok (muhtemelen meta-node)")
            continue
        p = os.path.join(args.out, f"video{idx}.png")
        cv2.imwrite(p, frame)
        print(f"  {d}: {w}x{h} @ {fps:.0f}fps  -> {p}")
        found += 1

    print(f"\n{found} calisan kamera. Onizlemelere bak, sonra .env:")
    print("  CAM_LEFT_DEV=/dev/videoX")
    print("  CAM_RIGHT_DEV=/dev/videoY")
    return 0 if found >= 2 else 1


if __name__ == "__main__":
    raise SystemExit(main())
