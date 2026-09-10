#!/usr/bin/env python3
"""
Arayuzsuz calistirma - 2 fotograf ver, olcum + durus + FR3 planini al.

Ornek:
    python run_cli.py sol.jpg sag.jpg --known-width 40 --json
    python run_cli.py sol.jpg sag.jpg --known height --known-width 55 --ply out.ply
"""
from __future__ import annotations

import argparse
import json
import sys

import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser(description="Valiz stereo olcum (CLI)")
    ap.add_argument("left", help="Sol / 1. fotograf")
    ap.add_argument("right", help="Sag / 2. fotograf")
    ap.add_argument("--known", default="width", choices=["width", "height", "depth"],
                    help="Bilinen gercek olcu ekseni")
    ap.add_argument("--known-width", type=float, default=40.0,
                    help="Bilinen eksenin GERCEK degeri (cm)")
    ap.add_argument("--focal-35mm", type=float, default=0.0)
    ap.add_argument("--focal-px", type=float, default=0.0)
    ap.add_argument("--swap", action="store_true")
    ap.add_argument("--robot", action="store_true", help="FR3'e GERCEKTEN gonder")
    ap.add_argument("--robot-ip", default="192.168.58.2")
    ap.add_argument("--ply", default="", help="Nokta bulutunu bu dosyaya yaz (.ply)")
    ap.add_argument("--depth-png", default="", help="Derinlik haritasini bu dosyaya yaz")
    ap.add_argument("--json", action="store_true", help="Ozeti JSON bas")
    args = ap.parse_args()

    import cv2

    from src import config
    from src.pipeline import run_pipeline

    img_l = cv2.imread(args.left, cv2.IMREAD_COLOR)
    img_r = cv2.imread(args.right, cv2.IMREAD_COLOR)
    if img_l is None or img_r is None:
        print("HATA: goruntu(ler) okunamadi.", file=sys.stderr)
        return 2

    run_cfg = config.RunConfig(
        known_width_cm=args.known_width, known_dimension=args.known,
        focal_length_35mm=args.focal_35mm, manual_focal_px=args.focal_px,
        swap_images=args.swap, robot_enabled=args.robot, robot_ip=args.robot_ip,
    )

    result = run_pipeline(img_l, img_r, run_cfg, args.left, args.right)

    if not result.is_suitcase:
        print(f"VALIZ DEGIL / bulunamadi: {result.message}", file=sys.stderr)
        return 1

    if args.ply and result.pointcloud is not None and len(result.pointcloud):
        with open(args.ply, "wb") as fh:
            fh.write(result.pointcloud.to_ply_bytes())
        print(f"[yazildi] {args.ply}  ({len(result.pointcloud)} nokta)")

    if args.depth_png and result.depth_color is not None:
        cv2.imwrite(args.depth_png, result.depth_color)
        print(f"[yazildi] {args.depth_png}")

    if args.robot and result.robot_command and result.robot_command.action != "none":
        from src.robot import FairinoFR3

        fr3 = FairinoFR3(ip=args.robot_ip, enabled=True)
        fr3.connect()
        exec_out = fr3.execute(result.robot_command)
        print("[FR3]", json.dumps(exec_out, ensure_ascii=False))

    summary = result.summary()
    if args.json:
        print(json.dumps(summary, ensure_ascii=False, indent=2))
    else:
        m = result.measurement
        print(f"\n=== VALIZ OLCUM ===")
        print(f"  en (genislik)   : {m.width_cm:6.1f} cm")
        print(f"  boy (yukseklik) : {m.height_cm:6.1f} cm")
        print(f"  derinlik        : {m.depth_cm:6.1f} cm")
        print(f"  yamukluk (2B)   : {m.rect.tilt_deg:+.1f}°  ({'YAMUK' if m.rect.is_skewed else 'duz'})")
        print(f"  egim (3B)       : {m.lean_deg:.1f}°")
        print(f"  tepe x,y        : {m.top_ref.pixel}")
        print(f"  durus           : {result.posture.label_tr}  (guven {result.posture.confidence:.0%})")
        print(f"  FR3 action      : {result.robot_command.action} - {result.robot_command.reason}")
        for w in result.warnings:
            print(f"  [uyari] {w}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
