#!/usr/bin/env python3
"""
HUCRE MODU calistirma (2 kamera + FR3).

Dosyadan:
    docker compose run --rm valiz-olcum python -m tools.run_cell \
        --left L.png --right R.png --json

Kameralardan (Ubuntu, --device gecirilmis):
    ... python -m tools.run_cell --cams --json

FR3'e GERCEKTEN gonder (dikkat, ROBOT_ENABLED=true da gerekir):
    ... python -m tools.run_cell --cams --execute
"""
from __future__ import annotations

import argparse
import json
import sys


def _grab_cams():
    import cv2
    from src import config
    capL = cv2.VideoCapture(config.CAM_LEFT_INDEX)
    capR = cv2.VideoCapture(config.CAM_RIGHT_INDEX)
    for c in (capL, capR):
        c.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAM_WIDTH)
        c.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAM_HEIGHT)
    for _ in range(5):
        capL.read(); capR.read()
    okL, fL = capL.read(); okR, fR = capR.read()
    capL.release(); capR.release()
    if not (okL and okR):
        raise RuntimeError("Kameralardan kare alinamadi.")
    return fL, fR


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--left"); ap.add_argument("--right")
    ap.add_argument("--cams", action="store_true", help="goruntuleri kameralardan al")
    ap.add_argument("--execute", action="store_true", help="FR3 programini uygula")
    ap.add_argument("--ply", default="", help="taban cercevesi bulutunu .ply yaz")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    import cv2
    from src.cell import run_cell
    from src.robot import FairinoFR3

    if args.cams:
        imgL, imgR = _grab_cams()
    else:
        if not (args.left and args.right):
            print("HATA: --left/--right ya da --cams gerekli", file=sys.stderr)
            return 2
        imgL, imgR = cv2.imread(args.left), cv2.imread(args.right)
        if imgL is None or imgR is None:
            print("HATA: goruntu okunamadi", file=sys.stderr)
            return 2

    res = run_cell(imgL, imgR)

    if args.ply and res.pointcloud_base is not None and len(res.pointcloud_base):
        open(args.ply, "wb").write(res.pointcloud_base.to_ply_bytes())
        print(f"[yazildi] {args.ply}  ({len(res.pointcloud_base)} nokta, robot taban cercevesi)")

    summ = res.summary()
    if args.json:
        print(json.dumps(summ, ensure_ascii=False, indent=2))
    else:
        print("\n=== HUCRE SONUCU ===")
        print(" ", res.message)
        if res.obb is not None:
            o = res.obb
            print(f"  olcu (cm): en {o.width_mm/10:.1f}  boy {o.height_mm/10:.1f}  "
                  f"kalinlik {o.thickness_mm/10:.1f}   yaw {o.yaw_deg:.0f} deg")
        if res.posture:
            print(f"  durus: {res.posture.label_tr}  (guven {res.posture.confidence:.0%})")
        if res.program:
            print(f"  FR3 program: {res.program.action}  uygulanabilir={res.program.feasible}")
            for k, a, n in res.program.steps:
                print(f"     - {n}")
        for w in res.warnings:
            print(f"  [uyari] {w}")

    if args.execute and res.program is not None:
        fr3 = FairinoFR3(enabled=True)
        fr3.connect()
        out = fr3.run_program(res.program)
        print("\n[FR3]", json.dumps(out, ensure_ascii=False, indent=2))

    return 0 if res.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
