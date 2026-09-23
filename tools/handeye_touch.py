#!/usr/bin/env python3
"""
HAND-EYE (dokunma + satranc tahtasi) - gripper'a marker YOK.

Tahtayi masaya duz koy (iki kamerada da tam gorunsun). Robotu elle jog'la,
TCP ucunu bir IC KOSEYE dokundur, sonra:

    docker compose run --rm valiz-olcum python -m tools.handeye_touch --corner 0,0
    ... (baska koseye dokundur) ...
    docker compose run --rm valiz-olcum python -m tools.handeye_touch --corner 8,0
    ... 5-6 kose ...
    docker compose run --rm valiz-olcum python -m tools.handeye_touch --solve

--corner r,c : (satir, sutun) iç köşe indeksi. 9x6 tahtada r=0..8, c=0..5.
--tool-offset x,y,z : TCP'den dokunan uca vektor (mm, robot taban ~). Uç TCP ise 0,0,0.
--square-mm / --cols / --rows : tahta parametreleri (varsayilan 24 / 9 / 6).
"""
from __future__ import annotations

import argparse
import glob
import json
import os

import numpy as np
import requests

from src import config

PTS = os.path.join(config.SNAP_DIR, "handeye", "touch_points.json")
# Robot artik ROS2/MoveIt (movecobot-web) uzerinden kontrol ediliyor - ayni anda
# HAM Fairino SDK ile de baglanmaya calismak (eski yontem) cakisma/fault riski
# tasiyor. TCP pozunu, zaten calisan ROS2 API'sinden okuyoruz (bkz.
# tools/pick_and_place.py - ayni desen).
ROS2_API = "http://host.docker.internal:8088"


def _umeyama(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S, D = src - mu_s, dst - mu_d
    U, _, Vt = np.linalg.svd(S.T @ D / len(src))
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    t = mu_d - R @ mu_s
    rms = float(np.sqrt(np.mean(np.sum((dst - (src @ R.T + t)) ** 2, axis=1))))
    return R, t, rms


def _board_corner_cam(cols, rows, square_mm, corner_rc):
    """Son stereo cift -> satranc ic koseleri -> istenen kosenin 3B kamera noktasi."""
    import cv2
    from src.calib import StereoRig
    from src.cell import _epipolar_correct
    rig = StereoRig.load()
    lp = sorted(glob.glob(os.path.join(config.SNAP_DIR, "left_*.png")))[-1]
    L = cv2.imread(lp); R = cv2.imread(lp.replace("left_", "right_"))
    lr, rr = rig.rectify(L, R)
    rr, _dy, _ = _epipolar_correct(lr, rr)
    gL = cv2.cvtColor(lr, cv2.COLOR_BGR2GRAY)
    gR = cv2.cvtColor(rr, cv2.COLOR_BGR2GRAY)
    pat = (cols, rows)
    fl = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    okL, cL = cv2.findChessboardCorners(gL, pat, fl)
    okR, cR = cv2.findChessboardCorners(gR, pat, fl)
    if not (okL and okR):
        raise RuntimeError(f"tahta bulunamadi (L={okL} R={okR}) - {os.path.basename(lp)}")
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-3)
    cL = cv2.cornerSubPix(gL, cL, (9, 9), (-1, -1), crit).reshape(rows, cols, 2)
    cR = cv2.cornerSubPix(gR, cR, (9, 9), (-1, -1), crit).reshape(rows, cols, 2)
    r, c = corner_rc
    pL = cL[r, c].reshape(2, 1)
    pR = cR[r, c].reshape(2, 1)
    X = cv2.triangulatePoints(rig.P1, rig.P2, pL, pR)
    return (X[:3] / X[3]).ravel().tolist(), os.path.basename(lp)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--corner", help="r,c ic kose indeksi")
    ap.add_argument("--solve", action="store_true")
    ap.add_argument("--reset", action="store_true", help="kayitli noktalari sil")
    ap.add_argument("--tool-offset", default="0,0,0")
    ap.add_argument("--square-mm", type=float, default=24.0)
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--out", default="src/calibration/base_T_cam.yaml")
    args = ap.parse_args()
    os.makedirs(os.path.dirname(PTS), exist_ok=True)

    if args.reset:
        if os.path.exists(PTS):
            os.remove(PTS)
        print("noktalar silindi"); return 0

    data = json.load(open(PTS)) if os.path.exists(PTS) else []

    if args.corner:
        r, c = (int(x) for x in args.corner.split(","))
        cam, nm = _board_corner_cam(args.cols, args.rows, args.square_mm, (r, c))
        st = requests.get(f"{ROS2_API}/api/state", timeout=10).json()
        if not st.get("ready"):
            print("ROS2 arayuzu hazir degil:", st.get("startup_error")); return 1
        tp = st.get("tcp_pose") or {}
        if not tp:
            print("TCP pozu okunamadi (tcp_pose bos)"); return 1
        off = np.array([float(x) for x in args.tool_offset.split(",")])
        tcp = (np.array([tp["x"], tp["y"], tp["z"]]) + off).tolist()
        data = [d for d in data if tuple(d["rc"]) != (r, c)]     # ayni koseyi guncelle
        data.append({"rc": [r, c], "cam_mm": cam, "base_mm": tcp, "cift": nm})
        json.dump(data, open(PTS, "w"), indent=1)
        print(f"kose ({r},{c})  kamera={[round(v,1) for v in cam]}  "
              f"robot={[round(v,1) for v in tcp]}   toplam {len(data)} nokta")
        return 0

    if args.solve:
        if len(data) < 4:
            print(f"HATA: sadece {len(data)} nokta (>=4 gerekli)"); return 1
        src = np.array([d["cam_mm"] for d in data])
        dst = np.array([d["base_mm"] for d in data])
        R, t, rms = _umeyama(src, dst)
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
        from src.calib import save_base_T_cam
        save_base_T_cam(args.out, T, residual_mm=rms)
        print(f"base_T_cam artik hata (RMS) = {rms:.1f} mm   ({len(data)} nokta)")
        if rms > 15:
            print("UYARI: >15 mm. Dokunuslar/tahta/kalibrasyon kontrol.")
        print(f"yazildi -> {args.out}")
        return 0

    print(f"kayitli {len(data)} nokta:")
    for d in data:
        print(f"  ({d['rc'][0]},{d['rc'][1]})  kam={[round(v,1) for v in d['cam_mm']]}  "
              f"rob={[round(v,1) for v in d['base_mm']]}")
    print("--corner r,c veya --solve kullan")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
