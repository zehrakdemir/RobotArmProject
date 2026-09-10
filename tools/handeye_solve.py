#!/usr/bin/env python3
"""
KAMERA -> ROBOT TABAN donusumu (eye-to-hand, GUI'siz).

Yontem: robot ucuna bir ArUco marker yapistir. Robotu N (>=6) farkli
poza gotur; her pozda hem L/R kare cek hem FR3 TCP pozunu kaydet
(tools/capture_pairs.py --fr3 bunu yapar). Sonra:

    docker compose run --rm valiz-olcum python -m tools.handeye_solve \
        --shots calib_shots --calib src/calibration/stereo_calib.yaml \
        --marker-mm 40 --aruco DICT_4X4_50 --tool-offset 0,0,0 \
        --out src/calibration/base_T_cam.yaml

Her pozda: rektifiye SOL karede marker merkezi -> stereo kamera cercevesinde
3B nokta.  FR3 TCP xyz (+ opsiyonel tool-offset) -> robot taban 3B nokta.
Umeyama (rigid) ile base_T_cam (4x4) cozulur; artik hata mm raporlanir.

Not: marker duz robot ucunda ise TCP ~ marker merkezi. Degilse
--tool-offset ile TCP'den marker merkezine vektoru (mm, robot taban DEGIL
tool cercevesi degil - basit yaklasim: kucukse ihmal edilebilir) verin.
"""
from __future__ import annotations

import argparse
import glob
import os

import numpy as np


def _umeyama(src: np.ndarray, dst: np.ndarray):
    """src (N,3) -> dst (N,3) rigid (R,t). Doner: R(3,3), t(3,), rms."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S, D = src - mu_s, dst - mu_d
    H = S.T @ D / len(src)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    t = mu_d - R @ mu_s
    rms = float(np.sqrt(np.mean(np.sum((dst - (src @ R.T + t)) ** 2, axis=1))))
    return R, t, rms


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--shots", default="calib_shots")
    ap.add_argument("--calib", default="src/calibration/stereo_calib.yaml")
    ap.add_argument("--marker-mm", type=float, required=True)
    ap.add_argument("--aruco", default="DICT_4X4_50")
    ap.add_argument("--tool-offset", default="0,0,0", help="TCP->marker (mm), robot taban approx")
    ap.add_argument("--out", default="src/calibration/base_T_cam.yaml")
    args = ap.parse_args()

    import cv2
    from src.calib import StereoRig, save_base_T_cam

    rig = StereoRig.load(args.calib)
    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.aruco))
    try:
        detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
        def _detect(g): return detector.detectMarkers(g)
    except AttributeError:  # eski opencv
        params = cv2.aruco.DetectorParameters_create()
        def _detect(g): return cv2.aruco.detectMarkers(g, aruco_dict, parameters=params)

    off = np.array([float(x) for x in args.tool_offset.split(",")], float)

    P1, P2 = rig.P1, rig.P2
    lefts = sorted(glob.glob(os.path.join(args.shots, "left_*.png")))
    cam_pts, base_pts = [], []
    for lp in lefts:
        idx = lp.split("left_")[-1].split(".")[0]
        rp = lp.replace("left_", "right_")
        pose_f = os.path.join(args.shots, f"pose_{idx}.txt")
        if not (os.path.exists(rp) and os.path.exists(pose_f)):
            continue
        imgL = cv2.imread(lp); imgR = cv2.imread(rp)
        lr, rr = rig.rectify(imgL, imgR)
        gL = cv2.cvtColor(lr, cv2.COLOR_BGR2GRAY)
        gR = cv2.cvtColor(rr, cv2.COLOR_BGR2GRAY)
        cL, idL, _ = _detect(gL)
        cR, idR, _ = _detect(gR)
        if idL is None or idR is None:
            print(f"  {idx}: marker yok"); continue
        # ortak ilk marker
        setL = {int(i): c for i, c in zip(idL.ravel(), cL)}
        setR = {int(i): c for i, c in zip(idR.ravel(), cR)}
        common = sorted(set(setL) & set(setR))
        if not common:
            print(f"  {idx}: ortak marker yok"); continue
        mid = common[0]
        ptL = setL[mid].reshape(-1, 2).mean(0)
        ptR = setR[mid].reshape(-1, 2).mean(0)
        X = cv2.triangulatePoints(P1, P2, ptL.reshape(2, 1), ptR.reshape(2, 1))
        X = (X[:3] / X[3]).ravel()                     # kamera cercevesi, mm
        tcp = np.loadtxt(pose_f)[:3] + off             # robot taban, mm
        cam_pts.append(X); base_pts.append(tcp)
        print(f"  {idx}: cam={np.round(X,1).tolist()}  base={np.round(tcp,1).tolist()}")

    if len(cam_pts) < 6:
        print(f"HATA: sadece {len(cam_pts)} gecerli ornek (>=6 gerekli).")
        return 1

    cam_pts = np.asarray(cam_pts); base_pts = np.asarray(base_pts)
    R, t, rms = _umeyama(cam_pts, base_pts)
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
    print(f"\nbase_T_cam artik hatasi (RMS) = {rms:.2f} mm")
    if rms > 10:
        print("UYARI: > 10 mm. Marker/tool-offset, kalibrasyon veya poz dagilimini kontrol edin.")
    save_base_T_cam(args.out, T, residual_mm=rms)
    print(f"yazildi -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
