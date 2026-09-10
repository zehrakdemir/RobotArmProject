#!/usr/bin/env python3
"""
Iki sabit kamera STEREO KALIBRASYONU (GUI'siz) + KOTU CIFT ATMA.

    docker compose run --rm valiz-olcum python -m tools.stereo_calibrate \
        --shots snaps --cols 9 --rows 6 --square-mm 24 \
        --out src/calibration/stereo_calib.yaml

  --cols / --rows : satranç tahtasindaki IC kose sayisi (kare degil!)
  --square-mm     : bir karenin kenari (mm) -> baseline ve 3B bu birimde cikar
  --keep          : outlier atmadan sonra tutulacak MIN cift sayisi (varsayilan 12)
  --max-view-rms  : bir cift bu pikselden fazla hata verirse atilir (varsayilan 1.0)

Cikti stereo_calib.yaml: K1,d1,K2,d2,R,T,image_size + RMS.
STEREO RMS < ~0.6 px ve rektifiye dikey hata < ~1.5 px iyi.
"""
from __future__ import annotations

import argparse
import glob
import os

import numpy as np


def _per_view_err(cv2, objp, imgp, K, d, rvecs, tvecs):
    errs = []
    for i in range(len(objp)):
        proj, _ = cv2.projectPoints(objp[i], rvecs[i], tvecs[i], K, d)
        a = imgp[i].reshape(-1, 2)
        b = proj.reshape(-1, 2)
        errs.append(float(np.sqrt(np.mean(np.sum((a - b) ** 2, axis=1)))))
    return np.array(errs)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--shots", default="calib_shots")
    ap.add_argument("--cols", type=int, required=True)
    ap.add_argument("--rows", type=int, required=True)
    ap.add_argument("--square-mm", type=float, required=True)
    ap.add_argument("--out", default="src/calibration/stereo_calib.yaml")
    ap.add_argument("--keep", type=int, default=12, help="outlier sonrasi min cift")
    ap.add_argument("--max-view-rms", type=float, default=1.0)
    args = ap.parse_args()

    import cv2
    from src.calib import save_stereo_calib

    pat = (args.cols, args.rows)
    objp = np.zeros((pat[0] * pat[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pat[0], 0:pat[1]].T.reshape(-1, 2) * args.square_mm

    lefts = sorted(glob.glob(os.path.join(args.shots, "left_*.png")))
    if not lefts:
        print(f"HATA: {args.shots}/left_*.png yok")
        return 2

    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 1e-4)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    objpts, ipL, ipR, names = [], [], [], []
    size = None
    for lp in lefts:
        rp = lp.replace("left_", "right_")
        if not os.path.exists(rp):
            continue
        gL = cv2.imread(lp, cv2.IMREAD_GRAYSCALE)
        gR = cv2.imread(rp, cv2.IMREAD_GRAYSCALE)
        if gL is None or gR is None:
            continue
        size = (gL.shape[1], gL.shape[0])
        okL, cL = cv2.findChessboardCorners(gL, pat, flags)
        okR, cR = cv2.findChessboardCorners(gR, pat, flags)
        if not (okL and okR):
            print(f"  atlandi (tahta yok): {os.path.basename(lp)}")
            continue
        cL = cv2.cornerSubPix(gL, cL, (11, 11), (-1, -1), crit)
        cR = cv2.cornerSubPix(gR, cR, (11, 11), (-1, -1), crit)
        objpts.append(objp); ipL.append(cL); ipR.append(cR); names.append(os.path.basename(lp))

    n = len(objpts)
    if n < 6:
        print(f"HATA: sadece {n} gecerli cift (>=6 gerekli).")
        return 1
    print(f"{n} gecerli cift.")

    keep = list(range(n))
    for it in range(4):
        op = [objpts[i] for i in keep]
        pl = [ipL[i] for i in keep]
        pr = [ipR[i] for i in keep]
        r1, K1, d1, rv1, tv1 = cv2.calibrateCamera(op, pl, size, None, None)
        r2, K2, d2, rv2, tv2 = cv2.calibrateCamera(op, pr, size, None, None)
        eL = _per_view_err(cv2, op, pl, K1, d1, rv1, tv1)
        eR = _per_view_err(cv2, op, pr, K2, d2, rv2, tv2)
        e = np.maximum(eL, eR)
        worst = float(e.max())
        print(f"  tur {it}: {len(keep)} cift | tek-RMS L={r1:.3f} R={r2:.3f} | "
              f"en kotu {worst:.2f}px ({names[keep[int(e.argmax())]]})")
        if worst <= args.max_view_rms or len(keep) <= max(args.keep, 6):
            break
        k = 2 if len(keep) > args.keep + 1 else 1
        drop = set(np.array(keep)[np.argsort(e)[-k:]].tolist())
        keep = [i for i in keep if i not in drop]

    op = [objpts[i] for i in keep]
    pl = [ipL[i] for i in keep]
    pr = [ipR[i] for i in keep]
    r1, K1, d1, _, _ = cv2.calibrateCamera(op, pl, size, None, None)
    r2, K2, d2, _, _ = cv2.calibrateCamera(op, pr, size, None, None)
    print(f"\nSON: {len(keep)}/{n} cift kullanildi | tek kamera RMS L={r1:.3f} R={r2:.3f}")

    # intrinsic'leri de ortak optimize et (FIX_INTRINSIC C310'larda dikey hata birakiyor)
    sc_flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_SAME_FOCAL_LENGTH
               | cv2.CALIB_FIX_ASPECT_RATIO)
    rms, K1, d1, K2, d2, R, T, E, F = cv2.stereoCalibrate(
        op, pl, pr, K1, d1, K2, d2, size, criteria=crit, flags=sc_flags,
    )
    print(f"STEREO RMS = {rms:.4f} px   baseline = {float(np.linalg.norm(T)):.1f} mm")
    # rektifiye dikey hata (epipolar hizalama) ~ F ile olc
    try:
        errs = []
        for a, b in zip(pl, pr):
            a2 = np.hstack([a.reshape(-1, 2), np.ones((len(a), 1))])
            b2 = np.hstack([b.reshape(-1, 2), np.ones((len(b), 1))])
            l = (F @ a2.T).T
            l /= np.linalg.norm(l[:, :2], axis=1, keepdims=True)
            errs.append(np.abs(np.sum(b2 * l, axis=1)))
        print(f"epipolar |ort| = {float(np.mean(np.concatenate(errs))):.2f} px")
    except Exception:  # noqa: BLE001
        pass
    save_stereo_calib(args.out, K1, d1, K2, d2, R, T, size, rms=float(rms))
    print(f"yazildi -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
