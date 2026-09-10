#!/usr/bin/env python3
"""
CEVRIMICI STEREO OZ-KALIBRASYON  (satranc tahtasi YOK).

Kameralar oturumlar arasi kayar/doner (kullanici sabitleyemiyor). LENS ic
parametreleri (K1,d1,K2,d2) kamera oynayinca DEGISMEZ; sadece kameralar-arasi
POZ (R,T) degisir. Bu modul dokulu sahnedeki ozellik eslesmelerinden R,T'yi
yeniden kestirir:

  ORB+AKAZE eslesme -> findEssentialMat(RANSAC) -> recoverPose
  -> Sampson hatasi uzerinde Levenberg-Marquardt ince ayar
  -> |T| ilk kalibrasyonun baseline'ina kilitli (kameralar dondu, uzaklasmadi)

`refresh_stereo_extrinsics()` sonucu `src/calibration/stereo_calib.yaml`'a yazar
(bind-mount -> konteyner yeniden baslatmaya gerek yok, sonraki oturumda gecerli).
`multi.detect_objects` gecerli disparite dususunce bunu otomatik cagirir.
"""
from __future__ import annotations

import logging

import numpy as np

logger = logging.getLogger(__name__)

_AUTO_DONE = False  # process basina bir kez otomatik dene


def _sampson(params, u1, u2):
    """Normalize noktalar icin Sampson epipolar hatasi (rvec+tvec parametreli)."""
    import cv2
    R = cv2.Rodrigues(np.asarray(params[:3], np.float64))[0]
    t = np.asarray(params[3:6], np.float64)
    tx = np.array([[0, -t[2], t[1]], [t[2], 0, -t[0]], [-t[1], t[0], 0]], np.float64)
    E = tx @ R
    x1 = np.hstack([u1, np.ones((len(u1), 1))])
    x2 = np.hstack([u2, np.ones((len(u2), 1))])
    Ex1 = x1 @ E.T
    Etx2 = x2 @ E
    num = np.sum(x2 * Ex1, axis=1)
    den = Ex1[:, 0] ** 2 + Ex1[:, 1] ** 2 + Etx2[:, 0] ** 2 + Etx2[:, 1] ** 2
    return num / np.sqrt(den + 1e-12)


def _matches(gL, gR):
    import cv2
    p1, p2 = [], []
    dets = [(cv2.ORB_create(12000, scaleFactor=1.2, nlevels=8), cv2.NORM_HAMMING)]
    try:
        dets.append((cv2.AKAZE_create(threshold=1e-3), cv2.NORM_HAMMING))
    except Exception:  # noqa: BLE001
        pass
    for det, norm in dets:
        ka, da = det.detectAndCompute(gL, None)
        kb, db = det.detectAndCompute(gR, None)
        if da is None or db is None or len(ka) < 100:
            continue
        for pair in cv2.BFMatcher(norm).knnMatch(da, db, k=2):
            if len(pair) != 2:
                continue
            m, n = pair
            if m.distance < 0.78 * n.distance:
                p1.append(ka[m.queryIdx].pt)
                p2.append(kb[m.trainIdx].pt)
    return np.float32(p1), np.float32(p2)


def estimate_extrinsics(imgL, imgR, K1, d1, K2, d2, baseline_mm):
    """Doner: (R 3x3, T 3x1 [mm], info dict). Hicbir dosya yazmaz."""
    import cv2
    from scipy.optimize import least_squares
    gL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY) if imgL.ndim == 3 else imgL
    gR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY) if imgR.ndim == 3 else imgR
    p1, p2 = _matches(gL, gR)
    if len(p1) < 120:
        raise RuntimeError(f"cok az eslesme ({len(p1)}) - doku/isik yetersiz")
    u1 = cv2.undistortPoints(p1.reshape(-1, 1, 2), K1, d1).reshape(-1, 2)
    u2 = cv2.undistortPoints(p2.reshape(-1, 1, 2), K2, d2).reshape(-1, 2)
    E, mask = cv2.findEssentialMat(u1, u2, np.eye(3), method=cv2.RANSAC,
                                   prob=0.9999, threshold=8e-4)
    if E is None or E.shape != (3, 3):
        raise RuntimeError("Essential matris kestirilemedi")
    n_in, R, t, mask2 = cv2.recoverPose(E, u1, u2, np.eye(3), mask=mask)
    keep = (mask2.ravel() > 0) if mask2 is not None else np.ones(len(u1), bool)
    if int(keep.sum()) < 60:
        raise RuntimeError(f"recoverPose inlier az ({int(keep.sum())})")
    t = t.reshape(3) / (np.linalg.norm(t) + 1e-12)

    x0 = np.hstack([cv2.Rodrigues(R)[0].ravel(), t])
    ui1, ui2 = u1[keep], u2[keep]
    rms0 = float(np.sqrt(np.mean(_sampson(x0, ui1, ui2) ** 2)))
    try:
        sol = least_squares(_sampson, x0, args=(ui1, ui2), method="lm", max_nfev=400)
        rms1 = float(np.sqrt(np.mean(_sampson(sol.x, ui1, ui2) ** 2)))
        if rms1 <= rms0:
            R = cv2.Rodrigues(sol.x[:3])[0]
            t = sol.x[3:6]
        else:
            rms1 = rms0
    except Exception as exc:  # noqa: BLE001
        logger.warning("LM ince ayar atlandi: %s", exc)
        rms1 = rms0

    t = t / (np.linalg.norm(t) + 1e-12) * float(baseline_mm)
    if t[0] < 0:                       # sag kamera +X'te olmali
        t = -t
    ang = float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))
    info = {"eslesme": int(len(p1)), "inlier": int(keep.sum()),
            "sampson_rms_x1e3": (round(rms0 * 1e3, 3), round(rms1 * 1e3, 3)),
            "T_mm": [round(float(v), 2) for v in t], "kameralar_arasi_aci_deg": round(ang, 2)}
    return R, t.reshape(3, 1), info


def rectify_y_error(imgL, imgR, K1, d1, K2, d2, size, R, T):
    """Bagimsiz dogrulama: verilen R,T ile rektifiye edip artik dikey hatayi olc."""
    import cv2
    R1, R2, P1, P2, _Q, _, _ = cv2.stereoRectify(
        K1, d1, K2, d2, size, R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0.0)
    m1x, m1y = cv2.initUndistortRectifyMap(K1, d1, R1, P1, size, cv2.CV_32FC1)
    m2x, m2y = cv2.initUndistortRectifyMap(K2, d2, R2, P2, size, cv2.CV_32FC1)
    lr = cv2.remap(imgL, m1x, m1y, cv2.INTER_LINEAR)
    rr = cv2.remap(imgR, m2x, m2y, cv2.INTER_LINEAR)
    g1 = cv2.cvtColor(lr, cv2.COLOR_BGR2GRAY); g2 = cv2.cvtColor(rr, cv2.COLOR_BGR2GRAY)
    a1, b1 = cv2.ORB_create(5000).detectAndCompute(g1, None)
    a2, b2 = cv2.ORB_create(5000).detectAndCompute(g2, None)
    if b1 is None or b2 is None:
        return 999.0
    ms = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(b1, b2)
    if len(ms) < 30:
        return 999.0
    q1 = np.float32([a1[m.queryIdx].pt for m in ms])
    q2 = np.float32([a2[m.trainIdx].pt for m in ms])
    dx = q1[:, 0] - q2[:, 0]; dy = q1[:, 1] - q2[:, 1]
    ok = (dx > 2) & (dx < 500)
    return float(np.median(np.abs(dy[ok]))) if ok.sum() >= 15 else 999.0


def refresh_stereo_extrinsics(imgL, imgR, rig=None, out_path=None, write=True):
    """Tam akis: kestir + dogrula + (istege bagli) yaml'a yaz. Doner: info dict.

    write=True ve yeni kalibrasyon eskisinden KOTU degilse yaml guncellenir.
    """
    from . import config
    from .calib import StereoRig, save_stereo_calib
    rig = rig or StereoRig.load()
    out_path = out_path or config.STEREO_CALIB_PATH
    size = rig.image_size

    before = rectify_y_error(imgL, imgR, rig.K1, rig.d1, rig.K2, rig.d2,
                             size, rig.R, rig.T)
    R, T, info = estimate_extrinsics(imgL, imgR, rig.K1, rig.d1, rig.K2, rig.d2,
                                     rig.baseline_mm)
    after = rectify_y_error(imgL, imgR, rig.K1, rig.d1, rig.K2, rig.d2, size, R, T)
    info["rektifiye_dikey_hata_px"] = {"once": round(before, 2), "sonra": round(after, 2)}

    worse = after > before + 0.3 and after > 1.5
    if write and not worse:
        save_stereo_calib(out_path, rig.K1, rig.d1, rig.K2, rig.d2, R, T, size,
                          rms=float(after))
        info["yazildi"] = out_path
    else:
        info["yazildi"] = None
        if worse:
            info["uyari"] = "yeni kalibrasyon daha kotu - yazilmadi (sahne dokusuz?)"
    return info


def maybe_auto_refresh(imgL, imgR, rig, valid_frac):
    """detect_objects icinden cagrilir. Gecerli disparite dusukse bir kez oz-kalibre
    et ve TAZE rig dondur; aksi halde None (cagiran mevcut rig'i kullanir)."""
    global _AUTO_DONE
    from . import config
    thr = float(getattr(config, "SELFCAL_MIN_VALID_FRAC", 0.18))
    if _AUTO_DONE or valid_frac >= thr:
        return None
    _AUTO_DONE = True
    try:
        info = refresh_stereo_extrinsics(imgL, imgR, rig=rig, write=True)
    except Exception as exc:  # noqa: BLE001
        logger.warning("otomatik oz-kalibrasyon basarisiz: %s", exc)
        return None
    logger.warning("OTOMATIK STEREO OZ-KALIBRASYON (gecerli disparite %%%.1f < %%%.0f): %s",
                   valid_frac * 100, thr * 100, info)
    if not info.get("yazildi"):
        return None
    from .calib import StereoRig
    return StereoRig.load()


if __name__ == "__main__":
    import argparse
    import glob
    import os
    import cv2
    from . import config
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--left"); ap.add_argument("--right")
    ap.add_argument("--no-write", action="store_true")
    a = ap.parse_args()
    if a.left and a.right:
        lp, rp = a.left, a.right
    else:
        lp = sorted(glob.glob(os.path.join(config.SNAP_DIR, "left_*.png")))[-1]
        rp = lp.replace("left_", "right_")
    print("cift:", os.path.basename(lp))
    out = refresh_stereo_extrinsics(cv2.imread(lp), cv2.imread(rp), write=not a.no_write)
    for k, v in out.items():
        print(f"  {k}: {v}")
