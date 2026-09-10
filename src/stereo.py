"""
Yogun (dense) disparite: SGBM + WLS post-filtre.

Bu modul kullanicinin "diger projemde calisiyordu" dedigi mimariyi
korur:
  - StereoSGBM (3-way mode) sol ve sag matcher
  - cv2.ximgproc.createRightMatcher + createDisparityWLSFilter
  - WLS guven haritasi esigi altindaki pikseller NaN

cv2.ximgproc icin `opencv-contrib-python` gerekir.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass

import numpy as np

from . import config

logger = logging.getLogger(__name__)


@dataclass
class DisparityResult:
    disparity: np.ndarray        # (H, W) float32 - piksel; gecersiz -> np.nan
    confidence: np.ndarray       # (H, W) float32 - 0..255 (WLS)
    valid_ratio: float           # gecerli piksel orani (tum kare)
    num_disparities: int
    params_used: str = ""        # hangi (numDisp/block/uniq/wlsConf) ile hesaplandi
    roi_valid_ratio: float = -1.0  # verilirse: valiz maskesi icindeki gecerli oran
    probe: dict = None           # probe_rectified_disparity ciktisi


def probe_rectified_disparity(gray_l: np.ndarray, gray_r: np.ndarray) -> dict:
    """
    Rektifiye ciftte SEYREK ORB eslesmelerinden gercek disparite araligini ve
    DIKEY hizalama hatasini olcer. SGBM aramasini veriye gore ayarlamak icin.

    Doner: {"n", "n_horiz", "dy_med", "dx_med", "dx_lo", "dx_hi", "ok"}
      dy_med buyukse (> ~3 px) rektifikasyon bozuk demektir.
      dx_med isareti disparite yonunu verir (negatifse sol/sag ters).
    """
    import cv2

    orb = cv2.ORB_create(nfeatures=4000)
    k1, d1 = orb.detectAndCompute(gray_l, None)
    k2, d2 = orb.detectAndCompute(gray_r, None)
    out = dict(n=0, n_horiz=0, dy_med=999.0, dx_med=0.0, dx_lo=0.0, dx_hi=0.0, ok=False)
    if d1 is None or d2 is None or len(k1) < 12 or len(k2) < 12:
        return out

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    knn = bf.knnMatch(d1, d2, k=2)
    good = [a for pair in knn if len(pair) == 2 for a, b in [pair]
            if a.distance < 0.8 * b.distance]
    if len(good) < 12:
        return out

    dx, dy = [], []
    for m in good:
        p1 = k1[m.queryIdx].pt
        p2 = k2[m.trainIdx].pt
        dx.append(p1[0] - p2[0])
        dy.append(p1[1] - p2[1])
    dx = np.asarray(dx)
    dy = np.asarray(dy)

    horiz = np.abs(dy) < 3.0
    out["n"] = int(len(good))
    out["n_horiz"] = int(horiz.sum())
    out["dy_med"] = float(np.median(np.abs(dy)))
    src = dx[horiz] if horiz.sum() >= 8 else dx
    out["dx_med"] = float(np.median(src))
    out["dx_lo"] = float(np.percentile(src, 5))
    out["dx_hi"] = float(np.percentile(src, 95))
    out["ok"] = out["n_horiz"] >= 8 and out["dy_med"] < 3.0
    logger.info(
        "Disparite probe: n=%d yatay=%d dy_med=%.1f dx=[%.0f..%.0f] med=%.0f ok=%s",
        out["n"], out["n_horiz"], out["dy_med"], out["dx_lo"], out["dx_hi"],
        out["dx_med"], out["ok"],
    )
    return out


def _round16(v: float) -> int:
    return int(np.floor(v / 16.0) * 16)


def _make_sgbm(cv2, block_size: int, num_disp: int, uniqueness: int, min_disp: int):
    channels = 3
    return cv2.StereoSGBM_create(
        minDisparity=min_disp,
        numDisparities=num_disp,
        blockSize=block_size,
        P1=8 * channels * block_size * block_size,
        P2=32 * channels * block_size * block_size,
        disp12MaxDiff=config.SGBM_DISP12_MAX_DIFF,
        uniquenessRatio=uniqueness,
        speckleWindowSize=config.SGBM_SPECKLE_WINDOW,
        speckleRange=config.SGBM_SPECKLE_RANGE,
        preFilterCap=config.SGBM_PRE_FILTER_CAP,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )


def compute_disparity(
    left: np.ndarray,
    right: np.ndarray,
    num_disparities: int | None = None,
    block_size: int | None = None,
    uniqueness: int | None = None,
    wls_conf: float | None = None,
    min_disparity: int | None = None,
    roi_mask: np.ndarray | None = None,
) -> DisparityResult:
    import cv2

    if not hasattr(cv2, "ximgproc"):
        raise RuntimeError(
            "cv2.ximgproc yok. `pip install opencv-contrib-python` gerekli "
            "(opencv-python ile birlikte kurulmamali)."
        )

    num_disp = int(num_disparities or config.SGBM_NUM_DISPARITIES)
    num_disp = max(16, (num_disp // 16) * 16)
    block_size = int(block_size or config.SGBM_BLOCK_SIZE)
    uniq = config.SGBM_UNIQUENESS_RATIO if uniqueness is None else int(uniqueness)
    conf_th = config.WLS_CONFIDENCE_THRESHOLD if wls_conf is None else float(wls_conf)
    min_disp = config.SGBM_MIN_DISPARITY if min_disparity is None else int(min_disparity)

    left_matcher = _make_sgbm(cv2, block_size, num_disp, uniq, min_disp)
    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

    wls = cv2.ximgproc.createDisparityWLSFilter(left_matcher)
    wls.setLambda(config.WLS_LAMBDA)
    wls.setSigmaColor(config.WLS_SIGMA)

    disp_l = left_matcher.compute(left, right)          # int16, x16 olcekli
    disp_r = right_matcher.compute(right, left)

    filtered = wls.filter(disp_l, left, disparity_map_right=disp_r)
    confidence = wls.getConfidenceMap()

    disp = filtered.astype(np.float32) / 16.0
    conf = confidence.astype(np.float32) if confidence is not None else np.full_like(disp, 255.0)

    # SGBM "eslesme yok" sentineli: (min_disp - 1). Onun disindaki her seyi tut.
    invalid = (
        (disp < (min_disp - 0.5))
        | (conf < conf_th)
        | ~np.isfinite(disp)
    )
    disp[invalid] = np.nan

    valid_ratio = float(np.isfinite(disp).mean())
    roi_ratio = -1.0
    if roi_mask is not None and roi_mask.any():
        roi_ratio = float(np.isfinite(disp[roi_mask]).mean())
    tag = f"minDisp={min_disp} numDisp={num_disp} block={block_size} uniq={uniq} wlsConf={conf_th:.0f}"
    logger.info("Disparite: %s | gecerli(kare)=%.1f%% | gecerli(ROI)=%.1f%%",
                tag, valid_ratio * 100.0, roi_ratio * 100.0)
    return DisparityResult(
        disparity=disp,
        confidence=conf,
        valid_ratio=valid_ratio,
        num_disparities=num_disp,
        params_used=tag,
        roi_valid_ratio=roi_ratio,
    )


def compute_disparity_adaptive(
    left: np.ndarray,
    right: np.ndarray,
    roi_mask: np.ndarray | None = None,
    target_roi_ratio: float = 0.25,
) -> DisparityResult:
    """
    1) Seyrek ORB probe ile GERCEK disparite araligini (isaret dahil) ve dikey
       hizalama hatasini olcer.
    2) SGBM min/numDisparities'i bu araliga gore ayarlar (negatif olabilir).
    3) Gerekirse (dusuk ROI) blok/uniqueness/WLS esigini kademeli gevsetir.
    EN IYI sonucu dondurur; probe bilgisini de tasir.
    """
    h, w = left.shape[:2]
    probe = probe_rectified_disparity(left, right)

    # --- disparite arama araligi ---
    if probe["ok"] or probe["n"] >= 12:
        margin = max(32.0, 0.25 * (probe["dx_hi"] - probe["dx_lo"]))
        lo = _round16(probe["dx_lo"] - margin)
        span = probe["dx_hi"] + margin - lo
        nd = int(np.ceil(span / 16.0) * 16)
        nd = int(np.clip(nd, 64, (min(int(w * 0.6), 640) // 16) * 16))
        min_d = int(np.clip(lo, -256, 256))
    else:
        # probe yok - kore genisligine gore genis tara
        min_d = 0
        nd = max(16, (min(int(w * 0.35), 512) // 16) * 16)

    wide_nd = int(np.clip(nd * 2, nd, (min(int(w * 0.8), 768) // 16) * 16))

    ladder = [
        dict(num_disparities=nd, min_disparity=min_d, block_size=config.SGBM_BLOCK_SIZE,
             uniqueness=config.SGBM_UNIQUENESS_RATIO, wls_conf=config.WLS_CONFIDENCE_THRESHOLD),
        dict(num_disparities=nd, min_disparity=min_d, block_size=5, uniqueness=8, wls_conf=80.0),
        dict(num_disparities=wide_nd, min_disparity=min_d, block_size=3, uniqueness=5, wls_conf=55.0),
        dict(num_disparities=wide_nd, min_disparity=min_d, block_size=7, uniqueness=3, wls_conf=35.0),
    ]

    best: DisparityResult | None = None
    for i, p in enumerate(ladder):
        try:
            r = compute_disparity(left, right, roi_mask=roi_mask, **p)
        except Exception as exc:  # noqa: BLE001
            logger.warning("Disparite denemesi %d basarisiz: %s", i, exc)
            continue
        r.probe = probe
        score = r.roi_valid_ratio if r.roi_valid_ratio >= 0 else r.valid_ratio
        best_score = (best.roi_valid_ratio if best and best.roi_valid_ratio >= 0
                      else (best.valid_ratio if best else -1.0))
        if best is None or score > best_score:
            best = r
        if score >= target_roi_ratio:
            break
    if best is None:
        raise RuntimeError("Disparite hicbir parametre kombinasyonuyla hesaplanamadi.")
    return best


def colorize_disparity(disp: np.ndarray) -> np.ndarray:
    """Gorsellestirme icin disparite -> renkli (BGR) harita."""
    import cv2

    finite = np.isfinite(disp)
    out = np.zeros((*disp.shape, 3), dtype=np.uint8)
    if finite.sum() == 0:
        return out
    lo, hi = np.nanpercentile(disp[finite], [2, 98])
    norm = np.clip((disp - lo) / max(hi - lo, 1e-6), 0, 1)
    norm8 = (np.nan_to_num(norm, nan=0.0) * 255).astype(np.uint8)
    color = cv2.applyColorMap(norm8, cv2.COLORMAP_TURBO)
    color[~finite] = (0, 0, 0)
    return color
