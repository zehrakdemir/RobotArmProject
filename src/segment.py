"""
Nokta bulutu ayiklama - ZEMIN kaldirma + gurultu temizleme + en buyuk kume.

Hizli Stereo (kalibrasyonsuz) modunda "sadece nesneyi al" icin. Kalibreli
Hucre Modu zaten YOLO maskesi + OBB kullaniyor, buna ihtiyac duymaz.

numpy (+ opsiyonel scipy). Birim onemli degil - esikler sahne olcegine
gore oranli.
"""
from __future__ import annotations

import logging

import numpy as np

logger = logging.getLogger(__name__)


def _scene_scale(pts: np.ndarray) -> float:
    """Sahnenin kaba buyuklugu (esikleri oranlamak icin)."""
    span = np.percentile(pts, 97, axis=0) - np.percentile(pts, 3, axis=0)
    return float(np.linalg.norm(span)) or 1.0


def fit_plane_ransac(pts: np.ndarray, thresh: float, iters: int = 250, seed: int = 0):
    """En cok noktayi barindiran duzlem. Doner: (normal(3,), d, inlier_mask)."""
    rng = np.random.default_rng(seed)
    n = len(pts)
    best_cnt, best = -1, (np.array([0, 0, 1.0]), 0.0, np.zeros(n, bool))
    for _ in range(iters):
        i = rng.choice(n, 3, replace=False)
        p0, p1, p2 = pts[i]
        nrm = np.cross(p1 - p0, p2 - p0)
        na = np.linalg.norm(nrm)
        if na < 1e-9:
            continue
        nrm = nrm / na
        d = float(-nrm @ p0)
        inl = np.abs(pts @ nrm + d) < thresh
        c = int(inl.sum())
        if c > best_cnt:
            best_cnt, best = c, (nrm, d, inl)
    nrm, d, inl = best
    if inl.sum() >= 3:  # inlier'larla yeniden uydur (kovaryans ozvektoru)
        q = pts[inl]
        c = q.mean(0)
        cov = np.cov((q - c).T)                 # 3x3
        w, v = np.linalg.eigh(cov)
        nrm = v[:, 0]                            # en kucuk ozdeger -> duzlem normali
        nrm = nrm / (np.linalg.norm(nrm) + 1e-12)
        d = float(-nrm @ c)
        inl = np.abs(pts @ nrm + d) < thresh
    return nrm, d, inl


def remove_ground(pts: np.ndarray, thresh_frac: float = 0.02):
    """
    Baskin duzlemi (zemin/masa) bul ve kaldir - AMA sadece geri kalan
    noktalar duzlemin BIR tarafinda toplaniyorsa (yani duzlem bir dayanak
    yuzeyi, ustundeki de nesne). Nesnenin kendi duz yuzu yanlislikla
    silinmesin diye. Doner: keep_mask (bool).
    """
    n = len(pts)
    if n < 50:
        return np.ones(n, bool)
    thresh = max(thresh_frac * _scene_scale(pts), 1e-6)
    nrm, d, inl = fit_plane_ransac(pts, thresh)
    frac = inl.mean()
    if not (0.12 <= frac <= 0.9):
        logger.info("Net dayanak duzlemi yok (frac=%.2f) - zemin kaldirilmadi.", frac)
        return np.ones(n, bool)

    signed = pts @ nrm + d                 # duzleme isaretli uzaklik
    off = signed[~inl]                     # duzlem disi noktalar
    if len(off) < 20:
        return np.ones(n, bool)
    same_side = max((off > thresh).mean(), (off < -thresh).mean())
    if same_side < 0.7:
        logger.info("Duzlem disi noktalar iki tarafa dagilmis (%.0f%%) - "
                    "zemin kaldirilmadi.", same_side * 100)
        return np.ones(n, bool)

    logger.info("Zemin/dayanak duzlemi kaldirildi: %d nokta (%.0f%%), "
                "nesne tek tarafta %%%.0f", int(inl.sum()), frac * 100, same_side * 100)
    return ~inl


def statistical_outlier_filter(pts: np.ndarray, k: int = 16, std_ratio: float = 2.0):
    """Komsularina ortalama uzakligi asiri olan noktalari at (SOR). scipy varsa."""
    if len(pts) < k + 1:
        return np.ones(len(pts), bool)
    try:
        from scipy.spatial import cKDTree
    except Exception:  # noqa: BLE001
        return np.ones(len(pts), bool)
    tree = cKDTree(pts)
    dist, _ = tree.query(pts, k=k + 1)
    mean_d = dist[:, 1:].mean(1)
    thr = mean_d.mean() + std_ratio * mean_d.std()
    keep = mean_d <= thr
    logger.info("SOR: %d/%d nokta tutuldu", int(keep.sum()), len(pts))
    return keep


def largest_cluster(pts: np.ndarray, voxel_frac: float = 0.03):
    """Voxel-izgara komsuluguna gore en buyuk bagli kume. scipy varsa."""
    if len(pts) < 50:
        return np.ones(len(pts), bool)
    try:
        from scipy.ndimage import label
    except Exception:  # noqa: BLE001
        return np.ones(len(pts), bool)
    v = max(voxel_frac * _scene_scale(pts), 1e-6)
    ijk = np.floor((pts - pts.min(0)) / v).astype(np.int64)
    dims = ijk.max(0) + 3
    if np.prod(dims) > 6_000_000:      # cok buyuk izgara - atla
        return np.ones(len(pts), bool)
    grid = np.zeros(dims, bool)
    grid[ijk[:, 0] + 1, ijk[:, 1] + 1, ijk[:, 2] + 1] = True
    lab, num = label(grid, structure=np.ones((3, 3, 3)))
    if num <= 1:
        return np.ones(len(pts), bool)
    sizes = np.bincount(lab.ravel())
    sizes[0] = 0
    big = int(np.argmax(sizes))
    keep = lab[ijk[:, 0] + 1, ijk[:, 1] + 1, ijk[:, 2] + 1] == big
    logger.info("En buyuk kume: %d/%d nokta (%d kume icinden)", int(keep.sum()), len(pts), num)
    return keep


def grid_depth_filter(pts: np.ndarray, up_axis: int = 2, cell_mm: float = 25.0,
                      band_mm: float = 45.0):
    """
    Derinlik-sisi temizleme: nesneyi up_axis'e dik duzlemde cell_mm'lik
    hucrelere bol; her hucrede medyan 'yukseklik'ten band_mm'den fazla sapan
    noktalari at (koyu/parlak yuzeyde stereo'nun urettigi yanlis-derinlik bulut).
    Doner: keep_mask (bool).
    """
    p = np.asarray(pts, float)
    if len(p) < 100:
        return np.ones(len(p), bool)
    ax = int(up_axis)
    hz = [i for i in range(3) if i != ax]
    ij = np.floor(p[:, hz] / max(cell_mm, 1.0)).astype(np.int64)
    # her hucre icin medyan up degeri
    order = np.lexsort((ij[:, 1], ij[:, 0]))
    keep = np.ones(len(p), bool)
    s = p[order, ax]
    ci = ij[order]
    start = 0
    for k in range(1, len(ci) + 1):
        if k == len(ci) or (ci[k] != ci[start]).any():
            seg = slice(start, k)
            med = np.median(s[seg])
            bad = np.abs(s[seg] - med) > band_mm
            keep[order[seg][bad]] = False
            start = k
    logger.info("grid_depth_filter: %d/%d nokta tutuldu (band %.0fmm)",
                int(keep.sum()), len(p), band_mm)
    return keep


def isolate_object(pts: np.ndarray, colors: np.ndarray | None = None,
                   drop_ground: bool = True, ground_thresh_frac: float = 0.02,
                   nearest_pct: float = 92.0, denoise: bool = True,
                   keep_largest: bool = True, depth_axis: int = 2):
    """
    Tum ayiklama adimlarini sirayla uygula. Doner: (pts_out, colors_out, info).

    SIRA (duz-yerde-duran nesne icin kritik):
      1. drop_ground: RANSAC ile zemin/masa duzlemini kaldir - ANA yontem.
         nearest_pct'ten ONCE calisir; yoksa nearest_pct zemin dilimini tutar.
      2. nearest_pct: zemin gittikten sonra kalan icinde en yakin %N derinlik
         (nesnenin arka/uzak sacilmalarini kirpar) - ikincil.
      3. denoise (SOR) + keep_largest (en buyuk bagli kume)
    """
    p = np.asarray(pts, float)
    c = None if colors is None else np.asarray(colors)
    keep = np.ones(len(p), bool)
    info = {"giris": len(p)}
    ax = int(depth_axis) if 0 <= int(depth_axis) < 3 else int(np.argmax(p.std(0)))

    if drop_ground and keep.sum() > 50:
        g = remove_ground(p[keep], ground_thresh_frac)
        idx = np.where(keep)[0]
        keep[idx[~g]] = False
        info["zemin_sonrasi"] = int(keep.sum())

    if nearest_pct and 0 < nearest_pct < 100 and keep.sum() > 20:
        sub = p[keep][:, ax]
        thr = np.percentile(sub, nearest_pct)
        idx = np.where(keep)[0]
        keep[idx[sub > thr]] = False
        info["en_yakin_%_sonrasi"] = int(keep.sum())

    if denoise and keep.sum() > 30:
        s = statistical_outlier_filter(p[keep])
        idx = np.where(keep)[0]
        keep[idx[~s]] = False
        info["gurultu_sonrasi"] = int(keep.sum())

    if keep_largest and keep.sum() > 50:
        cl = largest_cluster(p[keep])
        idx = np.where(keep)[0]
        keep[idx[~cl]] = False
        info["en_buyuk_kume_sonrasi"] = int(keep.sum())

    info["cikis"] = int(keep.sum())
    return p[keep], (None if c is None else c[keep]), info
