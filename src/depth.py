"""
Disparite -> 3B nokta haritasi + derinlik haritasi.

reprojectImageTo3D ile her piksel icin (X, Y, Z) uretilir. |t|=1
alindigi icin bu koordinatlar "baseline birimi" cinsindendir; metrik
olcek src/scale.py'de uygulanir.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass

import numpy as np

from . import config
from .stereo import DisparityResult

logger = logging.getLogger(__name__)


@dataclass
class DepthResult:
    points_xyz: np.ndarray       # (H, W, 3) - baseline birimi (olceklenmemis)
    depth: np.ndarray            # (H, W) - Z kanali, gecersiz -> np.nan
    valid: np.ndarray            # (H, W) bool

    def scaled(self, scale: float) -> "DepthResult":
        return DepthResult(
            points_xyz=self.points_xyz * scale,
            depth=self.depth * scale,
            valid=self.valid.copy(),
        )


def reproject(disp_res: DisparityResult, Q: np.ndarray) -> DepthResult:
    import cv2

    disp = disp_res.disparity.copy()
    finite = np.isfinite(disp)

    disp_filled = np.where(finite, disp, 0.0).astype(np.float32)
    pts = cv2.reprojectImageTo3D(disp_filled, Q.astype(np.float64))

    z = pts[:, :, 2]
    valid = finite & np.isfinite(z) & (z > 0) & (np.abs(z) < 1e5)

    # Aykiri Z kirpma (medyandan N std)
    zc = z[valid]
    if zc.size > 50:
        med = np.median(zc)
        std = np.std(zc)
        keep = np.abs(z - med) < config.DEPTH_OUTLIER_STD_THRESHOLD * std + 1e-6
        valid &= keep

    depth = np.where(valid, z, np.nan)
    pts_out = pts.copy()
    pts_out[~valid] = np.nan

    logger.info("Reprojeksiyon: gecerli 3B piksel = %d", int(valid.sum()))
    return DepthResult(points_xyz=pts_out, depth=depth, valid=valid)


def colorize_depth(depth: np.ndarray) -> np.ndarray:
    import cv2

    finite = np.isfinite(depth)
    out = np.zeros((*depth.shape, 3), dtype=np.uint8)
    if finite.sum() == 0:
        return out
    lo, hi = np.nanpercentile(depth[finite], [2, 98])
    norm = np.clip((depth - lo) / max(hi - lo, 1e-6), 0, 1)
    # yakin = sicak: ters cevir
    norm8 = ((1.0 - np.nan_to_num(norm, nan=1.0)) * 255).astype(np.uint8)
    color = cv2.applyColorMap(norm8, cv2.COLORMAP_INFERNO)
    color[~finite] = (0, 0, 0)
    return color
