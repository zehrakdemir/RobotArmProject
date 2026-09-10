"""
Bagil poza gore STEREO REKTIFIKASYON.

recoverPose'dan gelen (R, t_unit) ile iki goruntuyu ortak bir duzleme
hizalar; boylece epipolar cizgiler yatay olur ve SGBM yogun disparite
uretebilir.

|t| = 1 alindigi icin uretilen 3B koordinatlar "baseline birimi"
cinsindendir. Mutlak metrik olcek src/scale.py'de bilinen genislikle
uygulanir.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass

import numpy as np

from .camera import CameraIntrinsics
from .pose import RelativePose

logger = logging.getLogger(__name__)


@dataclass
class RectifiedPair:
    left: np.ndarray         # rektifiye sol goruntu (BGR veya gri)
    right: np.ndarray        # rektifiye sag goruntu
    Q: np.ndarray            # (4, 4) disparite -> 3B reprojeksiyon (baseline birimi)
    R1: np.ndarray
    R2: np.ndarray
    P1: np.ndarray
    P2: np.ndarray
    roi1: tuple
    roi2: tuple
    map1x: np.ndarray
    map1y: np.ndarray
    map2x: np.ndarray
    map2y: np.ndarray
    baseline_unit: float     # P2 icinden okunan baseline (genelde ~1.0)

    def remap_mask(self, mask: np.ndarray, which: str = "left") -> np.ndarray:
        """Bir ikili maskeyi rektifiye uzaya tasir."""
        import cv2

        mx, my = (self.map1x, self.map1y) if which == "left" else (self.map2x, self.map2y)
        warped = cv2.remap(
            mask.astype(np.uint8) * 255, mx, my, interpolation=cv2.INTER_NEAREST
        )
        return warped > 127


def rectify_pair(
    img_left: np.ndarray,
    img_right: np.ndarray,
    intr: CameraIntrinsics,
    pose: RelativePose,
) -> RectifiedPair:
    import cv2

    h, w = img_left.shape[:2]
    K = intr.K
    dist = intr.dist

    R = pose.R
    T = pose.t_unit.reshape(3, 1)

    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K, dist, K, dist, (w, h), R, T,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0.0,
    )

    map1x, map1y = cv2.initUndistortRectifyMap(K, dist, R1, P1, (w, h), cv2.CV_32FC1)
    map2x, map2y = cv2.initUndistortRectifyMap(K, dist, R2, P2, (w, h), cv2.CV_32FC1)

    left_r = cv2.remap(img_left, map1x, map1y, cv2.INTER_LINEAR)
    right_r = cv2.remap(img_right, map2x, map2y, cv2.INTER_LINEAR)

    # P2[0,3] = -f * baseline  ->  baseline = -P2[0,3] / f
    fx = P2[0, 0]
    baseline_unit = float(abs(P2[0, 3] / fx)) if fx != 0 else 1.0

    logger.info(
        "Rektifikasyon tamam | baseline_unit=%.4f | roi1=%s roi2=%s",
        baseline_unit, roi1, roi2,
    )
    return RectifiedPair(
        left=left_r, right=right_r, Q=Q,
        R1=R1, R2=R2, P1=P1, P2=P2, roi1=roi1, roi2=roi2,
        map1x=map1x, map1y=map1y, map2x=map2x, map2y=map2y,
        baseline_unit=baseline_unit,
    )
