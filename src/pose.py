"""
Iki fotograf arasindaki BAGIL POZ (relative pose) tahmini.

Cikti: R (rotasyon) ve t (oteleme YONU - birim vektor). Oteleme
BUYUKLUGU (baseline) monokuler goruntulerden bilinemez; mutlak olcek
sonradan bilinen bir gercek olcuyle kalibre edilir (src/scale.py).

Yontem (klasik epipolar geometri):
  1. SIFT (yoksa ORB) ile ozellik cikar - valiz maskesi + margin bolgesinde.
  2. FLANN/BF + Lowe ratio testi ile eslestir.
  3. cv2.findEssentialMat (RANSAC) -> E.
  4. cv2.recoverPose -> R, t (|t| = 1).
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional

import numpy as np

from . import config
from .camera import CameraIntrinsics
from .detector import dilate_mask

logger = logging.getLogger(__name__)


@dataclass
class RelativePose:
    R: np.ndarray            # (3, 3)
    t_unit: np.ndarray       # (3,) birim oteleme yonu
    num_inliers: int
    num_matches: int
    pts1: np.ndarray         # (M, 2) inlier eslesme noktalari (sol)
    pts2: np.ndarray         # (M, 2) inlier eslesme noktalari (sag)
    method: str

    @property
    def baseline_angle_deg(self) -> float:
        """Rotasyonun aci buyuklugu (iki bakis arasindaki aci)."""
        r = self.R
        cos_theta = (np.trace(r) - 1.0) / 2.0
        cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
        return float(np.degrees(np.arccos(cos_theta)))


class PoseEstimationError(RuntimeError):
    pass


def _detector_and_matcher(cv2):
    """SIFT varsa SIFT+FLANN, yoksa ORB+BF dondur."""
    try:
        sift = cv2.SIFT_create(nfeatures=config.MAX_FEATURES)
        index_params = dict(algorithm=1, trees=5)  # FLANN_INDEX_KDTREE
        search_params = dict(checks=64)
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        return sift, flann, "SIFT+FLANN", cv2.NORM_L2
    except Exception:  # pragma: no cover
        orb = cv2.ORB_create(nfeatures=config.MAX_FEATURES)
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        return orb, bf, "ORB+BF", cv2.NORM_HAMMING


def _mask_for_matching(mask: Optional[np.ndarray], shape) -> Optional[np.ndarray]:
    if mask is None:
        return None
    dil = dilate_mask(mask, config.FEATURE_MASK_DILATE_PX)
    return (dil.astype(np.uint8) * 255)


def estimate_relative_pose(
    img1_gray: np.ndarray,
    img2_gray: np.ndarray,
    intr: CameraIntrinsics,
    mask1: Optional[np.ndarray] = None,
    mask2: Optional[np.ndarray] = None,
) -> RelativePose:
    import cv2

    det, matcher, method, _norm = _detector_and_matcher(cv2)

    m1 = _mask_for_matching(mask1, img1_gray.shape)
    m2 = _mask_for_matching(mask2, img2_gray.shape)

    kp1, des1 = det.detectAndCompute(img1_gray, m1)
    kp2, des2 = det.detectAndCompute(img2_gray, m2)
    if des1 is None or des2 is None or len(kp1) < config.MIN_MATCH_COUNT:
        raise PoseEstimationError(
            f"Yeterli ozellik bulunamadi (kp1={len(kp1) if kp1 else 0}, "
            f"kp2={len(kp2) if kp2 else 0}). Valiz yuzeyi cok dokusuz olabilir; "
            "fotograflara biraz arka plan/zemin dokusu ekleyin."
        )

    # KNN + Lowe ratio testi
    good = []
    try:
        knn = matcher.knnMatch(des1, des2, k=2)
        for pair in knn:
            if len(pair) < 2:
                continue
            a, b = pair
            if a.distance < config.FEATURE_MATCH_RATIO * b.distance:
                good.append(a)
    except cv2.error:  # ORB + FLANN uyumsuzlugu vb.
        good = sorted(matcher.match(des1, des2), key=lambda x: x.distance)[:200]

    if len(good) < config.MIN_MATCH_COUNT:
        raise PoseEstimationError(
            f"Guvenilir eslesme sayisi yetersiz ({len(good)} < {config.MIN_MATCH_COUNT})."
        )

    pts1 = np.float64([kp1[m.queryIdx].pt for m in good])
    pts2 = np.float64([kp2[m.trainIdx].pt for m in good])

    E, e_mask = cv2.findEssentialMat(
        pts1, pts2, intr.K,
        method=cv2.RANSAC,
        prob=config.RANSAC_PROB,
        threshold=config.RANSAC_THRESHOLD_PX,
    )
    if E is None or E.shape != (3, 3):
        raise PoseEstimationError("Essential matris tahmin edilemedi.")

    e_mask = e_mask.ravel().astype(bool)
    p1_in, p2_in = pts1[e_mask], pts2[e_mask]
    if len(p1_in) < config.MIN_MATCH_COUNT:
        raise PoseEstimationError(
            f"RANSAC sonrasi inlier yetersiz ({len(p1_in)})."
        )

    n_pose, R, t, pose_mask = cv2.recoverPose(E, p1_in, p2_in, intr.K)
    pose_mask = pose_mask.ravel().astype(bool)

    t_unit = t.ravel()
    norm = np.linalg.norm(t_unit)
    if norm < 1e-9:
        raise PoseEstimationError("Oteleme vektoru sifira yakin (saf rotasyon?).")
    t_unit = t_unit / norm

    rp = RelativePose(
        R=R,
        t_unit=t_unit,
        num_inliers=int(n_pose),
        num_matches=len(good),
        pts1=p1_in[pose_mask],
        pts2=p2_in[pose_mask],
        method=method,
    )
    logger.info(
        "Bagil poz: %s | eslesme=%d inlier=%d | bakis acisi farki=%.1f deg",
        method, rp.num_matches, rp.num_inliers, rp.baseline_angle_deg,
    )
    return rp
