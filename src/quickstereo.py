"""
HIZLI STEREO cekirdegi - kalibrasyon YOK, tespit YOK.

Iki goruntu -> poz kestir -> rektifiye -> adaptif SGBM+WLS -> 3B.
Kameralari nisanlarken / hizalarken "bu ikili is goruyor mu" ve
disparite / derinlik / bulut'u ARAYUZDE gormek icin.

Hem tools/quick_stereo.py (CLI) hem pages/2_Hizli_Stereo.py bunu kullanir.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class QuickResult:
    ok: bool
    message: str

    rect_left: Optional[np.ndarray] = None
    rect_right: Optional[np.ndarray] = None
    disparity: Optional[np.ndarray] = None        # (H,W) float, gecersiz -> nan
    disparity_color: Optional[np.ndarray] = None  # (H,W,3) BGR
    depth: Optional[np.ndarray] = None           # (H,W) float (baseline birimi)
    depth_color: Optional[np.ndarray] = None
    points_xyz: Optional[np.ndarray] = None      # (H,W,3) baseline birimi
    roi_mask: Optional[np.ndarray] = None
    swapped: bool = False
    stats: dict = field(default_factory=dict)

    def pointcloud(self, isolate: bool = False, drop_ground: bool = False,
                   ground_thresh_frac: float = 0.02, nearest_pct: float = 60.0,
                   denoise: bool = True, keep_largest: bool = True):
        """
        src.pointcloud.PointCloud (renkli) dondurur; yoksa None.
        isolate=True: zemin kaldir + gurultu temizle + en buyuk kume -> SADECE nesne.
        `last_isolate_info` calisma bilgisini tasir.
        """
        if self.points_xyz is None:
            return None
        from .pointcloud import PointCloud, extract
        finite = np.isfinite(self.points_xyz).all(axis=2)
        m = (self.roi_mask & finite) if self.roi_mask is not None else finite
        pc = extract(self.points_xyz, m, color_bgr=self.rect_left, only_suitcase=True)
        self.last_isolate_info = {}
        if not isolate or len(pc) < 50:
            return pc
        from .segment import isolate_object
        xyz, rgb, info = isolate_object(
            pc.xyz, pc.rgb, drop_ground=drop_ground,
            ground_thresh_frac=ground_thresh_frac, nearest_pct=nearest_pct,
            denoise=denoise, keep_largest=keep_largest, depth_axis=2)
        self.last_isolate_info = info
        return PointCloud(xyz=xyz, rgb=rgb)


def run_quick_stereo(image_left: np.ndarray, image_right: np.ndarray,
                     focal_px: float = 0.0, center_roi: float = 0.6,
                     use_rig: bool = False,
                     roi_box: tuple | None = None) -> QuickResult:
    """roi_box: (x0,x1,y0,y1) 0..1 kesir - verilirse NESNE kutusu; bulut ve
    olcum sadece bu kutudan. Yoksa center_roi kullanilir."""
    """
    use_rig=True: stereo_calib.yaml kullan (KALIBRELI - metrik mm, duz zemin,
    odak/poz tahmini YOK). False: kalibrasyonsuz, poz kestirimi + odak tahmini.
    """
    import cv2

    from .camera import estimate_intrinsics
    from .config import RunConfig
    from .depth import colorize_depth, reproject
    from .pose import estimate_relative_pose
    from .rectify import rectify_pair
    from .stereo import colorize_disparity, compute_disparity_adaptive

    gL = cv2.cvtColor(image_left, cv2.COLOR_BGR2GRAY)
    gR = cv2.cvtColor(image_right, cv2.COLOR_BGR2GRAY)

    swapped = False
    metric = False
    if use_rig:
        from .calib import StereoRig
        try:
            rig = StereoRig.load()
        except FileNotFoundError as exc:
            return QuickResult(False, f"Kalibrasyon dosyasi yok: {exc}  "
                               "Once tools.stereo_calibrate calistirin.")
        lr, rr = rig.rectify(image_left, image_right)

        class _R:  # rectify_pair ile ayni arayuz
            pass
        rect = _R(); rect.left, rect.right = lr, rr
        rect.Q = rig.Q
        metric = True
    else:
        intr = estimate_intrinsics(image_left, RunConfig(manual_focal_px=focal_px))
        try:
            pose = estimate_relative_pose(gL, gR, intr)
        except Exception as exc:  # noqa: BLE001
            return QuickResult(False,
                f"Poz kestirilemedi: {exc}  ->  iki goruntu cok farkli / dokusuz. "
                "Kameralari yaklastir, ayni sahneye baktir, dokulu bir sey koy, "
                "ya da odak degerini duzelt (C310 @720p ~1150).")
        med_dx = float(np.median(pose.pts1[:, 0] - pose.pts2[:, 0])) if len(pose.pts1) else 0.0
        if med_dx < -1.0:
            image_left, image_right = image_right, image_left
            gL, gR = gR, gL
            pose = estimate_relative_pose(gL, gR, intr)
            swapped = True
        rect = rectify_pair(image_left, image_right, intr, pose)

    h, w = rect.left.shape[:2]
    m = np.zeros((h, w), bool)
    if roi_box is not None:
        x0, x1, y0, y1 = roi_box
        x0, x1 = sorted((float(np.clip(x0, 0, 1)), float(np.clip(x1, 0, 1))))
        y0, y1 = sorted((float(np.clip(y0, 0, 1)), float(np.clip(y1, 0, 1))))
        m[int(h*y0):max(int(h*y1), int(h*y0)+1), int(w*x0):max(int(w*x1), int(w*x0)+1)] = True
    else:
        r = float(np.clip(center_roi, 0.2, 1.0))
        m[int(h*(1-r)/2):int(h*(1+r)/2), int(w*(1-r)/2):int(w*(1+r)/2)] = True

    disp = compute_disparity_adaptive(cv2.cvtColor(rect.left, cv2.COLOR_BGR2GRAY),
                                      cv2.cvtColor(rect.right, cv2.COLOR_BGR2GRAY),
                                      roi_mask=m)
    d3 = reproject(disp, rect.Q)
    pr = disp.probe or {}
    roi_ratio = disp.roi_valid_ratio if disp.roi_valid_ratio >= 0 else disp.valid_ratio
    zr = None
    finite = np.isfinite(d3.depth)
    if finite.any():
        z = d3.depth[finite]
        zr = [round(float(np.percentile(z, 5)), 2), round(float(np.percentile(z, 95)), 2)]

    stats = {
        "mod": "KALIBRELI (stereo_calib.yaml)" if metric else "kalibrasyonsuz (poz tahmini)",
        "disparite_parametre": disp.params_used,
        "gecerli_disparite_%": round(roi_ratio * 100, 1),
        "sol_sag_degisti": swapped,
    }
    if metric:
        stats["derinlik_araligi_mm"] = zr
        stats["birim"] = "mm (metrik)"
        stats["kalib_rms_px"] = round(rig.rms_px, 3)
        stats["baseline_mm"] = round(rig.baseline_mm, 1)
        stats["odak_px"] = round(rig.focal_px, 0)
    else:
        stats["rektifiye_dikey_hata_px"] = round(pr.get("dy_med", -1), 1)
        stats["bakis_acisi_farki_derece"] = round(pose.baseline_angle_deg, 1)
        stats["eslesme"] = pose.num_matches
        stats["inlier"] = pose.num_inliers
        stats["goreli_derinlik_araligi_baseline_birimi"] = zr
        stats["odak_px"] = round(intr.focal_px, 0)
        stats["odak_kaynak"] = intr.source
        stats["birim"] = "baseline birimi (olcek yok)"

    unit = "mm" if metric else "baseline birimi"
    msg = f"Stereo tamam ({stats['mod']}). Gecerli disparite %{roi_ratio*100:.1f}. "
    if metric:
        msg += f"Kalib RMS {rig.rms_px:.2f}px, baseline {rig.baseline_mm:.0f}mm. "
    msg += f"Derinlik birimi: {unit}."
    if roi_ratio < 0.15:
        msg += "  UYARI: dusuk - parlama/doku/isik kontrol."
    return QuickResult(
        ok=roi_ratio >= 0.10, message=msg,
        rect_left=rect.left, rect_right=rect.right,
        disparity=disp.disparity, disparity_color=colorize_disparity(disp.disparity),
        depth=d3.depth, depth_color=colorize_depth(d3.depth),
        points_xyz=d3.points_xyz, roi_mask=m, swapped=swapped, stats=stats,
    )
