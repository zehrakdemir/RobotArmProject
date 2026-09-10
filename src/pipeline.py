"""
Uctan uca akis: 2 fotograf -> tespit -> bagil poz -> rektifikasyon ->
SGBM+WLS disparite -> 3B reprojeksiyon -> olcek kalibrasyonu -> olcum ->
nokta bulutu -> durus sinifi -> FR3 komut plani.

image_to_robot_frame() disinda her adim tam implemente edilmistir.
"""
from __future__ import annotations

import json
import logging
import os
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from . import config
from .camera import CameraIntrinsics, estimate_intrinsics
from .depth import DepthResult, colorize_depth, reproject
from .detector import Detection, SuitcaseDetector
from .measurement import MetricMeasurement, RawMeasurement, measure
from .pointcloud import PointCloud, extract, plotly_figure
from .pose import RelativePose, estimate_relative_pose
from .pose_classify import PostureResult, classify
from .rectify import RectifiedPair, rectify_pair
from .robot import FR3Command, plan_correction
from .scale import ScaleError, compute_scale
from .stereo import DisparityResult, colorize_disparity, compute_disparity_adaptive

logger = logging.getLogger(__name__)


# ----------------------------------------------------------------------
@dataclass
class PipelineResult:
    is_suitcase: bool
    message: str

    detection_left: Optional[Detection] = None
    detection_right: Optional[Detection] = None
    intrinsics: Optional[CameraIntrinsics] = None
    pose: Optional[RelativePose] = None
    rectified: Optional[RectifiedPair] = None
    disparity: Optional[DisparityResult] = None
    depth: Optional[DepthResult] = None          # metrik (cm)
    scale_cm_per_unit: float = 0.0
    raw_measurement: Optional[RawMeasurement] = None
    measurement: Optional[MetricMeasurement] = None
    pointcloud: Optional[PointCloud] = None
    posture: Optional[PostureResult] = None
    robot_command: Optional[FR3Command] = None

    disparity_color: Optional[np.ndarray] = None
    depth_color: Optional[np.ndarray] = None
    warnings: list[str] = field(default_factory=list)

    # -- teshis (olcum basarisiz oldugunda arayuzde gosterilir) --
    ok: bool = True                       # tam sonuc uretildi mi
    stage_failed: str = ""               # "" | "scale" | ...
    diagnostics: dict = field(default_factory=dict)

    # -- arayuz kolayligi --
    def summary(self) -> dict:
        out = {"valiz_mi": self.is_suitcase, "mesaj": self.message, "tam_sonuc": self.ok}
        if self.diagnostics:
            out["teshis"] = self.diagnostics
        if not self.ok:
            out["basarisiz_asama"] = self.stage_failed
        if self.measurement:
            out["olcum"] = self.measurement.as_dict()
        if self.posture:
            out["durus"] = {
                "etiket": self.posture.label,
                "aciklama": self.posture.label_tr,
                "robot_gerekli": self.posture.needs_robot,
                "guven": self.posture.confidence,
            }
        if self.robot_command:
            out["robot_komut"] = self.robot_command.to_dict()
        if self.pose:
            out["bakis_acisi_farki_derece"] = round(self.pose.baseline_angle_deg, 1)
        if self.warnings:
            out["uyarilar"] = self.warnings
        return out

    def pointcloud_figure(self):
        if self.pointcloud is None or self.measurement is None:
            return None
        m = self.measurement
        xyz = self.pointcloud.xyz
        center = tuple(np.median(xyz, axis=0)) if len(self.pointcloud) else (0, 0, 0)
        bbox = {"center": center, "width": m.width_cm, "height": m.height_cm, "depth": m.depth_cm}
        return plotly_figure(self.pointcloud, bbox_cm=bbox)


# ----------------------------------------------------------------------
_DETECTOR_SINGLETON: Optional[SuitcaseDetector] = None


def get_detector() -> SuitcaseDetector:
    global _DETECTOR_SINGLETON
    if _DETECTOR_SINGLETON is None:
        _DETECTOR_SINGLETON = SuitcaseDetector()
    return _DETECTOR_SINGLETON


def _to_gray(img: np.ndarray) -> np.ndarray:
    import cv2

    if img.ndim == 2:
        return img
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


# ----------------------------------------------------------------------
def run_pipeline(
    image_left: np.ndarray,
    image_right: np.ndarray,
    run_cfg: config.RunConfig,
    left_path: Optional[str] = None,
    right_path: Optional[str] = None,
    detector: Optional[SuitcaseDetector] = None,
) -> PipelineResult:
    """image_left / image_right : BGR uint8 numpy dizileri (cv2.imread formati)."""
    if run_cfg.swap_images:
        image_left, image_right = image_right, image_left
        left_path, right_path = right_path, left_path

    det = detector or get_detector()

    # --- 1) TESPIT: once valiz mi? ---
    d_left = det.detect(image_left)
    d_right = det.detect(image_right)

    if d_left is None and d_right is None:
        return PipelineResult(
            is_suitcase=False,
            message="Iki fotografta da valiz tespit edilmedi. (YOLO26-seg 'suitcase')",
        )
    if d_left is None or d_right is None:
        which = "sol" if d_left is None else "sag"
        return PipelineResult(
            is_suitcase=True,
            message=f"Valiz sadece bir fotografta bulundu ({which} fotografta yok). "
                    "Stereo olcum icin iki fotografta da valiz gorunmeli.",
            detection_left=d_left,
            detection_right=d_right,
        )

    res = PipelineResult(is_suitcase=True, message="Valiz tespit edildi.",
                         detection_left=d_left, detection_right=d_right)

    # --- 2) INTRINSIC ---
    intr = estimate_intrinsics(image_left, run_cfg, left_path)
    res.intrinsics = intr

    # --- 3) BAGIL POZ (bakis acilari arasi fark + piksel kaymasi) ---
    gray_l, gray_r = _to_gray(image_left), _to_gray(image_right)
    pose = estimate_relative_pose(gray_l, gray_r, intr, d_left.mask, d_right.mask)

    # Sol/sag SIRA kontrolu: statik noktalarin ortanca yatay kaymasi negatifse
    # ikinci foto sola kaymis demektir -> disparite negatif olur, SGBM bulamaz.
    # Goruntuleri degistirip pozu yeniden kestir (pozitif disparite icin).
    med_dx = float(np.median(pose.pts1[:, 0] - pose.pts2[:, 0])) if len(pose.pts1) else 0.0
    if med_dx < -1.0:
        image_left, image_right = image_right, image_left
        d_left, d_right = d_right, d_left
        gray_l, gray_r = gray_r, gray_l
        pose = estimate_relative_pose(gray_l, gray_r, intr, d_left.mask, d_right.mask)
        res.warnings.append(
            "Sol/sag otomatik degistirildi (baseline yonu; disparite negatiften pozitife)."
        )
    res.pose = pose

    # --- 4) REKTIFIKASYON ---
    rect = rectify_pair(image_left, image_right, intr, pose)
    res.rectified = rect
    mask_l_rect = rect.remap_mask(d_left.mask, "left")

    mask_px = int(mask_l_rect.sum())
    if mask_px < 200:
        res.warnings.append("Rektifiye maske cok kucuk; ROI/poz kalitesi dusuk olabilir.")

    # --- 5) DISPARITE (SGBM + WLS, adaptif) ---
    gray_lr = _to_gray(rect.left)
    gray_rr = _to_gray(rect.right)
    disp = compute_disparity_adaptive(gray_lr, gray_rr, roi_mask=mask_l_rect)
    res.disparity = disp
    res.disparity_color = colorize_disparity(disp.disparity)
    roi_ratio = disp.roi_valid_ratio if disp.roi_valid_ratio >= 0 else disp.valid_ratio
    if roi_ratio < 0.10:
        res.warnings.append(
            f"Valiz uzerinde gecerli disparite cok dusuk (%{roi_ratio*100:.1f}). "
            "Parlak/dokusuz yuzey + genis bakis acisi farki. "
            "Ipucu: iki cekim arasi aciyi azalt (~15°), valizi hic oynatma, "
            "kadrajda daha cok halı/zemin dokusu birak."
        )

    pr = disp.probe or {}
    res.diagnostics = {
        "bakis_acisi_farki_derece": round(pose.baseline_angle_deg, 1),
        "eslesme": pose.num_matches,
        "inlier": pose.num_inliers,
        "rektifiye_maske_piksel": mask_px,
        "disparite_gecerli_kare_%": round(disp.valid_ratio * 100, 1),
        "disparite_gecerli_ROI_%": round(roi_ratio * 100, 1),
        "disparite_parametre": disp.params_used,
        "rektifiye_dikey_hata_px": round(pr.get("dy_med", -1), 1),
        "probe_yatay_eslesme": pr.get("n_horiz", 0),
        "probe_disparite_araligi": [round(pr.get("dx_lo", 0)), round(pr.get("dx_hi", 0))],
        "intrinsic_kaynak": intr.source,
    }
    if pr and pr.get("dy_med", 0) > 3.0 and pr.get("n", 0) >= 12:
        res.warnings.append(
            f"REKTIFIKASYON BOZUK: rektifiye ciftte dikey hata ~{pr['dy_med']:.1f}px "
            "(olmasi gereken <1px). Poz kestirimi zayif - iki foto arasi aci/kayma "
            "cok buyuk ya da sahne duzlemsel/az dokulu. Daha kucuk bir bakis acisi "
            "farkiyla yeniden cekin."
        )

    # --- 6) 3B REPROJEKSIYON (baseline birimi) ---
    depth_unit = reproject(disp, rect.Q)

    # --- 7) HAM OLCUM (baseline birimi) ---
    raw = measure(depth_unit.points_xyz, mask_l_rect, intr.focal_px)
    res.raw_measurement = raw

    # --- 8) OLCEK KALIBRASYONU (bilinen gercek olcu) ---
    try:
        scale = compute_scale(raw, run_cfg)
    except ScaleError as exc:
        res.ok = False
        res.stage_failed = "scale"
        res.message = (
            "Olcum yapilamadi: yogun stereo, valiz yuzeyinde yeterli "
            f"3B nokta uretemedi. ({exc})"
        )
        res.warnings.append(
            "Bu iki fotograf bu yontem icin fazla farkli aciden veya yuzey "
            "fazla dokusuz. Teshis sekmesine bak; .env ile "
            "WLS_CONFIDENCE_THRESHOLD dusur / SGBM_NUM_DISPARITIES artir."
        )
        return res
    res.scale_cm_per_unit = scale

    # --- 9) METRIK OLCUM + METRIK DERINLIK ---
    metric = raw.to_metric(scale)
    res.measurement = metric
    res.depth = depth_unit.scaled(scale)
    res.depth_color = colorize_depth(res.depth.depth)

    # --- 10) NOKTA BULUTU (cm) ---
    pc = extract(res.depth.points_xyz, mask_l_rect, color_bgr=rect.left, only_suitcase=True)
    res.pointcloud = pc

    # --- 11) DURUŞ SINIFI (dik / yatik) ---
    posture = classify(metric)
    res.posture = posture

    # --- 12) FR3 KOMUT PLANI ---
    grasp_robot = None
    place_robot = None
    try:
        if raw.top_ref.valid and raw.top_ref.xyz_unit is not None:
            grasp_cam_cm = raw.top_ref.xyz_unit * scale
            grasp_robot = image_to_robot_frame(grasp_cam_cm)
    except NotImplementedError as exc:
        res.warnings.append(f"El-goz kalibrasyonu yok: {exc}. FR3 komutu kamera "
                            "cercevesinde/plansiz uretildi.")
    except Exception as exc:  # noqa: BLE001
        res.warnings.append(f"Kavrama noktasi robot cercevesine cevrilemedi: {exc}")

    res.robot_command = plan_correction(
        posture_label=posture.label,
        grasp_xyz_robot=grasp_robot,
        dims_cm=(metric.width_cm, metric.height_cm, metric.depth_cm),
        tilt_deg=metric.rect.tilt_deg,
        lean_deg=metric.lean_deg,
        place_xyz_robot=place_robot,
    )

    res.message = (
        f"Valiz olculdu: en {metric.width_cm:.1f} / boy {metric.height_cm:.1f} / "
        f"derinlik {metric.depth_cm:.1f} cm | durus: {posture.label_tr}"
    )
    return res


# ----------------------------------------------------------------------
def image_to_robot_frame(xyz_cam_cm: np.ndarray) -> np.ndarray:
    """
    Kamera cercevesindeki bir 3B noktayi (cm) FAIRINO FR3 TABAN cercevesine
    (mm) cevirir.

    Bu, EL-GOZ (hand-eye) KALIBRASYONUNA baglidir ve tamamen sizin
    kamera-robot montajiniza ozeldir. Varsayim yapmiyoruz.

    Beklenen kalibrasyon dosyasi:  config.HANDEYE_PATH  (JSON)
        {
          "R": [[..],[..],[..]],     # 3x3 rotasyon (kamera->taban)
          "t_mm": [tx, ty, tz],      # oteleme, mm
          "camera_units": "cm"       # giris birimi (cm|mm|m)
        }
    Ornek sablon: src/calibration/handeye.example.json

    Dosya yoksa NotImplementedError firlatilir; pipeline bu durumda
    FR3 komutunu yine uretir ama kavrama pozunu bos birakir (operator
    manuel jog ile tamamlar).
    """
    path = config.HANDEYE_PATH
    if not os.path.exists(path):
        raise NotImplementedError(
            f"Hand-eye kalibrasyon dosyasi bulunamadi: {path}. "
            "src/calibration/handeye.example.json dosyasini kopyalayip "
            "kendi R / t_mm degerlerinizle doldurun."
        )

    with open(path, "r", encoding="utf-8") as fh:
        data = json.load(fh)

    R = np.asarray(data["R"], dtype=np.float64).reshape(3, 3)
    t_mm = np.asarray(data["t_mm"], dtype=np.float64).reshape(3)
    unit = str(data.get("camera_units", "cm")).lower()
    factor = {"cm": 10.0, "mm": 1.0, "m": 1000.0}.get(unit, 10.0)

    p_cam_mm = np.asarray(xyz_cam_cm, dtype=np.float64).reshape(3) * factor
    p_base_mm = R @ p_cam_mm + t_mm
    return p_base_mm
