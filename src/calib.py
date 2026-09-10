"""
HUCRE MODU kalibrasyonu: sabit 2 kamera (kalibreli stereo) + robot taban donusumu.

Iki telefon fotografi surumunden farki: baseline BILINIYOR, poz kestirimi
ve "bilinen genislik gir" adimi YOK. Rektifikasyon haritalari ve Q matrisi
kalibrasyon dosyalarindan BIR KEZ hesaplanir.

Dosyalar (tools/ altindaki scriptler uretir):
  stereo_calib.yaml : K1,d1,K2,d2,R,T,image_size
  base_T_cam.yaml   : 4x4  base_T_cam  (kamera cercevesi -> robot taban)
"""
from __future__ import annotations

import logging
import os
from dataclasses import dataclass
from typing import Optional

import numpy as np

from . import config

logger = logging.getLogger(__name__)


# ----------------------------------------------------------------------
def _load_yaml(path: str) -> dict:
    import cv2

    if not os.path.exists(path):
        raise FileNotFoundError(f"Kalibrasyon dosyasi yok: {path}")
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    try:
        out: dict = {}
        for key in fs.root().keys():
            node = fs.getNode(key)
            if node.isString():
                out[key] = node.string()
            elif node.isInt() or node.isReal():
                out[key] = node.real()
            elif node.isSeq():
                out[key] = [node.at(i).real() for i in range(node.size())]
            else:
                try:
                    m = node.mat()
                    out[key] = None if m is None else np.asarray(m)
                except cv2.error:
                    pass  # okunamayan dugum - atla
        return out
    finally:
        fs.release()


def calibration_status() -> dict:
    """Arayuz icin: kalibrasyon dosyalari var mi, kalite metrikleri ne."""
    st = {"stereo": {"path": config.STEREO_CALIB_PATH, "var": False},
          "base_T_cam": {"path": config.BASE_T_CAM_PATH, "var": False},
          "hazir": False}
    try:
        y = _load_yaml(config.STEREO_CALIB_PATH)
        st["stereo"].update(var=True,
                            rms_px=float(y.get("rms_reproj_err", -1)),
                            boyut=[int(y.get("image_width", 0)), int(y.get("image_height", 0))])
    except Exception:  # noqa: BLE001
        pass
    try:
        y = _load_yaml(config.BASE_T_CAM_PATH)
        st["base_T_cam"].update(var=True,
                                residual_mm=float(y.get("fit_residual_mm", -1)))
    except Exception:  # noqa: BLE001
        pass
    st["hazir"] = st["stereo"]["var"]  # base_T_cam yoksa olcum/bulut yine calisir
    st["fr3_hazir"] = st["stereo"]["var"] and st["base_T_cam"]["var"]
    return st


def save_stereo_calib(path: str, K1, d1, K2, d2, R, T, image_size,
                      rms: float = -1.0) -> None:
    import cv2

    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    fs.write("K1", np.asarray(K1, float)); fs.write("d1", np.asarray(d1, float))
    fs.write("K2", np.asarray(K2, float)); fs.write("d2", np.asarray(d2, float))
    fs.write("R", np.asarray(R, float));   fs.write("T", np.asarray(T, float))
    fs.write("image_width", int(image_size[0]))
    fs.write("image_height", int(image_size[1]))
    fs.write("rms_reproj_err", float(rms))
    fs.release()
    logger.info("Stereo kalibrasyon yazildi: %s (rms=%.3f)", path, rms)


def save_base_T_cam(path: str, T4x4: np.ndarray, residual_mm: float = -1.0) -> None:
    import cv2

    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    fs.write("base_T_cam", np.asarray(T4x4, float))
    fs.write("fit_residual_mm", float(residual_mm))
    fs.release()
    logger.info("base_T_cam yazildi: %s (residual=%.2f mm)", path, residual_mm)


# ----------------------------------------------------------------------
@dataclass
class StereoRig:
    K1: np.ndarray; d1: np.ndarray
    K2: np.ndarray; d2: np.ndarray
    R: np.ndarray;  T: np.ndarray            # cam2, cam1'e gore
    image_size: tuple[int, int]              # (w, h)
    R1: np.ndarray; R2: np.ndarray
    P1: np.ndarray; P2: np.ndarray
    Q: np.ndarray
    map1x: np.ndarray; map1y: np.ndarray
    map2x: np.ndarray; map2y: np.ndarray
    baseline_mm: float
    focal_px: float
    rms_px: float = -1.0

    # -- yukleme --
    @classmethod
    def load(cls, path: Optional[str] = None) -> "StereoRig":
        import cv2

        path = path or config.STEREO_CALIB_PATH
        y = _load_yaml(path)
        rms = float(y.get("rms_reproj_err", -1.0))
        K1, d1 = y["K1"], y["d1"].ravel()
        K2, d2 = y["K2"], y["d2"].ravel()
        R, T = y["R"], y["T"].reshape(3, 1)
        w = int(y.get("image_width", config.CAM_WIDTH))
        h = int(y.get("image_height", config.CAM_HEIGHT))

        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            K1, d1, K2, d2, (w, h), R, T,
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=0.0,
        )
        m1x, m1y = cv2.initUndistortRectifyMap(K1, d1, R1, P1, (w, h), cv2.CV_32FC1)
        m2x, m2y = cv2.initUndistortRectifyMap(K2, d2, R2, P2, (w, h), cv2.CV_32FC1)

        fx = float(P1[0, 0])
        baseline_mm = float(abs(P2[0, 3] / fx)) if fx else float(np.linalg.norm(T))
        # T zaten mm ise (kare boyutu mm girildiyse) baseline_mm ~ |T|
        logger.info("StereoRig yuklendi: %dx%d  f=%.1fpx  baseline=%.1fmm", w, h, fx, baseline_mm)
        return cls(K1, d1, K2, d2, R, T, (w, h), R1, R2, P1, P2, Q,
                   m1x, m1y, m2x, m2y, baseline_mm, fx, rms)

    # -- kullanim --
    def rectify(self, img_left: np.ndarray, img_right: np.ndarray):
        import cv2

        l = cv2.remap(img_left, self.map1x, self.map1y, cv2.INTER_LINEAR)
        r = cv2.remap(img_right, self.map2x, self.map2y, cv2.INTER_LINEAR)
        return l, r

    def remap_mask(self, mask: np.ndarray, which: str = "left") -> np.ndarray:
        import cv2

        mx, my = (self.map1x, self.map1y) if which == "left" else (self.map2x, self.map2y)
        return cv2.remap(mask.astype(np.uint8) * 255, mx, my,
                         interpolation=cv2.INTER_NEAREST) > 127

    def reproject(self, disparity_px: np.ndarray) -> np.ndarray:
        """(H,W) disparite -> (H,W,3) 3B nokta (rektifiye sol kamera cercevesi, mm)."""
        import cv2

        disp = np.where(np.isfinite(disparity_px), disparity_px, 0.0).astype(np.float32)
        pts = cv2.reprojectImageTo3D(disp, self.Q.astype(np.float64))
        # Q, T'nin biriminde 3B verir. T mm ise mm; degilse baseline_mm ile olcekle.
        return pts


# ----------------------------------------------------------------------
@dataclass
class BaseTransform:
    """Kamera (rektifiye sol) cercevesinden robot taban cercevesine 4x4."""

    T: np.ndarray                     # (4,4)  base_T_cam
    residual_mm: float = -1.0

    @classmethod
    def load(cls, path: Optional[str] = None) -> "BaseTransform":
        path = path or config.BASE_T_CAM_PATH
        y = _load_yaml(path)
        T = np.asarray(y["base_T_cam"], float).reshape(4, 4)
        res = float(y.get("fit_residual_mm", -1.0)) if "fit_residual_mm" in y else -1.0
        return cls(T=T, residual_mm=res)

    @classmethod
    def identity(cls) -> "BaseTransform":
        return cls(T=np.eye(4), residual_mm=-1.0)

    def apply(self, pts_mm: np.ndarray) -> np.ndarray:
        """pts_mm: (...,3) kamera cercevesi -> (...,3) robot taban cercevesi."""
        p = np.asarray(pts_mm, float)
        shp = p.shape
        flat = p.reshape(-1, 3)
        h = np.c_[flat, np.ones(len(flat))]
        out = (self.T @ h.T).T[:, :3]
        return out.reshape(shp)

    def apply_vector(self, vec: np.ndarray) -> np.ndarray:
        """Yon vektoru (oteleme yok)."""
        v = np.asarray(vec, float).reshape(3)
        return (self.T[:3, :3] @ v)
