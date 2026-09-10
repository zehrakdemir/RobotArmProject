"""
Kamera intrinsic (K) tahmini.

Iki fotograf farkli acidan cekildi ve kalibrasyon dosyasi yok. K matrisi
sunlardan tahmin edilir (oncelik sirasiyla):
  1. Kullanicinin elle girdigi piksel odak (manual_focal_px)
  2. Kullanicinin girdigi 35mm-ekivalan odak (focal_length_35mm)
  3. EXIF: FocalLengthIn35mmFilm
  4. Sezgisel:  focal_px = width_px * DEFAULT_FOCAL_RATIO

Nihai metrik olcek zaten bilinen genislikle kalibre edildigi icin K'nin
kusursuz olmasi sart degil; amac rektifikasyon geometrisini makul
tutmak.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional

import numpy as np

from . import config

logger = logging.getLogger(__name__)


@dataclass
class CameraIntrinsics:
    K: np.ndarray            # (3, 3)
    image_size: tuple[int, int]  # (w, h)
    focal_px: float
    source: str              # nereden geldigi (log/arayuz icin)

    @property
    def dist(self) -> np.ndarray:
        # Distorsiyon bilinmiyor; sifir kabul ediliyor.
        return np.zeros(5, dtype=np.float64)


def _read_exif_focal_35mm(image_path: Optional[str]) -> Optional[float]:
    if not image_path:
        return None
    try:
        from PIL import Image, ExifTags
    except ImportError:  # pragma: no cover
        return None
    try:
        img = Image.open(image_path)
        exif = img.getexif()
        if not exif:
            return None
        tagmap = {ExifTags.TAGS.get(k, k): v for k, v in exif.items()}
        val = tagmap.get("FocalLengthIn35mmFilm") or tagmap.get("FocalLenIn35mmFilm")
        if val:
            f = float(val)
            if f > 0:
                return f
    except Exception as exc:  # noqa: BLE001
        logger.debug("EXIF odak okunamadi (%s): %s", image_path, exc)
    return None


def estimate_intrinsics(
    image: np.ndarray,
    run_cfg: config.RunConfig,
    image_path: Optional[str] = None,
) -> CameraIntrinsics:
    h, w = image.shape[:2]
    cx, cy = w / 2.0, h / 2.0

    focal_px: float
    source: str

    if run_cfg.manual_focal_px and run_cfg.manual_focal_px > 0:
        focal_px = float(run_cfg.manual_focal_px)
        source = "manuel (piksel)"
    elif run_cfg.focal_length_35mm and run_cfg.focal_length_35mm > 0:
        focal_px = run_cfg.focal_length_35mm / config.SENSOR_35MM_WIDTH * w
        source = f"manuel 35mm-ekiv ({run_cfg.focal_length_35mm:.0f}mm)"
    else:
        exif_f = _read_exif_focal_35mm(image_path)
        if exif_f:
            focal_px = exif_f / config.SENSOR_35MM_WIDTH * w
            source = f"EXIF 35mm-ekiv ({exif_f:.0f}mm)"
        else:
            focal_px = w * config.DEFAULT_FOCAL_RATIO
            source = f"sezgisel (width x {config.DEFAULT_FOCAL_RATIO})"

    K = np.array(
        [[focal_px, 0.0, cx],
         [0.0, focal_px, cy],
         [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    logger.info("Intrinsic kaynagi: %s | focal_px=%.1f", source, focal_px)
    return CameraIntrinsics(K=K, image_size=(w, h), focal_px=focal_px, source=source)
