"""
YOLO26-seg ile valiz (suitcase) tespiti + segmentasyon maskesi.

Akis:
  1. Modeli yukle (SEG_WEIGHTS; yoksa SEG_WEIGHTS_FALLBACK).
  2. Goruntude segmentasyon calistir.
  3. Sadece 'suitcase' sinifini (COCO id 28) filtrele.
  4. En yuksek guvenli valizi sec -> ikili maske + bbox + skor dondur.

Ultralytics API'si model mimarisinden bagimsizdir: yolov8-seg, yolo11-seg
ve yolo26-seg ayni `YOLO(weights).predict(...)` arayuzunu kullanir, bu
yuzden agirlik dosyasini degistirmek yeterli.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional

import numpy as np

from . import config

logger = logging.getLogger(__name__)


@dataclass
class Detection:
    """Tek bir valiz tespiti."""

    mask: np.ndarray            # (H, W) bool - tam goruntu cozunurlugunde
    bbox_xyxy: np.ndarray       # (4,) float - x1, y1, x2, y2 (piksel)
    score: float
    class_id: int
    class_name: str
    weights_used: str

    @property
    def area_px(self) -> int:
        return int(self.mask.sum())


class SuitcaseDetector:
    """YOLO26-seg tabanli valiz dedektoru (tek seferlik model yukleme)."""

    def __init__(
        self,
        weights: Optional[str] = None,
        fallback: Optional[str] = None,
        conf: Optional[float] = None,
        iou: Optional[float] = None,
        device: Optional[str] = None,
        accept_extra: Optional[set] = None,
    ) -> None:
        self.weights = weights or config.SEG_WEIGHTS
        self.fallback = fallback or config.SEG_WEIGHTS_FALLBACK
        self.conf = config.YOLO_CONF_THRESHOLD if conf is None else conf
        self.iou = config.YOLO_IOU_THRESHOLD if iou is None else iou
        self.device = config.YOLO_DEVICE if device is None else device
        # test amacli EK kabul edilen sinif adlari (config.ACCEPT_CLASS_NAMES'e eklenir)
        self.accept_extra = {str(s).strip().lower() for s in (accept_extra or set()) if str(s).strip()}
        self._model = None
        self._weights_used: str = ""

    # ------------------------------------------------------------------
    def _load(self):
        if self._model is not None:
            return self._model
        try:
            from ultralytics import YOLO  # gec import - agir bagimlilik
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError(
                "ultralytics kurulu degil. `pip install ultralytics` calistirin."
            ) from exc

        candidates = [self.weights]
        if self.fallback and self.fallback != self.weights:
            candidates.append(self.fallback)

        last_err: Optional[Exception] = None
        for cand in candidates:
            try:
                logger.info("YOLO agirligi yukleniyor: %s", cand)
                model = YOLO(cand)
                self._model = model
                self._weights_used = cand
                if cand != self.weights:
                    logger.warning(
                        "Asil agirlik '%s' bulunamadi; fallback '%s' kullaniliyor.",
                        self.weights, cand,
                    )
                return model
            except Exception as exc:  # noqa: BLE001 - hangi agirlik olursa dene
                logger.warning("Agirlik yuklenemedi (%s): %s", cand, exc)
                last_err = exc

        raise RuntimeError(
            f"Hicbir segmentasyon agirligi yuklenemedi ({candidates}). "
            f"Son hata: {last_err}"
        )

    # ------------------------------------------------------------------
    @property
    def weights_used(self) -> str:
        return self._weights_used

    def detect(self, image_bgr: np.ndarray) -> Optional[Detection]:
        """En guvenli valiz tespitini dondurur; valiz yoksa None."""
        detections = self.detect_all(image_bgr)
        if not detections:
            return None
        return max(detections, key=lambda d: d.score)

    def detect_all(self, image_bgr: np.ndarray) -> list[Detection]:
        model = self._load()
        h, w = image_bgr.shape[:2]

        predict_kwargs = dict(
            source=image_bgr,
            conf=self.conf,
            iou=self.iou,
            verbose=False,
            retina_masks=True,
        )
        if self.device:
            predict_kwargs["device"] = self.device

        results = model.predict(**predict_kwargs)
        if not results:
            return []
        res = results[0]

        names = getattr(res, "names", {}) or getattr(model, "names", {})
        if res.masks is None or res.boxes is None or len(res.boxes) == 0:
            return []

        masks = res.masks.data.cpu().numpy()          # (N, mh, mw) float 0/1
        boxes = res.boxes.xyxy.cpu().numpy()          # (N, 4)
        cls = res.boxes.cls.cpu().numpy().astype(int)  # (N,)
        scores = res.boxes.conf.cpu().numpy()         # (N,)

        out: list[Detection] = []
        for i in range(len(boxes)):
            class_id = int(cls[i])
            class_name = str(names.get(class_id, class_id))
            if not self._is_suitcase(class_id, class_name):
                continue

            m = masks[i]
            if m.shape != (h, w):
                m = _resize_mask(m, (h, w))
            mask_bool = m > 0.5

            out.append(
                Detection(
                    mask=mask_bool,
                    bbox_xyxy=boxes[i].astype(float),
                    score=float(scores[i]),
                    class_id=class_id,
                    class_name=class_name,
                    weights_used=self._weights_used,
                )
            )
        return out

    # ------------------------------------------------------------------
    def _is_suitcase(self, class_id: int, class_name: str) -> bool:
        if class_id == config.COCO_SUITCASE_CLASS_ID:
            return True
        name = class_name.strip().lower()
        if name in {"suitcase", "valiz", "luggage", "baggage"}:
            return True
        # Test amacli ek sinif adlari: .env ACCEPT_CLASS_NAMES + bu ornege ozel
        # (Hucre Modu 'diger nesneler' onayi -> accept_extra).
        return name in config.ACCEPT_CLASS_NAMES or name in self.accept_extra


# ----------------------------------------------------------------------
def _resize_mask(mask: np.ndarray, hw: tuple[int, int]) -> np.ndarray:
    """cv2 olmadan da calisan basit nearest-neighbor yeniden boyutlandirma."""
    try:
        import cv2

        return cv2.resize(
            mask.astype(np.float32), (hw[1], hw[0]), interpolation=cv2.INTER_NEAREST
        )
    except Exception:  # pragma: no cover
        src_h, src_w = mask.shape
        ys = (np.linspace(0, src_h - 1, hw[0])).astype(int)
        xs = (np.linspace(0, src_w - 1, hw[1])).astype(int)
        return mask[np.ix_(ys, xs)]


def dilate_mask(mask: np.ndarray, px: int) -> np.ndarray:
    """Maskeyi `px` kadar genislet (ozellik eslestirme bolgesi icin)."""
    if px <= 0:
        return mask.astype(bool)
    try:
        import cv2

        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * px + 1, 2 * px + 1))
        return cv2.dilate(mask.astype(np.uint8), k).astype(bool)
    except Exception:  # pragma: no cover
        from scipy.ndimage import binary_dilation

        return binary_dilation(mask, iterations=px)
