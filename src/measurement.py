"""
Valiz olcumu - MEDYAN tabanli en / boy / derinlik + yamukluk + tepe (x,y).

Kullanicinin istedigi yontem:
  * "valiz olan kismin en boy yuksekligini MEDYANA gore bul"
  * "x y koordinatlarini valizin ustune yerlestirerek yamuklugu bul"

Olcum iki kaynagi birlestirir:
  1. 2B maske -> cv2.minAreaRect  -> yonelimli dikdortgen, 4 kose, aci
     (yamukluk = acinin 0/90'dan sapmasi).
  2. Piksel-basi 3B nokta haritasi (baseline birimi) -> her satir/sutun
     icin kenar noktalari arasi 3B mesafe -> MEDYAN.

Tum ham olculer baseline birimindedir; `RawMeasurement.to_metric(scale)`
ile santimetreye cevrilir (scale, src/scale.py'den).
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from . import config

logger = logging.getLogger(__name__)


# ----------------------------------------------------------------------
@dataclass
class OrientedRect:
    center_px: tuple[float, float]
    size_px: tuple[float, float]        # (w, h) minAreaRect ham
    angle_deg: float                    # minAreaRect ham aci
    corners_px: np.ndarray              # (4, 2) - sirali (TL, TR, BR, BL yaklasik)
    tilt_deg: float                     # yamukluk: [-45, 45] normalize
    is_skewed: bool                     # |tilt| > TILT_THRESHOLD_DEG


@dataclass
class TopReference:
    """Valizin tepesine yerlestirilen referans (kavrama) noktasi."""

    pixel: tuple[int, int]              # (u, v)
    xyz_unit: Optional[np.ndarray]      # (3,) baseline birimi - 3B varsa
    valid: bool


@dataclass
class RawMeasurement:
    width_u: float                     # en  (baseline birimi)
    height_u: float                    # boy
    depth_u: float                     # derinlik (uzunluk)
    width_samples: int
    height_samples: int
    rect: OrientedRect
    top_ref: TopReference
    lean_deg: float                    # 3B: dikey eksenin dunya-dikeyinden sapmasi
    n_points: int
    method_notes: list[str] = field(default_factory=list)

    def to_metric(self, scale: float) -> "MetricMeasurement":
        return MetricMeasurement(
            width_cm=self.width_u * scale,
            height_cm=self.height_u * scale,
            depth_cm=self.depth_u * scale,
            scale=scale,
            rect=self.rect,
            top_ref=self.top_ref,
            lean_deg=self.lean_deg,
            n_points=self.n_points,
            method_notes=list(self.method_notes),
        )


@dataclass
class MetricMeasurement:
    width_cm: float
    height_cm: float
    depth_cm: float
    scale: float                       # baseline birimi -> cm carpani
    rect: OrientedRect
    top_ref: TopReference
    lean_deg: float
    n_points: int
    method_notes: list[str] = field(default_factory=list)

    @property
    def sorted_dims_cm(self) -> tuple[float, float, float]:
        return tuple(sorted([self.width_cm, self.height_cm, self.depth_cm], reverse=True))

    def as_dict(self) -> dict:
        return {
            "en_cm": round(self.width_cm, 1),
            "boy_cm": round(self.height_cm, 1),
            "derinlik_cm": round(self.depth_cm, 1),
            "yamukluk_derece": round(self.rect.tilt_deg, 1),
            "yamuk_mu": self.rect.is_skewed,
            "egim_3d_derece": round(self.lean_deg, 1),
            "olcek_cm_per_unit": round(self.scale, 4),
            "tepe_pikseli": self.top_ref.pixel,
            "nokta_sayisi": self.n_points,
        }


# ----------------------------------------------------------------------
def _order_corners(pts: np.ndarray) -> np.ndarray:
    """4 koseyi TL, TR, BR, BL sirasina sok."""
    pts = pts.astype(np.float64)
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1).ravel()
    tl = pts[np.argmin(s)]
    br = pts[np.argmax(s)]
    tr = pts[np.argmin(d)]
    bl = pts[np.argmax(d)]
    return np.array([tl, tr, br, bl])


def _oriented_rect(mask: np.ndarray) -> OrientedRect:
    import cv2

    m = (mask.astype(np.uint8) * 255)
    contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise ValueError("Maskede kontur yok - valiz bolgesi bos.")
    cnt = max(contours, key=cv2.contourArea)
    (cx, cy), (rw, rh), ang = cv2.minAreaRect(cnt)
    box = cv2.boxPoints(((cx, cy), (rw, rh), ang))
    corners = _order_corners(box)

    # Yamukluk: minAreaRect acisini [-45, 45]'e normalize et.
    tilt = ang
    if tilt < -45:
        tilt += 90
    elif tilt > 45:
        tilt -= 90
    # Uzun kenar dikeye yakinsa (valiz ayakta), acinin 90'a gore sapmasini al.
    if rh < rw:
        # OpenCV konvansiyonu: genislik uzun kenar olabilir; yamuklugu
        # her iki eksene gore de degerlendirip kucuk olani sec.
        alt = tilt + 90 if tilt < 0 else tilt - 90
        if abs(alt) < abs(tilt):
            tilt = alt

    return OrientedRect(
        center_px=(float(cx), float(cy)),
        size_px=(float(rw), float(rh)),
        angle_deg=float(ang),
        corners_px=corners,
        tilt_deg=float(tilt),
        is_skewed=abs(tilt) > config.TILT_THRESHOLD_DEG,
    )


def _top_reference(mask: np.ndarray, points_xyz: Optional[np.ndarray]) -> TopReference:
    ys, xs = np.where(mask)
    if ys.size == 0:
        return TopReference(pixel=(0, 0), xyz_unit=None, valid=False)
    y_top = ys.min()
    band = ys <= y_top + max(3, int(0.15 * (ys.max() - y_top + 1)))
    u = int(round(xs[band].mean()))
    v = int(round(ys[band].mean()))

    xyz = None
    valid = False
    if points_xyz is not None:
        # Tepe bandindaki gecerli 3B noktalarin medyani (gurultuye dayanikli)
        region = points_xyz[ys[band], xs[band]]
        finite = np.isfinite(region).all(axis=1)
        if finite.sum() >= 5:
            xyz = np.median(region[finite], axis=0)
            valid = True
    return TopReference(pixel=(u, v), xyz_unit=xyz, valid=valid)


def _median_span(points_xyz: np.ndarray, mask: np.ndarray, axis: str) -> tuple[float, int]:
    """
    Her satir (axis='row' -> EN) veya her sutun (axis='col' -> BOY) icin
    o hat uzerindeki en uc iki gecerli 3B nokta arasindaki Oklid mesafesini
    hesaplar; tum hatlarin MEDYANINI dondurur.
    """
    H, W = mask.shape
    finite = np.isfinite(points_xyz).all(axis=2) & mask
    spans: list[float] = []

    if axis == "row":
        lines = range(H)
        for y in lines:
            cols = np.where(finite[y])[0]
            if cols.size < 2:
                continue
            p_lo = points_xyz[y, cols.min()]
            p_hi = points_xyz[y, cols.max()]
            spans.append(float(np.linalg.norm(p_hi - p_lo)))
    else:
        for x in range(W):
            rows = np.where(finite[:, x])[0]
            if rows.size < 2:
                continue
            p_lo = points_xyz[rows.min(), x]
            p_hi = points_xyz[rows.max(), x]
            spans.append(float(np.linalg.norm(p_hi - p_lo)))

    if not spans:
        return 0.0, 0
    arr = np.asarray(spans)
    lo, hi = np.percentile(arr, [config.DIM_LOW_PERCENTILE, config.DIM_HIGH_PERCENTILE])
    core = arr[(arr >= lo) & (arr <= hi)]
    if core.size == 0:
        core = arr
    return float(np.median(core)), int(core.size)


def _depth_span(points_xyz: np.ndarray, mask: np.ndarray) -> float:
    """Derinlik (uzunluk): en yakin %P ile en uzak %P medyanlarinin farki."""
    z = points_xyz[:, :, 2]
    sel = mask & np.isfinite(z)
    vals = z[sel]
    if vals.size < 20:
        return 0.0
    p = config.DEPTH_EDGE_PERCENTILE
    near = np.median(vals[vals <= np.percentile(vals, p)])
    far = np.median(vals[vals >= np.percentile(vals, 100 - p)])
    return float(abs(far - near))


def _lean_angle(points_xyz: np.ndarray, mask: np.ndarray) -> float:
    """
    Valizin dikey ana ekseni ile dunya-dikeyi arasindaki aci (derece).
    Rektifiye kamera cercevesinde dunya-dikeyi ~ (0, -1, 0) (goruntu y
    asagi artar). PCA ile en buyuk uzanimi bul.
    """
    sel = mask & np.isfinite(points_xyz).all(axis=2)
    pts = points_xyz[sel]
    if pts.shape[0] < 30:
        return 0.0
    c = pts - pts.mean(axis=0)
    cov = np.cov(c.T)
    w, v = np.linalg.eigh(cov)
    principal = v[:, np.argmax(w)]          # en buyuk varyans yonu
    principal = principal / (np.linalg.norm(principal) + 1e-9)
    up = np.array([0.0, -1.0, 0.0])
    cos_a = abs(float(np.dot(principal, up)))
    cos_a = min(1.0, max(0.0, cos_a))
    return float(np.degrees(np.arccos(cos_a)))


# ----------------------------------------------------------------------
def measure(
    points_xyz: np.ndarray,
    mask_rect: np.ndarray,
    focal_px: float,
) -> RawMeasurement:
    """
    points_xyz : (H, W, 3) baseline birimi (rektifiye sol cerceve)
    mask_rect  : (H, W) bool - rektifiye edilmis valiz maskesi
    """
    notes: list[str] = []
    rect = _oriented_rect(mask_rect)
    top_ref = _top_reference(mask_rect, points_xyz)

    width_u, wn = _median_span(points_xyz, mask_rect, "row")
    height_u, hn = _median_span(points_xyz, mask_rect, "col")
    depth_u = _depth_span(points_xyz, mask_rect)
    lean = _lean_angle(points_xyz, mask_rect)

    n_points = int((mask_rect & np.isfinite(points_xyz).all(axis=2)).sum())

    # 3B cok seyrekse pikselden odak-tabanli yaklasik olcu (yedek)
    if width_u == 0.0 or height_u == 0.0:
        z = points_xyz[:, :, 2]
        zmed = np.nanmedian(z[mask_rect & np.isfinite(z)]) if n_points else np.nan
        if np.isfinite(zmed) and focal_px > 0:
            rw, rh = rect.size_px
            if width_u == 0.0:
                width_u = rw * zmed / focal_px
                notes.append("EN: 3B seyrek, pikselden yaklasik hesaplandi.")
            if height_u == 0.0:
                height_u = rh * zmed / focal_px
                notes.append("BOY: 3B seyrek, pikselden yaklasik hesaplandi.")

    return RawMeasurement(
        width_u=width_u,
        height_u=height_u,
        depth_u=depth_u,
        width_samples=wn,
        height_samples=hn,
        rect=rect,
        top_ref=top_ref,
        lean_deg=lean,
        n_points=n_points,
        method_notes=notes,
    )
