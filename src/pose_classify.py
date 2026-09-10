"""
Valiz durusu siniflandirmasi.

  * standing_on_wheels : tekerlekleri uzerinde, DIK, normal durus  -> robot MUDAHALE ETMEZ
  * tipped_over        : yatik / devrilmis                         -> robot DUZELTIR
  * uncertain          : sinir durum / veri yetersiz

Sezgi: tipik bir valizde  yukseklik > genislik > derinlik.
  - DIK (tekerlek uzeri): dikey uzanim EN BUYUK boyut, egim kucuk.
  - Yan yatmis          : dikey uzanim ORTA boyut.
  - Sirt/on uzeri yatmis: dikey uzanim EN KUCUK boyut.
"""
from __future__ import annotations

from dataclasses import dataclass

from . import config
from .measurement import MetricMeasurement


@dataclass
class PostureResult:
    label: str                     # standing_on_wheels | tipped_over | uncertain
    needs_robot: bool
    confidence: float              # 0..1
    aspect_h_over_w: float
    vertical_rank: str             # largest | middle | smallest
    reasons: list[str]

    @property
    def label_tr(self) -> str:
        return {
            "standing_on_wheels": "DIK (tekerlekleri uzerinde, normal durus)",
            "tipped_over": "YATIK / DEVRILMIS (robot duzeltmeli)",
            "uncertain": "BELIRSIZ (veri yetersiz)",
        }[self.label]


def classify_obb(obb, lean_deg: float = 0.0) -> PostureResult:
    """Hucre modu: grasp.OBB (mm) -> ayni siniflandirma.
    obb.extents = [uzun-yatay, kisa-yatay(kalinlik), dikey]."""
    return _classify_dims(
        width_cm=obb.width_mm / 10.0,
        height_cm=obb.height_mm / 10.0,
        depth_cm=obb.thickness_mm / 10.0,
        lean_deg=lean_deg,
    )


def classify(m: MetricMeasurement) -> PostureResult:
    return _classify_dims(m.width_cm, m.height_cm, m.depth_cm, m.lean_deg)


def _classify_dims(width_cm: float, height_cm: float, depth_cm: float,
                   lean_deg: float) -> PostureResult:
    class _M:
        pass
    m = _M()
    m.width_cm, m.height_cm, m.depth_cm, m.lean_deg = width_cm, height_cm, depth_cm, lean_deg
    w, h, d = m.width_cm, m.height_cm, m.depth_cm
    reasons: list[str] = []

    dims = {"genislik": w, "yukseklik": h, "derinlik": d}
    order = sorted(dims.items(), key=lambda kv: kv[1], reverse=True)
    rank_of_height = [k for k, _ in order].index("yukseklik")
    vertical_rank = ["largest", "middle", "smallest"][rank_of_height]

    aspect = h / w if w > 1e-6 else 0.0
    reasons.append(f"en={w:.1f} boy={h:.1f} derinlik={d:.1f} cm (boy/en={aspect:.2f})")
    reasons.append(f"dikey uzanim siralamasi: {vertical_rank}")
    reasons.append(f"3B egim acisi: {m.lean_deg:.1f} deg (esik {config.UPRIGHT_MAX_AXIS_DEG:.0f})")

    # --- DIK mi? ---
    upright_ok = (
        vertical_rank == "largest"
        and aspect >= config.UPRIGHT_MIN_ASPECT
        and m.lean_deg <= config.UPRIGHT_MAX_AXIS_DEG
    )
    if upright_ok:
        conf = min(1.0, 0.4 + 0.3 * (aspect - config.UPRIGHT_MIN_ASPECT)
                   + 0.3 * (1.0 - m.lean_deg / max(config.UPRIGHT_MAX_AXIS_DEG, 1e-6)))
        reasons.append("-> dikey en buyuk boyut + egim kucuk => DIK")
        return PostureResult(
            label="standing_on_wheels",
            needs_robot=False,
            confidence=round(max(0.0, min(1.0, conf)), 2),
            aspect_h_over_w=round(aspect, 2),
            vertical_rank=vertical_rank,
            reasons=reasons,
        )

    # --- YATIK mi? ---
    tipped_ok = (
        vertical_rank == "smallest"
        or (w / h if h > 1e-6 else 0.0) >= config.TIPPED_MIN_ASPECT
        or m.lean_deg >= 45.0
    )
    if tipped_ok:
        wh = (w / h) if h > 1e-6 else 0.0
        conf = 0.5
        if vertical_rank == "smallest":
            conf += 0.25
            reasons.append("-> dikey uzanim EN KUCUK => sirt/on uzeri yatmis")
        if wh >= config.TIPPED_MIN_ASPECT:
            conf += 0.15
            reasons.append(f"-> en/boy={wh:.2f} >= {config.TIPPED_MIN_ASPECT} => yatik")
        if m.lean_deg >= 45.0:
            conf += 0.1
            reasons.append("-> 3B egim >= 45 deg => devrilmis")
        return PostureResult(
            label="tipped_over",
            needs_robot=True,
            confidence=round(min(1.0, conf), 2),
            aspect_h_over_w=round(aspect, 2),
            vertical_rank=vertical_rank,
            reasons=reasons,
        )

    reasons.append("-> hicbir esik net gecilmedi => BELIRSIZ")
    return PostureResult(
        label="uncertain",
        needs_robot=False,
        confidence=0.3,
        aspect_h_over_w=round(aspect, 2),
        vertical_rank=vertical_rank,
        reasons=reasons,
    )
