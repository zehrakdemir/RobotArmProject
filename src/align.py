"""
Yan yatik valizi DIK konuma getirme (non-prehensile tip-up).

Senaryo: valizin buyuk kabuk yuzlerinden biri yerde. Robot, valizin
robota bakan KISA DIKEY yuzune (kalinlik yuzu) ust kismindan yatay iter;
valiz karsi alt kenari etrafinda donerek o kenar uzerinde dik kalir.
Sonra tekrar goruntu alinip DIK akisina gecilir.

DIKKAT: non-prehensile, surtunmeye/kaymaya duyarli. Varsayilan DRY-RUN.
Cikan plani gercek kolda dusuk hizda, adim adim dogrulayin ve
TIPUP_PUSH_DEPTH_MM'i deneyerek ayarlayin.

Sadece numpy.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional

import numpy as np

from . import config
from .grasp import OBB, fit_obb

logger = logging.getLogger(__name__)


@dataclass
class TipUpPlan:
    feasible: bool
    reason: str
    pre_contact: Optional[np.ndarray] = None    # (3) temas oncesi (geri + yukari)
    contact_point: Optional[np.ndarray] = None  # (3) itme temas noktasi
    push_dir: Optional[np.ndarray] = None       # (3) birim, yatay itme yonu
    push_distance_mm: float = 0.0
    retract_point: Optional[np.ndarray] = None  # (3) itme sonrasi geri cekilme
    pivot_edge_mid: Optional[np.ndarray] = None # (3) donme kenari ortasi (bilgi)
    expected_upright_height_mm: float = 0.0
    rpy_deg: tuple[float, float, float] = (180.0, 0.0, 0.0)
    notes: list[str] = None

    def as_dict(self) -> dict:
        def _l(v): return None if v is None else [round(float(x), 1) for x in v]
        return {
            "uygulanabilir": self.feasible,
            "sebep": self.reason,
            "temas_oncesi_mm": _l(self.pre_contact),
            "temas_noktasi_mm": _l(self.contact_point),
            "itme_yonu": _l(self.push_dir),
            "itme_mesafesi_mm": round(self.push_distance_mm, 1),
            "geri_cekilme_mm": _l(self.retract_point),
            "donme_kenari_ortasi_mm": _l(self.pivot_edge_mid),
            "beklenen_dik_yukseklik_mm": round(self.expected_upright_height_mm, 1),
            "rpy_derece": [round(v, 1) for v in self.rpy_deg],
            "notlar": self.notes or [],
        }


def plan_tipup(points_mm: np.ndarray,
               base_origin_xy: tuple[float, float] = (0.0, 0.0),
               obb: Optional[OBB] = None, force: bool = False) -> TipUpPlan:
    """
    points_mm : yan yatik valiz noktalari, robot taban cercevesi (mm, Z yukari).
    base_origin_xy : robot tabaninin XY'si (itme yonunu robottan UZAGA secmek icin).
    force : True ise "yan yatik gorunmuyor" saglik kontrolunu atla (operator elle
        YATIK dedi; gurultulu OBB dikey uzanimina guvenme).
    """
    p = np.asarray(points_mm, float)
    if len(p) < 200:
        return TipUpPlan(False, f"Cok az nokta ({len(p)}).")

    if obb is None:
        obb = fit_obb(p, ground_z=config.GROUND_Z_MM, up_axis=config.UP_AXIS)

    ax_idx = {"x": 0, "y": 1, "z": 2}[config.UP_AXIS.lower()]
    height = obb.height_mm                 # yatikken bu ~ valiz kalinligi
    short_ax = obb.thickness_axis         # yer duzleminde kisa kenar (yatay)
    b = obb.thickness_mm                  # kisa yatay uzunluk

    if height > 0.7 * min(obb.width_mm, b) and not force:
        return TipUpPlan(
            False,
            f"Bu bulut yan yatik gorunmuyor (dikey uzanim {height:.0f}mm). "
            "Once durus siniflandirmasini kontrol edin.",
        )

    # itme yonunu robottan UZAGA yonlendir
    s_hat = short_ax / (np.linalg.norm(short_ax) + 1e-12)
    to_obj = obb.center[[i for i in range(3) if i != ax_idx]] - np.asarray(base_origin_xy)
    s_hat_h = s_hat[[i for i in range(3) if i != ax_idx]]
    if float(np.dot(s_hat_h, to_obj)) < 0:
        s_hat = -s_hat
    push_dir = s_hat.copy()
    push_dir[ax_idx] = 0.0
    push_dir /= np.linalg.norm(push_dir) + 1e-12

    # yakin kenar (robota bakan) = merkez - push_dir * b/2 ; donme kenari = + tarafta
    near_edge = obb.center - push_dir * (b / 2.0)
    pivot_edge = obb.center + push_dir * (b / 2.0)
    pivot_edge[ax_idx] = obb.ground_z
    near_edge[ax_idx] = obb.ground_z

    contact = near_edge + push_dir * min(15.0, 0.15 * b)     # yuzden biraz iceri
    contact[ax_idx] = obb.ground_z + config.TIPUP_PUSH_HEIGHT_FRAC * height

    pre_contact = contact - push_dir * 60.0
    pre_contact[ax_idx] += 30.0
    retract = contact - push_dir * 40.0
    retract[ax_idx] += 80.0

    push_distance = float(config.TIPUP_PUSH_DEPTH_MM)
    # dik olunca yukseklik ~ kisa yatay kenar (b) olur
    expected_h = b

    yaw = float(np.degrees(np.arctan2(push_dir[1], push_dir[0])))
    notes = [
        "Non-prehensile: DRY-RUN dogrula, dusuk hiz.",
        f"Itme yonu robottan uzaga ({yaw:.0f}deg). Donme kenari +yonde.",
        "Itme sonrasi TEKRAR goruntu al; DIK degilse push_distance'i artir.",
        f"TIPUP_PUSH_DEPTH_MM={config.TIPUP_PUSH_DEPTH_MM:.0f} baslangic - deneyerek ayarla.",
    ]
    return TipUpPlan(
        feasible=True,
        reason="Tip-up plani hazir (dogrulama gerektirir).",
        pre_contact=pre_contact, contact_point=contact, push_dir=push_dir,
        push_distance_mm=push_distance, retract_point=retract,
        pivot_edge_mid=pivot_edge, expected_upright_height_mm=expected_h,
        rpy_deg=(180.0, 0.0, yaw), notes=notes,
    )
