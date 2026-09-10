"""
Kavrama planlama - 3B nokta bulutundan (robot taban cercevesi, mm, Z yukari).

Valizin 2 DUZ KULPSUZ KABUGU (buyuk on/arka yuz) arasindan, KALINLIK
ekseni boyunca kavranir. Kavrama noktasi bu iki yuzun orta duzlemi,
zeminden GRASP_HEIGHT_FRACTION kadar yukarida. Gripper tepeden iner,
parmaklar iki kabugun yanindan gecer, sonra kapanir.

Sadece numpy - cv2 gerekmez (test edilebilir).
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from . import config

logger = logging.getLogger(__name__)


@dataclass
class OBB:
    """Yerçekimine hizali yonelimli kutu (robot taban cercevesi, mm)."""

    center: np.ndarray                 # (3,) kutu merkezi
    axes: np.ndarray                   # (3,3) satirlar: uzun-yatay, kisa-yatay(kalinlik), dikey
    extents: np.ndarray               # (3,) her eksen boyunca TAM uzunluk (mm)
    ground_z: float
    yaw_deg: float                    # uzun yatay eksenin robot X'ine gore acisi

    @property
    def long_axis(self) -> np.ndarray: return self.axes[0]
    @property
    def thickness_axis(self) -> np.ndarray: return self.axes[1]
    @property
    def up_axis(self) -> np.ndarray: return self.axes[2]
    @property
    def width_mm(self) -> float: return float(self.extents[0])
    @property
    def thickness_mm(self) -> float: return float(self.extents[1])
    @property
    def height_mm(self) -> float: return float(self.extents[2])


@dataclass
class GraspPlan:
    feasible: bool
    reason: str
    grasp_point: Optional[np.ndarray] = None      # (3,) TCP hedef konumu (mm, taban)
    approach_point: Optional[np.ndarray] = None   # (3,) on-yaklasma (grasp + yukari)
    lift_point: Optional[np.ndarray] = None       # (3,) kaldirma sonrasi
    close_axis: Optional[np.ndarray] = None       # (3,) parmaklarin kapanma yonu (=kalinlik ekseni)
    approach_axis: Optional[np.ndarray] = None    # (3,) yaklasma yonu (genelde -Z)
    opening_mm: float = 0.0                       # gerekli acilma (kalinlik + bosluk)
    yaw_deg: float = 0.0                          # tool yaw (kapanma ekseni, taban X'e gore)
    rpy_deg: tuple[float, float, float] = (180.0, 0.0, 0.0)  # FR3 [rx,ry,rz] - tool asagi
    faces: dict = field(default_factory=dict)     # duz yuz teshis bilgisi

    def as_dict(self) -> dict:
        def _l(v): return None if v is None else [round(float(x), 1) for x in v]
        return {
            "uygulanabilir": self.feasible,
            "sebep": self.reason,
            "kavrama_noktasi_mm": _l(self.grasp_point),
            "yaklasma_noktasi_mm": _l(self.approach_point),
            "kaldirma_noktasi_mm": _l(self.lift_point),
            "kapanma_ekseni": _l(self.close_axis),
            "gerekli_acilma_mm": round(self.opening_mm, 1),
            "tool_yaw_derece": round(self.yaw_deg, 1),
            "rpy_derece": [round(v, 1) for v in self.rpy_deg],
            "duz_yuz_teshis": self.faces,
        }


# ----------------------------------------------------------------------
def _pca_xy(xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    c = xy - xy.mean(axis=0)
    cov = np.cov(c.T)
    w, v = np.linalg.eigh(cov)          # artan
    order = np.argsort(w)[::-1]         # buyukten kucuge
    return v[:, order[0]], v[:, order[1]]


def _robust_extent(vals: np.ndarray, lo=2.0, hi=98.0) -> float:
    a, b = np.percentile(vals, [lo, hi])
    return float(b - a)


def fit_obb(points_mm: np.ndarray, ground_z: Optional[float] = None,
            up_axis: str = "z") -> OBB:
    """
    points_mm : (N,3) robot taban cercevesi.
    Yerçekimi ekseni sabit (up_axis); yatay iki eksen XY-PCA'dan.
    """
    p = np.asarray(points_mm, float)
    ax_idx = {"x": 0, "y": 1, "z": 2}[up_axis.lower()]
    horiz_idx = [i for i in range(3) if i != ax_idx]

    up = np.zeros(3); up[ax_idx] = 1.0
    gz = float(np.percentile(p[:, ax_idx], 2)) if ground_z is None else float(ground_z)
    top = float(np.percentile(p[:, ax_idx], 98))
    height = max(top - gz, 1e-3)

    xy = p[:, horiz_idx]
    e_long, e_short = _pca_xy(xy)
    long3 = np.zeros(3); long3[horiz_idx] = e_long
    short3 = np.zeros(3); short3[horiz_idx] = e_short
    # dik uclu koordinat sistemi
    short3 = np.cross(up, long3); short3 /= np.linalg.norm(short3) + 1e-12
    long3 = np.cross(short3, up); long3 /= np.linalg.norm(long3) + 1e-12

    s_long = p @ long3
    s_short = p @ short3
    ext_long = _robust_extent(s_long)
    ext_short = _robust_extent(s_short)
    if ext_short > ext_long:                      # uzun daima >= kisa
        long3, short3 = short3, long3
        ext_long, ext_short = ext_short, ext_long

    center = np.zeros(3)
    center[horiz_idx] = xy.mean(axis=0)
    center[ax_idx] = gz + height / 2.0
    # merkezi eksen ortalarina cek
    center = center - long3 * (center @ long3 - np.median(s_long)) \
                    - short3 * (center @ short3 - np.median(s_short))
    center[ax_idx] = gz + height / 2.0

    yaw = float(np.degrees(np.arctan2(long3[1], long3[0])))
    axes = np.vstack([long3, short3, up])
    extents = np.array([ext_long, ext_short, height])
    return OBB(center=center, axes=axes, extents=extents, ground_z=gz, yaw_deg=yaw)


# ----------------------------------------------------------------------
def _face_flatness(points_mm: np.ndarray, obb: OBB, axis: np.ndarray,
                   half_extent: float) -> dict:
    """
    Kutunun +axis ve -axis yonundeki iki yuzune yakin noktalarin, o yuz
    duzlemine uzakliklarinin std'si. Kucukse yuz duz/pruzsuz (kulp yok).
    """
    s = points_mm @ axis
    s0 = obb.center @ axis
    band = 0.18 * (2 * half_extent)
    out = {}
    for sign, name in ((+1, "arti"), (-1, "eksi")):
        face_s = s0 + sign * half_extent
        near = np.abs(s - face_s) < band
        if near.sum() < 30:
            out[name] = dict(n=int(near.sum()), std_mm=None)
            continue
        resid = s[near] - np.median(s[near])
        out[name] = dict(n=int(near.sum()), std_mm=float(np.std(resid)))
    return out


def plan_grasp(points_mm: np.ndarray, obb: Optional[OBB] = None,
               force_grasp_z_mm: Optional[float] = None,
               force_thickness_mm: Optional[float] = None) -> GraspPlan:
    """
    points_mm : valiz noktalari, robot taban cercevesi (mm, Z yukari).
    Doner: GraspPlan (uygulanabilir mi + TCP hedefleri).

    force_grasp_z_mm   : verilirse kavrama TCP yuksekligi (tabandan mm) OBB yerine
                         bu deger. Gurultulu stereo'da dikey eksen guvenilmez;
                         operator elle girer.
    force_thickness_mm : verilirse gerekli acilma bu kalinliktan hesaplanir.
    """
    p = np.asarray(points_mm, float)
    if len(p) < 200:
        return GraspPlan(False, f"Cok az nokta ({len(p)}).")

    if obb is None:
        obb = fit_obb(p, ground_z=config.GROUND_Z_MM, up_axis=config.UP_AXIS)

    thick = obb.thickness_axis
    up = obb.up_axis
    thickness_mm = float(force_thickness_mm) if force_thickness_mm else obb.thickness_mm

    # --- 2 duz kabuk: kalinlik eksenine dik yuzler ---
    flat = _face_flatness(p, obb, thick, obb.thickness_mm / 2.0)
    stds = [f["std_mm"] for f in flat.values() if f.get("std_mm") is not None]
    faces_ok = len(stds) == 2 and max(stds) <= config.FLAT_FACE_STD_MM
    faces_info = {"eksen": "kalinlik", "yuzler": flat,
                  "duz_esik_mm": config.FLAT_FACE_STD_MM, "duz_mu": faces_ok}

    # --- gereken acilma ---
    opening = thickness_mm + 2 * config.GRASP_CLEARANCE_MM
    if opening > config.GRIPPER_MAX_OPENING_MM:
        return GraspPlan(
            False,
            f"Kalinlik {obb.thickness_mm:.0f}mm + bosluk -> {opening:.0f}mm > "
            f"gripper max {config.GRIPPER_MAX_OPENING_MM:.0f}mm. Kavranamaz.",
            faces=faces_info, opening_mm=opening,
        )
    if opening < config.GRIPPER_MIN_OPENING_MM:
        return GraspPlan(False, f"Kalinlik cok kucuk ({obb.thickness_mm:.0f}mm).",
                         faces=faces_info, opening_mm=opening)

    # --- kavrama noktasi: iki yuzun orta duzlemi, tabandan oranli yukarida ---
    gp = obb.center.copy()
    ax_idx = {"x": 0, "y": 1, "z": 2}[config.UP_AXIS.lower()]
    if force_grasp_z_mm is not None:
        gp[ax_idx] = obb.ground_z + float(force_grasp_z_mm)
    else:
        gp[ax_idx] = obb.ground_z + config.GRASP_HEIGHT_FRACTION * obb.height_mm

    approach_axis = -up                                   # tepeden in
    approach = gp - approach_axis * config.GRASP_APPROACH_MM
    lift = gp + up * config.GRASP_LIFT_MM

    # tool yaw: parmaklar kalinlik ekseni boyunca kapanir
    yaw = float(np.degrees(np.arctan2(thick[1], thick[0])))
    rpy = (180.0, 0.0, yaw)                               # tool asagi + yaw

    # parmak boyu yeterli mi (tepe ile kavrama yuksekligi arasi mesafe)
    depth_from_top = obb.height_mm * (1.0 - config.GRASP_HEIGHT_FRACTION)
    warn = ""
    if depth_from_top > config.GRIPPER_FINGER_LEN_MM:
        warn = (f" UYARI: parmak boyu {config.GRIPPER_FINGER_LEN_MM:.0f}mm, "
                f"tepeden kavrama derinligi {depth_from_top:.0f}mm - kisa kalabilir.")

    reason = "Kavrama plani hazir." + ("" if faces_ok else
             " (Duz yuz testi zayif - kabukta cikinti olabilir.)") + warn
    return GraspPlan(
        feasible=True, reason=reason,
        grasp_point=gp, approach_point=approach, lift_point=lift,
        close_axis=thick, approach_axis=approach_axis,
        opening_mm=opening, yaw_deg=yaw, rpy_deg=rpy, faces=faces_info,
    )
