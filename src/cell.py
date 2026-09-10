"""
HUCRE MODU orkestrasyonu: sabit 2 kamera (kalibreli) + FR3 + Jodell gripper.

Akis:
  L,R goruntu -> YOLO26-seg (VALIZ MI?) -> kalibreli rektifikasyon ->
  SGBM+WLS disparite -> Q ile 3B (mm, kamera) -> base_T_cam -> 3B bulut (robot taban) ->
  OBB + olcu + durus  ->  DIK: kavrama plani  /  YATIK: once dik konuma getir  ->
  FR3 programi (dry-run varsayilan).

pipeline.py (2 fotograf demo) ayni kaliyor; bu ayri giris noktasi.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from . import config
from .calib import BaseTransform, StereoRig
from .detector import Detection, SuitcaseDetector
from .grasp import OBB, GraspPlan, fit_obb, plan_grasp
from .align import TipUpPlan, plan_tipup
from .pointcloud import PointCloud, extract
from .pose_classify import PostureResult, classify_obb
from .robot import CellProgram, build_pick_place, build_tipup
from .stereo import colorize_disparity, compute_disparity_adaptive

logger = logging.getLogger(__name__)

_DET: Optional[SuitcaseDetector] = None
_DET_KEY: tuple = ()


def _detector(accept_extra: Optional[set] = None) -> SuitcaseDetector:
    """accept_extra degistiginde dedektoru yeniden kur (model yeniden yuklenir)."""
    global _DET, _DET_KEY
    key = tuple(sorted(accept_extra or ()))
    if _DET is None or key != _DET_KEY:
        _DET = SuitcaseDetector(accept_extra=set(key))
        _DET_KEY = key
    return _DET


def _to_gray(img: np.ndarray) -> np.ndarray:
    import cv2
    return img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def _row_energy(g):
    import cv2
    gx = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)       # dikey gradyan
    return np.abs(gx).mean(axis=1)                     # her satirin doku enerjisi


def _epipolar_correct(lr: np.ndarray, rr: np.ndarray):
    """Rijit montaj yok -> rektifiye ciftte artik DIKEY kayma kalir. TAM OTOMATIK,
    her karede: SAG goruntuyu dikeyde kaydirarak SOL ile en iyi hizayi ara
    (kaba-ince). Tekrarli/rastgele dokuda ORB sasar; bu yontem tum goruntuyu
    kullandigi icin saglam. Doner: (rr_duzeltilmis, dy, uygulandi_mi)."""
    import cv2
    gL = _to_gray(lr).astype(np.float32)
    gR = _to_gray(rr).astype(np.float32)
    H, W = gL.shape
    y0, y1 = int(H * 0.12), int(H * 0.88)
    x0, x1 = int(W * 0.12), int(W * 0.88)
    A = gL[y0:y1, x0:x1]
    B_full = gR

    def cost(dy):                                       # SAG'i dy kadar kaydir, farkin std'si
        M = np.float32([[1, 0, 0], [0, 1, dy]])
        Bs = cv2.warpAffine(B_full, M, (W, H), flags=cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_REPLICATE)[y0:y1, x0:x1]
        # yatay disparite var -> satir-enerji profillerini hizala (dikeye duyarli)
        ea, eb = _row_energy(A), _row_energy(Bs)
        ea = (ea - ea.mean()) / (ea.std() + 1e-6)
        eb = (eb - eb.mean()) / (eb.std() + 1e-6)
        return float(np.mean((ea - eb) ** 2))

    coarse = np.arange(-42, 43, 2.0)
    cc = [cost(dy) for dy in coarse]
    d0 = float(coarse[int(np.argmin(cc))])
    fine = np.arange(d0 - 2, d0 + 2.01, 0.25)
    fc = [cost(dy) for dy in fine]
    dy = float(fine[int(np.argmin(fc))])
    # guven: en iyi cost belirgin mi
    cmin, cmed = min(fc + cc), float(np.median(cc))
    if abs(dy) < 1.0 or abs(dy) > 40.0 or cmin > 0.85 * cmed:
        return rr, dy, False
    M = np.float32([[1, 0, 0], [0, 1, dy]])
    rr2 = cv2.warpAffine(rr, M, (W, H), flags=cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_REPLICATE)
    return rr2, dy, True


# ----- BOS MASA referansi (tools/learn_table.py ciktisi) -----
_TABLE_REF: Optional[dict] = None
_TABLE_REF_MTIME: float = -1.0


def _load_table_ref() -> Optional[dict]:
    """src/calibration/table_ref.npz -> {n,d,region,above_mm,thr}. Yoksa None.
    Dosya degistiyse otomatik yeniden yuklenir."""
    global _TABLE_REF, _TABLE_REF_MTIME
    import os
    p = os.path.join(config.CALIBRATION_DIR, "table_ref.npz")
    if not os.path.exists(p):
        return None
    m = os.path.getmtime(p)
    if _TABLE_REF is None or m != _TABLE_REF_MTIME:
        z = np.load(p, allow_pickle=False)
        _TABLE_REF = {
            "n": z["plane_n"].astype(float), "d": float(z["plane_d"]),
            "region": z["region"].astype(bool),
            "above_mm": float(z["above_mm"]), "thr": float(z["plane_thresh_mm"]),
        }
        _TABLE_REF_MTIME = m
        logger.info("table_ref yuklendi (%s)", p)
    return _TABLE_REF


def learn_table(image_left: np.ndarray, image_right: np.ndarray,
                rig: "Optional[StereoRig]" = None,
                out_path: Optional[str] = None, above_mm: float = 12.0) -> dict:
    """BOS MASA cifti -> masa duzlemi + goruntu bolgesi -> table_ref.npz kaydet.
    Sonrasinda run_cell YOLO bulamazsa nesneyi bununla otomatik ayirir."""
    import os
    import cv2
    from .calib import StereoRig
    from .segment import fit_plane_ransac
    from .stereo import compute_disparity_adaptive

    rig = rig or StereoRig.load()
    lr, rr = rig.rectify(image_left, image_right)
    rr, _dy, _ = _epipolar_correct(lr, rr)             # kamera kaymasini otomatik onar
    disp = compute_disparity_adaptive(_to_gray(lr), _to_gray(rr), roi_mask=None)
    pts = rig.reproject(disp.disparity)
    H, W = pts.shape[:2]
    finite = np.isfinite(pts).all(2) & (disp.disparity > 0)
    P = pts[finite]
    if len(P) < 5000:
        raise RuntimeError(f"Cok az gecerli 3B nokta ({len(P)}). Isik/doku/kalibrasyon?")
    span = np.percentile(P, 97, 0) - np.percentile(P, 3, 0)
    thr = max(0.01 * float(np.linalg.norm(span)), 4.0)
    n, d, inl = fit_plane_ransac(P, thr, iters=400)
    frac = float(inl.mean())
    c = P[inl].mean(0)
    if float(n @ (-c)) < 0:
        n, d = -n, -d
    tmask = np.zeros((H, W), bool)
    ys, xs = np.where(finite)
    tmask[ys[inl], xs[inl]] = True
    e = cv2.getStructuringElement
    tm = cv2.morphologyEx(tmask.astype(np.uint8), cv2.MORPH_CLOSE, e(cv2.MORPH_ELLIPSE, (15, 15)))
    tm = cv2.morphologyEx(tm, cv2.MORPH_OPEN, e(cv2.MORPH_ELLIPSE, (7, 7)))
    tm = cv2.morphologyEx(tm, cv2.MORPH_CLOSE, e(cv2.MORPH_ELLIPSE, (61, 61)))
    nlab, lab, stats, _ = cv2.connectedComponentsWithStats(tm, 8)
    if nlab > 1:
        tm = (lab == 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))).astype(np.uint8)
    region = cv2.erode(tm, e(cv2.MORPH_ELLIPSE, (11, 11))) > 0

    out_path = out_path or os.path.join(config.CALIBRATION_DIR, "table_ref.npz")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    np.savez_compressed(out_path, plane_n=n.astype(np.float64), plane_d=np.float64(d),
                        region=region, image_size=np.array([W, H]),
                        above_mm=np.float64(above_mm), plane_thresh_mm=np.float64(thr))
    global _TABLE_REF, _TABLE_REF_MTIME
    _TABLE_REF = None; _TABLE_REF_MTIME = -1.0        # cache'i temizle
    info = {"dosya": out_path, "duzlem_inlier_%": round(100 * frac, 1),
            "esik_mm": round(thr, 1), "bolge_%": round(100 * float(region.mean()), 1),
            "normal": [round(float(v), 3) for v in n]}
    logger.info("learn_table: %s", info)
    return info


def _refine_obb_with_plane(pc_xyz: np.ndarray, obb: "OBB", table_plane, up_idx: int,
                           res: "CellResult"):
    """Masa duzlemi (n,d) bilindiginde OBB'yi duzelt:
      yukseklik = noktalarin duzleme p98 mesafesi
      ayak izi  = TEPE %25 diliminin minAreaRect'i (yan yuzler ayak izini sisirmez)
    Doner: yeni OBB ya da None (yeterli nokta yoksa)."""
    import cv2
    n, d = table_plane
    p = np.asarray(pc_xyz, float)
    if len(p) < 200:
        return None
    s = p @ n + d
    obj = (s > 10.0) & (s < 800.0)
    if int(obj.sum()) < 200:
        return None
    po, so = p[obj], s[obj]
    H = float(np.percentile(so, 97))
    if H < 8.0:
        return None
    hz = [i for i in range(3) if i != up_idx]

    def _rect(mask):
        xy = po[mask][:, hz]
        lo, hi = np.percentile(xy, [1.5, 98.5], axis=0)   # sadece uc outlier'lari at
        k = np.all((xy >= lo) & (xy <= hi), axis=1)
        xy = xy[k] if k.sum() >= 30 else xy
        (cx, cy), (w1, w2), ang = cv2.minAreaRect(xy.astype(np.float32))
        return (cx, cy), max(w1, w2), min(w1, w2), ang

    # tum nesne noktalarindan kaba kisa-kenar -> ayakta mi yatik mi?
    _, _, short_all, _ = _rect(so > 10.0)
    if H > 1.3 * max(short_all, 1.0):
        # AYAKTA: yan yuzler ayak izini sisirir -> sadece TEPE dilimini kullan
        sel = so >= 0.72 * H
        if int(sel.sum()) < 80:
            sel = so >= 0.5 * H
        res.diagnostics["olcu_dilim"] = "ayakta -> tepe dilimi"
    else:
        # YATIK: gorunen ust yuzey zaten ayak izi -> TUM noktalar (daha yogun/dogru)
        sel = so > 10.0
        res.diagnostics["olcu_dilim"] = "yatik -> tum nokta"
    (cx, cy), long_mm, short_mm, ang = _rect(sel)
    if long_mm < 10 or short_mm < 5:
        return None
    center = obb.center.copy()
    center[hz[0]] = float(cx); center[hz[1]] = float(cy)
    center[up_idx] = obb.ground_z + H / 2.0
    axes = obb.axes.copy()
    th = np.radians(ang)
    long3 = np.zeros(3); long3[hz[0]] = np.cos(th); long3[hz[1]] = np.sin(th)
    short3 = np.zeros(3); short3[hz[0]] = -np.sin(th); short3[hz[1]] = np.cos(th)
    axes[0], axes[1] = long3, short3
    extents = np.array([long_mm, short_mm, H], float)
    yaw = float(np.degrees(np.arctan2(long3[1], long3[0])))
    res.diagnostics["olcu_kaynak"] = ("ayak izi=tepe dilimi minAreaRect, "
                                      "yukseklik=masa duzlemi mesafesi")
    res.diagnostics["ham_obb_mm"] = [round(obb.width_mm, 0), round(obb.height_mm, 0),
                                     round(obb.thickness_mm, 0)]
    return OBB(center=center, axes=axes, extents=extents,
              ground_z=obb.ground_z, yaw_deg=yaw)


def _table_object_mask(pts_cam: np.ndarray, disparity: np.ndarray,
                       ref: dict, res: "CellResult") -> Optional[np.ndarray]:
    """Rektifiye sol cercevede nesne maskesi: masa bolgesi ICINDE + masa
    duzleminin >above_mm USTUNDE (kameraya yakin) + gecerli disparite."""
    import cv2
    H, W = disparity.shape
    region = ref["region"]
    if region.shape != (H, W):
        region = cv2.resize(region.astype(np.uint8), (W, H),
                            interpolation=cv2.INTER_NEAREST) > 0
    finite = np.isfinite(pts_cam).all(2) & (disparity > 0)
    safe = np.where(finite[..., None], pts_cam, 0.0)
    signed = safe @ ref["n"] + ref["d"]              # masa ~0
    lo = max(ref["above_mm"], 8.0)
    up = finite & (signed > lo) & (signed < lo + 600.0)
    dn = finite & (signed < -lo) & (signed > -(lo + 600.0))
    band = up if int(up.sum()) >= int(dn.sum()) else dn     # nesnenin oldugu taraf
    obj = band & region
    if int(obj.sum()) < 150:                          # region nesneyi disliyor -> bolgesiz
        obj = band
        res.diagnostics["masa_ref_not"] = "region maskesi kullanilmadi (nesneyi dislemis)"
    m = obj.astype(np.uint8)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
    nlab, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)
    if nlab > 1:
        big = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        m = (lab == big).astype(np.uint8)
    mb = m > 0
    res.diagnostics["masa_ref_nesne_px"] = int(mb.sum())
    # tepeden kamera nesnenin YAN yuzlerini goremez -> yukseklik = masa duzleminden
    # RECONSTRUCT edilen EN YUKSEK noktaya mesafe (p98; ayakta nesnede tepe budur)
    sv = np.abs(signed[mb & finite])
    if len(sv) > 50:
        res.diagnostics["masa_ustu_yukseklik_mm"] = round(float(np.percentile(sv, 98)), 1)
        res.diagnostics["masa_ustu_medyan_mm"] = round(float(np.median(sv)), 1)
    return mb


def _lean_from_points(pts_mm: np.ndarray, up_idx: int) -> float:
    """Bulutun ana ekseninin dunya-dikeyinden sapmasi (derece)."""
    if len(pts_mm) < 50:
        return 0.0
    c = pts_mm - pts_mm.mean(0)
    w, v = np.linalg.eigh(np.cov(c.T))
    principal = v[:, int(np.argmax(w))]
    up = np.zeros(3); up[up_idx] = 1.0
    cos_a = abs(float(np.dot(principal / (np.linalg.norm(principal) + 1e-9), up)))
    return float(np.degrees(np.arccos(min(1.0, max(0.0, cos_a)))))


# ----------------------------------------------------------------------
@dataclass
class CellResult:
    is_suitcase: bool
    message: str
    ok: bool = True

    detection_left: Optional[Detection] = None
    detection_right: Optional[Detection] = None
    obb: Optional[OBB] = None
    posture: Optional[PostureResult] = None
    grasp: Optional[GraspPlan] = None
    tipup: Optional[TipUpPlan] = None
    program: Optional[CellProgram] = None

    pointcloud_base: Optional[PointCloud] = None
    disparity_color: Optional[np.ndarray] = None
    measurement_text: str = ""
    diagnostics: dict = field(default_factory=dict)
    warnings: list = field(default_factory=list)

    def summary(self) -> dict:
        out = {"valiz_mi": self.is_suitcase, "mesaj": self.message, "tam_sonuc": self.ok}
        if self.measurement_text:
            out["olcum"] = self.measurement_text
        if self.obb is not None:
            o = self.obb
            dims = sorted([o.width_mm, o.height_mm, o.thickness_mm], reverse=True)
            # ETIKET KONVANSIYONU (yonelimden bagimsiz):
            #   boy = EN UZUN,  en = ORTANCA,  derinlik = EN KISA (kavrama ekseni)
            out["olcu_cm"] = {
                "boy": round(dims[0] / 10, 1),
                "en": round(dims[1] / 10, 1),
                "derinlik": round(dims[2] / 10, 1),
                "en_buyuk_boyut_cm": round(dims[0] / 10, 1),
                "orta_boyut_cm": round(dims[1] / 10, 1),
                "en_kucuk_boyut_cm": round(dims[2] / 10, 1),
                "yaw_derece": round(o.yaw_deg, 1),
                "merkez_mm": [round(v, 1) for v in o.center],
                "cerceve": self.diagnostics.get("cerceve", "?"),
            }
        if self.posture:
            out["durus"] = {"etiket": self.posture.label, "aciklama": self.posture.label_tr,
                            "robot_gerekli": self.posture.needs_robot,
                            "guven": self.posture.confidence}
        if self.grasp:
            out["kavrama"] = self.grasp.as_dict()
        if self.tipup:
            out["dik_konuma_getirme"] = self.tipup.as_dict()
        if self.program:
            out["fr3_program"] = self.program.to_dict()
        if self.diagnostics:
            out["teshis"] = self.diagnostics
        if self.warnings:
            out["uyarilar"] = self.warnings
        return out


# ----------------------------------------------------------------------
def run_cell(
    image_left: np.ndarray,
    image_right: np.ndarray,
    rig: Optional[StereoRig] = None,
    base_T_cam: Optional[BaseTransform] = None,
    detector: Optional[SuitcaseDetector] = None,
    place_pose: Optional[list] = None,
    accept_extra: Optional[set] = None,
    posture_override: Optional[str] = None,
    grasp_z_mm: Optional[float] = None,
    grasp_thickness_mm: Optional[float] = None,
    roi_xywh: Optional[tuple] = None,
) -> CellResult:
    """accept_extra: test amacli ek kabul edilen sinif adlari (orn. {'backpack'}).

    posture_override : "standing_on_wheels" | "tipped_over" -> otomatik durus
        siniflandirmasini ATLA (gurultulu stereo'da operator elle karar verir).
    grasp_z_mm       : elle DIK'te kavrama TCP yuksekligi (tabandan mm).
    grasp_thickness_mm : elle DIK'te nesne kalinligi (gripper acilmasi bundan).
    roi_xywh : SOL goruntude nesneyi ceviren piksel dikdortgeni (x,y,w,h).
        Verilirse YOLO yerine bu kullanilir; icindeki masa duzlemi otomatik atilir.
    """
    det = detector or _detector(accept_extra)
    rig = rig or StereoRig.load()
    base = base_T_cam or _load_base()

    # --- 1) VALIZ MI?  (YOLO -> elle ROI -> BOS MASA referansi -> ham duzlem) ---
    plane_seg = False              # 3B temizlikte masa duzlemi mutlaka atilsin
    roi_mask_full = None
    table_ref = None
    dL = dR = None
    if roi_xywh is not None:
        x, y, w, h = (int(v) for v in roi_xywh)
        H, W = image_left.shape[:2]
        x = max(0, min(x, W - 2)); y = max(0, min(y, H - 2))
        w = max(2, min(w, W - x)); h = max(2, min(h, H - y))
        roi_mask_full = np.zeros((H, W), bool)
        roi_mask_full[y:y + h, x:x + w] = True
        plane_seg = True
        res = CellResult(True, f"Elle ROI ({x},{y},{w}x{h}).")
    else:
        dL = det.detect(image_left)
        dR = det.detect(image_right)
        if dL is not None and dR is not None:
            res = CellResult(True, "Valiz tespit edildi (YOLO).",
                             detection_left=dL, detection_right=dR)
        elif (dL is None) != (dR is None):
            return CellResult(True, "Valiz sadece bir kamerada goruldu; digerinde yok.",
                              ok=False, detection_left=dL, detection_right=dR)
        else:
            table_ref = _load_table_ref()
            if table_ref is not None:
                plane_seg = True
                res = CellResult(True, "YOLO yok -> BOS MASA referansindan nesne ayrildi.")
            elif getattr(config, "CELL_PLANE_SEG", True):
                plane_seg = True
                res = CellResult(True, "YOLO yok, masa referansi yok -> ham duzlem "
                                 "segmentasyonu (once `learn_table` calistir; daha saglam).")
            else:
                return CellResult(False, "Valiz tespit edilmedi. `tools/learn_table` ile "
                                  "bos masayi ogret ya da gercek valiz koy.")

    # --- 2) kalibreli rektifikasyon + disparite ---
    lr, rr = rig.rectify(image_left, image_right)
    rr, epi_dy, epi_ok = _epipolar_correct(lr, rr)     # kamera kaymasini otomatik onar
    if roi_mask_full is not None:
        disp_roi = rig.remap_mask(roi_mask_full, "left")
    elif dL is not None:
        disp_roi = rig.remap_mask(dL.mask, "left")
    else:
        disp_roi = None                                          # tum kare (masa/duzlem)
    disp = compute_disparity_adaptive(_to_gray(lr), _to_gray(rr), roi_mask=disp_roi)
    res.disparity_color = colorize_disparity(disp.disparity)

    roi_ratio = disp.roi_valid_ratio if disp.roi_valid_ratio >= 0 else disp.valid_ratio
    pr = disp.probe or {}
    res.diagnostics = {
        "disparite_gecerli_ROI_%": round(roi_ratio * 100, 1),
        "disparite_parametre": disp.params_used,
        "rektifiye_dikey_hata_px": round(pr.get("dy_med", -1), 1),
        "epipolar_oto_duzeltme_px": (f"{epi_dy:+.1f} uygulandi" if epi_ok
                                     else f"{epi_dy:+.1f} (gerek yok/guvensiz)"),
        "baseline_mm": round(rig.baseline_mm, 1),
        "base_T_cam_residual_mm": round(base.residual_mm, 2),
    }
    if roi_ratio < 0.10:
        res.ok = False
        res.message = f"Yeterli 3B nokta yok (ROI %{roi_ratio*100:.1f})."
        res.warnings.append("Isik/doku/kalibrasyon kontrol; SGBM/WLS .env ile gevset.")
        return res

    # --- 3) 3B bulut (mm, kamera) -> robot taban ---
    pts_cam = rig.reproject(disp.disparity)                # (H,W,3) mm (kalibreli)
    obj_mask_rect = None                                   # rektifiye sol cercevede nesne maskesi
    if disp_roi is not None:
        obj_mask_rect = disp_roi
    if table_ref is not None:
        obj_mask_rect = _table_object_mask(pts_cam, disp.disparity, table_ref, res)
        if obj_mask_rect is None or int(obj_mask_rect.sum()) < 150:
            res.ok = False
            res.message = "Bos masa referansina gore masada ayirt edilebilir nesne yok."
            return res
    if obj_mask_rect is not None:
        pc_cam = extract(pts_cam, obj_mask_rect, color_bgr=lr, only_suitcase=True)
    else:
        pc_cam = extract(pts_cam, np.ones(pts_cam.shape[:2], bool),
                         color_bgr=lr, only_suitcase=False)
    have_handeye = not np.allclose(base.T, np.eye(4))
    pc_base = pc_cam.transform(base.T) if have_handeye else pc_cam

    # Iki bulut:
    #  pc_full -> HAFIF temizlik (derinlik-sisi + gurultu). GOSTERILEN bulut, renkli.
    #  pc_obb  -> pc_full + agresif ayiklama (zemin/en-buyuk-kume). SADECE olcu/OBB icin.
    #            Agresif adim bulutu asiri kirparsa (%40'tan az kalirsa) pc_full'e don.
    n0 = len(pc_base)
    up_idx = {"x": 0, "y": 1, "z": 2}[config.UP_AXIS.lower()]
    pc_full = pc_base
    pc_obb = pc_base
    table_plane = None                                 # (n, d) rektifiye sol cerc.
    if n0 >= 200:
        from .pointcloud import PointCloud
        from .segment import (fit_plane_ransac, grid_depth_filter, isolate_object,
                              statistical_outlier_filter)
        xyz, rgb = pc_base.xyz, pc_base.rgb
        gk = grid_depth_filter(xyz, up_axis=up_idx, cell_mm=25.0, band_mm=60.0)
        xyz, rgb = xyz[gk], (rgb[gk] if rgb is not None else None)
        sk = statistical_outlier_filter(xyz, k=16, std_ratio=2.5)
        xyz, rgb = xyz[sk], (rgb[sk] if rgb is not None else None)
        # maske genis (plane_seg/ROI/hand-eye) -> EN BUYUK duzlemi (masa) ve onun
        # ARKASINDAKI her seyi kes; sadece kameraya dogru >15mm one cikanlari tut.
        # Deterministik - seyrek bulutta bile calisir. YOLO maskesinde masa yok.
        clean_mask = table_ref is not None
        wide_mask = (plane_seg and not clean_mask) or have_handeye
        cut_n = 0
        if wide_mask and len(xyz) >= 200:
            sp = np.percentile(xyz, 97, 0) - np.percentile(xyz, 3, 0)
            pthr = max(0.012 * float(np.linalg.norm(sp)), 4.0)
            pn, pd, pinl = fit_plane_ransac(xyz, pthr, iters=400)
            if 0.15 <= pinl.mean() <= 0.92:
                c = xyz[pinl].mean(0)
                if float(pn @ (-c)) < 0:            # normali kameraya dogru cevir
                    pn, pd = -pn, -pd
                signed = xyz @ pn + pd              # masa ~0, one cikan (nesne) > 0
                keep = signed > 15.0
                if int(keep.sum()) >= 150:
                    cut_n = len(xyz) - int(keep.sum())
                    xyz, rgb = xyz[keep], (rgb[keep] if rgb is not None else None)
                    table_plane = (pn.astype(float), float(pd))
        if len(xyz) >= 100:
            pc_full = PointCloud(xyz=xyz, rgb=rgb)
        res.diagnostics["bulut"] = f"ham {n0} -> gosterilen {len(pc_full)} (masa duzlemi kesildi: {cut_n} nokta)"

        # OBB icin: en buyuk kume ile daha da sik. Asiri kirparsa pc_full'e don.
        if len(pc_full) >= 150:
            oxyz, orgb, seg_info = isolate_object(
                pc_full.xyz, pc_full.rgb, drop_ground=False,
                nearest_pct=(100.0 if not plane_seg else 96.0),
                denoise=True, keep_largest=True)
            if plane_seg:
                res.diagnostics["duzlem_seg"] = seg_info
            if len(oxyz) >= max(400, 0.40 * len(pc_full)):
                pc_obb = PointCloud(xyz=oxyz, rgb=orgb)
                res.diagnostics["olcu_bulutu"] = f"{len(pc_full)} -> {len(oxyz)}"
            else:
                res.diagnostics["olcu_bulutu"] = (
                    f"en-buyuk-kume {len(pc_full)}->{len(oxyz)} cok kirpti, "
                    "gosterilen bulut kullanildi")
    res.pointcloud_base = pc_full
    pc_base = pc_obb                                   # asagidaki OBB/plan pc_obb kullanir
    res.diagnostics["cerceve"] = "robot_taban" if have_handeye else "kamera (hand-eye yok)"
    if not have_handeye:
        res.warnings.append(
            "base_T_cam yok -> OLCULER kamera cercevesinde ve METRIK (kalibreli "
            "stereodan). en/boy/derinlik dogru; ama FR3 pozlari GECERSIZ - "
            "kavrama/tip-up icin tools/handeye_solve.py calistirin.")
    if len(pc_base) < 300:
        res.ok = False
        res.message = f"Bulut cok seyrek ({len(pc_base)} nokta)."
        return res

    # --- 4) OBB + olcu + durus ---
    up_idx = {"x": 0, "y": 1, "z": 2}[config.UP_AXIS.lower()]
    # hand-eye varsa zemin Z'si bilinir; yoksa buluttan otomatik (en alt %2)
    gz = config.GROUND_Z_MM if have_handeye else None
    obb = fit_obb(pc_base.xyz, ground_z=gz, up_axis=config.UP_AXIS)
    # Egik tek-yon stereo: ayakta nesnenin YAN yuzleri XY'ye genis yansir -> ham
    # ayak izi sisirilir. Masa duzlemi varsa: YUKSEKLIK = duzleme p98 mesafe,
    # AYAK IZI = nesnenin TEPE diliminin siki dikdortgeni (yan yuzler haric).
    if table_plane is not None and not have_handeye:
        obb2 = _refine_obb_with_plane(pc_base.xyz, obb, table_plane, up_idx, res)
        if obb2 is not None:
            obb = obb2
    res.obb = obb
    # tepe-goruntu bulutu duz bir katman -> PCA egimi anlamsiz (hep ~90). O durumda
    # durusu SADECE boyut siralamasindan kararlastir (lean=0).
    lean = 0.0 if table_ref is not None else _lean_from_points(pc_base.xyz, up_idx)
    posture = classify_obb(obb, lean_deg=lean)
    res.diagnostics["egim_derece"] = round(lean, 1)
    if posture_override in ("standing_on_wheels", "tipped_over"):
        from .pose_classify import PostureResult
        res.diagnostics["durus_otomatik"] = f"{posture.label} (guven {posture.confidence})"
        posture = PostureResult(
            label=posture_override,
            needs_robot=(posture_override == "tipped_over"),
            confidence=1.0,
            aspect_h_over_w=posture.aspect_h_over_w,
            vertical_rank=posture.vertical_rank,
            reasons=[f"OPERATOR ELLE '{posture_override}' dedi (otomatik: {posture.label}, "
                     f"egim {lean:.0f}°)"],
        )
        res.warnings.append(f"Durus elle ayarlandi: {posture_override}. "
                            "Otomatik siniflandirma ve OBB dikey ekseni yok sayildi.")
    res.posture = posture

    # --- 4b) OLCULER  (boy=en uzun, en=ortanca, derinlik=en kisa) ---
    _sd = sorted([obb.width_mm, obb.height_mm, obb.thickness_mm], reverse=True)
    dims = (f"boy {_sd[0]/10:.1f} · en {_sd[1]/10:.1f} · "
            f"derinlik {_sd[2]/10:.1f} cm  (yaw {obb.yaw_deg:.0f}°)")
    res.measurement_text = dims

    # --- 5) plan ---
    if posture.label == "standing_on_wheels":
        gp = plan_grasp(pc_base.xyz, obb=obb,
                        force_grasp_z_mm=grasp_z_mm,
                        force_thickness_mm=grasp_thickness_mm)
        res.grasp = gp
        res.program = build_pick_place(gp, place_pose=place_pose or config.PLACE_POSE)
        el = " (elle DIK)" if posture_override == "standing_on_wheels" else ""
        res.message = f"VALIZ · {dims} · DIK{el}. " + (
            "Kavrama plani hazir." if gp.feasible else f"KAVRANAMAZ: {gp.reason}")
        res.ok = gp.feasible

    elif posture.label == "tipped_over":
        tu = plan_tipup(pc_base.xyz, base_origin_xy=(0.0, 0.0), obb=obb,
                        force=(posture_override == "tipped_over"))
        res.tipup = tu
        res.program = build_tipup(tu)
        res.message = (f"VALIZ · {dims} · YATIK -> once dik konuma getirme. " +
                       ("Uygula, sonra TEKRAR calistir." if tu.feasible else f"({tu.reason})"))
        res.ok = tu.feasible

    else:
        res.ok = True   # olcum basarili; sadece otomatik hareket uretilmedi
        res.message = f"VALIZ · {dims} · durus BELIRSIZ - otomatik hareket uretilmedi."

    return res


def _load_base() -> BaseTransform:
    try:
        return BaseTransform.load()
    except FileNotFoundError:
        logger.warning("base_T_cam yok -> BIRIM matris. Bulut kamera cercevesinde kalir; "
                       "FR3 pozlari GECERSIZ. tools/handeye_solve.py calistirin.")
        return BaseTransform.identity()
