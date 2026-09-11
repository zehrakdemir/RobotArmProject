"""
COK NESNELI algilama - robot kolun onundeki masada birden fazla kutu/valiz.

detect_objects(L, R) -> [DetectedObject, ...]   (HACIMCE BUYUKTEN KUCUGE sirali)

Boru hatti (Hucre Modu ile ayni kalibrasyon):
  rektifiye + otomatik epipolar duzeltme -> disparite -> 3B (kamera cerc.) ->
  RANSAC masa duzlemi -> duzlem ustu noktalar -> 2B izgarada BAGLI BILESEN
  ile nesnelere ayir -> her nesne icin OBB (ayak izi = tepe dilimi,
  yukseklik = masa duzlemine mesafe) + kavrama plani.

base_T_cam varsa olculer/pozlar robot taban cercevesine cevrilir.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from . import config
from .calib import BaseTransform, StereoRig
from .cell import _epipolar_correct
from .grasp import GraspPlan, OBB, fit_obb, plan_grasp
from .pose_classify import classify_obb
from .stereo import compute_disparity_adaptive

logger = logging.getLogger(__name__)


@dataclass
class DetectedObject:
    rank: int                         # 0 = en buyuk
    center_mm: list                   # OBB merkezi (kamera ya da robot taban)
    dims_cm: tuple                    # (boy, en, derinlik) = buyukten kucuge
    volume_l: float
    yaw_deg: float
    n_points: int
    posture: str                     # standing_on_wheels | tipped_over | uncertain
    bbox_px: tuple = (0, 0, 0, 0)     # rektifiye SOL goruntude (x0,y0,x1,y1)
    contour: np.ndarray = field(repr=False, default=None)  # 2B segmentasyon konturu
    obb: OBB = field(repr=False, default=None)
    grasp: Optional[GraspPlan] = field(repr=False, default=None)

    def as_dict(self) -> dict:
        return {
            "sira": self.rank, "boy_cm": self.dims_cm[0], "en_cm": self.dims_cm[1],
            "derinlik_cm": self.dims_cm[2], "hacim_L": round(self.volume_l, 2),
            "yaw": round(self.yaw_deg, 1), "nokta": self.n_points,
            "durus": self.posture, "merkez_mm": [round(v, 1) for v in self.center_mm],
            "kavranabilir": bool(self.grasp and self.grasp.feasible),
            "kavrama_sebep": (self.grasp.reason if self.grasp else "plan yok"),
        }


def _to_gray(img):
    import cv2
    return img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def _plane_and_objects(xyz: np.ndarray):
    """RANSAC masa duzlemi + normali kameraya cevir. Doner (n, d, signed)."""
    from .segment import fit_plane_ransac
    sp = np.percentile(xyz, 97, 0) - np.percentile(xyz, 3, 0)
    thr = max(0.012 * float(np.linalg.norm(sp)), 4.0)
    n, d, inl = fit_plane_ransac(xyz, thr, iters=500)
    if not (0.12 <= inl.mean() <= 0.95):
        return None
    c = xyz[inl].mean(0)
    if float(n @ (-c)) < 0:
        n, d = -n, -d
    return n.astype(float), float(d), (xyz @ n + d)


def _split_axis(pts_xy: np.ndarray, sel: np.ndarray, bin_mm: float = 15.0):
    """Bir kumeyi SADECE cok genisse (>28cm) ve ortada NET, GENIS bir yogunluk
    vadisi varsa 2'ye bol (bitisik 2 kutu). Aksi halde dokunma - tek nesneyi
    parcalamamak icin muhafazakar."""
    q = pts_xy[sel]
    if len(q) < 500:
        return [sel]
    c = q - q.mean(0)
    w, v = np.linalg.eigh(np.cov(c.T))
    t = c @ v[:, int(np.argmax(w))]
    span = t.max() - t.min()
    if span < 280.0:                               # tek test kutusu bu kadar genis olmaz
        return [sel]
    nb = max(10, int(span / bin_mm))
    hist, edges = np.histogram(t, bins=nb)
    peak = float(hist.max())
    lo_i, hi_i = int(nb * 0.30), int(nb * 0.70)    # sadece orta ucte ara
    if hi_i - lo_i < 3:
        return [sel]
    seg = hist[lo_i:hi_i]
    vi = lo_i + int(np.argmin(seg))
    # NET vadi: min < %12 tepe VE komsu 2 bin de < %30 tepe (genis bosluk)
    lo2, hi2 = max(vi - 1, 0), min(vi + 2, nb)
    if hist[vi] > 0.12 * peak or hist[lo2:hi2].max() > 0.30 * peak:
        return [sel]
    cut = edges[vi + 1]
    idx = np.where(sel)[0]
    left = t < cut
    m1 = np.zeros(len(sel), bool); m1[idx[left]] = True
    m2 = np.zeros(len(sel), bool); m2[idx[~left]] = True
    return [m for m in (m1, m2) if int(m.sum()) >= 200] or [sel]


def segment_image_masks(bgr: np.ndarray, min_area_px: int = 2500):
    """2B GORUNTU segmentasyonu (nokta bulutundan bagimsiz, temiz):
    duz acik masadan sapan bolgeler -> nesne maskeleri. Doner: [bool (H,W), ...]"""
    import cv2
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    sat = hsv[:, :, 1]
    # masa = genis bulanik hali (yumusak, dokusuz). Nesne = ondan sapan.
    bg = cv2.GaussianBlur(g, (0, 0), 25)
    dev = cv2.absdiff(g, bg)
    grad = cv2.morphologyEx(cv2.Laplacian(cv2.GaussianBlur(g, (3, 3), 0), cv2.CV_16S,
                                          ksize=3).astype(np.int16), cv2.MORPH_CLOSE,
                            np.ones((3, 3), np.uint8))
    fg = ((dev > 12) | (sat > 55) | (np.abs(grad) > 26)).astype(np.uint8) * 255
    fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE,
                          cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (45, 45)))
    fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN,
                          cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
    h, w = fg.shape
    table_g = float(np.median(g))
    nlab, lab, stats, _ = cv2.connectedComponentsWithStats(fg, 8)
    out = []
    for L in range(1, nlab):
        x, y, ww, hh, area = stats[L]
        if area < min_area_px:
            continue
        if ww >= w - 4 and hh >= h - 4:             # neredeyse tum kare -> arka plan
            continue
        comp = (lab == L)
        # GOLGE ele: masadan koyu + renksiz + zayif kenar
        if (float(g[comp].mean()) < table_g - 6 and float(sat[comp].mean()) < 28
                and float(np.abs(grad[comp]).mean()) < 12):
            continue
        comp = comp.astype(np.uint8)
        cs, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cs:
            continue
        c = max(cs, key=cv2.contourArea)
        hull = cv2.convexHull(c)                    # kutu ~ disbukey -> temiz, deliksiz kenar
        filled = np.zeros((h, w), np.uint8)
        cv2.drawContours(filled, [hull], -1, 255, -1)
        out.append(filled > 0)
    return out


def _merge_adjacent(xy: np.ndarray, clusters: list, table_xy: Optional[np.ndarray] = None,
                    gap_mm: float = 35.0, cell_mm: float = 8.0):
    """Iki kumeyi SADECE aralarinda MASA gorunmuyorsa birlestir. Yumusak cisimde
    (sirt cantasi koyu deri cep/dikis) stereo bosluk olur ama arada masa yoktur
    -> birlestir. Bitisik 2 kutuda aralarinda masa dokusu vardir -> ayri birak.
    `table_xy`: masa duzlemi hizasindaki noktalarin (duzlem cercevesi) xy'si."""
    import cv2
    if len(clusters) < 2:
        return clusters
    mn = xy.min(0)
    W = int((xy[:, 0].max() - mn[0]) / cell_mm) + 3
    H = int((xy[:, 1].max() - mn[1]) / cell_mm) + 3
    if H * W > 6_000_000:
        return clusters
    def _ker(mm):
        r = max(1, int(round(mm / 2.0 / cell_mm)))
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1,) * 2)
    ker_far = _ker(gap_mm)          # gap_mm'ye kadar kopru
    ker_touch = _ker(16.0)         # ~bitisik (parcalanma)
    masks, dil_far, dil_touch, aspect = [], [], [], []
    for sel in clusters:
        q = xy[sel]
        ij = np.floor((q - mn) / cell_mm).astype(np.int64)
        ij = np.clip(ij, 0, [W - 3, H - 3])
        g = np.zeros((H, W), np.uint8)
        g[ij[:, 1] + 1, ij[:, 0] + 1] = 1
        masks.append(g)
        dil_far.append(cv2.dilate(g, ker_far))
        dil_touch.append(cv2.dilate(g, ker_touch))
        (_, (w1, w2), _) = cv2.minAreaRect(q.astype(np.float32))
        aspect.append(max(w1, w2) / max(min(w1, w2), 1.0))
    # sap/kayis/kapak gibi UZUN INCE parcalar (aspect > 2.3) bitisik oldugunda
    # her zaman anagovdeye katilir (sirt cantasi vb.); iki KOMPAKT govde
    # (gercek 2 ayri kutu) bitisik olsa bile bu yolla otomatik birlesmez -
    # asagidaki masa-kontrolune duser.
    not_boxy = [a > 2.3 for a in aspect]
    tgrid = np.zeros((H, W), np.uint8)
    if table_xy is not None and len(table_xy):
        tij = np.floor((table_xy - mn) / cell_mm).astype(np.int64)
        ok = np.all((tij >= 0) & (tij < [W - 2, H - 2]), axis=1)
        tij = tij[ok]
        np.add.at(tgrid, (tij[:, 1] + 1, tij[:, 0] + 1), 1)
    tgrid = (tgrid >= 2).astype(np.uint8)               # hafif gurultuyu ele

    parent = list(range(len(clusters)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]; a = parent[a]
        return a

    for i in range(len(clusters)):
        for j in range(i + 1, len(clusters)):
            touching = bool(np.any(dil_touch[i] & dil_touch[j]))
            if touching and (not_boxy[i] or not_boxy[j]):   # ~bitisik + sap/kayis gibi -> birlestir
                parent[find(i)] = find(j); continue
            corridor = dil_far[i] & dil_far[j]
            if not np.any(corridor):
                continue                                # cok uzaklar -> ayri
            corr = cv2.dilate(corridor, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
            if int((corr & tgrid).sum()) >= 3:          # arada masa gorunuyor -> ayri nesneler
                continue
            parent[find(i)] = find(j)
    groups: dict = {}
    for i in range(len(clusters)):
        groups.setdefault(find(i), []).append(i)
    out = []
    for members in groups.values():
        m = clusters[members[0]].copy()
        for k in members[1:]:
            m |= clusters[k]
        out.append(m)
    return out


def _cluster_2d(xy: np.ndarray, cell_mm: float = 10.0, close_px: int = 1,
                min_cell: int = 12):
    """Yatay duzlemde 2B doluluk izgarasi -> bagli bilesenler; buyuk bilesenleri
    yogunluk vadisinden BOL (bitisik kutular). Doner: maske listesi (xy uzerinde)."""
    import cv2
    from scipy import ndimage
    ij = np.floor((xy - xy.min(0)) / cell_mm).astype(np.int64)
    H = int(ij[:, 1].max()) + 3
    W = int(ij[:, 0].max()) + 3
    if H * W > 4_000_000:
        return [np.ones(len(xy), bool)]
    grid = np.zeros((H, W), np.uint8)
    np.add.at(grid, (ij[:, 1] + 1, ij[:, 0] + 1), 1)
    occ = (grid >= 1).astype(np.uint8)
    if close_px > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * close_px + 1,) * 2)
        occ = cv2.morphologyEx(occ, cv2.MORPH_CLOSE, k)
    lab, num = ndimage.label(occ, structure=np.ones((3, 3)))
    out = []
    for L in range(1, num + 1):
        cells = (lab == L)
        if int((grid * cells).sum()) < min_cell:
            continue
        sel = cells[ij[:, 1] + 1, ij[:, 0] + 1]
        if int(sel.sum()) < 120:
            continue
        out.extend(_split_axis(xy, sel))
    return out


def _backproject(px_xy: np.ndarray, K: np.ndarray, n: np.ndarray, d: float):
    """Piksel isinlari x duzlem (n.X + d = 0) kesisimi. Doner: (M,3).
    NOT: bu rig'de kamera onu -Z (rig.reproject ile ayni yon) - isin z=-1."""
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    dirs = np.stack([(px_xy[:, 0] - cx) / fx, (px_xy[:, 1] - cy) / fy,
                     -np.ones(len(px_xy))], 1)
    den = dirs @ n
    ok = np.abs(den) > 1e-6
    t = -d / den[ok]
    XYZ = dirs[ok] * t[:, None]
    return XYZ[(t > 80) & (t < 4000)]


def _obb_from_mask(mask: np.ndarray, K: np.ndarray, plane, pts_above: np.ndarray,
                   up_idx: int):
    """OLCU (yogun + kararli):
      yukseklik H  = maske icindeki seyrek 3B noktalarin masa duzlemine p95 mesafesi
      ayak izi     = maskenin UST diliminin, H kadar oteye kaydirilmis duzleme
                     geri-izdusumu -> minAreaRect  (ayakta kutuda ust yuz = taban;
                     yatik kutuda zaten tum ust yuzey).
    Doner: OBB (kamera cerc.) ya da None."""
    import cv2
    n, d = plane
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    mask = cv2.dilate(mask.astype(np.uint8),
                      cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
    ys, xs = np.nonzero(mask)
    if len(xs) < 200:
        return None
    u = np.cross(n, np.array([0.0, 0.0, 1.0]))
    if np.linalg.norm(u) < 1e-6:
        u = np.cross(n, np.array([0.0, 1.0, 0.0]))
    u /= np.linalg.norm(u)
    v = np.cross(n, u); v /= np.linalg.norm(v)
    ylo, yhi = ys.min(), ys.max()

    # golge merkezi (tum maske -> masa duzlemi) ~ nesne merkezi
    XYZ_all = _backproject(np.stack([xs, ys], 1).astype(float), K, n, d)
    if len(XYZ_all) < 80:
        return None
    f2d_all = np.stack([XYZ_all @ u, XYZ_all @ v], 1).astype(np.float32)
    (cu, cv_), _, _ = cv2.minAreaRect(f2d_all)

    # YUKSEKLIK: seyrek 3B p92 + maske-tepe isini golge-merkezi ekseninde kesistir
    H = 25.0
    if pts_above is not None and len(pts_above) >= 40:
        H = float(np.percentile(np.abs(pts_above @ n + d), 92))
    topb = ys <= ylo + 3
    if int(topb.sum()) >= 6:
        tx, ty = xs[topb].astype(float), ys[topb].astype(float)
        dirs = np.stack([(tx - cx) / fx, (ty - cy) / fy, -np.ones_like(tx)], 1)
        du, dv = dirs @ u, dirs @ v
        s = (cu * du + cv_ * dv) / np.maximum(du * du + dv * dv, 1e-9)
        Xtop = dirs * s[:, None]
        H_ray = float(np.median(np.abs(Xtop @ n + d)))
        if 15.0 < H_ray < 800.0:
            H = max(H, H_ray)
    H = float(np.clip(H, 12.0, 800.0))

    # AYAK IZI: maskenin ust dilimini H kadar kaydirilmis duzleme izdusum
    band = ys <= ylo + 0.35 * (yhi - ylo)
    if int(band.sum()) < 120:
        band = ys <= ylo + 0.65 * (yhi - ylo)
    XYZ = _backproject(np.stack([xs[band], ys[band]], 1).astype(float), K, n, d - H)
    if len(XYZ) < 80:
        XYZ = XYZ_all
    f2d = np.stack([XYZ @ u, XYZ @ v], 1).astype(np.float32)
    (cu, cv_), (w1, w2), ang = cv2.minAreaRect(f2d)
    long_mm, short_mm = max(w1, w2), min(w1, w2)
    if long_mm < 20 or short_mm < 8:
        return None
    base_c = float(cu) * u + float(cv_) * v
    base_c = base_c - (base_c @ n + (d - H)) * n / (n @ n)        # ust yuz duzlemine oturt
    center = base_c - n * (H / 2.0)                              # n kameraya dogru -> taban -n yonu
    th = np.radians(ang)
    long3 = np.cos(th) * u + np.sin(th) * v
    short3 = -np.sin(th) * u + np.cos(th) * v
    long3 /= np.linalg.norm(long3); short3 /= np.linalg.norm(short3)
    up3 = np.zeros(3); up3[up_idx] = 1.0
    yaw = float(np.degrees(np.arctan2(long3[1], long3[0])))
    return OBB(center=center, axes=np.vstack([long3, short3, up3]),
              extents=np.array([long_mm, short_mm, H], float),
              ground_z=float(center[up_idx] - H / 2.0), yaw_deg=yaw)


def _obj_obb(pts: np.ndarray, plane, up_idx: int):
    """Tek nesne noktalarindan OBB: ayak izi = tepe %25 dilimi minAreaRect,
    yukseklik = masa duzlemine p98 mesafe."""
    import cv2
    n, d = plane
    s = pts @ n + d
    H = float(np.percentile(s, 98))
    if H < 8:
        return None
    hz = [i for i in range(3) if i != up_idx]
    sel = s >= 0.75 * H
    if sel.sum() < 60:
        sel = s >= 0.5 * H
    xy = pts[sel][:, hz]
    lo, hi = np.percentile(xy, [1.5, 98.5], axis=0)
    keep = np.all((xy >= lo) & (xy <= hi), axis=1)
    xy = xy[keep] if keep.sum() >= 25 else xy
    (cx, cy), (w1, w2), ang = cv2.minAreaRect(xy.astype(np.float32))
    long_mm, short_mm = max(w1, w2), min(w1, w2)
    if long_mm < 10 or short_mm < 4:
        return None
    gz = float(np.percentile(pts[:, up_idx], 2))
    center = np.zeros(3)
    center[hz[0]] = float(cx); center[hz[1]] = float(cy)
    center[up_idx] = gz + H / 2.0
    th = np.radians(ang)
    long3 = np.zeros(3); long3[hz[0]] = np.cos(th); long3[hz[1]] = np.sin(th)
    short3 = np.zeros(3); short3[hz[0]] = -np.sin(th); short3[hz[1]] = np.cos(th)
    up3 = np.zeros(3); up3[up_idx] = 1.0
    yaw = float(np.degrees(np.arctan2(long3[1], long3[0])))
    return OBB(center=center, axes=np.vstack([long3, short3, up3]),
              extents=np.array([long_mm, short_mm, H], float), ground_z=gz, yaw_deg=yaw)


# ----------------------------------------------------------------------
def _objects_from_masks(img_masks, P, PX, n, d, plane, K, W, up_idx,
                        have_he, base, min_footprint_cm, max_objects):
    """2B nesne maskeleri (jenerik segmentasyon YA DA YOLO26-seg) -> [DetectedObject].
    Her maske: masa duzlemine geri-izdusum ile ayak izi + seyrek 3B'den yukseklik."""
    import cv2
    lin = PX[:, 1].astype(np.int64) * W + PX[:, 0].astype(np.int64)
    objs: list[DetectedObject] = []
    dropped = []
    for mk in img_masks:
        mk = np.asarray(mk, bool)
        inside = mk.reshape(-1)[lin]                        # bu maskedeki 3B noktalar
        pts_in = P[inside]
        s_in = pts_in @ n + d if len(pts_in) else np.array([])
        pts_o = pts_in[(s_in > 12.0) & (s_in < 900.0)] if len(pts_in) else pts_in
        if len(pts_o) < 150:                                # 3B dogrulama YOK -> guvenilmez
            dropped.append(f"maske {int(mk.sum())}px -> 3B nokta {len(pts_o)} (<150) "
                           "OLCULEMEZ (isik/doku)"); continue
        obb = _obb_from_mask(mk, K, plane, pts_o, up_idx)
        if obb is None:
            dropped.append(f"maske {int(mk.sum())}px -> olcu uretilemedi"); continue
        dims = sorted([obb.width_mm, obb.height_mm, obb.thickness_mm], reverse=True)
        if dims[1] / 10.0 < min_footprint_cm:
            dropped.append(f"{tuple(round(v/10,1) for v in dims)} orta<{min_footprint_cm}"); continue
        obb_r = obb
        if have_he:
            R, t = base.T[:3, :3], base.T[:3, 3]
            obb_r = OBB(center=R @ obb.center + t, axes=(R @ obb.axes.T).T,
                        extents=obb.extents.copy(), ground_z=config.GROUND_Z_MM,
                        yaw_deg=obb.yaw_deg)
            pts_r = (R @ pts_o.T).T + t
        else:
            pts_r = pts_o
        posture = classify_obb(obb_r, lean_deg=0.0).label
        if pts_r is None or len(pts_r) < 400:              # seyrek -> OBB hacminden ornek
            rng = np.random.default_rng(0)
            g = rng.uniform(-0.5, 0.5, (1200, 3)) * obb_r.extents
            pts_r = (g @ obb_r.axes) + obb_r.center
        gp = plan_grasp(pts_r, obb=obb_r)
        vol_l = float(np.prod(dims)) / 1e6
        cnts, _ = cv2.findContours(mk.astype(np.uint8), cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        cnt = max(cnts, key=cv2.contourArea) if cnts else None
        x, y, ww, hh = cv2.boundingRect(cnt) if cnt is not None else (0, 0, 0, 0)
        objs.append(DetectedObject(
            rank=0, center_mm=[float(v) for v in obb_r.center],
            dims_cm=(round(dims[0] / 10, 1), round(dims[1] / 10, 1), round(dims[2] / 10, 1)),
            volume_l=vol_l, yaw_deg=obb_r.yaw_deg, n_points=len(pts_o),
            posture=posture, bbox_px=(x, y, x + ww, y + hh), contour=cnt,
            obb=obb_r, grasp=gp))
    if dropped:
        logger.info("elenen maske: %s", "; ".join(dropped))
    objs.sort(key=lambda o: o.volume_l, reverse=True)      # EN BUYUK once
    for i, o in enumerate(objs[:max_objects]):
        o.rank = i
    return objs[:max_objects]


_SUITCASE_DET = None


def _get_suitcase_detector():
    """YOLO26-seg valiz dedektoru - process basina bir kez yuklenir."""
    global _SUITCASE_DET
    if _SUITCASE_DET is None:
        from .detector import SuitcaseDetector
        _SUITCASE_DET = SuitcaseDetector()
    return _SUITCASE_DET


def detect_objects(image_left: np.ndarray, image_right: np.ndarray,
                   rig: Optional[StereoRig] = None,
                   base_T_cam: Optional[BaseTransform] = None,
                   min_footprint_cm: float = 2.8,
                   max_objects: int = 10,
                   frames: Optional[list] = None,
                   mode: Optional[str] = None,
                   diag: Optional[dict] = None) -> list[DetectedObject]:
    """frames verilirse (sabit sahnenin birden fazla L/R cifti) bulutlar
    BIRLESTIRILIR -> seyrek/kararsiz olcumler stabil hale gelir. Yoksa tek cift.

    mode:
      "nesne" (varsayilan) -> geometrik: masa duzlemi + 2B kumeleme; her cisim.
      "valiz"              -> YOLO26-seg maskeleri (sadece 'suitcase'); olcu ayni
                              maske->duzlem izdusum + seyrek-3B yukseklik yolundan.
      None -> config.DETECT_MODE

    diag: bos bir dict verilirse (mod FARK ETMEKSIZIN) icine gorsellestirme
      verisi doldurulur: "rektifiye_sol" (H,W,3 BGR), "disparite" (H,W float),
      "derinlik_mm" (H,W float, Z kanali, NaN=gecersiz), "bulut_mm" (N,3),
      "bulut_rgb" (N,3 uint8 veya None). Nesne tespitini etkilemez.
    """
    mode = (mode or getattr(config, "DETECT_MODE", "nesne") or "nesne").strip().lower()
    rig = rig or StereoRig.load()
    try:
        base = base_T_cam or BaseTransform.load()
    except FileNotFoundError:
        base = BaseTransform.identity()
    have_he = not np.allclose(base.T, np.eye(4))
    up_idx = {"x": 0, "y": 1, "z": 2}[config.UP_AXIS.lower()]

    pairs = frames if frames else [(image_left, image_right)]
    parts, px_parts = [], []
    epi_dy = 0.0; epi_ok = False
    lr_last = None
    for (pi, (fl, fr)) in enumerate(pairs):
        lr, rr = rig.rectify(fl, fr)
        rr, epi_dy, epi_ok = _epipolar_correct(lr, rr)
        disp = compute_disparity_adaptive(_to_gray(lr), _to_gray(rr), roi_mask=None)
        pts = rig.reproject(disp.disparity)
        fin = np.isfinite(pts).all(2) & (disp.disparity > 0)
        # --- kameralar kaymissa (gecerli disparite cok dusuk) bir kez oz-kalibre et
        if pi == 0:
            hh, ww = fin.shape
            roi = fin[int(hh*0.2):int(hh*0.8), int(ww*0.2):int(ww*0.8)]
            vfrac = float(roi.mean())
            try:
                from .selfcalib import maybe_auto_refresh
                new_rig = maybe_auto_refresh(fl, fr, rig, vfrac)
            except Exception:  # noqa: BLE001
                new_rig = None
            if new_rig is not None:
                rig = new_rig
                lr, rr = rig.rectify(fl, fr)
                rr, epi_dy, epi_ok = _epipolar_correct(lr, rr)
                disp = compute_disparity_adaptive(_to_gray(lr), _to_gray(rr), roi_mask=None)
                pts = rig.reproject(disp.disparity)
                fin = np.isfinite(pts).all(2) & (disp.disparity > 0)
        lr_last = lr
        parts.append(pts[fin])
        yy, xx = np.nonzero(fin)
        px_parts.append(np.column_stack([xx, yy]).astype(np.int32))
    P = np.concatenate(parts, axis=0) if len(parts) > 1 else parts[0]
    PX = np.concatenate(px_parts, axis=0) if len(px_parts) > 1 else px_parts[0]

    if diag is not None:
        depth_full = np.where(fin, pts[:, :, 2], np.nan).astype(np.float32)
        diag.update(rektifiye_sol=lr_last, disparite=disp.disparity.astype(np.float32),
                    derinlik_mm=depth_full)

    if len(P) < 3000:
        logger.warning("cok az 3B nokta (%d) - isik/doku?", len(P))
        return []

    pl = _plane_and_objects(P)
    if pl is None:
        logger.warning("masa duzlemi bulunamadi")
        return []
    n, d, signed = pl
    plane = (n, d)
    H, W = lr_last.shape[:2]
    K = np.asarray(rig.P1[:3, :3], float)
    import cv2

    above = (signed > 20.0) & (signed < 900.0)     # 20mm: dokulu sayfa kivrilmasini gec
    Pa, PXa = P[above], PX[above]
    P_table = P[(signed > -25.0) & (signed < 15.0)]  # masa hizasi -> kumeler arasi "masa var mi"

    if diag is not None:
        # GORSEL bulut: SADECE masa ustundeki (nesne) noktalar - zemin/masa disari
        # birakilir, gorselde net "burada nesne var" gorunsun.
        cloud_xyz, cloud_rgb = Pa, None
        try:
            cloud_rgb = lr_last[PXa[:, 1], PXa[:, 0]][:, ::-1].astype(np.uint8)  # BGR->RGB
        except Exception:  # noqa: BLE001
            pass
        cap = int(getattr(config, "POINTCLOUD_MAX_POINTS", 60000))
        if cap > 0 and len(cloud_xyz) > cap:
            csel = np.random.default_rng(0).choice(len(cloud_xyz), size=cap, replace=False)
            cloud_xyz = cloud_xyz[csel]
            cloud_rgb = cloud_rgb[csel] if cloud_rgb is not None else None
        diag.update(bulut_mm=cloud_xyz, bulut_rgb=cloud_rgb)

    # ================= VALIZ MODU: YOLO26-seg maskeleri (YOL A atlanir) ============
    if mode == "valiz":
        try:
            dets = _get_suitcase_detector().detect_all(lr_last)
        except Exception as exc:  # noqa: BLE001
            logger.warning("valiz modu: YOLO tespiti basarisiz (%s)", exc)
            return []
        if not dets:
            logger.info("valiz modu: goruntude valiz yok")
            return []
        objs = _objects_from_masks([dd.mask for dd in dets], P, PX, n, d, plane, K,
                                   W, up_idx, have_he, base, min_footprint_cm, max_objects)
        logger.info("%d valiz [YOLO26-seg %s] (epipolar %+.1fpx)", len(objs),
                    dets[0].weights_used, epi_dy)
        return objs

    # ================= YOL A: DOLU BULUT (dokulu masa) -> 3B kumeleme =================
    if len(Pa) >= 2500:
        objs = _detect_3d(Pa, PXa, n, d, K, up_idx, have_he, base, lr_last,
                          min_footprint_cm, max_objects, p_table=P_table)
        if objs:
            logger.info("%d nesne [3B yol] (epipolar %+.1fpx)", len(objs), epi_dy)
            return objs
        logger.info("3B yol nesne bulamadi -> 2B segmentasyona dusuluyor")

    # ================= YOL B: SEYREK BULUT -> 2B goruntu segmentasyonu ================
    img_masks = segment_image_masks(lr_last)
    if not img_masks:
        logger.warning("goruntude nesne bolgesi yok")
        return []
    objs = _objects_from_masks(img_masks, P, PX, n, d, plane, K, W, up_idx,
                               have_he, base, min_footprint_cm, max_objects)
    logger.info("%d nesne [2B segmentasyon] (epipolar %+.1fpx %s)", len(objs), epi_dy,
                "uygulandi" if epi_ok else "yok")
    return objs


def _cuboid_from_silhouette(px_xy: np.ndarray, img_hw, K, n, d):
    """Ayakta kutu: 3B bulutta yuksekligi olmayabilir. Silueti kullan:
      taban dikdortgeni = maskenin ALT konturunun masa duzlemine izdusumu
      yukseklik H = maske TEPE isini ile taban-uzak-kosesi (bfc + H*n) kesisimi
    Doner: (W_mm, D_mm, H_mm, yaw_deg, center3d) ya da None."""
    import cv2
    Him, Wim = img_hw
    m = np.zeros((Him, Wim), np.uint8)
    p = np.clip(px_xy, [0, 0], [Wim - 1, Him - 1]).astype(np.int32)
    m[p[:, 1], p[:, 0]] = 255
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE,
                         cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25)))
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not cs:
        return None
    c = max(cs, key=cv2.contourArea).reshape(-1, 2).astype(float)
    if len(c) < 30:
        return None
    fx, fy, cx0, cy0 = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    u = np.cross(n, np.array([0.0, 0.0, 1.0]))
    if np.linalg.norm(u) < 1e-6:
        u = np.cross(n, np.array([0.0, 1.0, 0.0]))
    u /= np.linalg.norm(u); v = np.cross(n, u); v /= np.linalg.norm(v)

    def bp(pix, dd):                                   # piksel -> duzlem (n.X+dd=0)
        dr = np.stack([(pix[:, 0] - cx0) / fx, (pix[:, 1] - cy0) / fy,
                       np.ones(len(pix))], 1)
        den = dr @ n; ok = np.abs(den) > 1e-6
        t = -dd / den[ok]
        X = dr[ok] * t[:, None]
        return X[(t > 60) & (t < 5000)]

    ylo, yhi = c[:, 1].min(), c[:, 1].max()
    botc = c[c[:, 1] >= yhi - 0.20 * (yhi - ylo)]      # alt kontur -> taban
    if len(botc) < 8:
        return None
    B = bp(botc, d)
    if len(B) < 8:
        return None
    b2 = np.stack([B @ u, B @ v], 1).astype(np.float32)
    # tam taban dikdortgeni: alt kontur + kutle merkezi birlikte
    allB = bp(c, d)
    a2 = np.stack([allB @ u, allB @ v], 1).astype(np.float32)
    (rcu, rcv), (w1, w2), ang = cv2.minAreaRect(np.vstack([b2, a2]))
    # golge -> egim yonunde H*tan sisik; alt kontur daha guvenilir taban verir:
    (rcu, rcv), (bw1, bw2), ang = cv2.minAreaRect(b2)
    box2d = cv2.boxPoints(((rcu, rcv), (bw1, bw2), ang))
    corners3d = np.array([cc[0] * u + cc[1] * v - d * n / (n @ n) for cc in box2d])
    corners3d = corners3d - ((corners3d @ n + d)[:, None]) * n     # duzleme otur
    far = corners3d[np.argmax(np.linalg.norm(corners3d, axis=1))]  # kameradan en uzak kose
    # yukseklik: tepe isinlari
    topc = c[c[:, 1] <= ylo + 0.06 * (yhi - ylo) + 2]
    if len(topc) < 4:
        return None
    dr = np.stack([(topc[:, 0] - cx0) / fx, (topc[:, 1] - cy0) / fy,
                   np.ones(len(topc))], 1)
    Hs = []
    for di in dr:
        A = np.stack([di, -n], 1)                      # [s, H] -> A x = far
        x, *_ = np.linalg.lstsq(A, far, rcond=None)
        if x[1] > 0:
            Hs.append(x[1])
    if not Hs:
        return None
    H = float(np.median(Hs))
    W_mm, D_mm = max(bw1, bw2), min(bw1, bw2)
    center = far - 0.5 * (W_mm * (box2d[1] - box2d[0]) / (np.linalg.norm(box2d[1] - box2d[0]) + 1e-9))[0] * u  # kaba
    center = corners3d.mean(0) + n * (H / 2.0)
    yaw = float(np.degrees(np.arctan2((box2d[1] - box2d[0])[1], (box2d[1] - box2d[0])[0])))
    return W_mm, D_mm, H, yaw, center


def _R_to_z(nrm: np.ndarray) -> np.ndarray:
    """nrm -> +z'ye goturen donme matrisi."""
    z = np.array([0.0, 0.0, 1.0])
    a = np.cross(nrm, z); na = np.linalg.norm(a)
    if na < 1e-9:
        return np.eye(3) if nrm[2] > 0 else np.diag([1.0, -1.0, -1.0])
    a /= na
    ang = np.arccos(np.clip(nrm @ z, -1, 1))
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K


def _detect_3d(Pa, PXa, n, d, K, up_idx, have_he, base, lr_last,
               min_footprint_cm, max_objects, p_table=None):
    """Dolu bulut: masa-hizali cerceve -> 2B izgara kumeleme -> her kume icin
    fit_obb; ayakta kutuda (bulut tepesi eksik) SILUET kuboid ile yuksekligi
    tamamla. Doner: [DetectedObject]."""
    import cv2
    from .grasp import fit_obb
    from .segment import statistical_outlier_filter
    Rz = _R_to_z(n)
    Pz = (Rz @ Pa.T).T                          # +z = masa normali
    gz = float(np.percentile(Pz[:, 2], 2))
    tbl_xy = None
    if p_table is not None and len(p_table):
        tbl_xy = (Rz @ np.asarray(p_table).T).T[:, :2]
    clusters = _cluster_2d(Pz[:, :2])
    clusters = _merge_adjacent(Pz[:, :2], clusters, tbl_xy,
                               gap_mm=float(getattr(config, "CLUSTER_MERGE_GAP_MM", 35.0)))
    objs = []
    for sel in clusters:
        if int(sel.sum()) < 250:
            continue
        idx = np.where(sel)[0]
        # istatistiksel aykiri temizligi -> saçilan tekil noktalari at (sel'i de guncelle)
        inl = statistical_outlier_filter(Pz[sel], k=18, std_ratio=2.5)
        if int(inl.sum()) < 200:
            continue
        sel = np.zeros(len(sel), bool)
        sel[idx[inl]] = True
        q = Pz[sel]
        obb_z = fit_obb(q, ground_z=gz, up_axis="z")
        yaw = obb_z.yaw_deg
        cen_cam = Rz.T @ obb_z.center
        note = "bulut-obb"
        # AYAK IZI: kume PIKSELLERINI (yogun) masa duzlemine geri-izdusum -> minAreaRect
        # YUKSEKLIK: kume noktalarinin masa duzlemine mesafesi (p95) -> yatik nesnede
        # kalinlik, dik nesnede boy; kume z-uzanimindan cok daha guvenilir.
        s_cl = np.abs(Pa[sel] @ n + d)
        # kalinlik = ust yuzey bandinin ortasi; p75 tepe gurultusunu ve hafif egimi dislar
        Hc = float(np.percentile(s_cl, 75)) * float(getattr(config, "MEASURE_SCALE", 1.0))
        Wm = Dm = None
        try:
            # kume PIKSEL maskesini kur, kenar fringe'ini (masa/golge sizmasi) EROZYONLA at
            H0, W0 = lr_last.shape[:2]
            mkc = np.zeros((H0, W0), np.uint8)
            pc0 = np.clip(PXa[sel], [0, 0], [W0 - 1, H0 - 1])
            mkc[pc0[:, 1], pc0[:, 0]] = 255
            mkc = cv2.morphologyEx(mkc, cv2.MORPH_CLOSE,
                                   cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11)))
            mkc = cv2.erode(mkc, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (17, 17)))
            ys2, xs2 = np.nonzero(mkc)
            if len(xs2) < 200:
                ys2, xs2 = PXa[sel][:, 1], PXa[sel][:, 0]
            pxc = np.stack([xs2, ys2], 1).astype(float)
            u = np.cross(n, [0.0, 0.0, 1.0]); u = u / (np.linalg.norm(u) + 1e-9)
            vv = np.cross(n, u); vv = vv / (np.linalg.norm(vv) + 1e-9)
            fx, fy, cx0, cy0 = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
            # NOT: bu rig'de kamera onu -Z (rig.reproject ile ayni yon) - isin z=-1
            # olmali, aksi halde t hep negatif cikip (t>60) filtresi HER SEYI eler
            # ve sessizce obb_z (daha az hassas) fallback'ine duser.
            dr = np.stack([(pxc[:, 0] - cx0) / fx, (pxc[:, 1] - cy0) / fy,
                           -np.ones(len(pxc))], 1)
            den = dr @ n; m2 = np.abs(den) > 1e-6
            t = -(d - Hc * 0.5) / den[m2]              # nesne ORTA yuksekligi duzlemi
            X = dr[m2] * t[:, None]
            X = X[(t > 60) & (t < 5000)]
            if getattr(config, "DEBUG_FOOTPRINT", False):
                logger.warning("TESHIS ayak-izi: eroded_px=%d projected=%d", len(xs2), len(X))
            if len(X) >= 100:
                f2 = np.stack([X @ u, X @ vv], 1).astype(np.float32)
                lo, hi = np.percentile(f2, [1, 99], axis=0)
                f2 = f2[np.all((f2 >= lo) & (f2 <= hi), axis=1)]
                (_, _), (w1, w2), ang = cv2.minAreaRect(f2)
                sc = float(getattr(config, "MEASURE_SCALE", 1.0))
                Wm, Dm, yaw = max(w1, w2) * sc, min(w1, w2) * sc, float(ang)
        except Exception:  # noqa: BLE001
            pass
        if Wm is None:
            Wm, Dm = obb_z.width_mm, obb_z.thickness_mm
        else:
            note = "maske-izdusum + duzlem-mesafe"
        W, Dp = Wm, Dm
        dims = sorted([W, Dp, Hc], reverse=True)
        if dims[1] / 10.0 < min_footprint_cm or dims[0] / max(dims[1], 1.0) > 6.0:
            continue

        obb_cam = OBB(center=cen_cam, axes=(Rz.T @ obb_z.axes.T).T,
                      extents=np.array([W, Dp, Hc]),
                      ground_z=float(cen_cam @ n + d - Hc / 2.0), yaw_deg=yaw)
        obb_r, pts_r = obb_cam, Pa[sel]
        if have_he:
            R, t = base.T[:3, :3], base.T[:3, 3]
            obb_r = OBB(center=R @ cen_cam + t, axes=(R @ obb_cam.axes.T).T,
                        extents=obb_cam.extents.copy(), ground_z=config.GROUND_Z_MM,
                        yaw_deg=yaw)
            pts_r = (R @ Pa[sel].T).T + t
        posture = classify_obb(obb_r, lean_deg=0.0).label
        gp = plan_grasp(pts_r, obb=obb_r)
        # SEGMENTASYON KONTURU: kume pikselleri -> uzak aykirilari kirp -> disbukey kabuk
        px = PXa[sel]
        cnt = None
        if len(px) >= 8:
            c2 = np.median(px, axis=0)
            rr = np.linalg.norm(px.astype(float) - c2, axis=1)
            good = rr < (np.percentile(rr, 95) * 1.3)
            if int(good.sum()) >= 8:
                px = px[good]
            cnt = cv2.convexHull(px.reshape(-1, 1, 2).astype(np.int32))
        x, y, ww, hh = cv2.boundingRect(cnt) if cnt is not None else (0, 0, 0, 0)
        logger.info("  kume: %s -> %.1f x %.1f x %.1f cm",
                    note, dims[0] / 10, dims[1] / 10, dims[2] / 10)
        objs.append(DetectedObject(
            rank=0, center_mm=[float(v) for v in obb_r.center],
            dims_cm=(round(dims[0] / 10, 1), round(dims[1] / 10, 1), round(dims[2] / 10, 1)),
            volume_l=float(np.prod(dims)) / 1e6, yaw_deg=obb_r.yaw_deg,
            n_points=len(q), posture=posture, bbox_px=(x, y, x + ww, y + hh),
            contour=cnt, obb=obb_r, grasp=gp))
    objs.sort(key=lambda o: o.volume_l, reverse=True)
    for i, o in enumerate(objs[:max_objects]):
        o.rank = i
    return objs[:max_objects]


def draw_overlay(image_left: np.ndarray, objects: list, rig: Optional[StereoRig] = None,
                 pick_index: int = 0) -> np.ndarray:
    """Rektifiye SOL goruntu + her nesnenin SEGMENTASYON maskesi (gercek sekil):
    yari-saydam renkli dolgu + kontur + etiket. pick_index yesil, digerleri turuncu."""
    import cv2
    rig = rig or StereoRig.load()
    lr, _ = rig.rectify(image_left, image_left)
    img = lr.copy()
    H, W = img.shape[:2]
    for o in objects:
        c = o.contour
        if c is None or cv2.contourArea(c) < 400:
            continue
        nxt = (o.rank == pick_index)
        col = (0, 210, 0) if nxt else (0, 150, 255)
        fill = img.copy()
        cv2.drawContours(fill, [c], -1, col, -1)
        cv2.addWeighted(fill, 0.35, img, 0.65, 0, img)
        cv2.drawContours(img, [c], -1, col, 2)
        x, y, w, h = cv2.boundingRect(c)
        tag = (f"#{o.rank+1} {o.dims_cm[0]}x{o.dims_cm[1]}x{o.dims_cm[2]}cm {o.volume_l:.1f}L"
               + ("  SIRADAKI" if nxt else ""))
        yy = max(y - 6, 14)
        cv2.rectangle(img, (x, yy - 14), (x + 8 * len(tag), yy + 4), col, -1)
        cv2.putText(img, tag, (x + 2, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (15, 15, 15), 1, cv2.LINE_AA)
    if not objects:
        cv2.putText(img, "nesne yok", (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (0, 0, 255), 2, cv2.LINE_AA)
    return img
