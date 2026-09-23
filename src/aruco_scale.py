"""
ArUco isaretiyle canli, otomatik olcek dogrulama/duzeltme (DENEYSEL).

Sorun: stereo (iki kamera) sistemler mutlak olcegi (1 piksel gercekte kac mm)
GORUNTUDEN TEK BASINA cikaramaz - "olcek belirsizligi" (bkz. proje notlari).
Su anki cozum (MEASURE_SCALE, .env) TEK SEFERLIK: bilinen boyutlu bir cisim
elle olculup hesaplaniyor (tools/calib_scale.py) - kameralar kayarsa/degisirse
elle tekrarlanmasi gerekiyor.

Bu modul, boyutu KESIN bilinen (yazdirilip kumpasla dogrulanmis) bir ArUco
isaretini TEK KAMERADAN (stereo'ya bile gerek olmadan) solvePnP ile mm
cinsinden olcuyor - bu olcum PnP'nin dogasi geregi MUTLAK'tir (belirsizlik
yoktur, cunku isaretin gercek boyutu zaten taninmis, tipki gozun bildigi
boyuttaki bir cismin uzakligini tahmin etmesi gibi). Ayni isaretin stereo/
disparite ile hesaplanan mesafesiyle karsilastirilinca, o anki gercek
MEASURE_SCALE degeri HER OLCUMDE otomatik/canli cikarilabilir.

KULLANIM:
    python -m tools.print_aruco --id 7 --size-mm 50 --out marker.png
    (tam boyutunda yazdirin, kumpasla dogrulayin, masaya sabitleyin)

    python -m tools.aruco_scale_check --left snaps/left_x.png --right snaps/right_x.png \\
        --marker-id 7 --marker-size-mm 50
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

DEFAULT_DICT = "DICT_4X4_50"


def _to_gray(img: np.ndarray) -> np.ndarray:
    import cv2
    return img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def _get_dictionary(name: str = DEFAULT_DICT):
    import cv2.aruco as aruco
    return aruco.getPredefinedDictionary(getattr(aruco, name))


def _get_detector(dict_name: str = DEFAULT_DICT):
    import cv2.aruco as aruco
    params = aruco.DetectorParameters()
    return aruco.ArucoDetector(_get_dictionary(dict_name), params)


@dataclass
class MarkerDetection:
    marker_id: int
    corners_px: np.ndarray   # (4,2) float32 - siralama: sol-ust, sag-ust, sag-alt, sol-alt


def detect_markers(image_bgr: np.ndarray, dict_name: str = DEFAULT_DICT
                    ) -> list[MarkerDetection]:
    """Goruntudeki TUM ArUco isaretlerini bulur. Bos liste = hicbiri gorunmuyor."""
    detector = _get_detector(dict_name)
    corners, ids, _rejected = detector.detectMarkers(_to_gray(image_bgr))
    if ids is None:
        return []
    return [MarkerDetection(marker_id=int(i), corners_px=c.reshape(4, 2).astype(np.float32))
            for c, i in zip(corners, ids.ravel())]


def find_marker(image_bgr: np.ndarray, marker_id: Optional[int] = None,
                 dict_name: str = DEFAULT_DICT) -> Optional[MarkerDetection]:
    """marker_id verilirse sadece onu, verilmezse ILK bulunani doner."""
    dets = detect_markers(image_bgr, dict_name)
    if not dets:
        return None
    if marker_id is None:
        return dets[0]
    for d in dets:
        if d.marker_id == marker_id:
            return d
    return None


def marker_pose_mm(corners_px: np.ndarray, marker_size_mm: float,
                    K: np.ndarray, dist: Optional[np.ndarray] = None
                    ) -> tuple[np.ndarray, np.ndarray]:
    """Isaretin KENDI GERCEK boyutunu (marker_size_mm) ve kameranin ic
    parametrelerini (K, dist) kullanarak, isaretin kameraya gore 3B pozunu
    (rvec, tvec - mm) TEK GORUNTUDEN cikarir (solvePnP, kare-duzlem icin
    ozellesmis IPPE_SQUARE cozucu). Bu mesafe MUTLAK'tir - stereo'nun aksine
    olcek belirsizligi yoktur, cunku gercek boyut zaten taninmis.

    corners_px sirasi OpenCV aruco konvansiyonu: sol-ust, sag-ust, sag-alt,
    sol-alt (goruntude, saat yonunde).
    """
    import cv2
    s = float(marker_size_mm) / 2.0
    obj_pts = np.array([
        [-s,  s, 0.0],
        [ s,  s, 0.0],
        [ s, -s, 0.0],
        [-s, -s, 0.0],
    ], dtype=np.float32)
    dist = np.zeros(5, np.float32) if dist is None else np.asarray(dist, np.float32)
    ok, rvec, tvec = cv2.solvePnP(obj_pts, corners_px.astype(np.float32),
                                   np.asarray(K, np.float32), dist,
                                   flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        raise RuntimeError("solvePnP basarisiz - isaret koseleri gecersiz olabilir")
    return rvec, tvec.reshape(3)


def marker_distance_mm(tvec: np.ndarray) -> float:
    return float(np.linalg.norm(tvec))


def estimate_scale_from_marker(image_left_rectified: np.ndarray, K1: np.ndarray,
                                marker_size_mm: float, marker_id: Optional[int],
                                stereo_points_mm: np.ndarray, stereo_px: np.ndarray,
                                dict_name: str = DEFAULT_DICT) -> dict:
    """Tek bir rektifiye SOL goruntude isareti bulur, MUTLAK (monoküler,
    solvePnP) mesafesini hesaplar; ayni isaretin stereo/disparite boru
    hattindan (rig.reproject) gelen 3B noktalarla karsilastirip anlik bir
    olcek duzeltme orani onerir.

    stereo_points_mm : (H,W,3) rig.reproject(disparity) ciktisi (kamera cerc.)
    stereo_px         : kullanilmiyor (arayuz simetrisi icin duruyor - ileride
                         kose bazinda karsilastirma icin genisletilebilir)

    Doner: {"bulundu": bool, "mono_mesafe_mm", "stereo_mesafe_mm",
            "onerilen_olcek", ...}
    """
    det = find_marker(image_left_rectified, marker_id, dict_name)
    if det is None:
        return {"bulundu": False, "sebep": "isaret goruntude bulunamadi"}

    rvec, tvec = marker_pose_mm(det.corners_px, marker_size_mm, K1)
    mono_dist = marker_distance_mm(tvec)

    cx, cy = det.corners_px.mean(axis=0)
    ix, iy = int(round(cx)), int(round(cy))
    H, W = stereo_points_mm.shape[:2]
    if not (0 <= ix < W and 0 <= iy < H):
        return {"bulundu": True, "sebep": "isaret merkezi goruntu disinda (tuhaf)",
                "mono_mesafe_mm": round(mono_dist, 1)}
    win = 5
    y0, y1 = max(0, iy - win), min(H, iy + win + 1)
    x0, x1 = max(0, ix - win), min(W, ix + win + 1)
    patch = stereo_points_mm[y0:y1, x0:x1]
    z_vals = patch[..., 2]
    valid = np.isfinite(z_vals) & (z_vals > 0)
    if int(valid.sum()) < 5:
        return {"bulundu": True, "sebep": "isaret bolgesinde gecerli disparite yok",
                "mono_mesafe_mm": round(mono_dist, 1)}
    stereo_center = patch[valid].mean(axis=0)
    stereo_dist = float(np.linalg.norm(stereo_center))

    ratio = mono_dist / stereo_dist if stereo_dist > 1e-6 else None
    return {
        "bulundu": True,
        "marker_id": det.marker_id,
        "mono_mesafe_mm": round(mono_dist, 1),
        "stereo_mesafe_mm": round(stereo_dist, 1),
        "onerilen_olcek": round(ratio, 4) if ratio else None,
    }
