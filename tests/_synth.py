"""Sentetik veri ureticileri (model/cv2 gerektirmez)."""
from __future__ import annotations

import numpy as np


def box_point_cloud(
    real_w: float, real_h: float, real_d: float,
    img_h: int = 200, img_w: int = 200,
    r0: int = 40, r1: int = 160, c0: int = 30, c1: int = 150,
    z0: float = 100.0,
    lean_deg: float = 0.0,
):
    """
    (H, W, 3) 3B nokta haritasi + (H, W) maske dondurur.

    Maske icindeki piksellerde:
        X  ~ [0, real_w]   (sutun boyunca)
        Y  ~ [0, real_h]   (satir boyunca, dikey)
        Z  ~ [z0, z0+real_d] (sutun boyunca -> derinlik gradyani)
    lean_deg > 0 ise dikey eksen X-Y duzleminde dondurulur.
    """
    pts = np.full((img_h, img_w, 3), np.nan, dtype=np.float64)
    mask = np.zeros((img_h, img_w), dtype=bool)

    rows = np.arange(r0, r1)
    cols = np.arange(c0, c1)
    nn_r = (rows - r0) / max(len(rows) - 1, 1)
    nn_c = (cols - c0) / max(len(cols) - 1, 1)

    theta = np.radians(lean_deg)
    ct, stt = np.cos(theta), np.sin(theta)

    for i, y in enumerate(rows):
        for j, x in enumerate(cols):
            X = nn_c[j] * real_w
            Y = nn_r[i] * real_h
            Z = z0 + nn_c[j] * real_d
            # dikey ekseni X-Y duzleminde dondur
            Xr = X * ct - Y * stt
            Yr = X * stt + Y * ct
            pts[y, x] = (Xr, Yr, Z)
            mask[y, x] = True
    return pts, mask


def box_surface_3d(dims_mm, center_mm=(500.0, 0.0, 0.0), yaw_deg=0.0,
                   ground_z=0.0, per_face=1400, seed=0):
    """
    Yerçekimine hizali kutu YUZEY noktalari (N,3), robot taban cercevesi (mm, Z yukari).
    dims_mm = (uzun_yatay, kisa_yatay, dikey).  center_mm[2] yok sayilir;
    kutu tabani ground_z'de oturur.
    """
    rng = np.random.default_rng(seed)
    a, b, hgt = [float(v) for v in dims_mm]
    cx, cy, _ = center_mm
    cz = ground_z + hgt / 2.0
    th = np.radians(yaw_deg)
    R = np.array([[np.cos(th), -np.sin(th), 0],
                  [np.sin(th),  np.cos(th), 0],
                  [0, 0, 1.0]])
    half = np.array([a / 2, b / 2, hgt / 2])
    pts = []
    for axis in range(3):
        for sign in (-1, 1):
            uv = rng.uniform(-1, 1, size=(per_face, 3))
            uv[:, axis] = sign
            pts.append(uv * half)
    P = np.vstack(pts)
    P = P @ R.T + np.array([cx, cy, cz])
    return P
