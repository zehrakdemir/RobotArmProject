#!/usr/bin/env python3
"""
KAMERA -> ROBOT TABAN cercevesi (base_T_cam) - ZEMIN DUZLEMINDEN.

Simulasyon robotu icin: kameralarin onunde fiziksel kol olmadigindan klasik
ArUco hand-eye yapilamaz. Bunun yerine robot tabanini SEN tanimlarsin:
  - Z ekseni  = zemin normali (yukari)
  - Orijin    = zeminde secilen bir nokta (varsayilan: sahne merkezinin zemine izdusumu)
  - X ekseni  = zemin duzleminde secilen bir yon (varsayilan: kameranin +X'i; --yaw-deg ile dondur)

Kalibreli stereo (stereo_calib.yaml) gerekir.

    docker compose run --rm valiz-olcum python -m tools.base_frame_from_floor
    # bos masanin en son ÇİFT KAYDET karesini kullanir -> src/calibration/base_T_cam.yaml

    ... --left snaps/left_X.png --right snaps/right_X.png --yaw-deg 90 --base-z-mm 0
"""
from __future__ import annotations

import argparse
import glob
import os

import numpy as np


def _latest_pair(folder="snaps"):
    ls = sorted(glob.glob(os.path.join(folder, "left_*.png")))
    if not ls:
        raise SystemExit(f"'{folder}/' bos. http://localhost:8600 -> ÇİFT KAYDET (bos masa).")
    return ls[-1], ls[-1].replace("left_", "right_")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--left")
    ap.add_argument("--right")
    ap.add_argument("--calib", default="src/calibration/stereo_calib.yaml")
    ap.add_argument("--out", default="src/calibration/base_T_cam.yaml")
    ap.add_argument("--yaw-deg", type=float, default=0.0,
                    help="X eksenini zemin duzleminde bu kadar dondur (derece)")
    ap.add_argument("--base-z-mm", type=float, default=0.0,
                    help="Robot tabani zeminin bu kadar USTUNDE (kaide varsa)")
    ap.add_argument("--origin-px", default="",
                    help="Sol rektifiye karede orijin pikseli 'x,y' (bos=sahne merkezi)")
    args = ap.parse_args()

    import cv2
    from src.calib import StereoRig, save_base_T_cam
    from src.segment import fit_plane_ransac
    from src.stereo import compute_disparity_adaptive

    lp, rp = (args.left, args.right) if args.left else _latest_pair()
    if not (args.right):
        rp = lp.replace("left_", "right_")
    print(f"cift: {os.path.basename(lp)} / {os.path.basename(rp)}")
    imgL, imgR = cv2.imread(lp), cv2.imread(rp)
    if imgL is None or imgR is None:
        raise SystemExit("goruntu okunamadi")

    rig = StereoRig.load(args.calib)
    lr, rr = rig.rectify(imgL, imgR)
    disp = compute_disparity_adaptive(cv2.cvtColor(lr, cv2.COLOR_BGR2GRAY),
                                      cv2.cvtColor(rr, cv2.COLOR_BGR2GRAY))
    pts = rig.reproject(disp.disparity)               # (H,W,3) mm, KAMERA cercevesi
    fin = np.isfinite(pts).all(axis=2)
    P = pts[fin]
    if len(P) < 2000:
        raise SystemExit(f"cok az 3B nokta ({len(P)}). Masada biraz doku olsun, isik esit.")

    # aykiri kirp, sonra zemin duzlemi
    for k in range(3):
        lo, hi = np.percentile(P[:, k], [1, 99]); P = P[(P[:, k] >= lo) & (P[:, k] <= hi)]
    scale = float(np.linalg.norm(np.percentile(P, 97, 0) - np.percentile(P, 3, 0)))
    nrm, d, inl = fit_plane_ransac(P, thresh=max(0.01 * scale, 3.0))
    frac = inl.mean()
    resid = float(np.abs(P[inl] @ nrm + d).std())
    print(f"zemin: inlier %{frac*100:.0f}, sapma {resid:.1f} mm, normal {np.round(nrm,3)}")
    if frac < 0.35:
        raise SystemExit("Belirgin bir zemin duzlemi bulunamadi (masa cok dagilik / dokusuz?).")

    # normal 'yukari' = kameraya DOGRU (kamera +Z ile sahneye bakar; zemin +Z'de)
    if nrm[2] > 0:
        nrm, d = -nrm, -d
    z_axis = nrm / np.linalg.norm(nrm)                # base Z, kamera cercevesinde

    # orijin: zemin uzerinde bir nokta
    if args.origin_px:
        ox, oy = (int(v) for v in args.origin_px.split(","))
        o_cam = pts[oy, ox]
        if not np.isfinite(o_cam).all():
            raise SystemExit("origin-px noktasinda 3B veri yok, baska piksel sec.")
    else:
        o_cam = P[inl].mean(0)                        # zemin inlier'larinin merkezi
    # tam zemine izdusur, sonra base-z kadar yukari (kaide)
    o_cam = o_cam - (o_cam @ nrm + d) * nrm
    o_cam = o_cam + z_axis * (-args.base_z_mm)        # z_axis kameraya dogru; +yukari

    # X ekseni: kameranin +X'ini zemine izdusur, sonra yaw uygula
    x_cam = np.array([1.0, 0.0, 0.0])
    x_axis = x_cam - (x_cam @ z_axis) * z_axis
    if np.linalg.norm(x_axis) < 1e-6:
        x_axis = np.array([0.0, 1.0, 0.0]) - 0 * z_axis
    x_axis /= np.linalg.norm(x_axis)
    th = np.radians(args.yaw_deg)
    x_axis = np.cos(th) * x_axis + np.sin(th) * np.cross(z_axis, x_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)

    # base eksenleri kamera cercevesinde -> R_cam_base (kolonlar)
    R_cb = np.column_stack([x_axis, y_axis, z_axis])
    R = R_cb.T                                        # base_T_cam donme
    t = -R @ o_cam                                    # base_T_cam oteleme
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t

    cam_height = float(abs(np.array([0, 0, 0]) @ nrm + d))
    save_base_T_cam(args.out, T, residual_mm=resid)
    print("\n=== base_T_cam yazildi ===", args.out)
    print(f"  kamera zeminden ~{cam_height:.0f} mm yukarida")
    print(f"  robot tabani orijini (kamera cerc., mm): {np.round(o_cam,1)}")
    print(f"  X ekseni (kamera cerc.): {np.round(x_axis,3)}   yaw={args.yaw_deg:.0f}")
    print(f"  4x4 T:\n{np.round(T,2)}")
    print("\nDogrula: Hucre Modu 'Olcu+Durus' sekmesinde 'cerceve: robot_taban' olmali;")
    print("bir valizin merkezi/z degeri makul mu bak. X yonu ters ise --yaw-deg 180 ekle.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
