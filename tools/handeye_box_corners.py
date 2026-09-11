#!/usr/bin/env python3
"""
HAND-EYE - satranc tahtasi/ArUco YOK, yazici gerekmez.

Masadaki HERHANGI bir kutunun GORUNEN kontur koselerini kullanir: sistem
zaten segmentasyondan o kosenin piksel konumunu VE stereo'dan 3B (mm, kamera
cercevesi) konumunu biliyor - tipki ekranda gordugun yesil/turuncu cerceve
gibi. Sen sadece robotu o kosenin ucuna goturup dokunduruyorsun.

1) Masaya TEK bir kutu koy (duz, net gorunsun).
   docker compose exec -T valiz-olcum python -m tools.handeye_box_corners --snap
   -> snaps/handeye/box_corners.png kaydeder: kontur + numarali kose noktalari.
      Terminaldeki kamera-mm degerleriyle gorsele bak, hangi kosenin hangisi
      oldugunu gor.

2) Robotu (pendant/jog) o kosenin TAM UCUNA goturup dokundur, sonra:
   docker compose exec -T valiz-olcum python -m tools.handeye_box_corners --mark 0
   ... (kutuyu BIRAZ kaydir/dondur/farkli yere koy, --snap ile tekrar kontrol
        et, farkli bir kosesine daha dokun - FARKLI POZLARDAN cesitlilik onemli
        ki cozum tek bir bolgeye asiri uymasin) ...
   En az 5-6 nokta birikince (ideali 8-10, birkac farkli kutu/poz):
   docker compose exec -T valiz-olcum python -m tools.handeye_box_corners --solve

--tool-offset x,y,z : TCP'den dokunan uca vektor (mm). Uç TCP ise 0,0,0.
--pick N            : masada >1 nesne varsa hangisi (varsayilan: en buyuk, 0).
"""
from __future__ import annotations

import argparse
import glob
import json
import os

import numpy as np

from src import config

PTS = os.path.join(config.SNAP_DIR, "handeye", "box_corners.json")


def _umeyama(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S, D = src - mu_s, dst - mu_d
    U, _, Vt = np.linalg.svd(S.T @ D / len(src))
    dsig = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, dsig]) @ U.T
    t = mu_d - R @ mu_s
    rms = float(np.sqrt(np.mean(np.sum((dst - (src @ R.T + t)) ** 2, axis=1))))
    return R, t, rms


def _latest_pair():
    ls = sorted(glob.glob(os.path.join(config.SNAP_DIR, "left_*.png")))
    if not ls:
        raise RuntimeError("snaps/ bos - once cam_web'den bir cift kaydet")
    return ls[-1], ls[-1].replace("left_", "right_")


def _corner_points(pick: int):
    """GORUNEN kontur koselerini bul + her birinin GERCEK 3B (kamera mm)
    konumunu ayni kareden dogrudan oku (OBB eksen varsayimi YOK - tipped/flat
    karisikligindan bagimsiz, kontur = ekranda goruneni birebir yansitir).
    Doner: (px_xy (K,2) int, xyz_mm (K,3), DetectedObject, rektifiye_sol_img, sol_yol)."""
    import cv2
    from src.calib import StereoRig
    from src.cell import _epipolar_correct
    from src.multi import detect_objects, _to_gray
    from src.stereo import compute_disparity_adaptive

    lp, rp = _latest_pair()
    L, R = cv2.imread(lp), cv2.imread(rp)
    if L is None or R is None:
        raise RuntimeError(f"cift okunamadi: {lp}")
    objs = detect_objects(L, R)
    if not objs:
        raise RuntimeError("nesne bulunamadi - masada tek, duz duran bir kutu birak")
    if pick >= len(objs):
        raise RuntimeError(f"--pick {pick} yok, sadece {len(objs)} nesne var")
    o = objs[pick]
    if o.contour is None or len(o.contour) < 3:
        raise RuntimeError("secilen nesnenin konturu yok")

    # ayni kareyi tekrar rektifiye+disparite -> yogun 3B (o.contour ile AYNI
    # rektifiye SOL goruntude, o yuzden piksel koordinatlari dogrudan eslesir)
    rig = StereoRig.load()
    lr, rr = rig.rectify(L, R)
    rr, _, _ = _epipolar_correct(lr, rr)
    disp = compute_disparity_adaptive(_to_gray(lr), _to_gray(rr), roi_mask=None)
    pts = rig.reproject(disp.disparity)
    fin = np.isfinite(pts).all(2) & (disp.disparity > 0)

    cnt = o.contour.reshape(-1, 2).astype(np.float32)
    peri = cv2.arcLength(cnt.astype(np.int32), True)
    approx = cv2.approxPolyDP(cnt.astype(np.int32), 0.02 * peri, True).reshape(-1, 2)
    if len(approx) > 8:                                # cok koseliyse en buyuk 6-8'i tut
        approx = cv2.approxPolyDP(cnt.astype(np.int32), 0.04 * peri, True).reshape(-1, 2)

    H0, W0 = fin.shape
    px_list, xyz_list = [], []
    for (x, y) in approx:
        x, y = int(np.clip(x, 0, W0 - 1)), int(np.clip(y, 0, H0 - 1))
        y0, y1 = max(0, y - 3), min(H0, y + 4)
        x0, x1 = max(0, x - 3), min(W0, x + 4)
        patch_ok = fin[y0:y1, x0:x1]
        if not patch_ok.any():
            continue
        patch = pts[y0:y1, x0:x1][patch_ok]
        xyz = np.median(patch, axis=0)
        px_list.append((x, y))
        xyz_list.append(xyz)
    if len(px_list) < 3:
        raise RuntimeError("kontur koselerinde yeterli 3B nokta yok (kenara cok yakin/gurultulu)")
    return np.array(px_list), np.array(xyz_list), o, lr, lp


def _annotate(lr_img, px_xy):
    import cv2
    img = lr_img.copy()
    for i, (x, y) in enumerate(px_xy):
        cv2.circle(img, (int(x), int(y)), 10, (0, 255, 0), -1)
        cv2.putText(img, str(i), (int(x) + 14, int(y) + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                   (0, 255, 0), 2, cv2.LINE_AA)
    out = os.path.join(config.SNAP_DIR, "handeye", "box_corners.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    cv2.imwrite(out, img)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--snap", action="store_true", help="son cifti isle, koseleri goster/kaydet")
    ap.add_argument("--mark", type=int, help="gorseldeki bu kose numarasina simdiki robot TCP'sini kaydet")
    ap.add_argument("--solve", action="store_true")
    ap.add_argument("--reset", action="store_true")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--pick", type=int, default=0, help="birden fazla nesne varsa hangisi (0=en buyuk)")
    ap.add_argument("--tool-offset", default="0,0,0")
    ap.add_argument("--out", default="src/calibration/base_T_cam.yaml")
    args = ap.parse_args()
    os.makedirs(os.path.dirname(PTS), exist_ok=True)

    if args.reset:
        if os.path.exists(PTS):
            os.remove(PTS)
        print("noktalar silindi"); return 0

    data = json.load(open(PTS)) if os.path.exists(PTS) else []

    if args.snap:
        px, xyz, o, lr, lp = _corner_points(args.pick)
        png = _annotate(lr, px)
        print(f"nesne #{o.rank} dims={o.dims_cm} cm  ({os.path.basename(lp)})")
        for i, c in enumerate(xyz):
            print(f"  kose {i}: kamera_mm = {[round(float(v), 1) for v in c]}")
        print(f"gorsel -> {png}  (numaralar uzerinde isaretli)")
        return 0

    if args.mark is not None:
        px, xyz, o, lr, lp = _corner_points(args.pick)
        if not (0 <= args.mark < len(xyz)):
            print(f"HATA: --mark 0..{len(xyz)-1} olmali (once --snap ile gorsele bak)"); return 2
        cam = xyz[args.mark].tolist()
        from src.robot import FairinoFR3
        fr3 = FairinoFR3(enabled=True)
        if not fr3.connect():
            print("FR3 BAGLANAMADI"); return 1
        j = fr3._joints_now()
        fk = fr3._call("GetForwardKin", j, timeout=8)
        if not (isinstance(fk, (list, tuple)) and int(fk[0]) == 0):
            print("TCP okunamadi"); return 1
        off = np.array([float(x) for x in args.tool_offset.split(",")])
        tcp = (np.array([float(v) for v in fk[1:7]])[:3] + off).tolist()
        data.append({"cam_mm": cam, "base_mm": tcp, "cift": os.path.basename(lp)})
        json.dump(data, open(PTS, "w"), indent=1)
        print(f"kose {args.mark}  kamera={[round(v,1) for v in cam]}  "
              f"robot={[round(v,1) for v in tcp]}   toplam {len(data)} nokta")
        if len(data) < 5:
            print("Daha fazla nokta lazim (>=5 oneri) - kutuyu/kutuyu kaldir-koy, tekrarla.")
        return 0

    if args.solve:
        if len(data) < 4:
            print(f"HATA: sadece {len(data)} nokta (>=4 gerekli, >=6-8 daha guvenli)"); return 1
        src = np.array([d["cam_mm"] for d in data])
        dst = np.array([d["base_mm"] for d in data])
        R, t, rms = _umeyama(src, dst)
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
        from src.calib import save_base_T_cam
        save_base_T_cam(args.out, T, residual_mm=rms)
        print(f"base_T_cam artik hata (RMS) = {rms:.1f} mm   ({len(data)} nokta)")
        if rms > 15:
            print("UYARI: >15 mm. Daha cok / daha cesitli poz ekle "
                  "(kutuyu farkli yerlere/acilara koyup tekrarla).")
        print(f"yazildi -> {args.out}")
        return 0

    print(f"kayitli {len(data)} nokta:")
    for d in data:
        print(f"  kam={[round(v,1) for v in d['cam_mm']]}  "
              f"rob={[round(v,1) for v in d['base_mm']]}  ({d.get('cift','?')})")
    print("--snap ile baslayin, sonra --mark N, sonunda --solve")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
