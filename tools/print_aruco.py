#!/usr/bin/env python3
"""
Yazdirilabilir ArUco isareti uretir - TAM olcekte.

--out .pdf ile bitiyorsa PDF olarak yazilir (ONERILEN): PDF'in sayfa boyutu
gercek fiziksel olcude tanimli oldugu icin yazici programlari neredeyse
her zaman "gercek boyut / %100" ile acar. PNG'deki DPI bilgisi ise pek
cok goruntu goruntuleyici/yazdirma penceresi tarafindan yok sayilip
"sayfaya sigdir" yapiliyor - bu da fiziksel boyutu bozuyor.

Yazdirdiktan SONRA mutlaka kumpas/cetvelle olculup dogrulanmali - yazicilar
%100 olcekte bile birkac yuzde kayabiliyor.

KULLANIM:
    python -m tools.print_aruco --id 7 --size-mm 50 --out marker_id7_50mm.pdf
    (yazdir, kumpasla olc, gercek deger farkliysa src/aruco_scale.py'ye
    verdiginiz marker_size_mm'i GERCEK olculen degerle degistirin)
"""
from __future__ import annotations

import argparse


def main() -> int:
    import cv2
    import cv2.aruco as aruco
    import numpy as np
    from PIL import Image

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--id", type=int, default=7, help="isaret ID'si (0-49, DICT_4X4_50)")
    ap.add_argument("--size-mm", type=float, default=50.0, help="isaretin kenar uzunlugu (mm)")
    ap.add_argument("--dpi", type=int, default=300, help="yazici cozunurlugu")
    ap.add_argument("--margin-mm", type=float, default=10.0,
                     help="etrafinda birakilacak beyaz kenar (mm) - algilamayi kolaylastirir")
    ap.add_argument("--out", default="aruco_marker.pdf")
    args = ap.parse_args()

    side_px = int(round(args.size_mm / 25.4 * args.dpi))
    margin_px = int(round(args.margin_mm / 25.4 * args.dpi))

    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    marker = aruco.generateImageMarker(dictionary, args.id, side_px)

    canvas = np.full((side_px + 2 * margin_px, side_px + 2 * margin_px), 255, np.uint8)
    canvas[margin_px:margin_px + side_px, margin_px:margin_px + side_px] = marker

    img = Image.fromarray(canvas)
    total_mm = args.size_mm + 2 * args.margin_mm
    if args.out.lower().endswith(".pdf"):
        img.save(args.out, "PDF", resolution=float(args.dpi))
        print(f"Yazildi: {args.out}  (PDF, sayfa {total_mm:.1f}x{total_mm:.1f}mm)")
    else:
        img.save(args.out, dpi=(args.dpi, args.dpi))
        print(f"Yazildi: {args.out}  ({canvas.shape[1]}x{canvas.shape[0]}px @ {args.dpi}dpi)")
        print("UYARI: PNG yazdirirken yaziciniz 'gercek boyut/actual size' secmezse "
              "(cogu goruntu goruntuleyici varsayilani 'sayfaya sigdir'dir) boyut bozulur "
              "- .pdf uzantisiyla calistirmak daha guvenilir.")
    print(f"Isaret ID={args.id}, kenar={args.size_mm}mm (DICT_4X4_50)")
    print("UYARI: yazdirdiktan sonra kumpas/cetvelle kenarini olcup dogrulayin - "
          "yazici olcek hatasi buradaki sayiyi gecersiz kilar.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
