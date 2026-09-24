# Kalibrasyon dosyalari

Bu klasor, `src/calibration/` altindaki (makineye ozel oldugu icin `.gitignore`'da
olan) guncel kalibrasyon dosyalarinin paylasim kopyasidir. Uretim tarihi: 2026-09-23
(stereo 14:00, el-goz 09:48) - kameralar bu tarihte yeniden kalibre edildi, 2026-09-17
kopyasinin yerine gecti (baseline degisti, bilerek yapildi).
Dosyalar OpenCV `FileStorage` (YAML) formatindadir: `cv2.FileStorage(path, cv2.FILE_STORAGE_READ)`.

## stereo_calib.yaml - stereo kamera cifti (ic + dis parametreler)

| Alan | Anlami |
|---|---|
| `K1`, `d1` | Sol kamera matrisi / distorsiyon (k1,k2,p1,p2,k3) |
| `K2`, `d2` | Sag kamera matrisi / distorsiyon |
| `R`, `T` | Sol kameradan saga donusum (OpenCV `stereoCalibrate` konvansiyonu); `T` **mm** cinsinden, baseline ~34.0 mm (2026-09-17 kopyasinda ~63.6 mm idi) |
| `image_width/height` | 720 x 1280 - kalibrasyon bu cozunurlukte yapildi, baska cozunurlukte K'yi olceklemeden kullanmayin |
| `rms_reproj_err` | 0.57 px |

Kamera indeksleri (cam_web.py): sol = 2, sag = 3.

## base_T_cam.yaml - el-goz (kamera -> robot taban)

4x4 homojen donusum: **sol kamera cercevesindeki bir noktayi FR3 taban cercevesine**
cevirir (`p_base = base_T_cam @ [p_cam, 1]`). Oteleme **mm**.
`fit_residual_mm` = 2.04 mm (2026-09-17 kopyasinda 10.1 mm idi - yeni kalibrasyonla
belirgin iyilesti).

## Kalibrasyon dosyasi olmayan ama gereken degerler

- `MEASURE_SCALE = 0.9997` (`.env`, gitignore'da; 2026-09-17 kopyasinda 1.1313 idi) -
  olcum boyutlarina uygulanan global olcek duzeltmesi. Hangi referans nesneyle
  `tools/calib_scale.py` calistirilarak bulundugu bu guncellemede dogrulanmadi.
- `YAW_CORRECTION_DEG = -46.37` (`tools/pick_and_place.py`, degismedi): `src/grasp.py`'nin
  urettigi tool yaw'i ROS2/MoveIt API'sine dogrudan verilince ~46 derece kayiyor;
  deneysel duzeltme, **kok neden henuz bulunamadi**.

## Bilinen acik sorun (2026-09-24)

Kucuk/dar nesnelerde (orn. sigara paketi, ~85x50x19mm) ayak izi olcumu sistematik
kucuk cikiyor (~13-15mm, boy ve en'de). Kaynak kalibrasyon degil: `src/multi.py`
~720. satirdaki maske erozyonu (`MORPH_ELLIPSE (17,17)`), kapatma kernelinden
(11,11) buyuk oldugu icin nesne sinirindan sabit bir miktar icin kirpiyor. Yukseklik
(derinlik) bu yoldan gecmiyor, o yuzden nispeten dogru. Henuz duzeltilmedi.
