# Kalibrasyon dosyalari

Bu klasor, `src/calibration/` altindaki (makineye ozel oldugu icin `.gitignore`'da
olan) guncel kalibrasyon dosyalarinin paylasim kopyasidir. Uretim tarihi: 2026-09-17.
Dosyalar OpenCV `FileStorage` (YAML) formatindadir: `cv2.FileStorage(path, cv2.FILE_STORAGE_READ)`.

## stereo_calib.yaml - stereo kamera cifti (ic + dis parametreler)

| Alan | Anlami |
|---|---|
| `K1`, `d1` | Sol kamera matrisi / distorsiyon (k1,k2,p1,p2,k3) |
| `K2`, `d2` | Sag kamera matrisi / distorsiyon |
| `R`, `T` | Sol kameradan saga donusum (OpenCV `stereoCalibrate` konvansiyonu); `T` **mm** cinsinden, baseline ~63.6 mm |
| `image_width/height` | 1280 x 720 - kalibrasyon bu cozunurlukte yapildi, baska cozunurlukte K'yi olceklemeden kullanmayin |
| `rms_reproj_err` | 0.56 px |

Kamera indeksleri (cam_web.py): sol = 2, sag = 3.

## base_T_cam.yaml - el-goz (kamera -> robot taban)

4x4 homojen donusum: **sol kamera cercevesindeki bir noktayi FR3 taban cercevesine**
cevirir (`p_base = base_T_cam @ [p_cam, 1]`). Oteleme **mm**.
`fit_residual_mm` = 10.1 mm (yani taban cercevesinde ~1 cm'lik artik hata var;
mutlak konum hassasiyeti bu mertebede beklenmeli).

## Kalibrasyon dosyasi olmayan ama gereken degerler

- `MEASURE_SCALE = 1.1313` (`.env`, gitignore'da): olcum boyutlarina uygulanan global
  olcek duzeltmesi, gercek bir sigara paketiyle (85x50x18 mm) `tools/calib_scale.py` ile bulundu.
- `YAW_CORRECTION_DEG = -46.37` (`tools/pick_and_place.py`): `src/grasp.py`'nin urettigi
  tool yaw'i ROS2/MoveIt API'sine dogrudan verilince ~46 derece kayiyor; deneysel
  duzeltme, **kok neden henuz bulunamadi**.
