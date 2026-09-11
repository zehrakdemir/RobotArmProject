# valiz-olcum-robot

Sabit **2 kamera (stereo)** + gerçek **Fairino FR3** kol ile çalışan bir hücre.
Masaya konan kutuları/valizleri kameradan **segmentleyip ölçer**, duruşunu
(dik / yatık) sınıflandırır, **en büyükten en küçüğe** sırayla FR3 ile alıp
başka bir konuma yerleştirir.

> Kartonla test ediliyor, hedef gerçek valizler. Görüntü + planlama tarafı bu
> repo; fiziksel montaj (kamera/robot konumu, gripper) kullanıcıya özeldir.

---

## Ne yapar

```
2 sabit kamera (masaya tepeden bakar)
        │
        ▼
1. STEREO REKTIFIKASYON     src/calib.py        stereo_calib.yaml → rektifiye + Q
        │                                       kameralar oynadıysa → src/selfcalib.py
        │                                       (dokulu sahneden R,T'yi otomatik yeniler)
        ▼
2. YOĞUN DERINLIK           src/stereo.py       SGBM + WLS → disparite → 3B nokta bulutu
        │
        ▼
3. ÇOK-NESNE TESPIT         src/multi.py        masa düzlemi (RANSAC) → üstteki noktalar
        │                                       → 2B kümeleme + yumuşak-cisim birleştirme
        │                                       → her nesne için OBB + segmentasyon konturu
        ▼
4. ÖLÇÜ                      src/multi.py        boy (en uzun) · en (ortanca) · derinlik (en kısa)
        │                                       ayak izi = maske→düzlem izdüşümü,
        │                                       yükseklik = düzleme uzaklık, MEASURE_SCALE düzeltmesi
        ▼
5. DURUŞ                     src/pose_classify.py   dik / yatık / belirsiz
        │
        ▼
6. KAVRAMA PLANI            src/grasp.py        2 düz yüz ortası, açıklık kontrolü
        │                    src/align.py       yatıksa: üstten it → dik konuma devir
        ▼
7. FR3                       src/robot/fairino_fr3.py   yaklaş · kapat · kaldır · taşı · bırak
                                                        (base_T_cam yoksa pozlar kamera çerçevesinde)
```

Sürekli döngü orkestrasyonu: [`src/auto.py`](src/auto.py) (`AutoCell`).

---

## Arayüz (Streamlit)

`http://localhost:8501` — `app.py` bir yönlendirici (`st.navigation`), gerçek
sayfalar `pages/` altında:

| Sayfa | İş |
|---|---|
| **Otomatik Hucre** (varsayılan) | BAŞLAT/DURDUR — çok nesneli sürekli al-yerleştir döngüsü + canlı segmentasyon + derinlik haritası + 3B bulut |
| **Hucre Modu** | Tek nesne: ölç + duruş + FR3 komutu (elle duruş/ROI override'ları burada) |
| **Hizli Stereo** | Tek çift fotoğraftan hızlı derinlik/bulut denemesi — kamera hizalama kontrolü |

Eski 2-fotoğraf demosu (`legacy/app_2foto_demo.py`) nav'dan kaldırıldı —
kalibrasyon kullanmıyordu, artık gerçek kullanım Otomatik Hücre. Kod duruyor,
`app.py`'deki `st.navigation` listesine bir `st.Page` daha ekleyip geri
getirebilirsin.

---

## Kurulum

```bash
cp .env.example .env          # kendi kurulumuna göre düzenle
docker compose up -d --build
```

- Kod (`src/`, `pages/`, `tools/`, `app.py`) imaja `COPY` ile gömülür →
  **kaynak değişince `docker compose up -d --build` gerekir.**
- `src/calibration/`, `models/`, `snaps/`, `calib_shots/` bind-mount'tur →
  bunları imajı yeniden kurmadan değiştirebilirsin.
- Kameralar host'ta ayrı bir servisle yayınlanır (`tools/cam_web.py`, port 8600);
  konteyner `CAM_WEB_URL` üzerinden çeker. USB kamera doğrudan konteynıra
  geçirmek için `docker-compose.cell.yml` (native Linux) — bkz. [DEPLOY_CELL.md](DEPLOY_CELL.md).

### Kamera yayını (host)

```bash
python tools/cam_web.py --left 2 --right 3 --port 8600 --snapdir ./snaps
```

`cv2` içeren bir Python gerekir (`pip install opencv-contrib-python`).

---

## Kalibrasyon

| # | Ne zaman | Komut | Çıktı |
|---|---|---|---|
| 1 | Bir kez (satranç tahtası) | `tools/capture_pairs` + `tools/stereo_calibrate` | `src/calibration/stereo_calib.yaml` |
| 2 | Kameralar her oynadığında | **otomatik** — `src/selfcalib.py`, `detect_objects` içinde tetiklenir; elle: `python -m tools.self_calibrate` | `stereo_calib.yaml` (R,T güncellenir) |
| 3 | Bir kez (kamera↔robot) | `tools/handeye_touch.py` — satranç tahtası köşelerine TCP dokundur (`--corner r,c` ×5-6, sonra `--solve`) | `src/calibration/base_T_cam.yaml` |
| 4 | Bilinen kutuyla ölçek kilidi | `python -m tools.calib_scale --dims boy,en,derinlik` | `.env` → `MEASURE_SCALE` |

Bu dosyalar makineye özeldir, **git'e girmez** (`.gitignore`). `stereo_calib.yaml`
yoksa kamera pipeline'ı çalışmaz; `base_T_cam.yaml` yoksa ölçüm/bulut çalışır ama
FR3 pozları **kamera çerçevesinde** kalır (kol nesneye tam gitmez).

### Online öz-kalibrasyon (`src/selfcalib.py`)

Kameralar sabitlenemediği için oturumlar arası kayabilir. Öz-kalibrasyon lens iç
parametrelerini (K, d — kamera oynayınca değişmez) korur, sadece kameralar-arası
pozu (R, T) dokulu masadan yeniden kestirir: ORB+AKAZE eşleşme →
`findEssentialMat` → `recoverPose` → Sampson hatası üzerinde LM ince ayar,
`|T|` ilk kalibrasyonun baseline'ına kilitli. `detect_objects` merkez ROI'de
geçerli disparite `SELFCAL_MIN_VALID_FRAC`'in altına düşerse **oturum başına bir
kez** otomatik çalışır.

---

## FR3 notları

- **Varsayılan `ROBOT_ENABLED=false`** → hiçbir komut kola gitmez, sadece loglanır.
- Bu firmware'de (FR3 V6.0 / kontrolcü V3.9.8) ham XML-RPC `MoveJ`/`MoveL`
  bozuk (hata 154/101). Hareket, kapalı-çevrim **JOG** ile yapılıyor
  (`_jog_to_joints`, `src/robot/fairino_fr3.py`). `StartJOG`/`StopJOG` sağlam.
- `prepare()` = `ResetAllError` → `Mode(0)` → `RobotEnable(1)`; hatadan sonra
  `_recover()` otomatik.
- Ağ: FRC100 kutusunda 2 NIC — Eth0 `192.168.57.2`, Eth1 `192.168.58.2`.
  `.env` → `ROBOT_IP`.

---

## Yapılandırma (`.env`)

Tam liste `.env.example` içinde. Öne çıkanlar:

| Değişken | Varsayılan | Ne |
|---|---|---|
| `ROBOT_ENABLED` / `ROBOT_IP` | `false` / `192.168.58.2` | FR3'e gerçek komut / kontrolcü IP |
| `MEASURE_SCALE` | `1.0` | Disparite→derinlik ölçek düzeltmesi (`tools/calib_scale` yazar) |
| `SELFCAL_MIN_VALID_FRAC` | `0.18` | Bunun altında geçerli disparite → otomatik öz-kalibrasyon |
| `CLUSTER_MERGE_GAP_MM` | `35` | Ayak izleri bu kadar yakın kümeler tek nesne (yumuşak cisim parçalanması) |
| `CAM_WEB_URL` | `http://host.docker.internal:8600` | Host kamera yayını |
| `PLACE_SLOTS` / `PLACE_POSE` | — | Yerleştirme konumları `x,y,z,rx,ry,rz` |
| `GRIPPER_ENABLED` / `GRIPPER_MAX_OPENING_MM` | `false` / `80` | Jodell gripper — **açıklığı ölç ve gir** |

Etiket kuralı: **boy = en uzun** boyut · **en = ortanca** · **derinlik = en kısa**.

---

## Testler

```bash
pytest -q tests/     # model / cv2 gerektirmeyen birim testleri
```

---

## Dizin yapısı

```
app.py                  Streamlit giriş — sayfa yönlendirici (st.navigation)
pages/                  Streamlit sayfaları (Otomatik Hucre, Hucre Modu, Hizli Stereo)
src/ui.py                                           ortak tema (yalnız görsel)
legacy/                 nav'dan kaldırılmış eski 2-foto demosu (kod duruyor)
src/
  calib.py   selfcalib.py   stereo.py   multi.py     stereo + çok-nesne çekirdeği
  auto.py                                            AutoCell al-yerleştir döngüsü
  grasp.py   align.py   pose_classify.py             kavrama / tip-up / duruş
  robot/fairino_fr3.py                               FR3 sürücü (JOG tabanlı)
  cell.py    pipeline.py                             tek-nesne / 2-fotoğraf akışları
  config.py                                          tüm .env anahtarları
tools/                  CLI: kalibrasyon, cam_web, run_cell, handeye, self_calibrate ...
src/calibration/        stereo_calib.yaml + base_T_cam.yaml (git'e girmez)
```

---

## Durum

- ✅ Stereo + online öz-kalibrasyon + çok-nesne segmentasyon/ölçüm çalışıyor
- ✅ FR3 gerçek donanımda JOG hareketi doğrulandı
- ⏳ Hand-eye (`base_T_cam`) — satranç tahtasıyla yapılacak
- ⏳ Gerçek uçtan uca al-yerleştir denemesi (hand-eye sonrası)
