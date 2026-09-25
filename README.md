# RobotArmProject

Sabit **stereo kamera** + **Fairino FR3** iş birlikçi kolu ile çalışan bir al-yerleştir
hücresi. Kamera masaya konan kutuları/valizleri segmentleyip **ölçer**, duruşunu
(dik / yatık) sınıflandırır ve kavrama noktasını çıkarır; kol da nesneleri sırayla
alıp hedef konuma yerleştirir.

> Kartonla test ediliyor, hedef gerçek valizler. Bu depo görüntü işleme + planlama
> tarafıdır; fiziksel montaj (kamera/robot konumu, gripper) kuruluma özeldir.

---

## Depo nasıl bölünmüş

Proje iki yarımdan oluşuyor ve **her yarım kendi branch'inde** geliştiriliyor.
`main` sadece bu giriş sayfasıdır — kod içermez.

| Branch | Yarım | Ne yapar | Teknoloji |
|---|---|---|---|
| [**`zehra`**](https://github.com/zehrakdemir/RobotArmProject/tree/zehra) | **Görme** | stereo rektifikasyon → yoğun derinlik → çok-nesne segmentasyon → ölçü → duruş → kavrama planı → FR3'e al-yerleştir | OpenCV (SGBM+WLS), YOLO-seg, Streamlit, FastAPI, Docker |
| [**`huseyin`**](https://github.com/zehrakdemir/RobotArmProject/tree/huseyin) | **Kol** | robot algılama, hareket planlama, öğret/kaydet/oynat, gripper sürücüsü, MTC pick & place, web arayüzü | ROS2 Humble, MoveIt2, MoveIt Task Constructor, Docker |

İki branch yalnızca ilk commit'i paylaşır; dosya ağaçları tamamen ayrıdır
(ikisinde de `src/`, `tests/`, `tools/`, `Dockerfile` var ama içerikleri farklı işler).
**Bu yüzden birbirine merge edilmezler** — ayrı kalmaları kasıtlıdır. Konuştukları
yer kod değil, aşağıdaki HTTP arayüzüdür.

Ayrıntılı kurulum ve kullanım her branch'in kendi README'sindedir:
[görme](https://github.com/zehrakdemir/RobotArmProject/blob/zehra/README.md) ·
[kol](https://github.com/zehrakdemir/RobotArmProject/blob/huseyin/README.md)

---

## Mimari

```
             2 USB kamera (masaya tepeden bakar)
                             │
                             ▼
   tools/cam_web.py          host · :8600 · MJPEG yayın + çift kaydet
                             │
                             ▼
 ── GÖRME ─ branch zehra ─ Docker: valiz-olcum ─────────────────────────
     stereo rektifikasyon → yoğun derinlik → çok-nesne segmentasyon
     → ölçü → duruş → kavrama planı

        :8501  Streamlit     ölçüm · duruş · 3B nokta bulutu
        :8090  pickcobot     görme + kol tek sayfada, her adım onaylı
        :8502  detect_api    vizyon API köprüsü
 ──────────────────────────────────────────────────────────────────────
                             │
                             │  HTTP   /detect · /slots · /health
                             ▼
 ── KOL ─ branch huseyin ─ Docker: cobot ───────────────────────────────
        :8088  movecobot-web
               ROS2 Humble + MoveIt2 + MoveIt Task Constructor
 ──────────────────────────────────────────────────────────────────────
                             │  yamalı frcobot_ros2 sürücüsü
                             ▼
        Fairino FR3  +  Jodell EPG40-050 gripper
                             ▲
                             └─ görme tarafı XML-RPC JOG ile doğrudan da sürebilir
                                (src/robot/fairino_fr3.py)
```

Kola iki bağımsız yol var ve **ikisi aynı anda robota komut göndermemelidir**.
`pickcobot` (:8090) bu ikisini tek sayfada birleştirir: her fazı (yaklaş · in ·
kavra · kaldır · taşı · bırak) kullanıcı tek tıkla kendisi tetikler, hiçbir adım
otomatik zincirlenmez.

### İki yarımı bağlayan sözleşme

`detect_api` (görme, :8502) ↔ `src/fairino_cobot/vizyon.py` (kol) arasındaki HTTP API:

| Uç nokta | Döner |
|---|---|
| `GET /health` | `{"ok": bool, "handeye": bool}` |
| `GET /slots` | yerleştirme konumları — `PLACE_SLOTS` |
| `POST /detect` | her nesne için ölçü (cm), duruş, OBB, kavrama noktası + RPY |

Tüm konumlar **robot taban çerçevesinde, mm**. Bu, hand-eye kalibrasyonunun
(`base_T_cam.yaml`) geçerli olmasını gerektirir; yoksa `handeye: false` döner ve
pozlar kamera çerçevesinde kalır. Sözleşme iki branch'te birden tanımlıdır —
değiştiren her iki tarafı da günceller.

---

## Hızlı başlangıç

Her iki yarım da Docker ile çalışır, host'a Python/ROS kurulumu gerekmez.

**Görme** ([ayrıntı](https://github.com/zehrakdemir/RobotArmProject/blob/zehra/README.md)):

```bash
git checkout zehra
cp .env.example .env          # ROBOT_IP, MEASURE_SCALE, PLACE_SLOTS ...
python tools/cam_web.py --left 2 --right 3 --port 8600 --snapdir ./snaps   # host
docker compose up -d --build  # → http://localhost:8501
```

**Kol** ([ayrıntı](https://github.com/zehrakdemir/RobotArmProject/blob/huseyin/README.md)):

```bash
git checkout huseyin
cp .env.example .env          # robotun IP'si
docker compose up cobot       # → http://localhost:8088
```

Aynı klasörde `git checkout` ile branch değiştirmek, diğer yarımın dosyalarını
çalışma kopyasından **kaldırır** (kod branch'te durur, kaybolmaz — ama klasör boş
görünür). İkisiyle birlikte çalışacaksan ayrı çalışma kopyaları aç:

```bash
git worktree add ../RobotArmProject-huseyin huseyin
```

---

## Günlük işletim notları

- **Kod değiştiyse `--build` şart.** Görme tarafında `src/`, `pages/`, `tools/`, `app.py`
  imaja `COPY` ile gömülür; `docker compose up -d` tek başına eski kodu çalıştırır.
- **Kalibrasyon ve çıktı klasörleri bind-mount'tur** (`src/calibration/`, `models/`,
  `snaps/`, `calib_shots/`) — bunları imajı yeniden kurmadan değiştirebilirsin.
- **`pickcobot` (:8090) ve `detect_api` (:8502) imajın CMD'si değildir**, her yeniden
  başlatmadan sonra elle ayağa kaldırılır:

  ```bash
  docker compose exec -d valiz-olcum python -m tools.pnp_web.server
  ```

  `detect_api` ise ayrı bir `detect-api-bridge` konteynerinde durur (aynı imaj);
  8502 eşlemesi port çakışması nedeniyle `docker-compose.yml`'de kapalıdır.
- **Varsayılan `ROBOT_ENABLED=false`** → hiçbir komut kola gitmez, sadece loglanır.
  Gerçek donanıma geçmeden önce bilinçli olarak açılır.

---

## Kalibrasyon

Kalibrasyon dosyaları makineye özeldir ve **git'e girmez**. Üretimi ve paylaşımı
görme branch'indedir:

| Ne | Ne zaman | Çıktı |
|---|---|---|
| Stereo (satranç tahtası) | bir kez | `src/calibration/stereo_calib.yaml` |
| Online öz-kalibrasyon | kameralar oynadığında, otomatik | `stereo_calib.yaml` (R, T) |
| Hand-eye (kamera ↔ robot) | bir kez | `src/calibration/base_T_cam.yaml` |
| Ölçek kilidi (bilinen kutu) | ölçüm sapınca | `.env` → `MEASURE_SCALE` |

`stereo_calib.yaml` yoksa kamera hattı hiç çalışmaz. `base_T_cam.yaml` yoksa ölçüm
ve nokta bulutu çalışır ama robot pozları kamera çerçevesinde kalır — kol nesneye
tam gitmez.

---

## Donanım

| Parça | Not |
|---|---|
| Kol | Fairino FR3 (V6.0 / kontrolcü V3.9.8). Kol tarafı FR5/FR10/FR16/FR20/FR30'u da destekler |
| Ağ | FRC100 kutusunda 2 NIC — Eth0 `192.168.57.2`, Eth1 `192.168.58.2`; `.env` → `ROBOT_IP` |
| Gripper | Jodell EPG40-050, Modbus RTU. WebApp'te `End_Jodell_V1.0` protokolü uygulanmış olmalı |
| Kamera | 2 × USB, masaya tepeden bakacak şekilde sabit |
