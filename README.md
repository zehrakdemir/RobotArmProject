# RobotArmProject — Fairino kol kontrolü

Fairino iş birlikçi kollarını (FR3/FR5/FR10/FR16/FR20/FR30) **ROS2 Humble + MoveIt2**
üzerinden süren araçlar: web arayüzü, hareket öğretme/kaydetme/oynatma, adaptif kavrama
yapan gripper sürücüsü ve MoveIt Task Constructor tabanlı pick & place.

Tamamı tek bir Docker imajında: ROS2, MoveIt2, MoveIt Task Constructor, yamalı Fairino
sürücüsü ve Fairino Python SDK. Kendi makinene hiçbir şey kurmuyorsun.

```
Robot algılama  ──►  ROS2 sürücü + MoveIt2  ──►  web arayüzü   ──►  gripper (Modbus RTU)
(.env / tarama)      (frcobot_ros2, yamalı)     :8088              (Jodell EPG40-050)
                                                 movecobot (CLI)
```

Öne çıkanlar:

- **Tek komutla ayağa kalkar.** `docker compose up cobot` → robotu algılar, modeline uyan
  sürücü kaynağını kurup derler, MoveIt2'yi başlatır ve web arayüzünü açar.
- **Hiçbir IP kodda sabit değil.** Robotun IP'si `.env` ile verilir; verilmezse ağ taranır.
- **Çok katmanlı güvenlik.** Planlanan her yol zemin ve self-collision kontrolünden geçer,
  onay istenir, çalıştıktan sonra SDK üzerinden hedefe varıldığı doğrulanır.
  *(Manuel mod / drag-teach hariç — orada tek güvenlik ağı robotun kendi donanım seviyesindeki
  çarpışma algılamasıdır.)*
- **Vendor yamaları yeniden uygulanabilir.** `frcobot_ros2` üzerindeki düzeltmeler tek bir
  komutla, desteklenen **her** Fairino modeline uygulanır.

Görüntü işleme tarafı ayrı bir projede
([zehrakdemir/robot-arm-simulation](https://github.com/zehrakdemir/robot-arm-simulation)).
Bu repo yalnızca **kol** tarafıdır; ikisi
[`examples/vision_integration_example.py`](examples/vision_integration_example.py) üzerinden
konuşur.

## Önkoşullar

| Gereksinim | Not |
|---|---|
| Docker | Compose v2 ile. WSL2 üzerinde Docker Desktop ya da WSL içinde yerli docker engine |
| Disk | İmaj için ~8 GB |
| Robot | Açık, **WebApp'ten enable edilmiş** ve bilgisayarla aynı ağda ulaşılabilir |
| Gripper (kullanılacaksa) | Jodell EPG40-050 + WebApp'te `End_Jodell_V1.0` protokolü uygulanmış ve Configure edilmiş |

Kendi makinene ROS2/MoveIt2 kurmak istersen (Docker olmadan): **[`docs/SETUP.md`](docs/SETUP.md)**.

## Kurulum

### 1. Klonla

```bash
git clone -b huseyin https://github.com/zehrakdemir/RobotArmProject.git && cd RobotArmProject
```

### 2. Robotun IP adresini ver

```bash
cp .env.example .env
```

`.env` içine robotun IP adresini yaz:

```
FAIRINO_ROBOT_IP=192.168.57.2
```

> **Bu adım Docker tarafında pratikte zorunlu.** Container içinden ağ taraması genelde
> çalışmaz: robotun ağı container için yerel değil, yönlendirilmiş bir ağdır — tarama
> hiçbir şey bulamaz. Adres verilince tarama zaten hiç yapılmaz.
> Adresi bilmiyorsan robotun kendi WebApp adresi neyse odur.
>
> `.env` git'e **girmez** ([`.gitignore`](.gitignore)) — makineye özel.

### 3. İmajı derle ve başlat

```bash
docker compose up cobot
```

İlk derleme **~25-40 dakika** sürer (MoveIt2'nin `moveit_py` paketi ve Fairino sürücüsü
kaynaktan derleniyor). Sonraki açılışlar saniyeler.

### 4. Arayüzü aç

**<http://localhost:8088>**

Robot hazır değilse arayüz yine açılır ve **sebebini ekranda gösterir**; robot hazır olunca
kendiliğinden bağlanır — container'ı yeniden başlatmaya gerek yok.

Arka planda çalıştırmak için `docker compose up -d cobot`, logları izlemek için
`docker compose logs -f cobot`.

## Web arayüzü

`http://localhost:8088` — `movecobot`'un yapabildiği her şey, artı fazlası.

| Bölüm | Ne yapar |
|---|---|
| **E-STOP** | Süren yörüngeyi iptal eder ve SDK üzerinden `StopMotion` gönderir. Hareket kilidini beklemez. |
| **Canlı durum** | Eklem açıları, TCP pozu, fault kodları, gripper durumu — WebSocket ile akar |
| **Eklem kontrolü** | Her ekseni tek tek sür, ya da altı açıyı birden ver |
| **Jog** | Eklem ve TCP (araç eksenine göre) adımlı hareket |
| **Önizleme** | Hedefe gitmeden önce, ileri kinematikle TCP nerede olacak göster |
| **Gripper** | Aç/kapa, kuvvet ve hız ayarı, durum okuma |
| **Kayıtlı hareketler** | Listele, oynat, sil — `movecobot` ile aynı dosyaları kullanır |
| **Günlük** | O anki işin adım adım çıktısı, başarısızsa sebebi |

Arayüzde **kimlik doğrulaması yok** ve **gerçek robotu hareket ettirir**. Yalnızca güvendiğin
bir ağda aç; `8088` portunu internete açma.

## Terminal komutları

Container içinde:

```bash
docker compose exec cobot movecobot
```

| Komut | Ne yapar |
|---|---|
| `movecobot` | Kol + gripper kontrolü, hareket kaydetme/oynatma, pick & place |
| `movecobot --list` | Kayıtlı hareketleri listeler (robota bağlanmaz) |
| `movecobot --replay <ad>` | Kayıtlı hareketi oynatır |
| `konumcobot` | Sadece konum okur (movecobot çalışırken güvenli — ikinci bağlantı açmaz) |
| `gripper-gui` | Gripper için Tkinter arayüzü |
| `detect-robot` | Hangi robot bağlı, bulur ve yazdırır |

## Sorun mu var

```bash
docker compose run --rm verify
```

Ortamın doğru kurulduğunu **robota hiç dokunmadan** doğrular. Bir imaj "başarıyla" derlenip
işe yaramaz çıkabiliyor — bu betik öyle sessiz boşlukları yakalar.

Sonrası, ağ/RT/X ayarlarının gerekçeleri ve sık karşılaşılan hatalar:
[`docs/DOCKER.md`](docs/DOCKER.md).

## Docker olmadan (native)

Native kurulum hâlâ destekleniyor ve çalışıyor:

```bash
bash scripts/install.sh
```

```bash
python3 tools/apply_ros2_patches.py && cd ~/ros2_ws && colcon build
```

```bash
bash scripts/fairino_launch.sh
```

Sonra ayrı bir terminalde `movecobot`. Tam kurulum adımları, karşılaşılan sorunlar ve kök
sebepleri: [`docs/SETUP.md`](docs/SETUP.md).

## Geliştirme

```bash
docker compose run --rm test
```

Ya da yerelde:

```bash
pip install -r requirements-dev.txt && pytest -q && ruff check .
```

Testler robot, ROS2 veya SDK **gerektirmez** — `fairino_cobot` paketi alt modüllerini import
etmeyecek şekilde yazıldı, böylece temiz bir makinede de anlamlı doğrulama yapılabiliyor.
CI (`.github/workflows/ci.yml`) ayrıca vendor sürücüsünü klonlayıp yamaların gerçekten
uygulandığını doğrular.

## Belgeler

| Dosya | İçerik |
|---|---|
| [`docs/DOCKER.md`](docs/DOCKER.md) | Container hedefleri, ağ/RT/X ayarları ve gerekçeleri, simülatörün neden çalışmadığı |
| [`docs/SETUP.md`](docs/SETUP.md) | ROS2 Humble + MoveIt2 + SDK kurulumunun tamamı (native) |
| [`docs/ALIASES.md`](docs/ALIASES.md) | `~/.bashrc` alias listesi |
| [`docs/auto-launch.md`](docs/auto-launch.md) | Otomatik robot algılama/kurulum akışı |
| [`ros2/README.md`](ros2/README.md) | Vendor sürücüsüne uygulanan yamalar ve gerekçeleri |
| [`docs/journal/`](docs/journal/) | Günlük çalışma notları — **ölçülüp çürütülen teoriler dahil** |

`docs/journal/` özellikle işe yarar: aynı yanlış yola ikinci kez girmemek için, denenip
çalışmadığı **ölçümle** gösterilen yaklaşımlar da kayıt altında.

## Güvenlik notu

Planlı hareketler (jog / kayıtlı oynatma / web arayüzü) zemin ve self-collision kontrolünden
geçiyor, onay isteniyor. **Manuel mod (drag-teach)** ise robotu fiziksel olarak elle hareket
ettirdiği için yazılımsal collision koruması **içermiyor** — tek güvenlik ağı robotun kendi
donanım seviyesindeki çarpışma algılamasıdır. Acil stop elinin altında olsun.
