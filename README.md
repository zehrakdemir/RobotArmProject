# robot-arm-simulation — Fairino FR3 kol kontrolü

Fairino FR3 iş birlikçi kolunu **ROS2 Humble + MoveIt2** üzerinden süren araçlar: hareket
öğretme/kaydetme/oynatma, adaptif kavrama yapan gripper sürücüsü ve MoveIt Task Constructor
tabanlı pick & place.

Bu dal (`huseyin`) projenin **kol** tarafıdır; görüntü işleme tarafı `main` dalındaki
`vision/` klasöründedir.

```
Robot algılama  ──►  ROS2 sürücü + MoveIt2  ──►  movecobot  ──►  gripper (Modbus RTU)
(ağ taraması)        (frcobot_ros2, yamalı)     (planla/onayla/    (Jodell EPG40-050)
                                                 çalıştır/doğrula)
```

Öne çıkanlar:

- **Hiçbir IP kodda sabit değil.** `detect-robot` ağı tarayıp hangi model/sürüm robotun bağlı
  olduğunu bulur, doğru sürücü kaynağını kurar ve derler. İstersen `.env` ile IP verip
  taramayı tamamen atlarsın.
- **Çok katmanlı güvenlik.** Planlanan her yol zemin ve self-collision kontrolünden geçer,
  onay istenir, çalıştıktan sonra SDK üzerinden hedefe varıldığı doğrulanır.
  *(Manuel mod / drag-teach hariç — orada tek güvenlik ağı robotun kendi donanım seviyesindeki
  çarpışma algılamasıdır.)*
- **Vendor yamaları yeniden uygulanabilir.** `frcobot_ros2` üzerindeki düzeltmeler tek bir
  komutla, desteklenen **her** Fairino modeline uygulanır.

## Önkoşullar

| Gereksinim | Sürüm / not |
|---|---|
| Ubuntu | 22.04 (WSL2 üzerinde de olur) |
| ROS2 | Humble |
| MoveIt2 | 2.5.9 — **kaynaktan derlenmiş olmalı**, `moveit_py` apt'te yok |
| MoveIt Task Constructor | `ros-humble-moveit-task-constructor-core` (pick & place için) |
| Python | 3.10+ |
| Fairino Python SDK | [FAIR-INNOVATION/fairino-python-sdk](https://github.com/FAIR-INNOVATION/fairino-python-sdk) — PyPI'de yok, ayrıca klonlanır |
| Fairino ROS2 sürücüsü | [FAIR-INNOVATION/frcobot_ros2](https://github.com/FAIR-INNOVATION/frcobot_ros2), `V3.0.0_RobotV3.9.3` tag'i |
| Gripper (kullanılacaksa) | Jodell EPG40-050 + WebApp'te `End_Jodell_V1.0` protokolü uygulanmış ve Configure edilmiş |

ROS2/MoveIt2 kurulumunun tamamı: **[`docs/SETUP.md`](docs/SETUP.md)**.

## Kurulum

```bash
git clone https://github.com/zehrakdemir/robot-arm-simulation.git
```

```bash
cd robot-arm-simulation && git checkout huseyin && bash scripts/install.sh
```

`install.sh` paketi kurar, `.env` dosyasını oluşturur ve çözülen yolları (SDK, durum dizini,
robot IP) ekrana basar — bir şey bulunamazsa nereye bakıldığını da söyler.

Ardından ROS2 sürücüsüne yamaları uygula:

```bash
python3 tools/apply_ros2_patches.py && cd ~/ros2_ws && colcon build
```

### Ayarlar

Her şey `.env` ile, hepsi isteğe bağlı ([`.env.example`](.env.example)):

```bash
cp .env.example .env
```

| Değişken | Ne işe yarar |
|---|---|
| `FAIRINO_ROBOT_IP` | Verilirse ağ taraması hiç yapılmaz |
| `FAIRINO_SDK_DIR` | Fairino Python SDK'nın yeri |
| `FAIRINO_ROS2_WS` | `frcobot_ros2`'nin kurulu olduğu colcon workspace'i |
| `FAIRINO_STATE_DIR` | Kayıtlı hareketlerin tutulduğu dizin |

## Çalıştırma

Önce MoveIt2'yi başlat (robotu algılar, doğru paketi kurar/derler, rviz'i açar):

```bash
bash scripts/fairino_launch.sh
```

Sonra ayrı bir terminalde:

```bash
movecobot
```

| Komut | Ne yapar |
|---|---|
| `movecobot` | Kol + gripper kontrolü, hareket kaydetme/oynatma, pick & place |
| `konumcobot` | Sadece konum okur (movecobot çalışırken güvenli — robota ikinci bağlantı açmaz) |
| `gripper-gui` | Gripper için Tkinter arayüzü |
| `detect-robot` | Hangi robot bağlı, bulur ve yazdırır |

## Docker

```bash
docker compose run --rm test
```

Tam çalışma ortamı (ROS2 + MoveIt2 + sürücü) da var ama **desteklenen yol native kurulumdur**;
container tarafı gerçek robotla henüz doğrulanmadı. Gerekçeler ve zorunlu host ayarları
(host network, RT önceliği, X soketi): [`docs/DOCKER.md`](docs/DOCKER.md).

## Geliştirme

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
| [`docs/SETUP.md`](docs/SETUP.md) | ROS2 Humble + MoveIt2 + SDK kurulumunun tamamı, karşılaşılan sorunlar ve kök sebepleri |
| [`docs/DOCKER.md`](docs/DOCKER.md) | Container hedefleri ve zorunlu host ayarları |
| [`docs/ALIASES.md`](docs/ALIASES.md) | `~/.bashrc` alias listesi |
| [`docs/auto-launch.md`](docs/auto-launch.md) | Otomatik robot algılama/kurulum akışı |
| [`ros2/README.md`](ros2/README.md) | Vendor sürücüsüne uygulanan yamalar ve gerekçeleri |
| [`docs/journal/`](docs/journal/) | Günlük çalışma notları — **ölçülüp çürütülen teoriler dahil** |

`docs/journal/` özellikle işe yarar: aynı yanlış yola ikinci kez girmemek için, denenip
çalışmadığı **ölçümle** gösterilen yaklaşımlar da kayıt altında.

## Güvenlik notu

`movecobot.py`'deki planlı hareketler (jog / kayıtlı oynatma) zemin + self-collision
kontrolünden geçiyor ve onay istiyor. **Manuel mod (drag-teach)** ise robotu fiziksel olarak
elle hareket ettirdiği için yazılımsal collision koruması **içermiyor** — tek güvenlik ağı
robotun kendi donanım seviyesindeki çarpışma algılamasıdır. Acil stop elinin altında olsun.
