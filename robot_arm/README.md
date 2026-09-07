# robot_arm — Fairino FR3 kol kontrolü

`robot-arm-simulation` projesinin kol-kontrol tarafı: `vision/` klasöründeki tespit/parça
tanıma çıktısını kullanarak Fairino FR3 kolektif kolu (ROS2 Humble + MoveIt2 + Fairino
Python SDK üzerinden) hareket ettiren araçlar.

## Hızlı bakış

- **Kurulum**: [`SETUP.md`](SETUP.md) — WSL2 + ROS2 Humble + MoveIt2 + Fairino SDK kurulumunun
  tamamı, ayrıca "Güncel Durum" bölümünde gerçek robotla çalışma şekli.
- **Ana araç**: [`client/movecobot.py`](client/movecobot.py) — kol + gripper'ı tek oturumda
  kontrol eden, hareket kaydedip isimle (`~/.bashrc` alias'ı olarak) tekrar oynatabilen
  interaktif araç. `movecobot --help` ile kullanım.
- **Gripper**: [`client/gripper_controller.py`](client/gripper_controller.py) (Jodell
  EPG40-050 sürücüsü, nesne büyüklüğüne göre adaptif kavrama) +
  [`client/gripper_gui.py`](client/gripper_gui.py) (Tkinter arayüz).
- **ROS2/MoveIt2 config değişiklikleri**: [`ros2_config_patches/`](ros2_config_patches/) —
  vendor `frcobot_ros2` reposunun üzerine uygulanacak override'lar (ör. jerk/collision
  false-positive düzeltmesi).
- **Otomatik robot algılama + kurulum**: [`auto_launch/`](auto_launch/) — `fairino_launch`
  komutunun çalıştırdığı script'ler: hangi model/sürüm robot bağlıysa otomatik bulur, doğru
  kaynağı kurar/yamalar/derler, MoveIt2'yi başlatır.

## Bağımlılıklar (bu repoya dahil DEĞİL — `.gitignore`'a bak)

- `client/fairino-python-sdk/` — resmi SDK, `SETUP.md`'deki adımlarla ayrıca klonlanmalı
  (`git clone https://github.com/FAIR-INNOVATION/fairino-python-sdk.git`).
- `~/ros2_ws/src/frcobot_ros2/` — resmi ROS2 sürücüsü, `SETUP.md`'deki tag ile ayrıca
  klonlanmalı (`FAIR-INNOVATION/frcobot_ros2`, `V3.0.0_RobotV3.9.3` tag'i).
- FAIRINO SimMachine Docker imajı (simülatörle çalışmak isteyenler için, gerçek robotla
  çalışırken gerekmez) — `SETUP.md`'deki SimMachine bölümüne bak.

## Güvenlik notu

`movecobot.py`'deki planlı hareketler (jog/kayıtlı oynatma) zemin + self-collision
kontrolünden geçiyor ve onay istiyor; **manuel mod (drag-teach)** ise robotu fiziksel
olarak elle hareket ettirdiği için yazılımsal collision koruması İÇERMİYOR — tek güvenlik
ağı robotun kendi donanım seviyesindeki çarpışma algılaması. Ayrıntılar `SETUP.md`'de.
