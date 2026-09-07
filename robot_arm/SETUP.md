# FAIRINO SimMachine + ROS2 Humble + MoveIt2 + pymoveit2 — WSL2 Kurulum Notları

## Ortam
- WSL2 üzerinde Ubuntu 22.04
- ROS2 Humble kurulu, `~/.bashrc` içine şu satırlar eklendi (her terminalde otomatik aktif):
  - `source /opt/ros/humble/setup.bash`
  - `source ~/ros2_ws/install/setup.bash`
- Docker native çalışıyor (`docker --version` ile doğrulanabilir)

## ÖNEMLİ DERS: Workspace Temizleyip Yeniden Derledikten Sonra Yeni Terminal Aç
`rm -rf build install log` + `colcon build` sonrası, **eskiden açık olan terminal
pencerelerinde** `export` ile ayarlanmış ortam değişkenleri (özellikle `LD_LIBRARY_PATH`)
hâlâ eski/geçersiz bir değere işaret ediyor olabilir — dosyaları düzenlemek/silmek, o an
açık bir terminalin hafızasındaki değeri geri almıyor. **Her büyük workspace değişikliğinden
sonra mutlaka yepyeni bir terminal penceresi aç**, aksi halde saatlerce "neden çalışmıyor"
diye debug edebilirsin (bugün tam bunu yaşadık).

## Dosya Konumları
- SimMachine docker image tar'ı: `~/fairino/simmachine/FAIRINOSimMachine.tar`
- Python SDK + demo script'ler + pymoveit2 test script'leri: `~/fairino/client/`
  - `movecobot.py` — ana birleşik kontrol/kayıt aracı (bkz. yukarıdaki "Güncel Durum")
  - `gripper_controller.py` — Jodell EPG40-050 gripper sürücüsü (adaptif `grip()` dahil)
  - `gripper_gui.py` — gripper için Tkinter arayüzü (movecobot içinden de otomatik açılıyor)
  - `konumcobot.py` — movecobot çalışırken robota ikinci bağlantı açmadan durum okuyan
    hafif "sadece göster" aracı
  - `fairino_controller.py`, `fairino_params.py` — düşük seviye SDK yardımcıları
- Başlat/durdur/launch script'leri: `~/fairino/run_simmachine.sh`, `~/fairino/stop_simmachine.sh`, `~/fairino/launch_moveit.sh`
- ROS2 workspace: `~/ros2_ws/src/frcobot_ros2/` — **`V3.0.0_RobotV3.9.3` tag'inde** (bkz. aşağı)

## GÜNCEL DURUM: Gerçek Robot (FR3, movecobot.py) — Ana Çalışma Şekli

Bu dosyanın geri kalanı, projenin SimMachine (Docker simülatörü) ile başlayan ilk kurulum
aşamasını anlatıyor ve hâlâ geçerli/doğru — ama proje o zamandan beri **gerçek fiziksel
FR3 kolu** (WSL'den `192.168.57.2` üzerinden erişilen, simülatördeki `192.168.58.2`'den
FARKLI bir cihaz) ile çalışacak şekilde ilerledi. Yeni bir kurulumda önce aşağıdakileri oku,
sonra gerekiyorsa simülatör bölümlerine geç.

### movecobot.py — birleşik interaktif kontrol + hareket kaydı aracı
`client/movecobot.py` — kolu + Jodell EPG40-050 gripper'ı TEK oturumda kontrol eden,
"öğret ve tekrar oynat" katmanlı ana araç. Robotu istediğin poza getirip adım olarak
ekleyip bir isimle kaydediyorsun; bu isim otomatik `~/.bashrc`'ye alias olarak yazılıyor,
sonra terminale sadece o ismi yazarak (ör. `pick1`) aynı hareketi tek komutla tekrar
oynatabiliyorsun.
```bash
movecobot                  # interaktif menü (jog + kayıt + oynatma + düzenleme)
movecobot --replay <isim>  # kayıtlı bir hareketi doğrudan, etkileşimsiz oynatır
movecobot --list           # kayıtlı hareketleri listeler
movecobot --clear-fault    # WebApp'te "Clear" hiçbir şey yapmıyormuş gibi görünüyorsa
```
Ön koşul: `fairino_launch` (ROS2 move_group + rviz) ayakta olmalı — movecobot kendisi
başlatmaz, çalışan bir MoveIt sunucusuna bağlanır.

Öne çıkan tasarım kararları:
- **Güvenlik ağı**: her planlı hareket zemin çarpışma kontrolü + mesafe-bazlı self-collision
  kontrolünden (12mm eşik, salt "değiyor mu" değil) geçiyor, uygulanmadan önce onay isteniyor,
  uygulandıktan sonra hedefe gerçekten ulaşılıp ulaşılmadığı SDK'dan (ROS2 üzerinden DEĞİL,
  doğrudan) doğrulanıyor. Jodell gripper URDF'de modellenmediği için `wrist3_link`'e sabit
  bir collision kutusu ekleniyor (bkz. `movecobot.py` içindeki `_add_gripper_collision`).
- **Adaptif kavrama**: `gripper.grip()` (bkz. `gripper_controller.py`) nesne büyüklüğüne göre
  gerektiği kadar kapanıp tutuyor (pozisyon olarak değil, "kavra" eylemi olarak kaydediliyor);
  hem `grippergui`'de hem `movecobot`'un hareket kaydında/düzenlemesinde kullanılabiliyor.
- **Manuel mod (drag-teach)**: robotu fiziksel olarak elle hareket ettirip pozları/kavramaları
  adım olarak eklemeyi sağlıyor — bu modda yazılımsal collision koruması YOK (planlama
  kullanılmıyor), tek güvenlik ağı robotun kendi tork-tabanlı algılaması.
- **Hareket kaydı dayanıklılığı**: her adımdan sonra taslak diske yazılıyor — robot bir fault
  verip launch'ı yeniden başlatman gerekse bile yarım kalan kayıt kaybolmuyor, devam
  ettirilebiliyor. Aynı arayüzle mevcut kayıtlı bir hareket de sonradan düzenlenebiliyor
  (adım ekle/sil/listele, bekleme adımı, adım-bazlı hız/ivme).

### "Shaft collision fault" — kök sebep ve çözüm (ÇÖZÜLDÜ)
Küçük (10°'nin altında) eklem hareketlerinde bile robotun tork-tabanlı çarpışma algılaması
sertçe devreye girip duruyordu — kullanıcı hiçbir yere ÇARPMADIĞINI doğruladı, yani gerçek
bir çarpışma değil, hareketin ani ivme geçişinden (jerk sınırsızlığından) kaynaklanan yanlış
pozitifti. Denenip **başarısız olan** yol: `AddRuckigTrajectorySmoothing`'i
`AddTimeOptimalParameterization`'ın YERİNE koymak — bu, planlamayı zamanlamasız/ham bir yola
düşürüyor, gerçek controller de bu yolu reddediyordu (robot hiç hareket etmiyordu). Kütüphane
sürümü de (0.9.2 → kaynaktan derlenen 0.14.0) denendi, fark etmedi — sorun sürüm değil,
kullanım şekliydi.

**Çözüm**: MoveIt'in resmi zaman-parametreleştirme kılavuzuna göre Ruckig, TOTG'nin YERİNE
değil ONUNLA BİRLİKTE kullanılmalı — TOTG önce hız/ivme parametrelerini hesaplıyor, Ruckig
(listede EN BAŞTA — adaptör zinciri dışarıdan içeriye çalışıyor) o zaten-zamanlanmış
trajectory üzerine jerk sınırı ekliyor. Bkz.
`~/ros2_ws/src/frcobot_ros2/fairino3_v6_moveit2_config/config/ompl_planning.yaml`
(`request_adapters` sırası + dosyadaki ayrıntılı yorum). Planlama-only diagnostikte
(0.1°-10° sweep) ve canlı robotta doğrulandı.

### WSL'in İnternet Erişimi YOK
WSL2 tarafında DNS çözümlemesi çalışmıyor (`/etc/resolv.conf` yok, `ping 8.8.8.8` %100 kayıp)
— GitHub gibi dış servislere WSL içinden ULAŞILAMIYOR, sadece LAN'daki robota (`192.168.57.2`)
erişim var. `git clone`/`push` gibi işlemler Windows tarafındaki (Git Bash/PowerShell) git ile
yapılmalı; WSL'deki dosyalar `\\wsl.localhost\Ubuntu-22.04\...` UNC yolundan Windows'a
kopyalanabilir.

## SimMachine Container'ı Başlatma
```bash
bash ~/fairino/run_simmachine.sh
```

### KRİTİK: Neden bu run flag'leri gerekli
İmajın içindeki başlangıç script'i, kendi ağ arayüzüne sabit `192.168.58.2` IP'sini elle
atamaya çalışıyor. Bu üç şey olmadan bu atama sessizce başarısız oluyor — container ayakta
görünüyor, Apache/WebApp (port 80) bile çalışıyor, AMA canlı-durum WebSocket'i (9998/9999)
hiç dinlemiyor ve WebApp modu (otomatik/manuel) hiç güncellenmiyor:
- `--privileged --cap-add=NET_ADMIN` → container'a kendi ağ arayüzünü yapılandırma izni verir
- `--network fairino-net --ip 192.168.58.2` → image'in beklediği IP'yi gerçekten atar

## WebApp
- Adres: http://127.0.0.1 (sonuna `:8080` gibi bir port EKLEME)
- Giriş: admin / 123

## Python SDK Kullanımı
```bash
cd ~/fairino/client
python3 move_robot.py              # tek hareket + geri dönüş demosu
python3 random_motion.py           # sürekli rastgele hareket (Ctrl+C ile durdur)
```
SDK pip ile kurulu değil — `fairino-python-sdk/linux/` klasörü `sys.path`'e ekleniyor.

Eklem açılarını / TCP pozunu doğrudan SDK'dan okuma:
```bash
python3 -c "
import sys
sys.path.insert(0, 'fairino-python-sdk/linux')
from fairino import Robot
robot = Robot.RPC('127.0.0.1')
print('Eklem açıları (derece):', robot.robot.GetActualJointPosDegree(0)[1:7])
print('TCP pozu (mm, derece):', robot.robot.GetActualTCPPose(0))
"
```

## ROS2 + MoveIt2 + fairino_hardware Kurulumu

**MoveIt2:**
```bash
sudo apt install ros-humble-moveit -y
```

**ros2_control:**
```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-controller-manager ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller -y
```

**frcobot_ros2 workspace — DOĞRU SÜRÜM ÖNEMLİ:**
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/FAIR-INNOVATION/frcobot_ros2.git
cd frcobot_ros2
git checkout V3.0.0_RobotV3.9.3
cd ~/ros2_ws
colcon build
```

> **NEDEN TAG ÖNEMLİ:** `main` branch'inde SimMachine'imizin sürümüne (v3.9.3) birebir denk
> gelen bir klasör yok, en yakın seçenekler (`fairino_hardware_v3_9_4` gibi) sürüm
> uyuşmazlığına yol açıyor — resmi Fairino dokümantasyonu bunun "veri tipi farkına" yol
> açabileceğini belirtiyor. **`V3.0.0_RobotV3.9.3` tag'i tam eşleşme sağlıyor** ve bizim
> uzun süre yaşadığımız "Running/Stopped" titremesini kalıcı olarak çözdü. Bu tag'de paket
> adı da sadeleşiyor: `fairino_hardware_v3_9_4` yerine artık düz `fairino_hardware`.

**Plugin satırı değişikliği** (mock_components yerine gerçek donanım eklentisi):
```bash
sed -i 's|<plugin>mock_components/GenericSystem</plugin>|<plugin>fairino_hardware/FairinoHardwareInterface</plugin>|' \
  ~/ros2_ws/src/frcobot_ros2/fairino5_v6_moveit2_config/config/fairino5_v6_robot.ros2_control.xacro
colcon build --packages-select fairino5_v6_moveit2_config
```

### ÇÖZÜLDÜ — libfairino.so.2 / LD_LIBRARY_PATH sorunu
Eski `fairino_hardware_v3_9_4` sürümünde `ros2_control_node`, `libfairino.so.2` kütüphanesini
bulamayıp çöküyordu, elle `LD_LIBRARY_PATH` eklemek gerekiyordu. **`V3.0.0_RobotV3.9.3`
tag'ine geçince bu sorun tamamen ortadan kalktı** — kütüphane artık standart
`install/fairino_hardware/lib/` yoluna düzgün yerleşiyor, ekstra path'e gerek yok.
`launch_moveit.sh`'ta bu satır artık gereksiz ama zararsız (dokunulmadı, script `bash` ile
çalıştırıldığı için dışarıyı etkilemiyor).

### ÇÖZÜLDÜ — RViz görüntü bozukluğu (WSLg + D3D12)
Kamera döndürülünce ekranda iz/karmaşa birikiyordu. Çözüm — `launch_moveit.sh` içinde hâlâ
aktif:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
```
(FPS düşüyor ama görüntü doğru çiziliyor.)

### SORUN: "Running/Stopped" durumu ikide bir değişiyor, hareket pürüzsüz değil

## launch_moveit.sh (güncel hali)
```bash
#!/bin/bash
if ! docker ps --filter "name=fairino-simmachine" --filter "status=running" | grep -q fairino-simmachine; then
  echo "SimMachine ayakta değil, başlatılıyor..."
  bash ~/fairino/run_simmachine.sh
  sleep 8
fi
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=$HOME/ros2_ws/src/frcobot_ros2/fairino_hardware_v3_9_4/libfairino/lib:$LD_LIBRARY_PATH
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```
(Not: LD_LIBRARY_PATH satırı artık gereksiz ama zararsız duruyor — bilerek dokunulmadı,
"Yol Y" prensibi: script kendi izole alt sürecinde çalıştığı için terminali kirletmiyor.)

## pymoveit2 Kurulumu
```bash
cd ~/ros2_ws/src
git clone https://github.com/AndrejOrsula/pymoveit2.git
cd ~/ros2_ws
sudo rosdep init   # zaten kuruluysa hata verir, sorun değil
rosdep update
rosdep install -y -r -i --rosdistro humble --from-paths src/pymoveit2
colcon build --packages-select pymoveit2
source install/setup.bash
```

Test script'leri: `~/fairino/client/pymoveit2_test.py` (eklem açısı hedefi) ve
`~/fairino/client/pymoveit2_pose_test.py` (Cartesian X,Y,Z + quaternion hedefi — FoundationPose'un
üreteceği veri tipine yakın). İkisi de doğrulandı, SDK ile okunan gerçek pozisyon hedefle
örtüştü.

## FR3'e Model Değiştirme (henüz yapılamadı)
SimMachine'in robot modelini FR3'e çevirmek WebApp → System Settings → Maintenance Mode →
Controller Compatibility üzerinden mümkün (değer "2" = FR3 V6.0), ama bu menü ayrı bir
şifre istiyor — hocadan alınması gerekiyor. `fairino3_v6_moveit2_config` paketi zaten
`~/ros2_ws/src/frcobot_ros2/` içinde hazır duruyor.

## Bilinen Tuhaflıklar
- `docker context ls` bazen `default` ve `desktop-linux` diye iki context gösterebiliyor —
  Docker komutlarını HER ZAMAN WSL'den çalıştır, PowerShell'den değil.
- `~/.docker/config.json` içindeki `credsStore: desktop.exe` satırı WSL içinden yeni image
  çekmeyi engelleyebiliyor (`~/.docker/config.json.bak` olarak yedeklendi/devre dışı).
- WSL/Docker'ı sıfırlamak gerekirse: `wsl --shutdown` (PowerShell'den), Docker Desktop'ı
  yeniden başlat, sonra `bash ~/fairino/launch_moveit.sh`.

## Reboot Sonrası Hızlı Başlangıç
```bash
bash ~/fairino/launch_moveit.sh   # SimMachine'i otomatik başlatır + RViz/MoveIt2'yi açar
```
Ayrı bir terminalde:
```bash
cd ~/fairino/client && python3 pymoveit2_pose_test.py
```

## YARIN YAPILACAKLAR (sırayla)

1. **moveit_py kurulumu** — resmi MoveIt2 Python API'sini de deneyelim (pymoveit2'ye ek/alternatif
   olarak, daha önce Humble'da stabil olmadığı için pymoveit2'yi seçmiştik, yine de bir dene
   yapılacak).
2. **Robotun yerin altına girmesi** — `add_floor.py` ile zemin çarpışma nesnesi ekleniyor ama
   MoveIt2 her yeniden başladığında bu bilgi sıfırlanıyor (kalıcı değil). Çözüm taslağı:
   `launch_moveit.sh`'a otomatik `add_floor.py` çağrısı eklemek (arka planda `ros2 launch`
   başlatıp, birkaç saniye bekleyip, sonra zemin eklemek) — taslak script hazır ama henüz test
   edilmedi.
3. **Hız hatası ("command speed... exceeds the limit")** — `joint_limits.yaml`'daki j4-j6 için
   düzeltme yapıldı (gerçek FR5 spesifikasyonuna, 180°/s = π rad/s'ye çekildi) ve bir noktada
   sorunsuz çalıştı, ama zemin düzeltmesi sonrası MoveIt2'nin ürettiği yollar değişebileceği
   için tekrar test edilecek.
4. **WebApp'teki hata "Clear" butonuna basınca temizlenmiyor** — `ros2_control_node` arka planda
   çalışmaya devam ederken alarm sürekli yeniden tetikleniyor gibi görünüyor; MoveIt2/launch
   terminalini kapatıp SDK'dan `ResetAllError()` çağırınca çalıştığı bir kez gözlemlendi ama
   kalıcı/otomatik bir çözüm henüz yok. 2 ve 3 düzelince bunun kendiliğinden azalıp azalmadığına
   bakılacak.

## moveit_py Kurulumu (Kaynak Koddan Derleme)

### Neden kaynak koddan
`ros-humble-moveit-py` diye bir apt paketi yok — `apt-cache search moveit-py` ile kontrol
edildi, sadece `ros-iron-moveit-py` ve `ros-rolling-moveit-py` bulundu. Tek yol MoveIt2'nin
tamamını kaynaktan derlemek.

### Kurulum adımları
```bash
mkdir -p ~/ws_moveit2/src && cd ~/ws_moveit2/src
git clone https://github.com/moveit/moveit2.git -b humble
for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do
  vcs import < "$repo"
done
sudo apt remove -y "ros-humble-moveit*"
cd ~/ws_moveit2
rosdep update
rosdep install -r --from-paths . --ignore-src --rosdistro humble -y
source /opt/ros/humble/setup.bash
export MAKEFLAGS="-j2"
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
```
(`--mixin release` yerine `--cmake-args -DCMAKE_BUILD_TYPE=Release` kullanıldı —
`colcon mixin add/update` denendi ama kayıt düzgün oturmadı, CMake parametresi
doğrudan aynı sonucu veriyor.)

### KRİTİK: WSL Bellek Sınırlaması Şart
Sınırsız paralel derleme (`colcon` varsayılanı, CPU çekirdek sayısı kadar iş başlatıyor),
~16GB RAM'li bu makinede **WSL'in tamamını kilitledi** (`wsl` komutu bile yanıt vermez hale
geldi, `wsl --shutdown` ile zorla kapatmak gerekti). Çözüm — Windows tarafında
`%USERPROFILE%\.wslconfig` dosyasına bellek sınırı eklemek + derleme sırasında
`export MAKEFLAGS="-j2"`. Bu ikisiyle derleme sorunsuz tamamlandı (46 paket, 46 dakika
25 saniye).

### Doğrulama
```bash
source ~/ws_moveit2/install/setup.bash
python3 -c "from moveit.planning import MoveItPy; print('OK')"
```

## moveit_py Launch Dosyası ve Test Script'leri

`demo.launch.py` (moveit_configs_utils'ın `generate_demo_launch()` fonksiyonu) `move_group`
node'unu da başlatıyor — bu bizim durumumuzda gereksiz/çakışan, çünkü `moveit_py` MoveIt2
çekirdeğini kendi process'i içinde çalıştırıyor. Bu yüzden özel bir launch dosyası yazıldı,
`move_group` hariç: `~/fairino/client/moveit_py_demo.launch.py` — içeriği
`moveit_configs_utils/launches.py` kaynak kodundaki `generate_demo_launch()` fonksiyonu
referans alınarak, `static_virtual_joint_tfs` + `rsp` + `ros2_control_node` (elle, aynı
`generate_demo_launch()`'taki gibi) + `spawn_controllers` + `moveit_rviz` içeriyor
(`move_group` hariç).

moveit_py config dosyası: `~/fairino/client/fairino5_moveit_py.yaml` (planning_scene_monitor
ve planning_pipelines ayarları).

Test script'i: `~/fairino/client/moveit_py_gohome.py` — `MoveItConfigsBuilder(...).moveit_cpp(
file_path=...)` ile `MoveItPy` başlatıyor, `get_planning_component()` ile plan/execute
yapıyor. `execute()` fonksiyonunun doğru imzası: `execute(group_name: str, robot_trajectory,
blocking=True)` — `(trajectory, controllers=[])` DEĞİL, ilk denemede bu yüzden hata alındı.

### Bilinen: RViz'de robot modeli görünmüyor
`moveit_rviz.launch.py` launch dosyasına eklendi, RViz açılıyor ama robot modeli
görünmüyor — çünkü `move_group` olmadığı için `moveit.rviz` config'indeki "RobotModel"
display'i besleyen kaynak yok. Elle "RobotModel" display eklenip `robot_description`
parametresi ayarlanırsa muhtemelen düzelir, ama SDK ve WebApp ile doğrulama yeterli
görüldüğü için bu ayrıca çözülmedi.

### Bilinen: "Currently in a singular pose" uyarısı
Robotu `[0,0,0,0,0,0]` (tüm eklemler sıfır derece) hedefine hareket ettirince WebApp'te bu
uyarı çıktı — `[0,0,0,0,0,0]`'da j5=0° olduğu için klasik bir bilek tekilliği (wrist
singularity) oluşuyor. `ResetAllError()` ile SDK'dan temizlemeye çalışıldı, `Reset: 0`
(başarılı) döndü ama uyarı WebApp'te kalmaya devam etti. Robot güvenli bir hedefe
(`[0,-90,90,-90,-90,0]`) hareket ettirilince uyarı **kendiliğinden** kayboldu — yani bu bir
"hata bayrağı" değil, robotun o anki gerçek pozisyonuna bağlı canlı bir durum göstergesi.

## Kısayollar (Güncel — moveit_py)
```bash
cat >> ~/.bashrc << 'ALIASEOF'
alias launch_moveit_py='source /opt/ros/humble/setup.bash && source ~/ws_moveit2/install/setup.bash && source ~/ros2_ws/install/setup.bash && export LD_LIBRARY_PATH=$HOME/ros2_ws/src/frcobot_ros2/fairino_hardware/lib:$LD_LIBRARY_PATH && export LIBGL_ALWAYS_SOFTWARE=1 && ros2 launch ~/fairino/client/moveit_py_demo.launch.py'
alias gohome_py='source /opt/ros/humble/setup.bash && source ~/ws_moveit2/install/setup.bash && source ~/ros2_ws/install/setup.bash && python3 ~/fairino/client/moveit_py_gohome.py'
alias fairino_params='python3 ~/fairino/client/fairino_params.py'
ALIASEOF
```
Eski `pymoveit2` tabanlı alias'lar (`gohome`, `runpose`, `addfloor`) kaldırıldı.

## KRİTİK: Login Shell / .bashrc Sorunu
`wsl` komutuyla açılan terminaller "login shell" olarak başlıyor ve `.bashrc`'yi otomatik
okumuyor (sadece `.bash_profile` varsa ve onu çağırıyorsa okunuyor). Bu yüzden alias'lar ve
`source /opt/ros/humble/setup.bash` gibi satırlar bazı terminallerde hiç aktif olmuyordu.
Kalıcı çözüm:
```bash
cat > ~/.bash_profile << 'EOF'
if [ -f ~/.bashrc ]; then
    . ~/.bashrc
fi

## RX/RY/RZ Açı Konvansiyonu (Deneysel Doğrulama)

Fairino'nun resmi dokümantasyonunda RX/RY/RZ'nin hangi konvansiyonu (sırayı) kullandığı
açıkça yazmıyor. Kullanıcı, WebApp'in jog panelinden tek tek her ekseni test edip TCP'nin
hangi yöne hareket ettiğini gözlemleyerek deneysel olarak belirledi:

Test verileri (0,0,0 referans alınarak):
- rx=90,ry=0,rz=0: X ileri, Y yukarı, Z sağa
- rx=0,ry=90,rz=0: X aşağı, Y sola, Z ileri
- rx=0,ry=0,rz=90: X sola, Y geri, Z yukarı
- rx=90,ry=90,rz=0: X aşağı, Y ileri, Z sağa
- rx=90,ry=0,rz=90: X sola, Y yukarı, Z ileri
- rx=90,ry=90,rz=90: X aşağı, Y sola, Z ileri
- rx=0,ry=90,rz=90: HATA verdi (gimbal lock — Pitch=90°'de klasik bir kilitlenme noktası)

Sonuç: Konvansiyon **extrinsic ZYX** (= intrinsic XYZ) — önce Roll(X), sonra Pitch(Y),
sonra Yaw(Z), sabit/dünya eksenlerine göre. `moveit_py_interactive.py`'deki
`euler_deg_to_quat()` fonksiyonu bu konvansiyonu kullanıyor, veriyle tutarlı.

## wrist3_link ile Fairino TCP Arasındaki Sabit Ofset (ÇÖZÜLDÜ)

Cartesian pose hedefleri gönderirken, MoveIt2'ye verilen Z ile SDK'nın okuduğu gerçek Z
arasında tutarsız görünen bir fark vardı — bazen +100mm, bazen -100mm. Sistematik testlerle
(aynı X,Y,Z'ye farklı RX değerleriyle giderek) örüntü bulundu: fark, **RX'e göre cos(RX)**
oranında değişiyor (RX=0'da +100mm, RX=180'de -100mm).

Kök sebep: `wrist3_link` (URDF/MoveIt2'nin hedef aldığı nokta) ile Fairino SDK'nın "TCP"
dediği nokta arasında, **aracın kendi lokal Z ekseni boyunca sabit ~100mm mesafe var**.
Robot döndükçe bu lokal mesafe global eksenlere farklı yansıyor.

Çözüm — `moveit_py_interactive.py` içinde, Cartesian hedef göndermeden önce, istenen
oryantasyonun quaternion'undan aracın lokal Z ekseninin global yönünü hesaplayıp, hedefi
buna göre düzeltiyoruz:
```python
local_z_in_global = [
    2 * (qx * qz + qy * qw),
    2 * (qy * qz - qx * qw),
    1 - 2 * (qx * qx + qy * qy),
]
TCP_OFFSET_M = 0.1
wrist3_x = tx/1000 - TCP_OFFSET_M * local_z_in_global[0]
wrist3_y = ty/1000 - TCP_OFFSET_M * local_z_in_global[1]
wrist3_z = tz/1000 - TCP_OFFSET_M * local_z_in_global[2]
```
Öz-tutarlılık testiyle doğrulandı: robotun mevcut TCP pozunu (fairino_params'tan okunan)
aynen hedef olarak verince, düzeltmeden önce ~100mm fark çıkıyordu, düzeltmeden sonra
TCP hatası **~0.04mm**'ye düştü.

## AÇIK SORU: Cartesian Hedeflerde Beklenmedik Eklem Konfigürasyonu

Yukarıdaki öz-tutarlılık testinde TCP pozisyonu neredeyse mükemmel eşleşse de, **eklem
açıları büyük ölçüde değişti** (J2 ~86°, J3 ~180°, J4 ~94° fark) — yani robot aynı TCP
noktasına, çok farklı bir kol duruşuyla gitti.

MoveIt2 kaynak kodunda (`constraint_samplers/default_constraint_samplers.cpp`,
`IKConstraintSampler::sampleHelper`) incelendi: IK çözücü ilk denemede (`a==0`) mevcut
durumu seed olarak kullanıyor (`use_as_seed=true`), sadece bu deneme başarısız olursa
sonraki denemelerde rastgele seed'e düşüyor. Zemin çarpışma nesnesinin bu "ilk denemeyi
reddetme" davranışına sebep olup olmadığı test edildi (collision nesnesi geçici olarak
kaldırılıp aynı test tekrarlandı) — **hipotez çürüdü**, collision nesnesi olsun olmasın
aynı büyük sıçrama gerçekleşti. Kök sebep tam olarak bulunamadı, açık soru olarak kalıyor.

**Pratik sonuç:** Cartesian poz hedefleri minimal hareketi garanti etmiyor. Minimal hareket
gerekiyorsa Joint modu (doğrudan eklem hedefi) kullanılmalı.

## Zemin Çarpışmadan Kaçınma (ÇALIŞIYOR)

`moveit_py_interactive.py` içinde `add_floor_collision()` fonksiyonu, planlama öncesi
gerçek bir collision nesnesi ekliyor (`get_planning_scene_monitor().read_write()` +
`apply_collision_object()`) — ilk denemede hatasız çalıştı, planlayıcı zeminden kaçınan
alternatif yollar buluyor. Ek olarak `check_trajectory_for_floor()` fonksiyonu, planlanan
yolun her noktasında (`trajectory[i]` — RobotTrajectory Python listesi gibi indekslenebilir
davranıyor, `len()` ve `[i]` destekliyor) tüm linklerin (`shoulder_link` ... `wrist3_link`)
pozisyonunu kontrol edip yedek bir güvenlik katmanı sağlıyor.

## moveit_py_interactive.py (movepy) — Güncel Özellikler

- Hareket tipi: Joint (eklem açıları) / Cartesian (X,Y,Z+RX,RY,RZ)
- Cartesian için referans: Base (mutlak) / Tool (mevcut poza göre, basitleştirilmiş) / Wobj
  (henüz tanımlı değil, seçilirse uyarı verip çıkıyor)
- Hız/ivme yüzdesi sorusu (0-100)
- TCP offset düzeltmesi (yukarıda açıklanan cos(RX) telafisi)
- Zemin çarpışmadan kaçınma (collision nesnesi + yedek kontrol)
- Çalıştırmadan önce onay adımı
- Sonucu SDK ile otomatik doğrulama (eklem açıları + TCP pozu)

## Yardımcı Script'ler ve Kısayollar (Bugün Eklenen)

- `fairino_params.py` / alias `fairino_params` — şu anki eklem açılarını ve TCP pozunu
  temiz formatta yazdırır
- `fixweb.sh` / alias `fixweb` — docker daemon çökmesi / image kaybı / container port
  kopması senaryolarının hepsini otomatik teşhis edip sırayla düzeltir:
```bash
  if ! docker ps > /dev/null 2>&1; then sudo dockerd & sleep 5; fi
  if ! docker images | grep -q fairino_simmachine; then docker load -i ...; fi
  if container varsa: docker restart, yoksa: run_simmachine.sh
  sleep 10 && curl ile dogrula
```

## Bugün Sık Karşılaşılan Docker/WebApp Sorunları (Kalıcı Çözülmedi, Tekrar Edebilir)
- Docker daemon zaman zaman kendiliğinden duruyor (`sudo dockerd &` ile elle başlatmak
  gerekiyor — WSL'de systemd olmadığı için otomatik yeniden başlamıyor)
- Container ayaktayken bile WebApp'e port yönlendirmesi bazen kopuyor (`docker restart`
  ile düzeliyor)
- Bu iki durumu ayırt edip otomatik düzeltmek için `fixweb` yazıldı, ama **kök sebep**
  (neden bu kadar sık oluyor) hâlâ bilinmiyor — muhtemelen WSL2 kaynak baskısıyla
  (bellek/CPU) ilgili, kesin değil.
