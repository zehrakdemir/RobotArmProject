# FAIRINO SimMachine + ROS2 Humble + MoveIt2 + moveit_py — WSL2 Kurulum Notları

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
diye debug edebilirsin.

## Dosya Konumları
- SimMachine docker image tar'ı: `~/fairino/simmachine/FAIRINOSimMachine.tar`
- Python SDK + demo script'ler + moveit_py test script'leri: `~/fairino/client/`
  - `movecobot.py` — ana birleşik kontrol/kayıt aracı (bkz. aşağıdaki "Güncel Durum")
  - `gripper_controller.py` — Jodell EPG40-050 gripper sürücüsü (adaptif `grip()` dahil)
  - `gripper_gui.py` — gripper için Tkinter arayüzü (movecobot içinden de otomatik açılıyor)
  - `konumcobot.py` — movecobot çalışırken robota ikinci bağlantı açmadan durum okuyan
    hafif "sadece göster" aracı
  - `fairino_controller.py`, `fairino_params.py` — düşük seviye SDK yardımcıları
  - `vision_integration_example.py` — vision tarafının (bkz. GitHub'daki `vision/` klasörü)
    tespit ettiği 6D pozu `FairinoController.move_to_pose()`'a nasıl vereceğini gösteren
    entegrasyon iskeleti (şu an uydurma/stub veriyle)
- Otomatik robot algılama + kurulum script'leri: `~/detect_robot.py`, `~/setup_fairino_target.py`
  — bkz. "fairino_launch — Otomatik Robot Algılama ve Kurulum" bölümü
- Başlat/durdur/launch script'leri: `~/fairino/run_simmachine.sh`, `~/fairino/stop_simmachine.sh`, `~/fairino/launch_moveit.sh`, `~/auto_fairino_launch.sh`
- ROS2 workspace: `~/ros2_ws/src/frcobot_ros2/` — **`V3.0.0_RobotV3.9.3` tag'inde** (bkz. aşağı)

## GÜNCEL DURUM: Gerçek Robot (FR3, movecobot.py) — Ana Çalışma Şekli

Bu dosyanın geri kalanı, projenin SimMachine (Docker simülatörü) ile başlayan ilk kurulum
aşamasını anlatıyor ve hâlâ geçerli/doğru — ama proje o zamandan beri **gerçek fiziksel
FR3 kolu** (WSL'den `192.168.57.2` üzerinden erişilen, simülatördeki `192.168.58.2`'den
FARKLI bir cihaz) ile çalışacak şekilde ilerledi. Yeni bir kurulumda önce aşağıdakileri oku,
sonra gerekiyorsa simülatör bölümlerine geç.

Kol kontrolü **moveit_py** (resmi MoveIt2 Python API'si, `moveit.planning.MoveItPy`) üzerine
kurulu — bu bilinçli bir tercih (bkz. "moveit_py Kurulumu" bölümü: `pymoveit2` daha önce
denenmiş ama moveit_py'ye geçilmiş ve öyle kalınmış durumda). `movecobot.py` içindeki
`CobotArm` sınıfı `MoveItPy` + `PlanningComponent` + `PlanRequestParameters` + doğrudan
`PlanningSceneMonitor`/`RobotState` erişimini kullanıyor — çarpışma korumaları, FK önizleme
ve adım-bazlı hız/ivme özelliklerinin hepsi bu API'ye dayanıyor.

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
başlatmaz, çalışan bir MoveIt sunucusuna bağlanır (bkz. "fairino_launch" bölümü).

Ana menü (1-9): Kolu hareket ettir / Gripper'i hareket ettir / Şu anki durumu göster /
Yeni hareket kaydet / **Kayıtlı bir hareketi düzenle** / Kayıtlı bir hareketi oynat /
Kayıtlı hareketleri listele / Kayıtlı bir hareketi sil / Çıkış.

Öne çıkan tasarım kararları:
- **Güvenlik ağı**: her planlı hareket zemin çarpışma kontrolü + mesafe-bazlı self-collision
  kontrolünden (12mm eşik, salt "değiyor mu" değil) geçiyor, uygulanmadan önce onay isteniyor,
  uygulandıktan sonra hedefe gerçekten ulaşılıp ulaşılmadığı SDK'dan (ROS2 üzerinden DEĞİL,
  doğrudan) doğrulanıyor. Jodell gripper URDF'de modellenmediği için `wrist3_link`'e sabit
  bir collision kutusu ekleniyor (bkz. `movecobot.py` içindeki `_add_gripper_collision`).
- **Joint modunda FK önizleme**: eklem açılarıyla jog ederken (mutlak ya da göreceli), "bu
  hedefe uygulansın mı?" onayından ÖNCE, girilen açıların forward-kinematics ile karşılığı
  olan X/Y/Z/RX/RY/RZ hesaplanıp ekrana basılıyor (`CobotArm.predict_tcp_pose` +
  `quat_to_euler_deg`) — RX/RY/RZ'yi Joint modunda AYRICA girdirmek mümkün değil (6 eklem
  zaten TCP pozunu FK ile tek anlamlı belirliyor, aşırı-belirlenmiş olurdu), bu yüzden hareket
  etmeden önce nereye/ne yönelime gidileceğini görmek için önizleme eklendi.
- **Adaptif kavrama**: `gripper.grip()` (bkz. `gripper_controller.py`) nesne büyüklüğüne göre
  gerektiği kadar kapanıp tutuyor (pozisyon olarak değil, "kavra" eylemi olarak kaydediliyor);
  hem `grippergui`'de hem `movecobot`'un hareket kaydında/düzenlemesinde kullanılabiliyor.
- **Manuel mod (drag-teach)**: robotu fiziksel olarak elle hareket ettirip pozları/kavramaları
  adım olarak eklemeyi sağlıyor — bu modda yazılımsal collision koruması YOK (planlama
  kullanılmıyor), tek güvenlik ağı robotun kendi tork-tabanlı algılaması.
- **Hareket kaydı dayanıklılığı**: her adımdan sonra taslak diske yazılıyor — robot bir fault
  verip launch'ı yeniden başlatman gerekse bile yarım kalan kayıt kaybolmuyor, devam
  ettirilebiliyor. Aynı taslak mekanizması yarım kalan bir DÜZENLEMEyi de korur.
- **Kayıtlı bir hareketi düzenleme** (`edit_saved_move`, ana menü 5): mevcut bir kaydı seçip
  aynı adım-ekleme arayüzüyle (adım ekle/sil/listele, bekleme adımı, adım-bazlı hız/ivme)
  değiştirip üzerine kaydediyor. Düzenleme sırasında dizi bir DERİN KOPYA üzerinde tutuluyor —
  `[i] iptal` seçilirse orijinal kayıt hiç değişmeden kalıyor.
- **Bekleme adımı**: hareket/kavrama içermeyen, sadece oynatma sırasında N saniye bekleyen bir
  adım tipi (ör. kavrama sonrası nesnenin oturması için).
- **Adım-bazlı hız/ivme**: her hareket adımı kendi hız/ivme yüzdesini taşıyabiliyor; oynatma
  sırasında o TEK adım için `moveit.planning.PlanRequestParameters` ile oturumun genel hızı
  geçici olarak ezilip uygulanıyor (`CobotArm._plan_and_execute`) — eski kayıtlar/jog hareketi
  bu parametre verilmediğinde eskisi gibi davranmaya devam ediyor (geriye dönük uyumlu).
- **`gripper_gui.py` — "Pozisyonu Robottan Oku"**: pencere açılışında ve istenildiğinde bu
  düğmeyle, slider'ın HEDEF değeri gripper'ın GERÇEK pozisyonuyla senkronize ediliyor — eskiden
  slider hep 0'dan başlıyordu, yani gripper örneğin 80'deyken pencereyi açıp dokunmadan
  "Uygula"ya basmak onu beklenmedik şekilde 0'a (tam açık) götürebiliyordu.

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

**ÖNEMLİ — TEK modelle sınırlı kalmasın**: `frcobot_ros2`, desteklediği HER Fairino modeli
için ayrı bir `fairino<N>_v6_moveit2_config` paketi içeriyor ve `movecobot.py` hangi robot
bağlıysa onu otomatik algılayıp (`detect_robot()`) doğru paketi seçiyor — yani bu düzeltme
SADECE `fairino3_v6_moveit2_config`'e eklenirse, başka bir model (FR5, FR10, ...) takıldığında
düzeltmesiz/varsayılan (jerk sınırsız) config'e geri düşülür. Bu yüzden aynı
`ompl_planning.yaml` bulunan HER `fairino*_v6_moveit2_config/config/` klasörüne kopyalanmalı —
bkz. `robot_arm/ros2_config_patches/apply_patch.sh` (GitHub reposundaki genel uygulama
script'i, tüm mevcut model paketlerini tek seferde günceller).

### ÇÖZÜLDÜ — "command speed... exceeds the limit" hatası (joint_limits.yaml ivme eksikliği)
`joint_limits.yaml`'da (her model paketi için) tüm eklemlerde `has_acceleration_limits: false`
idi — yani MoveIt'in `time_optimal_trajectory_generation` adımı ölçeklenecek bir
`max_acceleration` BULAMIYOR, kendi iç sabit varsayılanını (1 rad/s²) kullanıyordu. Bu
varsayılan, movecobot'un sorduğu "ivme %" (`accel_pct` /
`max_acceleration_scaling_factor`) ile ÖLÇEKLENMİYORDU — kullanıcı hangi ivme yüzdesini
seçerse seçsin robota hep AYNI (ölçeklenmemiş) ivme komut ediliyordu. Büyük, tek-eklemli
geniş dönüşlerde (ör. J6'da 140°+) bu, "command speed exceeds the limit" hatasına yol açtı.

**Çözüm**: `has_acceleration_limits: true` + `max_acceleration: 6.2831` (=360°/s²) — FR3 resmi
kullanıcı kılavuzundaki ("full performance" bölümü) %100 hız/360°/s² ivme tavanı referans
alındı; movecobot'un `accel_pct`'i bu tavanı aşağı doğru ölçekliyor (tıpkı WebApp'teki hız
yüzdesinin bu tavanı ölçeklemesi gibi). Kılavuz, 3-5kg genişletilmiş yük taşırken bunun ALTINDA
bir ivme önerir — ağır bir nesne taşırken `accel_pct`'i düşük (%15-20 gibi) tutmak hâlâ
mantıklı. Değişiklik `fairino3_v6_moveit2_config` VE `fairino5_v6_moveit2_config`'in
`joint_limits.yaml`'ında mevcut (diğer modellere henüz uygulanmadı — gerekirse aynı mantıkla
eklenmeli).

### WSL'in İnternet Erişimi YOK
WSL2 tarafında DNS çözümlemesi çalışmıyor (`/etc/resolv.conf` yok, `ping 8.8.8.8` %100 kayıp)
— GitHub gibi dış servislere WSL içinden ULAŞILAMIYOR, sadece LAN'daki robota (`192.168.57.2`)
erişim var. `git clone`/`push` gibi işlemler Windows tarafındaki (Git Bash/PowerShell) git ile
yapılmalı; WSL'deki dosyalar `\\wsl.localhost\Ubuntu-22.04\...` UNC yolundan Windows'a
kopyalanabilir. Aynı sebeple `~/fairino_official_mirror` (bkz. aşağı) da internetten değil,
Windows tarafında indirilip WSL'e kopyalanan bir arşivden geliyor.

### ÇÖZÜLDÜ — Manuel mod sırasında `clearfault` işe yaramıyordu
Manuel (drag-teach) modda robot fault verirse (ör. elle hızlı hareket ettirince "hız limiti
aşıldı"), AYRI bir terminalde `movecobot --clear-fault` (ya da `clearfault` alias'ı)
çalıştırmak işe yaramıyordu. Kök sebep: o komut `ros2_control`'u YENİDEN AKTİVE ediyor, ama
drag-teach zaten `ros2_control`'u BİLİNÇLİ olarak inaktif bırakıp robotu SDK'nin kendi
`Mode(1)+DragTeachSwitch(1)`'iyle elle-itilebilir hale getiriyor — iki kontrol mekanizması
çakışıyor, robot ya tekrar elle-itilemez oluyor ya hata hemen geri geliyordu.

**Çözüm**: `drag_teach_clear_fault()` — `ros2_control`'a hiç dokunmadan, manuel mod için zaten
açık olan GEÇİCİ bağlantı üzerinden `ResetAllError()` + `Mode(1)`/`DragTeachSwitch(1)`'i
yeniden assert ediyor. Manuel moddaki adım-ekleme menüsüne `[c]` seçeneği olarak eklendi —
launch'ı durdurmaya/movecobot'u yeniden başlatmaya gerek yok, o ana kadar kaydedilen adımlar
korunuyor.

### BİLİNEN TUZAK — Gripper deaktifken `grip()`/`move()` sessizce başarısız görünüyor
`enter_drag_teach()`, elle çene açıp kapatabilmek için gripper'ı deaktive ediyor (tutma
torkunu kesiyor). Fairino SDK'sının garip bir davranışı: gripper deaktifken `MoveGripper()`
çağrısı hata FIRLATMIYOR, `err=0` (başarılı) dönüyor ama gripper GERÇEKTE HİÇ HAREKET
ETMİYOR. Canlı testte tam bu yaşandı — "kapandı ama nesne algılanmadı, pozisyon=0" uyarısı
gelip kullanıcı 'e'ye basınca robot birden kavradı (asıl hareket o an tetiklenmiş oldu).
**Düzeltme**: `GripperController.grip()` artık başında `is_active()` kontrolü yapıp gerekirse
otomatik `activate(force=True)` çağırıyor — bu tuzağı bir daha yaşatmıyor, ama SDK'nın bu
"deaktifken sessizce no-op" davranışı gripper'a dokunan her yeni kod için akılda tutulmalı.

## fairino_launch — Otomatik Robot Algılama ve Kurulum (Güncel Ana Yöntem)

`movecobot.py`'yi (ya da herhangi bir kol kontrolünü) çalıştırmadan ÖNCE gereken MoveIt2
sunucusunu ayağa kaldırmanın güncel yolu, aşağıdaki "ROS2 + MoveIt2 + fairino_hardware
Kurulumu" bölümündeki MANUEL adımlar (tek tag'e sabit git checkout + sed ile plugin satırı
değişikliği) DEĞİL, tek bir komut:
```bash
fairino_launch
```
Bu alias `~/auto_fairino_launch.sh`'ı çalıştırır ve şunları TAMAMEN OTOMATİK yapar:

1. **`~/detect_robot.py --full`** — Windows + WSL ağ arayüzlerindeki TÜM subnet'leri tarar
   (port 20003 açık cihazlar), bulduğu her adayı Fairino RPC'siyle doğrulayıp
   `GetSoftwareVersion()` ile model + web sürümü + controller sürümünü okur. Önbellek
   (`~/.fairino_last_target`) varsa önce onu hızlıca doğrulamayı dener (tam tarama saniyeler
   yerine anında sonuç verir), tutmazsa sessizce tam taramaya döner — yani asla eski/yanlış
   bir robotu "doğrulanmış" gibi sunmaz. `movecobot.py`'nin kendi `detect_robot()`'u da aynı
   script'i (önbellek olmadan, `--full` ile) kullanıyor.
2. **`~/setup_fairino_target.py <model> <maj> <min> <min2> <robot_ip>`** — algılanan
   model+sürüme uygun `fairino_hardware` ve `fairino<N>_v6_moveit2_config` kaynağını
   `~/fairino_official_mirror/` içinden bulur (öncelik sırası: sürüme özel mirror klasörü →
   `~/frcobot_ros2_v393/fairino_hardware` [sadece 3.9.3 için, tam depo klonu] → izole
   `~/fairino_hardware_sources/`), `~/ros2_ws/src/frcobot_ros2/` altına kopyalar,
   `fairino_hardware_interface.cpp`'nin `on_init()`'ine bir `robot_ip` parametre desteği
   yamalar (yoksa), ilgili `*.ros2_control.xacro`'da mock plugin'i gerçek donanım plugin'iyle
   VE doğru `robot_ip`'yle değiştirir, sadece DEĞİŞEN paketleri `colcon build
   --packages-select` ile derler (kaynak içeriğinin SHA256'ı `~/.fairino_build_hashes/`'e
   yazılıp bir sonraki çalıştırmada karşılaştırılıyor — değişmemiş bir paket her
   `fairino_launch`'ta gereksiz yere silinip yeniden derlenmiyor).
3. Son adımda `~/.fairino_last_target` güncellenir ve `ros2 launch $MOVEIT_PACKAGE
   demo.launch.py` çalıştırılır (gerçek robotsa hareket öncesi bir onay/acil-stop uyarısı
   gösterir).

**Ön koşul**: `~/fairino_official_mirror/` dizini dolu olmalı — bu, `~/fairino_official_mirror.
tar.gz` (Windows tarafında indirilip WSL'e kopyalanan bir arşiv, bkz. "WSL'in İnternet Erişimi
YOK") açılarak elde ediliyor:
```bash
cd ~ && tar -xzf fairino_official_mirror.tar.gz
```
İçinde desteklenen HER model (`fairino3/5/10/16/20/30/3mt_v6_moveit2_config`) için config
paketi ve birden fazla `fairino_hardware` sürümü (3.8.2.11 - 3.9.9 arası + master) bulunuyor —
`setup_fairino_target.py` hangi sürüm gerekiyorsa onu seçiyor. `~/ros2_ws/src/frcobot_ros2/`
(aşağıdaki manuel kurulumla, TAG'iyle bir kere klonlanmış temel iskelet) hâlâ gerekli — mirror
bunun İÇİNE spesifik paketleri kopyalayıp yamalıyor, sıfırdan bir workspace kurmuyor.

**Ne zaman hâlâ manuel adımlara (aşağıdaki "ROS2 + MoveIt2 + fairino_hardware Kurulumu"
bölümü) bakılmalı**: sadece `~/ros2_ws/src/frcobot_ros2/`'nin İLK KEZ, sıfırdan kurulması
gerektiğinde (temel iskelet + `fairino_msgs`/`fairino_description` gibi paylaşılan paketler
için) — günlük kullanımda `fairino_launch` tek başına yeterli, sed ile plugin satırı elle
değiştirmeye gerek yok (`setup_fairino_target.py` bunu zaten otomatik yapıyor).

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
- Adres: http://127.0.0.1 (sonuna `:8080` gibi bir port EKLEME) — SimMachine simülatörü için.
  Gerçek robotun kendi WebApp'i `http://192.168.57.2` üzerinden erişiliyor.
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
(`fairino_params` alias'ı bunu zaten temiz formatta yazdırıyor, bkz. "Kısayollar" bölümü.)

## ROS2 + MoveIt2 + fairino_hardware Kurulumu (TEK SEFERLİK temel iskelet)

Bu bölüm `~/ros2_ws/src/frcobot_ros2/`'yi SIFIRDAN kurmak içindir — günlük kullanımda
(hangi model/sürüm robot bağlıysa ona göre doğru kaynağı seçip yamalama) yukarıdaki
"fairino_launch" bölümündeki otomasyon kullanılıyor, bu adımların çoğu ELLE tekrar
yapılmıyor.

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
> açabileceğini belirtiyor. **`V3.0.0_RobotV3.9.3` tag'i tam eşleşme sağlıyor** ve daha önce
> yaşanan "Running/Stopped" titremesini kalıcı olarak çözdü. Bu tag'de paket adı da
> sadeleşiyor: `fairino_hardware_v3_9_4` yerine artık düz `fairino_hardware`.

**Plugin satırı değişikliği** (mock_components yerine gerçek donanım eklentisi — ARTIK ELLE
GEREKMİYOR, `fairino_launch`/`setup_fairino_target.py` bunu otomatik yapıyor; sadece
`fairino_launch` kullanılmadan, tamamen manuel bir kurulumda gerekir):
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

### ÇÖZÜLDÜ — RViz görüntü bozukluğu (eski not, artık geçersiz)
Önceden kamera döndürülünce ekranda iz/karmaşa birikiyordu, çözüm olarak
`export LIBGL_ALWAYS_SOFTWARE=1` (yazılım render) kullanılıyordu. **Bu satır artık
KULLANILMIYOR** (`~/auto_fairino_launch.sh`'ta bilinçli olarak kaldırıldı) — güncel WSLg'de
GPU hızlandırmalı render (OpenGL 4.1, GLSL 4.1) sorunsuz çalışıyor ve yazılım render'ine göre
çok daha az CPU kullanıyor; yazılım render'i rviz'i ~%190 CPU'da sınırsız FPS ile döndürüp
pencerenin donmuş/kapanmış gibi görünmesine (WSLg'nin pencereyi düzgün compose edememesine)
sebep oluyordu. Eski `launch_moveit.sh` script'inde bu satır hâlâ duruyor olabilir ama
`fairino_launch` bunu kullanmıyor.

## launch_moveit.sh (eski/manuel — günlük kullanımda `fairino_launch` kullanılıyor)
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
Bu script SimMachine simülatörünü (sabit `fairino5_v6_moveit2_config`) hedefliyor — gerçek
robotla ve/veya farklı modellerle çalışırken `fairino_launch` kullan (bkz. yukarıki bölüm).
`launch_moveit` alias'ı hâlâ bu script'i çalıştırıyor.

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
`~/ws_moveit2/install/setup.bash`, `movecobot` dahil moveit_py kullanan HER alias'ta
source ediliyor — bu workspace hâlâ (ve kalıcı olarak) projenin temeli.

## moveit_py Launch Dosyası ve İlk Test Script'leri (movecobot.py'den ÖNCEKİ aşama)

`demo.launch.py` (moveit_configs_utils'ın `generate_demo_launch()` fonksiyonu) `move_group`
node'unu da başlatıyor — moveit_py kendi process'i içinde MoveIt2 çekirdeğini çalıştırdığı
için bu gereksiz/çakışan. Bu yüzden `move_group` hariç özel bir launch dosyası yazıldı:
`~/fairino/client/moveit_py_demo.launch.py` — `static_virtual_joint_tfs` + `rsp` +
`ros2_control_node` (elle) + `spawn_controllers` + `moveit_rviz` içeriyor.

moveit_py config dosyası: `~/fairino/client/fairino5_moveit_py.yaml` (planning_scene_monitor
ve planning_pipelines ayarları).

İlk test script'i: `~/fairino/client/moveit_py_gohome.py` — `MoveItConfigsBuilder(...).
moveit_cpp(file_path=...)` ile `MoveItPy` başlatıyor, `get_planning_component()` ile
plan/execute yapıyor. `execute()` fonksiyonunun doğru imzası: `execute(group_name: str,
robot_trajectory, blocking=True)` — `(trajectory, controllers=[])` DEĞİL.

Bu iki dosya moveit_py'nin ilk doğrulama/proof-of-concept aşamasından kalma — o zamandan beri
`movecobot.py` çok daha olgun bir `CobotArm` sınıfıyla (kendi collision yönetimi, TCP-offset
telafisi, adım kaydı) aynı temelin üzerine kuruldu ve ana araç oldu. `gohome_py` alias'ı hâlâ
çalışıyor ama günlük akışta `fairino_launch` + `movecobot` kullanılıyor
(`launch_moveit_py` alias'ı, yerini `fairino_launch`'a bıraktığı için `.bashrc`'den
kaldırıldı).

### Bilinen: RViz'de robot modeli görünmüyor (moveit_py_demo.launch.py'de)
RViz açılıyor ama robot modeli görünmüyor — çünkü `move_group` olmadığı için `moveit.rviz`
config'indeki "RobotModel" display'i besleyen kaynak yok. Elle "RobotModel" display eklenip
`robot_description` parametresi ayarlanırsa muhtemelen düzelir, ama SDK ve WebApp ile
doğrulama yeterli görüldüğü için bu ayrıca çözülmedi. (`fairino_launch`'ın kullandığı
`demo.launch.py` bu sorunu yaşamıyor, `move_group` + standart rviz config'i kullanıyor.)

### Bilinen: "Currently in a singular pose" uyarısı
Robotu `[0,0,0,0,0,0]` (tüm eklemler sıfır derece) hedefine hareket ettirince WebApp'te bu
uyarı çıktı — `[0,0,0,0,0,0]`'da j5=0° olduğu için klasik bir bilek tekilliği (wrist
singularity) oluşuyor. `ResetAllError()` ile SDK'dan temizlemeye çalışıldı, `Reset: 0`
(başarılı) döndü ama uyarı WebApp'te kalmaya devam etti. Robot güvenli bir hedefe
(`[0,-90,90,-90,-90,0]`) hareket ettirilince uyarı **kendiliğinden** kayboldu — yani bu bir
"hata bayrağı" değil, robotun o anki gerçek pozisyonuna bağlı canlı bir durum göstergesi.

## Kısayollar (~/.bashrc alias'ları — güncel, tam liste)

Ana araçlar:
```bash
alias fairino_launch='bash ~/auto_fairino_launch.sh'          # MoveIt2 sunucusunu otomatik baslat (bkz. ilgili bolum)
alias movecobot='... python3 ~/fairino/client/movecobot.py'   # ana kontrol/kayit araci
alias grippergui='cd ~/fairino/client && python3 gripper_gui.py'
alias konumcobot='cd ~/fairino/client && python3 konumcobot.py'   # movecobot calisirken ikinci baglanti acmadan durum goster
alias clearfault='... movecobot.py --clear-fault'              # WebApp Clear isini yapmiyorsa
alias clearcobot='bash ~/fairino/client/clearcobot.sh'          # movecobot'un respawn ettigi sahipsiz ros2_control_node surecini temizle
alias fairino_params='python3 ~/fairino/client/fairino_params.py'  # su anki eklem acilari + TCP pozu
```

Yardımcı/altyapı:
```bash
alias fixweb='bash ~/fairino/fixweb.sh'          # docker daemon/image/port sorunlarini otomatik teshis+duzelt
alias run_sim='bash ~/fairino/run_simmachine.sh'
alias stop_sim='bash ~/fairino/stop_simmachine.sh'
alias launch_moveit='bash ~/fairino/launch_moveit.sh'   # eski/manuel, SimMachine'e sabit
alias visiontest='... python3 ~/fairino/client/vision_integration_example.py'
```

Önceki nesil / moveit_py'nin ilk test araçları (movecobot bunları işlevsel olarak kapsıyor,
sadece `gohome_py` ve `movegripper` alias'ı kaldı — `movepy` ve `launch_moveit_py` artık
`.bashrc`'de yok, yerlerini `fairino_launch` + `movecobot` aldı):
```bash
alias movegripper='... python3 gripper_interactive.py'
alias gohome_py='... python3 ~/fairino/client/moveit_py_gohome.py'
```
`moveit_py_interactive.py` dosyasının kendisi hâlâ duruyor (`client/` altında), sadece
`.bashrc`'de kısayolu yok — gerekirse `python3 ~/fairino/client/moveit_py_interactive.py`
ile doğrudan çalıştırılabilir.

Eski `pymoveit2` tabanlı `runpose` alias'ı da `.bashrc`'den kaldırıldı (bkz. "pymoveit2
Kurulumu" — artık kullanılmıyor, moveit_py'ye geçildi).

Kayıtlı hareketler: `movecobot`'ta bir hareket kaydedince, o ismi (ör. `pick1`) kendisi
`# >>> movecobot kayitli hareketler (otomatik - elle duzenleme) >>>` bloğuna otomatik
alias olarak ekliyor — bu blok elle düzenlenmemeli, `movecobot`'un kendisi yönetiyor.

## pymoveit2 Kurulumu (eski/terk edilmiş — artık kullanılmıyor)

Projenin ilk aşamasında `pymoveit2` (üçüncü parti, topic/action tabanlı bir MoveIt2 Python
sarmalayıcısı) denendi, sonra resmi `moveit_py` API'sine geçildi (bkz. yukarıdaki "Güncel
Durum" notu) ve movecobot.py dahil tüm güncel araçlar moveit_py üzerine kuruldu. Bu bölüm
sadece tarihsel referans için tutuluyor.

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
`~/fairino/client/pymoveit2_pose_test.py` (Cartesian X,Y,Z + quaternion hedefi). İkisi de
o dönem doğrulanmıştı, SDK ile okunan gerçek pozisyon hedefle örtüşüyordu — ama artık aktif
kullanılmıyorlar.

## Bilinen Tuhaflıklar (SimMachine/Docker)
- `docker context ls` bazen `default` ve `desktop-linux` diye iki context gösterebiliyor —
  Docker komutlarını HER ZAMAN WSL'den çalıştır, PowerShell'den değil.
- `~/.docker/config.json` içindeki `credsStore: desktop.exe` satırı WSL içinden yeni image
  çekmeyi engelleyebiliyor (`~/.docker/config.json.bak` olarak yedeklendi/devre dışı).
- WSL/Docker'ı sıfırlamak gerekirse: `wsl --shutdown` (PowerShell'den), Docker Desktop'ı
  yeniden başlat, sonra `bash ~/fairino/launch_moveit.sh`.

## Sık Karşılaşılan Docker/WebApp Sorunları (SimMachine, kalıcı çözülmedi, tekrar edebilir)
- Docker daemon zaman zaman kendiliğinden duruyor (`sudo dockerd &` ile elle başlatmak
  gerekiyor — WSL'de systemd olmadığı için otomatik yeniden başlamıyor)
- Container ayaktayken bile WebApp'e port yönlendirmesi bazen kopuyor (`docker restart`
  ile düzeliyor)
- Bu iki durumu ayırt edip otomatik düzeltmek için `fixweb` yazıldı, ama **kök sebep**
  (neden bu kadar sık oluyor) hâlâ bilinmiyor — muhtemelen WSL2 kaynak baskısıyla
  (bellek/CPU) ilgili, kesin değil. Gerçek robotla çalışırken (SimMachine kullanılmıyorsa)
  bu sorunlar zaten söz konusu değil.

## Reboot Sonrası Hızlı Başlangıç

**Gerçek robot (asıl kullanılan yol):**
```bash
fairino_launch      # robotu otomatik algılar, dogru paketi kurar/derler, MoveIt2'yi baslatir
```
Ayrı bir terminalde:
```bash
movecobot            # kol + gripper kontrolu, hareket kaydi/oynatma
```

**SimMachine simülatörü (alternatif, gerçek robot yoksa):**
```bash
bash ~/fairino/launch_moveit.sh   # SimMachine'i otomatik başlatır + RViz/MoveIt2'yi açar
```

## FR3'e Model Değiştirme (SimMachine, artık ihtiyaç kalmadı)
SimMachine simülatörünün robot modelini FR3'e çevirmek WebApp → System Settings →
Maintenance Mode → Controller Compatibility üzerinden mümkündü (değer "2" = FR3 V6.0, ayrı
bir şifre gerektiriyordu) — ama proje o zamandan beri GERÇEK bir FR3 kolla çalıştığı için bu
artık gerekli değil. `fairino_launch` zaten hangi model bağlıysa onu otomatik algılıyor.

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
sonra Yaw(Z), sabit/dünya eksenlerine göre. `moveit_py_interactive.py` VE `movecobot.py`
içindeki `euler_deg_to_quat()` fonksiyonu bu konvansiyonu kullanıyor, veriyle tutarlı.
`movecobot.py`'deki `quat_to_euler_deg()` (Joint modu FK önizlemesi için) bunun matematiksel
tersidir — round-trip testle (2000 rastgele açı, ~1e-13° hata) doğrulandı.

## wrist3_link ile Fairino TCP Arasındaki Sabit Ofset (ÇÖZÜLDÜ)

Cartesian pose hedefleri gönderirken, MoveIt2'ye verilen Z ile SDK'nın okuduğu gerçek Z
arasında tutarsız görünen bir fark vardı — bazen +100mm, bazen -100mm. Sistematik testlerle
(aynı X,Y,Z'ye farklı RX değerleriyle giderek) örüntü bulundu: fark, **RX'e göre cos(RX)**
oranında değişiyor (RX=0'da +100mm, RX=180'de -100mm).

Kök sebep: `wrist3_link` (URDF/MoveIt2'nin hedef aldığı nokta) ile Fairino SDK'nın "TCP"
dediği nokta arasında, **aracın kendi lokal Z ekseni boyunca sabit ~100mm mesafe var**.
Robot döndükçe bu lokal mesafe global eksenlere farklı yansıyor.

Çözüm — Cartesian hedef göndermeden önce, istenen oryantasyonun quaternion'undan aracın
lokal Z ekseninin global yönünü hesaplayıp, hedefi buna göre düzeltmek:
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
TCP hatası **~0.04mm**'ye düştü. `movecobot.py`'de bu artık sabit `TCP_OFFSET_M` değil,
`compute_tcp_offset()` ile CANLI ölçülen tam 3B vektör (X/Y/Z bileşenleri dahil, sadece Z
değil) olarak uygulanıyor — bkz. dosyadaki ilgili yorum.

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

Planlama öncesi gerçek bir collision nesnesi ekleniyor
(`get_planning_scene_monitor().read_write()` + `apply_collision_object()`) — planlayıcı
zeminden kaçınan alternatif yollar buluyor. Ek olarak planlanan yolun her noktasında tüm
linklerin pozisyonunu kontrol eden bir yedek güvenlik katmanı var. Bu mantık ilk olarak
`moveit_py_interactive.py`'deki `add_floor_collision()`/`check_trajectory_for_floor()`
fonksiyonlarında uygulandı; `movecobot.py`'de `CobotArm._add_floor_collision` /
`check_trajectory_for_floor` olarak (self-collision/gripper kutusu kontrolüyle birlikte)
GÜNCEL ve aktif kullanılan versiyonu bu — her `movecobot`/`fairino_launch` başlatmasında
otomatik ekleniyor, ayrı bir `add_floor.py` çağrısına gerek yok.

## moveit_py_interactive.py — Özellikler (movecobot.py'den ÖNCEKİ araç, artık alias'ı yok)

`movecobot.py`'den ÖNCEKİ, kol-only (gripper'sız) interaktif araç — `.bashrc`'deki `movepy`
alias'ı kaldırıldı (movecobot bunun işlevini tamamen kapsıyor), dosyanın kendisi hâlâ
`client/` altında duruyor, gerekirse `python3 ~/fairino/client/moveit_py_interactive.py` ile
doğrudan çalıştırılabilir. Güncel/genişletilmiş işlevsellik (gripper, hareket kaydı, adaptif
kavrama, manuel mod) sadece `movecobot.py`'de var.

- Hareket tipi: Joint (eklem açıları) / Cartesian (X,Y,Z+RX,RY,RZ)
- Cartesian için referans: Base (mutlak) / Tool (mevcut poza göre, basitleştirilmiş) / Wobj
  (tanımlı değil, seçilirse uyarı verip çıkıyor)
- Hız/ivme yüzdesi sorusu (0-100)
- TCP offset düzeltmesi (yukarıda açıklanan cos(RX) telafisi)
- Zemin çarpışmadan kaçınma (collision nesnesi + yedek kontrol)
- Çalıştırmadan önce onay adımı
- Sonucu SDK ile otomatik doğrulama (eklem açıları + TCP pozu)

## AÇIK SORULAR / DEVAM EDEN İŞLER (gerçek robot, movecobot.py sonrası)

1. **Titreme (vibration)** — kullanıcı hareket sırasında titreme fark etti, henüz
   incelenmedi/kök sebebi araştırılmadı.
2. **J6'nın base X eksenine dik durması için gereken açı** — verilen eklem açılarından
   (J1-J6, FK ile) gripper'ı base'in X eksenine dik yapacak J6 düzeltmesi hesaplandı
   (94.558°) ve bu hesap robotun kendi raporladığı yönelimle (`rz=86.973°` ≈ hesaplanan
   86.972°) neredeyse birebir örtüştü — yani FK hesabı doğru. Ama pratikte robotta
   denendiğinde "3 derece yetmez onu dik yapmaya" geri bildirimi geldi. Bu, hesap hatası
   değil, flanş ile gripper'ın gerçek çene yönü arasında MODELLENMEMİŞ bir fiziksel montaj
   ofseti olduğunu düşündürüyor — gerçek offset'i bulmak için kullanıcının gözle tahmin
   ettiği sapma açısı bekleniyor.
3. **Collision seviyesi / dual-encoder çarpışma algılama metodu** — `SetAnticollision`
   (Level×10 = hız yüzdesi kuralı, ör. Level 5 → %50) ve `SetCollisionDetectionMethod`
   (method 0=tork, 1=çift-enkoder, 2=ikisi birden — FR3 donanımı çift enkoderli)
   araştırıldı ama HİÇBİR AYAR DEĞİŞTİRİLMEDİ (kullanıcı "önce araştır, hiçbir şey
   değiştirme" dedi) — ikisi de hâlâ açık, denenmemiş seçenekler olarak duruyor.
4. **pymoveit2'ye geçiş** — gündeme geldi, tartışıldı, KARAR: moveit_py'de kalınıyor (kod
   değişikliği yapılmadı) — bkz. yukarıdaki "Güncel Durum" notu.
5. **`joint_limits.yaml` ivme düzeltmesi** sadece `fairino3` ve `fairino5` paketlerinde —
   diğer modeller (`fairino10/16/20/30/3mt`) için henüz uygulanmadı, gerekirse aynı mantıkla
   (`ompl_planning.yaml` düzeltmesinde yapıldığı gibi) tüm modellere genellenebilir.
6. **Yayınlanan kod**: bu araçların (client/ + ros2_config_patches/) bir kopyası
   `github.com/zehrakdemir/robot-arm-simulation` reposunun `huseyin` branch'inde,
   `robot_arm/` klasörü altında tutuluyor (ana repodaki `vision/` ile paralel).
