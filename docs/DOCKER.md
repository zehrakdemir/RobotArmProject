# Docker

İki hedef var ve amaçları farklı.

| Hedef | Ne içerir | Ne için |
|---|---|---|
| `dev` | Python 3.10 + paketimiz + pytest/ruff. **ROS2 yok.** | Lint ve test. CI bunu kullanır. Saniyeler içinde ayağa kalkar. |
| `runtime` | ROS2 Humble + MoveIt2 + MTC + yamalı Fairino sürücüsü + SDK + web arayüzü | Gerçek robotu sürmek. **Gerçek robotla doğrulandı.** |

## Hızlı başlangıç

```bash
cp .env.example .env     # içine FAIRINO_ROBOT_IP yaz
```

```bash
docker compose up cobot
```

Arayüz: **<http://localhost:8088>**

İmajın `CMD`'si [`scripts/start_all.sh`](../scripts/start_all.sh):

1. `fairino_launch` — robotu algıla, modeline/sürümüne uyan sürücü kaynağını kur ve derle,
   `move_group` + rviz başlat
2. `movecobot-web` — HTTP arayüzü, port 8088

**Sıra önemli ama bekleme sınırlı.** Arayüz `move_group` için en fazla 3 dakika bekler,
sonra yine açılır. Bu bilerek böyle: robot hazır değilken kullanıcının elinde hiçbir şey
kalmamasındansa, sebebi ekranda gösteren bir arayüz olması iyidir. Arayüz zaten kendi başına
yeniden denemeye devam eder (10s → 60s kademeli bekleme, sonsuza kadar), robot hazır olunca
kendiliğinden bağlanır.

**Sadece arayüz ölürse container kapanır.** `fairino_launch` ölürse container ayakta kalır;
robotu taktıktan sonra elle yeniden başlatmak yeterli:

```bash
docker compose exec -d cobot bash /app/scripts/fairino_launch.sh
```

## Ne doğrulandı

**Gerçek robotla, container içinden** (canlı ölçüm):

- `Successful set up FIFO RT scheduling policy with priority 50` — RT önceliği alındı
- Servo akışı UDP kanalından (`comType=1`, port 20007), döngü periyodu 0.00999–0.01003 s
  (hedef 10 ms) — sıçrama yok, hız limiti faultu yok
- `/execute_task_solution` mevcut (MTC pick & place çalışabilir durumda)
- SDK durum okuması `err = 0`, eklem açıları ve `gripper_active` okundu
- Web arayüzü `:8088` üzerinden cevap veriyor

**Robota dokunmadan** (`docker compose run --rm verify`):

- Sürücü **bizim tam yamamızla** derleniyor: `_servo_com_type` (UDP kanalı), `SERVOJ_CMDT`
  (ölçülen cmdT), `MAX_SAFE_JOINT_VEL_DEG_S` (hız sınırlayıcı), `IDLE_EXIT_S` (boşta servo
  çıkışı), `robot_ip`
- `libfairino_hardware.so` üretiliyor
- `moveit_py`, MoveIt Task Constructor, Fairino SDK ve uygulama modülleri yükleniyor
- MoveIt config yamaları (jerk sınırı, çarpışma kontrol sıklığı, JTC toleransları) yerinde
- Konsol komutları PATH'te

`verify` bunu **ilk** çalıştırmaya değer. Bir imaj "başarıyla" derlenip işe yaramaz
çıkabiliyor — gerçek bir örnek: `colcon build --packages-select moveit_py` paketi
bulamayınca **uyarı verip 0 ile çıkıyor**; build yeşil, `install/` bomboş, robot hiç
sürülemiyor. Bu betik tam olarak öyle sessiz boşlukları yakalar. Native kurulumda da çalışır:

```bash
bash scripts/verify_container.sh
```

## Robotun adresi — Docker'da neden elle vermek gerekiyor

```
FAIRINO_ROBOT_IP=192.168.57.2
```

Native tarafta `detect-robot` ağı tarayıp robotu kendisi buluyor. **Container içinde bu
genelde çalışmaz:** robotun ağı (örn. `192.168.57.0/24`) container için yerel değil,
*yönlendirilmiş* bir ağdır. Tarama yerel alt ağa bakar, orada robot yoktur, hiçbir şey
bulunamaz.

Native'in bulabilmesinin sebebi ayrıca ilginç: `detect_robot` WSL altında Windows'un ağ
arayüzlerini `ipconfig.exe` üzerinden okuyor — container'da öyle bir kaçış yolu yok.

Adres verildiğinde tarama **hiç yapılmaz**, açılış da hızlanır. `.env` imaja **girmez**
([`.dockerignore`](../.dockerignore)), çalışma zamanında environment olarak geçer.

## Ağ: köprü (varsayılan)

Bu dosyada eskiden `network_mode: host` **zorunlu** diye yazıyordu. **Ölçüldü, zorunlu
değil:** köprü ağında gerçek robot sorunsuz sürüldü.

Sebebi basit: `move_group`, `ros2_control` ve web arayüzü **aynı container'da** çalışıyor,
yani DDS keşfi container içinde kalıyor — multicast'e ihtiyaç yok. Robota giden trafik
(20003 XML-RPC, 20004 durum, 20007 UDP servo) sıradan IP yönlendirmesiyle gidiyor.

**Köprü ağının bedeli:** host'tan `ros2 topic list` container'ın düğümlerini **görmez**.
Görmen gerekiyorsa host ağına geç:

```bash
docker compose run --rm --network host cobot bash /app/scripts/start_all.sh
```

> Docker Desktop for Windows'ta host networking Linux'takiyle aynı davranmıyor. Host ağı
> gerekiyorsa WSL2 içinde çalışan yerli docker engine kullan.

## RT önceliği: `cap_add: SYS_NICE` + `ulimits.rtprio: 99`

`ros2_control` kontrol döngüsünü FIFO gerçek-zamanlı öncelikle çalıştırabilmesi için. Bu izin
olmadan döngü gecikip **patlama halinde** yetişmeye çalışıyor (canlı ölçümde `write()`
çağrıları 10 ms yerine 3 ms arayla görüldü), ServoJ akışı düzensizleşiyor ve robot
`axis N command speed exceeds the limit` faultu verebiliyor.

Native tarafta aynı sorun `Could not enable FIFO RT scheduling policy: Operation not
permitted` olarak görülmüştü; çözümü PAM'i devreye sokan `sudo` üzerinden `rtprio 99`
almaktı. Container'da karşılığı bu iki ayar.

Doğrulama — container içinde `ulimit -r` `0` dönüyorsa ayar geçerli değil; entrypoint bunu
zaten uyarı olarak basar. Launch çıktısında şunu görmelisin:

```
Successful set up FIFO RT scheduling policy with priority 50.
```

## X soketi (rviz2 ve gripper GUI için)

WSLg altında `DISPLAY`, `XDG_RUNTIME_DIR` ve `/mnt/wslg` bind-mount'u gerekiyor. Saf Linux'ta
`/mnt/wslg` satırını kaldır, `/tmp/.X11-unix` yeter.

`LIBGL_ALWAYS_SOFTWARE=1` bilerek açık: WSLg'de glamor/GBM tutarsız çalışıyor ve rviz'in
kendi 3B render'ı (Ogre) içerik çizmeyen bir duruma girebiliyor.

Web arayüzü X **gerektirmez** — rviz açılmasa da arayüz çalışır.

## Kalıcı veri

Kayıtlı hareketler, durum önbelleği ve yarım kalmış kayıt taslağı `/state` altında
(`FAIRINO_STATE_DIR`), `fairino-state` adlı named volume'a bağlı — container silinince
kaybolmazlar.

## compose yerine docker run

[`scripts/run_container.sh`](../scripts/run_container.sh) aynı bayrakları elle `docker run`
ile kurar; compose kullanmak istemeyenler için:

```bash
bash scripts/run_container.sh          # gerçek robot
bash scripts/run_container.sh --logs   # başlat ve logları izle
bash scripts/run_container.sh --stop   # durdur
```

## Simülatör (SimMachine) — çalışmıyor

`scripts/run_container.sh --sim` ve `scripts/run_simmachine.sh` duruyor, ama **simülatörle
robot sürülemiyor.** Bu bir yapılandırma eksikliği değil, simülatörün kendi sınırı.

Ölçülen sebep: simülatör **gerçek-zamanlı durum akışını sunmuyor.**

| Port | Ne | Simülatör |
|---|---|---|
| 20003 | XML-RPC (komut) | çalışıyor — `GetSoftwareVersion` cevap veriyor |
| 20004 | gerçek-zamanlı durum | dinliyor ama **0 bayt** gönderiyor |
| 20005 | CNDE (SDK v2.2.5+) | **hiç yok** — bağlantı reddediliyor |

Bunun iki sonucu var ve ikisi de aynı kök sebebe iniyor:

- **SDK tarafı:** `GetActualJointPosDegree()` →
  `TypeError: '_ctypes.CField' object is not subscriptable` — durum paketi hiç oluşmamış.
  `movecobot` ve web arayüzü daha MoveIt başlamadan burada duruyor.
- **Sürücü tarafı:** `FairinoHardwareInterface` activate olamıyor
  (`机械臂SDK连接失败`), `ros2_control_node` `std::runtime_error` ile ölüyor,
  `/controller_manager` hiç doğmuyor, spawner'lar sonsuza kadar bekliyor, `/joint_states`
  hiç yayınlanmıyor. `move_group` ve rviz ayakta ama boşlar.

Fairino'nun **kendi** ROS2 sürücüsü de simülatörle çalışmıyor:
[frcobot_ros2 issue #12](https://github.com/FAIR-INNOVATION/frcobot_ros2/issues/12) —
açık ve cevapsız.

Denenen ve **işe yaramayan** yollar (bir daha denememek için):

- Yeni SDK (`v2.2.9_robot_v3.9.9`) ile CNDE: 20005'te kimse dinlemiyor, `SendCNDEOutputConfig`
  zaman aşımı (`rtn -3`).
- `ROBOT_CNDE_PORT = 8083` override: 8083 veri akıtıyor ama tek yönlü yayın, yapılandırma
  el sıkışmasına cevap vermiyor.
- Docker'ın yayımlanan portları **yanıltıyor**: `-p 20005:20005` ile docker-proxy, içeride
  hiçbir şey dinlemese bile TCP bağlantısını kabul ediyor. "Port açık" demek "servis var"
  demek değil — simülatörün içinde `netstat` ile bakmak gerekiyor.

Dönülecek yer varsa orası **20005 değil**: 20001, 20002 ve 8083 `5a5a` başlıklı (CNDE
çerçeveli) veri akıtıyor. Bunları çözmek belgelenmemiş bir ikili protokolü tersine çevirmek
demek ve doğruluğu ancak gerçek robotla karşılaştırılarak doğrulanabilir.

## Bilinen sınırlar

- `runtime` build'i sıfırdan ~25-40 dakika (`moveit_py` + Fairino sürücüsü derlemesi).
  Katmanlar önbelleklendikten sonra çok daha hızlı.
- İmaj **FR3'e göre** hazırlanıyor (`ROBOT_MODEL` build arg). Başka bir model takılıysa
  `fairino_launch` çalışma anında doğru paketi imajın içindeki aynadan kurup derliyor —
  çalışır, sadece ilk açılışta bekletir. Baştan başka modele derlemek için:
  `docker build --build-arg ROBOT_MODEL=5 ...`
- Web arayüzünde **kimlik doğrulaması yok** ve gerçek robotu hareket ettirir. `8088`
  portunu güvenmediğin bir ağa açma.
- Sürücü doğrudan LAN'a eriştiği için container'ın izole olması **güvenlik sınırı değil**.

## Robotun sürümü aynada yoksa

Aynadaki (`V3.0.0_RobotV3.9.9`) sürücü sürümleri: 3.8.2.11, 3.9.0, 3.9.0.1, 3.9.4–3.9.9.
Robot başka bir sürüm bildirirse `fairino_launch` "kaynağı bulunamadı" deyip durur.

Çözüm: o sürümün kaynağını **kendi tag'iyle** indir. Araç robota bağlanmaz.

```bash
docker compose exec cobot fairino-fetch-source 3.9.2
```

```bash
docker compose restart cobot
```

- İndirilen yer: `/state/frcobot_sources/frcobot_ros2_v3_9_2/`
  (host: `~/.local/state/fairino-cobot/frcobot_sources/`). Container silinince kaybolmaz.
- **Neden aynaya kopyalanmıyor:** Aynada bir sürü `fairino_hardware` var ama `fairino_msgs`
  tek (3.9.9'unki). Başka tag'den gelen sürücü onunla derlenince kırılabiliyor; 3.9.3'te
  `'RobotNonrtState_' has no member named 'servoj_cmd_num'` hatası böyle çıktı.
  Tam tag klonunda üç paket aynı sürümden yan yana durur.
- Aynı sürümün ekli bir tag'i de varsa (ör. `v3.0.0_RobotV3.9.8_v1.1`) araç ekisiz olanı
  seçer ve diğerlerini listeler. Farkını Fairino belgelemiyor; diğeri için `--tag`.
- Araç, sürücü yamamızın yeni kaynağa tutup tutmadığını `--dry-run` ile söyler.
  **Tutmazsa `setup_target` durur**, yamasız sürücüyle devam etmez.
- GitHub'daki tag'ler: `fairino-fetch-source --list`

## Sürüm sabitlemeleri

| Ne | Değer | Neden |
|---|---|---|
| `MOVEIT_COMMIT` | `2b2030dd7` | **Tag değil, commit.** `2.5.9` tag'inde `moveit_py` paketi henüz yok; native kurulum humble dalında, tag'den 11 commit sonrasında. İmaj native ile aynı sürümü kullansın diye sabitlendi. |
| `FRCOBOT_TAG` | `V3.0.0_RobotV3.9.9` | Bu tag, `fairino_hardware`'in v3.8.2.11'den v3.9.9'a **tüm sürümlerini** içinde taşıyor. Native kurulumdaki "fairino_official_mirror" klasörünün kaynağı da budur — özel bir şey değil, herkes klonlayabilir. |
| `FRCOBOT_SIM_TAG` | `V3.0.0_RobotV3.9.3` | 3.9.3 yukarıdaki tag'de **yok**, ayrı tag. Simülatör kendini 3.9.3 diye tanıttığı için ayrıca klonlanıyor. |
| `SDK_TAG` | `v2.2.3_robot3.9.3` | v2.2.5+ durum okumasını CNDE/20005'e taşıyor ve bu, ROS2 sürücüsünün kendi bağlantısıyla çakışıyor (ölçüldü). Eski SDK port 20004'te kalıyor, çakışma olmuyor. Dockerfile bunu `grep` ile doğruluyor. |

## Sık karşılaşılanlar

**`docker compose up cobot` sonrası `localhost:8088` açılmıyor**
İlk derleme sürüyor olabilir. `docker compose logs -f cobot` ile bak; `=== 2/2 Web arayüzü`
satırını görene kadar bekle.

**Arayüz açılıyor ama "robot hazır değil" diyor**
Robot açık ve WebApp'ten **enable** edilmiş mi? `.env`'deki adres doğru mu? Logda
`ros2_control_node` satırları sebebi söyler.

**`ulimit -r` = 0**
`cap_add: SYS_NICE` ve `ulimits.rtprio` geçerli değil. Docker Desktop kullanıyorsan
WSL entegrasyonunun açık olduğundan emin ol.

**Port 8088 kullanımda**
`MOVECOBOT_WEB_PORT=8090 docker compose up cobot`. (Windows'ta Hyper-V/WinNAT bazı portları
rezerve edebiliyor — `netsh interface ipv4 show excludedportrange protocol=tcp` ile bakılır.)
