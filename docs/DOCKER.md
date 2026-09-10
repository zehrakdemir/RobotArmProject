# Docker

İki hedef var ve amaçları farklı.

| Hedef | Ne içerir | Ne için |
|---|---|---|
| `dev` | Python 3.10 + paketimiz + pytest/ruff. **ROS2 yok.** | Lint ve test. CI bunu kullanır. Saniyeler içinde ayağa kalkar. |
| `runtime` | ROS2 Humble + MoveIt2 + MTC + Fairino sürücüsü + SDK | Gerçek robotu sürmek. Derlendi ve içi doğrulandı; **robotla henüz denenmedi.** |

### Ne doğrulandı, ne doğrulanmadı

**Doğrulandı** (imaj derlenip içinde koşturularak):

- İmaj derleniyor ve `moveit_py` gerçekten kuruluyor
- Sürücü **bizim tam yamamızla** derleniyor: `_servo_com_type` (UDP kanalı),
  `SERVOJ_CMDT` (ölçülen cmdT), `MAX_SAFE_JOINT_VEL_DEG_S` (hız sınırlayıcı),
  `IDLE_EXIT_S` (boşta servo çıkışı), `robot_ip`
- `libfairino_hardware.so` üretiliyor
- `moveit_py`, MoveIt Task Constructor, Fairino SDK ve uygulama modülleri
  (`movecobot`, `pickplace`, `gripper_controller`, `konumcobot`) yükleniyor
- MoveIt config yamaları (jerk sınırı, çarpışma kontrol sıklığı, JTC
  toleransları) yerinde
- Konsol komutları PATH'te

**Doğrulanmadı:** container'dan **gerçek robot sürmek**. Native kurulum
çalışıyor ve şimdilik desteklenen yol o ([`SETUP.md`](SETUP.md)). Aşağıdaki üç
ayardan biri eksik kalırsa robot ya bulunamaz ya da hareket bozulur.

## Hızlı test (ROS2 gerekmez)

```bash
docker compose run --rm test
```

## Ortam sağlıklı mı (robot gerekmez)

```bash
docker compose run --rm verify
```

Bunu **ilk** çalıştır. Bir imaj "başarıyla" derlenip işe yaramaz çıkabiliyor —
gerçek bir örnek: `colcon build --packages-select moveit_py` paketi
bulamayınca **uyarı verip 0 ile çıkıyor**; build yeşil, `install/` bomboş,
robot hiç sürülemiyor. Bu betik tam olarak öyle sessiz boşlukları yakalar.

Native kurulumda da çalışır:

```bash
bash scripts/verify_container.sh
```

## Tam ortam

```bash
docker compose run --rm cobot
```

Container içinde `movecobot`, `konumcobot`, `detect-robot` komutları hazırdır.

## Üç zorunlu ayar ve nedenleri

### 1. `network_mode: host`

Robota LAN üzerinden ulaşmak (port 20003 XML-RPC, 20004 durum paketi, 20007 UDP
servo) ve ROS2 DDS'in multicast keşfi için. Bridge ağında DDS düğümleri
birbirini görmez.

> **Docker Desktop for Windows uyarısı:** host networking orada Linux'takiyle
> aynı davranmıyor. Bu container'ı **WSL2 içinde çalışan yerli docker engine**
> ile kullan (`sudo service docker start` ya da systemd ile), Docker Desktop
> entegrasyonuyla değil.

### 2. `cap_add: SYS_NICE` + `ulimits.rtprio: 99`

`ros2_control` kontrol döngüsünü FIFO gerçek-zamanlı öncelikle çalıştırabilmesi
için. Bu izin olmadan döngü gecikip **patlama halinde** yetişmeye çalışıyor
(canlı ölçümde `write()` çağrıları 10 ms yerine 3 ms arayla görüldü), ServoJ
akışı düzensizleşiyor ve robot `axis N command speed exceeds the limit` faultu
verebiliyor.

Native tarafta aynı sorun `Could not enable FIFO RT scheduling policy:
Operation not permitted` olarak görülmüştü; çözümü PAM'i devreye sokan `sudo`
üzerinden `rtprio 99` almaktı. Container'da karşılığı bu iki ayar.

Doğrulama — container içinde:

```bash
ulimit -r
```

`0` dönüyorsa ayar geçerli değil; entrypoint bunu zaten uyarı olarak basar.
Launch çıktısında da şunu görmelisin:
`Successful set up FIFO RT scheduling policy with priority 50.`

### 3. X soketi (rviz2 ve gripper GUI için)

WSLg altında `DISPLAY`, `XDG_RUNTIME_DIR` ve `/mnt/wslg` bind-mount'u gerekiyor.
Saf Linux'ta `/mnt/wslg` satırını kaldır, `/tmp/.X11-unix` yeter.

`LIBGL_ALWAYS_SOFTWARE=1` bilerek açık: WSLg'de glamor/GBM tutarsız çalışıyor ve
rviz'in kendi 3B render'ı (Ogre) içerik çizmeyen bir duruma girebiliyor.

## Kalıcı veri

Kayıtlı hareketler, durum önbelleği ve yarım kalmış kayıt taslağı `/state`
altında (`FAIRINO_STATE_DIR`), `fairino-state` adlı named volume'a bağlı —
container silinince kaybolmazlar.

## Ayarlar

Robot IP'si ve diğer değerler `.env` ile veriliyor
([`.env.example`](../.env.example)). `.env` **imaja girmez** (`.dockerignore`),
çalışma zamanında environment olarak geçer.

```bash
cp .env.example .env
```

## İmajda ne kaynaktan derleniyor, neden

`ros-humble-moveit` (2.5.9) apt'te var ama **`ros-humble-moveit-py` yok** —
native kurulumda MoveIt2'nin kaynaktan derlenmesinin sebebi tam olarak bu.
İmajda tam kaynak derlemesi yerine **sadece `moveit_py`**, apt'ten gelen
2.5.9'a karşı derleniyor: çok daha hafif, sürüm birebir aynı kalıyor.

Fairino sürücüsü (`frcobot_ros2`) pinlenmiş tag'den klonlanıp yamalarımız
[`tools/apply_ros2_patches.py`](../tools/apply_ros2_patches.py) ile
uygulanıyor — native kurulumla **aynı araç**, böylece imaj ile native ortam
birbirinden ayrılmıyor.

## Bilinen sınırlar

- **Gerçek robotla denenmedi.** İmaj derlendi ve içi doğrulandı, ama kolu
  container'dan sürmek henüz test edilmedi. Native kurulum çalışıyor.
- `runtime` build'i sıfırdan ~25 dakika (moveit_py + Fairino sürücüsü
  derlemesi). Katmanlar önbelleklendikten sonra çok daha hızlı.
- İmaj **FR3'e göre** hazırlanıyor (`ROBOT_MODEL` build arg). Başka bir model
  takılıysa `fairino_launch` çalışma anında doğru paketi aynadan kurup
  derliyor — ayna imajın içinde olduğu için çalışır, sadece ilk açılışta
  bekletir. Baştan başka modele derlemek için:
  `docker build --build-arg ROBOT_MODEL=5 ...`
- Sürücü doğrudan LAN'a eriştiği için container'ın izole olması **güvenlik
  sınırı değil**: `network_mode: host` ile ağ yalıtımı yok.

## Sürüm sabitlemeleri

| Ne | Değer | Neden |
|---|---|---|
| `MOVEIT_COMMIT` | `2b2030dd7` | **Tag değil, commit.** `2.5.9` tag'inde `moveit_py` paketi henüz yok; native kurulum humble dalında, tag'den 11 commit sonrasında. İmaj native ile aynı sürümü kullansın diye sabitlendi. |
| `FRCOBOT_TAG` | `V3.0.0_RobotV3.9.9` | Bu tag, `fairino_hardware`'in v3.8.2.11'den v3.9.9'a **tüm sürümlerini** içinde taşıyor. Native kurulumdaki "fairino_official_mirror" klasörünün kaynağı da budur — özel bir şey değil, herkes klonlayabilir. |
