# Docker

İki hedef var ve amaçları farklı.

| Hedef | Ne içerir | Ne için |
|---|---|---|
| `dev` | Python 3.10 + paketimiz + pytest/ruff. **ROS2 yok.** | Lint ve test. CI bunu kullanır. Saniyeler içinde ayağa kalkar. |
| `runtime` | ROS2 Humble + MoveIt2 + MTC + Fairino sürücüsü + SDK | Gerçek robotu sürmek. **Deneysel.** |

> **Desteklenen yol hâlâ native kurulumdur** ([`SETUP.md`](SETUP.md)). `runtime`
> imajı tekrarlanabilirlik ve yeni geliştiricinin işe başlaması için var;
> **gerçek robotla henüz doğrulanmadı.** Aşağıdaki üç ayardan biri eksik
> kalırsa robot ya bulunamaz ya da hareket bozulur.

## Hızlı test (ROS2 gerekmez)

```bash
docker compose run --rm test
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

- İmaj **build edilip gerçek robotla test edilmedi**. Native kurulum çalışıyor
  ve desteklenen yol o.
- `runtime` imajının build'i uzun sürer (moveit_py + frcobot_ros2 derlemesi).
- Sürücü doğrudan LAN'a eriştiği için container'ın izole olması **güvenlik
  sınırı değil**: `network_mode: host` ile ağ yalıtımı yok.
