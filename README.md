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

## Kurulum

İki yol var:

| Yol | Kimin için | Gerekenler |
|---|---|---|
| **A. Hazır imaj** (önerilen) | Robotu kullanacak kişi | Docker + imaja erişimi olan bir GitHub hesabı |
| **B. Kaynaktan derleme** | Kodu değiştiren geliştirici | Linux ya da WSL2, Docker, bu repo |

### Ortak önkoşullar

| Gereksinim | Not |
|---|---|
| Robot | Açık, **WebApp'ten enable edilmiş**, Auto modda ve bilgisayarla aynı ağda ulaşılabilir |
| Gripper (kullanılacaksa) | Jodell EPG40-050 + WebApp'te `End_Jodell_V1.0` protokolü uygulanmış ve Configure edilmiş |
| İşlemci | 64-bit Intel/AMD (imaj `amd64`). Apple işlemcili Mac (M1–M4) desteklenmiyor |
| Disk | Hazır imaj: ~0,9 GB indirme, açılınca ~4,2 GB. Kaynaktan derleme: ~8 GB |

## A. Hazır imajla kurulum

İmaj GitHub Container Registry'de (GHCR) **özel** paket; herkes indiremez.

1. **Erişim iste:** GitHub kullanıcı adını imajın sahibine (`huseyinai`) ver; sana **Read** erişimi tanımlar.
2. **Token aç:** GitHub → Settings → Developer settings → Personal access tokens →
   **Tokens (classic)** → Generate new token (classic) → yalnızca **`read:packages`** işaretli.
   GHCR yalnızca *classic* token kabul ediyor. Token bir paroladır, kimseyle paylaşma.

### A1. Windows (Docker Desktop)

**Gerekenler:** Windows 10 22H2 (build 19045) ya da Windows 11 23H2 ve üstü, 64-bit, 8 GB RAM,
BIOS'ta sanallaştırma açık. [Docker Desktop](https://docs.docker.com/desktop/setup/install/windows-install/)'ı
kur ve aç.

Ubuntu, ROS ya da WSL ayarı gerekmez. İmaj Linux olduğu için Docker Desktop arka planda kendi
Linux sanal makinesini açar ve container'ı onun içinde çalıştırır.

Komutlar **PowerShell**'de çalıştırılır.

**1. GHCR'ye giriş.** Parola sorulunca token'ı yapıştır:

```powershell
docker login ghcr.io -u <github-kullanici-adin>
```

**2. İmajı indir:**

```powershell
docker pull ghcr.io/huseyinai/fairino-cobot:latest
```

**3. Robotun IP adresini dosyaya yaz** (`192.168.57.2` yerine kendi robotunun adresi; robotun
WebApp adresi neyse odur):

```powershell
Set-Content -Path "$env:USERPROFILE\cobot.env" -Value "FAIRINO_ROBOT_IP=192.168.57.2" -Encoding ascii
```

**4. İlk çalıştırma:**

```powershell
docker run -d --name cobot --env-file "$env:USERPROFILE\cobot.env" --cap-add SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 -p 8088:8088 -v fairino-state:/state ghcr.io/huseyinai/fairino-cobot:latest
```

**5. Arayüzü aç:** **<http://localhost:8088>**. Robot algılanıp MoveIt ayağa kalkana kadar ~1
dakika sürer. Robotun sürümü imajdakinden farklıysa ilk açılışta sürücü derlenir, daha uzun sürer.

**Sonraki kullanımlar:** Docker Desktop → **Containers** → `cobot` → Start / Stop. Container
4. adımdaki ayarlarla bir kez oluşturulduğu için Start aynı ayarlarla açar. Docker Desktop
açılınca kendiliğinden **başlamaz**.

> **Neden Docker Desktop'taki Run butonu kullanılmıyor:** O pencerede yalnızca ad, port,
> volume ve ortam değişkeni verilebiliyor ([Docker Docs](https://docs.docker.com/desktop/use-desktop/images/)).
> `--cap-add SYS_NICE` ve `--ulimit rtprio=99` verilemiyor. Bunlar olmadan kontrol döngüsü
> gerçek zamanlı öncelik alamıyor, servo akışı düzensizleşiyor ve robot hız limiti faultu
> verebiliyor.

> **Windows'ta zamanlama:** Container Docker Desktop'ın sanal makinesinde çalıştığı için
> gerçek zamanlı öncelik doğrudan Linux'taki kadar garanti değil
> ([ROS2 Tutorial](https://ros2-tutorial.readthedocs.io/en/latest/docker/),
> [docker/for-win #2982](https://github.com/docker/for-win/issues/2982)). Başka Windows
> bilgisayarlarda ölçülmedi. Logda şu satır görülmeli:
> `Successful set up FIFO RT scheduling policy with priority 50`

**Günlük komutlar:**

| İş | Komut |
|---|---|
| Loglar | `docker logs -f cobot` |
| Terminal aracı | `docker exec -it cobot bash -lc movecobot` |
| Durdur | `docker stop cobot` |
| Tekrar başlat | `docker start cobot` |
| Robota dokunmadan kontrol | `docker run --rm --network none ghcr.io/huseyinai/fairino-cobot:latest bash /app/scripts/verify_container.sh` |

**Güncelleme** (robot hareket etmiyorken):

```powershell
docker rm -f cobot
```

```powershell
docker pull ghcr.io/huseyinai/fairino-cobot:latest
```

Sonra 4. adımdaki `docker run` komutu. Kayıtlı hareketler `fairino-state` volume'unda durur,
silinmez.

### A2. Linux ve Mac (Intel)

**Gerekenler:**

- **Linux:** [Docker Engine](https://docs.docker.com/engine/install/). Komutları `sudo`'suz
  çalıştırmak için [kurulum sonrası adımlar](https://docs.docker.com/engine/install/linux-postinstall/).
  Linux'ta container doğrudan bilgisayarın çekirdeğinde çalışır, arada sanal makine yoktur.
- **Mac (Intel işlemcili):** [Docker Desktop for Mac](https://docs.docker.com/desktop/setup/install/mac-install/)
  (Intel chip sürümü) kur ve aç. Komutlar **Terminal** uygulamasında çalıştırılır. Windows'taki gibi
  container Docker Desktop'ın sanal makinesinde çalışır; gerçek zamanlı zamanlama doğrudan
  Linux'taki kadar garanti değil. **Mac'te denenmedi.**

> **Apple işlemcili Mac (M1, M2, M3, M4) desteklenmiyor.** Robot sürücüsünün bağlı olduğu Fairino
> kütüphanesi (`libfairino.so`) yalnızca x86-64 için derlenmiş; ARM sürümü yok. İmaj bu Mac'lerde
> ancak emülasyonla çalışır, servo akışı düzensizleşebilir. Robotu sürmek için kullanma.
> Mac'in işlemcisini görmek için: sol üstteki Apple menüsü → **Bu Mac Hakkında** → "İşlemci" ya da "Çip" satırı.

```bash
docker login ghcr.io -u <github-kullanici-adin>
```

```bash
docker pull ghcr.io/huseyinai/fairino-cobot:latest
```

```bash
echo "FAIRINO_ROBOT_IP=192.168.57.2" > ~/cobot.env
```

```bash
docker run -d --name cobot --env-file ~/cobot.env --cap-add SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 -p 8088:8088 -v fairino-state:/state ghcr.io/huseyinai/fairino-cobot:latest
```

Arayüz: **<http://localhost:8088>**. Günlük komutlar ve güncelleme A1'deki ile aynı.

## B. Kaynaktan derleme (geliştirici)

Linux'ta ya da Windows'ta **WSL2 içindeki Ubuntu'da** yapılır. Windows'ta repoyu doğrudan bir
Windows klasörüne klonlama:

- Git for Windows `.sh` dosyalarını CRLF satır sonuna çeviriyor (`core.autocrlf=true`);
  container içindeki betikler çalışmaz.
- PowerShell'de `HOME` tanımlı değil; kayıtlı hareketlerin dizini yanlış yere düşer.

Kendi makinene ROS2/MoveIt2 kurmak istersen (Docker olmadan): **[`docs/SETUP.md`](docs/SETUP.md)**.

### B1. Klonla

```bash
git clone -b huseyin https://github.com/zehrakdemir/RobotArmProject.git && cd RobotArmProject
```

### B2. Robotun IP adresini ver

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

### B3. İmajı derle ve başlat

```bash
docker compose up cobot
```

İlk derleme **~25-40 dakika** sürer (MoveIt2'nin `moveit_py` paketi ve Fairino sürücüsü
kaynaktan derleniyor). Sonraki açılışlar saniyeler.

### B4. Arayüzü aç

**<http://localhost:8088>**

Robot hazır değilse arayüz yine açılır ve **sebebini ekranda gösterir**; robot hazır olunca
kendiliğinden bağlanır — container'ı yeniden başlatmaya gerek yok.

Arka planda çalıştırmak için `docker compose up -d cobot`, logları izlemek için
`docker compose logs -f cobot`.

### B5. İmajı GHCR'ye yükleme (bakımcı)

A yolundaki hazır imaj buradan gelir. Yükleyen hesapta `write:packages` izinli **classic** token
gerekir ([GitHub Docs](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry)).

```bash
docker login ghcr.io -u huseyinai
```

```bash
docker compose build cobot
```

```bash
docker tag fairino-cobot:runtime ghcr.io/huseyinai/fairino-cobot:latest && docker tag fairino-cobot:runtime ghcr.io/huseyinai/fairino-cobot:$(git rev-parse --short HEAD)
```

```bash
docker push ghcr.io/huseyinai/fairino-cobot:latest && docker push ghcr.io/huseyinai/fairino-cobot:$(git rev-parse --short HEAD)
```

Commit etiketi, hangi imajın hangi kodla derlendiğini izlemek için.

- İlk yüklemede paket **özel** olur. Erişim vermek için: GitHub → profil → **Packages** →
  `fairino-cobot` → **Package settings** → **Manage access** → **Invite teams or people** →
  Role: **Read**.
- **Paketi herkese açık yapma.** Açık paket bir daha özel yapılamıyor
  ([GitHub Docs](https://docs.github.com/en/packages/learn-github-packages/configuring-a-packages-access-control-and-visibility)).
  Ayrıca imajdaki Fairino sürücüsünün (`frcobot_ros2`) lisansı yok
  ([frcobot_ros2 #40](https://github.com/FAIR-INNOVATION/frcobot_ros2/issues/40)).

## Web arayüzü

`http://localhost:8088` — `movecobot`'un yapabildiği her şey, artı fazlası.

| Bölüm | Ne yapar |
|---|---|
| **E-STOP** | Süren yörüngeyi iptal eder ve SDK üzerinden `StopMotion` gönderir. Hareket kilidini beklemez. |
| **Canlı durum** | Eklem açıları, TCP pozu, fault kodları, gripper durumu — WebSocket ile akar |
| **Eklem kontrolü** | Her ekseni tek tek sür, ya da altı açıyı birden ver |
| **Jog — base ekseni** | TCP'yi sabit dünya eksenlerinde ötele: +Z her zaman yukarı, alet nasıl dönmüş olursa olsun. Yönelim değişmez |
| **Jog — araç ekseni** | TCP'yi kendi eksenlerinde ötele (eksenler aletle birlikte döner) — yaklaşma/geri çekilme için |
| **Mutlak TCP pozu** | X/Y/Z/RX/RY/RZ'yi base çerçevesinde doğrudan ver; "Şu anki" ile doldurup tek değeri değiştir |
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
