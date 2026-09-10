# Hücre Modu Kurulumu — Native Ubuntu (FR3 makinesi)

WSL2'de USB kamera görünmez. Gerçek hücreyi (2 kamera + FR3) **FR3'ün bağlı
olduğu native Ubuntu makinede** çalıştırın. Aşağıdaki adımlar tam sıralı.

---

## 0. Ön koşullar (bir kez)

```bash
sudo apt update
sudo apt install -y docker.io docker-compose-v2 v4l-utils git
sudo usermod -aG docker $USER          # sonra: çıkış-giriş ya da `newgrp docker`
```

Projeyi bu makineye taşıyın:

```bash
git clone <repo>   # ya da valiz-olcum-robot.zip'i açın
cd valiz-olcum-robot
cp .env.example .env
```

---

## 1. Kameraları bağla ve tanı

İki Logitech C310'u USB'ye tak. Sonra:

```bash
v4l2-ctl --list-devices
```

Her C310 genelde 2 düğüm gösterir (`/dev/video0`+`/dev/video1`, `/dev/video2`+`/dev/video3`).
Hangisi capture, hangisi sol/sağ belirsizse imaj kurulduktan sonra (adım 3):

```bash
docker compose run --rm valiz-olcum python -m tools.list_cameras --out cam_probe
```

`cam_probe/videoN.png` önizlemelerine bak. **Alana bakarken solda kalan** kamera = sol.

`.env`'ye yaz:

```
CAM_LEFT_DEV=/dev/video0
CAM_RIGHT_DEV=/dev/video2
GRIPPER_MAX_OPENING_MM=80     # <<< test Jodell'ini cetvelle ölç, gerçek değeri yaz
GROUND_Z_MM=0                 # robot tabanında zemin Z'si
PLACE_POSE=500,0,150,180,0,0  # hedef bırakma pozu
ROBOT_IP=192.168.58.2
```

> `docker-compose.cell.yml` host'taki bu iki cihazı konteyner içinde **sabit
> `/dev/video0` (sol)** ve **`/dev/video1` (sağ)** yapar; kod hep index 0/1 kullanır.

---

## 2. Fiziksel montaj

- İki kamera rijit bir çubukta, **aynı yükseklik, lensler paralel**, FR3'ün
  önündeki bırakma bölgesine tepeden bakacak.
- **Baseline** (lens arası): ~12–18 cm başla. İkisi de tüm bölgeyi bol
  örtüşmeyle görsün.
- **Cıvatayla sabitle.** Oynarsa stereo + hand-eye kalibrasyonu bozulur.
- Işık eşit/dağınık, valiz kabuğunda parlama olmasın.
- Pozlamayı kilitle (tutarlılık için):
  ```bash
  for d in $CAM_LEFT_DEV $CAM_RIGHT_DEV; do
    v4l2-ctl -d $d -c auto_exposure=1 -c white_balance_automatic=0
  done
  ```

---

## 3. İmajı kur ve başlat

```bash
docker compose -f docker-compose.yml -f docker-compose.cell.yml up -d --build
```

Sağlık: `curl -s localhost:8501/_stcore/health` → `ok`.
Arayüz: `http://localhost:8501` → sol menüde **Hucre Modu**.

Kısayol (`~/.bashrc`):
```bash
alias cell='cd ~/valiz-olcum-robot && docker compose -f docker-compose.yml -f docker-compose.cell.yml'
# kullanım:  cell up -d   /   cell run --rm valiz-olcum python -m tools.run_cell --cams --json
```

---

## 4. Üç kalibrasyon (bir kez, kameralar sabitlendikten SONRA)

Hepsi konteyner içinde, GUI yok:

```bash
C='docker compose -f docker-compose.yml -f docker-compose.cell.yml run --rm valiz-olcum'

# 4.1 Stereo: satranç tahtası A4 çıktısını iki kameraya ~20 pozda göster
$C python -m tools.capture_pairs --out calib_shots --count 20 --interval 1.5
$C python -m tools.stereo_calibrate --shots calib_shots --cols 9 --rows 6 --square-mm 25
#   -> src/calibration/stereo_calib.yaml   (RMS < ~0.5 px hedefle)

# 4.2 Hand-eye: robot ucuna ArUco marker (DICT_4X4_50, örn 40 mm) yapıştır.
#     Robotu ~8 farklı poza götür; her pozda:
$C python -m tools.capture_pairs --out calib_shots --fr3 --count 8 --interval 3
#     (--fr3: her karede FR3 TCP pozunu da yazar; ROBOT_ENABLED=true gerekir)
$C python -m tools.handeye_solve --shots calib_shots --marker-mm 40
#   -> src/calibration/base_T_cam.yaml     (artık hata < ~10 mm hedefle)
```

`src/calibration/` klasörü host'a mount edildiği için dosyalar kalıcıdır.

---

## 5. Çalıştır

**Arayüz:** `http://localhost:8501` → Hucre Modu → "Kameralardan çek" → Çalıştır.

**CLI:**
```bash
$C python -m tools.run_cell --cams --json --ply bulut.ply
$C python -m tools.run_cell --cams --execute        # FR3'e gönder (ROBOT_ENABLED=true)
```

Akış: valiz mi? → 3B bulut (robot taban) → OBB + duruş →
**DİK** ise kavrama planı (2 düz kabuk ortası, açılma ≤ gripper) →
**YATIK** ise önce dik konuma getirme (tip-up, dry-run doğrula) → FR3 programı.

---

## Güvenlik

- İlk denemeler **`ROBOT_ENABLED=false`** (dry-run) — sadece plan loglanır.
- `true` yapmadan: acil-stop erişilebilir, robot çevresinde kimse yok,
  hız düşük (`ROBOT_TCP_SPEED=10`), tip-up manevrasını adım adım doğrula.
- `base_T_cam.yaml` yoksa FR3 pozları **geçersiz** (birim matris) — arayüz uyarır.
