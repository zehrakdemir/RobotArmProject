# auto_launch — otomatik robot algılama ve kurulum

`fairino_launch` alias'ının çalıştırdığı script'ler. Ayrıntılı açıklama için ana
`../SETUP.md`'deki "fairino_launch — Otomatik Robot Algılama ve Kurulum" bölümüne bak; burada
sadece kısa özet var.

- **`detect_robot.py`** — Windows + WSL ağ arayüzlerini tarayıp Fairino RPC'sine (port 20003)
  cevap veren cihazı bulur, `GetSoftwareVersion()` ile model + sürümü okur. Bir önbellek
  dosyası (`~/.fairino_last_target`) kullanarak tekrar taramaları hızlandırır.
- **`setup_fairino_target.py`** — algılanan model+sürüme uygun `fairino_hardware` ve
  `fairino<N>_v6_moveit2_config` kaynağını bir yerel mirror'dan (`~/fairino_official_mirror/`,
  bu repoya DAHİL DEĞİL — bkz. ana SETUP.md) bulup workspace'e kurar, `robot_ip` parametresini
  otomatik yamalar, sadece değişen paketleri derler.
- **`auto_fairino_launch.sh`** — ikisini sırayla çağırıp `ros2 launch` ile MoveIt2'yi başlatır.

## Kurulum (bu repodan kopyalarken)

Bu üç dosya normalde `$HOME` altında duruyor (`~/detect_robot.py`,
`~/setup_fairino_target.py`, `~/auto_fairino_launch.sh`) ve `~/.bashrc`'deki
```bash
alias fairino_launch='bash ~/auto_fairino_launch.sh'
```
alias'ı onları oradan çağırıyor. Bu repodan yeni bir makineye kurarken:
```bash
cp auto_launch/detect_robot.py auto_launch/setup_fairino_target.py auto_launch/auto_fairino_launch.sh ~/
chmod +x ~/auto_fairino_launch.sh
echo "alias fairino_launch='bash ~/auto_fairino_launch.sh'" >> ~/.bashrc
```

## Bağımlılıklar (bu repoya dahil DEĞİL)
- `~/fairino_official_mirror/` — `setup_fairino_target.py`'nin kaynak aradığı yerel mirror,
  ayrıca (Windows tarafında indirilip WSL'e kopyalanan) bir arşivden elde ediliyor.
- `~/ros2_ws/src/frcobot_ros2/` — temel iskelet, ana `SETUP.md`'deki "ROS2 + MoveIt2 +
  fairino_hardware Kurulumu" bölümündeki adımlarla bir kere klonlanmalı.
