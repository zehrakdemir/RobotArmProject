# ROS2 / MoveIt2 config yamaları

`frcobot_ros2` (FAIR-INNOVATION'ın resmi ROS2 sürücü paketi, bkz. ana `SETUP.md`),
desteklediği HER Fairino modeli için ayrı bir `fairino<N>_v6_moveit2_config` paketi
içeriyor (ör. `fairino3_v6_moveit2_config`, `fairino5_v6_moveit2_config`,
`fairino10_v6_moveit2_config`, ...). `movecobot.py` hangi robot bağlıysa onu otomatik
algılayıp (`detect_robot()`) doğru paketi seçiyor — yani bu düzeltmenin TEK bir modele
değil, olası HER modele uygulanmış olması gerekiyor, aksi halde farklı bir robot takıldığında
düzeltmesiz (varsayılan) config'e geri düşülür.

## ompl_planning.yaml

`AddRuckigTrajectorySmoothing` + `AddTimeOptimalParameterization`'ı BİRLİKTE kullanacak
şekilde düzeltildi (varsayılan config sadece TOTG içeriyordu, jerk sınırsızdı). Küçük eklem
hareketlerinde bile gerçek robotta yanlış-pozitif "shaft collision fault" alınmasının kök
sebebiydi — ayrıntılı açıklama dosyanın kendi içindeki yorumda ve `SETUP.md`'deki
"Shaft collision fault — kök sebep ve çözüm" bölümünde.

Uygulamak için (`frcobot_ros2` altında BULUNAN HER model paketine, tek seferde):
```bash
bash ros2_config_patches/apply_patch.sh          # varsayılan: ~/ros2_ws/src/frcobot_ros2
# farklı bir yoldaysa: bash ros2_config_patches/apply_patch.sh /baska/yol/frcobot_ros2
cd ~/ros2_ws && colcon build
```
FAIR-INNOVATION `frcobot_ros2`'ye ileride yeni bir model paketi eklerse, script'i tekrar
çalıştırmak yeterli — klasör adlarını elle takip etmeye gerek yok.
