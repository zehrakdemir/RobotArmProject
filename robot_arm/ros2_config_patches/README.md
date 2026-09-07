# ROS2 / MoveIt2 config yamaları

Bu klasördeki dosyalar, `frcobot_ros2` (FAIR-INNOVATION'ın resmi ROS2 sürücü paketi,
bkz. ana `SETUP.md`) içindeki ilgili config dosyalarının ÜZERİNE, aynı göreli yolla
kopyalanacak "override" dosyalarıdır — vendor reposunun tamamı burada TEKRAR
yayınlanmıyor, sadece bizim değiştirdiğimiz dosyalar.

## fairino3_v6_moveit2_config/config/ompl_planning.yaml

`AddRuckigTrajectorySmoothing` + `AddTimeOptimalParameterization`'ı BİRLİKTE kullanacak
şekilde düzeltildi (varsayılan config sadece TOTG içeriyordu). Küçük eklem hareketlerinde
bile gerçek robotta yanlış-pozitif "shaft collision fault" alınmasının kök sebebiydi —
ayrıntılı açıklama dosyanın kendi içindeki yorumda ve `SETUP.md`'deki
"Shaft collision fault — kök sebep ve çözüm" bölümünde.

Uygulamak için:
```bash
cp ros2_config_patches/fairino3_v6_moveit2_config/config/ompl_planning.yaml \
   ~/ros2_ws/src/frcobot_ros2/fairino3_v6_moveit2_config/config/ompl_planning.yaml
cd ~/ros2_ws && colcon build --packages-select fairino3_v6_moveit2_config
```
