#!/bin/bash
# ompl_planning.yaml duzeltmesini (Ruckig+TOTG birlikte kullanimi - "shaft
# collision fault" yanlis-pozitifini onler, bkz. README.md) frcobot_ros2
# altinda BULUNAN HER Fairino model paketine uygular - hangi model robot
# takiliysa (movecobot.py bunu detect_robot() ile otomatik algiliyor ve
# f"fairino{model_num}_v6_moveit2_config" paketini kullaniyor) o paket zaten
# bu duzeltmeye sahip olsun diye. Yeni bir model paketi eklenirse (FAIR-
# INNOVATION frcobot_ros2'ye yeni bir robot eklerse) script'i TEKRAR
# calistirmak yeterli - klasor adini elle degistirmene gerek yok.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FRCOBOT_DIR="${1:-$HOME/ros2_ws/src/frcobot_ros2}"

if [ ! -d "$FRCOBOT_DIR" ]; then
  echo "HATA: $FRCOBOT_DIR bulunamadi. Kullanim: $0 [frcobot_ros2 klasoru]"
  exit 1
fi

applied=0
for d in "$FRCOBOT_DIR"/fairino*_v6_moveit2_config; do
  [ -d "$d/config" ] || continue
  cp "$SCRIPT_DIR/ompl_planning.yaml" "$d/config/ompl_planning.yaml"
  echo "uygulandi: $d/config/ompl_planning.yaml"
  applied=$((applied + 1))
done

if [ "$applied" -eq 0 ]; then
  echo "UYARI: '$FRCOBOT_DIR' altinda hicbir fairino*_v6_moveit2_config klasoru bulunamadi."
  exit 1
fi

echo
echo "$applied paket guncellendi. Simdi yeniden derle:"
echo "  cd ~/ros2_ws && colcon build --packages-select \$(basename -a $FRCOBOT_DIR/fairino*_v6_moveit2_config)"
