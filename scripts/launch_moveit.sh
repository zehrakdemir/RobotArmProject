#!/bin/bash
# SIMULATOR (SimMachine) ile MoveIt2 baslatir.
#
# GERCEK ROBOT ICIN BUNU DEGIL fairino_launch.sh'i kullan - o hangi model
# ve surum bagliysa otomatik algilar. Bu betik SimMachine'in sabit FR5
# yapilandirmasina gore yazilmistir.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! docker ps --filter "name=fairino-simmachine" --filter "status=running" | grep -q fairino-simmachine; then
  echo "SimMachine ayakta değil, başlatılıyor..."
  bash "$SCRIPT_DIR/run_simmachine.sh"
  sleep 8
fi

# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
# shellcheck source=/dev/null
source "${FAIRINO_ROS2_WS:-$HOME/ros2_ws}/install/setup.bash"

# WSLg'de glamor/GBM tutarsiz calisiyor; bu olmadan rviz'in kendi 3B
# render'i icerik cizmeyen bir duruma girebiliyor.
export LIBGL_ALWAYS_SOFTWARE=1

ros2 launch fairino5_v6_moveit2_config demo.launch.py
