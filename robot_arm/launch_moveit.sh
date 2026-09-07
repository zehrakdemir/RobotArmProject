#!/bin/bash
if ! docker ps --filter "name=fairino-simmachine" --filter "status=running" | grep -q fairino-simmachine; then
  echo "SimMachine ayakta değil, başlatılıyor..."
  bash ~/fairino/run_simmachine.sh
  sleep 8
fi
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch fairino5_v6_moveit2_config demo.launch.py
