#!/bin/bash
# TAM OTOMATIK: hangi model (FR3/FR5/FR10/FR16/FR20/FR30...) ve hangi
# sürüm olursa olsun, robota sorup öğrenir, resmi mirror'dan doğru
# kaynağı bulup kurar, derler, başlatır. Elle hiçbir şey seçmiyorsun.
#
# Kullanım:
#   bash auto_fairino_launch.sh

set -e

# LIBGL_ALWAYS_SOFTWARE=1: bir ara GPU hizlandirmali render (glamor) sorunsuz
# calistigi icin bu satir kaldirilmisti (yazilim render'i rviz'i yuksek
# CPU'da sinirsiz FPS ile dondurup pencerenin donmus/kapanmis gibi
# gorunmesine sebep oluyordu). Ama sonradan makinede glamor/GBM tekrar
# calismaz oldu ("Xwayland glamor: GBM Wayland interfaces not available,
# falling back to sw" - Xwayland zaten kendiligind en yazilima duesuyor
# ama Mesa/OpenGL tarafinda LIBGL_ALWAYS_SOFTWARE olmadan rviz'in kendi
# 3B render'i (Ogre) tutarsiz/render-etmeyen bir duruma giriyordu, pencere
# sadece gorev cubugunda goruklu icerik cizilmiyordu). Bu satir GERI
# EKLENDI - eger ileride glamor tekrar duzelirse (WSLg/surucu guncellemesi)
# ve performans sorun olursa, tekrar kaldirmadan once glamor durumunu
# ("Xwayland glamor" satirini stderr.log'da) kontrol et.
export LIBGL_ALWAYS_SOFTWARE=1

DETECT_SCRIPT="$HOME/detect_robot.py"
SETUP_SCRIPT="$HOME/setup_fairino_target.py"
ROS2_WS="$HOME/ros2_ws"
STATE_FILE="$HOME/.fairino_last_target"

echo "================================================"
echo " Robot algılanıyor (ağ taraması + GetSoftwareVersion)..."
echo "================================================"

DETECT_OUTPUT=$(python3 "$DETECT_SCRIPT" --full)
STATUS=$?

if [ $STATUS -ne 0 ] || [ -z "$DETECT_OUTPUT" ]; then
    echo ""
    echo "HATA: Robot algılanamadı, yukarıdaki log satırlarına bak."
    exit 1
fi

eval "$(echo "$DETECT_OUTPUT" | grep -E '^[A-Z_][A-Z0-9_]*=')"

echo ""
echo "================================================"
echo " Algılanan: FR${ROBOT_MODEL_NUM}  ${ROBOT_VERSION_MAJ}.${ROBOT_VERSION_MIN}.${ROBOT_VERSION_MIN2}  (${ROBOT_IP})"
echo "================================================"

echo ""
echo "================================================"
echo " Doğru kaynak kuruluyor ve derleniyor..."
echo "================================================"
python3 "$SETUP_SCRIPT" "$ROBOT_MODEL_NUM" "$ROBOT_VERSION_MAJ" "$ROBOT_VERSION_MIN" "$ROBOT_VERSION_MIN2" "$ROBOT_IP"

if [ ! -f "$STATE_FILE" ]; then
    echo "HATA: $STATE_FILE oluşmadı, setup_fairino_target.py başarısız olmuş olabilir."
    exit 1
fi
source "$STATE_FILE"

echo ""
echo "================================================"
echo " ros2 launch $MOVEIT_PACKAGE demo.launch.py"
echo "================================================"
# ws_moveit2 moveit_configs_utils gibi Python modullerini sagliyor;
# bu source olmadan launch "ModuleNotFoundError: moveit_configs_utils"
# ile patliyor ve rviz hic acilmiyordu.
source "$HOME/ws_moveit2/install/setup.bash"
source "$ROS2_WS/install/setup.bash"

if [ "$ROBOT_IP" != "127.0.0.1" ]; then
    echo "UYARI: Gerçek robot hareket edecek. Acil stop elinin altında olsun."
    read -rp "Devam etmek için Enter'a bas (iptal için Ctrl+C)..." _
fi

ros2 launch "$MOVEIT_PACKAGE" demo.launch.py
