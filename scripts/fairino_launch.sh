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

# ---------------------------------------------------------------------------
# GERCEK ZAMANLI (RT) ONCELIK
# ---------------------------------------------------------------------------
# ros2_control baslarken su uyariyi veriyordu:
#   "Could not enable FIFO RT scheduling policy: Operation not permitted"
# Yani controller_manager kontrol dongusunu gercek-zamanli oncelikle
# calistiramiyor. Sonucu: dongu duzenli 100Hz yerine gecikip sonra PATLAMA
# halinde yetismeye calisiyor (canli logda write() cagrilari 10ms yerine
# 3ms arayla goruldu). Robota giden ServoJ akisi boylece duzensizlesiyor ->
# hareket "sicramali" oluyor ve robot "axis N command speed exceeds the
# limit" faultu verebiliyor. (Ayni kok sebep Universal Robots surucusunde
# de tespit edilmis: ur_modern_driver issue #153 - "ag paketleri
# geciktiginde robot ani sekilde yetismeye calisiyor, hareket sicramali
# oluyor".)
#
# Gerekli izin makinede ASLINDA TANIMLI:
#   /etc/security/limits.d/99-realtime.conf -> @realtime grubuna rtprio 99
#   ve kullanici zaten 'realtime' grubunda.
# Ama WSL oturumlari tam bir PAM login sureci uzerinden acilmadigi icin
# limits.conf HIC uygulanmiyor (ulimit -r = 0 kaliyor). PAM'i devreye sokan
# bir yol (sudo) uzerinden baslatinca limit dogru uygulaniyor (rtprio 99).
# -E ortami koruyor (rviz'in DISPLAY/WAYLAND_DISPLAY/XDG_RUNTIME_DIR'i dahil).
if sudo -n -u "$USER" true 2>/dev/null && \
   [ "$(sudo -n -u "$USER" bash -c 'ulimit -r' 2>/dev/null)" != "0" ]; then
    echo "RT oncelik izni mevcut (rtprio 99) - kontrol dongusu gercek-zamanli baslatiliyor."
    exec sudo -n -E -u "$USER" bash -c "
        source /opt/ros/humble/setup.bash
        source '$HOME/ws_moveit2/install/setup.bash'
        source '$ROS2_WS/install/setup.bash'
        exec ros2 launch '$MOVEIT_PACKAGE' demo.launch.py capabilities:=move_group/ExecuteTaskSolutionCapability"
else
    echo "UYARI: RT oncelik alinamiyor (ulimit -r = 0). Kontrol dongusu duzensiz"
    echo "       calisabilir - hareket sicramali olabilir, robot hiz limiti faultu"
    echo "       verebilir. Normal modda baslatiliyor."
    exec ros2 launch "$MOVEIT_PACKAGE" demo.launch.py capabilities:=move_group/ExecuteTaskSolutionCapability
fi
