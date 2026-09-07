#!/bin/bash
# clearcobot - movecobot'un ros2_control_node "respawn" mekanizmasindan
# (bkz. movecobot.py _respawn_ros2_control) kalabilecek SAHIPSIZ surecleri
# temizler. Manuel/drag-teach moddan cikarken ROS2 kontrolu yeniden aktive
# edilemezse movecobot, move_group/rviz'e dokunmadan YENI, AYRI bir
# ros2_control_node sureci baslatir (start_new_session=True) - movecobot
# kapansa bile bu surec KENDILIGINDEN olmez. Sahipsiz kalirsa robota 100Hz'de
# eski/tutarsiz bir hedefe ServoJ komutu gondermeye devam edebilir; bu da
# WebApp'te "Clear"a bassan bile hatanin (ornegin eksen hiz limiti asimi)
# aninda geri gelmesine yol acar.
#
# Bu surecler komut satirlarinda /tmp/movecobot_respawn_hw_ gecen, movecobot'a
# OZGU bir gecici launch dosyasindan baslatildigi icin sadece BUNLAR hedeflenir
# - calisan fairino_launch/movecobot oturumlarina DOKUNULMAZ.
#
# Kullanim: clearcobot (alias, ~/.bashrc)

set -u

collect_tree() {
    # $1: kok pid - kendisi + tum alt surecleri (2 seviye yeterli, ros2 launch
    # -> ros2_control_node dogrudan cocuk oluyor)
    local root="$1"
    local all="$root"
    local children
    children=$(pgrep -P "$root" 2>/dev/null || true)
    for c in $children; do
        all="$all $c"
        local grandchildren
        grandchildren=$(pgrep -P "$c" 2>/dev/null || true)
        all="$all $grandchildren"
    done
    echo "$all"
}

echo "movecobot'tan kalma sahipsiz surecler araniyor..."
ROOT_PIDS=$(pgrep -f "movecobot_respawn_hw" || true)

if [ -z "$ROOT_PIDS" ]; then
    echo "  Temiz - sahipsiz respawn sureci bulunamadi."
else
    ALL_PIDS=""
    for pid in $ROOT_PIDS; do
        ALL_PIDS="$ALL_PIDS $(collect_tree "$pid")"
    done
    ALL_PIDS=$(echo "$ALL_PIDS" | tr ' ' '\n' | sort -u | tr '\n' ' ')

    echo "  Bulundu:"
    ps -o pid,ppid,cmd -p $ALL_PIDS 2>/dev/null

    echo "  Kapatiliyor (SIGTERM)..."
    kill -TERM $ALL_PIDS 2>/dev/null || true
    sleep 2

    STILL=""
    for pid in $ALL_PIDS; do
        if kill -0 "$pid" 2>/dev/null; then
            STILL="$STILL $pid"
        fi
    done
    if [ -n "$STILL" ]; then
        echo "  Hala kapanmayanlar zorla kapatiliyor (SIGKILL)..."
        kill -KILL $STILL 2>/dev/null || true
    fi
    echo "  Kapatildi."
fi

rm -f /tmp/movecobot_respawn_hw_*.launch.py 2>/dev/null

echo ""
echo "Su an calisan ilgili surecler (bilgi amacli - bunlara dokunulmadi):"
FOUND=$(ps aux | grep -E "ros2_control_node|move_group|movecobot\.py|rviz2" | grep -v grep)
if [ -n "$FOUND" ]; then
    echo "$FOUND"
else
    echo "  (yok)"
fi
