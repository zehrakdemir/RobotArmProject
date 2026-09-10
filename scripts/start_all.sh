#!/bin/bash
# Container'in varsayilan komutu: ROS2 yigini + web arayuzu.
#
#   1) fairino_launch  (robot algila, surucuyu kur/derle, move_group + rviz)
#   2) movecobot-web   (HTTP arayuzu, varsayilan 8088)
#
# NEDEN TEK CONTAINER: web arayuzu robota SDK ile baglanmak zorunda ve
# Fairino'nun durum soketi tek-tuketicili - ayri bir container'dan ikinci bir
# baglanti acmak ikisini de kilitleyebiliyor. Ayrica arayuz MoveItPy
# kullaniyor, yani move_group ile ayni ROS ortaminda olmasi gerekiyor.
#
# SIRA ONEMLI: arayuz MoveItPy'yi baslatirken move_group'un ayakta olmasini
# bekliyor. O yuzden once launch, sonra arayuz.

set -uo pipefail

PORT="${MOVECOBOT_WEB_PORT:-8088}"
LAUNCH_LOG=/tmp/fairino_launch.log

temizle() {
    echo ""
    echo "Kapatiliyor..."
    [ -n "${WEB_PID:-}" ] && kill "$WEB_PID" 2> /dev/null
    [ -n "${LAUNCH_PID:-}" ] && kill "$LAUNCH_PID" 2> /dev/null
    wait 2> /dev/null
}
trap temizle INT TERM

echo "=== 1/2  fairino_launch baslatiliyor (log: $LAUNCH_LOG) ==="
bash /app/scripts/fairino_launch.sh > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

# move_group'un ROS grafinde gorunmesini bekle. Sabit bir "sleep 30" yerine
# gercekten hazir olmasini beklemek gerekiyor: makineye gore sure degisiyor
# ve surucu ilk acilista derleme yapabiliyor.
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
# shellcheck source=/dev/null
source /opt/moveit_py_ws/install/setup.bash
# shellcheck source=/dev/null
source "${FAIRINO_ROS2_WS:-/opt/fairino_ws}/install/setup.bash"

echo "move_group bekleniyor..."
hazir=0
for _ in $(seq 1 120); do
    if ! kill -0 "$LAUNCH_PID" 2> /dev/null; then
        echo "HATA: fairino_launch cikti. Son satirlar:"
        tail -25 "$LAUNCH_LOG"
        exit 1
    fi
    if ros2 node list 2> /dev/null | grep -q "move_group"; then
        hazir=1
        break
    fi
    sleep 2
done

if [ "$hazir" -ne 1 ]; then
    echo "UYARI: move_group 240 saniyede gorunmedi. Arayuz yine de aciliyor;"
    echo "       kola baglanamazsa fairino_launch loguna bak: $LAUNCH_LOG"
else
    echo "move_group hazir."
fi

echo ""
echo "=== 2/2  Web arayuzu: http://localhost:$PORT ==="
echo "UYARI: arayuzde kimlik dogrulamasi YOK ve GERCEK ROBOTU hareket ettirir."
echo ""

movecobot-web --port "$PORT" &
WEB_PID=$!

# Ikisinden biri olurse container'i duesur - sessizce yarim calismasin.
wait -n "$LAUNCH_PID" "$WEB_PID"
echo "Bilesenlerden biri sonlandi, container kapaniyor."
temizle
exit 1
