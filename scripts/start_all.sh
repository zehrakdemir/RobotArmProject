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
# ROS2'nin setup.bash'i tanimsiz degiskenlere dokunuyor
# (AMENT_TRACE_SETUP_FILES). "set -u" acikken bu betigi OLDURUYOR:
#   /opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
# Bu yuzden source'lar suresince -u kapatiliyor, sonra geri aciliyor.
set +u
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
# shellcheck source=/dev/null
source "${FAIRINO_MOVEIT_WS:-/opt/moveit_py_ws}/install/setup.bash"
# shellcheck source=/dev/null
source "${FAIRINO_ROS2_WS:-/opt/fairino_ws}/install/setup.bash"
set -u

# ARAYUZ SINIRSIZ BEKLETILMEZ - BU BILEREK BOYLE.
#
# Eskiden arayuz, move_group gorunene kadar bekletiliyordu ve bekleme
# basarisiz olursa container komple duesuyordu. Robot ya da simulator hazir
# degilse kullanicinin elinde HICBIR SEY kalmiyordu: ne ekran ne hata,
# sadece "localhost:8088 baglanamadi".
#
# Oysa arayuz zaten kendi basina yeniden deniyor (RobotSession.start) ve
# baglanamadiginda NEDENINI ekranda gosteriyor - yani en cok ihtiyac duyuldugu
# an, tam da launch'in takildigi andir.
#
# NEDEN YINE DE BIRAZ BEKLENIYOR: fairino_launch robotun modeline gore YENI
# bir paket kurabiliyor (simulator FR5 ise fairino5_v6_moveit2_config).
# Arayuz baslamadan once ortamin tazelenmesi gerek, yoksa:
#   PackageNotFoundError: package 'fairino5_v6_moveit2_config' not found
# Bu yuzden: sinirli sure bekle, olmazsa yine de ac.
echo "move_group bekleniyor (en fazla 3 dakika; olmazsa arayuz yine acilir)..."
for _ in $(seq 1 90); do
    if ! kill -0 "$LAUNCH_PID" 2> /dev/null; then
        echo "UYARI: fairino_launch cikti. Son satirlar:"
        tail -20 "$LAUNCH_LOG"
        echo "Arayuz yine de aciliyor - sebebini orada da gorebilirsin."
        break
    fi
    if ros2 node list 2> /dev/null | grep -q "move_group"; then
        echo "move_group hazir."
        break
    fi
    sleep 2
done

# ORTAMI TAZELE (yukaridaki nedenle).
set +u
# shellcheck source=/dev/null
source "${FAIRINO_ROS2_WS:-/opt/fairino_ws}/install/setup.bash"
set -u

echo ""
echo "=== 2/2  Web arayuzu: http://localhost:$PORT ==="
echo "UYARI: arayuzde kimlik dogrulamasi YOK ve GERCEK ROBOTU hareket ettirir."
echo ""

movecobot-web --port "$PORT" &
WEB_PID=$!

# SADECE ARAYUZ OLURSE container kapanir.
#
# Eskiden "ikisinden biri olurse kapat" deniyordu. Ama fairino_launch robot
# yokken oluyor ve o zaman arayuz de birlikte gidiyordu - oysa kullanicinin
# tam o anda arayuze ihtiyaci var: sebebi orada yaziyor ve robot gelince
# kendiliginden baglaniyor.
#
# Launch olurse container ayakta kalir; robotu takip fairino_launch'i elle
# yeniden baslatmak yeterli:
#   docker exec -d cobot bash /app/scripts/fairino_launch.sh
wait "$WEB_PID"
echo "Web arayuzu sonlandi, container kapaniyor."
temizle
exit 1
