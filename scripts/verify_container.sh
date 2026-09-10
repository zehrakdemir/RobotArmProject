#!/bin/bash
# Container (ya da native kurulum) gercekten calisir durumda mi?
#
# Kullanim:
#   docker compose run --rm cobot bash /app/scripts/verify_container.sh
#   bash scripts/verify_container.sh          # native kurulumda da calisir
#
# ROBOTA BAGLANMAZ - sadece ortamin dogru kuruldugunu dogrular. Bu yuzden
# robot yokken de kosturulabilir.
#
# NEDEN VAR: imaj "basariyla" derlenip ise yaramaz cikabiliyor. Gercek bir
# ornek: `colcon build --packages-select moveit_py` paketi bulamayinca UYARI
# verip 0 ile cikiyor - build yesil, install/ bombos, robot hic surulemiyor.
# Bu betik tam olarak boyle sessiz bosluklari yakalamak icin.

hata=0

basarili() { printf '  \033[32mOK\033[0m    %s\n' "$1"; }
basarisiz() { printf '  \033[31mHATA\033[0m  %s\n' "$1"; hata=1; }

echo "=== 1) Surucu yamasi ==="
CPP="${FAIRINO_ROS2_WS:-$HOME/ros2_ws}/src/frcobot_ros2/fairino_hardware/src/fairino_hardware_interface.cpp"
if [ -f "$CPP" ]; then
    # Bu dortu olmadan robot sicramali calisir ve hiz limiti faultu verebilir.
    for k in _servo_com_type SERVOJ_CMDT MAX_SAFE_JOINT_VEL_DEG_S IDLE_EXIT_S robot_ip; do
        if grep -q "$k" "$CPP"; then basarili "$k"; else basarisiz "$k yama icinde yok"; fi
    done
else
    basarisiz "surucu kaynagi bulunamadi: $CPP"
fi

echo
echo "=== 2) Surucu derlendi mi ==="
if find "${FAIRINO_ROS2_WS:-$HOME/ros2_ws}/install" -name 'libfairino_hardware*.so' 2>/dev/null | grep -q .; then
    basarili "libfairino_hardware.so"
else
    basarisiz "libfairino_hardware.so uretilmemis"
fi

echo
echo "=== 3) Python tarafi ==="
python3 - <<'PY'
import sys

sonuc = 0


def dene(ad, fn):
    global sonuc
    try:
        fn()
        print(f"  \033[32mOK\033[0m    {ad}")
    except Exception as e:
        print(f"  \033[31mHATA\033[0m  {ad} -> {type(e).__name__}: {e}")
        sonuc = 1


def moveit_py():
    from moveit.core.robot_state import RobotState  # noqa: F401
    from moveit.planning import MoveItPy  # noqa: F401


def mtc():
    # MTC ile moveit_py ayni "moveit" ad alanini paylasiyor; biri digerini
    # golgeleyebiliyor. pickplace.py bunu extend_path ile cozuyor.
    import pkgutil

    import moveit
    moveit.__path__ = pkgutil.extend_path(moveit.__path__, "moveit")
    from moveit.task_constructor import core, stages  # noqa: F401


def sdk():
    # SDK sys.path'e calisma aninda ekleniyor - ciplak "import fairino"
    # calismaz, ensure_sdk_on_path() sonrasi calisir.
    from fairino_cobot import paths
    paths.ensure_sdk_on_path()
    from fairino import Robot  # noqa: F401


def uygulama():
    # Asil kanit: uygulamanin kendi modulleri yukleniyor mu.
    import importlib
    for m in ("fairino_cobot.movecobot", "fairino_cobot.pickplace",
              "fairino_cobot.gripper_controller", "fairino_cobot.konumcobot"):
        importlib.import_module(m)


dene("moveit_py (MoveItPy, RobotState)", moveit_py)
dene("moveit_configs_utils", lambda: __import__("moveit_configs_utils"))
dene("MoveIt Task Constructor", mtc)
dene("Fairino SDK", sdk)
dene("uygulama modulleri", uygulama)
sys.exit(sonuc)
PY
[ $? -ne 0 ] && hata=1

echo
echo "=== 4) Komutlar ==="
for c in movecobot konumcobot detect-robot gripper-gui fairino-setup-target; do
    if command -v "$c" > /dev/null 2>&1; then basarili "$c"; else basarisiz "$c yok"; fi
done

echo
echo "=== 5) MoveIt config yamalari ==="
bulunan=0
for f in "${FAIRINO_ROS2_WS:-$HOME/ros2_ws}"/src/frcobot_ros2/fairino*_v6_moveit2_config/config/ompl_planning.yaml; do
    [ -f "$f" ] || continue
    bulunan=$((bulunan + 1))
    pkg=$(basename "$(dirname "$(dirname "$f")")")
    if grep -q "longest_valid_segment_fraction" "$f" && grep -q "AddRuckigTrajectorySmoothing" "$f"; then
        basarili "$pkg (jerk siniri + carpisma kontrol sikligi)"
    else
        basarisiz "$pkg yamasiz"
    fi
done
[ "$bulunan" -eq 0 ] && basarisiz "hicbir model paketi bulunamadi"

echo
echo "=== 6) JTC hedef toleranslari ==="
for f in "${FAIRINO_ROS2_WS:-$HOME/ros2_ws}"/src/frcobot_ros2/fairino*_v6_moveit2_config/config/ros2_controllers.yaml; do
    [ -f "$f" ] || continue
    pkg=$(basename "$(dirname "$(dirname "$f")")")
    if grep -q "goal_time" "$f"; then basarili "$pkg"; else basarisiz "$pkg constraints yok"; fi
done

echo
echo "=== 7) Yollar ==="
python3 - <<'PY'
from fairino_cobot import paths

for ad, fn in [("SDK", paths.sdk_dir), ("ayna", paths.mirror_dir),
               ("ros2_ws", paths.ros2_ws), ("yamalar", paths.patches_dir),
               ("durum", paths.state_dir)]:
    try:
        print(f"  {ad:9s} {fn()}")
    except Exception as e:
        print(f"  {ad:9s} \033[31mHATA\033[0m {str(e).splitlines()[0]}")
PY

echo
if [ "$hata" -eq 0 ]; then
    printf '\033[32m=== ORTAM HAZIR ===\033[0m\n'
    echo "Robotu surmek icin ayrica: gercek IP (.env / FAIRINO_ROBOT_IP),"
    echo "host network, ve RT onceligi (cap_add SYS_NICE + ulimits.rtprio)."
else
    printf '\033[31m=== EKSIKLER VAR (yukari bak) ===\033[0m\n'
fi
exit "$hata"
