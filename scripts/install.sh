#!/bin/bash
# Native kurulum: paketi kurar, konsol komutlarini ve alias'lari hazirlar.
#
# Kullanim:
#   bash scripts/install.sh
#
# Ne YAPMAZ: ROS2 Humble, MoveIt2 ve Fairino SDK'yi kurmaz - onlar
# docs/SETUP.md'de anlatiliyor. Bu betik sadece BU repoyu devreye alir.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

echo "== Repo: $REPO_ROOT"
echo

# ---------------------------------------------------------------------------
# .env
# ---------------------------------------------------------------------------
if [ ! -f .env ]; then
    cp .env.example .env
    echo "== .env olusturuldu (.env.example'dan kopyalandi)."
    echo "   Robot IP'si ve yollari oraya yazabilirsin - hepsi istege bagli."
else
    echo "== .env zaten var, dokunulmadi."
fi
echo

# ---------------------------------------------------------------------------
# Paket
# ---------------------------------------------------------------------------
# --user: sistem Python'una yazmamak icin. Bir sanal ortam icindeysen
# (VIRTUAL_ENV tanimliysa) --user gecersiz olur, o yuzden ayirt ediliyor.
echo "== Paket kuruluyor (editable)"
if [ -n "${VIRTUAL_ENV:-}" ]; then
    python3 -m pip install -e .
else
    python3 -m pip install --user -e .
fi
echo

# ---------------------------------------------------------------------------
# Konsol komutlari PATH'te mi
# ---------------------------------------------------------------------------
USER_BIN="$(python3 -c 'import site,os; print(os.path.join(site.USER_BASE, "bin"))')"
if [ -z "${VIRTUAL_ENV:-}" ] && [[ ":$PATH:" != *":$USER_BIN:"* ]]; then
    echo "UYARI: $USER_BIN PATH'te degil - 'movecobot' komutu bulunamaz."
    echo "       ~/.bashrc'ye ekle:"
    echo "         export PATH=\"\$PATH:$USER_BIN\""
    echo
fi

# ---------------------------------------------------------------------------
# Kurulum dogrulamasi
# ---------------------------------------------------------------------------
echo "== Yollar cozuluyor"
python3 - <<'PY'
from fairino_cobot import paths

def satir(ad, fn):
    try:
        print(f"   {ad:12s} {fn()}")
    except paths.PathError as e:
        print(f"   {ad:12s} BULUNAMADI")
        print(f"                {str(e).splitlines()[0]}")

satir("SDK", paths.sdk_dir)
satir("detect", paths.detect_robot_script)
satir("durum", paths.state_dir)
satir("ros2_ws", paths.ros2_ws)
ip = paths.robot_ip_override()
print(f"   {'robot IP':12s} {ip if ip else '(verilmedi - ag taranacak)'}")
PY
echo

# ---------------------------------------------------------------------------
# ROS2 yamalari
# ---------------------------------------------------------------------------
echo "== ROS2 vendor yamalari (dry-run - uygulamak icin --dry-run'siz calistir)"
python3 tools/apply_ros2_patches.py --dry-run 2>&1 | sed 's/^/   /' || \
    echo "   (frcobot_ros2 bulunamadi - docs/SETUP.md'ye bak)"
echo

cat <<'EOF'
== Bitti.

Komutlar:
    movecobot          kol + gripper kontrolu (ana arac)
    konumcobot         sadece konum okuma
    gripper-gui        gripper icin Tkinter arayuzu
    detect-robot       hangi robot bagli, bul ve yazdir

MoveIt2'yi baslatmak icin:
    bash scripts/fairino_launch.sh

ROS2 yamalari henuz uygulanmadiysa:
    python3 tools/apply_ros2_patches.py && cd ~/ros2_ws && colcon build
EOF
