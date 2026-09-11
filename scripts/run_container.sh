#!/bin/bash
# Container'i dogru bayraklarla baslatir. Elle "docker run ..." yazmak yerine
# bunu kullan - bayraklarin biri eksik kalirsa robot ya bulunamaz ya da
# hareket bozulur, ve bunu ancak kol sicramaya baslayinca fark edersin.
#
# Kullanim:
#   bash scripts/run_container.sh              # gercek robot
#   bash scripts/run_container.sh --sim        # SimMachine simulatoru
#   bash scripts/run_container.sh --logs       # baslat ve loglari izle
#   bash scripts/run_container.sh --stop       # durdur
#
# Acilinca: http://localhost:8088
#
# --sim NE YAPIYOR: SimMachine ayri bir container ve kendi ag'inda
# (fairino-net, 192.168.58.2) duruyor. Bizim container'i AYNI aga baglayip
# robot IP'si olarak onu veriyoruz. 127.0.0.1 ISE YARAMAZ - container'in
# icinde o adres container'in kendisidir, simulator degil.

set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AD="${COBOT_CONTAINER:-cobot}"
IMAJ="${COBOT_IMAGE:-fairino-cobot:runtime}"
PORT="${MOVECOBOT_WEB_PORT:-8088}"
DURUM_DIZINI="${FAIRINO_STATE_DIR:-$HOME/.local/state/fairino-cobot}"

SIM=0
LOGLAR=0
for arg in "$@"; do
    case "$arg" in
        --stop)
            docker rm -f "$AD" > /dev/null 2>&1 && echo "$AD durduruldu." || echo "$AD zaten calismiyor."
            exit 0 ;;
        --sim)  SIM=1 ;;
        --logs) LOGLAR=1 ;;
        *) echo "Bilinmeyen secenek: $arg"; echo "Kullanim: $0 [--sim] [--logs] [--stop]"; exit 1 ;;
    esac
done

if ! docker info > /dev/null 2>&1; then
    echo "HATA: Docker daemon'a ulasilamiyor."
    echo "  Docker Desktop acik mi? WSL2'de calisiyorsan:"
    echo "  Docker Desktop -> Settings -> Resources -> WSL Integration ->"
    echo "  dagitimini ac -> Apply & Restart."
    exit 1
fi

if ! docker image inspect "$IMAJ" > /dev/null 2>&1; then
    echo "HATA: '$IMAJ' imaji yok. Once derle:"
    echo "  cd $REPO && docker build --target runtime -t $IMAJ ."
    exit 1
fi

if [ ! -f "$REPO/.env" ]; then
    echo "UYARI: .env yok, .env.example'dan olusturuluyor."
    cp "$REPO/.env.example" "$REPO/.env"
    echo "       Robotun IP'sini oraya yaz: FAIRINO_ROBOT_IP=192.168.x.y"
    echo "       (Container icinden ag taramasi CALISMAZ - robotun agi orada"
    echo "        yerel degil, yonlendirilmis bir ag.)"
fi

EK_BAYRAK=()
if [ "$SIM" -eq 1 ]; then
    SIM_AD="${SIMMACHINE_NAME:-fairino-simmachine}"

    if ! docker ps --filter "name=^${SIM_AD}$" --filter "status=running" --format "{{.Names}}" | grep -q .; then
        echo "SimMachine calismiyor, baslatiliyor..."
        bash "$REPO/scripts/run_simmachine.sh" || {
            echo "HATA: SimMachine baslatilamadi."
            exit 1
        }
        echo "WebApp'in acilmasi icin bekleniyor..."
        sleep 10
    fi

    # Ayni aga bagla ve robot IP'si olarak simulatoru ver. .env'deki gercek
    # robot IP'si bu -e ile EZILIYOR (docker'da sonraki -e kazanir).
    # SIMULATORE HANGI ADRESTEN BAGLANILIYOR
    # Simulatorun KENDI container IP'si (192.168.58.2) ise yaramiyor: orada
    # yalnizca 20003 (XML-RPC) aciliyor, gercek-zamanli durum portu 20004
    # dinlenmiyor. SDK durum paketini alamayinca robot_state_pkg hic
    # olusmuyor ve ilk okumada su hata cikiyor:
    #   TypeError: '_ctypes.CField' object is not subscriptable
    # Simulator o portlari HOST'a yayimliyor; host uzerinden ikisi de acik.
    #
    # --add-host: Docker Desktop host.docker.internal'i kendisi saglar, duz
    # Linux Docker saglamaz - orada da calissin diye acikca ekleniyor.
    EK_BAYRAK+=(--add-host "host.docker.internal:host-gateway"
                -e "FAIRINO_ROBOT_IP=${SIM_ROBOT_HOST:-host.docker.internal}"
                -e "FAIRINO_FORCE_HW_VERSION=${SIM_HW_VERSION:-3.9.9}")
    echo "Simulator modu: $SIM_AD (host uzerinden, yayimlanan portlar)"
    echo "  surucu surumu: ${SIM_HW_VERSION:-3.9.9} (simulatorun bildirdigi 3.9.3 degil)"
fi

mkdir -p "$DURUM_DIZINI"
docker rm -f "$AD" > /dev/null 2>&1

# Bayraklarin gerekcesi:
#   --env-file .env        robot IP'si ve yollar (makineye ozel, git'te degil)
#   --cap-add SYS_NICE     kontrol dongusunun FIFO gercek-zamanli onceligi.
#   --ulimit rtprio=99     Bu ikisi olmadan dongu gecikip PATLAMA halinde
#                          yetismeye calisiyor, ServoJ akisi duzensizlesiyor
#                          ve robot "axis N command speed exceeds the limit"
#                          faultu verebiliyor.
#   -p 8088:8088           web arayuzu
#   /tmp/.X11-unix, wslg   rviz ve gripper GUI icin X soketi
#   -v ...:/state          kayitli hareketler container silinince kaybolmasin
docker run -d --name "$AD" \
    --env-file "$REPO/.env" \
    --cap-add=SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 \
    -p "$PORT:8088" \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" \
    -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /mnt/wslg:/mnt/wslg \
    -v "$DURUM_DIZINI:/state" \
    "${EK_BAYRAK[@]+"${EK_BAYRAK[@]}"}" \
    "$IMAJ" > /dev/null || { echo "HATA: container baslatilamadi."; exit 1; }

echo "Container basladi: $AD"
echo "Robot algilanip MoveIt ayaga kalkana kadar ~1 dakika surer."
echo ""
echo "  Arayuz  : http://localhost:$PORT"
echo "  Loglar  : docker logs -f $AD"
echo "  Terminal: docker exec -it $AD bash -lc movecobot"
echo "  Durdur  : bash scripts/run_container.sh --stop"

if [ "$LOGLAR" -eq 1 ]; then
    echo ""
    docker logs -f "$AD"
fi
