#!/bin/bash
# FAIRINO SimMachine simulatorunu baslatir.
#
# Kullanim:
#   bash scripts/run_simmachine.sh
#
# NEDEN BU KADAR UZUN: Windows/WSL2 uzerinde Docker calisirken bazi portlar
# host tarafinda BAGLANAMAZ oluyor ve "docker run" komple duesuyor:
#
#   docker: Error response from daemon: ports are not available:
#   exposing port TCP 0.0.0.0:8081 -> 127.0.0.1:0
#
# Sebep bir program DEGIL: Hyper-V / WinNAT dinamik port araliklari rezerve
# ediyor ve o portlar Windows cekirdegi (System, pid 4) tarafindan tutuluyor
# gorunuyor. Hangi portlarin rezerve edildigi MAKINEDEN MAKINEYE DEGISIYOR
# ve yeniden baslatinca da degisebiliyor:
#
#   netsh interface ipv4 show excludedportrange protocol=tcp
#
# Eskiden butun portlar tek bir "docker run" satirinda sabitti; boyle bir
# rezervasyon denk gelince simulator HIC acilmiyordu. Artik yayimlanamayan
# port otomatik atlanip tekrar deneniyor - simulator eksik bir portla da olsa
# ayaga kalkiyor ve hangi portun atlandigi ekrana yaziliyor.
#
# ONEMLI: atlanan port container ICINDE calismaya devam ediyor, sadece host'a
# yayimlanmiyor. Asagidaki ZORUNLU listesindeki portlardan biri atlanirsa
# betik hata verip duruyor - onlar olmadan robot surulemez.

set -uo pipefail

IMAGE="${SIMMACHINE_IMAGE:-fairino_simmachine:v3.9.3}"
NAME="${SIMMACHINE_NAME:-fairino-simmachine}"
NET="${SIMMACHINE_NET:-fairino-net}"
SUBNET="${SIMMACHINE_SUBNET:-192.168.58.0/24}"
IP="${SIMMACHINE_IP:-192.168.58.2}"
MAX_DENEME=12

# Bunlar olmadan robot surulemez - atlanmalari halinde betik durur.
#   80    WebApp
#   502   Modbus (gripper)
#   8080  surucunun kullandigi kanal
#   20003 XML-RPC, 20004 durum paketi, 20005 komut
#   20006/20007 UDP servo akisi
ZORUNLU=(80 502 8080 20003 20004 20005 20006 20007)

PORTLAR=(
  80 502 8055 9998 9999 10000
  8060 8061 8062 8063 8065 8066 8067
  8080 8081 8082 8083
  20001 20002 20003 20004 20005 20010 39819
)
UDP_PORTLAR=(20006 20007)

atlanan=()

zorunlu_mu() {
  local p="$1"
  for z in "${ZORUNLU[@]}"; do [ "$z" = "$p" ] && return 0; done
  return 1
}

port_args() {
  local args=()
  local p
  for p in "${PORTLAR[@]}"; do
    printf '%s\n' "${atlanan[@]:-}" | grep -qx "$p" && continue
    args+=(-p "$p:$p")
  done
  for p in "${UDP_PORTLAR[@]}"; do
    printf '%s\n' "${atlanan[@]:-}" | grep -qx "$p/udp" && continue
    args+=(-p "$p:$p/udp")
  done
  printf '%s\n' "${args[@]}"
}

if ! docker info > /dev/null 2>&1; then
  echo "HATA: Docker daemon'a ulasilamiyor."
  echo "  WSL2 kullaniyorsan: Docker Desktop -> Settings -> Resources ->"
  echo "  WSL Integration -> dagitimini ac -> Apply & Restart."
  exit 1
fi

docker network create --subnet="$SUBNET" "$NET" > /dev/null 2>&1 || true

for ((deneme = 1; deneme <= MAX_DENEME; deneme++)); do
  docker rm -f "$NAME" > /dev/null 2>&1 || true

  mapfile -t PARGS < <(port_args)
  cikti=$(docker run -d --name "$NAME" \
    --privileged --cap-add=NET_ADMIN \
    --network "$NET" --ip "$IP" \
    "${PARGS[@]}" \
    "$IMAGE" 2>&1)
  rc=$?

  if [ $rc -eq 0 ]; then
    echo "SimMachine calisiyor: $NAME ($IP)"
    if [ ${#atlanan[@]} -gt 0 ]; then
      echo
      echo "UYARI: su portlar host'a yayimlanamadi, atlandi: ${atlanan[*]}"
      echo "       Windows bunlari rezerve etmis (Hyper-V/WinNAT). Container"
      echo "       ICINDE calismaya devam ediyorlar; sadece host'tan"
      echo "       erisilemiyor. Kontrol icin:"
      echo "         netsh interface ipv4 show excludedportrange protocol=tcp"
    fi
    exit 0
  fi

  # "ports are not available: exposing port TCP 0.0.0.0:8081 -> ..." icinden
  # sorunlu portu cikar.
  port=$(printf '%s' "$cikti" | grep -oE 'exposing port (TCP|UDP) [0-9.]+:[0-9]+' | grep -oE '[0-9]+$' | head -1)
  proto=$(printf '%s' "$cikti" | grep -oE 'exposing port (TCP|UDP)' | grep -oE '(TCP|UDP)$' | head -1)

  if [ -z "$port" ]; then
    echo "HATA: SimMachine baslatilamadi."
    printf '%s\n' "$cikti"
    exit 1
  fi

  if zorunlu_mu "$port"; then
    echo "HATA: $port portu host'a yayimlanamiyor ama ZORUNLU - onsuz robot surulemez."
    echo "  Windows bu portu rezerve etmis olabilir:"
    echo "    netsh interface ipv4 show excludedportrange protocol=tcp"
    echo "  Yonetici olarak WinNAT'i yeniden baslatmayi dene:"
    echo "    net stop winnat && net start winnat"
    exit 1
  fi

  if [ "$proto" = "UDP" ]; then
    atlanan+=("$port/udp")
  else
    atlanan+=("$port")
  fi
  echo "  $port portu yayimlanamadi (Windows rezerve etmis), atlanip tekrar deneniyor..."
done

echo "HATA: $MAX_DENEME denemede baslatilamadi. Atlanan portlar: ${atlanan[*]}"
exit 1
