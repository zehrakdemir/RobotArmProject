#!/bin/bash
# SimMachine simulatorunun WebApp'ini tekrar ayaga kaldirir.
# (Gercek robotla calisirken gerekmez - o zaman WebApp robotun kendi
# IP'sinde, port 80'de.)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! docker ps > /dev/null 2>&1; then
  echo "Docker daemon calismiyor, baslatiliyor..."
  sudo -v || { echo "HATA: sudo yetkilendirme basarisiz."; exit 1; }
  # Yonlendirme sudo'dan ETKILENMEZ - dosya cagiran kullanicinin haklariyla
  # acilir. Log'u root'un acmasi icin komutun tamami sudo'nun icinde.
  sudo sh -c 'dockerd > /tmp/dockerd.log 2>&1' &

  echo "Docker daemon hazir olmasi bekleniyor..."
  for i in $(seq 1 30); do
    if docker info > /dev/null 2>&1; then
      echo "Docker daemon hazir."
      break
    fi
    sleep 1
    if [ "$i" -eq 30 ]; then
      echo "HATA: Docker daemon 30 saniyede hazir olmadi, /tmp/dockerd.log kontrol edin."
      exit 1
    fi
  done
fi

if ! docker images | grep -q fairino_simmachine; then
  echo "Image bulunamadi, yeniden yukleniyor (2GB, biraz surer)..."
  docker load -i ~/fairino/simmachine/FAIRINOSimMachine.tar
fi

if docker ps -a --format '{{.Names}}' | grep -q '^fairino-simmachine$'; then
  echo "Container mevcut, yeniden baslatiliyor..."
  docker restart fairino-simmachine
else
  echo "Container yok, sifirdan olusturuluyor..."
  bash "$SCRIPT_DIR/run_simmachine.sh"
fi

sleep 10
echo "WebApp kontrol ediliyor..."
# Port YOK - WebApp 80'de. (Bir ara 8080 sanilmisti, oyle degil.)
curl -sI http://127.0.0.1
