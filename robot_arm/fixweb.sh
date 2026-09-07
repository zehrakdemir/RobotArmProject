#!/bin/bash
if ! docker ps > /dev/null 2>&1; then
  echo "Docker daemon calismiyor, baslatiliyor..."
  sudo -v || { echo "HATA: sudo yetkilendirme basarisiz."; exit 1; }
  sudo dockerd > /tmp/dockerd.log 2>&1 &

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
  bash ~/fairino/run_simmachine.sh
fi

sleep 10
echo "WebApp kontrol ediliyor..."
curl -sI http://127.0.0.1