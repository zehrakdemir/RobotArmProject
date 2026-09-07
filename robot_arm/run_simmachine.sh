#!/bin/bash
docker network create --subnet=192.168.58.0/24 fairino-net 2>/dev/null || true
docker rm -f fairino-simmachine 2>/dev/null || true
docker run -d --name fairino-simmachine \
  --privileged --cap-add=NET_ADMIN \
  --network fairino-net --ip 192.168.58.2 \
  -p 80:80 -p 9998:9998 -p 9999:9999 \
  -p 20003:20003 -p 20004:20004 \
  -p 502:502 -p 8055:8055 \
  -p 8060-8063:8060-8063 -p 8065-8067:8065-8067 \
  -p 8080-8083:8080-8083 -p 10000:10000 \
  -p 20001-20002:20001-20002 -p 20010:20010 \
  -p 39819:39819 -p 20006-20007:20006-20007/udp \
  fairino_simmachine:v3.9.3
