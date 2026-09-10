#!/usr/bin/env python3
"""
FR3 SimMachine baglanti + kucuk guvenli hareket testi.

    docker compose run --rm valiz-olcum python -m tools.fr3_test
    docker compose run --rm valiz-olcum python -m tools.fr3_test --move   # J1'i +10deg oynat, geri getir

Ag: valiz konteyneri `fairino-net` agina bagli olmali (docker-compose.yml).
"""
from __future__ import annotations

import argparse
import time

from src import config
from src.robot import FairinoFR3


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default=config.ROBOT_IP)
    ap.add_argument("--move", action="store_true", help="J1'i +10deg oynatip geri getir")
    args = ap.parse_args()

    fr3 = FairinoFR3(ip=args.ip, enabled=True)
    print(f"-> baglaniliyor: {args.ip}:20003 / :20004")
    if not fr3.connect():
        print("BAGLANAMADI. SimMachine ayakta mi? `docker ps | grep fairino`. "
              "Ag: `docker network inspect fairino-net` icinde bu konteyner var mi?")
        return 1

    st = fr3.status()
    print("DURUM:", st)

    if args.move:
        prep = fr3.prepare()
        print("HAZIRLIK:", prep)
        if not prep.get("prepared"):
            return 1
        start = fr3._joints_now()
        print("baslangic eklemler:", [round(v, 2) for v in start])
        target = list(start); target[0] += 10.0
        print("hedef eklemler   :", [round(v, 2) for v in target])
        fr3._movej(target, 20); time.sleep(0.5)
        fr3._movej(start, 20); time.sleep(0.5)
        print("son eklemler     :", [round(v, 2) for v in fr3._joints_now()])
        print("OK — hareket testi tamam.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
