#!/usr/bin/env python3
"""
OTOMATIK COK-NESNELI HUCRE dongusu (CLI).

Kuru (robot yok, sadece algila+plan):
    docker compose run --rm valiz-olcum python -m tools.auto_run --dry-run

Gercek (FR3 ethernet takili, ROBOT_ENABLED=true):
    docker compose run --rm valiz-olcum python -m tools.auto_run

Onkosul: host'ta cam_web calisiyor olmali:
    <py313> tools/cam_web.py --left 2 --right 3 --port 8600 --snapdir <proje>/snaps
"""
from __future__ import annotations

import argparse
import logging


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true", help="robota gonderme, sadece plan")
    ap.add_argument("--max-cycles", type=int, default=50)
    ap.add_argument("--once", action="store_true", help="tek tur calis, cik")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(message)s")
    from src.auto import AutoCell

    cell = AutoCell(dry_run=args.dry_run)
    if args.once:
        r = cell.step()
        print("SONUC:", r)
        return 0
    cell.run(max_cycles=args.max_cycles)
    print(f"\nToplam yerlestirilen: {cell.state.placed}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
