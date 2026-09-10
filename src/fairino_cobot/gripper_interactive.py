#!/usr/bin/env python3
"""Interaktif Jodell EPG40-050 kavrayici kontrolu. Position/speed/force
sorar, gripper_controller.py'nin GripperController sinifi ile hareket
ettirir.

Kullanim: movegripper (alias) veya: python3 gripper_interactive.py
"""
import sys

from fairino_cobot.gripper_controller import GripperController, GripperError


def ask_int(prompt, default, lo=0, hi=100):
    while True:
        raw = input(f"{prompt} [{default}]: ").strip()
        if raw == "":
            return default
        try:
            val = int(raw)
        except ValueError:
            print(f"  Gecersiz sayi, tekrar gir.")
            continue
        if not (lo <= val <= hi):
            print(f"  {lo}-{hi} araliginda olmali, tekrar gir.")
            continue
        return val


def main():
    print("Robot araniyor ve kavrayici hazirlaniyor...")
    try:
        gripper = GripperController()
    except Exception as e:
        print(f"HATA: baglanti kurulamadi: {e}")
        sys.exit(1)

    if not gripper.is_active():
        print("Aktive ediliyor...")
        try:
            gripper.activate()
        except GripperError as e:
            print(f"HATA: aktivasyon basarisiz: {e}")
            sys.exit(1)

    print("Hazir. Cikmak icin 'q' yaz.\n")

    try:
        while True:
            raw_pos = input("Pozisyon (0=tam acik, 100=tam kapali) veya q: ").strip()
            if raw_pos.lower() in ("q", "quit", "exit"):
                break
            try:
                position = int(raw_pos)
            except ValueError:
                print("  Gecersiz deger, sayi gir.\n")
                continue
            if not (0 <= position <= 100):
                print("  0-100 araliginda olmali.\n")
                continue

            speed = ask_int("Hiz (0-100)", 50)
            force = ask_int("Kuvvet (0-100)", 50)

            print(f"  -> pozisyon={position} hiz={speed} kuvvet={force} gonderiliyor...")
            try:
                gripper.move(position=position, speed=speed, force=force)
                status = gripper.status()
                print(f"  Tamamlandi. Durum: {status}\n")
            except GripperError as e:
                print(f"  HATA: {e}\n")
    except (KeyboardInterrupt, EOFError):
        print("\nCikiliyor.")


if __name__ == "__main__":
    main()
