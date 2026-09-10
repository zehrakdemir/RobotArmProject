#!/usr/bin/env python3
"""konumcobot - Robotun (ve varsa gripper'in) su anki konumunu hizlica
gosterir ve cikar.

movecobot'un aksine ROS2/MoveIt/fairino_launch GEREKTIRMEZ - dogrudan SDK
uzerinden okur (Robot.RPC, movecobot.py'nin CobotArm.get_current_state() ile
AYNI cagri sekli), bu yuzden ROS2 ayakta olmasa bile aninda calisir. Sadece
konum kontrol etmek icin movecobot'un tum MoveIt/rclpy baslatma suresini
beklemek istemedigin durumlar icin.

Kullanim: konumcobot
"""
import json
import os
import subprocess
import sys
import time
from pathlib import Path

from fairino_cobot import paths

# SDK'nin yeri makineye gore degisiyor - tek yerden cozuluyor (paths.py).
paths.ensure_sdk_on_path()

from fairino import Robot
from fairino_cobot.gripper_controller import (
    GripperController,
    GripperError,
    detect_robot_ip,
)

JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]

# movecobot.py ile AYNI dosya (bkz. movecobot.py STATE_CACHE_FILE) - movecobot
# calisirken en son okudugu durumu buraya yazar, konumcobot da (movecobot
# calisiyorsa) robota IKINCI bir baglanti acmak yerine bunu okur.
STATE_CACHE_FILE = paths.state_cache_file()

# Fairino robotu ayni anda pratikte TEK bir SDK baglantisini (Robot.RPC)
# destekliyor - port 20004 realtime durum soketi tek-tuketicili. Bu
# scriptlerden biri zaten calisirken konumcobot AYRI bir surec olarak ikinci
# bir baglanti acmaya calisirsa robot cevap vermeyebilir (RPC "cevap vermedi"
# ile zaman asimina ugrar) - kod hatasi degil, beklenen bir kaynak catismasi.
# movecobot.py'nin ozel bir cache mekanizmasi var (asagida); digerleri icin
# sadece uyari verilip yine de dogrudan baglanti denenir.
CONFLICTING_SCRIPTS = ["movecobot.py", "moveit_py_interactive.py", "gripper_interactive.py", "gripper_gui.py"]


def find_conflicting_process():
    try:
        result = subprocess.run(["pgrep", "-af", "python3"], capture_output=True, text=True, timeout=5)
    except Exception:
        return None
    for line in result.stdout.splitlines():
        for script in CONFLICTING_SCRIPTS:
            if script in line:
                return line.strip()
    return None


def read_state_cache():
    try:
        return json.loads(STATE_CACHE_FILE.read_text(encoding="utf-8"))
    except Exception:
        return None


def print_arm_state(joints, tcp_pose, is_dict_pose):
    print("\n=== Kol durumu ===")
    for name, val in zip(JOINT_NAMES, joints):
        print(f"  {name.upper()}: {val:8.3f}")
    p = tcp_pose
    if is_dict_pose:
        print(f"  TCP: X={p['x']:.3f} Y={p['y']:.3f} Z={p['z']:.3f} RX={p['rx']:.3f} RY={p['ry']:.3f} RZ={p['rz']:.3f}")
    else:
        print(f"  TCP: X={p[0]:.3f} Y={p[1]:.3f} Z={p[2]:.3f} RX={p[3]:.3f} RY={p[4]:.3f} RZ={p[5]:.3f}")


def print_from_cache(cache):
    age = time.time() - cache.get("timestamp", 0)
    print(f"movecobot su an calisiyor - robota ikinci bir baglanti acmak yerine onun son "
          f"bildirdigi durumu gosteriyorum ({age:.0f} saniye once, {cache.get('robot_ip', '?')}).")
    if age > 15:
        print("UYARI: bu bilgi biraz eski olabilir - movecobot menusunde bir durum kontrolu "
              "([3] Su anki durumu goster, ya da [d]) yapmadan bu deger tazelenmiyor.")
    print_arm_state(cache["joints_deg"], cache["tcp_pose"], is_dict_pose=True)
    gstat = cache.get("gripper")
    if gstat:
        print(f"=== Gripper durumu === pozisyon={gstat['position_pct']} hiz={gstat['speed_pct']} "
              f"motion_status={gstat['motion_status']}")
    print()


def print_live(ip, conflict):
    sdk = Robot.RPC(ip)

    try:
        # movecobot.py CobotArm.get_current_state() ile AYNI cagri sekli -
        # bu SDK'da dogrudan sdk.GetActualJointPosDegree(...) yerine
        # sdk.robot.GetActualJointPosDegree(...) (ic xmlrpc proxy uzerinden)
        # kullanmak canli robotta dogrulanmis olan yol.
        joints = sdk.robot.GetActualJointPosDegree(0)[1:7]
        pose = sdk.robot.GetActualTCPPose(0)[1:7]
    except Exception as e:
        print(f"HATA: konum okunamadi: {e}")
        if conflict:
            print(f"(Muhtemel sebep: '{conflict.split()[-1]}' sureci robotun baglantisini "
                  f"elinde tutuyor olabilir.)")
        sys.exit(1)

    print_arm_state(joints, pose, is_dict_pose=False)

    try:
        gripper = GripperController(sdk=sdk)
        active = gripper.is_active()
        gstat = gripper.status()
        print("\n=== Gripper durumu ===")
        print(f"  Aktif: {'evet' if active else 'hayir'}  Pozisyon: {gstat['position_pct']}  "
              f"Hiz: {gstat['speed_pct']}  Hareket durumu: {gstat['motion_status']}  "
              f"Hata kodu: {gstat['fault']}")
    except GripperError as e:
        print(f"\nUYARI: gripper durumu okunamadi: {e}")
    except Exception as e:
        print(f"\nUYARI: gripper baglantisi kurulamadi: {e}")

    try:
        sdk.CloseRPC()
    except Exception:
        pass
    print()


def main():
    conflict = find_conflicting_process()

    if conflict and "movecobot.py" in conflict:
        cache = read_state_cache()
        if cache:
            print_from_cache(cache)
            return
        print("movecobot su an calisiyor ama henuz bir durum onbellegi yok. movecobot "
              "menusunde bir kere '[3] Su anki durumu goster' calistir, sonra konumcobot'u "
              "tekrar dene - ya da dogrudan movecobot'un o menusunu kullan.")
        return

    if conflict:
        print("UYARI: Robota zaten baska bir surec baglanmis gorunuyor:")
        print(f"  {conflict}")
        print("Fairino robotu ayni anda pratikte TEK bir SDK baglantisini destekliyor - "
              "bu surec acikken konumcobot baglanamayabilir/zaman asimina ugrayabilir. "
              "Yine de deneniyor...\n")

    print("Robot araniyor (detect_robot.py)...")
    try:
        ip = detect_robot_ip()
    except RuntimeError as e:
        print(f"HATA: {e}")
        if conflict:
            print(f"(Muhtemel sebep: '{conflict.split()[-1]}' sureci robotun baglantisini "
                  f"elinde tutuyor olabilir.)")
        sys.exit(1)

    print_live(ip, conflict)


if __name__ == "__main__":
    main()
