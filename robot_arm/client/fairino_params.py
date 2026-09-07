#!/usr/bin/env python3
"""Robotun güncel eklem açılarını ve TCP pozisyonunu temiz biçimde yazdırır."""
import sys
sys.path.insert(0, '/home/huseyin/fairino/client/fairino-python-sdk/linux')
from fairino import Robot

robot = Robot.RPC('127.0.0.1')

joints = robot.robot.GetActualJointPosDegree(0)
pose = robot.robot.GetActualTCPPose(0)

print("=== Eklem Açıları (derece) ===")
for label, val in zip(["J1", "J2", "J3", "J4", "J5", "J6"], joints[1:7]):
    print(f"  {label}: {val:8.3f}")

print("\n=== TCP Pozisyonu ===")
for (label, unit), val in zip(
    [("X", "mm"), ("Y", "mm"), ("Z", "mm"), ("RX", "°"), ("RY", "°"), ("RZ", "°")],
    pose[1:7],
):
    print(f"  {label}: {val:8.3f} {unit}")
