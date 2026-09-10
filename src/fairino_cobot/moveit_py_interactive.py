#!/usr/bin/env python3
"""Interaktif moveit_py kontrol scripti — mevcut durumu gosterir, hareket tipi
(Joint/Cartesian/Basit TCP-yerel, Joint ve Cartesian mutlak/goreceli) sorar, zemin collision nesnesi ekler,
TCP offset'i telafi eder, onay alir, calistirir, SDK ile dogrular.

GUNCELLEME: Artik hangi robota (FR3, FR5, FR10...) bagli oldugunu ~/detect_robot.py
ile OTOMATIK algiliyor - IP, grup adi, robot_description adi, config paketi hepsi
buna gore kendiliginden ayarlaniyor. Elle FR5/127.0.0.1 degistirmene gerek yok.
"""
import math
import os
import re
import subprocess
import sys
import threading

from fairino_cobot import paths

# SDK'nin yeri makineye gore degisiyor - tek yerden cozuluyor (paths.py).
paths.ensure_sdk_on_path()

import rclpy
from fairino import Robot
from geometry_msgs.msg import Pose, PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_msgs.msg import CollisionObject
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]
TIP_LINK = "wrist3_link"
ALL_LINKS = ["shoulder_link", "upperarm_link", "forearm_link", "wrist1_link", "wrist2_link", "wrist3_link"]
FLOOR_Z_M = 0.0


def detect_robot():
    script_path = str(paths.detect_robot_script())
    if not os.path.exists(script_path):
        print(f"HATA: {script_path} bulunamadi.")
        sys.exit(1)

    print("Robot algilaniyor (ag taramasi)...")
    try:
        result = subprocess.run(
            [sys.executable, script_path],
            stdout=subprocess.PIPE, stderr=None, text=True,
            timeout=120,
        )
    except subprocess.TimeoutExpired:
        print("HATA: detect_robot.py 120 saniyede bitmedi (zaman asimi).")
        sys.exit(1)

    if result.returncode != 0 or not result.stdout.strip():
        print(f"HATA: Robot algilanamadi. Tani icin: returncode={result.returncode}, "
              f"stdout={result.stdout!r}")
        sys.exit(1)

    info = {}
    for line in result.stdout.splitlines():
        m = re.match(r"^([A-Z_][A-Z0-9_]*)=(.*)$", line)
        if m:
            info[m.group(1)] = m.group(2)

    required = ["ROBOT_IP", "ROBOT_MODEL_NUM"]
    missing = [k for k in required if k not in info]
    if missing:
        print(f"HATA: detect_robot.py ciktisinda eksik alan(lar): {missing}")
        print(f"Ham stdout: {result.stdout!r}")
        sys.exit(1)

    return info


def euler_deg_to_quat(rx, ry, rz):
    r, p, y = math.radians(rx), math.radians(ry), math.radians(rz)
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def compute_tcp_offset(robot_model, group_name, current_joints_deg, current_tcp_pose):
    state_now = RobotState(robot_model)
    state_now.set_joint_group_positions(group_name, [math.radians(v) for v in current_joints_deg])
    state_now.update()
    wrist3_pose = state_now.get_pose(TIP_LINK)

    wx = wrist3_pose.position.x
    wy = wrist3_pose.position.y
    wz = wrist3_pose.position.z

    qx = wrist3_pose.orientation.x
    qy = wrist3_pose.orientation.y
    qz = wrist3_pose.orientation.z
    qw = wrist3_pose.orientation.w
    local_z_in_global = [
        2 * (qx * qz + qy * qw),
        2 * (qy * qz - qx * qw),
        1 - 2 * (qx * qx + qy * qy),
    ]

    tx_m = current_tcp_pose[0] / 1000.0
    ty_m = current_tcp_pose[1] / 1000.0
    tz_m = current_tcp_pose[2] / 1000.0

    diff = [tx_m - wx, ty_m - wy, tz_m - wz]
    offset = diff[0] * local_z_in_global[0] + diff[1] * local_z_in_global[1] + diff[2] * local_z_in_global[2]

    lateral = [
        diff[0] - offset * local_z_in_global[0],
        diff[1] - offset * local_z_in_global[1],
        diff[2] - offset * local_z_in_global[2],
    ]
    lateral_mag_mm = math.sqrt(sum(c * c for c in lateral)) * 1000.0
    if lateral_mag_mm > 2.0:
        print(f"UYARI: wrist3->TCP farkinin {lateral_mag_mm:.1f}mm'lik bir kismi "
              f"lokal-Z ekseninde degil (yanal). Bu script offset'i saf eksenel "
              f"(Z) mesafe varsayiyor - eger TCP, bilekten yana kaymis bir uc "
              f"aletiyse (offset tool), hesap tam dogru olmayabilir.")

    return offset


def ask_float(prompt, default=None):
    while True:
        raw = input(f"{prompt}" + (f" [{default}]" if default is not None else "") + ": ").strip()
        if raw == "" and default is not None:
            return default
        try:
            return float(raw)
        except ValueError:
            print("  Gecersiz sayi, tekrar dene.")


def ask_choice(prompt, options):
    print(prompt)
    for i, opt in enumerate(options, 1):
        print(f"  {i}) {opt}")
    while True:
        raw = input("Secim: ").strip()
        if raw.isdigit() and 1 <= int(raw) <= len(options):
            return int(raw)
        print("  Gecersiz secim, tekrar dene.")


def write_moveit_cpp_yaml(path, max_vel, max_accel):
    content = f"""planning_scene_monitor_options:
  name: "planning_scene_monitor"
  robot_description: "robot_description"
  joint_state_topic: "/joint_states"
  attached_collision_object_topic: "/moveit_cpp/planning_scene_monitor"
  publish_planning_scene_topic: "/moveit_cpp/publish_planning_scene"
  monitored_planning_scene_topic: "/moveit_cpp/monitored_planning_scene"
  wait_for_initial_state_timeout: 10.0

planning_pipelines:
  pipeline_names: ["ompl"]

plan_request_params:
  planning_attempts: 1
  planning_pipeline: ompl
  max_velocity_scaling_factor: {max_vel}
  max_acceleration_scaling_factor: {max_accel}
"""
    with open(path, "w") as f:
        f.write(content)


def add_floor_collision(moveit_instance):
    try:
        planning_scene_monitor = moveit_instance.get_planning_scene_monitor()

        collision_object = CollisionObject()
        collision_object.header.frame_id = "base_link"
        collision_object.id = "floor"

        box_pose = Pose()
        box_pose.position.x = 0.0
        box_pose.position.y = 0.0
        box_pose.position.z = -0.05

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [2.0, 2.0, 0.1]

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD

        with planning_scene_monitor.read_write() as scene:
            scene.apply_collision_object(collision_object)
            scene.current_state.update()

        print("Zemin collision nesnesi eklendi.")
        return True
    except Exception as e:
        print(f"UYARI: Zemin collision nesnesi eklenemedi ({type(e).__name__}: {e})")
        return False


def check_trajectory_for_floor(trajectory, links, floor_z_m):
    violations = []
    for i in range(len(trajectory)):
        state = trajectory[i]
        for link in links:
            pose = state.get_pose(link)
            z = pose.position.z
            if z < floor_z_m:
                violations.append((i, link, z))
    return violations


def main():
    robot_info = detect_robot()
    robot_ip = robot_info["ROBOT_IP"]
    model_num = robot_info["ROBOT_MODEL_NUM"]

    group_name = f"fairino{model_num}_v6_group"
    robot_desc_name = f"fairino{model_num}_v6_robot"
    package_name = f"fairino{model_num}_v6_moveit2_config"

    print(f"\n>>> Algilanan: FR{model_num} ({robot_ip})")
    print(f">>> Grup: {group_name}  |  Paket: {package_name}\n")

    sdk_robot = Robot.RPC(robot_ip)
    current_joints = sdk_robot.robot.GetActualJointPosDegree(0)[1:7]
    current_pose = sdk_robot.robot.GetActualTCPPose(0)[1:7]

    print("=== Mevcut Durum ===")
    print("Eklem acilari (derece):")
    for name, val in zip(JOINT_NAMES, current_joints):
        print(f"  {name.upper()}: {val:8.3f}")
    print("TCP pozu (mm / derece):")
    for label, val in zip(["X", "Y", "Z", "RX", "RY", "RZ"], current_pose):
        print(f"  {label}: {val:8.3f}")

    move_type = ask_choice(
        "\nHareket tipi sec:",
        [
            "Eklem acilari (Joint)",
            "Cartesian pozisyon (X,Y,Z + RX,RY,RZ)",
            "Basit hareket (sadece X,Y,Z, TCP'ye gore goreceli)",
        ],
    )

    joint_mode = None
    joint_targets_deg = None
    joint_deltas_deg = None
    pose_target = None
    frame_choice = None
    simple_deltas = None

    if move_type == 1:
        joint_mode = ask_choice(
            "\nEklem modu sec:",
            ["Mutlak (kesin aci gir)", "Goreceli (mevcut acinin uzerine ekle/cikar)"],
        )
        if joint_mode == 1:
            print("\nEklem acilarini derece cinsinden gir (mutlak hedef):")
            joint_targets_deg = [ask_float(f"  {j.upper()}") for j in JOINT_NAMES]
        else:
            print("\nHer eklem icin degisim miktarini gir (derece, 0 = degistirme):")
            joint_deltas_deg = [ask_float(f"  {j.upper()} degisim", default=0.0) for j in JOINT_NAMES]
    elif move_type == 2:
        frame_choice = ask_choice(
            "\nReferans koordinat sistemi:",
            ["Base (mutlak hedef)", "Tool (X,Y,Z goreceli; RX,RY,RZ HER ZAMAN mutlak)", "Wobj (henuz tanimli degil)"],
        )
        if frame_choice == 3:
            print("\nWobj bu projede henuz tanimlanmadi. Cikiliyor.")
            return
        print("\nHedef degerleri gir:")
        pose_target = {
            "x": ask_float("  X (mm)"),
            "y": ask_float("  Y (mm)"),
            "z": ask_float("  Z (mm)"),
            "rx": ask_float("  RX (derece)", default=0.0),
            "ry": ask_float("  RY (derece)", default=0.0),
            "rz": ask_float("  RZ (derece)", default=0.0),
        }
    else:
        print("\nTCP'nin kendi eksenlerine gore (yerel), kac mm hareket edecegini gir (0 = degistirme):")
        simple_deltas = {
            "x": ask_float("  X (mm)", default=0.0),
            "y": ask_float("  Y (mm)", default=0.0),
            "z": ask_float("  Z (mm)", default=0.0),
        }

    speed_pct = ask_float("\nHiz yuzdesi (0-100)", default=30.0)
    accel_pct = ask_float("Ivme yuzdesi (0-100)", default=30.0)
    max_vel = max(0.01, min(1.0, speed_pct / 100.0))
    max_accel = max(0.01, min(1.0, accel_pct / 100.0))

    yaml_path = f"/tmp/fairino{model_num}_moveit_py_interactive.yaml"
    write_moveit_cpp_yaml(yaml_path, max_vel, max_accel)

    rclpy.init()
    node = Node("moveit_py_interactive")
    # (Eskiden burada bir ReentrantCallbackGroup olusturuluyordu ama hicbir
    # yere verilmiyordu - olu koddu, kaldirildi. Executor zaten cok is
    # parcacikli.)
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    moveit_config = (
        MoveItConfigsBuilder(robot_desc_name, package_name=package_name)
        .moveit_cpp(file_path=yaml_path)
        .to_moveit_configs()
    )
    moveit_instance = MoveItPy(node_name="moveit_py_interactive_core", config_dict=moveit_config.to_dict())
    planning_component = moveit_instance.get_planning_component(group_name)
    planning_component.set_start_state_to_current_state()

    add_floor_collision(moveit_instance)

    robot_model = moveit_instance.get_robot_model()
    tcp_offset_m = compute_tcp_offset(robot_model, group_name, current_joints, current_pose)
    print(f">>> Olculen wrist3->TCP offset: {tcp_offset_m*1000:.1f}mm (bu FR{model_num} icin gercek zamanli hesaplandi)\n")

    def shutdown():
        executor.shutdown()
        rclpy.shutdown()
        executor_thread.join(timeout=2)

    if move_type == 1:
        if joint_mode == 2:
            fresh_current_joints = sdk_robot.robot.GetActualJointPosDegree(0)[1:7]
            joint_targets_deg = [fresh_current_joints[i] + joint_deltas_deg[i] for i in range(6)]
            print(f"\n[Goreceli -> Mutlak hedef: {[round(v, 2) for v in joint_targets_deg]}]")
        target_state = RobotState(robot_model)
        target_state.set_joint_group_positions(group_name, [math.radians(v) for v in joint_targets_deg])
        planning_component.set_goal_state(robot_state=target_state)
        print(f"Hedef eklem acilari (derece): {[round(v, 2) for v in joint_targets_deg]}")
    else:
        if move_type == 2:
            if frame_choice == 1:
                tx, ty, tz = pose_target["x"], pose_target["y"], pose_target["z"]
                trx, trye, trz = pose_target["rx"], pose_target["ry"], pose_target["rz"]
            else:
                cur = sdk_robot.robot.GetActualTCPPose(0)
                cx, cy, cz, crx, cry, crz = cur[1:7]
                tx, ty, tz = cx + pose_target["x"], cy + pose_target["y"], cz + pose_target["z"]
                trx, trye, trz = pose_target["rx"], pose_target["ry"], pose_target["rz"]
        else:
            cur = sdk_robot.robot.GetActualTCPPose(0)
            cx, cy, cz, crx, cry, crz = cur[1:7]
            cqx, cqy, cqz, cqw = euler_deg_to_quat(crx, cry, crz)
            cur_x_axis = [1 - 2 * (cqy * cqy + cqz * cqz), 2 * (cqx * cqy + cqz * cqw), 2 * (cqx * cqz - cqy * cqw)]
            cur_y_axis = [2 * (cqx * cqy - cqz * cqw), 1 - 2 * (cqx * cqx + cqz * cqz), 2 * (cqy * cqz + cqx * cqw)]
            cur_z_axis = [2 * (cqx * cqz + cqy * cqw), 2 * (cqy * cqz - cqx * cqw), 1 - 2 * (cqx * cqx + cqy * cqy)]
            dx, dy, dz = simple_deltas["x"], simple_deltas["y"], simple_deltas["z"]
            tx = cx + dx * cur_x_axis[0] + dy * cur_y_axis[0] + dz * cur_z_axis[0]
            ty = cy + dx * cur_x_axis[1] + dy * cur_y_axis[1] + dz * cur_z_axis[1]
            tz = cz + dx * cur_x_axis[2] + dy * cur_y_axis[2] + dz * cur_z_axis[2]
            trx, trye, trz = crx, cry, crz
            print(f"\n[TCP-yerel hareket: X={dx:+.1f} Y={dy:+.1f} Z={dz:+.1f} mm]")

        quat = euler_deg_to_quat(trx, trye, trz)
        qx, qy, qz, qw = quat
        local_z_in_global = [
            2 * (qx * qz + qy * qw),
            2 * (qy * qz - qx * qw),
            1 - 2 * (qx * qx + qy * qy),
        ]
        wrist3_x = tx / 1000.0 - tcp_offset_m * local_z_in_global[0]
        wrist3_y = ty / 1000.0 - tcp_offset_m * local_z_in_global[1]
        wrist3_z = tz / 1000.0 - tcp_offset_m * local_z_in_global[2]
        print(f"\n[TCP hedefi: X={tx:.1f} Y={ty:.1f} Z={tz:.1f} -> wrist3_link hedefi: "
              f"X={wrist3_x*1000:.1f} Y={wrist3_y*1000:.1f} Z={wrist3_z*1000:.1f}]")

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose.position.x = wrist3_x
        pose_goal.pose.position.y = wrist3_y
        pose_goal.pose.position.z = wrist3_z
        pose_goal.pose.orientation.x = quat[0]
        pose_goal.pose.orientation.y = quat[1]
        pose_goal.pose.orientation.z = quat[2]
        pose_goal.pose.orientation.w = quat[3]
        planning_component.set_goal_state(pose_stamped_msg=pose_goal, pose_link=TIP_LINK)
        print(f"Hedef poz (base_link, mm/derece): X={tx:.1f} Y={ty:.1f} Z={tz:.1f} RX={trx:.1f} RY={trye:.1f} RZ={trz:.1f}")

    print(f"Hiz: %{speed_pct:.0f}  Ivme: %{accel_pct:.0f}")
    print("\nPlanlaniyor (zemin engelli, alternatif yol aranacak)...")
    plan_result = planning_component.plan()

    if not plan_result:
        print("HATA: Planlama basarisiz oldu.")
        shutdown()
        return

    print("Planlama basarili. Ek guvenlik kontrolu yapiliyor...")
    violations = check_trajectory_for_floor(plan_result.trajectory, ALL_LINKS, FLOOR_Z_M)

    if violations:
        worst = min(violations, key=lambda v: v[2])
        worst_i, worst_link, worst_z = worst
        print(f"\n{'='*60}")
        print("BU KOMUT UYGULANAMAZ.")
        print(f"Sebep: Collision nesnesine ragmen planlanan yolun {len(violations)} noktasinda")
        print(f"robotun bir parcasi zeminin altina iniyor (en kotu: adim {worst_i}, link '{worst_link}', Z={worst_z*1000:.1f}mm).")
        print(f"{'='*60}")
        shutdown()
        return

    print("Guvenlik kontrolu gecti — yol hicbir noktada zeminin altina inmiyor.")
    answer = input("\nBu hedefe uygulansin mi? (e/h): ").strip().lower()
    if answer != "e":
        print("Iptal edildi.")
        shutdown()
        return

    print("Calistiriliyor...")
    moveit_instance.execute(group_name, plan_result.trajectory)
    print("Tamamlandi!")

    joints = sdk_robot.robot.GetActualJointPosDegree(0)
    pose = sdk_robot.robot.GetActualTCPPose(0)
    print("\n=== Dogrulama (SDK) ===")
    print(f"Eklem acilari: {[round(v, 3) for v in joints[1:7]]}")
    print(f"TCP pozu: {[round(v, 3) for v in pose[1:7]]}")

    shutdown()


if __name__ == "__main__":
    main()
