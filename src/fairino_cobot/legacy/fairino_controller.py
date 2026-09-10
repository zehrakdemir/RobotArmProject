#!/usr/bin/env python3
"""Cagrilabilir robot kontrol modulu — vision pipeline (FoundationPose) entegrasyonuna
hazir. Bugune kadar movepy'de dogrulanan her sey (TCP offset duzeltmesi, zemin kacinma,
dogru aci matematigi) korunuyor, ama artik interaktif input() yerine dogrudan Python
fonksiyon cagrisi ile calisiyor.

Kullanim ornegi:
    from fairino_controller import FairinoController

    with FairinoController(speed_pct=30) as robot:
        result = robot.move_to_pose(x_mm=-497, y_mm=-102, z_mm=477, rx_deg=180, ry_deg=0, rz_deg=90)
        if result["success"]:
            print("Basarili:", result["final_state"])
        else:
            print("Basarisiz:", result["reason"])
"""
import math
import os
import sys
import threading

from fairino_cobot import paths

# SDK'nin yeri makineye gore degisiyor - tek yerden cozuluyor (paths.py).
paths.ensure_sdk_on_path()

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit_configs_utils import MoveItConfigsBuilder
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from fairino import Robot

JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]
GROUP_NAME = "fairino5_v6_group"
TIP_LINK = "wrist3_link"
ALL_LINKS = ["shoulder_link", "upperarm_link", "forearm_link", "wrist1_link", "wrist2_link", "wrist3_link"]
FLOOR_Z_M = 0.0
TCP_OFFSET_M = 0.1  # wrist3_link ile Fairino TCP arasindaki deneysel lokal-Z mesafesi


def _euler_deg_to_quat(rx, ry, rz):
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


def _write_moveit_cpp_yaml(path, max_vel, max_accel):
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


def _check_trajectory_for_floor(trajectory, links, floor_z_m):
    violations = []
    for i in range(len(trajectory)):
        state = trajectory[i]
        for link in links:
            pose = state.get_pose(link)
            z = pose.position.z
            if z < floor_z_m:
                violations.append((i, link, z))
    return violations


class FairinoController:
    """Bir kere baslatilir (MoveItPy kurulumu pahali), sonra istenildigi kadar
    move_to_pose() / move_to_joints() cagrisi kabul eder. Context manager destekli
    (with FairinoController() as robot: ...) — cikista otomatik shutdown() cagirir."""

    def __init__(self, speed_pct=30.0, accel_pct=30.0, node_name="fairino_controller"):
        self.speed_pct = speed_pct
        self.accel_pct = accel_pct
        max_vel = max(0.01, min(1.0, speed_pct / 100.0))
        max_accel = max(0.01, min(1.0, accel_pct / 100.0))

        yaml_path = "/tmp/fairino5_moveit_py_controller.yaml"
        _write_moveit_cpp_yaml(yaml_path, max_vel, max_accel)

        rclpy.init(args=None)
        self._node = Node(node_name)
        callback_group = ReentrantCallbackGroup()
        self._executor = MultiThreadedExecutor(2)
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._executor_thread.start()

        moveit_config = (
            MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config")
            .moveit_cpp(file_path=yaml_path)
            .to_moveit_configs()
        )
        self._fairino5 = MoveItPy(node_name=f"{node_name}_core", config_dict=moveit_config.to_dict())
        self._planning_component = self._fairino5.get_planning_component(GROUP_NAME)

        self._add_floor_collision()

        self.sdk = Robot.RPC("127.0.0.1")
        self._shutdown_called = False

    def _add_floor_collision(self):
        try:
            planning_scene_monitor = self._fairino5.get_planning_scene_monitor()
            collision_object = CollisionObject()
            collision_object.header.frame_id = "base_link"
            collision_object.id = "floor"
            box_pose = Pose()
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
            return True
        except Exception:
            return False

    def get_current_state(self):
        """Su anki eklem acilarini (derece) ve TCP pozunu (mm/derece) dondurur."""
        joints = self.sdk.robot.GetActualJointPosDegree(0)[1:7]
        pose = self.sdk.robot.GetActualTCPPose(0)[1:7]
        return {
            "joints_deg": [round(v, 3) for v in joints],
            "tcp_pose": {
                "x_mm": round(pose[0], 3), "y_mm": round(pose[1], 3), "z_mm": round(pose[2], 3),
                "rx_deg": round(pose[3], 3), "ry_deg": round(pose[4], 3), "rz_deg": round(pose[5], 3),
            },
        }

    def move_to_pose(self, x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg, confirm=False):
        """Mutlak (Base frame) Cartesian hedefe hareket eder. TCP offset otomatik telafi
        edilir, zemin carpismasi kontrol edilir. confirm=True ise calistirmadan once
        terminalde onay ister (interaktif kullanim icin); varsayilan False, programatik
        (vision pipeline) kullanim icin onay istemeden calisir.

        Doner: {"success": bool, "reason": str (basarisizsa), "final_state": dict (basarili ise)}"""
        self._planning_component.set_start_state_to_current_state()

        quat = _euler_deg_to_quat(rx_deg, ry_deg, rz_deg)
        qx, qy, qz, qw = quat
        local_z_in_global = [
            2 * (qx * qz + qy * qw),
            2 * (qy * qz - qx * qw),
            1 - 2 * (qx * qx + qy * qy),
        ]
        wrist3_x = x_mm / 1000.0 - TCP_OFFSET_M * local_z_in_global[0]
        wrist3_y = y_mm / 1000.0 - TCP_OFFSET_M * local_z_in_global[1]
        wrist3_z = z_mm / 1000.0 - TCP_OFFSET_M * local_z_in_global[2]

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose.position.x = wrist3_x
        pose_goal.pose.position.y = wrist3_y
        pose_goal.pose.position.z = wrist3_z
        pose_goal.pose.orientation.x = quat[0]
        pose_goal.pose.orientation.y = quat[1]
        pose_goal.pose.orientation.z = quat[2]
        pose_goal.pose.orientation.w = quat[3]
        self._planning_component.set_goal_state(pose_stamped_msg=pose_goal, pose_link=TIP_LINK)

        return self._plan_and_execute(confirm)

    def move_to_joints(self, joint_degrees, confirm=False):
        """Mutlak eklem hedefine hareket eder. joint_degrees: [j1,j2,j3,j4,j5,j6] derece."""
        self._planning_component.set_start_state_to_current_state()
        robot_model = self._fairino5.get_robot_model()
        target_state = RobotState(robot_model)
        target_state.set_joint_group_positions(GROUP_NAME, [math.radians(v) for v in joint_degrees])
        self._planning_component.set_goal_state(robot_state=target_state)
        return self._plan_and_execute(confirm)

    def move_relative_joints(self, joint_deltas_deg, confirm=False):
        """Her eklemi mevcut acisina gore kaydirir. joint_deltas_deg: [dj1,...,dj6] derece."""
        current = self.sdk.robot.GetActualJointPosDegree(0)[1:7]
        target = [current[i] + joint_deltas_deg[i] for i in range(6)]
        return self.move_to_joints(target, confirm=confirm)

    def _plan_and_execute(self, confirm):
        plan_result = self._planning_component.plan()
        if not plan_result:
            return {"success": False, "reason": "planning_failed"}

        violations = _check_trajectory_for_floor(plan_result.trajectory, ALL_LINKS, FLOOR_Z_M)
        if violations:
            worst = min(violations, key=lambda v: v[2])
            return {
                "success": False,
                "reason": "floor_collision",
                "details": f"waypoint {worst[0]}, link '{worst[1]}', Z={worst[2]*1000:.1f}mm",
            }

        if confirm:
            answer = input("Bu hedefe uygulansin mi? (e/h): ").strip().lower()
            if answer != "e":
                return {"success": False, "reason": "user_cancelled"}

        self._fairino5.execute(GROUP_NAME, plan_result.trajectory)
        return {"success": True, "final_state": self.get_current_state()}

    def shutdown(self):
        if self._shutdown_called:
            return
        self._shutdown_called = True
        self._executor.shutdown()
        rclpy.shutdown()
        self._executor_thread.join(timeout=2)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.shutdown()
