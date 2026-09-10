#!/usr/bin/env python3
"""Robotu tüm eklemler 0 derece olan 'home' pozisyonuna götürür.
Bu, test scriptlerimizde kullandığımız Cartesian hedeflerden bağımsız,
sabit ve güvenli bir referans pozisyon."""
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2


def main():
    rclpy.init()
    node = Node("go_home")
    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=["j1", "j2", "j3", "j4", "j5", "j6"],
        base_link_name="base_link",
        end_effector_name="wrist3_link",
        group_name="fairino5_v6_group",
        callback_group=callback_group,
    )
    moveit2.max_velocity = 0.3
    moveit2.max_acceleration = 0.2

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    node.get_logger().info("Home pozisyonuna (tüm eklemler 0°) dönülüyor...")
    moveit2.move_to_configuration(home_joints)
    moveit2.wait_until_executed()
    node.get_logger().info("Home pozisyonuna ulaşıldı!")

    executor.shutdown()
    rclpy.shutdown()
    executor_thread.join(timeout=2)


if __name__ == "__main__":
    main()
