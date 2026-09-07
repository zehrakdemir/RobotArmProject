#!/usr/bin/env python3
"""pymoveit2 ile FR5 (fairino5_v6_group) hareket testi."""
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2


def main():
    rclpy.init()
    node = Node("pymoveit2_fairino5_test")
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
    moveit2.max_acceleration = 0.3

    # move_to_configuration çağrısı arka planda action/service beklediği için
    # node'un ayrı bir thread'de sürekli "spin" edilmesi gerekiyor.
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Başlangıç pozisyonuna yakın, j1'i ~10 derece kaydıran bir hedef (radyan cinsinden)
    joint_goal = [0.1745, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]

    node.get_logger().info("Hedefe hareket ediliyor...")
    moveit2.move_to_configuration(joint_goal)
    moveit2.wait_until_executed()
    node.get_logger().info("Tamamlandı!")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
