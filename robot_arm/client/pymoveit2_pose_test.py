#!/usr/bin/env python3
"""pymoveit2 ile Cartesian pose (X,Y,Z + quaternion) hareket testi.
FoundationPose'un üreteceği veri tipine yakın bir test — eklem açısı değil,
uzaydaki bir hedef pozisyon veriyoruz, IK'yı MoveIt2 hesaplıyor."""
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2


def main():
    rclpy.init()
    node = Node("pymoveit2_fairino5_pose_test")
    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=["j1", "j2", "j3", "j4", "j5", "j6"],
        base_link_name="base_link",
        end_effector_name="wrist3_link",
        group_name="fairino5_v6_group",
        callback_group=callback_group,
    )
    moveit2.max_velocity = 0.1
    moveit2.max_acceleration = 0.3

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Test hedefi: base_link'e göre X=0.3m ileri, Y=0.0 (merkez), Z=0.3m yukarı.
    # Yönelim: kimlik quaternion (hiç döndürme yok) — sadece bağlantı testi.
    target_position = [0.3, 0.0, 0.3]
    target_orientation = [0.0, 0.0, 0.0, 1.0]  # [x, y, z, w]

    node.get_logger().info(f"Hedef poza hareket ediliyor: {target_position}")
    moveit2.move_to_pose(
        position=target_position,
        quat_xyzw=target_orientation,
        cartesian=False,
    )
    moveit2.wait_until_executed()
    node.get_logger().info("Tamamlandı!")

    executor.shutdown()
    rclpy.shutdown()
    executor_thread.join(timeout=2)


if __name__ == "__main__":
    main()
