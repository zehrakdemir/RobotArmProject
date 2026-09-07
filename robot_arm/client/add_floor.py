#!/usr/bin/env python3
"""Planning scene'e zemin (Z=0'ın altı yasak) çarpışma nesnesi ekler."""
import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from pymoveit2 import MoveIt2


def main():
    rclpy.init()
    node = Node("add_floor_collision")
    callback_group = ReentrantCallbackGroup()
    moveit2 = MoveIt2(
        node=node,
        joint_names=["j1", "j2", "j3", "j4", "j5", "j6"],
        base_link_name="base_link",
        end_effector_name="wrist3_link",
        group_name="fairino5_v6_group",
        callback_group=callback_group,
    )
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    import time
    time.sleep(1.0)  # bağlantının oturması için kısa bekleme

    moveit2.add_collision_box(
        id="floor",
        size=(2.0, 2.0, 0.1),
        position=[0.0, 0.0, -0.05],
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        frame_id="base_link",
    )
    node.get_logger().info("Zemin çarpışma nesnesi eklendi (Z=0'ın altı artık yasak).")
    time.sleep(1.0)

    executor.shutdown()
    rclpy.shutdown()
    executor_thread.join(timeout=2)


if __name__ == "__main__":
    main()
