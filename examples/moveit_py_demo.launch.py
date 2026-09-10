from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "fairino5_v6_robot", package_name="fairino5_v6_moveit2_config"
    ).to_moveit_configs()

    launch_package_path = moveit_config.package_path
    ld = LaunchDescription()

    virtual_joints_launch = launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    if virtual_joints_launch.exists():
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(virtual_joints_launch))
        ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_package_path / "launch/rsp.launch.py"))
    ))

    ld.add_action(Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
        output="screen",
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_package_path / "launch/spawn_controllers.launch.py"))
    ))

    return ld
