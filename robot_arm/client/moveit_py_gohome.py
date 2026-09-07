#!/usr/bin/env python3
"""moveit_py ile home pozisyonuna hareket testi."""
import os
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit_configs_utils import MoveItConfigsBuilder


def main():
    yaml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fairino5_moveit_py.yaml")
    moveit_config = (
        MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config")
        .moveit_cpp(file_path=yaml_path)
        .to_moveit_configs()
    )
    fairino5 = MoveItPy(node_name="moveit_py_fairino5", config_dict=moveit_config.to_dict())
    planning_component = fairino5.get_planning_component("fairino5_v6_group")

    planning_component.set_start_state_to_current_state()
    robot_model = fairino5.get_robot_model()
    home_state = RobotState(robot_model)
    home_state.set_joint_group_positions("fairino5_v6_group", [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
    planning_component.set_goal_state(robot_state=home_state)

    print("Planlanıyor...")
    plan_result = planning_component.plan()
    if plan_result:
        print("Planlama başarılı, çalıştırılıyor...")
        fairino5.execute("fairino5_v6_group", plan_result.trajectory)
        print("Tamamlandı!")
    else:
        print("HATA: Planlama başarısız oldu.")


if __name__ == "__main__":
    main()
