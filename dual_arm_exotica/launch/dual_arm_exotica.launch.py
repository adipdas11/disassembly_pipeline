from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ik_node = Node(
        package="dual_arm_exotica",
        executable="dual_arm_ik_planner",
        name="dual_arm_exotica_ik_planner",
        output="screen",
    )

    trajectory_node = Node(
        package="dual_arm_exotica",
        executable="dual_arm_trajectory_planner",
        name="dual_arm_exotica_trajectory_planner",
        output="screen",
    )

    return LaunchDescription(
        [
            ik_node,
            trajectory_node,
        ]
    )
