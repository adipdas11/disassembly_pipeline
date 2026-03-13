import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    hw_type = LaunchConfiguration("hardware_type").perform(context)
    enable_servo = LaunchConfiguration("enable_servo").perform(context).lower() == "true"

    print(f"\n{'=' * 50}\nSTARTING DUAL-ARM MOVEIT IN [{hw_type.upper()}] MODE\n{'=' * 50}\n")

    pkg_share = get_package_share_directory("dual_arm_moveit_config")
    tool_pkg_share = get_package_share_directory("tool_controller")
    rviz_config_file = os.path.join(pkg_share, "rviz", "dual_arm.rviz")
    ros2_controllers_path = os.path.join(pkg_share, "config", "ros2_controllers.yaml")
    tool_params_path = os.path.join(tool_pkg_share, "config", "tool_params.yaml")
    xarm_servo_yaml = os.path.join(pkg_share, "config", "xarm_servo.yaml")
    uf_servo_yaml = os.path.join(pkg_share, "config", "uf_servo.yaml")
    with open(xarm_servo_yaml, "r", encoding="utf-8") as file:
        xarm_servo_params = yaml.safe_load(file)
    with open(uf_servo_yaml, "r", encoding="utf-8") as file:
        uf_servo_params = yaml.safe_load(file)
    xarm_filter_params = {
        "online_signal_smoothing": {
            "butterworth_filter_coeff": xarm_servo_params.pop("butterworth_filter_coeff", 1.5),
        }
    }
    uf_filter_params = {
        "online_signal_smoothing": {
            "butterworth_filter_coeff": uf_servo_params.pop("butterworth_filter_coeff", 1.5),
        }
    }

    moveit_config = (
        MoveItConfigsBuilder("dual_arm_world", package_name="dual_arm_moveit_config")
        .robot_description(
            file_path=os.path.join(pkg_share, "config", "dual_arm_world.urdf.xacro"),
            mappings={"hw_type": hw_type},
        )
        .robot_description_semantic(file_path=os.path.join(pkg_share, "config", "dual_arm_world.srdf"))
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "world_world"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": False,
                "trajectory_execution.allowed_execution_duration_scaling": 4.0,
                "trajectory_execution.allowed_goal_duration_margin": 2.5,
                "trajectory_execution.execution_duration_monitoring": True,
            },
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config.to_dict()],
        output="screen",
    )

    xarm_servo = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="xarm_servo_node",
        parameters=[
            {"moveit_servo": xarm_servo_params},
            xarm_filter_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": False},
        ],
        output="screen",
    )

    uf_servo = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="uf_servo_node",
        parameters=[
            {"moveit_servo": uf_servo_params},
            uf_filter_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": False},
        ],
        output="screen",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    teleop_bridge = Node(
        package="dual_arm_moveit_config",
        executable="teleop_bridge.py",
        name="teleop_bridge",
        output="screen",
    )

    tool_commander = Node(
        package="tool_controller",
        executable="tool_commander",
        name="tool_commander",
        output="screen",
        parameters=[tool_params_path],
    )

    real_hardware_bridge = Node(
        package="dual_arm_moveit_config",
        executable="real_hardware.py",
        name="real_hardware_bridge",
        output="screen",
    )

    controller_manager_args = ["--controller-manager", "/controller_manager"]
    controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", *controller_manager_args],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["xarm_controller", *controller_manager_args],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["uf_controller", *controller_manager_args],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["rg6_controller", *controller_manager_args],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["slider_controller", *controller_manager_args],
            output="screen",
        ),
    ]

    launch_actions = [
        LogInfo(msg="Step 1: Publishing robot description"),
        static_tf,
        robot_state_publisher,
    ]

    if hw_type == "real":
        launch_actions.extend(
            [
                TimerAction(
                    period=1.0,
                    actions=[
                        LogInfo(msg="Step 1.5: Starting real hardware bridge"),
                        real_hardware_bridge,
                    ],
                ),
            ]
        )

    launch_actions.extend(
        [
        TimerAction(
            period=2.0,
            actions=[
                LogInfo(msg=f"Step 2: Starting ros2_control for hardware_type={hw_type}"),
                ros2_control_node,
            ],
        ),
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg="Step 3: Spawning controllers"),
                *controller_spawners,
            ],
        ),
        TimerAction(
            period=8.0,
            actions=[
                LogInfo(msg="Step 4: Starting MoveIt and RViz"),
                move_group,
                rviz,
            ],
        ),
        ]
    )

    if enable_servo:
        launch_actions.append(
            TimerAction(
                period=10.0,
                actions=[
                    LogInfo(msg="Step 5: Starting MoveIt Servo"),
                    xarm_servo,
                    uf_servo,
                ],
            )
        )

    launch_actions.append(
        TimerAction(
            period=12.0 if enable_servo else 9.0,
            actions=[
                LogInfo(msg="Step 6: Starting joystick and teleop bridge"),
                joy_node,
                teleop_bridge,
                tool_commander,
            ],
        )
    )

    return launch_actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_type",
                default_value="fake",
                description='Hardware backend: "fake", "real", or "isaac"',
            ),
            DeclareLaunchArgument(
                "enable_servo",
                default_value="true",
                description="Start MoveIt Servo nodes for xArm and UF arms",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
