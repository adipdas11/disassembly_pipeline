import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def find_repo_root(current_path, target_name="disassembly_pipeline"):
    curr = os.path.abspath(current_path)
    while curr != os.path.dirname(curr):
        if os.path.basename(curr) == target_name:
            return curr
        if os.path.exists(os.path.join(curr, target_name)):
            return os.path.join(curr, target_name)
        curr = os.path.dirname(curr)
    return None

def launch_setup(context, *args, **kwargs):
    hw_type = LaunchConfiguration("hardware_type").perform(context)
    enable_servo = LaunchConfiguration("enable_servo").perform(context).lower() == "true"
    
    # 1. SETUP CONFIG
    moveit_config_pkg = "dual_arm_moveit_config"
    pkg_share = get_package_share_directory(moveit_config_pkg)
    disassembly_skills_share = get_package_share_directory("disassembly_skills")
    tool_pkg_share = get_package_share_directory("tool_controller")
    
    rviz_config_file = os.path.join(pkg_share, "rviz", "dual_arm.rviz")
    ros2_controllers_path = os.path.join(pkg_share, "config", "ros2_controllers.yaml")
    tool_params_path = os.path.join(tool_pkg_share, "config", "tool_params.yaml")
    
    # Robust calibration file path
    handeye_calibration_file = os.path.join(disassembly_skills_share, "config", "realsense_handeye.calib")
    if not os.path.exists(handeye_calibration_file):
        ws_root = find_repo_root(__file__)
        if ws_root:
            handeye_calibration_file = os.path.join(ws_root, "disassembly_skills", "config", "realsense_handeye.calib")

    moveit_config = (
        MoveItConfigsBuilder("dual_arm_world", package_name=moveit_config_pkg)
        .robot_description(
            file_path=os.path.join(pkg_share, "config", "dual_arm_world.urdf.xacro"),
            mappings={"hw_type": hw_type}
        )
        .robot_description_semantic(file_path="config/dual_arm_world.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # 2. DEFINE NODES
    
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "world_world"],
    )

    run_rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # --- Hardware Control Node ---
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    # --- State Management Nodes ---
    state_manager = Node(
        package="dual_arm_moveit_config",
        executable="state_manager.py",
        name="disassembly_state_manager",
        output="screen",
    )

    hold_state_manager = Node(
        package="dual_arm_moveit_config",
        executable="object_hold_state.py",
        name="object_hold_state_manager",
        output="screen",
    )

    # B. Camera Calibration (Handeye Publisher)
    handeye_publisher = Node(
        package="easy_handeye2",
        executable="handeye_publisher",
        name="handeye_publisher",
        parameters=[{
            "name": "realsense_handeye",
            "calibration_file": handeye_calibration_file,
        }],
        output="screen",
    )

    # C. FT Sensor
    robotiq_ft_sensor = Node(
        package="robotiq_ft_sensor_hardware",
        executable="robotiq_ft_sensor_standalone_node",
        name="robotiq_ft_sensor",
        parameters=[
            {"max_retries": 100},
            {"read_rate": 100},
            {"frame_id": "ft_robotiq_ft_frame_id"},
        ],
        remappings=[
            ('robotiq_force_torque_sensor_broadcaster/wrench', '/robotiq_force_torque_sensor_broadcaster/wrench')
        ],
        output="screen",
    )

    # D. Move Group
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
    )
    
    # E. Rviz
    run_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],  
        parameters=[moveit_config.to_dict()],
    )

    # F. Real Hardware Bridge
    real_hardware_bridge = Node(
        package="dual_arm_moveit_config",
        executable="real_hardware.py",
        name="real_hardware_bridge",
        output="screen",
    )

    # G. Tool Commander
    tool_commander = Node(
        package="tool_controller",
        executable="tool_commander",
        name="tool_commander",
        output="screen",
        parameters=[tool_params_path],
    )

    # --- MoveIt Servo Setup ---
    xarm_servo_yaml = os.path.join(pkg_share, "config", "xarm_servo.yaml")
    uf_servo_yaml = os.path.join(pkg_share, "config", "uf_servo.yaml")
    
    with open(xarm_servo_yaml, "r", encoding="utf-8") as file:
        xarm_servo_params = yaml.safe_load(file)
    with open(uf_servo_yaml, "r", encoding="utf-8") as file:
        uf_servo_params = yaml.safe_load(file)

    xarm_servo = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="xarm_servo_node",
        parameters=[
            {"moveit_servo": xarm_servo_params},
            moveit_config.to_dict(),
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
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
        output="screen",
    )

    # --- Joystick and Teleop Bridge ---
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

    # --- Controller Spawners ---
    controller_manager_args = ["--controller-manager", "/controller_manager"]
    controller_spawners = [
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster", *controller_manager_args]),
        Node(package="controller_manager", executable="spawner", arguments=["xarm_controller", *controller_manager_args]),
        Node(package="controller_manager", executable="spawner", arguments=["uf_controller", *controller_manager_args]),
        Node(package="controller_manager", executable="spawner", arguments=["rg6_controller", *controller_manager_args]),
        Node(package="controller_manager", executable="spawner", arguments=["slider_controller", *controller_manager_args]),
    ]

    # --- Startup Sequence ---
    actions = [
        LogInfo(msg="Step 1: Starting TF, Calibration, State Managers and Control Manager"),
        static_tf,
        run_rsp_node,
        handeye_publisher,
        robotiq_ft_sensor,
        state_manager,
        hold_state_manager,
        ros2_control_node,
    ]

    if hw_type == "real":
        actions.append(TimerAction(period=1.0, actions=[real_hardware_bridge]))

    actions.append(TimerAction(period=3.0, actions=[
        LogInfo(msg="Step 2: Spawning Controllers"),
        *controller_spawners
    ]))

    actions.append(TimerAction(period=6.0, actions=[
        LogInfo(msg="Step 3: Starting MoveIt and RViz"),
        run_move_group_node, 
        run_rviz_node
    ]))

    if enable_servo:
        actions.append(TimerAction(period=8.0, actions=[
            LogInfo(msg="Step 4: Starting MoveIt Servo"),
            xarm_servo,
            uf_servo,
            joy_node,
            teleop_bridge,
        ]))

    actions.append(TimerAction(period=10.0 if enable_servo else 8.0, actions=[
        LogInfo(msg="Step 5: Starting Tool Commander"),
        tool_commander
    ]))

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hardware_type", default_value="fake"),
        DeclareLaunchArgument("enable_servo", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
