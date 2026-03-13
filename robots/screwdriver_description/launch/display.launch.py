#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('screwdriver_description')
    default_urdf = os.path.join(pkg_share, 'urdf', 'screwdriver.urdf')
    default_rviz = os.path.join(pkg_share, 'rviz', 'view.rviz')

    urdf_arg = DeclareLaunchArgument(
        'urdf', default_value=default_urdf,
        description='Path to URDF file'
    )
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Use joint_state_publisher_gui for interactive sliders'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz if os.path.exists(default_rviz) else '',
        description='RViz config file (optional)'
    )

    urdf_path = LaunchConfiguration('urdf')

    # Read URDF into a parameter (simple & fast for a single file)
    with open(default_urdf, 'r') as f:
        robot_description = f.read()

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint State Publisher (GUI)
    jsp_gui = Node(
        condition=IfCondition(LaunchConfiguration('gui')),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Joint State Publisher (headless; still lets you publish fixed poses via parameters if needed)
    jsp_headless = Node(
        condition=UnlessCondition(LaunchConfiguration('gui')),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        # parameters=[{'rate': 30.0}]  # uncomment to change publish rate
    )

    # RViz2 (optional)
    rviz_args = []
    rviz_config = LaunchConfiguration('rviz_config')
    # Only add the -d arg if a config file exists
    if os.path.exists(default_rviz):
        rviz_args = ['-d', rviz_config]

    rviz2 = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args
    )

    return LaunchDescription([
        urdf_arg, gui_arg, rviz_arg, rviz_config_arg,
        rsp,
        jsp_gui,
        jsp_headless,
        rviz2,
    ])
