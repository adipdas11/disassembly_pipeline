#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('uf850_mount_description')

    default_xacro = os.path.join(pkg, 'urdf', 'uf850_mount_instance.urdf.xacro')
    default_rviz  = os.path.join(pkg, 'rviz', 'uf850_mounting_plate.rviz') 

    # ----- Args -----
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_xacro,
        description='Path to u850 mounting plate instance xacro'
    )

    prefix_arg = DeclareLaunchArgument(
        'plate_prefix',
        default_value='u850_',
        description='Prefix for link/joint/material names'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Use joint_state_publisher_gui if true'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2 if true'
    )

    rviz_cfg_default = default_rviz if os.path.exists(default_rviz) else ''
    rviz_cfg_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_cfg_default,
        description='RViz config file (optional)'
    )

    # xacro -> URDF
    xacro_cmd = Command([
        FindExecutable(name='xacro'), ' ',
        LaunchConfiguration('model'), ' ',
        'plate_prefix:=', LaunchConfiguration('plate_prefix')
    ])

    robot_description = ParameterValue(xacro_cmd, value_type=str)

    # ----- Nodes -----
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    jsp_gui = Node(
        condition=IfCondition(LaunchConfiguration('gui')),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    jsp_headless = Node(
        condition=UnlessCondition(LaunchConfiguration('gui')),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    rviz_args = []
    if os.path.exists(default_rviz):
        rviz_args = ['-d', LaunchConfiguration('rviz_config')]

    rviz2 = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args
    )

    return LaunchDescription([
        model_arg,
        prefix_arg,
        gui_arg,
        rviz_arg,
        rviz_cfg_arg,
        rsp,
        jsp_gui,
        jsp_headless,
        rviz2
    ])
