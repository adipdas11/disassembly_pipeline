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
    pkg = get_package_share_directory('screwdriver_description')
    default_xacro = os.path.join(pkg, 'urdf', 'screwdriver_instance.urdf.xacro')
    default_rviz  = os.path.join(pkg, 'rviz', 'screwdriver.rviz')  # optional

    model_arg  = DeclareLaunchArgument('model', default_value=default_xacro)
    gui_arg    = DeclareLaunchArgument('gui',   default_value='true')
    rviz_arg   = DeclareLaunchArgument('rviz',  default_value='true')

    prefix_arg = DeclareLaunchArgument('prefix', default_value='sd1_')
    mesh_pkg_arg    = DeclareLaunchArgument('mesh_pkg', default_value='screwdriver_description')
    mesh_subdir_arg = DeclareLaunchArgument('mesh_subdir', default_value='meshes/visual')
    color_primary_arg   = DeclareLaunchArgument('color_primary',   default_value='0.75 0.75 0.78 1.0')
    color_secondary_arg = DeclareLaunchArgument('color_secondary', default_value='0.60 0.60 0.65 1.0')

    xacro_cmd = Command([
        FindExecutable(name='xacro'), ' ',
        LaunchConfiguration('model'), ' ',
        'prefix:=',        LaunchConfiguration('prefix'), ' ',
        'mesh_pkg:=',      LaunchConfiguration('mesh_pkg'), ' ',
        'mesh_subdir:=',   LaunchConfiguration('mesh_subdir'), ' ',
        'color_primary:=',  '"', LaunchConfiguration('color_primary'),  '"', ' ',
        'color_secondary:=','"', LaunchConfiguration('color_secondary'),'"',
    ])
    robot_description = ParameterValue(xacro_cmd, value_type=str)

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
    jsp = Node(
        condition=UnlessCondition(LaunchConfiguration('gui')),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    rviz_args = ['-d', default_rviz] if os.path.exists(default_rviz) else []
    rviz2 = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args
    )

    return LaunchDescription([
        model_arg, gui_arg, rviz_arg,
        prefix_arg, mesh_pkg_arg, mesh_subdir_arg,
        color_primary_arg, color_secondary_arg,
        rsp, jsp_gui, jsp, rviz2
    ])
