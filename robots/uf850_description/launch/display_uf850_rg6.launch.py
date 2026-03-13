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
    pkg = get_package_share_directory('uf850_description')

    default_xacro = os.path.join(pkg, 'urdf', 'uf850_rg6.urdf.xacro')
    default_rviz  = os.path.join(pkg, 'rviz', 'uf850.rviz')  # optional file

    # ---- Args ----
    xacro_arg = DeclareLaunchArgument(
        'model', default_value=default_xacro,
        description='Path to .xacro (instance file)'
    )
    prefix_arg = DeclareLaunchArgument('prefix', default_value='u1_', description='Prefix for link/joint names')
    mesh_pkg_arg = DeclareLaunchArgument('mesh_pkg', default_value='uf850_description', description='Mesh package')
    mesh_subdir_arg = DeclareLaunchArgument('mesh_subdir', default_value='meshes/visual', description='Mesh subdir')
    gui_arg  = DeclareLaunchArgument('gui',  default_value='true', description='Use joint_state_publisher_gui')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Start RViz2')
    rviz_cfg_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz if os.path.exists(default_rviz) else '',
        description='RViz config file (optional)'
    )

    xacro_path = LaunchConfiguration('model')

    # Render xacro -> URDF at runtime
    xacro_cmd = Command([
        FindExecutable(name='xacro'), ' ',
        xacro_path, ' ',
        'prefix:=',      LaunchConfiguration('prefix'), ' ',
        'mesh_pkg:=',    LaunchConfiguration('mesh_pkg'), ' ',
        'mesh_subdir:=', LaunchConfiguration('mesh_subdir')
    ])
    robot_description = ParameterValue(xacro_cmd, value_type=str)

    # ---- Nodes ----
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
        xacro_arg, prefix_arg, mesh_pkg_arg, mesh_subdir_arg,
        gui_arg, rviz_arg, rviz_cfg_arg,
        rsp, jsp_gui, jsp_headless, rviz2
    ])
