import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('tool_controller'),
        'config',
        'tool_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='tool_controller',
            executable='tool_commander',
            name='tool_commander',
            output='screen',
            parameters=[config]
        )
    ])