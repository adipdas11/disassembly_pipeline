import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Hardcoded path to your config file
    config_file = os.path.join(
        os.getenv('HOME'), 
        'workspace/disassembly_ws/src/disassembly_skills/config/tags.yaml'
    )

    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            # 1. REMAPPINGS (Matches your "-r" flags)
            remappings=[
                ('image_rect', '/camera/camera/color/image_raw'),
                ('camera_info', '/camera/camera/color/camera_info'),
            ],
            # 2. PARAMETERS (Loads the YAML + Overrides)
            parameters=[
                config_file,
                {
                    'camera_frame': 'camera_color_optical_frame', # Matches "-p camera_frame"
                    'image_transport': 'compressed'               # FORCE compressed mode
                }
            ]
        )
    ])