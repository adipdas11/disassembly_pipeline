from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Get path to config file in the NEW package
    config_file = os.path.join(
        get_package_share_directory('tool_camera_pkg'),
        'config',
        'tool_camera_info.yaml'
    )

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='tool_camera',
            namespace='tool_cam',
            parameters=[{
                'video_device': '/dev/video10',  
                'framerate': 30.0,
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'yuyv2rgb',     
                'camera_name': 'tool_camera',
                'camera_info_url': 'file://' + config_file
            }]
        )
    ])