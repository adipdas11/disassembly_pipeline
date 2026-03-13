import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'disassembly_skills'
    
    # 1. Load Calibration File
    config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'camera_calibration.yaml'
    )
    
    # Python trick to read the YAML strictly for the static publisher
    # (Since static_transform_publisher doesn't take YAML directly, it takes args)
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
        
    cam_conf = config['camera_transform']
    trans = cam_conf['translation']
    rot = cam_conf['rotation_rpy']
    parent = cam_conf['parent_frame']
    child = cam_conf['child_frame']

    return LaunchDescription([
        # --- A. STATIC TRANSFORM PUBLISHER ---
        # This takes the numbers from YAML and broadcasts them to /tf_static
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_broadcaster',
            arguments=[
                str(trans[0]), str(trans[1]), str(trans[2]),  # X, Y, Z
                str(rot[0]), str(rot[1]), str(rot[2]),        # Roll, Pitch, Yaw
                parent, child                                 # Frames
            ],
            output='screen'
        ),

        # --- B. VISION AGENT (The AI Models) ---
        # Assuming your agent_node is in a package named 'vision_system'
        Node(
            package='vision_system', 
            executable='agent_node',
            name='vision_agent',
            output='screen'
        ),

        # --- C. VISION BRIDGE (The Math Converter) ---
        Node(
            package='disassembly_skills',
            executable='vision_bridge',
            name='vision_bridge',
            output='screen'
        )
    ])