from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_agent',
            executable='start_vision_v2',
            name='agent_vision_node',
            output='screen'
            # REMOVED: env={'PYTHONPATH': ...} 
            # This ensures we inherit the correct LD_LIBRARY_PATH from the terminal
        )
    ])