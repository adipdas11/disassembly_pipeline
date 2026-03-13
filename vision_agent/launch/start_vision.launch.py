from launch import LaunchDescription
from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='vision_agent',
#             executable='start_vision',
#             name='agent_vision_node',
#             output='screen'
#         )
#     ])
    
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_agent',
            executable='start_vision_rtdetr',
            name='agent_vision_node',
            output='screen'
        )
    ])