from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. RealSense Camera
    rs_pkg = FindPackageShare('realsense2_camera')
    launch_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rs_pkg, '/launch/rs_launch.py'])
    )

    # 2. Workspace Visualizer Node
    workspace_node = Node(
        package='vision_agent',
        executable='detect_workspace',  # We will define this entry point next
        name='workspace_detector',
        output='screen'
    )

    return LaunchDescription([
        LogInfo(msg="🚀 Starting RealSense and Workspace Visualizer..."),
        launch_realsense,
        workspace_node
    ])