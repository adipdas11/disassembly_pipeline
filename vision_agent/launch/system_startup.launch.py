from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- 1. DEFINE PATHS ---
    rs_pkg = FindPackageShare('realsense2_camera')
    tool_pkg = FindPackageShare('tool_camera_pkg')
    agent_pkg = FindPackageShare('vision_agent')

    # --- 2. DEFINE LAUNCH ACTIONS ---
    
    # A. RealSense (Global Scout)
    # Keeping the necessary parameters for PointCloud and Depth synchronization
    launch_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rs_pkg, '/launch/rs_launch.py']),
        launch_arguments={
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'pointcloud.stream_filter': '2', # Texture from Color
            'pointcloud.allow_no_texture_points': 'true',
            'pointcloud.ordered_pc': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_sync': 'true',
            'tf_publish_rate': '10.0',
        }.items()
    )

    # B. Tool Camera (Local Sniper)
    launch_tool_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([tool_pkg, '/launch/tool_camera.launch.py'])
    )

    # C. Vision Agent (Brain)
    launch_agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([agent_pkg, '/launch/start_vision.launch.py'])
    )

    # --- 3. CREATE STARTUP SEQUENCE WITH LOGS ---
    return LaunchDescription([
        
        # T+0: Start RealSense
        LogInfo(msg="🚀 [1/3] INITIALIZING GLOBAL SCOUT (REALSENSE)... 📷"),
        launch_realsense,

        # T+3: Start Tool Camera (Wait 3s for RS to settle)
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg="🔧 [2/3] STARTING LOCAL SNIPER (TOOL CAM)... 🔬"),
                launch_tool_cam
            ]
        ),

        # T+6: Start AI Agent (Wait 3s for Tool Cam)
        TimerAction(
            period=6.0,
            actions=[
                LogInfo(msg="🧠 [3/3] ACTIVATING VISION AGENT BRAIN... 🤖"),
                launch_agent
            ]
        ),

        # T+8: Final Ready Message
        TimerAction(
            period=8.0,
            actions=[
                LogInfo(msg="✅ SYSTEM READY: ALL NODES ONLINE. OPENING EYES... 👀")
            ]
        )
    ])