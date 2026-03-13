from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os 

def generate_launch_description():
    # --- 1. DEFINE PATHS ---
    rs_pkg_share = FindPackageShare('realsense2_camera')
    tool_pkg_share = FindPackageShare('tool_camera_pkg')
    agent_pkg_share = FindPackageShare('vision_agent')

    # --- 2. DEFINE LAUNCH ACTIONS ---
    
    # A. RealSense (Global Scout)
    launch_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rs_pkg_share, '/launch/rs_launch.py']),
        launch_arguments={
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'pointcloud.allow_no_texture_points': 'true',
        }.items()
    )

    # B. Tool Camera (Local Sniper)
    launch_tool_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([tool_pkg_share, '/launch/tool_camera.launch.py'])
    )

    # C. Vision Agent (Brain)
    launch_agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([agent_pkg_share, '/launch/start_vision.launch.py'])
    )

    # --- 3. CREATE STARTUP SEQUENCE WITH LOGS ---
    return LaunchDescription([
        
        # T+0: Start RealSense
        LogInfo(msg="🚀 [1/3] INITIALIZING GLOBAL SCOUT (REALSENSE)... 📷"),
        launch_realsense,

        # T+3: Start Tool Camera
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg="🔧 [2/3] STARTING LOCAL SNIPER (TOOL CAM)... 🔬"),
                launch_tool_cam
            ]
        ),

        # T+6: Start AI Agent
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