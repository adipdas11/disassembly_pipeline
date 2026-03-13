from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arg_name = DeclareLaunchArgument('name')
    arg_calibration_file = DeclareLaunchArgument('calibration_file', default_value='')

    handeye_publisher = Node(package='easy_handeye2', executable='handeye_publisher', name='handeye_publisher', parameters=[{
        'name': LaunchConfiguration('name'),
        'calibration_file': LaunchConfiguration('calibration_file'),
    }])

    return LaunchDescription([
        arg_name,
        arg_calibration_file,
        handeye_publisher,
    ])
