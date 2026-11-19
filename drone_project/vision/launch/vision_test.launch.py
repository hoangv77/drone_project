"""Launch with synthetic frames and explicitly synthetic calibration."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('drone_vision'))
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='two_tags'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/vision.launch.py')),
            launch_arguments={'tag_config': str(share / 'config/apriltag.yaml')}.items()),
        Node(package='drone_vision', executable='test_camera', output='screen',
             parameters=[{'scenario': LaunchConfiguration('scenario')}]),
    ])
