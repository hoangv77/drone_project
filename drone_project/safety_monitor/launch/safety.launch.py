"""Start the telemetry adapter and safety monitor together."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory('safety_monitor')) /
                 'config' / 'safety_monitor.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        Node(package='px4_interface_node', executable='px4_interface_node', output='screen'),
        Node(package='safety_monitor', executable='safety_monitor', name='safety_monitor',
             parameters=[LaunchConfiguration('config')], output='screen'),
    ])
