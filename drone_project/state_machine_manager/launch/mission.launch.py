"""Start the passive manager; optionally include the existing telemetry/safety pair."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    own_share = Path(get_package_share_directory('state_machine_manager'))
    safety_share = Path(get_package_share_directory('safety_monitor'))
    return LaunchDescription([
        DeclareLaunchArgument('start_interfaces', default_value='false'),
        DeclareLaunchArgument('config', default_value=str(own_share / 'config' / 'mission.yaml')),
        DeclareLaunchArgument('safety_config', default_value=str(safety_share / 'config' / 'safety_monitor.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(safety_share / 'launch' / 'safety.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_interfaces')),
            launch_arguments={'config': LaunchConfiguration('safety_config')}.items()),
        Node(package='state_machine_manager', executable='state_machine_manager',
             name='state_machine_manager', parameters=[LaunchConfiguration('config')], output='screen'),
    ])
