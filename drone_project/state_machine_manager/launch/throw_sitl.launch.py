"""Opt-in SITL manager, read-only native observer, existing interface and safety."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    safety = Path(get_package_share_directory('safety_monitor'))
    return LaunchDescription([
        DeclareLaunchArgument('px4_bin'),
        DeclareLaunchArgument('observer_script'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(safety/'launch/safety.launch.py'))),
        Node(package='state_machine_manager', executable='throw_launch_manager', output='screen'),
        ExecuteProcess(cmd=['python3', LaunchConfiguration('observer_script'),
                            '--px4-bin', LaunchConfiguration('px4_bin')], output='screen'),
    ])
