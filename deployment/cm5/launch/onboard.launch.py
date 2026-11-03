"""Launch the CM5 monitoring stack with fixed command and mission interlocks."""
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import rclpy
from rclpy.context import Context


def generate_launch_description():
    safety = Path(get_package_share_directory('safety_monitor'))
    mission = Path(get_package_share_directory('state_machine_manager'))
    nodes = [
        Node(package='px4_interface_node', executable='px4_interface_node', output='screen',
             parameters=[{'read_only': True, 'use_sim_time': False}]),
        Node(package='safety_monitor', executable='safety_monitor', output='screen',
             parameters=[LaunchConfiguration('safety_config'), {'use_sim_time': False}]),
        Node(package='state_machine_manager', executable='state_machine_manager', output='screen',
             parameters=[LaunchConfiguration('mission_config'), {
                 'monitor_only': True, 'enable_stage_feedback': False, 'use_sim_time': False,
             }]),
    ]

    def start(context):
        graph_context = Context()
        rclpy.init(args=[], context=graph_context)
        probe = rclpy.create_node('onboard_graph_guard', context=graph_context,
                                  enable_rosout=False, start_parameter_services=False)
        closed = False

        def conflict(starting):
            managers = [name for name, _ in probe.get_node_names_and_namespaces()
                        if name in ('state_machine_manager', 'throw_launch_manager')]
            outbound = any(topic.startswith('/fmu/in/') and probe.count_publishers(topic)
                           for topic, _ in probe.get_topic_names_and_types())
            return ('throw_launch_manager' in managers or len(managers) > int(not starting)
                    or probe.count_publishers('/drone/mission_state') > int(not starting)
                    or outbound)

        def close(_):
            nonlocal closed
            if not closed:
                closed = True
                probe.destroy_node()
                if graph_context.ok():
                    graph_context.shutdown()

        def required_node_exited(_, launch_context):
            if not launch_context.is_shutdown:
                return [EmitEvent(event=Shutdown(reason='Required onboard node exited'))]

        def watch(_):
            if conflict(False):
                return [EmitEvent(event=Shutdown(reason='Conflicting ROS flight endpoint detected'))]
            return [TimerAction(period=2.0, actions=[OpaqueFunction(function=watch)])]

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if conflict(True):
                close(context)
                raise RuntimeError('Use a dedicated ROS domain and stop conflicting managers or command publishers')
            time.sleep(0.1)
        return [
            RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=close)])),
            *[RegisterEventHandler(OnProcessExit(
                target_action=node, on_exit=required_node_exited))
              for node in nodes],
            LogInfo(msg='CM5 monitoring only. PX4 commands and mission requests are disabled.'),
            *nodes,
            TimerAction(period=2.0, actions=[OpaqueFunction(function=watch)]),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('safety_config', default_value=str(safety / 'config/safety_monitor.yaml')),
        DeclareLaunchArgument('mission_config', default_value=str(mission / 'config/mission.yaml')),
        OpaqueFunction(function=start),
    ])
