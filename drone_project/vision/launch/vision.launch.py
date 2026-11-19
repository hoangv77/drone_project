"""Launch a camera-independent AprilTag pipeline."""
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def start(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    config = yaml.safe_load(Path(arg('camera_config')).read_text())['camera_gate']['ros__parameters']
    tags = yaml.safe_load(Path(arg('tag_config')).read_text())['apriltag']['ros__parameters']
    sizes = [tags['size'], *tags.get('tag', {}).get('sizes', [])]
    if not all(type(size) in (int, float) and math.isfinite(size) and size > 0 for size in sizes):
        raise ValueError('Tag edge sizes must be positive meters')
    optical = config['optical_frame']
    body = config['body_frame']
    actions = []
    if arg('driver_launch'):
        actions.append(IncludeLaunchDescription(AnyLaunchDescriptionSource(arg('driver_launch'))))
    if arg('mount'):
        mount = yaml.safe_load(Path(arg('mount')).read_text())
        xyz, rotation = mount['translation'], mount['quaternion_xyzw']
        parent, child = mount['parent'], mount['child']
        if (not isinstance(xyz, list) or not isinstance(rotation, list)
                or len(xyz) != 3 or len(rotation) != 4
                or not all(isinstance(value, (float, int)) and math.isfinite(value)
                           for value in xyz + rotation)
                or not math.isclose(sum(value ** 2 for value in rotation), 1, abs_tol=1e-5)
                or parent != body or child in (parent, optical) or not child or not optical):
            raise ValueError('Mount requires measured XYZ, a unit quaternion and distinct frames')
        for source, target, values in (
            (parent, child, xyz + rotation),
            (child, optical, [0, 0, 0, -0.5, 0.5, -0.5, 0.5]),
        ):
            # Optical convention is z forward, x right, y down.
            actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                                arguments=[*(str(value) for value in values), source, target]))
    return actions + [
        Node(package='drone_vision', executable='camera_gate', name='camera_gate',
             parameters=[arg('camera_config'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
             output='screen'),
        Node(package='apriltag_ros', executable='apriltag_node', name='apriltag',
             parameters=[arg('tag_config'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
             remappings=[('image_rect', '/vision/image_rect'), ('camera_info', '/vision/camera_info'),
                         ('detections', '/vision/detections')], output='screen'),
    ]


def generate_launch_description():
    config = Path(get_package_share_directory('drone_vision')) / 'config'
    return LaunchDescription([
        DeclareLaunchArgument('camera_config', default_value=str(config / 'camera.yaml')),
        DeclareLaunchArgument('tag_config', description='AprilTag YAML with explicitly selected tag sizes'),
        DeclareLaunchArgument('driver_launch', default_value=''),
        DeclareLaunchArgument('mount', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=start),
    ])
