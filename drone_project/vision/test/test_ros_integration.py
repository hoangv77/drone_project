"""Opt-in tests using the installed Humble package in an isolated domain."""
from contextlib import contextmanager
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest


@contextmanager
def process(command, **kwargs):
    proc = subprocess.Popen(command, start_new_session=True, **kwargs)
    try:
        yield proc
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)


@unittest.skipUnless(os.environ.get('PHASE8_TEST_ROS') == '1', 'requires installed Humble vision overlay')
class IntegrationTest(unittest.TestCase):
    def setUp(self):
        import rclpy
        from ament_index_python.packages import get_package_prefix

        self.camera = Path(get_package_prefix('drone_vision')) / 'lib/drone_vision/test_camera'
        os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='91')
        rclpy.init(args=[])
        self.addCleanup(rclpy.try_shutdown)
        self.node = rclpy.create_node('phase8_integration_test')
        self.addCleanup(self.node.destroy_node)

    def until(self, predicate, timeout=10):
        import rclpy
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return
        self.fail('Timed out waiting for expected ROS result')

    def test_camera_calibration_detection_and_disconnect(self):
        from apriltag_msgs.msg import AprilTagDetectionArray
        from ament_index_python.packages import get_package_share_directory
        from diagnostic_msgs.msg import DiagnosticArray
        from tf2_msgs.msg import TFMessage

        detections, diagnostics, transforms = [], [], []
        self.node.create_subscription(AprilTagDetectionArray, '/vision/detections', detections.append, 10)
        self.node.create_subscription(DiagnosticArray, '/diagnostics', diagnostics.append, 10)
        self.node.create_subscription(TFMessage, '/tf', transforms.append, 10)

        def diagnostic(text):
            return bool(diagnostics and text in diagnostics[-1].status[0].message)

        tag_config = Path(get_package_share_directory('drone_vision')) / 'config/apriltag.yaml'
        with process(['ros2', 'launch', 'drone_vision', 'vision.launch.py', f'tag_config:={tag_config}']):
            self.until(lambda: diagnostic('Camera stream unavailable'))
            for scenario, expected in [
                ('missing_info', 'Missing synchronized'), ('wrong_resolution', 'resolution differs'),
                ('stale', 'Stale or future-dated'), ('two_tags', {0, 1}), ('blank', set()),
            ]:
                detections.clear()
                diagnostics.clear()
                with process([self.camera, '--ros-args',
                              '-p', f'scenario:={scenario}']):
                    if isinstance(expected, str):
                        self.until(lambda: diagnostic(expected))
                        self.assertFalse(detections)
                    else:
                        self.until(lambda: any({d.id for d in frame.detections} == expected
                                               for frame in detections))
                        self.assertEqual(detections[-1].header.frame_id, 'camera_optical_frame')
                        if expected:
                            self.until(lambda: any(
                                {tf.child_frame_id for tf in msg.transforms
                                 if tf.header.frame_id == 'camera_optical_frame'
                                 and any(tf.header.stamp == frame.header.stamp for frame in detections)}
                                == {'tag36h11:0', 'tag36h11:1'} for msg in transforms))
                        self.until(lambda: diagnostic('Body-to-optical transform unavailable'))
                self.until(lambda: diagnostic('Camera stream unavailable'))

    def test_validated_stream_requires_measurement_time_transform(self):
        from ament_index_python.packages import get_package_share_directory
        from apriltag_msgs.msg import AprilTagDetectionArray
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import StaticTransformBroadcaster
        import yaml

        raw, validated = [], []
        self.node.create_subscription(AprilTagDetectionArray, '/vision/detections', raw.append, 10)
        self.node.create_subscription(AprilTagDetectionArray, '/vision/validated_detections', validated.append, 10)
        share = Path(get_package_share_directory('drone_vision'))
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'camera.yaml'
            config = yaml.safe_load((share / 'config/camera.yaml').read_text())
            config['camera_gate']['ros__parameters']['validated_tag_ids'] = [0, 1]
            path.write_text(yaml.safe_dump(config))
            with process(['ros2', 'launch', 'drone_vision', 'vision.launch.py',
                          f'camera_config:={path}', f'tag_config:={share / "config/apriltag.yaml"}']), \
                    process([self.camera]):
                self.until(lambda: len(raw) >= 3)
                self.assertFalse(validated)
                broadcaster = StaticTransformBroadcaster(self.node)
                transform = TransformStamped()
                transform.header.stamp = self.node.get_clock().now().to_msg()
                transform.header.frame_id = 'base_link'
                transform.child_frame_id = 'camera_optical_frame'
                transform.transform.rotation.w = 1.0
                broadcaster.sendTransform(transform)
                self.until(lambda: bool(validated))
                self.assertEqual({tag.id for tag in validated[-1].detections}, {0, 1})

    def test_bag_video_stop_and_playback(self):
        from ament_index_python.packages import get_package_share_directory
        from sensor_msgs.msg import CameraInfo
        import yaml
        from drone_vision.recording_store import read_metadata

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            share = Path(get_package_share_directory('drone_vision'))
            config = yaml.safe_load((share / 'config/recording.yaml').read_text())
            config.update(directory=str(root / 'sessions'), video=True, topics=['/camera/camera_info'],
                          split_mib=1, session_mib=16, total_mib=32, free_mib=1)
            path = root / 'config.yaml'
            path.write_text(yaml.safe_dump(config))
            recorder = ['ros2', 'run', 'drone_vision', 'recorder', '--config', str(path)]
            with process([self.camera]):
                self.until(lambda: self.node.count_publishers('/camera/camera_info') > 0)
                with process([*recorder, 'start', '--git-commit', 'a' * 40]) as session_process:
                    frames = lambda: list(root.glob('sessions/*/video/frames.csv'))
                    self.until(lambda: bool(frames() and len(frames()[0].read_text().splitlines()) > 12),
                               timeout=20)
                    subprocess.run([*recorder, 'stop'], check=True, timeout=10)
                    self.assertEqual(session_process.wait(timeout=20), 0)
            session, = (root / 'sessions').glob('*/metadata.json')
            session = session.parent
            metadata = read_metadata(session)
            self.assertEqual(metadata['state'], 'finished')
            self.assertEqual(metadata['git_commit'], 'a' * 40)
            self.assertTrue(metadata['end_utc'].endswith('+00:00'))
            self.assertEqual(metadata['topics'], ['/camera/camera_info'])
            video, = (session / 'video').glob('*.mkv')
            codec = subprocess.check_output(['ffprobe', '-v', 'error', '-select_streams', 'v:0',
                                             '-show_entries', 'stream=codec_name', '-of',
                                             'default=nw=1:nk=1', video], text=True)
            self.assertEqual(codec.strip(), 'h264')
            received = []
            self.node.create_subscription(CameraInfo, '/camera/camera_info', received.append, 10)
            with process(['ros2', 'bag', 'play', str(session / 'rosbag'), '--loop']):
                self.until(lambda: len(received) >= 2)

    def test_encoder_failure_interrupted_session_and_restart(self):
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        from drone_vision.recording_store import read_metadata
        import yaml

        with tempfile.TemporaryDirectory() as directory:
            work = Path(directory)
            root = work / 'sessions'
            share = Path(get_package_share_directory('drone_vision'))
            config = yaml.safe_load((share / 'config/recording.yaml').read_text())
            config.update(directory=str(root), bag=False, video=True, free_mib=1)
            config['camera']['timeout_s'] = 30
            yaml_file = work / 'recording.yaml'
            yaml_file.write_text(yaml.safe_dump(config))
            encoder = work / 'ffmpeg'
            encoder.write_text('#!/bin/sh\nif [ "$1" = "-version" ]; then echo test; exit; fi\nexit 19\n')
            encoder.chmod(0o755)
            env = dict(os.environ, PATH=f'{work}:{os.environ["PATH"]}')
            exe = Path(get_package_prefix('drone_vision')) / 'lib/drone_vision/recorder'
            recorder = [exe, '--config', yaml_file]
            start = [*recorder, 'start', '--git-commit', 'a' * 40]
            result = subprocess.run(start, env=env, timeout=12)
            self.assertEqual(result.returncode, 1)
            first, = root.glob('*/metadata.json')
            self.assertIn('Encoder exited (19)', read_metadata(first.parent)['errors'])

            ready = work / 'encoder.ready'
            encoder.write_text(f'#!/bin/sh\nif [ "$1" = "-version" ]; then echo test; exit; fi\n'
                               f'touch "{ready}"\nexec sleep 30\n')
            with process(start, env=env) as supervisor:
                self.until(ready.exists)
                supervisor.kill()
                supervisor.wait(timeout=5)
            status = subprocess.check_output([*recorder, 'status'], text=True, timeout=5)
            self.assertIn('interrupted', status)

            encoder.write_text('#!/bin/sh\nif [ "$1" = "-version" ]; then echo test; exit; fi\nexit 19\n')
            self.assertEqual(subprocess.run(start, env=env, timeout=12).returncode, 1)
            self.assertEqual(len(list(root.glob('*/metadata.json'))), 3)
