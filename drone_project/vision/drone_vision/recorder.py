"""Explicitly supervised, noncritical bag and camera recording."""
import argparse
from collections import deque
import fcntl
import os
from pathlib import Path
import platform
import re
import shutil
import signal
import subprocess
import sys
import time

import yaml

from .recording_store import (MIB, atomic_json, attach_ulog, hash_file, new_session,
                              read_metadata, selected_session, storage_status, utc_now)


def load_config(path):
    config = yaml.safe_load(path.read_text())
    if any(type(config[key]) is not bool for key in ('bag', 'video')):
        raise ValueError('bag and video must be YAML booleans')
    for name in ('split_mib', 'session_mib', 'total_mib', 'free_mib'):
        if type(config[name]) is not int or config[name] <= 0:
            raise ValueError(f'{name} must be a positive integer')
    if not config['split_mib'] <= config['session_mib'] <= config['total_mib']:
        raise ValueError('Expected split_mib <= session_mib <= total_mib')
    if (not isinstance(config['topics'], list) or not all(
            isinstance(topic, str) and re.fullmatch(r'/[A-Za-z0-9_/]+', topic)
            for topic in config['topics'])):
        raise ValueError('Use explicit absolute topic names')
    camera = config['camera']
    if not isinstance(camera['topic'], str) or not re.fullmatch(r'/[A-Za-z0-9_/]+', camera['topic']):
        raise ValueError('Camera video topic must be an absolute topic name')
    for name in ('width', 'height', 'fps', 'bitrate_kbps', 'timeout_s'):
        if type(camera[name]) not in (int, float) or not 0 < camera[name] < 100000:
            raise ValueError(f'Invalid camera {name}')
    if any(type(camera[key]) is not int or camera[key] % 2 for key in ('width', 'height')):
        raise ValueError('Camera width and height must be even integers')
    if camera['encoder'] not in ('libx264', 'h264_v4l2m2m'):
        raise ValueError('Select libx264 or an operator-verified h264_v4l2m2m encoder')
    return config


def command_output(args):
    try:
        return subprocess.check_output(args, stderr=subprocess.DEVNULL, text=True, timeout=4).strip()
    except (OSError, subprocess.SubprocessError):
        return 'unavailable'


def ffmpeg_command(directory, camera):
    codec = camera['encoder']
    preset = ['-preset', 'ultrafast'] if codec == 'libx264' else []
    return [
        'ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'warning',
        '-f', 'image2pipe', '-framerate', str(camera['fps']), '-c:v', 'mjpeg', '-i', 'pipe:0',
        '-an', '-vf', f'scale={camera["width"]}:{camera["height"]}',
        '-c:v', codec, *preset, '-pix_fmt', 'yuv420p',
        '-b:v', f'{camera["bitrate_kbps"]}k', '-g', str(max(1, round(camera['fps'] * 2))),
        '-f', 'segment', '-segment_time', '10', '-reset_timestamps', '1',
        str(directory / 'camera_%05d.mkv'),
    ]


def spawn_media(command, **options):
    return subprocess.Popen(
        [sys.executable, '-m', 'drone_vision.media_process', str(os.getpid()), *command],
        start_new_session=True, **options)


class Video:
    def __init__(self, session, camera, log):
        self.camera = camera
        directory = session / 'video'
        directory.mkdir()
        self.timestamps = (directory / 'frames.csv').open('w', buffering=1)
        self.timestamps.write('input_frame,source_ros_ns,received_monotonic_ns\n')
        try:
            self.process = spawn_media(ffmpeg_command(directory, camera),
                                       stdin=subprocess.PIPE, bufsize=0, stdout=log, stderr=log)
        except OSError:
            self.timestamps.close()
            raise
        os.set_blocking(self.process.stdin.fileno(), False)
        self.queue = deque(maxlen=1)
        self.pending = None
        self.last_received = time.monotonic()
        self.last_queued = 0.0
        self.last_written = self.last_received
        self.index = 0

    def receive(self, image):
        now = time.monotonic()
        if ('jpeg' not in image.format.lower() and 'jpg' not in image.format.lower()) or not image.data:
            return
        # Bound the buffer even when the encoder stalls or the source bursts.
        self.last_received = now
        if len(image.data) > 8 * MIB or now - self.last_queued < 0.95 / self.camera['fps']:
            return
        stamp = image.header.stamp
        self.queue.append((bytes(image.data), stamp.sec * 10**9 + stamp.nanosec, time.monotonic_ns()))
        self.last_queued = now

    def pump(self):
        if self.pending is None and self.queue:
            data, self.stamp, self.received = self.queue.popleft()
            self.pending = memoryview(data)
        if self.pending is None:
            return
        try:
            count = os.write(self.process.stdin.fileno(), self.pending)
        except BlockingIOError:
            return
        self.pending = self.pending[count:]
        self.last_written = time.monotonic()
        if not self.pending:
            self.timestamps.write(f'{self.index},{self.stamp},{self.received}\n')
            self.index += 1
            self.pending = None

    def close(self):
        self.process.stdin.close()
        self.timestamps.close()
        try:
            self.process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            stop_process(self.process)


def stop_process(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=3)


def start(root, config, commit):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage

    if not config['bag'] and not config['video']:
        raise ValueError('Enable bag or video in the recording configuration')
    status = storage_status(root, None, config)
    if status['limits_reached']:
        raise ValueError(f"Storage limits reached: {status['limits_reached']}")
    if not re.fullmatch(r'[0-9a-f]{40}', commit):
        raise ValueError('Supply the full deployed Git commit with --git-commit')
    session = new_session(root)
    metadata = {
        'session': session.name, 'state': 'running', 'start_utc': utc_now(), 'end_utc': None,
        'git_commit': commit, 'ros_distribution': os.environ.get('ROS_DISTRO', 'unknown'),
        'device': {'machine': platform.machine(), 'kernel': platform.release(), 'host': platform.node()},
        'software': {'ros_packages': command_output(['dpkg-query', '-W', 'ros-humble-ros2bag',
                                                    'ros-humble-apriltag-ros']),
                     'ffmpeg': command_output(['ffmpeg', '-version']).splitlines()[0]},
        'camera': config['camera'], 'topics': [], 'unavailable_topics': [], 'errors': [],
    }
    atomic_json(session / 'metadata.json', metadata)
    (root / '.active').write_text(session.name)
    print(f'Recording session {session}', flush=True)
    bag = video = subscription = node = None
    ending = False

    def finish(*_):
        nonlocal ending
        ending = True

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, finish)
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('drone_recording_supervisor')
        with (session / 'logs/rosbag.log').open('w') as bag_log, \
                (session / 'logs/video.log').open('w') as video_log:
            until = time.monotonic() + 2
            while config['bag'] and not ending and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.1)
            if config['bag'] and not ending:
                requested = list(dict.fromkeys(config['topics']))
                metadata['topics'] = [topic for topic in requested if node.count_publishers(topic)]
                metadata['unavailable_topics'] = [topic for topic in requested
                                                  if topic not in metadata['topics']]
                if metadata['topics']:
                    try:
                        bag = spawn_media([
                            'ros2', 'bag', 'record', '--storage', 'sqlite3',
                            '--max-bag-size', str(config['split_mib'] * MIB),
                            '--output', str(session / 'rosbag'), *metadata['topics'],
                        ], stdout=bag_log, stderr=bag_log)
                    except OSError as exc:
                        metadata['errors'].append(f'Bag startup failed: {exc}')
                else:
                    metadata['errors'].append('No configured bag topics had publishers')
            if config['video'] and not ending:
                try:
                    video = Video(session, config['camera'], video_log)
                    subscription = node.create_subscription(
                        CompressedImage, config['camera']['topic'], video.receive, qos_profile_sensor_data)
                except OSError as exc:
                    metadata['errors'].append(f'Video startup failed: {exc}')
            atomic_json(session / 'metadata.json', metadata)
            next_storage = 0.0
            warned = set()
            while (bag is not None or video is not None) and not ending and not (session / '.stop').exists():
                rclpy.spin_once(node, timeout_sec=0.04)
                now = time.monotonic()
                if bag and bag.poll() is not None:
                    metadata['errors'].append(f'Bag exited during recording ({bag.returncode})')
                    bag = None
                if video:
                    reason = ''
                    if video.process.poll() is not None:
                        reason = f'Encoder exited ({video.process.returncode})'
                    elif now - video.last_received > video.camera['timeout_s']:
                        reason = 'Compressed camera stream disconnected'
                    else:
                        try:
                            video.pump()
                        except OSError as exc:
                            reason = f'Encoder pipe failed: {exc}'
                        if video.pending and now - video.last_written > 3:
                            reason = 'Encoder pipe stalled'
                    if reason:
                        metadata['errors'].append(reason)
                        node.destroy_subscription(subscription)
                        video.close()
                        video = None
                        atomic_json(session / 'metadata.json', metadata)
                if now >= next_storage:
                    next_storage = now + 0.5
                    status = storage_status(root, session, config)
                    for limit in set(status['near_limits']) - warned:
                        print(f'WARNING recording approaches {limit} limit', file=sys.stderr, flush=True)
                        warned.add(limit)
                    if status['limits_reached']:
                        metadata['errors'].append(f"Storage limits reached: {status['limits_reached']}")
                        break
    except Exception as exc:
        metadata['errors'].append(str(exc))
    finally:
        for resource, close in ((bag, stop_process), (video, lambda stream: stream.close())):
            if resource is not None:
                try:
                    close(resource)
                    if resource is video and video.process.returncode:
                        metadata['errors'].append(f'Encoder shutdown code {video.process.returncode}')
                    if resource is bag and not (session / 'rosbag/metadata.yaml').is_file():
                        metadata['errors'].append('Bag shutdown did not produce metadata.yaml')
                except (OSError, subprocess.SubprocessError) as exc:
                    metadata['errors'].append(f'Media shutdown failed: {exc}')
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
        metadata.update(state='finished_with_errors' if metadata['errors'] else 'finished',
                        end_utc=utc_now(), storage=storage_status(root, session, config))
        try:
            atomic_json(session / 'metadata.json', metadata)
        finally:
            (root / '.active').unlink(missing_ok=True)
        for error in metadata['errors']:
            print(f'ERROR {error}', file=sys.stderr)
    return int(bool(metadata['errors']))


def main():
    from ament_index_python.packages import get_package_share_directory

    default = Path(get_package_share_directory('drone_vision')) / 'config/recording.yaml'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=default)
    commands = parser.add_subparsers(dest='action', required=True)
    record = commands.add_parser('start')
    record.add_argument('--git-commit', required=True)
    commands.add_parser('stop')
    commands.add_parser('status')
    attach = commands.add_parser('attach-ulog')
    attach.add_argument('session')
    attach.add_argument('source', type=Path)
    manifest = commands.add_parser('manifest')
    manifest.add_argument('session')
    args = parser.parse_args()
    try:
        config = load_config(args.config)
        root = Path(config['directory']).expanduser().resolve()
        root.mkdir(mode=0o750, parents=True, exist_ok=True)
        with (root / '.lock').open('a') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                running = False
            except BlockingIOError:
                running = True
            if args.action == 'start':
                if running:
                    raise ValueError('A session is already recording in this directory')
                return start(root, config, args.git_commit)
            if args.action == 'status':
                print(storage_status(root, None, config))
                for file in sorted(root.glob('*/metadata.json')):
                    try:
                        session = read_metadata(file.parent)
                    except (OSError, ValueError):
                        print(f'{file.parent.name} metadata_unreadable')
                        continue
                    active = running and (root / '.active').exists() and (root / '.active').read_text() == file.parent.name
                    state = session['state'] if active or session['state'] != 'running' else 'interrupted'
                    print(f'{file.parent.name} {state} {session["start_utc"]}')
            elif args.action == 'stop':
                if not running or not (root / '.active').exists():
                    raise ValueError('No active recorder')
                selected_session(root, (root / '.active').read_text()).joinpath('.stop').touch()
            else:
                if running:
                    raise ValueError('Stop recording before retrieving or attaching files')
                session = selected_session(root, args.session)
                if args.action == 'attach-ulog':
                    status = storage_status(root, session, config)
                    size = args.source.stat().st_size
                    if (status['free_bytes'] - size < config['free_mib'] * MIB
                            or status['used_bytes'] + size > config['total_mib'] * MIB
                            or status['session_bytes'] + size > config['session_mib'] * MIB):
                        raise ValueError('ULog would exceed recording storage limits')
                    print(attach_ulog(session, args.source))
                else:
                    for file in sorted(session.rglob('*')):
                        if file.is_file() and not file.is_symlink():
                            print(f'{hash_file(file)}  {file.relative_to(session)}')
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        parser.exit(1, f'ERROR {exc}\n')
    return 0
