#!/usr/bin/env python3
"""Read-only SITL uORB observation; never synthesize PX4 telemetry."""
import argparse
import os
from pathlib import Path
import re
import subprocess
import time


def parse_native(output):
    result = {}
    for name in ['timestamp', 'armed', 'lockdown', 'kill', 'termination']:
        match = re.search(r'^\s*' + name + r':\s*(\d+|True|False|true|false)\b', output, re.M)
        if not match:
            raise ValueError('Missing native field: ' + name)
        value = match.group(1)
        result[name] = int(value) if value.isdigit() else int(value.lower() == 'true')
    return result


def main():
    import rclpy
    from state_machine_manager.msg import NativeLaunchStatus
    parser = argparse.ArgumentParser()
    parser.add_argument('--px4-bin', type=Path, required=True)
    args = parser.parse_args()
    # Symlink clients talk only to the local POSIX PX4 daemon. No hardware path,
    # MAVLink endpoint, or parameter write is accepted by this helper.
    binary = args.px4_bin.resolve()
    if binary.name != 'bin' or not (binary/'px4-listener').is_file():
        raise SystemExit('Expected installed PX4 SITL build bin directory')
    rclpy.init()
    node = rclpy.create_node('sitl_native_launch_observer')
    pub = node.create_publisher(NativeLaunchStatus, '/simulation/native_launch_status', 1)
    last = 0
    while rclpy.ok():
        try:
            enabled = subprocess.run([str(binary/'px4-param'), 'show', '-q', 'COM_THROW_EN'],
                                     capture_output=True, text=True, timeout=1, check=True)
            if enabled.stdout.strip() != '1':
                raise ValueError('COM_THROW_EN is not 1')
            result = subprocess.run([str(binary/'px4-listener'), 'actuator_armed', '-n', '1'],
                                    capture_output=True, text=True, timeout=1, check=True)
            data = parse_native(result.stdout)
            if data['timestamp'] > last:
                message = NativeLaunchStatus()
                message.px4_timestamp = data['timestamp']
                for field in ['armed', 'lockdown', 'kill', 'termination']:
                    setattr(message, field, bool(data[field]))
                pub.publish(message)
                last = data['timestamp']
        except (subprocess.SubprocessError, ValueError) as error:
            node.get_logger().warning(str(error))
        rclpy.spin_once(node, timeout_sec=0)
        time.sleep(0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try: main()
    except KeyboardInterrupt: pass
