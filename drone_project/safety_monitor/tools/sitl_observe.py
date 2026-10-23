#!/usr/bin/env python3
"""Observe live SITL safety telemetry, then verify adapter-loss blocking.

Does not send flight commands, change PX4 parameters, or publish fake telemetry.
"""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.qos import qos_profile_sensor_data
from px4_interface_node.msg import DroneState
from safety_monitor.msg import SafetyStatus


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    processes = subprocess.run(['pgrep', '-af', 'px4_sitl_default/bin/px4'],
                               capture_output=True, text=True, check=False)
    if processes.returncode:
        raise RuntimeError('Start local PX4 SITL, Gazebo server, and MicroXRCEAgent first')
    prefix = '/safety_sitl_validation'
    interface_bin = Path(get_package_prefix('px4_interface_node')) / 'lib/px4_interface_node/px4_interface_node'
    monitor_bin = Path(get_package_prefix('safety_monitor')) / 'lib/safety_monitor/safety_monitor'
    rclpy.init()
    node = rclpy.create_node('safety_sitl_observer')
    states, statuses = [], []
    node.create_subscription(DroneState, prefix + '/state', states.append, qos_profile_sensor_data)
    node.create_subscription(SafetyStatus, prefix + '/safety', statuses.append, qos_profile_sensor_data)

    def until(predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise RuntimeError('SITL observation timed out; check topics, DDS, and message versions')
            rclpy.spin_once(node, timeout_sec=0.05)

    children = []
    with tempfile.TemporaryFile(mode='w+') as logs:
        try:
            interface = subprocess.Popen([
                str(interface_bin), '--ros-args', '-r', '__node:=safety_sitl_interface',
                '-r', '/drone/state:=' + prefix + '/state',
                '-r', '/drone/set_flight_mode:=' + prefix + '/set_flight_mode',
                '-r', '/fmu/in/vehicle_command:=' + prefix + '/unused_command',
                '-p', 'source_component:=193',
            ], stdout=logs, stderr=logs)
            children.append(interface)
            monitor = subprocess.Popen([
                str(monitor_bin), '--ros-args', '-r', '__node:=safety_sitl_monitor',
                '-r', '/drone/state:=' + prefix + '/state',
                '-r', '/drone/safety_status:=' + prefix + '/safety',
            ], stdout=logs, stderr=logs)
            children.append(monitor)
            until(lambda: states and statuses and statuses[-1].telemetry_healthy)
            state, status = states[-1], statuses[-1]
            # Strict policy remains active: absent RC/GCS must never be READY.
            if state.manual_control_signal_lost or state.gcs_connection_lost:
                assert not status.launch_permitted and not status.operator_link_healthy
            report = {
                'live_sitl': {
                    'level': status.level, 'reason': status.reason,
                    'telemetry_healthy': status.telemetry_healthy,
                    'launch_permitted': status.launch_permitted,
                    'failsafe_flags_fresh': state.failsafe_flags_fresh,
                    'land_detected_fresh': state.land_detected_fresh,
                    'preflight_passed': state.pre_flight_checks_pass,
                    'rc_lost': state.manual_control_signal_lost,
                    'gcs_lost': state.gcs_connection_lost,
                    'armed': state.armed, 'landed': state.landed,
                },
            }
            stop(interface)
            until(lambda: not statuses[-1].telemetry_healthy, timeout=3)
            lost = statuses[-1]
            assert lost.level in (SafetyStatus.BLOCKED, SafetyStatus.EMERGENCY)
            assert not any((lost.launch_permitted, lost.autonomous_flight_permitted,
                            lost.visual_landing_permitted))
            report['adapter_loss'] = {'level': lost.level, 'reason': lost.reason,
                                      'all_permissions_denied': True}
            encoded = json.dumps(report, indent=2)
            if args.report:
                args.report.write_text(encoded + '\n', encoding='utf-8')
            print(encoded)
        except Exception:
            logs.seek(0)
            print(logs.read()[-5000:])
            raise
        finally:
            for child in reversed(children):
                stop(child)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
