#!/usr/bin/env python3
"""Verify the command API against an already running, landed PX4 SITL.

Starts an isolated interface instance, changes LOITER -> LAND -> LOITER while
disarmed, and optionally tests a guarded arm/disarm without takeoff. A live GCS
link must remain connected. Never use this test against a physical vehicle.
"""

import argparse
import json
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node

from px4_interface_node.msg import DroneState
from px4_interface_node.srv import ExecuteCommand


def local_sitl_running():
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            executable = path.read_bytes().split(b'\0')[0]
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        if executable.endswith(b'/px4_sitl_default/bin/px4'):
            return True
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--test-arming', action='store_true')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    # An explicit local SITL process is required before publishing any command.
    if not local_sitl_running():
        raise RuntimeError('No local px4_sitl_default process found')
    executable = Path(get_package_prefix('px4_interface_node')) / 'lib/px4_interface_node/px4_interface_node'
    interface = subprocess.Popen([
        str(executable), '--ros-args', '-r', '__node:=phase3_sitl_interface',
        '-r', '/drone/state:=/phase3/drone/state',
        '-r', '/drone/execute_command:=/phase3/drone/execute_command',
        '-r', '/drone/set_flight_mode:=/phase3/drone/set_flight_mode',
        '-p', 'source_component:=192',
    ])
    rclpy.init()
    node = Node('phase3_sitl_test_client')
    state = None
    last_state_rx = 0.0
    results = []
    arm_requested = False
    mode_requested = False

    def receive(message):
        nonlocal state, last_state_rx
        state = message
        last_state_rx = time.monotonic()

    subscription = node.create_subscription(DroneState, '/phase3/drone/state', receive, 10)
    client = node.create_client(ExecuteCommand, '/phase3/drone/execute_command')

    def until(predicate, timeout=15.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError('SITL state/service deadline exceeded')
            rclpy.spin_once(node, timeout_sec=0.05)

    def execute(command, mode=''):
        future = client.call_async(ExecuteCommand.Request(command=command, mode=mode))
        until(future.done)
        result = future.result()
        record = {
            'request_id': result.request_id, 'command': command, 'mode': mode,
            'success': result.success, 'accepted': result.accepted,
            'result': result.result, 'ack_result': result.ack_result,
            'current_mode': result.current_mode, 'armed': result.armed,
            'status_fresh': result.status_fresh, 'message': result.message,
        }
        results.append(record)
        print(json.dumps(record), flush=True)
        if not result.success or not result.accepted or not result.status_fresh:
            raise RuntimeError(f'PX4 command verification failed: {record}')
        return result

    def on_ground():
        return (state and time.monotonic() - last_state_rx <= 2.0 and
                state.status_fresh and state.land_detected_fresh and state.landed)

    try:
        if not client.wait_for_service(timeout_sec=10):
            raise TimeoutError('Interface command service did not start')
        until(lambda: on_ground() and not state.armed)
        if state.flight_mode != 'LOITER':
            raise RuntimeError('Start this test in disarmed LOITER/Hold mode')
        mode_requested = True
        execute(ExecuteCommand.Request.SET_MODE, 'LAND')
        execute(ExecuteCommand.Request.SET_MODE, 'LOITER')
        until(lambda: on_ground() and not state.armed and state.flight_mode == 'LOITER')
        if args.test_arming:
            until(lambda: on_ground() and state.ready_to_arm and
                  state.pre_flight_checks_pass and not state.gcs_connection_lost and not state.failsafe)
            execute(ExecuteCommand.Request.RUN_PREFLIGHT_CHECKS)
            arm_requested = True
            execute(ExecuteCommand.Request.ARM)
            execute(ExecuteCommand.Request.DISARM)
            until(lambda: on_ground() and not state.armed)
            arm_requested = False
        if args.report:
            args.report.write_text(json.dumps(results, indent=2) + '\n')
        print('SITL verification passed; vehicle is disarmed in LOITER.', flush=True)
    finally:
        # Cleanup follows the same landed/fresh-telemetry policy as the API.
        # It never sends a force command or bypasses an unknown airborne state.
        if arm_requested and on_ground():
            try:
                until(lambda: state and not state.command_pending)
                execute(ExecuteCommand.Request.DISARM)
            except Exception as error:
                print(f'Disarm cleanup failed; inspect SITL immediately: {error}', flush=True)
        if mode_requested and on_ground() and not state.armed and state.flight_mode != 'LOITER':
            try:
                execute(ExecuteCommand.Request.SET_MODE, 'LOITER')
            except Exception as error:
                print(f'Mode cleanup failed: {error}', flush=True)
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
        interface.send_signal(signal.SIGINT)
        try:
            interface.wait(timeout=10)
        except subprocess.TimeoutExpired:
            interface.kill()
            interface.wait()


if __name__ == '__main__':
    main()
