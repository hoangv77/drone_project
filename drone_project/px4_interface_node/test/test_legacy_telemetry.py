"""Real legacy node with isolated DDS peers; no physical controller required."""
import math
from pathlib import Path
import subprocess
import time
import unittest

from ament_index_python.packages import get_package_prefix

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from px4_interface_node.msg import DroneState
from px4_interface_node.srv import ExecuteCommand, SetFlightMode
from px4_msgs.msg import FailsafeFlags, SensorGps, VehicleLocalPosition, VehicleOdometry, VehicleStatus

P = '/test_legacy_px4'
RAW = {
    '/fmu/out/vehicle_status': VehicleStatus,
    '/fmu/out/vehicle_local_position': VehicleLocalPosition,
    '/fmu/out/vehicle_odometry': VehicleOdometry,
    '/fmu/out/vehicle_gps_position': SensorGps,
    '/fmu/out/failsafe_flags': FailsafeFlags,
}


@pytest.mark.launch_test
def generate_test_description():
    endpoints = list(RAW) + ['/drone/state', '/drone/execute_command', '/drone/set_flight_mode',
                             '/fmu/in/vehicle_command', '/fmu/out/battery_status',
                             '/fmu/out/vehicle_land_detected', '/fmu/out/vehicle_command_ack']
    node = launch_ros.actions.Node(
        package='px4_interface_node', executable='px4_interface_node', name='legacy_interface_test',
        parameters=[{'compatibility_profile': 'legacy_1_14_passive',
                     'read_only': False, 'sitl_commands': True}],
        remappings=[(name, P + name) for name in endpoints], output='screen')
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class TestLegacyTelemetry(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('legacy_px4_peer')
        cls.pubs = {name: cls.node.create_publisher(kind, P + name, qos_profile_sensor_data)
                    for name, kind in RAW.items()}
        cls.states = []
        cls.sub = cls.node.create_subscription(DroneState, P + '/drone/state', cls.states.append, 10)
        cls.command = cls.node.create_client(ExecuteCommand, P + '/drone/execute_command')
        cls.mode = cls.node.create_client(SetFlightMode, P + '/drone/set_flight_mode')
        assert cls.command.wait_for_service(timeout_sec=10)
        assert cls.mode.wait_for_service(timeout_sec=10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def until(self, predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.assertLess(time.monotonic(), deadline, 'DDS condition timed out')
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def call(self, client, request):
        future = client.call_async(request)
        self.until(future.done)
        return future.result()

    def test_runtime_profile_mismatch_exits(self):
        executable = Path(get_package_prefix('px4_interface_node')) / 'lib/px4_interface_node/px4_interface_node'
        result = subprocess.run([
            'bash', '-c', 'ulimit -c 0; exec "$@"', 'profile_mismatch', str(executable),
            '--ros-args', '-r', '__node:=legacy_mismatch', '-p', 'compatibility_profile:=modern_px4',
        ], capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('differs from compiled firmware contract', result.stderr)
        self.assertEqual(self.node.count_publishers(P + '/fmu/in/vehicle_command'), 0)

    def test_contract(self):
        self.until(lambda: self.states and all(p.get_subscription_count() == 1 for p in self.pubs.values()))
        self.assertFalse(self.states[-1].status_fresh)
        get = self.node.create_client(GetParameters, '/legacy_interface_test/get_parameters')
        self.assertTrue(get.wait_for_service(timeout_sec=5))
        params = self.call(get, GetParameters.Request(names=['read_only', 'compatibility_profile']))
        self.assertTrue(params.values[0].bool_value)
        self.assertEqual(params.values[1].string_value, 'legacy_1_14_passive')
        setter = self.node.create_client(SetParameters, '/legacy_interface_test/set_parameters')
        self.assertTrue(setter.wait_for_service(timeout_sec=5))
        for name, value in [('read_only', False), ('compatibility_profile', 'modern_px4')]:
            result = self.call(setter, SetParameters.Request(parameters=[
                Parameter(name, value=value).to_parameter_msg()]))
            self.assertFalse(result.results[0].successful)

        # Rejection is independent of telemetry, including every valid command.
        def rejected():
            for kind in [ExecuteCommand.Request.ARM, ExecuteCommand.Request.DISARM,
                         ExecuteCommand.Request.RUN_PREFLIGHT_CHECKS, ExecuteCommand.Request.SET_MODE, 255]:
                response = self.call(self.command, ExecuteCommand.Request(
                    command=kind, mode='TAKEOFF' if kind == ExecuteCommand.Request.SET_MODE else ''))
                self.assertFalse(response.success or response.accepted)
                self.assertEqual(response.result, ExecuteCommand.Response.REJECTED)
                self.assertIn('read-only', response.message)
            for mode in ['TAKEOFF', 'LAND', 'LOITER']:
                response = self.call(self.mode, SetFlightMode.Request(mode=mode))
                self.assertFalse(response.success or response.accepted)
                self.assertIn('read-only', response.message)
            self.assertEqual(self.node.count_publishers(P + '/fmu/in/vehicle_command'), 0)

        rejected()
        messages = {name: kind() for name, kind in RAW.items()}
        status = messages['/fmu/out/vehicle_status']
        status.system_id = status.component_id = 1
        status.arming_state = VehicleStatus.ARMING_STATE_ARMED
        status.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
        status.nav_state_user_intention = VehicleStatus.NAVIGATION_STATE_MANUAL
        status.pre_flight_checks_pass = status.safety_off = True
        local = messages['/fmu/out/vehicle_local_position']
        local.xy_valid = local.z_valid = local.v_xy_valid = local.v_z_valid = True
        local.x, local.y, local.z = 1.0, 2.0, -3.0
        local.vx, local.vy, local.vz = 0.1, 0.2, -0.3
        gps = messages['/fmu/out/vehicle_gps_position']
        gps.fix_type, gps.lat, gps.lon = 3, 410000000, -870000000
        flags = messages['/fmu/out/failsafe_flags']
        flags.primary_geofence_breached = True
        flags.battery_warning = 2

        def publish():
            for name, message in messages.items():
                message.timestamp = time.time_ns() // 1000
                self.pubs[name].publish(message)
            self.spin(0.04)

        for _ in range(8):
            publish()
        self.until(lambda: self.states[-1].position_valid and self.states[-1].gps_valid)
        state = self.states[-1]
        self.assertTrue(state.status_fresh and state.odometry_fresh and state.failsafe_flags_fresh)
        self.assertTrue(state.armed)
        self.assertEqual(state.flight_mode, 'LOITER')
        self.assertEqual(state.nav_state_user_intention, VehicleStatus.NAVIGATION_STATE_MANUAL)
        self.assertEqual((state.x, state.y, state.z), (1.0, 2.0, -3.0))
        self.assertAlmostEqual(state.vz, -0.3)
        self.assertTrue(state.geofence_breached)
        self.assertEqual(state.failsafe_battery_warning, 2)
        self.assertFalse(state.battery_fresh or state.land_detected_fresh or state.ready_to_arm)
        self.assertTrue(math.isnan(state.battery_remaining))
        self.assertIn('read-only', state.arming_block_reason)
        for name in ['battery_status', 'vehicle_land_detected', 'vehicle_command_ack']:
            self.assertEqual(self.node.count_subscribers(P + '/fmu/out/' + name), 0)
        rejected()

        # Wrong identity, zero timestamps and replays cannot change cached state.
        status.arming_state = VehicleStatus.ARMING_STATE_STANDBY
        status.system_id = 2
        status.timestamp = time.time_ns() // 1000
        self.pubs['/fmu/out/vehicle_status'].publish(status)
        status.system_id = 1
        status.timestamp = 0
        self.pubs['/fmu/out/vehicle_status'].publish(status)
        local.timestamp = 1
        local.x = 99.0
        self.pubs['/fmu/out/vehicle_local_position'].publish(local)
        self.spin(0.2)
        self.assertTrue(self.states[-1].armed)
        self.assertEqual(self.states[-1].x, 1.0)

        # Publishing cached timestamps repeatedly must not refresh the stream.
        deadline = time.monotonic() + 2.5
        while time.monotonic() < deadline:
            for name, message in messages.items():
                self.pubs[name].publish(message)
            self.spin(0.05)
        stale = self.states[-1]
        for field in ['status', 'local_position', 'odometry', 'gps', 'failsafe_flags', 'battery', 'land_detected']:
            self.assertFalse(getattr(stale, field + '_fresh'), field)
        self.assertFalse(stale.position_valid or stale.gps_valid or stale.ready_to_arm)
        self.assertTrue(math.isnan(stale.vx) and math.isnan(stale.battery_remaining))
        rejected()

        # Invalid integer GPS coordinates cannot become a valid fix.
        gps.lat = 1000000000
        for _ in range(5):
            publish()
        self.assertFalse(self.states[-1].gps_valid)
        self.assertTrue(self.states[-1].gps_fresh)

        # Multiple publishers invalidate the stream until a single source resumes.
        duplicate = self.node.create_publisher(VehicleStatus, P + '/fmu/out/vehicle_status', qos_profile_sensor_data)
        self.until(lambda: duplicate.get_subscription_count() == 1)
        for _ in range(8):
            publish()
        self.until(lambda: not self.states[-1].status_fresh)
        self.node.destroy_publisher(duplicate)
        for _ in range(8):
            publish()
        self.until(lambda: self.states[-1].status_fresh)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
