"""Exercise the real node over DDS with an isolated, simulated PX4 endpoint."""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters

from px4_interface_node.msg import DroneState
from px4_interface_node.srv import ExecuteCommand, SetFlightMode
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleLandDetected, VehicleStatus
try:
    from px4_msgs.msg import SensorGps as GpsMessage
    LEGACY_GPS = True
except ImportError:
    from px4_msgs.msg import VehicleGnss as GpsMessage
    LEGACY_GPS = False


PREFIX = '/test_px4_interface'
STATUS = PREFIX + '/fmu/out/vehicle_status_v4'
ACK = PREFIX + '/fmu/out/vehicle_command_ack_v1'
COMMAND = PREFIX + '/fmu/in/vehicle_command'
STATE = PREFIX + '/drone/state'
SERVICE = PREFIX + '/drone/set_flight_mode'
COMMAND_SERVICE = PREFIX + '/drone/execute_command'
LAND = PREFIX + '/fmu/out/vehicle_land_detected'
GPS = PREFIX + ('/fmu/out/vehicle_gps_position' if LEGACY_GPS else '/fmu/out/vehicle_gnss')
R = SetFlightMode.Response
A = VehicleCommandAck
V = VehicleStatus
E = ExecuteCommand.Response
Q = ExecuteCommand.Request


@pytest.mark.launch_test
def generate_test_description():
    # Never send test commands to the real /fmu/in endpoint, even if SITL runs.
    topics = [
        '/fmu/out/vehicle_status_v4', '/fmu/out/vehicle_local_position_v1',
        '/fmu/out/vehicle_odometry', '/fmu/out/vehicle_gps_position',
        '/fmu/out/vehicle_gnss',
        '/fmu/out/vehicle_land_detected', '/drone/execute_command',
        '/fmu/out/battery_status_v1', '/fmu/out/vehicle_command_ack_v1',
        '/fmu/in/vehicle_command', '/drone/state', '/drone/set_flight_mode',
    ]
    interface = launch_ros.actions.Node(
        package='px4_interface_node', executable='px4_interface_node',
        name='px4_interface_under_test',
        parameters=[{
            'target_system': 7, 'target_component': 1,
            'source_system': 42, 'source_component': 190,
            'ack_timeout_ms': 700, 'mode_timeout_ms': 700,
        }],
        remappings=[(topic, PREFIX + topic) for topic in topics],
        output='screen',
    )
    return launch.LaunchDescription([
        interface, launch_testing.actions.ReadyToTest(),
    ]), {'interface': interface}


class TestFlightMode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('flight_mode_test_client')
        cls.client = cls.node.create_client(SetFlightMode, SERVICE)
        cls.command_client = cls.node.create_client(ExecuteCommand, COMMAND_SERVICE)
        cls.land_pub = cls.node.create_publisher(VehicleLandDetected, LAND, qos_profile_sensor_data)
        cls.status_pub = cls.node.create_publisher(V, STATUS, qos_profile_sensor_data)
        cls.ack_pub = cls.node.create_publisher(A, ACK, qos_profile_sensor_data)
        cls.gps_pub = cls.node.create_publisher(GpsMessage, GPS, qos_profile_sensor_data)
        cls.commands = []
        cls.states = []
        cls.command_sub = cls.node.create_subscription(
            VehicleCommand, COMMAND, cls.commands.append, qos_profile_sensor_data)
        cls.state_sub = cls.node.create_subscription(
            DroneState, STATE, cls.states.append, 10)
        cls.status_enabled = False
        cls.status_mode = V.NAVIGATION_STATE_MANUAL
        cls.status_armed = False
        cls.status_intention = None
        cls.preflight = True
        cls.gcs_lost = False
        cls.failsafe = False
        cls.safety_off = True
        cls.calibration = False
        cls.landed = True
        cls.land_enabled = True
        # Mock PX4 is driven by elapsed time, not adjustable WSL/ROS wall time.
        cls.timer = cls.node.create_timer(
            0.03, cls.status_tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        assert cls.client.wait_for_service(timeout_sec=10.0)
        assert cls.command_client.wait_for_service(timeout_sec=10.0)
        deadline = time.monotonic() + 10.0
        while (cls.status_pub.get_subscription_count() == 0 or
               cls.ack_pub.get_subscription_count() == 0 or
               cls.gps_pub.get_subscription_count() == 0 or
               cls.land_pub.get_subscription_count() == 0 or
               cls.node.count_publishers(COMMAND) == 0):
            assert time.monotonic() < deadline, 'DDS discovery timed out'
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        type(self).status_enabled = False
        type(self).status_mode = V.NAVIGATION_STATE_MANUAL
        type(self).status_armed = False
        type(self).status_intention = None
        type(self).preflight = True
        type(self).gcs_lost = False
        type(self).failsafe = False
        type(self).safety_off = True
        type(self).calibration = False
        type(self).landed = True
        type(self).land_enabled = True

    @staticmethod
    def px4_time():
        return time.monotonic_ns() // 1000

    @classmethod
    def publish_status(cls, mode, timestamp=None, system=7, component=1):
        msg = V()
        msg.timestamp = cls.px4_time() if timestamp is None else timestamp
        msg.nav_state = mode
        msg.system_id = system
        msg.component_id = component
        msg.arming_state = V.ARMING_STATE_ARMED if cls.status_armed else V.ARMING_STATE_DISARMED
        msg.nav_state_user_intention = mode if cls.status_intention is None else cls.status_intention
        msg.pre_flight_checks_pass = cls.preflight
        msg.gcs_connection_lost = cls.gcs_lost
        msg.failsafe = cls.failsafe
        msg.safety_off = cls.safety_off
        msg.calibration_enabled = cls.calibration
        cls.status_pub.publish(msg)

    @classmethod
    def status_tick(cls):
        if cls.status_enabled:
            cls.publish_status(cls.status_mode)
            if cls.land_enabled:
                msg = VehicleLandDetected()
                msg.timestamp = cls.px4_time()
                msg.landed = cls.landed
                cls.land_pub.publish(msg)

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def until(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.assertLess(time.monotonic(), deadline, 'Condition timed out')
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def prepare_status(self, mode=V.NAVIGATION_STATE_MANUAL):
        count = len(self.states)
        type(self).status_mode = mode
        type(self).status_enabled = True
        expected_mode = 'LOITER' if mode == V.NAVIGATION_STATE_AUTO_LOITER else 'MANUAL'
        self.until(lambda: len(self.states) > count and self.states[-1].status_fresh and
                   self.states[-1].flight_mode == expected_mode and
                   self.states[-1].armed == self.status_armed and
                   self.states[-1].pre_flight_checks_pass == self.preflight and
                   self.states[-1].gcs_connection_lost == self.gcs_lost and
                   self.states[-1].failsafe == self.failsafe and
                   self.states[-1].safety_off == self.safety_off and
                   self.states[-1].calibration_enabled == self.calibration and
                   self.states[-1].nav_state_user_intention ==
                   (mode if self.status_intention is None else self.status_intention) and
                   (not self.land_enabled or (self.states[-1].land_detected_fresh and
                                              self.states[-1].landed == self.landed)))

    def request(self, mode='LOITER', expect_command=True):
        count = len(self.commands)
        future = self.client.call_async(SetFlightMode.Request(mode=mode))
        if expect_command:
            self.until(lambda: len(self.commands) > count)
        return future

    def ack(self, result=A.VEHICLE_CMD_RESULT_ACCEPTED, **overrides):
        msg = A()
        msg.timestamp = self.px4_time()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.target_system = 42
        msg.target_component = 190
        msg.result = result
        for key, value in overrides.items():
            setattr(msg, key, value)
        self.ack_pub.publish(msg)

    def execute(self, kind, mode='', expect_command=True):
        count = len(self.commands)
        future = self.command_client.call_async(Q(command=kind, mode=mode))
        if expect_command:
            self.until(lambda: len(self.commands) > count)
        return future

    def response(self, future, result, accepted=False, ack_result=R.NO_ACK):
        self.until(future.done)
        response = future.result()
        self.assertEqual(response.result, result, response.message)
        self.assertEqual(response.success, result == R.SUCCESS)
        self.assertEqual(response.accepted, accepted)
        self.assertEqual(response.ack_result, ack_result)
        return response

    def test_00_missing_status(self):
        count = len(self.commands)
        self.response(self.request(expect_command=False), R.STATUS_UNAVAILABLE)
        self.assertEqual(len(self.commands), count)

    def test_01_success_requires_ack_and_actual_mode(self):
        self.prepare_status()
        future = self.request()
        state_count = len(self.states)
        self.ack()
        self.spin_for(0.15)
        self.assertFalse(future.done(), 'ACCEPTED alone is not mode entry')
        self.assertGreater(len(self.states), state_count, 'Telemetry must keep publishing')
        type(self).status_mode = V.NAVIGATION_STATE_AUTO_LOITER
        response = self.response(future, R.SUCCESS, True, A.VEHICLE_CMD_RESULT_ACCEPTED)
        self.assertEqual(response.current_mode, 'LOITER')
        self.assertTrue(response.status_fresh)

    def test_02_status_can_arrive_before_ack(self):
        self.prepare_status()
        type(self).status_enabled = False
        future = self.request()
        timestamp = self.px4_time()
        self.publish_status(V.NAVIGATION_STATE_AUTO_LOITER, timestamp + 1000)
        self.spin_for(0.1)
        self.assertFalse(future.done())
        self.ack(timestamp=timestamp)
        self.response(future, R.SUCCESS, True, A.VEHICLE_CMD_RESULT_ACCEPTED)

    def test_03_terminal_rejections(self):
        for result in [
            A.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED,
            A.VEHICLE_CMD_RESULT_DENIED, A.VEHICLE_CMD_RESULT_UNSUPPORTED,
            A.VEHICLE_CMD_RESULT_FAILED, A.VEHICLE_CMD_RESULT_CANCELLED, 99,
        ]:
            with self.subTest(ack_result=result):
                self.prepare_status()
                future = self.request()
                self.ack(result)
                self.response(future, R.REJECTED, ack_result=result)

    def test_04_mode_without_ack_times_out(self):
        self.prepare_status()
        future = self.request()
        count = len(self.commands)
        type(self).status_mode = V.NAVIGATION_STATE_AUTO_LOITER
        self.response(future, R.ACK_TIMEOUT)
        self.assertEqual(len(self.commands), count, 'Commands must not be retried automatically')

    def test_05_accepted_without_mode_times_out(self):
        self.prepare_status()
        future = self.request()
        self.ack()
        self.response(future, R.MODE_TIMEOUT, True, A.VEHICLE_CMD_RESULT_ACCEPTED)

    def test_06_in_progress_then_accepted(self):
        self.prepare_status()
        future = self.request()
        self.ack(A.VEHICLE_CMD_RESULT_IN_PROGRESS)
        self.spin_for(0.1)
        self.assertFalse(future.done())
        self.ack()
        type(self).status_mode = V.NAVIGATION_STATE_AUTO_LOITER
        self.response(future, R.SUCCESS, True, A.VEHICLE_CMD_RESULT_ACCEPTED)

    def test_07_progress_does_not_extend_ack_deadline(self):
        self.prepare_status()
        started = time.monotonic()
        future = self.request()
        while not future.done() and time.monotonic() - started < 1.5:
            self.ack(A.VEHICLE_CMD_RESULT_IN_PROGRESS)
            self.spin_for(0.08)
        self.response(future, R.ACK_TIMEOUT, ack_result=A.VEHICLE_CMD_RESULT_IN_PROGRESS)
        self.assertLess(time.monotonic() - started, 1.5)

    def test_08_concurrent_request_is_busy(self):
        self.prepare_status()
        first = self.request()
        count = len(self.commands)
        second = self.request('LAND', expect_command=False)
        self.response(second, R.BUSY)
        self.assertEqual(len(self.commands), count)
        self.ack()
        type(self).status_mode = V.NAVIGATION_STATE_AUTO_LOITER
        self.response(first, R.SUCCESS, True, A.VEHICLE_CMD_RESULT_ACCEPTED)

    def test_09_invalid_modes_do_not_publish(self):
        self.prepare_status()
        count = len(self.commands)
        for mode in ['', 'UNKNOWN', 'loiter', 'not-a-mode']:
            self.response(self.request(mode, expect_command=False), R.INVALID_MODE)
        self.assertEqual(len(self.commands), count)

    def test_10_stale_status_blocks_commands(self):
        self.prepare_status()
        type(self).status_enabled = False
        self.spin_for(2.2)
        count = len(self.commands)
        response = self.response(self.request(expect_command=False), R.STATUS_UNAVAILABLE)
        self.assertFalse(response.status_fresh)
        self.assertEqual(len(self.commands), count)

    def test_11_unrelated_or_stale_acks_are_ignored(self):
        self.prepare_status()
        old_timestamp = self.px4_time() - 1000000
        future = self.request()
        for overrides in [
            {'command': VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM},
            {'target_system': 43}, {'target_component': 191},
            {'from_external': True}, {'timestamp': old_timestamp},
        ]:
            self.ack(A.VEHICLE_CMD_RESULT_DENIED, **overrides)
        self.spin_for(0.1)
        self.assertFalse(future.done())
        self.ack()
        type(self).status_mode = V.NAVIGATION_STATE_AUTO_LOITER
        self.response(future, R.SUCCESS, True, A.VEHICLE_CMD_RESULT_ACCEPTED)

    def test_12_cached_matching_status_cannot_confirm_entry(self):
        self.prepare_status(V.NAVIGATION_STATE_AUTO_LOITER)
        type(self).status_enabled = False
        future = self.request()
        self.ack()
        self.response(future, R.MODE_TIMEOUT, True, A.VEHICLE_CMD_RESULT_ACCEPTED)

    def test_13_already_in_mode_still_needs_ack_and_new_status(self):
        self.prepare_status(V.NAVIGATION_STATE_AUTO_LOITER)
        future = self.request()
        self.spin_for(0.1)
        self.assertFalse(future.done())
        self.ack()
        self.response(future, R.SUCCESS, True, A.VEHICLE_CMD_RESULT_ACCEPTED)

    def test_14_late_ack_from_previous_request_is_ignored(self):
        self.prepare_status()
        previous = self.request()
        old_timestamp = self.px4_time()
        self.response(previous, R.ACK_TIMEOUT)
        self.prepare_status()
        current = self.request()
        self.ack(timestamp=old_timestamp)
        type(self).status_mode = V.NAVIGATION_STATE_AUTO_LOITER
        self.spin_for(0.1)
        self.assertFalse(current.done())
        self.ack()
        self.response(current, R.SUCCESS, True, A.VEHICLE_CMD_RESULT_ACCEPTED)

    def test_15_all_mode_mappings_and_command_fields(self):
        modes = [
            ('MANUAL', V.NAVIGATION_STATE_MANUAL, 1.0, 0.0),
            ('ALTITUDE', V.NAVIGATION_STATE_ALTCTL, 2.0, 0.0),
            ('POSITION', V.NAVIGATION_STATE_POSCTL, 3.0, 0.0),
            ('MISSION', V.NAVIGATION_STATE_AUTO_MISSION, 4.0, 4.0),
            ('LOITER', V.NAVIGATION_STATE_AUTO_LOITER, 4.0, 3.0),
            ('RTL', V.NAVIGATION_STATE_AUTO_RTL, 4.0, 5.0),
            ('OFFBOARD', V.NAVIGATION_STATE_OFFBOARD, 6.0, 0.0),
            ('STABILIZED', V.NAVIGATION_STATE_STAB, 7.0, 0.0),
            ('TAKEOFF', V.NAVIGATION_STATE_AUTO_TAKEOFF, 4.0, 2.0),
            ('LAND', V.NAVIGATION_STATE_AUTO_LAND, 4.0, 6.0),
            ('PRECISION_LAND', V.NAVIGATION_STATE_AUTO_PRECLAND, 4.0, 9.0),
        ]
        for name, nav_state, main_mode, sub_mode in modes:
            with self.subTest(mode=name):
                self.prepare_status()
                future = self.request(name)
                command = self.commands[-1]
                self.assertEqual(command.command, VehicleCommand.VEHICLE_CMD_DO_SET_MODE)
                self.assertEqual((command.param1, command.param2, command.param3),
                                 (1.0, main_mode, sub_mode))
                self.assertEqual((command.target_system, command.target_component), (7, 1))
                self.assertEqual((command.source_system, command.source_component), (42, 190))
                self.assertTrue(command.from_external)
                self.assertEqual(command.confirmation, 0)
                self.assertGreater(command.timestamp, 0)
                self.ack()
                type(self).status_mode = nav_state
                response = self.response(future, R.SUCCESS, True, A.VEHICLE_CMD_RESULT_ACCEPTED)
                self.assertEqual(response.current_mode, name)

    def test_16_latest_status_must_match(self):
        self.prepare_status()
        type(self).status_enabled = False
        future = self.request()
        timestamp = self.px4_time()
        self.publish_status(V.NAVIGATION_STATE_AUTO_LOITER, timestamp + 1000)
        self.spin_for(0.05)
        self.publish_status(V.NAVIGATION_STATE_MANUAL, timestamp + 2000)
        self.spin_for(0.05)
        self.ack(timestamp=timestamp)
        self.spin_for(0.1)
        self.assertFalse(future.done())
        self.publish_status(V.NAVIGATION_STATE_AUTO_LOITER)
        self.response(future, R.SUCCESS, True, A.VEHICLE_CMD_RESULT_ACCEPTED)

    def test_17_wrong_vehicle_status_cannot_confirm_entry(self):
        self.prepare_status()
        type(self).status_enabled = False
        future = self.request()
        self.ack()
        self.publish_status(V.NAVIGATION_STATE_AUTO_LOITER, system=8)
        self.publish_status(V.NAVIGATION_STATE_AUTO_LOITER, component=2)
        self.spin_for(0.1)
        self.assertFalse(future.done())
        self.publish_status(V.NAVIGATION_STATE_AUTO_LOITER)
        self.response(future, R.SUCCESS, True, A.VEHICLE_CMD_RESULT_ACCEPTED)

    def test_18_paused_ros_clock_does_not_stop_timeout(self):
        self.prepare_status()
        parameters = self.node.create_client(
            SetParameters, '/px4_interface_under_test/set_parameters')
        self.assertTrue(parameters.wait_for_service(timeout_sec=3.0))
        set_clock = parameters.call_async(SetParameters.Request(parameters=[
            Parameter('use_sim_time', value=True).to_parameter_msg()]))
        self.until(set_clock.done)
        self.assertTrue(set_clock.result().results[0].successful)
        try:
            started = time.monotonic()
            future = self.request()
            self.response(future, R.ACK_TIMEOUT)
            self.assertLess(time.monotonic() - started, 1.5)
        finally:
            restore = parameters.call_async(SetParameters.Request(parameters=[
                Parameter('use_sim_time', value=False).to_parameter_msg()]))
            self.until(restore.done)
            self.assertTrue(restore.result().results[0].successful)
            self.node.destroy_client(parameters)

    def test_19_gps_telemetry_validity(self):
        self.prepare_status()
        gps = GpsMessage()
        gps.timestamp = self.px4_time()
        receiver = gps if LEGACY_GPS else gps.receiver
        receiver.fix_type = 3
        if LEGACY_GPS:
            receiver.latitude_deg = 41.0
            receiver.longitude_deg = -87.0
        else:
            gps.usable = True
            receiver.latitude = 41.0
            receiver.longitude = -87.0
        self.gps_pub.publish(gps)
        self.until(lambda: self.states[-1].gps_valid and self.states[-1].gps_fresh)
        receiver.fix_type = 2
        self.gps_pub.publish(gps)
        self.until(lambda: not self.states[-1].gps_valid and self.states[-1].gps_fresh)

    def test_20_arm_preconditions_block_publication(self):
        for field, value, result in [
            ('preflight', False, E.PREFLIGHT_FAILED),
            ('gcs_lost', True, E.GCS_UNAVAILABLE),
            ('failsafe', True, E.FAILSAFE_ACTIVE),
            ('safety_off', False, E.PREFLIGHT_FAILED),
            ('calibration', True, E.PREFLIGHT_FAILED),
            ('landed', False, E.PREFLIGHT_FAILED),
        ]:
            with self.subTest(field=field):
                setattr(type(self), field, value)
                self.prepare_status()
                self.spin_for(0.05)
                count = len(self.commands)
                self.response(self.execute(Q.ARM, expect_command=False), result)
                self.assertEqual(len(self.commands), count)
                setattr(type(self), field, not value)

    def test_21_arm_requires_ack_and_armed_telemetry(self):
        self.prepare_status()
        self.until(lambda: self.states[-1].ready_to_arm)
        future = self.execute(Q.ARM)
        command = self.commands[-1]
        self.assertEqual(command.command, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM)
        self.assertEqual((command.param1, command.param2), (1.0, 0.0))
        self.ack(command=command.command)
        self.spin_for(0.1)
        self.assertFalse(future.done())
        type(self).status_armed = True
        response = self.response(future, E.SUCCESS, True, A.VEHICLE_CMD_RESULT_ACCEPTED)
        self.assertTrue(response.armed)
        self.assertEqual(response.command, Q.ARM)
        self.assertGreater(response.request_id, 0)

    def test_22_disarm_requires_ack_and_disarmed_telemetry(self):
        type(self).status_armed = True
        self.prepare_status()
        future = self.execute(Q.DISARM)
        command = self.commands[-1]
        self.assertEqual((command.command, command.param1, command.param2),
                         (VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0))
        self.ack(command=command.command)
        self.spin_for(0.1)
        self.assertFalse(future.done())
        type(self).status_armed = False
        response = self.response(future, E.SUCCESS, True, A.VEHICLE_CMD_RESULT_ACCEPTED)
        self.assertFalse(response.armed)

    def test_23_airborne_disarm_is_blocked(self):
        type(self).landed = False
        type(self).status_armed = True
        self.prepare_status()
        self.spin_for(0.05)
        count = len(self.commands)
        self.response(self.execute(Q.DISARM, expect_command=False), E.UNSAFE_DISARM)
        self.assertEqual(len(self.commands), count)

    def test_24_stale_land_detection_blocks_arm_and_disarm(self):
        self.prepare_status()
        type(self).land_enabled = False
        self.spin_for(2.2)
        for kind in [Q.ARM, Q.DISARM]:
            self.response(self.execute(kind, expect_command=False), E.LANDED_UNAVAILABLE)
        self.assertFalse(self.states[-1].ready_to_arm)

    def test_25_armed_state_without_ack_is_not_success(self):
        self.prepare_status()
        future = self.execute(Q.ARM)
        type(self).status_armed = True
        self.response(future, E.ACK_TIMEOUT)

    def test_26_disarm_ack_without_transition_times_out(self):
        type(self).status_armed = True
        self.prepare_status()
        future = self.execute(Q.DISARM)
        self.ack(command=VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM)
        self.response(future, E.TRANSITION_TIMEOUT, True, A.VEHICLE_CMD_RESULT_ACCEPTED)

    def test_27_services_share_one_pending_command(self):
        self.prepare_status()
        first = self.request()
        count = len(self.commands)
        self.response(self.execute(Q.ARM, expect_command=False), E.BUSY)
        self.assertEqual(len(self.commands), count)
        self.ack()
        type(self).status_mode = V.NAVIGATION_STATE_AUTO_LOITER
        self.response(first, E.SUCCESS, True, A.VEHICLE_CMD_RESULT_ACCEPTED)

    def test_28_arm_ignores_mode_ack_and_preserves_denial_details(self):
        self.prepare_status()
        future = self.execute(Q.ARM)
        self.ack()
        self.spin_for(0.1)
        self.assertFalse(future.done())
        self.ack(A.VEHICLE_CMD_RESULT_DENIED,
                 command=VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                 result_param1=17, result_param2=23)
        response = self.response(future, E.REJECTED, ack_result=A.VEHICLE_CMD_RESULT_DENIED)
        self.assertEqual((response.ack_result_param1, response.ack_result_param2), (17, 23))

    def test_29_preflight_check_reports_actual_outcome(self):
        for passed in [False, True]:
            with self.subTest(passed=passed):
                type(self).preflight = passed
                self.prepare_status()
                future = self.execute(Q.RUN_PREFLIGHT_CHECKS)
                self.assertEqual(self.commands[-1].command, VehicleCommand.VEHICLE_CMD_RUN_PREARM_CHECKS)
                self.ack(command=VehicleCommand.VEHICLE_CMD_RUN_PREARM_CHECKS)
                self.response(future, E.SUCCESS if passed else E.PREFLIGHT_FAILED,
                              True, A.VEHICLE_CMD_RESULT_ACCEPTED)

    def test_30_invalid_commands_are_not_published(self):
        self.prepare_status()
        count = len(self.commands)
        for kind, mode in [(200, ''), (Q.SET_MODE, ''), (Q.ARM, 'LOITER'), (Q.DISARM, 'force')]:
            self.response(self.execute(kind, mode, expect_command=False), E.INVALID_REQUEST)
        self.assertEqual(len(self.commands), count)

    def test_31_failsafe_allows_only_recovery_modes(self):
        type(self).failsafe = True
        self.prepare_status()
        self.response(self.execute(Q.SET_MODE, 'LOITER', expect_command=False), E.FAILSAFE_ACTIVE)
        future = self.execute(Q.SET_MODE, 'LAND')
        self.ack()
        type(self).status_mode = V.NAVIGATION_STATE_AUTO_LAND
        self.response(future, E.SUCCESS, True, A.VEHICLE_CMD_RESULT_ACCEPTED)

    def test_32_status_loss_during_transition_is_reported(self):
        self.prepare_status()
        type(self).status_enabled = False
        self.spin_for(1.7)
        future = self.request()
        self.ack()
        response = self.response(future, E.STATUS_UNAVAILABLE, True, A.VEHICLE_CMD_RESULT_ACCEPTED)
        self.assertFalse(response.status_fresh)

    def test_33_command_relevant_state_is_exposed(self):
        self.prepare_status()
        self.until(lambda: self.states[-1].ready_to_arm)
        state = self.states[-1]
        self.assertTrue(state.pre_flight_checks_pass)
        self.assertFalse(state.gcs_connection_lost)
        self.assertFalse(state.failsafe)
        self.assertTrue(state.landed and state.land_detected_fresh)
        self.assertEqual((state.system_id, state.component_id), (7, 1))
        self.assertEqual(state.arming_state, V.ARMING_STATE_DISARMED)
        type(self).gcs_lost = True
        self.until(lambda: self.states[-1].gcs_connection_lost)
        self.assertFalse(self.states[-1].ready_to_arm)
        self.assertIn('ground-control', self.states[-1].arming_block_reason)

    def test_34_late_arm_ack_cannot_complete_disarm(self):
        self.prepare_status()
        arm = self.execute(Q.ARM)
        old_timestamp = self.px4_time()
        self.ack(command=VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, timestamp=old_timestamp)
        type(self).status_armed = True
        armed_result = self.response(arm, E.SUCCESS, True, A.VEHICLE_CMD_RESULT_ACCEPTED)
        self.prepare_status()
        disarm = self.execute(Q.DISARM)
        type(self).status_armed = False
        self.ack(command=VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, timestamp=old_timestamp)
        self.spin_for(0.1)
        self.assertFalse(disarm.done())
        self.ack(command=VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM)
        disarmed_result = self.response(disarm, E.SUCCESS, True, A.VEHICLE_CMD_RESULT_ACCEPTED)
        self.assertGreater(disarmed_result.request_id, armed_result.request_id)

    def test_35_user_intention_does_not_confirm_active_mode(self):
        type(self).status_intention = V.NAVIGATION_STATE_AUTO_LOITER
        self.prepare_status()
        self.assertEqual(self.states[-1].nav_state_user_intention, V.NAVIGATION_STATE_AUTO_LOITER)
        self.assertEqual(self.states[-1].flight_mode, 'MANUAL')
        future = self.request()
        self.ack()
        self.response(future, E.TRANSITION_TIMEOUT, True, A.VEHICLE_CMD_RESULT_ACCEPTED)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
