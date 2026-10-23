"""Drive real C++ nodes with isolated PX4 topics; never publish real commands."""
import copy
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
from rclpy.qos import qos_profile_sensor_data
from px4_interface_node.msg import DroneState
from px4_msgs.msg import (
    BatteryStatus, FailsafeFlags, VehicleLandDetected, VehicleLocalPosition, VehicleStatus,
)
from safety_monitor.msg import SafetyStatus as S
try:
    from px4_msgs.msg import SensorGps as GpsMessage
    LEGACY_GPS = True
except ImportError:
    from px4_msgs.msg import VehicleGnss as GpsMessage
    LEGACY_GPS = False

PREFIX = '/test_safety'
GPS = '/fmu/out/vehicle_gps_position' if LEGACY_GPS else '/fmu/out/vehicle_gnss'
RAW = {
    '/fmu/out/vehicle_status_v4': VehicleStatus,
    '/fmu/out/vehicle_local_position_v1': VehicleLocalPosition,
    '/fmu/out/battery_status_v1': BatteryStatus,
    '/fmu/out/failsafe_flags': FailsafeFlags,
    '/fmu/out/vehicle_land_detected': VehicleLandDetected,
    GPS: GpsMessage,
}


@pytest.mark.launch_test
def generate_test_description():
    topics = list(RAW) + [
        '/fmu/out/vehicle_odometry', '/fmu/out/vehicle_command_ack_v1',
        '/fmu/in/vehicle_command', '/drone/state', '/drone/set_flight_mode',
    ]
    interface = launch_ros.actions.Node(
        package='px4_interface_node', executable='px4_interface_node',
        name='safety_interface_under_test',
        remappings=[(t, PREFIX + t) for t in topics], output='screen')
    monitor = launch_ros.actions.Node(
        package='safety_monitor', executable='safety_monitor', name='safety_under_test',
        remappings=[('/drone/state', PREFIX + '/drone/state'),
                    ('/drone/safety_status', PREFIX + '/drone/safety_status')], output='screen')
    # Direct input tests verify loss of the adapter itself, including /clock=0.
    direct = launch_ros.actions.Node(
        package='safety_monitor', executable='safety_monitor', name='direct_safety_under_test',
        parameters=[{'use_sim_time': True}],
        remappings=[('/drone/state', PREFIX + '/direct/state'),
                    ('/drone/safety_status', PREFIX + '/direct/safety')], output='screen')
    return launch.LaunchDescription([
        interface, monitor, direct, launch_testing.actions.ReadyToTest(),
    ])


class TestSafetyPipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('safety_pipeline_test_client')
        cls.publishers = {
            topic: cls.node.create_publisher(kind, PREFIX + topic, qos_profile_sensor_data)
            for topic, kind in RAW.items()
        }
        cls.direct_pub = cls.node.create_publisher(DroneState, PREFIX + '/direct/state', 1)
        cls.states = []
        cls.statuses = []
        cls.direct_statuses = []
        cls.subscriptions = [
            cls.node.create_subscription(DroneState, PREFIX + '/drone/state', cls.states.append, 1),
            cls.node.create_subscription(S, PREFIX + '/drone/safety_status', cls.statuses.append, 1),
            cls.node.create_subscription(S, PREFIX + '/direct/safety', cls.direct_statuses.append, 1),
        ]
        cls.enabled = False
        cls.skipped = set()
        cls.messages = {}
        cls.direct_message = None
        cls.timer = cls.node.create_timer(
            0.04, cls.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        deadline = time.monotonic() + 10
        while not (all(p.get_subscription_count() for p in cls.publishers.values()) and
                   cls.direct_pub.get_subscription_count() and cls.statuses and cls.direct_statuses):
            assert time.monotonic() < deadline, 'DDS discovery failed'
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def tick(cls):
        if cls.enabled:
            for topic, message in cls.messages.items():
                if topic not in cls.skipped:
                    message.timestamp = time.monotonic_ns() // 1000
                    cls.publishers[topic].publish(message)
        if cls.direct_message is not None:
            cls.direct_pub.publish(cls.direct_message)

    def until(self, predicate, timeout=4.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.assertLess(time.monotonic(), deadline, 'Condition timed out')
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def prepare(self):
        cls = type(self)
        cls.messages = {topic: kind() for topic, kind in RAW.items()}
        status = cls.messages['/fmu/out/vehicle_status_v4']
        status.system_id = status.component_id = 1
        status.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        status.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
        status.pre_flight_checks_pass = status.safety_off = True
        local = cls.messages['/fmu/out/vehicle_local_position_v1']
        local.xy_valid = local.z_valid = local.v_xy_valid = local.v_z_valid = True
        battery = cls.messages['/fmu/out/battery_status_v1']
        battery.connected = True
        battery.remaining = 0.8
        cls.messages['/fmu/out/vehicle_land_detected'].landed = True
        gps = cls.messages[GPS]
        receiver = gps if LEGACY_GPS else gps.receiver
        receiver.fix_type = 3
        if LEGACY_GPS:
            receiver.latitude_deg, receiver.longitude_deg = 41.0, -87.0
        else:
            gps.usable = True
            receiver.latitude, receiver.longitude = 41.0, -87.0
        cls.enabled = True
        cls.skipped = set()
        cls.direct_message = None
        count = len(cls.statuses)
        self.until(lambda: len(cls.statuses) > count and cls.statuses[-1].launch_permitted)

    def wait_level(self, level):
        count = len(self.statuses)
        self.until(lambda: len(self.statuses) > count and self.statuses[-1].level == level)
        return self.statuses[-1]

    def flying(self):
        self.messages['/fmu/out/vehicle_status_v4'].arming_state = VehicleStatus.ARMING_STATE_ARMED
        self.messages['/fmu/out/vehicle_land_detected'].landed = False
        self.until(lambda: self.statuses[-1].autonomous_flight_permitted)

    def test_00_startup_unknown(self):
        # The adapter is alive but its source flags are all stale: BLOCKED.
        # A monitor with no adapter input at all remains UNKNOWN.
        self.assertEqual(self.statuses[-1].level, S.BLOCKED)
        self.assertEqual(self.statuses[-1].phase, S.PHASE_UNKNOWN)
        self.assertEqual(self.direct_statuses[-1].level, S.UNKNOWN)
        self.assertFalse(self.statuses[-1].launch_permitted)
        self.assertFalse(self.direct_statuses[-1].autonomous_flight_permitted)

    def test_01_healthy_pipeline_and_preflight(self):
        self.prepare()
        state = self.states[-1]
        self.assertTrue(state.pre_flight_checks_pass and state.safety_off and state.landed)
        self.assertTrue(state.failsafe_flags_fresh and state.land_detected_fresh)
        self.assertFalse(self.statuses[-1].visual_landing_permitted)
        self.messages['/fmu/out/vehicle_status_v4'].pre_flight_checks_pass = False
        r = self.wait_level(S.BLOCKED)
        self.assertIn('preflight', r.reason)
        self.assertFalse(r.launch_permitted)

    def test_02_rc_and_gcs_loss_from_px4(self):
        for source, field in [('/fmu/out/failsafe_flags', 'manual_control_signal_lost'),
                              ('/fmu/out/vehicle_status_v4', 'gcs_connection_lost'),
                              ('/fmu/out/failsafe_flags', 'gcs_connection_lost')]:
            with self.subTest(source=source, field=field):
                self.prepare()
                setattr(self.messages[source], field, True)
                self.assertFalse(self.wait_level(S.BLOCKED).operator_link_healthy)

    def test_03_failsafe_and_diagnostic_failure(self):
        for field in ['failsafe', 'critical_failure', 'geofence_breached']:
            with self.subTest(field=field):
                self.prepare()
                self.flying()
                if field == 'failsafe':
                    self.messages['/fmu/out/vehicle_status_v4'].failsafe = True
                else:
                    name = 'fd_motor_failure' if field == 'critical_failure' else field
                    setattr(self.messages['/fmu/out/failsafe_flags'], name, True)
                r = self.wait_level(S.EMERGENCY)
                self.assertFalse(r.autonomous_flight_permitted)

    def test_04_battery_and_invalid_position(self):
        self.prepare()
        self.flying()
        self.messages['/fmu/out/battery_status_v1'].remaining = 0.25
        r = self.wait_level(S.WARNING)
        self.assertTrue(r.autonomous_flight_permitted)
        self.messages['/fmu/out/failsafe_flags'].battery_warning = BatteryStatus.WARNING_CRITICAL
        self.assertFalse(self.wait_level(S.EMERGENCY).battery_healthy)
        for field in ['faults', 'is_powering_off']:
            self.prepare()
            self.flying()
            setattr(self.messages['/fmu/out/battery_status_v1'], field, 1 if field == 'faults' else True)
            self.assertFalse(self.wait_level(S.EMERGENCY).battery_healthy)
        self.prepare()
        self.flying()
        self.messages['/fmu/out/vehicle_local_position_v1'].x = float('nan')
        self.assertFalse(self.wait_level(S.EMERGENCY).position_healthy)

    def test_05_source_loss_while_interface_keeps_publishing(self):
        self.prepare()
        self.flying()
        type(self).skipped = {'/fmu/out/failsafe_flags'}
        before = len(self.states)
        self.assertFalse(self.wait_level(S.EMERGENCY).telemetry_healthy)
        self.assertGreater(len(self.states), before)
        self.assertFalse(self.states[-1].failsafe_flags_fresh)
        self.prepare()
        type(self).skipped = {'/fmu/out/vehicle_land_detected'}
        self.assertFalse(self.wait_level(S.BLOCKED).launch_permitted)
        self.assertFalse(self.states[-1].land_detected_fresh)

    def test_06_manual_takeover_and_visual_mode(self):
        self.prepare()
        self.flying()
        status = self.messages['/fmu/out/vehicle_status_v4']
        status.nav_state = VehicleStatus.NAVIGATION_STATE_POSCTL
        r = self.wait_level(S.BLOCKED)
        self.assertFalse(r.autonomous_flight_permitted)
        status.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_PRECLAND
        self.until(lambda: self.statuses[-1].phase == S.VISUAL_LANDING)
        self.assertFalse(self.statuses[-1].visual_landing_permitted)

    def test_07_adapter_loss_and_flight_latch_with_paused_ros_clock(self):
        self.prepare()
        self.flying()
        cls = type(self)
        cls.direct_message = copy.deepcopy(self.states[-1])
        self.until(lambda: self.direct_statuses[-1].autonomous_flight_permitted)
        self.assertEqual(self.direct_statuses[-1].timestamp.sec, 0)
        cls.direct_message = None
        self.until(lambda: not self.direct_statuses[-1].telemetry_healthy)
        self.assertEqual(self.direct_statuses[-1].level, S.EMERGENCY)
        # Stale 'disarmed' data cannot clear the last armed/airborne state.
        cls.direct_message = copy.deepcopy(self.states[-1])
        cls.direct_message.armed = False
        cls.direct_message.landed = True
        cls.direct_message.status_fresh = False
        count = len(self.direct_statuses)
        self.until(lambda: len(self.direct_statuses) > count and
                   self.direct_statuses[-1].level == S.EMERGENCY)
        cls.direct_message = None
        self.until(lambda: not self.direct_statuses[-1].telemetry_healthy)
        self.assertEqual(self.direct_statuses[-1].level, S.EMERGENCY)
        # Only fresh disarmed + landed confirmation clears the flight latch.
        cls.direct_message = copy.deepcopy(self.states[-1])
        cls.direct_message.armed = False
        cls.direct_message.landed = True
        self.until(lambda: self.direct_statuses[-1].launch_permitted)
        cls.direct_message = None
        self.until(lambda: not self.direct_statuses[-1].telemetry_healthy)
        self.assertEqual(self.direct_statuses[-1].level, S.BLOCKED)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
