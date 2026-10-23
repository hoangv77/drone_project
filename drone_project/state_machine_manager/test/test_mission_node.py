"""Exercise operator APIs and observation-only transitions over isolated DDS."""
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
from px4_interface_node.msg import DroneState
from safety_monitor.msg import SafetyStatus as S
from state_machine_manager.msg import MissionState as M, StageFeedback as F
from std_srvs.srv import Trigger

PREFIX = '/test_mission'


@pytest.mark.launch_test
def generate_test_description():
    actions = []
    for name, future in [('passive', False), ('future', True)]:
        topics = ['/drone/state', '/drone/safety_status', '/drone/mission_state',
                  '/drone/mission/stage_feedback', '/drone/mission/start',
                  '/drone/mission/abort', '/drone/mission/reset']
        remaps = [(t, PREFIX + (t if t in topics[:2] else '/' + name + t)) for t in topics]
        actions.append(launch_ros.actions.Node(
            package='state_machine_manager', executable='state_machine_manager',
            name='mission_' + name, remappings=remaps,
            parameters=[{
                'use_sim_time': True, 'enable_stage_feedback': future,
                'telemetry_timeout_ms': 200, 'safety_timeout_ms': 200,
                'feedback_timeout_ms': 200, 'arming_handoff_timeout_ms': 400,
                'mission_timeout_ms': 20000, 'preflight_timeout_ms': 1200,
                'armed_waiting_timeout_ms': 2000, 'throw_timeout_ms': 1500,
                'stabilize_timeout_ms': 1500, 'search_timeout_ms': 1500,
                'tracking_timeout_ms': 2000, 'tag_loss_timeout_ms': 500,
                'landing_timeout_ms': 2000, 'recovery_timeout_ms': 800,
            }], output='screen'))
    return launch.LaunchDescription(actions + [launch_testing.actions.ReadyToTest()])


class TestMissionNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('mission_test_client')
        cls.drone_pub = cls.node.create_publisher(DroneState, PREFIX + '/drone/state', 1)
        cls.safety_pub = cls.node.create_publisher(S, PREFIX + '/drone/safety_status', 1)
        cls.feedback_pub = {}
        cls.states = {'passive': [], 'future': []}
        cls.subscriptions = []
        cls.clients = {}
        for name in cls.states:
            base = PREFIX + '/' + name + '/drone'
            cls.feedback_pub[name] = cls.node.create_publisher(F, base + '/mission/stage_feedback', 1)
            cls.subscriptions.append(cls.node.create_subscription(
                M, base + '/mission_state', cls.states[name].append, 1))
            for operation in ['start', 'abort', 'reset']:
                client = cls.node.create_client(Trigger, base + '/mission/' + operation)
                assert client.wait_for_service(timeout_sec=10)
                cls.clients[name, operation] = client
        cls.enabled = False
        cls.safety_enabled = True
        cls.drone = DroneState()
        cls.safety = S()
        cls.timer = cls.node.create_timer(
            0.03, cls.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        deadline = time.monotonic() + 10
        while not all(cls.states.values()):
            assert time.monotonic() < deadline
            rclpy.spin_once(cls.node, timeout_sec=0.02)

    @classmethod
    def tick(cls):
        if cls.enabled:
            cls.drone_pub.publish(cls.drone)
            if cls.safety_enabled:
                cls.safety_pub.publish(cls.safety)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def until(self, predicate, timeout=4):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.assertLess(time.monotonic(), deadline, 'Mission condition timed out')
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def call(self, operation, name='passive'):
        future = self.clients[name, operation].call_async(Trigger.Request())
        self.until(future.done)
        return future.result()

    def wait_state(self, expected, name='passive'):
        self.until(lambda: self.states[name][-1].current_state == expected)
        return self.states[name][-1]

    def ground(self):
        cls = type(self)
        cls.drone = DroneState()
        d = cls.drone
        d.status_fresh = d.land_detected_fresh = d.failsafe_flags_fresh = True
        d.local_position_fresh = d.position_valid = d.battery_fresh = True
        d.pre_flight_checks_pass = d.safety_off = d.ready_to_arm = d.landed = True
        d.flight_mode = 'LOITER'
        d.battery_remaining = 0.8
        cls.safety = S()
        s = cls.safety
        s.level, s.phase = S.READY, S.GROUNDED
        s.telemetry_healthy = s.launch_permitted = s.landed = True
        cls.enabled = cls.safety_enabled = True
        self.spin_for(0.18)

    def tearDown(self):
        self.ground()
        for name in self.states:
            if self.states[name][-1].mission_active:
                self.call('abort', name)
            self.until(lambda: not self.states[name][-1].mission_active)
            if self.states[name][-1].current_state != M.STANDBY:
                self.assertTrue(self.call('reset', name).success)
                self.wait_state(M.STANDBY, name)

    def arm(self, name='passive'):
        self.ground()
        self.assertTrue(self.call('start', name).success)
        type(self).drone.armed = True
        type(self).drone.ready_to_arm = False
        type(self).safety.phase = S.FLIGHT
        type(self).safety.launch_permitted = False
        type(self).safety.autonomous_flight_permitted = True
        self.wait_state(M.ARMED_WAITING, name)

    def feedback(self, event, name='future'):
        state = self.states[name][-1]
        f = F(mission_id=state.mission_id, transition_sequence=state.transition_sequence,
              observation_sequence=time.monotonic_ns(), verified=True, event=event)
        self.feedback_pub[name].publish(f)

    def test_00_missing_input_and_no_flight_control_endpoints(self):
        self.assertFalse(self.call('start').success)
        self.assertFalse(self.states['passive'][-1].safety_fresh)
        for name in ['mission_passive', 'mission_future']:
            publishers = self.node.get_publisher_names_and_types_by_node(name, '/')
            clients = self.node.get_client_names_and_types_by_node(name, '/')
            self.assertFalse(any('/fmu/in/' in topic for topic, _ in publishers))
            self.assertFalse(any('execute_command' in topic or 'set_flight_mode' in topic
                                 for topic, _ in clients))

    def test_01_operator_start_abort_reset_and_metadata(self):
        self.ground()
        self.assertTrue(self.call('start').success)
        state = self.wait_state(M.PREFLIGHT)
        self.assertEqual(state.previous_state, M.STANDBY)
        self.assertTrue(state.mission_active)
        self.assertGreater(state.mission_id, 0)
        self.assertFalse(self.call('start').success)
        self.spin_for(0.2)
        self.assertEqual(self.states['passive'][-1].current_state, M.PREFLIGHT)
        self.assertTrue(self.call('abort').success)
        self.wait_state(M.ABORTED)
        self.assertTrue(self.call('reset').success)
        self.wait_state(M.STANDBY)

    def test_02_safety_permission_is_required(self):
        self.ground()
        type(self).safety.level = S.BLOCKED
        type(self).safety.launch_permitted = False
        self.spin_for(0.18)
        self.assertFalse(self.call('start').success)
        self.assertEqual(self.states['passive'][-1].current_state, M.STANDBY)

    def test_03_stale_safety_blocks_active_mission_and_reset(self):
        self.ground()
        self.assertTrue(self.call('start').success)
        self.wait_state(M.PREFLIGHT)
        type(self).safety_enabled = False
        state = self.wait_state(M.ABORTED)
        self.assertIn('safety', state.transition_reason.lower())
        self.assertFalse(self.call('reset').success)

    def test_04_default_feedback_cannot_enter_flight_states(self):
        self.arm()
        before = self.states['passive'][-1].rejected_requests
        self.feedback(F.THROW_DETECTED, 'passive')
        self.until(lambda: self.states['passive'][-1].rejected_requests > before)
        type(self).drone.landed = type(self).safety.landed = False
        self.wait_state(M.RECOVERY)
        self.wait_state(M.FAULT)
        self.assertFalse(self.call('reset').success)
        self.ground()
        self.assertTrue(self.call('reset').success)
        self.wait_state(M.STANDBY)

    def test_05_future_observation_path_and_tag_loss(self):
        self.arm('future')
        type(self).drone.landed = type(self).safety.landed = False
        self.spin_for(0.08)
        for event, state in [(F.THROW_DETECTED, M.THROW_LAUNCH),
                             (F.LAUNCH_COMPLETE, M.STABILIZE),
                             (F.STABILIZED, M.SEARCH_TAG), (F.TAG_ACQUIRED, M.TRACK_TAG)]:
            self.feedback(event)
            self.wait_state(state, 'future')
        self.feedback(F.LANDING_READY)
        self.until(lambda: 'Precision landing blocked' in self.states['future'][-1].last_rejection)
        self.assertEqual(self.states['future'][-1].current_state, M.TRACK_TAG)
        self.feedback(F.TAG_LOST)
        self.wait_state(M.SEARCH_TAG, 'future')
        self.assertTrue(self.call('abort', 'future').success)
        self.wait_state(M.RECOVERY, 'future')

    def test_06_timeout_runs_with_paused_ros_clock(self):
        self.ground()
        self.assertTrue(self.call('start').success)
        self.wait_state(M.PREFLIGHT)
        self.assertEqual(self.states['passive'][-1].timestamp.sec, 0)
        self.wait_state(M.ABORTED)
        self.assertIn('timed out', self.states['passive'][-1].transition_reason)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
