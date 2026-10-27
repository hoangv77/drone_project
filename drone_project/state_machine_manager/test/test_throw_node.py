"""ROS contract tests use an isolated peer, never a physical vehicle."""
import time
import unittest
import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rclpy.clock import Clock, ClockType
from px4_interface_node.msg import DroneState
from px4_interface_node.srv import ExecuteCommand as C
from px4_msgs.msg import VehicleAttitude
from safety_monitor.msg import SafetyStatus as S
from state_machine_manager.msg import MissionState as M, NativeLaunchStatus as N
from std_srvs.srv import Trigger

P = '/test_throw'


@pytest.mark.launch_test
def generate_test_description():
    endpoints = ['/drone/state', '/drone/safety_status', '/drone/mission_state',
                 '/simulation/native_launch_status', '/fmu/out/vehicle_attitude',
                 '/drone/execute_command'] + ['/drone/mission/' + s for s in ['start','abort','reset']]
    return launch.LaunchDescription([
        launch_ros.actions.Node(package='state_machine_manager', executable='throw_launch_manager',
                                remappings=[(s, P+s) for s in endpoints], output='screen'),
        launch_testing.actions.ReadyToTest()])


class TestThrowNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(); cls.node = rclpy.create_node('throw_contract_peer')
        cls.states = []; cls.commands = []
        cls.sub = cls.node.create_subscription(M, P+'/drone/mission_state', cls.states.append, 10)
        cls.dp = cls.node.create_publisher(DroneState, P+'/drone/state', 1)
        cls.sp = cls.node.create_publisher(S, P+'/drone/safety_status', 1)
        cls.np = cls.node.create_publisher(N, P+'/simulation/native_launch_status', 1)
        cls.ap = cls.node.create_publisher(VehicleAttitude, P+'/fmu/out/vehicle_attitude', 1)
        cls.peer = cls.node.create_service(C, P+'/drone/execute_command', cls.command)
        cls.clients = {s:cls.node.create_client(Trigger, P+'/drone/mission/'+s) for s in ['start','abort','reset']}
        for client in cls.clients.values(): assert client.wait_for_service(timeout_sec=10)
        cls.enabled = False
        cls.timer = cls.node.create_timer(0.04, cls.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    @classmethod
    def command(cls, request, response):
        cls.commands.append(request.command)
        response.accepted = response.success = not cls.reject
        response.status_fresh = True
        response.result = C.Response.REJECTED if cls.reject else C.Response.SUCCESS
        response.message = 'isolated peer response'
        if not cls.reject:
            if request.command == C.Request.SET_MODE: cls.drone.flight_mode = request.mode
            if request.command == C.Request.ARM: cls.drone.armed = True
            if request.command == C.Request.DISARM: cls.drone.armed = False
        response.armed = cls.drone.armed
        response.current_mode = cls.drone.flight_mode
        return response

    @classmethod
    def publish(cls):
        if not cls.enabled: return
        if cls.telemetry: cls.dp.publish(cls.drone)
        cls.safety.phase = S.FLIGHT if cls.drone.armed else S.GROUNDED
        cls.safety.landed = cls.drone.landed
        cls.safety.launch_permitted = not cls.drone.armed and cls.safety.level == S.READY
        cls.safety.autonomous_flight_permitted = cls.drone.armed
        cls.sp.publish(cls.safety)
        timestamp = time.monotonic_ns()//1000
        if cls.native_enabled:
            n = N(); n.px4_timestamp = timestamp; n.armed = cls.drone.armed; n.lockdown = cls.lockdown
            cls.np.publish(n)
        a = VehicleAttitude(); a.timestamp = timestamp; a.q = cls.quaternion
        cls.ap.publish(a)

    def spin(self, seconds=0.3):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline: rclpy.spin_once(self.node, timeout_sec=0.01)

    def until(self, predicate, timeout=5):
        deadline = time.monotonic()+timeout
        while not predicate():
            self.assertLess(time.monotonic(), deadline, 'contract condition timed out')
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def call(self, name):
        future = self.clients[name].call_async(Trigger.Request())
        self.until(future.done); return future.result()

    def setUp(self):
        cls = type(self)
        cls.reject = False; cls.telemetry = True; cls.native_enabled = True
        cls.lockdown = True; cls.quaternion = [1.,0.,0.,0.]
        d = DroneState()
        for name in ['status_fresh','local_position_fresh','land_detected_fresh','failsafe_flags_fresh',
                     'battery_fresh','position_valid','ready_to_arm','pre_flight_checks_pass','safety_off','landed']:
            setattr(d, name, True)
        d.flight_mode = 'LOITER'; d.battery_remaining = 0.8
        cls.drone = d; cls.safety = S(); cls.safety.level = S.READY; cls.safety.telemetry_healthy = True
        cls.enabled = True; self.spin(0.4)
        if self.states[-1].mission_active: self.call('abort'); self.spin(0.4)
        if self.states[-1].current_state in [M.ABORTED,M.FAULT]:
            self.assertTrue(self.call('reset').success)
        self.spin(0.2); cls.commands.clear()
        self.assertEqual(self.states[-1].current_state, M.STANDBY)

    def arm(self):
        self.assertTrue(self.call('start').success)
        self.until(lambda: self.states[-1].current_state == M.ARMED_WAITING)

    def test_01_no_automatic_arming(self):
        self.spin(); self.assertFalse(self.drone.armed); self.assertEqual(self.commands, [])

    def test_02_safety_denial(self):
        type(self).safety.level = S.BLOCKED; self.spin()
        self.assertFalse(self.call('start').success); self.assertEqual(self.commands, [])

    def test_03_command_rejection(self):
        type(self).reject = True; self.assertTrue(self.call('start').success)
        self.until(lambda: self.states[-1].current_state == M.ABORTED)
        self.assertNotIn(C.Request.ARM, self.commands)

    def test_04_abort_before_release(self):
        self.arm(); self.assertTrue(self.call('abort').success)
        self.until(lambda: self.states[-1].current_state == M.ABORTED)
        self.assertFalse(self.drone.armed); self.assertIn(C.Request.DISARM, self.commands)

    def test_05_airborne_alone_does_not_prove_native_detection(self):
        self.arm(); type(self).drone.landed = False; self.spin(0.3)
        self.assertEqual(self.states[-1].current_state, M.ARMED_WAITING)

    def test_06_native_observer_loss(self):
        self.arm(); type(self).native_enabled = False
        self.until(lambda: self.states[-1].current_state == M.RECOVERY)
        self.assertNotEqual(self.states[-1].current_state, M.THROW_LAUNCH)

    def test_07_telemetry_loss(self):
        self.arm(); type(self).telemetry = False
        self.until(lambda: self.states[-1].current_state == M.RECOVERY)

    def test_08_native_detection_and_stabilization(self):
        self.arm(); type(self).lockdown = False; type(self).drone.landed = False
        self.until(lambda: self.states[-1].current_state == M.STANDBY)
        self.assertEqual(self.states[-1].mission_result, M.RESULT_SUCCEEDED)
        self.assertTrue(any(s.current_state == M.THROW_LAUNCH for s in self.states))
        self.assertTrue(any(s.current_state == M.STABILIZE for s in self.states))
        self.assertFalse(any(s.current_state == M.SEARCH_TAG for s in self.states))
        self.assertFalse(self.call('start').success)

    def test_09_unfavorable_attitude_cannot_complete(self):
        self.arm(); type(self).lockdown = False; type(self).drone.landed = False
        type(self).quaternion = [0.,1.,0.,0.]
        self.until(lambda: self.states[-1].current_state == M.STABILIZE)
        self.spin(1.2); self.assertEqual(self.states[-1].current_state, M.STABILIZE)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node(); rclpy.shutdown()
