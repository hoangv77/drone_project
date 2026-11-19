"""Integration test for a disconnected onboard launch in an isolated DDS domain."""
import os
from pathlib import Path
import signal
import subprocess
import time
import unittest

CM5 = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get('CM5_TEST_ROS') == '1', 'requires sourced ROS overlay')
class PassiveLaunchTests(unittest.TestCase):
    def test_disconnected_launch_and_service_interlocks(self):
        import rclpy
        from px4_interface_node.msg import DroneState
        from px4_interface_node.srv import ExecuteCommand, SetFlightMode
        from rclpy.qos import qos_profile_sensor_data
        from safety_monitor.msg import SafetyStatus
        from state_machine_manager.msg import MissionState
        from std_srvs.srv import Trigger

        os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='92')
        rclpy.init(args=[])
        self.addCleanup(rclpy.shutdown)
        probe = rclpy.create_node('cm5_passive_test')
        self.addCleanup(probe.destroy_node)
        mission, drone, safety = [], [], []
        probe.create_subscription(MissionState, '/drone/mission_state', mission.append, 10)
        probe.create_subscription(DroneState, '/drone/state', drone.append, qos_profile_sensor_data)
        probe.create_subscription(SafetyStatus, '/drone/safety_status', safety.append, qos_profile_sensor_data)
        profile = os.environ['PX4_COMPATIBILITY_PROFILE']
        launch = subprocess.Popen(['ros2', 'launch', str(CM5 / 'launch/onboard.launch.py'),
                                   f'compatibility_profile:={profile}'],
                                  start_new_session=True)

        def stop():
            if launch.poll() is None:
                os.killpg(launch.pid, signal.SIGINT)
                try:
                    launch.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGKILL)
                    launch.wait()
        self.addCleanup(stop)

        deadline = time.monotonic() + 20
        while (not mission or not drone) and time.monotonic() < deadline:
            self.assertIsNone(launch.poll())
            rclpy.spin_once(probe, timeout_sec=0.2)
        self.assertTrue(mission and drone)
        self.assertFalse(drone[-1].status_fresh)
        self.assertEqual(mission[-1].current_state, MissionState.STANDBY)

        if profile == 'legacy_1_14_passive':
            # Exercise the unchanged consumers with the actual legacy adapter.
            from px4_msgs.msg import FailsafeFlags, SensorGps, VehicleLocalPosition, VehicleOdometry, VehicleStatus
            types = {'vehicle_status': VehicleStatus, 'vehicle_local_position': VehicleLocalPosition,
                     'vehicle_odometry': VehicleOdometry, 'vehicle_gps_position': SensorGps,
                     'failsafe_flags': FailsafeFlags}
            publishers = {name: probe.create_publisher(kind, '/fmu/out/' + name, qos_profile_sensor_data)
                          for name, kind in types.items()}
            messages = {name: kind() for name, kind in types.items()}
            messages['vehicle_status'].system_id = messages['vehicle_status'].component_id = 1
            messages['vehicle_status'].arming_state = VehicleStatus.ARMING_STATE_STANDBY
            messages['vehicle_status'].pre_flight_checks_pass = True
            messages['vehicle_status'].safety_off = True
            local = messages['vehicle_local_position']
            local.xy_valid = local.z_valid = local.v_xy_valid = local.v_z_valid = True
            gps = messages['vehicle_gps_position']
            gps.fix_type, gps.lat, gps.lon = 3, 410000000, -870000000
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                for name, message in messages.items():
                    message.timestamp = time.time_ns() // 1000
                    publishers[name].publish(message)
                rclpy.spin_once(probe, timeout_sec=0.05)
                if (drone[-1].status_fresh and drone[-1].position_valid and safety and
                        safety[-1].preflight_passed):
                    break
            self.assertTrue(drone[-1].status_fresh and drone[-1].position_valid)
            self.assertFalse(drone[-1].battery_fresh or drone[-1].land_detected_fresh)
            self.assertTrue(safety[-1].preflight_passed)
            self.assertEqual(safety[-1].phase, SafetyStatus.PHASE_UNKNOWN)
            self.assertEqual(safety[-1].level, SafetyStatus.BLOCKED)
            self.assertFalse(safety[-1].battery_healthy or safety[-1].landed or
                             safety[-1].launch_permitted or safety[-1].autonomous_flight_permitted)
            self.assertEqual(mission[-1].current_state, MissionState.STANDBY)
            self.assertFalse(mission[-1].mission_active)
            self.assertEqual(probe.count_publishers('/fmu/in/vehicle_command'), 0)
        self.assertFalse(mission[-1].mission_active)
        self.assertEqual(probe.count_publishers('/fmu/in/vehicle_command'), 0)

        calls = [
            (ExecuteCommand, '/drone/execute_command', ExecuteCommand.Request(command=1), 'read-only'),
            (SetFlightMode, '/drone/set_flight_mode', SetFlightMode.Request(mode='TAKEOFF'), 'read-only'),
            (Trigger, '/drone/mission/start', Trigger.Request(), 'monitoring only'),
        ]
        for service, name, request, reason in calls:
            client = probe.create_client(service, name)
            self.assertTrue(client.wait_for_service(timeout_sec=5))
            future = client.call_async(request)
            rclpy.spin_until_future_complete(probe, future, timeout_sec=5)
            self.assertTrue(future.done())
            self.assertFalse(future.result().success)
            self.assertIn(reason, future.result().message)
        self.assertEqual(probe.count_publishers('/fmu/in/vehicle_command'), 0)
        self.assertEqual(mission[-1].current_state, MissionState.STANDBY)

        # A competing mission publisher forces the onboard launch down.
        competing = probe.create_publisher(MissionState, '/drone/mission_state', 10)
        self.assertEqual(launch.wait(timeout=12), 0)
        probe.destroy_publisher(competing)


if __name__ == '__main__':
    unittest.main()
