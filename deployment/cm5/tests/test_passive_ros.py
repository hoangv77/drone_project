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
        from state_machine_manager.msg import MissionState
        from std_srvs.srv import Trigger

        os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='92')
        rclpy.init(args=[])
        self.addCleanup(rclpy.shutdown)
        probe = rclpy.create_node('cm5_passive_test')
        self.addCleanup(probe.destroy_node)
        mission, drone = [], []
        probe.create_subscription(MissionState, '/drone/mission_state', mission.append, 10)
        probe.create_subscription(DroneState, '/drone/state', drone.append, qos_profile_sensor_data)
        launch = subprocess.Popen(['ros2', 'launch', str(CM5 / 'launch/onboard.launch.py')],
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
