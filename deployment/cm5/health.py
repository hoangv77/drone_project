"""Bounded live read-only checks. This program never calls a flight service."""
import sys
import time


def telemetry_advancing(samples):
    fields = ('status', 'land_detected', 'local_position', 'odometry', 'gps',
              'battery', 'failsafe_flags')
    if len(samples) < 2:
        return False
    first, last = samples[0], samples[-1]
    return (first.last_update_time != last.last_update_time
            and all(getattr(last, field + '_fresh') for field in fields))


def main(mode):
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rclpy.qos import qos_profile_sensor_data
    from px4_interface_node.msg import DroneState
    from safety_monitor.msg import SafetyStatus
    from state_machine_manager.msg import MissionState

    rclpy.init(args=[])
    node = rclpy.create_node('cm5_verifier')
    received = {}

    def observe(topic):
        def callback(message):
            samples = received.setdefault(topic, [])
            samples.append((time.monotonic(), message))
        return callback

    topics = {'/drone/state': DroneState, '/drone/safety_status': SafetyStatus,
              '/drone/mission_state': MissionState}
    for topic, schema in topics.items():
        node.create_subscription(schema, topic, observe(topic), qos_profile_sensor_data)
    settings = {'px4_interface_node': 'read_only', 'state_machine_manager': 'monitor_only'}
    clients = {name: node.create_client(GetParameters, f'/{name}/get_parameters')
               for name in settings}
    futures = {}
    failures = 0

    def check(label, passed):
        nonlocal failures
        print(f'{"PASS" if passed else "FAIL"} {label}', flush=True)
        failures += not passed

    try:
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            for name, client in clients.items():
                if name not in futures and client.service_is_ready():
                    futures[name] = client.call_async(GetParameters.Request(names=[settings[name]]))
            rclpy.spin_once(node, timeout_sec=0.15)

        recent = {topic: [sample for stamp, sample in received.get(topic, [])
                          if time.monotonic() - stamp < 1.0] for topic in topics}
        for topic, samples in recent.items():
            check(f'{topic} streaming', len(samples) >= 2 and node.count_publishers(topic) == 1)
        mission = recent['/drone/mission_state']
        check('mission idle in STANDBY with feedback disabled',
              bool(mission) and all(m.current_state == m.STANDBY and not m.mission_active
                                    and not m.stage_feedback_enabled for m in mission))
        for name, setting in settings.items():
            future = futures.get(name)
            result = future.result() if future and future.done() and not future.exception() else None
            check(f'{name}.{setting}', bool(result and result.values and result.values[0].bool_value))
        outbound = [topic for topic, _ in node.get_topic_names_and_types()
                    if topic.startswith('/fmu/in/') and node.count_publishers(topic)]
        check('no FMU input publishers', not outbound)
        if mode in ('sitl', 'hardware'):
            drone = recent['/drone/state']
            check('fresh, advancing PX4 telemetry on all required streams', telemetry_advancing(drone))
            check('observed disarmed and landed', bool(drone) and drone[-1].status_fresh
                  and drone[-1].land_detected_fresh and not drone[-1].armed and drone[-1].landed)
        else:
            print('SKIP PX4 telemetry and physical state (passive startup check)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return int(bool(failures))


if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))
