#!/usr/bin/env python3
"""Foreground localhost SITL GCS link: receive diagnostics and send heartbeats.

Requires pymavlink. This does not arm, fly, change parameters, or publish fake
PX4 telemetry. Use QGroundControl for normal operation; this link exists only
while this foreground process is running.
"""

import argparse
import time

from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=60.0)
    args = parser.parse_args()
    if not 1.0 <= args.duration <= 3600.0:
        parser.error('duration must be in [1, 3600] seconds')
    link = mavutil.mavlink_connection(
        'udpin:127.0.0.1:14550', source_system=245, source_component=190)
    try:
        heartbeat = link.wait_heartbeat(timeout=10)
        if heartbeat is None or heartbeat.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
            raise RuntimeError('No PX4 heartbeat received on localhost UDP 14550')
        print('Connected to local PX4; monitoring diagnostics and sending GCS heartbeats.', flush=True)
        deadline = time.monotonic() + args.duration
        next_heartbeat = 0.0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_heartbeat:
                link.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
                next_heartbeat = time.monotonic() + 1.0
            message = link.recv_match(blocking=True, timeout=0.1)
            if message and message.get_type() == 'STATUSTEXT':
                print(f'PX4 severity={message.severity}: {message.text}', flush=True)
    finally:
        link.close()


if __name__ == '__main__':
    main()
