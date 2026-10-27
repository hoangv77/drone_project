#!/usr/bin/env python3
"""Foreground, local SITL operator link: GCS heartbeat and neutral joystick."""
import time
from pymavlink import mavutil

link = mavutil.mavlink_connection('udpin:127.0.0.1:14550', source_system=245, source_component=190)
try:
    if link.wait_heartbeat(timeout=120) is None: raise RuntimeError('No localhost SITL heartbeat')
    heartbeat = 0
    while True:
        now = time.monotonic()
        if now >= heartbeat:
            link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
            heartbeat = now+1
        link.mav.manual_control_send(link.target_system, 0, 0, 500, 0, 0)
        message = link.recv_match(blocking=True, timeout=0.04)
        if message and message.get_type() == 'STATUSTEXT': print(message.text, flush=True)
except KeyboardInterrupt:
    pass
finally:
    link.close()
