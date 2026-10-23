# drone_project

ROS 2 interface that publishes PX4 telemetry on `/drone/state` at 10 Hz and
handles verified flight-mode changes through `/drone/set_flight_mode`.

## Structure

```text
px4_interface_node/
├── CMakeLists.txt
├── package.xml
├── msg/DroneState.msg
├── srv/SetFlightMode.srv
├── test/test_flight_mode.py
└── src/px4_interface_node.cpp
```

## Prerequisites

- ROS 2 Humble or later with `ament_cmake`, `rclcpp`, and interface generation
- `px4_msgs` matching the firmware's message definitions (or a compatible PX4
  message translation node)
- PX4 SITL or a vehicle connected through the Micro XRCE-DDS agent

## Build

Place `px4_interface_node` and `px4_msgs` inside your ROS 2 workspace's `src/`,
then run from the workspace root:

```bash
source /opt/ros/humble/setup.bash  # Adjust for your ROS 2 distribution.
colcon build --packages-up-to px4_interface_node
source install/setup.bash
```

## Run

```bash
ros2 run px4_interface_node px4_interface_node
```

## Request and verify a mode change

```bash
ros2 service call /drone/set_flight_mode px4_interface_node/srv/SetFlightMode "{mode: 'LOITER'}"
ros2 topic echo /drone/state
```

The service sends `VEHICLE_CMD_DO_SET_MODE` on `/fmu/in/vehicle_command` and
defers its response while telemetry and acknowledgments continue to arrive.
`success: true` requires both:

1. A matching terminal `ACCEPTED` on `/fmu/out/vehicle_command_ack_v1`.
2. Fresh `/fmu/out/vehicle_status_v4` reporting the requested **active**
   `nav_state`, with a PX4 timestamp at least as new as that acknowledgment.

An acknowledgment alone does not prove mode entry. PX4 can accept a requested
mode while a failsafe keeps a different mode active. Status and ACKs may arrive
in either order; cached status from before the command cannot complete it.
Even a request for the current mode requires an ACK and new status.

Supported names are case-sensitive and match `/drone/state.flight_mode`:
`MANUAL`, `ALTITUDE`, `POSITION`, `MISSION`, `LOITER`, `RTL`, `OFFBOARD`,
`STABILIZED`, `TAKEOFF`, `LAND`, `PRECISION_LAND`.

The response includes `result`, `accepted`, raw `ack_result`, `current_mode`,
`status_fresh`, and a descriptive `message`:

| Result | Meaning |
| --- | --- |
| `SUCCESS` (0) | PX4 accepted and fresh telemetry confirmed the mode. |
| `INVALID_MODE` (1) | Unsupported name; no command sent. |
| `BUSY` (2) | Another mode request is pending; no command sent. |
| `STATUS_UNAVAILABLE` (3) | Target vehicle status is absent or older than two seconds; no command sent. |
| `REJECTED` (4) | PX4 returned a terminal result other than `ACCEPTED`; see `ack_result`. |
| `ACK_TIMEOUT` (5) | No terminal ACK arrived before the acknowledgment deadline. |
| `MODE_TIMEOUT` (6) | PX4 accepted, but fresh telemetry did not confirm entry before the mode deadline. |

`accepted` records a terminal PX4 `ACCEPTED`, including when mode entry later
times out. `ack_result` is 255 (`NO_ACK`) if no matching ACK arrived, otherwise
the latest matching raw PX4 result. `IN_PROGRESS` keeps the request pending
without extending the ACK deadline. All other non-accepted results, including
temporary rejection, return `REJECTED`. `current_mode` is last known;
`status_fresh` indicates whether it is current.

Only one request is sent at a time. No automatic retries are made: a timeout
does not cancel the command in PX4, and the mode may still change afterward.
Inspect fresh telemetry before deciding whether to issue another request.
The node sends mode changes only; it does not arm or supply offboard setpoints.
PX4's normal prerequisites still apply (for example, a suitable position
estimate for LOITER and an existing offboard heartbeat/setpoint stream for
OFFBOARD).

## Routing and timeouts

Startup parameters (read-only after startup):

| Parameter | Default | Meaning |
| --- | --- | --- |
| `target_system` | 1 | PX4 system ID; also filters incoming vehicle status. |
| `target_component` | 1 | PX4 autopilot component ID; also filters vehicle status. |
| `source_system` | 1 | This command sender's system ID. |
| `source_component` | 191 | This command sender's component ID. |
| `ack_timeout_ms` | 2000 | Deadline from publication to terminal ACK. |
| `mode_timeout_ms` | 5000 | Deadline from ACCEPTED to confirmed mode. |

IDs must be in `[1, 255]`; timeout values must be in `[100, 60000]` ms. Deadlines
and telemetry freshness use a monotonic clock, so paused ROS simulation time
does not prevent a timeout. Commands carry ROS timestamps in microseconds;
ACK/status ordering compares only the timestamps supplied by PX4.

ACKs must identify `DO_SET_MODE`, target this node's source IDs, originate from
PX4 (`from_external: false`), and be newer than the pre-command status snapshot.
ACKs carry no transaction ID or requested mode. Reserve the source ID pair for
this node, and do not run another command sender using the same IDs. Serialization
and timestamp checks reject unrelated and already-old ACKs, but cannot identify
an arbitrarily delayed earlier ACK generated after the next request's status
snapshot. Separate vehicles must use separate DDS namespaces/remappings.

```bash
ros2 run px4_interface_node px4_interface_node --ros-args \
  -p target_system:=2 -p source_component:=192 \
  -p ack_timeout_ms:=3000 -p mode_timeout_ms:=8000
```

The topic names follow the repository's versioned telemetry interfaces:

| Direction | Topic | Type |
| --- | --- | --- |
| In | `/fmu/out/vehicle_status_v4` | `px4_msgs/msg/VehicleStatus` |
| In | `/fmu/out/vehicle_local_position_v1` | `px4_msgs/msg/VehicleLocalPosition` |
| In | `/fmu/out/vehicle_odometry` | `px4_msgs/msg/VehicleOdometry` |
| In | `/fmu/out/vehicle_gps_position` | `px4_msgs/msg/SensorGps` |
| In (newer GNSS interface) | `/fmu/out/vehicle_gnss` | `px4_msgs/msg/VehicleGnss` |
| In | `/fmu/out/battery_status_v1` | `px4_msgs/msg/BatteryStatus` |
| In | `/fmu/out/vehicle_command_ack_v1` | `px4_msgs/msg/VehicleCommandAck` |
| Out | `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` |
| Out | `/drone/state` | `px4_interface_node/msg/DroneState` |

The build selects the original `SensorGps` interface when available; otherwise
it uses the selected `VehicleGnss` receiver published by current PX4 firmware.
Only one of these GPS subscriptions is created. Both require a 3D fix and
finite coordinates; `VehicleGnss` must also report `usable: true`.

PX4 inputs use best-effort sensor-data subscriptions; commands use reliable,
volatile QoS. Ensure the command and ACK topics are exposed by PX4's DDS bridge.
Use `ros2 topic list` to verify topic names and ROS remappings if your bridge
uses different namespaces or version suffixes. Remapping alone does not convert
incompatible message definitions.

## Tests

Install the package's test dependencies with `rosdep`, then run:

```bash
colcon test --packages-select px4_interface_node --event-handlers console_direct+
colcon test-result --verbose
```

The launch test starts the real C++ node and a simulated PX4 peer over DDS,
remapping every endpoint under `/test_px4_interface` to isolate commands from
any connected vehicle. It covers success, ACK/status reordering, every supported
mode's wire encoding, rejections, progress, both timeouts, stale/missing/wrong
vehicle status, unrelated/late ACKs, invalid modes, concurrent requests, and
continued telemetry publication while a request is pending. This test does not
replace PX4 SITL or hardware validation. A GPS validity regression test also
covers the GPS/GNSS interface selected at build time.

PX4 protocol references: [ROS 2 user guide](https://docs.px4.io/main/en/ros2/user_guide),
[custom mode definitions](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/px4_custom_mode.h),
[acknowledgment definition](https://github.com/PX4/px4_msgs/blob/main/msg/VehicleCommandAck.msg).

The package still contains placeholder maintainer metadata; update it before
publishing a release.
