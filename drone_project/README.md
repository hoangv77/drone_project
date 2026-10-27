# drone_project

Phase 6 adds an opt-in [native throw-launch SITL integration](simulation/README.md)
with a CAD-envelope-based provisional model, project-local PX4 parameters,
physical release fixture and repeatable tests. Existing passive launches remain
available. Hardware mass and propulsion measurements are still required.

ROS 2 interface that publishes PX4 state at 10 Hz and provides guarded mode,
arm, disarm, and preflight-check commands. Success requires both a terminal
PX4 `ACCEPTED` acknowledgment and fresh telemetry confirming the result.

The [safety_monitor](safety_monitor/README.md) package adds a separate C++ node
that publishes phase-aware permissions on `/drone/safety_status` at 10 Hz.
Build both with `colcon build --packages-up-to safety_monitor`, source the
overlay, and run `ros2 launch safety_monitor safety.launch.py`. It requires fresh
PX4 failsafe flags and land detection in addition to the existing telemetry.
Rebuild all `DroneState` consumers after updating this branch. Visual landing
remains blocked pending AprilTag and landing-envelope integration.

The [state_machine_manager](state_machine_manager/README.md) package provides
the passive mission FSM, operator start/abort/reset services, and mission-state
publication. Build with `colcon build --packages-up-to state_machine_manager`
and launch with `ros2 launch state_machine_manager mission.launch.py`. It reuses
the safety monitor's permissions and sends no PX4 flight commands.

## Structure

```text
px4_interface_node/
├── CMakeLists.txt
├── package.xml
├── msg/DroneState.msg
├── srv/SetFlightMode.srv
├── srv/ExecuteCommand.srv
├── test/test_flight_mode.py
├── tools/sitl_gcs.py
├── tools/sitl_smoke_test.py
└── src/px4_interface_node.cpp
```

## Prerequisites

- ROS 2 Humble or later with `ament_cmake`, `rclcpp`, and interface generation
- `px4_msgs` compatible with the PX4 firmware in use
- A ROS 2 workspace and a PX4 telemetry source (such as PX4 SITL)

## Build

Place `px4_interface_node` inside the `src/` directory of your ROS 2 workspace, then run from the workspace root:

```bash
source /opt/ros/humble/setup.bash  # Adjust for your ROS distribution.
colcon build --packages-up-to px4_interface_node
source install/setup.bash
```

## Run

```bash
ros2 run px4_interface_node px4_interface_node
```

Stop older interface instances before launching the rebuilt node. Rebuild and
re-source consumers of `DroneState` after changing its message definition.

## State Machine Manager API

`/drone/execute_command` uses `px4_interface_node/srv/ExecuteCommand`. Only
these commands are accepted; raw PX4 command IDs and force flags are not exposed.

| Request `command` | Meaning | `mode` |
| --- | --- | --- |
| 0 (`SET_MODE`) | Request a flight mode | Required uppercase name |
| 1 (`ARM`) | Arm normally, with readiness guards | Empty |
| 2 (`DISARM`) | Disarm normally, while landed | Empty |
| 3 (`RUN_PREFLIGHT_CHECKS`) | Ask PX4 to refresh its checks | Empty |

```bash
ros2 service call /drone/execute_command px4_interface_node/srv/ExecuteCommand "{command: 0, mode: 'LOITER'}"
ros2 service call /drone/execute_command px4_interface_node/srv/ExecuteCommand "{command: 3}"
ros2 service call /drone/execute_command px4_interface_node/srv/ExecuteCommand "{command: 1}"
ros2 service call /drone/execute_command px4_interface_node/srv/ExecuteCommand "{command: 2}"
```

The mode-only service remains available:

```bash
ros2 service call /drone/set_flight_mode px4_interface_node/srv/SetFlightMode "{mode: 'LOITER'}"
```

Supported names match `DroneState.flight_mode`: `MANUAL`, `ALTITUDE`, `POSITION`,
`MISSION`, `LOITER`, `RTL`, `OFFBOARD`, `STABILIZED`, `TAKEOFF`, `LAND`, and
`PRECISION_LAND`. PX4 still enforces its own mode prerequisites. In particular,
the interface does not generate the heartbeat/setpoints needed for OFFBOARD
and does not arm as a side effect of requesting a mode.

One request is active across both services. Their deferred responses leave
telemetry callbacks running. The generic response contains a local `request_id`,
the command kind, `success`, `accepted`, result code, raw ACK result/reason
parameters, and the last known mode/armed state plus `status_fresh`.

`accepted` means PX4 sent terminal `ACCEPTED`; `IN_PROGRESS` is insufficient.
`success` additionally requires a fresh `VehicleStatus` timestamp at least as
new as the ACK and the requested **active** `nav_state` or arming state.
`nav_state_user_intention` and cached pre-command telemetry cannot complete a
request. A preflight-check request returns success only when the updated status
reports passed checks; PX4 may accept running the checks while they still fail.

| Result | Meaning |
| --- | --- |
| 0 `SUCCESS` | Accepted and confirmed in telemetry |
| 1 `INVALID_REQUEST` | Unsupported command/mode or invalid arguments; nothing sent |
| 2 `BUSY` | Another request is pending; nothing sent |
| 3 `STATUS_UNAVAILABLE` | Status missing/stale, including loss during a request |
| 4 `REJECTED` | Terminal PX4 rejection; inspect raw ACK result and reason parameters |
| 5 `ACK_TIMEOUT` | No terminal ACK by the acknowledgment deadline |
| 6 `TRANSITION_TIMEOUT` | Accepted, but requested state not confirmed by the transition deadline |
| 7 `PREFLIGHT_FAILED` | Failed checks, safety/calibration constraint, or non-landed arm request |
| 8 `GCS_UNAVAILABLE` | Arming blocked by lost GCS connection |
| 9 `FAILSAFE_ACTIVE` | Arming or a non-recovery mode blocked by failsafe |
| 10 `LANDED_UNAVAILABLE` | Missing/stale land-detection telemetry for arm/disarm |
| 11 `UNSAFE_DISARM` | In-flight disarm request blocked |

Raw `ack_result` is 255 (`NO_ACK`) until a matching ACK is received. The legacy
mode service uses `INVALID_MODE` for result 1 and `MODE_TIMEOUT` for result 6.

## Readiness and protections

`/drone/state` now includes `failsafe`, `pre_flight_checks_pass`,
`gcs_connection_lost`, arming state, active/requested navigation states, safety
and calibration flags, vehicle IDs, `landed`, `land_detected_fresh`,
`ready_to_arm`, `arming_block_reason`, and `command_pending`.

```bash
ros2 topic echo /drone/state
```

- All commands require target `VehicleStatus` received within two seconds.
- Arming also requires no failsafe, GCS connected, PX4 preflight checks passed,
  safety off, no calibration, and fresh landed telemetry confirming the ground.
- Disarming requires fresh landed telemetry. It remains available with a lost
  GCS or failed preflight checks, so those conditions cannot prevent a safe
  ground disarm. Force arm/disarm is never sent (`param2` remains zero).
- During failsafe, mode requests are limited to recovery modes `RTL` and `LAND`.
- `ready_to_arm` is false while armed, busy, or any readiness guard fails. Raw
  status flags remain last known; consumers must honor their freshness flags.
- No automatic retries are made. A timeout does not cancel a command already
  received by PX4; inspect fresh telemetry before requesting another action.

ACKs must match the pending PX4 command ID, target this node's source IDs,
originate from PX4, and have a timestamp newer than the pre-command status.
Only one request is allowed in flight; stale and unrelated ACKs are ignored.
PX4 ACKs carry neither the requested mode/arm action nor a transaction ID, so
an arbitrarily delayed earlier ACK generated after the next status snapshot
cannot be uniquely distinguished. Reserve the source ID pair for this node;
the response's `request_id` correlates ROS requests, not the PX4 wire protocol.

## Topics and configuration

| Direction | Topic | Type |
| --- | --- | --- |
| In | `/fmu/out/vehicle_status_v4` | `VehicleStatus` |
| In | `/fmu/out/vehicle_land_detected` | `VehicleLandDetected` |
| In | `/fmu/out/failsafe_flags` | `FailsafeFlags` |
| In | `/fmu/out/vehicle_local_position_v1` | `VehicleLocalPosition` |
| In | `/fmu/out/vehicle_odometry` | `VehicleOdometry` |
| In | `/fmu/out/battery_status_v1` | `BatteryStatus` |
| In | `/fmu/out/vehicle_command_ack_v1` | `VehicleCommandAck` |
| In, when legacy headers exist | `/fmu/out/vehicle_gps_position` | `SensorGps` |
| In, otherwise | `/fmu/out/vehicle_gnss` | `VehicleGnss` |
| Out | `/fmu/in/vehicle_command` | `VehicleCommand` |
| Out | `/drone/state` | `px4_interface_node/msg/DroneState` |

PX4 message types are from `px4_msgs`. The build preserves the original GPS
interface where available; current GNSS telemetry uses `receiver` coordinates
and requires `usable`, a 3D fix, and finite coordinates. PX4 inputs use sensor
best-effort QoS; outgoing commands use reliable, volatile QoS. Use matching
firmware/message definitions, or PX4's message translation support; changing a
topic suffix alone does not convert schemas. Separate vehicles need separate
DDS namespaces and topic remappings.

Startup parameters are read-only: `target_system`/`target_component` default to
1/1, `source_system`/`source_component` to 1/191, `ack_timeout_ms` to 2000, and
`transition_timeout_ms` to 5000. IDs must be 1–255; timeouts must be 100–60000 ms.
The legacy `mode_timeout_ms` supplies the transition default unless
`transition_timeout_ms` is explicitly set. Deadlines and freshness use a
monotonic clock, including when ROS simulation time is paused. The ACK deadline
starts at publication; the transition deadline starts at terminal ACCEPTED.

## SITL readiness and verification

The observed `gcs_connection_lost: true` came from an absent MAVLink GCS link.
ROS DDS telemetry alone does not establish that connection. A live GCS link
restored `gcs_connection_lost: false`, `pre_flight_checks_pass: true`, and
`commander check: OK` in the tested SITL. Manual-control and other failed checks
may still prohibit specific modes; verify readiness again after mode changes.
Do not disable safety checks to make an arming test pass.

Use QGroundControl for normal operation. For a foreground localhost-only SITL
diagnostic link, `tools/sitl_gcs.py` requires `pymavlink`, receives diagnostics,
and sends actual MAVLink GCS heartbeats; it never fabricates DDS status flags.
If no MAVLink endpoint exists, start one in the **SITL PX4 shell**:

```text
mavlink start -u 18570 -o 14550 -t 127.0.0.1 -m minimal -r 20000
```

In a separate ROS/WSL terminal:

```bash
python3 src/px4_interface_node/tools/sitl_gcs.py --duration 180
```

The helper must remain running through the test; GCS loss returns after it
exits unless another GCS is connected. It is a bounded SITL diagnostic tool.
From disarmed, landed LOITER/Hold, with the new package sourced:

```bash
# Mode changes only, without arming:
python3 src/px4_interface_node/tools/sitl_smoke_test.py
# Also exercise a normal landed arm/disarm, gated on fresh readiness:
python3 src/px4_interface_node/tools/sitl_smoke_test.py --test-arming --report sitl-result.json
```

The smoke test checks for a local SITL process and launches a temporary interface
with `/phase3/drone/...` endpoints and source component 192. It tests
`LOITER -> LAND -> LOITER` while disarmed, then optionally runs preflight checks,
arms, and immediately disarms. Every result requires accepted ACK plus updated
state. It leaves the vehicle disarmed in LOITER and shuts down its own interface;
it does not take off. The real SITL mode/preflight/arm/disarm sequence passed
during implementation with all five responses accepted and confirmed.

## Automated tests

Install test dependencies with `rosdep`, then run:

```bash
colcon test --packages-select px4_interface_node --event-handlers console_direct+
colcon test-result --verbose
```

The launch suite exercises the real C++ node against a simulated PX4 peer,
remapping all endpoints under `/test_px4_interface`. It covers command encoding,
mode and arm/disarm confirmations, cross-command ACK filtering, rejections,
progress, timeouts, stale status/land detection, readiness guards, shared-service
serialization, paused ROS time, continued telemetry, and GPS/GNSS validity.

Protocol references: [PX4 ROS 2 guide](https://docs.px4.io/main/en/ros2/user_guide)
and [arming checks](https://docs.px4.io/v1.18/en/advanced_config/prearm_arm_disarm).

## Notes

The package contains placeholder maintainer metadata; update it before publishing
a release. Validation used ROS 2 Humble, PX4 SITL, and compatible `px4_msgs`.
