# Safety monitor

This C++ ROS 2 node consumes `/drone/state` and publishes
`safety_monitor/msg/SafetyStatus` on `/drone/safety_status` at 10 Hz. It grants
permissions for high-level transitions and never sends PX4 commands or changes
PX4 safety settings.

## Build and run

Place both packages and compatible `px4_msgs` in your ROS workspace, install
dependencies with `rosdep`, then build and source the entire overlay:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to safety_monitor
source install/setup.bash
ros2 launch safety_monitor safety.launch.py
ros2 topic echo /drone/safety_status
```

Stop an older interface before launching the pair. `DroneState` has changed;
rebuild and re-source every consumer. To run only the monitor alongside a rebuilt
interface, use `ros2 run safety_monitor safety_monitor`. Launch accepts
`config:=/absolute/path/to/safety_monitor.yaml` for a custom configuration.

## Policy

All permissions default to false. Before the first state, level is `UNKNOWN`.
Every decision requires a recent adapter message plus fresh status, battery,
local position, land detection, and failsafe flags. Missing telemetry is
`BLOCKED` on the ground and `EMERGENCY` after the vehicle has been armed or
observed airborne. Only fresh **disarmed and landed** confirmation clears this
flight latch. Elapsed-time checks and the publication timer use a monotonic
clock, so loss detection continues with a paused `/clock`.

| Condition | Ground / launch | Armed or airborne |
| --- | --- | --- |
| Failed preflight, safety on, calibration | Block launch | Not a new in-flight preflight requirement |
| PX4 failsafe, critical control failure, geofence breach | BLOCKED | EMERGENCY; defer to PX4 recovery |
| Missing/invalid position, required operator link, or telemetry | BLOCKED | EMERGENCY |
| Missing required GNSS | BLOCKED | BLOCKED; no forced emergency landing |
| Battery below 30%, above/equal 20% | Block launch | WARNING; flight permission can remain true |
| Battery below 20%, above 10% | BLOCKED | BLOCKED; abort normal mission transitions |
| Battery at/below 10%, PX4 critical/emergency/failed/unhealthy warning | BLOCKED | EMERGENCY |
| PX4 LOW warning with adequate reserve | Block launch | WARNING |
| Charging or unknown battery warning code | BLOCKED | BLOCKED |
| Manual/operator flight mode | Launch only in configured POSITION or LOITER | Block autonomy, respect takeover |
| PRECISION_LAND | Block launch | Block visual guidance until tracker/envelope integration |

Thresholds are fractions of remaining charge, not percentages or voltage. Values
are conservative development defaults and need vehicle-specific validation.
PX4's warnings, battery faults/power-off indication, and low-remaining-flight-time/health flags take precedence over a
healthy percentage. Warning codes are evaluated individually: `CHARGING=7` is a
state and must not be ordered above `EMERGENCY=3`.

Ground permission requires fresh landed data, disarmed state, passed preflight,
safety off, no calibration, adequate battery reserve, and a permitted launch
mode. It does not mean a throw launch has been validated. Autonomous flight
permission additionally requires confirmed armed state and one of LOITER,
MISSION, OFFBOARD, RTL, TAKEOFF, or LAND. OFFBOARD checks the PX4 signal-loss flag;
RTL checks home validity. PX4 still owns each mode's complete prerequisites.

Phase is inferred from armed/landed state and the **active** mode. It describes
the observed state, not the state machine's requested next phase. Consumers must
check the permission for the transition they want. AprilTag tracking, rangefinder
quality, and the landing envelope are deferred; `visual_landing_permitted` is
always false with an explicit explanation. Normal PX4 LAND remains distinct from
future visual guidance.

`reason` explains the current phase. Each permission also has a separate block
reason. `READY` means the current phase checks passed; it does not make every
operation permissible. `WARNING` requires explicit state-machine policy for
continuation. `BLOCKED` or `EMERGENCY` stops normal mission progression; neither
means a consumer should suppress PX4 recovery, ground disarm, or an independently
validated recovery procedure.

Startup-only parameters are in `config/safety_monitor.yaml`. GCS, RC/operator
control, and GNSS are all required by default. RC readiness uses PX4's aggregated
`FailsafeFlags.manual_control_signal_lost`, not receiver RSSI or an assumed ELRS
connection. Simulation without RC, or an indoor mission without GNSS, must make
that requirement optional explicitly in a separate configuration. Missing
failsafe telemetry is always blocked even if a particular link is optional.
Mode lists can narrow the supported launch/autonomy sets, not add unsafe modes.
Changing monitor policy does not configure or disable PX4's onboard failsafes.

For multiple vehicles, isolate FMU and drone topics with DDS namespaces/remaps.
Failsafe/land/battery messages do not carry the VehicleStatus system/component
IDs used by the adapter's status filter.

`FailsafeFlags` is unversioned in the tested firmware. Its definition must match
that firmware exactly; a topic name appearing in `ros2 topic list` does not prove
messages can be decoded. During development, the local `px4_msgs` checkout had
an extra `mode_not_supported` field absent from the running firmware, preventing
delivery of failsafe flags. A separate validation overlay with the firmware's
matching definition restored delivery. Keep your workspace's `px4_msgs` aligned
with the selected firmware and rebuild the dependency and both nodes; do not
remove the freshness requirement to work around a schema mismatch.

## State machine integration

There is no `state_machine_manager` package in this repository yet. The exported
`safety_monitor/safety_gate.hpp` implements its transition gate, including a
500 ms receipt timeout. Add `safety_monitor` as a dependency and subscribe with
depth 1, volatile QoS. Keep the subscription alive as a node member:

```cpp
safety_monitor::SafetyGate safety_gate_;
rclcpp::Subscription<safety_monitor::msg::SafetyStatus>::SharedPtr safety_sub_;

// In the state-machine constructor:
safety_sub_ = create_subscription<safety_monitor::msg::SafetyStatus>(
    "/drone/safety_status", rclcpp::QoS(1).reliable().durability_volatile(),
    [this](safety_monitor::msg::SafetyStatus::SharedPtr status) {
      safety_gate_.receive(*status);
    });

// Evaluate immediately before each transition and during normal mission ticks:
if (!safety_gate_.permits(safety_monitor::SafetyGate::Operation::Launch)) {
  return;  // Keep launch blocked; retain/report the received block reason.
}
```

Use `AutonomousFlight` or `VisualLanding` for their respective operations. The
gate rejects missing/stale/unknown/blocked/emergency messages and false
permissions. Passing `true` as the second argument opts into WARNING only when
that operation's permission remains true. Receipt expiry is evaluated on every
call, even when no new safety message arrives. A state machine must also handle
blocked/emergency progression explicitly; a gate does not implement recovery.

## Tests and SITL

```bash
colcon test --packages-select px4_interface_node safety_monitor --event-handlers console_direct+
colcon test-result --verbose
```

C++ tests cover policy and consumer expiry. The launch test drives the real
interface and monitor from isolated mock PX4 topics, testing healthy readiness,
preflight/link/estimator/battery failures, mode changes, source loss, adapter
loss, and phase retention while ROS time is paused. All test command endpoints
are remapped away from `/fmu/in`.

With local PX4 SITL, Gazebo server, and MicroXRCEAgent already running, observe
the actual telemetry without issuing flight commands:

```bash
ros2 run safety_monitor sitl_observe.py --report sitl-safety-result.json
```

The tool runs temporary nodes on `/safety_sitl_validation/...`, checks live
safety-source freshness, confirms missing operator links deny launch, then stops
only its own adapter to check that all permissions expire. It leaves PX4 and
Gazebo running. The GUI is optional. Live SITL may correctly report BLOCKED
until a genuine operator/GCS link and PX4 preflight readiness are established.
Do not publish fabricated healthy flags onto actual FMU topics for testing.

Development validation passed the healthy/fault scenarios over isolated DDS.
With firmware-matched messages, the live Gazebo/PX4 SITL reported fresh safety
telemetry and correctly blocked launch because RC/GCS links were lost and PX4
preflight checks had not passed. Stopping the temporary adapter then denied all
permissions. This observation test did not arm, fly, or alter PX4 parameters.

For hardware, independently configure and validate PX4's companion/offboard,
RC, data-link, battery, and estimator-loss responses, and the physical operator
intervention path. Those protections remain on the flight controller if the
companion loses power. This monitor does not validate those physical protections.
See [PX4 safety configuration](https://docs.px4.io/main/en/config/safety) and
[ROS 2 message compatibility](https://docs.px4.io/main/en/ros2/user_guide).
