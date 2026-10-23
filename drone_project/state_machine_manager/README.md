# State machine manager

A separate ROS 2 Humble C++ package for deterministic high-level mission
decisions. It reuses the repository's `px4_interface_node/msg/DroneState`,
`safety_monitor/msg/SafetyStatus`, and exported `safety_monitor::SafetyGate`.
The pure `MissionMachine` core has an injected monotonic clock and no ROS node
or flight-control dependency. The node owns subscriptions, services, timers,
state publication, and structured transition logs.

This version observes externally controlled flight stages. It cannot arm,
disarm, change modes, drive motors, or publish Offboard setpoints. There are no
PX4 command clients or FMU input publishers. `entry_action` is informational;
it is not an executable controller request. Starting a mission does not start
the motors or request any flight mode.

## Build and launch

Use the existing feature branch `hoang/implement_system_machine_manager`.
Place all three packages in a ROS workspace with firmware-compatible
`px4_msgs`, install dependencies through `rosdep`, then:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to state_machine_manager
source install/setup.bash

# Existing telemetry and safety nodes are already running:
ros2 launch state_machine_manager mission.launch.py

# Or start the existing telemetry + safety launch alongside the manager:
ros2 launch state_machine_manager mission.launch.py start_interfaces:=true
```

Launch defaults to `start_interfaces:=false` to avoid duplicate adapters.
`config:=/absolute/path/mission.yaml` selects manager parameters;
`safety_config:=/absolute/path/safety_monitor.yaml` configures the included
safety launch. Stop older adapters and source the matching overlay before
starting the full stack. The existing unversioned PX4 `FailsafeFlags` schema
must match the running firmware; see the safety monitor's compatibility notes.
Separate vehicles require explicit remapping of absolute `/drone/...` and FMU
topics, not just changing the node namespace.

## Operator API

All three services use the existing `std_srvs/srv/Trigger` interface:

```bash
ros2 service call /drone/mission/start std_srvs/srv/Trigger '{}'
ros2 service call /drone/mission/abort std_srvs/srv/Trigger '{}'
ros2 service call /drone/mission/reset std_srvs/srv/Trigger '{}'
ros2 topic echo /drone/mission_state
```

A rejected request returns `success: false` with a reason and leaves the state
unchanged. There is no arbitrary "set state" or skip-stage service. Only START
from STANDBY and RESET from ABORTED/FAULT are valid. START and RESET require
fresh launch permission and verified disarmed ground readiness. An abort while
disarmed and landed enters ABORTED. An abort while armed, airborne, with a
pending command, or without ground confirmation enters RECOVERY. Repeated abort
in RECOVERY records the operator's intent without restarting its deadline.

## Published mission state

`/drone/mission_state` uses `state_machine_manager/msg/MissionState`, at 10 Hz
and immediately after transitions/operator requests, reliable and volatile with
depth 1. It contains:

- Current/previous state, readable name, persistent transition reason, and
  monotonic state elapsed time.
- Mission ID, transition sequence, mission activity, abort intent, and outcome.
- Drone/safety receipt freshness, feedback enablement, informational entry
  action, last rejection, and rejected-request count.

Mission IDs are seeded randomly per process and increment for new missions;
transition sequence increases for every state entry. IDs remain visible after
completion/reset for correlation. Outcomes are NONE, RUNNING, SUCCEEDED,
ABORTED, or FAILED. RECOVERY remains active until observed ground completion
or timeout, but normal mission progression stops. State publication alone is
never a PX4 command or proof of physical flight behavior.

## Transition contract

| State | Entry action and allowed exit | Default deadline |
| --- | --- | --- |
| STANDBY | Wait for operator START; safe launch readiness enters PREFLIGHT | None |
| PREFLIGHT | Observe external arming; armed + landed PX4 telemetry and flight safety permission enter ARMED_WAITING | 30 s |
| ARMED_WAITING | Wait for verified THROW_DETECTED plus armed + airborne PX4 state | 30 s |
| THROW_LAUNCH | Observe external launch stage; LAUNCH_COMPLETE enters STABILIZE | 5 s |
| STABILIZE | STABILIZED plus valid PX4 position and flight permission enter SEARCH_TAG | 10 s |
| SEARCH_TAG | TAG_ACQUIRED enters TRACK_TAG | 30 s |
| TRACK_TAG | TAG_TRACKED renews tracking lease; TAG_LOST/lease expiry return to SEARCH_TAG; LANDING_READY can enter PRECISION_LAND only with visual permission and active PX4 PRECISION_LAND mode | 60 s; tracking lease 1 s |
| PRECISION_LAND | Observe external landing; fresh disarmed + landed confirmation returns to STANDBY with SUCCEEDED outcome | 60 s |
| RECOVERY | Wait for external recovery and fresh safety/ground evidence; enter ABORTED; timeout enters FAULT | 30 s |
| ABORTED | Mission stopped; operator RESET after safe readiness returns to STANDBY | None |
| FAULT | Latched fault; operator RESET after safe readiness returns to STANDBY | None |

Normal flight stages require fresh safety flight permission, fresh PX4 status,
land/estimator/failsafe data, verified armed state, finite position/velocity,
no active failsafe/critical failure/pending command, and active LOITER,
OFFBOARD, TAKEOFF, or MISSION mode. Manual takeover or native RTL/LAND stops
normal mission progression and awaits external recovery; it does not override
those modes. The precision-landing stage uses visual permission and the active
PX4 PRECISION_LAND mode instead. Ground preflight checks are not reinterpreted
as new arming requirements during established flight.

Missing/stale drone or safety information, UNKNOWN/invalid permissions, and
critical PX4 conditions stop progression. Fresh ground proof allows ABORTED;
otherwise the manager enters RECOVERY and eventually FAULT. It never sends a
recovery command. A fresh BLOCKED safety report can acknowledge a physically
disarmed/landed recovery completion, but cannot authorize RESET, START, or a
flight-stage transition. WARNING is rejected by default; optional
`allow_flight_warnings:=true` still requires the existing operation-specific
permission. Launch and visual landing always require READY.

State and safety arrive on separate topics. Arming can wait up to 1 s for the
safety monitor's updated flight-phase permission, without advancing. A newly
airborne ARMED_WAITING vehicle can wait up to 1 s for fresh throw evidence;
absence of that evidence enters RECOVERY. The manager never infers a throw from
arming or altitude alone. A global 180 s mission deadline bounds repeated tag
reacquisition. State deadlines take precedence over observations arriving at the
deadline. Terminal states require operator reset and never auto-resume.

Entry actions reset the state's deadline, feedback queue, observation sequence,
and relevant tracking/handoff leases. Each tick advances at most one state.
Ground completion is an explicit exception to operator reset for a successful
PRECISION_LAND: it returns to STANDBY with SUCCEEDED rather than labeling a
successful landing ABORTED. All actual flight controllers remain external.

## Future stage adapters

`/drone/mission/stage_feedback` uses `StageFeedback`. It is disabled by default
(`enable_stage_feedback: false`), including in the supplied launch config.
Only explicitly validated integrations should enable it. Its event types are
THROW_DETECTED, LAUNCH_COMPLETE, STABILIZED, TAG_ACQUIRED, TAG_TRACKED, TAG_LOST,
LANDING_READY, and STAGE_FAILED.

An adapter must echo the current mission ID and transition sequence, supply a
strictly increasing nonzero observation sequence for that state entry, and set
`verified` only for a validated observation. Feedback is queued and evaluated
on the next tick against current PX4 state and SafetyGate permissions. It
cannot force a transition, skip states, override safety, or complete recovery.
Wrong missions/entries, duplicates, reordered observations, unverified events,
and unknown event codes are rejected. Entry changes clear queued observations;
STAGE_FAILED takes priority over positive observations. Negative failure/loss
events remain latched for the current entry across executor delays. Positive
feedback has a 500 ms receipt lease, a queue bound of 16, and separate tracking heartbeat expiry.

The message timestamp is for diagnostics; expiry uses local monotonic receipt
time. Sequence/entry checks do not authenticate ROS publishers or prove the age
of a delayed newly received observation within the same entry. Future adapters
must validate sensor capture time, tracking confidence, controller readiness,
and transport delay themselves. Use depth 1, volatile QoS and trusted producers.
Recovery and reset rely on telemetry, not an adapter's claim of completion.

A future PX4 command adapter should use the existing ExecuteCommand API and
confirm ACK plus updated telemetry before reporting stage completion. Throw
detection, native throw-launch configuration, stabilization control, AprilTag
tracking, the landing envelope, and precision-landing control are not implemented
here. In particular, today's safety monitor always publishes
`visual_landing_permitted: false`, so real precision landing stays blocked.
The future visual branch is tested with synthetic permitted observations only.

## Tests and verification

```bash
colcon test --packages-select px4_interface_node safety_monitor state_machine_manager \
  --event-handlers console_direct+
colcon test-result --verbose
```

Unit tests inject time directly and cover every state, valid and invalid
transitions, telemetry/safety freshness, raw PX4 guards, arming handoff,
disabled/stale/replayed feedback, stage failure, tag loss, every stage deadline,
mission deadline, abort, recovery timeout, and safe fault reset. DDS tests run
the actual node with isolated inputs and operator services, including a paused
ROS clock, default-disabled flight-stage feedback, and graph checks proving no
FMU command publishers or PX4 command clients exist. Existing package tests
remain part of regression validation.

Development validation built all three packages on ROS 2 Humble and passed 116
reported checks with zero failures. A separate installed C++ consumer compiled
and ran against both exported libraries/headers, and launch arguments resolved.
With firmware-matched message definitions, live PX4/Gazebo SITL supplied fresh
telemetry and safety data; START was correctly rejected for the existing lost
operator links and failed preflight checks. Stopping the temporary safety node
made the manager mark safety stale and continue to reject START. No flight
commands were sent. Healthy/future flight-stage transitions were exercised only
with isolated synthetic inputs, not by physically launching or flying SITL.

This is not a validated throw-launch or landing controller. Actual throw,
stabilization, vision-guided flight, precision landing, hardware fault response,
and external recovery actions require subsequent controller integration and
separate SITL/hardware validation. Do not enable future-stage input to simulate
health on a connected vehicle; use isolated test topics/domains.
