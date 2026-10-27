# Phase 6: native PX4 throw launch in SITL

This integration is opt-in and uses the existing safety policy, mission core,
and guarded command service. The normal passive mission launch is unchanged.
No physical configuration is provided. Hardware throw launch requires separate
measurement, propulsion validation, and operator approval.

The attached STEP is a mechanical assembly, **not a full aircraft specification**.
The model uses its envelope but has provisional deployed propulsion geometry
and a provisional mass budget. A successful run does not validate the hardware.

## Audited environment and existing contracts

Inspected Ubuntu 22.04 WSL2, ROS 2 Humble, Gazebo Harmonic **8.15.0**,
PX4 **v1.18.0-beta1-1026-g97a3c7f017** at
`97a3c7f01793e2b0b9e5a30425b8e88e69741f27`.
The firmware is a development revision, not a released 1.18 tag. The start script
checks this commit; audit source and message schemas before changing it.

* `px4_interface_node`: `/drone/execute_command` serializes whitelisted commands;
  success requires terminal ACCEPTED plus fresh PX4 telemetry at/after the ACK.
  It checks preflight, safety switch, calibration, landed state and link status;
  force arming/disarming is unsupported. No changes to this interface were needed.
* `safety_monitor`: phase-specific launch/autonomous permissions, independent
  source freshness, GNSS, GCS, RC and battery checks. Its policy is unchanged.
* `state_machine_manager`: deterministic mission core; passive defaults remain.
  `throw_launch_manager` owns that same core and the same operator services.
  Run exactly one manager at a time.
* Existing simulator assets are PX4's `x500`/`x500_base` with airframe 4001.
  This project uses 4001 for standard quad controller/sensor startup, overrides
  geometry/allocation/motor scaling locally, and supplies a standalone SDF.
  No global source files, model files or hardware parameter stores are edited.

## Measurements and missing specifications

`config/cad_inventory.json` records the STEP SHA256, transformed bounds, solid
volumes and centroids, reproduced with Open Cascade 7.9.3.1. STEP length units
are millimetres. There are 11 solids, including two tiny geometry fragments;
solid count does not mean 11 independently specified flight components.

| Property | Available value / initial model | Classification and limitation |
|---|---|---|
| Assembly envelope | 80 × 150 × 25.5 mm; x ±40, y ±75, z −3.5 to 22 mm | Calculated from supplied transformed CAD geometry; deployed extent unknown |
| Part names | Drone Assembly, Drone_Arm_Horiz, BearingLower, Component9 | CAD metadata; no motor or battery identification |
| Material | Nylon 12 (Formlabs Fuse 1), 1,015 kg/m³ | CAD property attached only to Component9; not measured whole-frame density |
| All-up mass | Missing; initial 0.350 kg | Assumed: frame 80 g, electronics 50 g, battery 140 g, four motors 17 g, four props 3 g |
| Deployed motor centres | Missing; x ±55, y ±100, z 25 mm in FLU | Assumed X arrangement, 228.3 mm diagonal wheelbase; must be measured |
| Centre of gravity | Missing; calculated from the assumed mass budget | Base aggregate z 21.12 mm; prop masses remain separate SDF links |
| Inertia | Missing; component box + parallel-axis calculation | Calculated from provisional masses/positions; not CAD material inertia or measured inertia |
| Propeller | Missing; diameter 76.2 mm (3 in) | Assumed; pitch and manufacturer unknown |
| Motor/ESC | Missing; kT=1.8e−6 N/(rad/s)², kM=0.016 m, max 1000 rad/s | Assumed; 25/50 ms spool-up/down, unvalidated thrust/current/lag |
| Battery | Missing; 140 g, 60 × 30 × 25 mm, centre z=30 mm | Assumed mechanical properties; electrical capacity/cell count/sag unknown |
| Sensors | Missing; pose at body origin | Assumed pose; sensor schemas/noise copied from installed PX4 x500 base |
| Aerodynamics | Missing | Provisional rotor drag, no calibrated body drag, wind or ground effect |

No manufacturer specifications or measured flight weights were supplied.
Obtain actual all-up weight, **deployed** motor centres/thrust directions,
component masses/poses, CG, prop diameter/pitch, motor KV/thrust/current curve,
ESC limits/response, battery specs and flight-controller/sensor orientation.
The density of one CAD part cannot determine total aircraft mass. Unknown CAD
infill, assembly overlap and material assignments prevent credible mass inference.

Edit `config/drone.json` as measurements become available. The generator records
assumptions and sensor-source SHA256 in `model_report.json`. Battery/electronics/
motor masses are aggregated into base inertial properties; they are not counted
again as independent masses. Four prop masses and their inertias are separate.
`I = sum(I_component + m*((r dot r)*identity - r*r_transpose))` about the computed CG.
The visual is a metre-scaled tessellation of the actual STEP assembly; Open
Cascade skipped six faces with null triangulation. Collision uses the CAD
bounding box, so neither the visual nor collision implies a watertight or
aerodynamically validated mesh. No x500 mass, inertia or rotor position is reused. Standard controller gains still
require identification/tuning against this smaller vehicle.

## Native throw behavior and state sequence

Verified in the installed source:
`src/modules/commander/MulticopterThrowLaunch/MulticopterThrowLaunch.{cpp,hpp}`,
`src/modules/commander/Commander.cpp`, `msg/ActuatorArmed.msg`, and
`src/modules/uxrce_dds_client/dds_topics.yaml`.
Native parameters are `COM_THROW_EN=1`, `COM_THROW_SPEED=5.0` m/s.
PX4 arms in actuator lockdown, observes speed magnitude strictly greater than
the threshold, then releases lockdown after positive NED vz (falling).
It does **not** use a ROS motor-level detector. Unfavorable orientation is not
necessarily rejected by this native algorithm; evaluate its recovery outcome.

The default DDS bridge does not export `actuator_armed`. `native_observer.py`
uses the existing POSIX SITL `px4-param` and `px4-listener` clients read-only;
it checks throw enable and publishes native armed/lockdown/kill/termination
with the original uORB timestamp. Replayed timestamps do not renew freshness.
This avoids a global DDS configuration edit or a firmware rebuild. Detection
time is observed lockdown release at the apex, not the earlier speed crossing.

An explicit `/drone/mission/start` request is required. The sequence is:

`STANDBY → PREFLIGHT → ARMED_WAITING → THROW_LAUNCH → STABILIZE → STANDBY`.

PREFLIGHT requests prearm checks, LOITER, then ARM via the existing service.
Fresh safety and PX4 readiness guard each step; ARMED_WAITING additionally
requires successful arming service confirmation. Native armed lockdown must be
seen before its release can prove detection, and airborne PX4 telemetry must
corroborate it. Stabilization requires one continuous second of tilt <0.17 rad,
speed <0.7 m/s, valid position, LOITER, and current safety permission.
Airborne STANDBY denotes completion of this phase while **PX4 continues LOITER**.
It cannot start another launch while armed/airborne. Land using the existing
LAND command service before shutdown. No tag stage is entered.

Timeouts: command response 9 s, preflight 30 s, armed wait 15 s, native status
1.2 s, drone/safety stream 0.5 s, airborne/native handoff 2.5 s, launch 5 s,
stabilization 15 s, recovery 30 s.
Abort/failure enters recovery: normal DISARM only when fresh native lockdown
and landed state confirm pre-release; otherwise request LAND using fresh PX4
status. Never force-disarm in flight. On lost communication, no successful
recovery is assumed; PX4's existing onboard failsafes remain responsible.
Failed recovery requests are bounded to three attempts. No failsafe is disabled.

## Build on Ubuntu 22.04 / ROS 2 Humble

The installed `~/ros2_ws` message package was checked and its FailsafeFlags
schema differs from this firmware (extra `mode_not_supported` field). Stage a
project-local package using exact firmware definitions first. Avoid discovering old nested
validation workspaces by listing the three package paths explicitly.

```bash
cd /mnt/c/dev/drone_project   # or the repository root on native Ubuntu
source /opt/ros/humble/setup.bash
python3 drone_project/simulation/tools/prepare_px4_msgs.py \
  --px4 "$HOME/PX4-Autopilot" --template "$HOME/ros2_ws/src/px4_msgs" \
  --output .phase6/matched-src/px4_msgs
colcon build --base-paths .phase6/matched-src/px4_msgs \
  --build-base /tmp/drone-phase6-msgs-build --install-base /tmp/drone-phase6-matched-install \
  --packages-select px4_msgs
source /tmp/drone-phase6-matched-install/setup.bash
rosdep install --from-paths drone_project/px4_interface_node \
  drone_project/safety_monitor drone_project/state_machine_manager --ignore-src -r -y
colcon build --base-paths drone_project/px4_interface_node \
  drone_project/safety_monitor drone_project/state_machine_manager \
  --build-base /tmp/drone-phase6-final-build --install-base /tmp/drone-phase6-final-install \
  --packages-select px4_interface_node safety_monitor state_machine_manager \
  --executor sequential --cmake-args \
  -Dpx4_msgs_DIR=/tmp/drone-phase6-matched-install/px4_msgs/share/px4_msgs/cmake
source /tmp/drone-phase6-final-install/setup.bash
cmake -S drone_project/simulation/gz -B .phase6/gz-build
cmake --build .phase6/gz-build -j4
```

Gazebo Harmonic development packages (`libgz-sim8-dev`) and `pymavlink` must be
available; this machine already has them. The existing built PX4 SITL binary
must be present at `$PX4_ROOT/build/px4_sitl_default/bin/px4`.
Message staging requires a new output directory. On repeat builds, reuse the
staged directory only if its schema manifest matches the same firmware source.
Linux-local `/tmp` build/install paths avoid very slow WSL writes to `/mnt/c`;
they are reproducible build products and may need rebuilding after cleanup.

## Run the full stack and explicit release

Stop other SITL instances, agents and mission managers first. The stack uses
localhost MAVLink and the default local DDS domain. Do not connect a physical
vehicle to that domain. The foreground operator peer sends a GCS heartbeat and
neutral simulated joystick at 25 Hz so normal link checks can pass; it neither
fabricates telemetry nor disables link-loss handling.

```bash
export PX4_ROOT="$HOME/PX4-Autopilot"
export MICRO_XRCE_AGENT="$HOME/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent"
# Starts simulator, agent, operator link, telemetry, safety and throw manager.
# This alone never requests arming or release. Ctrl-C stops child processes.
python3 drone_project/simulation/tools/run_stack.py --scenario valid
```

In another sourced terminal, explicitly request and measure the selected test:

```bash
python3 drone_project/simulation/tools/run_scenario.py --scenario valid \
  --output .phase6/sim/valid-metrics.json
```

Alternatively `run_stack.py --scenario valid --test` starts the stack and
explicitly authorizes that test's launch request. It stops children when the
test finishes. For manual operation use the existing Trigger start/abort/reset
services, then `gz service -s /phase6/release --reqtype gz.msgs.Boolean
--reptype gz.msgs.Boolean --timeout 2000 --req 'data: true'` only after
ARMED_WAITING. The release service controls only Gazebo physics; authorization
is enforced by the scenario runner/ROS launch manager, not by Gazebo Transport.

For separately managed components:

```bash
bash drone_project/simulation/tools/run_sitl.sh valid
# In separate sourced terminals:
"$MICRO_XRCE_AGENT" udp4 -p 8888
python3 drone_project/simulation/tools/operator_link.py
ros2 launch state_machine_manager throw_sitl.launch.py \
  px4_bin:="$PX4_ROOT/build/px4_sitl_default/bin" \
  observer_script:="$PWD/drone_project/simulation/tools/native_observer.py"
```

The world provides a narrow physical pedestal at 1.2 m, allowing stationary
IMU specific force. When triggered, the plugin lifts the initial pose 5 cm to
clear contact and applies the specified orientation. It then supplies **a 0.2 s
hand push through world forces at each link's centre of mass**, using
`F_i=m_i*(v_release/0.2 - gravity)`. At the end it stops applying forces and
publishes `/phase6/released` with the simulator release time. The IMU observes
the acceleration; no PX4 estimator messages are injected. This avoids both a
non-observable instantaneous velocity reset and retained zero-velocity commands.

For the provisional 0.350 kg model and [0.4,0,6.5] m/s world ENU velocity,
release momentum is [0.14,0,2.275] N·s and average hand force is approximately
[0.7,0,14.805] N including gravity compensation. The push raises the aircraft
about 0.65 m, so nominal release altitude is approximately 1.9 m above ground.
The duration, 5 cm clearance, uniform acceleration, initial orientation reset
and absence of hand compliance/rotational impulse are documented assumptions.
The release is one-shot per world; after it the plugin exerts no forces or
velocity constraints.

## Scenarios and validation

Each physical scenario requires a fresh simulator/world so native throw state,
EKF, fixture and parameters reset. `run_stack.py` creates a fresh runtime per run.

`python3 drone_project/simulation/tools/run_suite.py --output .phase6/suite.json`
runs two nominal trials and the four physical negative scenarios in fresh worlds
and records their observed results and nominal success fraction. Run the ROS
contract tests below for the remaining three controlled fault scenarios.

| Scenario | Repeatable mechanism | Expected evaluation |
|---|---|---|
| valid | 1.2 m fixture, ~1.9 m release, 6.5 m/s up, small roll/pitch | Native detection + stable LOITER without ground contact |
| below_threshold | 0.6 m fixture, 2 m/s up | Predicted pre-impact speed <5 m/s; no native detection; do not count impact-triggered detection as success |
| unfavorable_orientation | 2.8 rad roll, valid speed | Record native detection and stabilization/recovery; native detector has no orientation veto |
| launch_timeout | Do not release | Timeout, recovery/disarm |
| operator_abort | Abort in ARMED_WAITING, no release | Normal disarm, ABORTED |
| command_rejection | Isolated ROS service peer returns rejection | No subsequent ARM; contract test |
| safety_denial | Isolated safety peer denies permission | Start rejected; no command; contract test |
| communication_loss | Stop isolated drone/native stream | Recovery; no fabricated successful transition; contract test |

The latter three are controlled ROS contract tests, **not claims of PX4/Gazebo
fault-injection coverage**. Their scenario entries intentionally refuse physical
release. For RC/GCS loss with real SITL, stop `operator_link.py` and record native
failsafe behavior; do not alter failsafe parameters. The ROS tests also cover
no implicit arming, unfavorable attitude and native-observer loss. Existing
mission tests cover per-stage deadlines and recovery deadlines.

```bash
python3 -m unittest discover -s drone_project/simulation/test -v
colcon test --build-base /tmp/drone-phase6-final-build --install-base /tmp/drone-phase6-final-install \
  --packages-select px4_interface_node safety_monitor state_machine_manager \
  --event-handlers console_direct+
colcon test-result --test-result-base /tmp/drone-phase6-final-build --verbose
python3 drone_project/simulation/tools/prepare_model.py --px4 "$PX4_ROOT" \
  --output .phase6/sim --scenario valid
gz sdf -k .phase6/sim/models/drone_phase6/model.sdf
```

Metrics JSON includes native detection, release-to-detection, release-to-stable,
altitude loss and maximum tilt, plus samples. Detection timing uses monotonic
receipt times and includes transport/observer latency. Altitude and tilt metrics
use an independent Gazebo pose subscription; estimator altitude loss is kept
separately. A valid trial additionally requires a rising throw trajectory,
one second of stable simulator motion, and no ground contact before completion.
Each stack run saves separate timestamped logs and metrics. Aggregate success rate only across
completed comparable trials; missing data is null. ULog files remain under the
project runtime for independent native-state review. Variants are controlled by
`config/scenarios.json`; record the file and mass configuration for every series.
See `TEST_RESULTS.md` for executed results and remaining validation limits.

The independent ballistic regression runs in a separate Gazebo Transport
partition, without PX4 or ROS, and checks apex altitude/time and subsequent fall:

```bash
python3 drone_project/simulation/tools/verify_release_physics.py \
  --px4 "$PX4_ROOT" --plugin-dir .phase6/gz-build \
  --output .phase6/ballistic-result.json
```

The scenario/ballistic tools require Gazebo's Python transport/message bindings
(`python3-gz-transport13`, `python3-gz-msgs10`), available in the inspected setup.

Optional CAD inventory regeneration (not required to run simulation):

```bash
python3 -m pip install --target .cad-deps cadquery-ocp==7.9.3.1.1
PYTHONPATH="$PWD/.cad-deps" python3 drone_project/simulation/tools/extract_cad.py \
  '/mnt/c/Users/hoang/Downloads/Drone Assembly.step' \
  --output drone_project/simulation/config/cad_inventory.json \
  --mesh drone_project/simulation/models/drone_phase6/meshes/frame.stl
```

References: [PX4 native throw guide](https://docs.px4.io/main/en/flight_modes_mc/throw_launch.html),
[audited native detector source](https://github.com/PX4/PX4-Autopilot/blob/97a3c7f01793e2b0b9e5a30425b8e88e69741f27/src/modules/commander/MulticopterThrowLaunch/MulticopterThrowLaunch.cpp).
Sensor blocks are derived from the installed PX4 x500 base model; PX4's BSD
license and the upstream source revision apply to that sensor definition data.
