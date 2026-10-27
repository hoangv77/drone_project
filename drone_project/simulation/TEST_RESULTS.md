# Phase 6 validation — 2026-10-26

Environment: Ubuntu 22.04 WSL2, ROS 2 Humble, Gazebo Harmonic 8.15.0,
PX4 `v1.18.0-beta1-1026-g97a3c7f017` at
`97a3c7f01793e2b0b9e5a30425b8e88e69741f27`.
Branch: `codex/phase6-throw-launch`.

## Executed builds and checks

| Check | Observed result |
|---|---|
| Firmware-matched px4_msgs | Built 271 message schemas staged from the installed firmware; no global checkout edits |
| ROS packages | Built px4_interface_node, safety_monitor, state_machine_manager, including throw_launch_manager |
| Gazebo release plugin | Compiled and loaded against gz-sim8 8.15.0 |
| ROS tests | **129 reported tests, 0 errors, 0 failures, 0 skipped** on the final matched-schema build |
| Model/parser checks | **4 passed** |
| Gazebo SDF validation | **Valid**; upstream sensor `gz_frame_id` extensions produce schema warnings but work at runtime |
| Independent ballistic regression | **Passed**, without PX4/ROS; apex 3.988 m, request-to-apex 1.068 simulator seconds, followed by 0.960 m descent |
| CAD extraction | Open Cascade recovered transformed bounds, 11 solid volumes/centroids and an actual geometry visual; six null-triangulation faces were skipped |

The final ROS run includes the existing command ACK/telemetry tests, safety
pipeline/policy tests and mission tests, plus nine coordinator contract tests.
The new core tests include abort before the first pending-command telemetry
snapshot, ACK-gated arming and completion in airborne hold without tag search.
A startup-only race in the existing safety test was corrected by waiting for
its BLOCKED publication rather than assuming discovery implied publication.
No safety policy was relaxed.

The inspected system's existing px4_msgs had an extra `mode_not_supported`
FailsafeFlags field absent from PX4. It could not decode that telemetry.
The local schema overlay fixes the mismatch. The first Windows-filesystem
build emitted clock-skew warnings and installed very slowly; final validation
used Linux-local `/tmp/drone-phase6-*` build/install directories.

## Final force-driven physical trials

The final launch mechanism applies mass-derived hand forces for 0.2 s and
then relinquishes physics control. Native detection is observed through the
original actuator_armed lockdown state, corroborated by PX4 console events.
An independent Gazebo pose subscription verifies trajectory, tilt, speed and
absence of ground contact in nominal trials.

| Trial | Native detection after release | ROS stable hold after release | Maximum loss below release height | Maximum physical tilt | Outcome |
|---|---:|---:|---:|---:|---|
| Nominal 1 | 0.699 s | 3.271 s | 0 m | 0.1465 rad / 8.39° | Stable LOITER, STANDBY; no ground contact |
| Nominal 2 | 0.691 s | 3.327 s | 0 m | 0.1416 rad / 8.11° | Stable LOITER, STANDBY; no ground contact |
| Below threshold | Not detected | Not stabilized as a launch | 0.231 m | 0 rad | Returned to support, timeout/recovery, ABORTED and disarmed |
| No release | Not detected | Not applicable | Not applicable | Not applicable | Armed-wait timeout, normal disarm, ABORTED |
| Operator abort before release | Not detected | Not applicable | Not applicable | Not applicable | Normal disarm, ABORTED |
| Unfavorable orientation | Detected | **Failed** | Ground impact occurred | Approximately inverted | STABILIZE → RECOVERY → **FAULT** |

Nominal success fraction was **2/2** for this final configuration. Two repeats
do not establish a statistical reliability rate. Nominal peak altitudes were
6.743 m and 6.917 m; this altitude excursion and controller response still
require calibration against actual propulsion, mass and flight measurements.
The second trial independently confirmed stable simulator motion by 3.393 s
after release. Timing uses receipt times and includes transport/observer delay.

**The unfavorable-orientation trial did not recover safely.** It failed to
stabilize, requested LAND, and reached a bounded recovery timeout with PX4's
critical-flight-control flag and the monitor's EMERGENCY level. The final
reported armed state was true. The manager did not force-disarm or report a
successful launch. The test stopped its SITL processes after recording FAULT;
this outcome requires operator intervention in any future hardware design.
The native detector does not provide an orientation veto.

## Controlled ROS faults

| Scenario | Executed evidence | Scope |
|---|---|---|
| Command rejection | Service rejection stops the sequence before ARM; existing interface tests also cover rejected PX4 ACKs | Isolated ROS peer, not a real PX4 rejection injected into Gazebo |
| Safety denial | Operator start rejected; no commands emitted | Isolated safety topic plus real coordinator; existing policy tests retained |
| Telemetry/communication loss | Stale drone/native streams enter recovery without synthetic successful feedback | Isolated ROS streams; not a physical radio-loss test |
| No explicit request | No arming command or state progression | Real coordinator against isolated peer |
| Airborne without native detection | Does not establish a throw | Real coordinator against isolated peer |
| Unfavorable attitude | Cannot satisfy stabilization | Real coordinator against isolated peer |

Actual RC/GCS loss, DDS-agent failure, hardware timing, camera/Pi deployment,
wind, calibrated electrical behavior and broader release-condition sweeps
were **not validated**. No physical hardware tests were performed.

## Invalidated development attempts

The first continuous velocity hold produced zero stationary specific force;
PX4 correctly refused preflight. An installed-schema trial also remained
disarmed. A later one-step velocity-command attempt constrained subsequent
motion because Gazebo retained zeroed command components; its apparent ROS
completion was invalidated by ground truth. Removing those components proved
ballistic motion, but the instantaneous velocity reset did not supply a
realistic IMU-observable hand acceleration and allowed ground impact before
recovery. Those attempts are excluded from nominal final-configuration results.

The first orientation test runner stopped before the complete recovery
deadline. Its rerun observed the terminal FAULT described above. Initial
test failures and these harness/model corrections are not presented as
successful physical trials.

## Reproduce and inspect

Use the exact build, single-trial, suite and ballistic commands in
[README.md](README.md). Saved summaries:

* [Final trial metrics and configuration hashes](results/final_trials.json)
* [Ballistic regression](results/ballistic.json)
* [Final ROS test output](results/ros_tests.txt)
* [Native PX4 throw events](results/native_throw_events.txt)

Raw metrics, full logs and PX4 ULogs remain under the ignored `.phase6/`
directory in the repository. Each new suite run creates distinct timestamped
runtime/log/metrics files. The original CAD attachment was only read.

The geometry envelope and visual are based on the supplied CAD. The 350 g
takeoff weight, deployed motor centres, prop/motor/ESC characteristics,
battery and sensor poses are provisional. The measurement inventory and
missing specifications are in the README and `config/drone.json`; simulation
success alone does not validate these assumptions or authorize hardware use.
