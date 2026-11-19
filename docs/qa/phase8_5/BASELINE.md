# Phase 8.5 baseline

Audited source d3a6e928a63bedc211e4689e1d5077067df44d8a. Original audit 0ca5b89ef28971f53098d861a71ba7523d065a0b lives on audit/comprehensive-qa-phase8. Its README, evidence, transition matrix, JSON metrics, and probes are preserved in the audit worktree. ROS Humble runs in Ubuntu 22.04 x86_64 WSL2 with pinned PX4 SITL 97a3c7f01793e2b0b9e5a30425b8e88e69741f27, Gazebo Harmonic and Agent v2.4.2. Physical CM5 is outside this test environment.

The PX4 adapter publishes normalized state. The safety monitor and mission core consume it, and only the command-enabled adapter in the ROS stack publishes VehicleCommand. The native Throw coordinator calls its service in SITL. Vision and recording remain separate. The CM5 profile is passive.

| Audit ID and priority | Baseline and source | Expected change and regression |
|---|---|---|
| QA-01/15 critical, QA-08 high | simulation/tools/run_scenario.py treats three armed FAULTs as successful. The transition matrix shows the native release precedes the companion tilt check. | Separate execution, scenario behavior and safety verdict. Replay historical negative fixtures and attempt fresh SITL. Preserve aircraft dynamics and the physical prohibition. |
| QA-02/05 high | px4_interface_node.cpp dispatches ARM with PX4 checks but without companion permission or mission context. ROS graph access is a separate trust boundary. | Require current safety and mission authorization at dispatch. Test direct service calls, blocked/stale/missing status and recovery restrictions. |
| QA-03/07 high | PX4 adapter refreshes cache on repeated receipt. The consumer graph can mix sources. | Gate progress per source and reject ambiguous publisher sets in sensitive paths. Test repeats, regression, clock limits and restart. |
| QA-04 high | Same-command ACK/status pair can confirm a later retry after timeout. | Fail closed on uncertain transactions and add pre-dispatch temporal barrier. Test delayed/duplicate ACKs and valid confirmations. |
| QA-06 high | Throw coordinator can remain latched pending after lost response. | Retain explicit operator-only reconciliation for uncertain arming. Review any automatic recovery change separately. |
| QA-09 medium | update.sh traps handled failures but SIGKILL leaves candidate current. | Durable activation journal and trusted startup reconciliation. Test interruption and failed rollback. |
| QA-11 medium | Vision validates camera inputs, but raw detections and TF are not a mission authorization contract. | Isolate detection validation from mission authority. Test timing, provenance, frame and exact-time transform. |
| QA-10/12/13/14/16 | Shared disk, mutable dependencies, recording provenance, duplication, and flaky fixture startup. | Keep changes scoped. Add CI and validate relevant existing suites. Record deferred work. |

Baseline reproduction from the audit worktree with the Humble overlays sourced:

```bash
python3 audit/phase8/probes/ros_boundaries.py
python3 audit/phase8/probes/deployment_recording.py
```

Existing inventory includes PX4 interface, safety policy and pipeline, mission state and native coordinator ROS tests, simulation model tests, CM5 shell/systemd/passive integration tests, and Phase 8 vision/recording tests. The original audit recorded 129 ROS checks after an isolated rerun, 14 deployment, 8 vision, and 4 model checks. Baseline SITL included nominal stabilization, safe no-release and abort, and three unsafe armed terminal outcomes.
