# Phase 8.5 executed results

Ubuntu 22.04 x86_64 WSL2 with ROS 2 Humble, pinned PX4 source and pinned Agent v2.4.2. The build reused the existing audited message overlay as an underlay. Colcon warned of an RPATH ordering cycle caused by the prior full overlay. The remediated packages built successfully. The CI workflows and target ARM64 systemd migration have not run on this machine.

| Run | Observed result |
|---|---|
| Original audit probes on audited build | Reproduced stale-timestamp freshness, old ACK/status retry confirmation, direct ARM despite BLOCKED safety, and SIGKILL leaving the candidate selected. The mock endpoint was isolated. |
| Intermediate ROS suite | 131 reported, zero errors, failures or skips. |
| Full ROS run after clock continuity and authority tightening | 132 reported, zero errors, 4 reported failures. Two underlying assertions were the DISARM timeout encountering a clock discontinuity, and the Throw test fixture denying START. CTest and child xUnit count the same failures separately. |
| First isolated ROS rerun | 134 reported, zero errors, 4 reported failures. Two underlying assertions were an expected mock command not dispatched during the stale ACK test, and the safety fixture observing a BLOCKED report before the expected land source expiry. The new old-ACK/status replay, safety writer restart/conflict, and replayed mission input tests passed. |
| Final isolated ROS build and regression | 134 reported, zero errors, failures or skips. The safety fixture now waits for the specific missing-land-source reason. The mock fixture reports the actual service denial when dispatch is expected. |
| Safety-monitor shutdown investigation | Whole-package ASan/UBSan builds completed, but initial tests failed in ASan runtime loading inside the Python launcher. Preloading did not resolve that instrumentation failure. With instrumentation limited to the safety executable, three isolated runs each reported 34 checks with zero errors, failures or skips. The historical teardown SIGSEGV remains an unresolved intermittent finding. |
| CM5 deployment and systemd | 19 passed, zero skips. Includes mock activation failure, live process kill with journal reconciliation, real WSL user systemd restart and passive ROS interlock. |
| Vision and recording | 10 passed, zero skips in the full WSL run, including existing bag playback, actual AprilTags and video. The additional ROS integration test passed separately. It observed raw AprilTags while body TF was absent, then saw the validated stream after supplying the measurement-time transform. |
| Simulation model and verdict fixtures | 7 passed in WSL, zero skips. Windows-only rerun skipped the one PX4 source-dependent model test. |
| Historical armed-FAULT replay | 3 of 3 fail the safety verdict even though their original process exit codes were zero. Terminal freshness remains unknown in the compact historical evidence. |
| First fresh hardened SITL suite | All five trials exited nonzero before release. The model and PX4 parameters were preserved. |
| Syntax | Python compilation, Bash parsing, GitHub workflow YAML and evidence JSON checked locally. The Windows copy of the inherited simulation shell files was normalized for the Bash syntax check. |

## Fresh SITL outcomes

| Scenario | Fresh outcome |
|---|---|
| Nominal | PX4 acknowledged RUN_PREARM_CHECKS, then reported failed preflight. Coordinator RECOVERY → ABORTED, disarmed. No release. |
| Unfavorable orientation | Same prearm rejection. No adverse release exercised in this run. |
| Below threshold | Same prearm rejection. No sub-threshold release exercised in this run. |
| No-release deadline | Same prearm rejection. The armed no-release deadline was not reached. |
| Operator abort | Same prearm rejection. The armed abort path was not reached. |

The first fresh suite is evidence of the rejection path only. It does not establish end-to-end nominal compatibility or reproduce the hazardous physical trajectories on the remediated build. Local run logs and raw observations remain in /var/tmp/drone-phase85-sitl.ghdq6Y7w. See [historical replay](HISTORICAL_VERDICTS.json) and [investigation](THROW_LAUNCH_INVESTIGATION.md).

## Nominal comparison and clock probe

An additional nominal baseline run stabilized with exit 0. The hardened run aborted disarmed before release after PX4 reported failed prearm checks. The comparison leaves nominal end-to-end compatibility open. It does not establish the sole cause of the differing preflight result.

The hardened passive SIH verification confirmed disarmed and landed but failed the all-stream freshness check. It stopped before the Agent interruption phase. An independent 25-second raw DDS probe saw new samples on all seven streams and a simultaneous jump to approximately 1.7 seconds ahead of the WSL system clock. The lowest observed age was −1739.82 ms on odometry. The source-time gate intentionally refused those future observations. A production clock guarantee and recovery procedure require an isolated lab and target-hardware evidence. [Fresh SITL results](FRESH_SITL_RESULTS.json)
