# Drone autonomy development roadmap

Updated: 2026-11-09. This roadmap distinguishes code implementation from verified readiness. See [CM5 deployment](../deployment/cm5/README.md), [vision](../drone_project/vision/README.md), and [Phase 8.5 QA PR](https://github.com/hoangv77/drone_project/pull/8).

## Mission and safety boundaries

Target architecture: PX4 flight controller owns stabilization and failsafes; CM5 runs ROS 2 autonomy; tablet QGroundControl provides GCS; RadioMaster Pocket with a compatible receiver provides independent manual control/recovery. Manual override must suppress autonomy until explicitly reauthorized. **Physical Throw Launch remains BLOCKED** pending verified orientation and recovery outcomes. No propellers during initial CM5/PX4 bench work.

## Status and gates

| Phase | Scope | Status | Exit criterion |
| --- | --- | --- | --- |
| 1–3 | PX4 telemetry, ROS interface, guarded command path | Implemented | Regression tests and matched PX4 firmware schema |
| 4–5 | Safety monitor and passive mission manager | Implemented | Safety/freshness and passive-command rejection tests |
| 6 | Native PX4 Throw Launch + Gazebo SITL | Implemented; safety-blocked | All orientation, below-threshold, stabilization and abort cases end in demonstrably safe state |
| 7 | CM5 deployment, services, updates and rollback | Implemented; physical validation pending | ARM64 installation, power/thermal, boot/reboot, PX4 link, rollback and passive checks |
| 8 | Calibrated AprilTag vision, ROS bag and video | Implemented; physical calibration pending | Measured camera calibration/TF, latency, pose accuracy and storage checks |
| 8.5 | Safety remediation and audit | Code and QA commits present; release gates unresolved | Re-run complete tests and close remaining unsafe outcomes with evidence |
| 9 | Tag selection, filtering and tracking | Planned | Reject stale/out-of-order/missing TF/invalid detections; validated offline |
| 10 | PX4 offboard guidance | Planned | Bounded setpoints, watchdogs, authority arbitration, failsafe SITL tests |
| 11 | Integrated mission state flow | Planned | Deterministic transitions and aborts from launch to precision landing |
| 12 | End-to-end SITL | Planned | Nominal and fault-injection evidence with safe terminal states |
| 13 | Final software QA and release | Planned | Reproducible build, CI, audit, signed-off release gates |
| 14 | CM5 hardware integration and staged flight testing | Pending | Props-off bench, controlled manual flight, supervised autonomy; Throw Launch only after independent approval |

## Immediate critical path

1. **Reassess Phase 8.5** on the current branch/PR: run regression, adversarial SITL and recovery checks. Do not mark complete from commits alone. Specifically verify command authorization at the PX4 dispatch boundary, freshness/replay and ACK handling, input provenance, exact-time TF, deployment recovery and truthful SITL exit codes.
2. **Resolve the Phase 6 Throw Launch blocker.** Previously observed unfavorable-orientation and below-threshold trials ended FAULT while armed despite LAND requests. Require a measured safe outcome; never equate an issued LAND request with confirmed landing.
3. **Build Phase 9** against simulated or recorded AprilTag detections, without giving tracker output flight authority.
4. **Build Phase 10** guidance in SITL only after command authority, manual takeover and watchdog behavior are verified.
5. Integrate mission logic and end-to-end SITL (Phases 11–12), then conduct independent Phase 13 QA.

## CM5 flashing is a parallel track, not a software blocker

The CM5 with onboard eMMC is not yet flashed. Keep Phase 7 **implemented but hardware-unverified**. Use the carrier's documented USB provisioning path and a compatible data cable when available; do not assume the carrier's USB-A peripheral port can program eMMC.

Once available:
1. Confirm Waveshare CM5-NANO-B revision, CM5 variant, boot method and power.
2. Flash supported Ubuntu Server 24.04 ARM64 to eMMC and verify cold boot, networking and SSH.
3. Set up the documented Ubuntu 22.04 Jammy LXD environment for ROS 2 Humble.
4. Verify ARM64 dependencies, camera, UART/USB PX4 transport, power, temperature and storage.
5. Deploy **passive/read-only** services, verify no FMU command publisher, restart/reconnect and rollback.
6. Validate physical camera calibration, measured mounting transform and AprilTag accuracy.

Hardware tests can begin as soon as flashing is available; do not postpone all CM5 integration until Phase 14. **No physical autonomous flight or Throw Launch is authorized by completion of software phases alone.**

## Verification references

- [Phase 8.5 code audit and QA PR](https://github.com/hoangv77/drone_project/pull/8)
- [CM5 deployment instructions](../deployment/cm5/README.md)
- [Phase 8 vision and recording instructions](../drone_project/vision/README.md)
