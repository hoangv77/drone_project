# Command authorization

QA-02, QA-05 and QA-07. The old adapter applied PX4 readiness checks to both ROS services but never required companion authority. The new gate runs in the same mutually exclusive executor as both services and checks authorization immediately before publishing the sole in-tree VehicleCommand output. The CM5 launch sets read-only and the adapter's command publisher also requires explicit SITL opt-in in a nonzero localhost DDS domain.

| Request | Additional dispatch authorization |
|---|---|
| ARM, prearm checks and PREFLIGHT LOITER | Fresh safety permission and healthy telemetry, current PREFLIGHT context, disarmed and confirmed landed. Existing command-specific PX4 checks still apply. |
| Bootstrap LOITER | Current idle mission, disarmed landed vehicle and healthy preflight, navigation, battery and operator-link inputs. This allows the ground mode change needed to evaluate launch permission. |
| LAND and RTL | Current RECOVERY, fresh recovery-eligible safety context and confirmed armed airborne state. An ACK and mode match confirm mode entry only. |
| DISARM | Current RECOVERY, fresh recovery-eligible safety context and confirmed armed landed state. Native coordinator additionally requires observed lockdown before requesting it. No force flags. |
| Other mapped modes | Rejected by this SITL profile, including TAKEOFF, MISSION, PRECISION_LAND and OFFBOARD. |

[Dispatch and source checks](../../../drone_project/px4_interface_node/src/px4_interface_node.cpp#L316) · [Safety context producer](../../../drone_project/safety_monitor/src/safety_monitor.cpp#L98) · [Mission context producer](../../../drone_project/state_machine_manager/src/throw_launch_node.cpp#L104)

The graph check requires one discovered writer from each designated safety/mission node and matches its DDS GID to the actual message. Contexts are volatile and expire in 500 ms. The default adapter command profile is disabled. Existing command service types remain unchanged. The ROS command service admits requests inside the trusted SITL graph according to the table. It has no authenticated per-caller identity. Node names, counts and GIDs are consistency checks. A process admitted to the DDS domain can impersonate nodes or bypass the ROS service by writing directly to FMU input topics. Isolate the entire SITL graph and transport from hardware, keep the protected CM5 read-only profile, and require DDS/transport policy and independent review before any physical command-enabled deployment. Direct QGroundControl or PX4 access is outside this application gate. [PX4 DDS security](https://docs.px4.io/main/en/middleware/uxrce_dds)

Regression coverage includes stale/blocked safety, rejected mode requests, shared serialization of commands, normal recovery constraints and passive mode.
