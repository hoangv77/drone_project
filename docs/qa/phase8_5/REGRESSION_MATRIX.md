# Regression matrix

| Finding | Test and scope | Boundary |
|---|---|---|
| QA-08/15 | test_verdict.py covers nominal hold, expected rejection fixture, disarmed recovery, impact, contradiction, missing evidence, malformed data and subprocess failure. | The full suite additionally checks actual PX4 terminal flags and Gazebo pose. A near-ground cutoff is a proxy, so independent contact instrumentation is still required for a physical safety case. |
| QA-02 | test_flight_mode.py covers blocked/stale safety and denies unmapped or unauthorized modes. test_passive_ros.py exercises disabled command publishing. | Context publisher identities are validated inside the isolated graph. Transport and caller authentication remain external. |
| QA-03 | test_flight_mode.py exercises repeated, regressing and future status and restored progression. test_safety_pipeline.py covers loss and stale source transitions. | Boot-time timestamps without clock synchronization are invalid. Physical clock recovery is pending. |
| QA-04 | test_flight_mode.py covers cached status, delayed ACK, shared pending transaction, timeouts and a historical ACK/status pair above the original status watermark. | Protocol has no unique topic ACK token. |
| QA-07/11 | Mission ROS fixtures publish system stamps and stage authority. Offline vision tests exercise invalid provenance, age, duplicate ID and frame, with existing exact-TF/invalid-calibration ROS tests. | Physical tag pose, calibrated camera mounting, cross-sensor identity and DDS policy remain pending. |
| QA-09 | test_activation.py exercises interruption before/after pointer change, killed updater, boot rollback, invalid predecessor, corrupt journal, and pre-commit fsync failure. Existing test_deployment.py exercises failed health rollback. | Real ARM64 power-cut testing and installed systemd migration are separate release gates. |
| QA-01/06/15 | Re-evaluate audit observations and run isolated SITL. Retain bad recovery outcomes. | No physical release and no speculative emergency behavior. |

[Executed results](TEST_RESULTS.md)
