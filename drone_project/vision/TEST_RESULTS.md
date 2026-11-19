# Phase 8 validation

Executed in Ubuntu 22.04 x86_64 under WSL with ROS 2 Humble. The build used the installed Ubuntu OpenCV and NumPy packages. The first run found a user-site NumPy 2 ABI conflict. Setting `PYTHONNOUSERSITE=1` resolved it and is part of the optional CM5 runtime entry points.

| Check | Result |
| --- | --- |
| Vision package | Built and installed with colcon |
| Vision and recording tests | 8 passed, including known two-tag and blank frames, optical TF with image timestamps, missing or invalid calibration, stale and disconnected inputs, SQLite recording and replay, H.264 video, interrupted session detection, restart, encoder failure and storage accounting |
| ROS regressions | 129 reported tests after the isolated safety rerun, 0 errors, 0 failures, 0 skipped |
| Phase 6 model checks | 4 passed |
| Phase 7 deployment checks | 14 passed, including service interlocks, systemd restart and rollback |
| Passive SITL | Fresh PX4 telemetry and disarmed/landed state verified. Stale telemetry was rejected after the simulator stopped. |

The first ROS regression run observed a safety-monitor SIGSEGV during teardown after its behavioral assertions passed. The isolated safety rerun passed. Existing safety logic was not edited during Phase 8. These tests establish software behavior on x86_64. ARM64 runtime, boot, a physical flight controller, camera calibration and mounting, hardware H.264 and metric accuracy require target-system evidence.
