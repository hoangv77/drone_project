# Release gates

| Scope | Status and required evidence |
|---|---|
| Phase 9 isolated tracker development | CONDITIONAL. Use isolated image and TF fixtures, explicit family/ID and measured calibration. Consume the validated stream and independently require measurement-time tag poses. |
| CM5 passive deployment | NOT VERIFIED on ARM64. Disconnected passive launch passed in WSL. Connected SIH failed full freshness verification during a run with evidence of clock instability. Verify boot, clock synchronization, serial unplug/replug, systemd restart, journal migration, and power-cut reconciliation with propellers removed. |
| Physical flight | NOT READY. Resolve command authority and ACK correlation limitations, verify source identity and hardware telemetry, and obtain independent validation of operator and PX4 failsafes. |
| Physical Throw Launch | NOT READY. Prohibited. Requires a separate independently reviewed envelope, release veto and recovery safety case with staged hardware evidence. |

Fast GitHub checks run syntax and deterministic regressions. The extended job is restricted to a manual trusted main run on a preprovisioned software-only SITL host. Run it on reviewed release candidates in equivalent isolation before promotion. Preserve failed SITL reports and ROS XML. A negative-case detection success does not override a failed safe-terminal verdict. Hardware validation remains a separate approval gate.
