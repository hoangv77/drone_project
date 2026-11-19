# Phase 8.5 remediation map

The audited revision is d3a6e928a63bedc211e4689e1d5077067df44d8a. The original audit remains a separate unchanged commit. See [Baseline](BASELINE.md) for the source finding and historical measurements.

- **SITL safety verdict** moved from process outcome heuristics to separate executed, behavior and terminal-safety results. The scenario, stack supervisor and suite use structured evidence and propagate failure. Regression fixtures replay the unsafe classifications. [Verdict](../../../drone_project/simulation/tools/verdict.py) · [Throw investigation](THROW_LAUNCH_INVESTIGATION.md)
- **Dispatch and telemetry** now require scoped SITL safety/mission context, synchronized source progression and a pre-dispatch ACK barrier. Existing services retain their types, and their command authority is narrowed. [Authorization](COMMAND_AUTHORIZATION.md) · [Freshness](TELEMETRY_FRESHNESS.md) · [ACK correlation](ACK_CORRELATION.md)
- **Mission and perception inputs** check timing and provenance. The offline validation stream requires the configured ID/family and measurement-time body/optical TF. [Input integrity](MISSION_INPUT_INTEGRITY.md)
- **Deployment recovery** uses an atomically persisted journal and stable service supervisor. [Deployment recovery](DEPLOYMENT_RECOVERY.md)
- **Verification** includes mock and offline fixtures, ROS and SITL runs, and separate CI/release gates. [Regression matrix](REGRESSION_MATRIX.md) · [Results](TEST_RESULTS.md) · [Release gates](RELEASE_GATES.md)

QA-01 and QA-15 remain physical blockers. QA-05/07 still require authenticated transport and independent vehicle identity validation, QA-04 retains protocol limitations, and QA-06 remains an operator-reconciliation problem. QA-10/13/14 are outside the scoped production changes. CI coverage is added for QA-12/16, while hardware validation and test-harness reliability remain tracked.
