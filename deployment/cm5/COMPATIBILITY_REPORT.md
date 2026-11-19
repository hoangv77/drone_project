# Phase 1 / Phase 2 implementation validation

Working branch: `hoang/PX4_Compatibility_Update`. Validation date: 2026-11-17
(America/Chicago). The requested branch spelling contained a space/backslash;
the working name uses valid Git ref characters. The pre-existing `.gitignore`
changes were preserved.

## Implementation

Phase 1 introduces explicit `modern_px4` and `legacy_1_14_passive` contracts.
Preparation archives the requested Git commit, stages unmodified firmware
messages, validates required DDS publications and adapter fields/constants,
checks nested message/service dependencies, and records complete schema/input
digests. Legacy preparation and verification also compare all 185 firmware
inputs against digests derived from the exact vendor commit. Modern version
requirements remain intact. Build/runtime/verification profile mismatches fail.

Phase 2 adds centralized compiled adapters and configurable startup topics.
Legacy GPS uses integer coordinates; legacy primary-geofence diagnostics map
to the normalized field. Status fields/constants were checked in the exact
source. Battery and landing data remain explicitly unavailable, independent of
FailsafeFlags battery warnings. All command services reject requests when the
publisher is absent. The legacy profile forces the effective read-only setting
and cannot construct a command publisher even with conflicting overrides.
The `/drone/state` wire fields/types are unchanged; comments clarify unknown
landing/battery semantics. Timestamp and publisher-count checks prevent replay
or ambiguous sources from keeping telemetry fresh.

## Environment and real firmware inputs

The application checkout is `C:\dev\drone_project`, mounted in WSL at
`/mnt/c/dev/drone_project`. Windows itself has no ROS/colcon toolchain. Local
WSL provides Ubuntu 22.04.5, ROS 2 Humble, GCC 11 and colcon on **x86_64**.
This is not the CM5 ARM64 container and no hardware was connected or modified.

The existing firmware repository `/home/hoang/PX4-Autopilot` contains the modern
Phase 6 commit `97a3c7f01793e2b0b9e5a30425b8e88e69741f27`, but initially lacked
the custom legacy commit. The exact legacy Git object was downloaded into
`.phase6/compat-validation/vendor-firmware` from
`https://github.com/Minderring/PX4-Autopilot.git`, commit
`4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c`.
No upstream replacement or fabricated legacy definitions were used for builds.

Legacy preparation stages 184 messages; modern preparation stages 271 messages
and one service. Their optional/nested definitions are compiled along with the
telemetry messages. Initial build outputs were under
`/tmp/drone-px4-compat-20261117/{legacy_1_14_passive,modern_px4}` in WSL, and
execution logs are retained under `.phase6/compat-validation` in the checkout.
WSL later restarted and cleared those temporary overlays. Final validation uses
persistent WSL storage at
`/home/hoang/drone-px4-compat-20261117/{legacy_1_14_passive,modern_px4}`.

## Commands executed

The following commands were executed from the checkout (Linux commands via
`wsl -e bash -lc`, with the existing ROS Humble installation). The initial
network fetch needed sandbox escalation; it then succeeded.

```powershell
git switch -c hoang/PX4_Compatibility_Update
git init .phase6/compat-validation/vendor-firmware
git -C .phase6/compat-validation/vendor-firmware fetch --depth=1 https://github.com/Minderring/PX4-Autopilot.git 4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c
git diff --check
```

```bash
PX4_LEGACY_SOURCE="$PWD/.phase6/compat-validation/vendor-firmware" \
PX4_MODERN_SOURCE=/home/hoang/PX4-Autopilot \
PX4_MODERN_COMMIT=97a3c7f01793e2b0b9e5a30425b8e88e69741f27 \
  python3 -m unittest discover -s deployment/cm5/tests -v \
  > .phase6/compat-validation/unit.log 2>&1

bash -n deployment/cm5/build.sh deployment/cm5/verify.sh \
  deployment/cm5/run.sh deployment/cm5/update.sh deployment/cm5/lib.sh \
  deployment/cm5/tests/validate_profiles.sh

python3 -m py_compile deployment/cm5/schema.py deployment/cm5/health.py \
  deployment/cm5/launch/onboard.launch.py deployment/cm5/tests/test_schema_profiles.py \
  deployment/cm5/tests/test_passive_ros.py \
  drone_project/px4_interface_node/test/test_legacy_telemetry.py

bash deployment/cm5/tests/validate_profiles.sh /tmp/drone-px4-compat-20261117 \
  .phase6/compat-validation/vendor-firmware /home/hoang/PX4-Autopilot \
  97a3c7f01793e2b0b9e5a30425b8e88e69741f27 \
  > .phase6/compat-validation/run.log 2>&1
```

For each profile, `validate_profiles.sh` runs these exact build/test steps, with
`root`, `firmware`, `revision` and `profile` set to the values above and a
separate output directory per profile:

```bash
source /opt/ros/humble/setup.bash
python3 "$repo/deployment/cm5/schema.py" prepare "$root" "$firmware" "$revision" --profile "$profile"
cd "$root"
export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
colcon --log-base log build --base-paths "$root/px4_msgs" \
  "$repo/drone_project/px4_interface_node" "$repo/drone_project/safety_monitor" \
  "$repo/drone_project/state_machine_manager" --merge-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DPX4_COMPATIBILITY_PROFILE="$profile"
python3 "$repo/deployment/cm5/schema.py" verify "$root" "$root/install" \
  --profile "$profile" --firmware-commit "$revision"
source "$root/install/local_setup.bash"
# ROS_LOCALHOST_ONLY=1; separate loopback DDS domains; logs kept in each root.
colcon --log-base log test --merge-install --executor sequential \
  --base-paths "$repo/drone_project/px4_interface_node" "$repo/drone_project/safety_monitor" \
  "$repo/drone_project/state_machine_manager" \
  --packages-select px4_interface_node safety_monitor state_machine_manager --event-handlers console_direct+
colcon test-result --verbose
CM5_TEST_ROS=1 python3 -m unittest discover -s "$repo/deployment/cm5/tests" -p test_passive_ros.py -v
```

Additional inspection used `git show`, `git ls-tree`, Python digest calculation,
and direct schema preparation under `.phase6/compat-validation`. These were
read-only firmware inspections or disposable overlay staging steps.

The package tests were also run with corrected explicit discovery paths using:

```bash
bash .phase6/compat-validation/run_tests.sh legacy_1_14_passive \
  > .phase6/compat-validation/legacy-tests.log 2>&1
# After adding the runtime-profile mismatch test:
source /opt/ros/humble/setup.bash
source /tmp/drone-px4-compat-20261117/legacy_1_14_passive/install/local_setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=93 \
ROS_LOG_DIR=/tmp/drone-px4-compat-20261117/legacy_1_14_passive/ros-log \
  ctest --test-dir /tmp/drone-px4-compat-20261117/legacy_1_14_passive/build/px4_interface_node \
  --output-on-failure > .phase6/compat-validation/legacy-interface-final.log 2>&1
```

`run_tests.sh` is a disposable validation driver under the ignored output
directory; it sources the selected overlay and runs the explicit `colcon test
--base-paths ...` and `colcon test-result --verbose` commands shown above.
After extending dependency validation to services during the modern build,
`python3 .phase6/compat-validation/refresh_modern_manifest.py` regenerated its
manifest through a fresh `schema.prepare` from the exact modern Git commit.
It asserted that every compiler input, schema hash, firmware digest and other
manifest field was identical before updating only dependency metadata. No
message definitions or binaries were altered by that refresh.

The final validation driver is run from start to finish in persistent
storage with eight compiler jobs on this 15 GiB/16-CPU host:

```bash
VALIDATION_JOBS=8 bash deployment/cm5/tests/validate_profiles.sh \
  /home/hoang/drone-px4-compat-20261117 \
  .phase6/compat-validation/vendor-firmware /home/hoang/PX4-Autopilot \
  97a3c7f01793e2b0b9e5a30425b8e88e69741f27 \
  > .phase6/compat-validation/persistent-run.log 2>&1

# Final combined pass after the mission graph-discovery test fix:
VALIDATION_JOBS=8 bash deployment/cm5/tests/validate_profiles.sh \
  /home/hoang/drone-px4-compat-20261117 \
  .phase6/compat-validation/vendor-firmware /home/hoang/PX4-Autopilot \
  97a3c7f01793e2b0b9e5a30425b8e88e69741f27 --resume \
  > .phase6/compat-validation/final-run.log 2>&1
```

The host driver sets `CMAKE_BUILD_PARALLEL_LEVEL` and `MAKEFLAGS` from
`VALIDATION_JOBS`; the deployment build remains single-job. A `--resume` attempt
against the cleared `/tmp` overlays failed closed because `schema.json` was
absent; it did not reuse or invent validation inputs.

## Results

| Validation | Result |
| --- | --- |
| Linux deployment/schema unittest suite | 32 tests: 30 passed, 2 explicitly skipped (ROS launch and systemd opt-in checks) |
| Legacy exact-commit schema preparation and installed verification | Passed; 184 compiled/installed schemas |
| Legacy build: px4_msgs, interface, safety monitor, mission manager | All four packages built successfully |
| Legacy unfiltered CTest/ROS/GTest run | First persistent run: 85 reported tests, 0 errors/failures. Later combined rerun: 4 reported failures from the unchanged throw fixture (three setup assertions plus its CTest wrapper) |
| Legacy onboard-launch integration | Passed, including live legacy inputs, consumer blocking, service rejection, and competing-manager shutdown |
| Modern schema preparation and Python manifest verification | Passed; 271 messages plus one service |
| Modern full build | All four packages built; installed schemas verified |
| Modern PX4 command/telemetry regressions | 36 active unittest cases plus clean-shutdown check passed |
| Modern safety pipeline | 8 active unittest cases plus clean-shutdown check passed; safety policy GTests passed |
| Broader modern manager regressions | Mission graph-discovery test fixed and manager CTest rerun passed; unchanged throw fixture remains intermittently timing-sensitive in combined runs |
| Final scoped validation (throw fixture explicitly excluded) | Passed: 4 legacy and 5 modern CTest targets, plus both onboard-launch integration tests; zero scoped failures |
| Shell syntax and Python compilation checks | Passed |
| `git diff --check` | Passed |

Negative tests cover wrong firmware pins, wrong profiles, mismatched compiled
markers, missing/incorrect/duplicate DDS publications, modified archived,
staged or installed schemas, altered DDS configuration, missing referenced
fields/constants/dependencies, and a rewritten manifest attempting to redefine
the exact legacy contract. Runtime tests cover integer legacy GPS, active-mode
versus user-intention status, NED position/velocity, separate failsafe warnings,
unavailable battery/landing/ACK streams, stale/replayed/zero-timestamp or
wrong-target telemetry, multiple publishers, immutable passive parameters,
mismatched runtime profile startup, and rejected command/mode services before,
during and after fresh telemetry.

During implementation, one modern field-type check initially assumed a
16-bit command target component; the real source requires `uint8`, and the
validator was corrected to that exact definition. One Python test initially
assumed an unversioned Odometry source path; it was corrected to locate the
actual versioned firmware file. An initial colcon test invocation discovered
zero external-source packages; explicit `--base-paths` fixed discovery and the
real tests were rerun. The reusable driver now also rejects missing test-result
files. Editing the initial driver while it was still executing also produced
a trailing shell-command error after both builds/onboard checks had succeeded;
the final driver was kept unchanged during the persistent run. The run results
above distinguish successful phase checks from broader fixture failures.

The builds emit harmless CMake unused-variable warnings for global colcon
options passed to packages that do not consume them. No build errors are
suppressed.

The first persistent modern run passed the PX4 interface and safety suites,
but its broader manager fixtures reported a DDS graph-discovery exception in
`test_00_missing_input_and_no_flight_control_endpoints` and three setup failures
after the unchanged throw fixture's native-observer-loss case. On an isolated
rerun, the throw suite passed unchanged; the graph exception reproduced.
The mission graph test now waits up to its existing bounded timeout for both
mission publishers to appear, catching only the specific nonexistent-node
discovery exception, then applies the original strict endpoint assertions.
The manager suite subsequently passed all three CTest targets. A later combined
rerun reproduced the unchanged throw fixture's setup failure under the legacy
profile as well: after native-observer-loss recovery it sometimes sees
`RECOVERY` (8) when setup immediately expects `STANDBY` (0). This fixture drives
the manager directly with normalized messages and does not run the PX4 adapter.
Its implementation and fixture are outside this task's requested throw-launch
scope and are left unchanged. The unfiltered suite is therefore **not claimed
fully green**. The original
failed run is retained in `persistent-run.log`; isolated reruns are retained in
`modern-manager-rerun.log` and `modern-manager-final.log`. No throw implementation
or throw fixture was changed.

The successful final scoped run explicitly excludes only `test_test_throw_node.py`, keeps
the mission state-machine and mission node checks, and reruns both profile
adapters/safety checks and onboard launches:

```bash
bash .phase6/compat-validation/scoped-final.sh \
  > .phase6/compat-validation/scoped-final.log 2>&1
```

That disposable driver iterates both persistent profile roots, sources Humble
and the selected overlay (with `set +u` while sourcing), verifies schemas, and
executes the following commands for each of `px4_interface_node`,
`safety_monitor` and `state_machine_manager`:

```bash
ctest --test-dir "$root/build/$package" --output-on-failure -E test_test_throw_node.py
CM5_TEST_ROS=1 python3 -m unittest discover -s "$repo/deployment/cm5/tests" -p test_passive_ros.py -v
```

The onboard command runs once per profile, after that profile's package tests.

## Remaining deployment validation

The release-building `deployment/cm5/build.sh` was not run on CM5/ARM64. Its
platform/clean-checkout/install prerequisites remain in place. Run it after
reviewing and committing the changes, with the exact firmware source and the
explicit legacy profile. Verify the generated release on ARM64 ROS 2 Humble.

No hardware DDS, UART, reconnection, cold-boot, timing, systemd deployment,
firmware identity on the connected controller, or physical-state validation is
claimed. The firmware still does not publish battery, landing or ACK streams;
this implementation deliberately leaves them unavailable. Safety permissions
stay blocked in that condition. A timestamp epoch reset after a controller
reboot requires restarting the interface. Flight capability is outside this
passive profile's contract.

The unchanged throw test's intermittent recovery/setup race remains a broader
test-suite limitation, separately from PX4 1.14.3 schema/telemetry compatibility.

ROS integration tests use isolated simulated DDS peers, not a live PX4/SITL
firmware process. They prove adapter/consumer behavior and regression coverage;
they do not establish physical DDS interoperability or flight readiness.

No deployment units were activated, no firmware was flashed, and no physical
flight commands were issued. The existing simulation-only throw-launch
implementation was not changed.

## Changed files

| File | Purpose |
| --- | --- |
| `deployment/cm5/schema.py` | Profile contracts, fields/constants/dependencies, pinned input and installation verification |
| `deployment/cm5/legacy_contract.json` | Exact vendor-commit schema/DDS digests |
| `deployment/cm5/build.sh` | Explicit profile through preparation, CMake and verification |
| `deployment/cm5/verify.sh` | Explicit profile/pin checks and profile-aware health verification |
| `deployment/cm5/lib.sh` | Require explicit runtime profile configuration |
| `deployment/cm5/config/drone.env.example` | Legacy passive runtime profile example |
| `deployment/cm5/run.sh` | Verify schemas/profile before launching the stack |
| `deployment/cm5/update.sh` | Carry selected profile through candidate build/verification |
| `deployment/cm5/launch/onboard.launch.py` | Explicit runtime profile and fixed passive command parameters |
| `deployment/cm5/health.py` | Profile-aware telemetry requirements and unknown-value checks |
| `deployment/cm5/tests/test_deployment.py` | Runtime-profile shell fixtures |
| `deployment/cm5/tests/test_schema_profiles.py` | Real-firmware positive/negative schema and manifest tests |
| `deployment/cm5/tests/test_passive_ros.py` | Profile launch interlocks and legacy consumer integration |
| `deployment/cm5/tests/validate_profiles.sh` | Repeatable, isolated host build/test flow for both profiles |
| `deployment/cm5/README.md` | Updated profile selection, build/update/verify commands |
| `deployment/cm5/COMPATIBILITY.md` | Original flow, contract semantics and validation guide |
| `deployment/cm5/COMPATIBILITY_REPORT.md` | This completion/validation report |
| `drone_project/px4_interface_node/CMakeLists.txt` | Compiled profile, installed marker and profile-appropriate tests |
| `drone_project/px4_interface_node/src/compatibility.hpp.in` | Central legacy/modern topic and field adapters |
| `drone_project/px4_interface_node/src/px4_interface_node.cpp` | Passive enforcement, configurable topics, conservative telemetry/freshness |
| `drone_project/px4_interface_node/msg/DroneState.msg` | Unknown-value documentation only; no wire-field changes |
| `drone_project/px4_interface_node/test/test_legacy_telemetry.py` | Real node/DDS tests with exact legacy generated messages |
| `drone_project/px4_interface_node/test/test_flight_mode.py` | Advance GPS timestamps when publishing a new sample |
| `drone_project/safety_monitor/CMakeLists.txt` | Keep modern raw-DDS fixture in modern builds; normalized policy tests in both |
| `drone_project/state_machine_manager/test/test_mission_node.py` | Bounded DDS discovery wait before existing strict control-endpoint graph assertions |

`prepare_px4_msgs.py` and the message package template were inspected and retain
their existing firmware-copy/generation behavior. Firmware source, generated
definitions and validation build outputs are not application source changes.

## Merge validation — 2026-11-18

PR #9 was reconciled with `main` at
`7fe96e3db368a668318d654324e9ce12dfd8ca89`. The merge retains the legacy passive
profile alongside the base branch's explicit SITL opt-in, localhost/domain
restriction, safety/mission authorization, timestamp-age/ACK checks, activation
journal recovery and optional vision verification. The pinned legacy firmware's
`dds_topics.h.em` and `Tools/msg/templates/ucdr/msg.h.em` confirm outgoing
timestamps include the Agent time offset. Legacy test publishers now use that
translated system-clock timestamp contract. CI and WSL helpers pass explicit
profile arguments.

Validation used the retained overlays and the commands below:

```bash
PX4_LEGACY_SOURCE="$PWD/.phase6/compat-validation/vendor-firmware" \
PX4_MODERN_SOURCE=/home/hoang/PX4-Autopilot \
PX4_MODERN_COMMIT=97a3c7f01793e2b0b9e5a30425b8e88e69741f27 \
  python3 -m unittest discover -s deployment/cm5/tests -v
VALIDATION_JOBS=8 bash deployment/cm5/tests/validate_profiles.sh \
  /home/hoang/drone-px4-compat-20261117 \
  .phase6/compat-validation/vendor-firmware /home/hoang/PX4-Autopilot \
  97a3c7f01793e2b0b9e5a30425b8e88e69741f27 --resume
python3 -m unittest discover -s drone_project/simulation/test -v
PYTHONPATH=drone_project/vision python3 -m unittest discover -s drone_project/vision/test -v
```

Both four-package ROS builds and both installed-schema checks passed. Legacy
ROS/GTest validation reported 86 tests, zero failures; both profile onboard
launch checks passed. The schema/deployment suite ran 37 tests with two opt-in
skips; simulation ran seven successful tests; vision ran 11 tests with four
opt-in skips. All 46 tracked Python files compiled, shell syntax checks passed,
and the workflow YAML parsed.

The unfiltered modern run reported failures in the mode-intention case and an
existing throw-fixture timeout. Logs show a backward WSL system-clock step
during the pending command; the retained clock-discontinuity guard correctly
rejected it. A full interface rerun reproduced that environment-sensitive
result (39 of 40 active cases passed). The exact mode-intention test and its
shutdown assertion passed when run alone, without changing its assertions or
the clock guard. The modern onboard test also passed separately. These results
do not claim a fully green modern runtime suite or physical/ARM64 validation.
Merge logs are retained as `.phase6/compat-validation/merge-*.log`.
