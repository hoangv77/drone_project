# PX4 compatibility profiles

## Existing flow (before this update)

`build.sh` required a clean application checkout and a full firmware commit.
`schema.py prepare` archived `msg/`, optional `srv/`, and `dds_topics.yaml`
from that Git object. `prepare_px4_msgs.py` copied the firmware schemas into
the local package template, preferring `msg/versioned` on name collisions.
Validation required modern VehicleStatus v4, VehicleLocalPosition v1 and
BatteryStatus v1 plus the other onboard streams. `colcon` built the overlay;
`verify.sh` compared installed message hashes and DDS configuration to the
manifest. The runtime had fixed modern status/position/battery/ACK topics and
an independent read-only publisher/service interlock.

## Selected contracts

| Contract | `modern_px4` | `legacy_1_14_passive` |
| --- | --- | --- |
| Firmware identity | Explicit full commit; existing modern version requirements | Only `4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c` |
| Status / local position | `/fmu/out/vehicle_status_v4`, `/fmu/out/vehicle_local_position_v1` | `/fmu/out/vehicle_status`, `/fmu/out/vehicle_local_position` |
| GPS | Existing SensorGps or VehicleGnss interface selected by compiled schemas | `/fmu/out/vehicle_gps_position`, SensorGps |
| Odometry / failsafe flags | `/fmu/out/vehicle_odometry`, `/fmu/out/failsafe_flags` | Same unversioned names |
| Battery / landing / ACK | Existing topics and handlers retained | Unavailable; no subscriptions created |
| Flight commands | Existing SITL behavior, gated by `read_only` and `sitl_commands` | Always disabled, regardless of parameter overrides or telemetry health |

The vendor source is [Minderring/PX4-Autopilot at the exact firmware commit](https://github.com/Minderring/PX4-Autopilot/tree/4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c).
`legacy_contract.json` contains SHA-256 digests computed directly from that Git
commit: 184 unmodified message definitions and `dds_topics.yaml`. These are
validation metadata, not hand-maintained replacement message definitions.
Preparation archives Git objects, so working-tree edits are not silently used.
Changes to this pin require a separately reviewed compatibility contract.

Both profiles validate required DDS topic names/types, typed fields, referenced
constants and nested message dependencies. All staged schemas (including
unpublished BatteryStatus, VehicleLandDetected, VehicleCommandAck and
VehicleCommand) must match the selected firmware bytes. The package template
globs all firmware-derived schemas; no definitions are patched by hand.
Legacy messages without `MESSAGE_VERSION` are recorded as JSON `null`.
Their exact pinned digests and field checks establish compatibility; the absent
version alone does not. Modern version checks remain unchanged.

The manifest records `compatibility_profile`, `firmware_commit`,
`dds_topics_sha256`, `firmware_inputs`, `schemas`, `message_dependencies`,
`telemetry_message_versions`, `required_publications` and
`unavailable_publications`. Verification compares the installed definitions,
staged compilation inputs, archived firmware inputs and the interface's
installed `compatibility_profile` marker. It rejects old manifests, a different
requested profile/commit, changed files, or a mismatched compiled profile.
These checks establish local source/build consistency; identifying the firmware
actually connected to DDS still requires the controller's build records.

## Build and runtime selection

Use a separate overlay for each profile: the generated `px4_msgs` ABIs differ.
The deployment build requires an explicit third argument:

```bash
bash deployment/cm5/build.sh "$PX4_SOURCE" \
  4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c legacy_1_14_passive
bash deployment/cm5/verify.sh build "$RELEASE" legacy_1_14_passive \
  4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c
```

For isolated modern SITL, supply `modern_px4` and its exact modern firmware
commit instead. The Phase 6 reference is
`97a3c7f01793e2b0b9e5a30425b8e88e69741f27`. The deployment build still requires
ARM64 Jammy and a clean application checkout. It does not activate services.

Set `PX4_COMPATIBILITY_PROFILE` in `/etc/drone/drone.env` to the same explicit
profile. Existing configuration is preserved by install; operators upgrading
from an older release must add this setting. `run.sh stack` verifies the release
before launching it, and passes the profile to `onboard.launch.py`. Direct
launch also requires `compatibility_profile:=PROFILE`. The onboard launch fixes
`read_only=true`, `sitl_commands=false`, and mission monitoring interlocks for
both profiles. Runtime profile changes are rejected by the compiled node.

For local colcon development, pass
`--cmake-args -DPX4_COMPATIBILITY_PROFILE=PROFILE`. CMake's default remains
`modern_px4` to preserve existing SITL builds. The compiled profile becomes the
node's startup parameter default, is logged, and is immutable. The legacy node
ignores a `read_only=false` override and reports the effective value `true`.
No PX4 input publisher is constructed in that build, even when
`sitl_commands=true`. Command services remain available but reject all requests.
Modern `sitl_commands` defaults to false, preserving the base branch's explicit
SITL opt-in. Enabling it requires `ROS_LOCALHOST_ONLY=1` and a dedicated nonzero
DDS domain (1–99). The base branch's fresh safety/mission authorization contexts
and command acknowledgment correlation remain enforced. The legacy profile
never creates these command endpoints even when the opt-in is supplied.

Topic defaults are centralized in `src/compatibility.hpp.in` and the node's
startup configuration. ROS remappings or immutable `topics.vehicle_status`,
`topics.vehicle_local_position`, `topics.vehicle_odometry`, `topics.gps`, and
`topics.failsafe_flags` parameters support isolated tests/namespaced endpoints.
Modern builds also expose battery, landing and ACK topic parameters. Overrides
do not change the compiled schema or the passive restriction; deployed endpoint
names must still match the verified DDS contract.

## Telemetry semantics and consumer compatibility

The normalized `/drone/state` fields and wire types are unchanged. In legacy
mode, `battery_fresh=false`, `battery_remaining=NaN`,
`land_detected_fresh=false`, `ready_to_arm=false` and `command_pending=false`.
`landed=false` with `land_detected_fresh=false` means **unknown**, never confirmed
airborne. Battery warnings from FailsafeFlags remain separate diagnostics and
cannot make battery telemetry fresh or establish battery health.
Existing safety and mission consumers already require battery/landing freshness
for permission. They remain conservatively blocked without those streams; the
passive mission manager remains in monitoring mode.

Legacy SensorGps validates `fix_type >= 3` and range-checks integer `lat`/`lon`
(degrees times 1e7). It has no `FIX_TYPE_3D` constant or floating degree fields.
Legacy `primary_geofence_breached` maps to the existing normalized geofence
diagnostic. The exact legacy VehicleStatus includes all status fields read by
the node, including `nav_state_user_intention`; active flight mode still comes
from `nav_state`. Arming and mode constants are validated against the source.

Freshness carries the age of Agent-translated system timestamps into a
monotonic expiry deadline, preserving the base branch's source-age checks.
Zero, repeated, out-of-order, future-dated or already-stale PX4 timestamps
cannot refresh caches, and exactly one publisher is required for
each source. Status from another target system/component is ignored. Fast
position/odometry data expires after 500 ms; status, GPS, flags, battery and
landing expire after two seconds. Invalid/stale velocity and battery readings
are NaN. Position, armed/mode and landed caches remain last-known values behind
their explicit freshness flags. A backward timestamp epoch or host-clock reset
can invalidate the stored watermarks; restart the interface after clocks are
synchronized if fresh source timestamps cannot advance beyond them.

Live verification checks the runtime profile and absence of all `/fmu/in/`
publishers. Legacy live telemetry requires only the five supported streams and
checks that unavailable values remain unknown. It explicitly skips landing
confirmation and cannot certify physical flight readiness.

## Reproducible host validation

On a Linux ROS 2 Humble host, with both real firmware repositories available:

```bash
PX4_LEGACY_SOURCE="$LEGACY_SOURCE" \
PX4_MODERN_SOURCE="$MODERN_SOURCE" \
PX4_MODERN_COMMIT=97a3c7f01793e2b0b9e5a30425b8e88e69741f27 \
  python3 -m unittest discover -s deployment/cm5/tests -v
bash deployment/cm5/tests/validate_profiles.sh NEW_OUTPUT_ROOT \
  "$LEGACY_SOURCE" "$MODERN_SOURCE" \
  97a3c7f01793e2b0b9e5a30425b8e88e69741f27
```

The second command builds all three consumer packages with each generated
`px4_msgs` package, verifies installed schemas, runs the profile-appropriate DDS
tests plus normalized safety/mission regressions, and tests the disconnected
onboard launch. It uses separate output roots and loopback DDS domains. It does
not start deployment units or access a flight controller. Firmware-dependent
Python tests explicitly skip when source paths are absent. The modern raw DDS
safety fixture uses modern schemas and runs only in the modern profile; the
normalized safety policy tests run with both.

Choose persistent build storage on WSL; its `/tmp` can be cleared when the
distribution restarts. `VALIDATION_JOBS` defaults to 1; an adequately provisioned
host can select more compiler jobs for this host-only driver. The production
CM5 build remains single-job. `--resume` as a final argument verifies an
existing overlay's schemas, profile and selected firmware commit before
incrementally rebuilding and rerunning its tests. Missing/stale/mismatched
overlays fail verification rather than triggering a profile fallback.

See [implementation validation report](COMPATIBILITY_REPORT.md) for executed
commands, results, changed files and remaining hardware validation.
