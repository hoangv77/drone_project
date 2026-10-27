#!/usr/bin/env bash
set -euo pipefail
# Run in its own shell. No global PX4 source, parameters, or model files change.
root=$(cd "$(dirname "$0")/../../.." && pwd)
sim="$root/drone_project/simulation"
px4=${PX4_ROOT:-"$HOME/PX4-Autopilot"}
run=${PHASE6_RUN_DIR:-"$root/.phase6/sim"}
scenario=${1:-valid}
mkdir -p "$run/runtime"
python3 "$sim/tools/prepare_model.py" --px4 "$px4" --output "$run" --scenario "$scenario" > "$run/model_report.stdout.json"
if [ ! -f "$root/.phase6/gz-build/CMakeCache.txt" ]; then
  cmake -S "$sim/gz" -B "$root/.phase6/gz-build"
fi
cmake --build "$root/.phase6/gz-build" -j 4
source "$run/px4.env"
export PX4_GZ_MODELS="$run/models" PX4_GZ_WORLDS="$run/worlds"
export GZ_SIM_RESOURCE_PATH="$run/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$root/.phase6/gz-build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export HEADLESS=1 PX4_GZ_NO_FOLLOW=1
# Fail closed on a different firmware implementation; re-audit before changing.
test "$(git -C "$px4" rev-parse HEAD)" = 97a3c7f01793e2b0b9e5a30425b8e88e69741f27 || {
  echo 'PX4 revision differs from audited build. Review native throw logic and schemas first.' >&2; exit 1;
}
cd "$run/runtime"
exec "$px4/build/px4_sitl_default/bin/px4" -d "$px4/build/px4_sitl_default/etc"
