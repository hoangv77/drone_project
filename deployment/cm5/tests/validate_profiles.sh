#!/usr/bin/env bash
# Host-only schema/build/ROS regression validation. Never starts deployment units.
set -eo pipefail
[[ $# == 4 || ( $# == 5 && $5 == --resume ) ]] || {
    echo "Usage: bash validate_profiles.sh OUTPUT_ROOT LEGACY_SOURCE MODERN_SOURCE MODERN_COMMIT [--resume]" >&2
    exit 1
}
repo=$(cd "$(dirname "$0")/../../.." && pwd)
. "$repo/deployment/cm5/lib.sh"
output=$(realpath -m "$1")
legacy=$(realpath "$2")
modern=$(realpath "$3")
modern_commit=$4
resume=${5:-}
jobs=${VALIDATION_JOBS:-1}
[[ $jobs =~ ^[1-9][0-9]*$ ]] || { echo "VALIDATION_JOBS must be a positive integer" >&2; exit 1; }
if [[ -z $resume && -e $output ]]; then
    echo "Output root must be new (or pass --resume to verify and reuse it): $output" >&2
    exit 1
fi
mkdir -p "$output"
for profile in legacy_1_14_passive modern_px4; do (
    source_ros
    set -u
    root=$output/$profile
    if [[ $profile == legacy_1_14_passive ]]; then
        firmware=$legacy
        revision=4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c
        domain=93
    else
        firmware=$modern
        revision=$modern_commit
        domain=94
    fi
    if [[ -n $resume ]]; then
        # Reuse only an already-built overlay whose installed/staged inputs,
        # profile and exact selected commit still pass full verification.
        python3 "$repo/deployment/cm5/schema.py" verify "$root" "$root/install" \
            --profile "$profile" --firmware-commit "$revision"
    else
        mkdir "$root"
        python3 "$repo/deployment/cm5/schema.py" prepare "$root" "$firmware" "$revision" --profile "$profile"
    fi
    cd "$root"
    export CMAKE_BUILD_PARALLEL_LEVEL=$jobs MAKEFLAGS=-j$jobs
    colcon --log-base log build --base-paths "$root/px4_msgs" \
        "$repo/drone_project/px4_interface_node" "$repo/drone_project/safety_monitor" \
        "$repo/drone_project/state_machine_manager" --merge-install --executor sequential \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DPX4_COMPATIBILITY_PROFILE="$profile"
    python3 "$repo/deployment/cm5/schema.py" verify "$root" "$root/install" \
        --profile "$profile" --firmware-commit "$revision"
    set +u
    source "$root/install/local_setup.bash"
    set -u
    export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=$domain PX4_COMPATIBILITY_PROFILE=$profile
    export ROS_LOG_DIR=$root/ros-log
    colcon --log-base log test --merge-install --executor sequential \
        --base-paths "$repo/drone_project/px4_interface_node" "$repo/drone_project/safety_monitor" \
        "$repo/drone_project/state_machine_manager" \
        --packages-select px4_interface_node safety_monitor state_machine_manager --event-handlers console_direct+
    for package in px4_interface_node safety_monitor state_machine_manager; do
        compgen -G "$root/build/$package/test_results/$package/*.xml" >/dev/null || {
            echo "No test result files for $package" >&2; exit 1;
        }
    done
    colcon test-result --verbose
    CM5_TEST_ROS=1 python3 -m unittest discover -s "$repo/deployment/cm5/tests" -p test_passive_ros.py -v
) done
