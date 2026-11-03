#!/usr/bin/env bash
set -euo pipefail
trap 'code=$?; if (( code == 0 )); then echo "PASS verification completed"; else echo "FAIL verification"; fi' EXIT
here=$(cd "$(dirname "$0")" && pwd)
. "$here/lib.sh"
mode=${1:-build}
release=${2:-/opt/drone/state/current}
(( $# <= 2 )) || die "Expected a verification mode and optional release path."
case $mode in build|passive|sitl|hardware) ;; *) die "Usage: bash verify.sh [build|passive|sitl|hardware] [release]";; esac

trap 'printf "FAIL verification aborted at line %s\n" "$LINENO" >&2' ERR
# Run in a subshell so die() contributes a check result.
if (require_platform); then echo 'PASS ARM64 Ubuntu 22.04'; else echo 'FAIL operating environment'; exit 1; fi
source_ros
source_overlay "$release"
echo 'PASS Humble and deployment overlay loaded'
for executable in 'px4_interface_node px4_interface_node' 'safety_monitor safety_monitor' \
    'state_machine_manager state_machine_manager' 'state_machine_manager throw_launch_manager'; do
    read -r pkg name <<< "$executable"
    prefix=$(ros2 pkg prefix "$pkg")
    [[ $(realpath "$prefix") == "$(realpath "$release/install")" &&
       -x $prefix/lib/$pkg/$name ]] || die "FAIL missing deployed executable $pkg/$name"
done
for interface in px4_msgs/msg/FailsafeFlags px4_msgs/msg/VehicleStatus \
    px4_interface_node/msg/DroneState px4_interface_node/srv/ExecuteCommand \
    px4_interface_node/srv/SetFlightMode safety_monitor/msg/SafetyStatus \
    state_machine_manager/msg/MissionState; do
    ros2 interface show "$interface" >/dev/null
done
echo 'PASS installed packages, executables and ROS interfaces'
if [[ -f $release/vision.enabled ]]; then
    vision=$(ros2 pkg prefix drone_vision)
    for executable in camera_gate test_camera recorder; do
        [[ $(realpath "$vision") == "$(realpath "$release/install")" &&
           -x $vision/lib/drone_vision/$executable ]] || die "FAIL missing vision executable $executable"
    done
    echo 'PASS optional vision and recording executables'
fi
python3 "$here/schema.py" verify "$release" "$(ros2 pkg prefix px4_msgs)"
echo 'PASS installed schemas match the selected source revision'
if [[ $mode == build ]]; then
    echo 'SKIP live DDS, telemetry, mission and command interlock checks (build only)'
    exit 0
fi
load_runtime_config
if [[ $mode == hardware ]]; then
    [[ $PX4_TRANSPORT == serial || $PX4_TRANSPORT == usb ]] ||
        die "FAIL hardware verification expects a dedicated serial or USB connection."
fi
if [[ $mode == sitl ]]; then
    [[ $PX4_TRANSPORT == udp ]] || die "FAIL SITL verification expects UDP."
fi
status=0
if systemctl is-active --quiet drone-agent; then
    echo 'PASS DDS Agent unit active (telemetry checked separately)'
elif [[ $mode == passive ]]; then
    echo 'SKIP DDS Agent inactive in passive startup check'
else
    echo 'FAIL DDS Agent unit inactive'
    status=1
fi
python3 "$here/health.py" "$mode" || status=1
echo 'SKIP live firmware identity and vehicle readiness require separate operator evidence'
exit "$status"
