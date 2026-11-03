set -eo pipefail
root=/mnt/c/dev/drone_project
release=/var/tmp/drone-phase7-ros-final/release
px4=$HOME/PX4-Autopilot/build/px4_sitl_default
agent=/var/tmp/drone-phase7-agent-final/install/bin/MicroXRCEAgent
d=$(mktemp -d /tmp/drone-phase7-sih.XXXXXXXX)
echo "$d"
if pgrep -x px4 >/dev/null; then echo "Existing PX4 process"; exit 1; fi
for i in $(seq 1 240); do
    [[ -f $release/install/share/state_machine_manager/config/mission.yaml && -x $agent ]] && break
    sleep 2
done
test -f "$release/install/share/state_machine_manager/config/mission.yaml"
source /opt/ros/humble/setup.bash
source "$release/install/local_setup.bash"
export ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$root/deployment/cm5/config/fastdds.xml"
pids=()
cleanup() {
    for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
    sleep 2
    for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
    wait || true
}
trap cleanup EXIT
cd "$d"
"$agent" udp4 -p 28888 >agent.log 2>&1 &
pids+=("$!")
PX4_SYS_AUTOSTART=10040 PX4_SIM_MODEL=sihsim_quadx PX4_PARAM_UXRCE_DDS_DOM_ID=43 \
    PX4_UXRCE_DDS_PORT=28888 PX4_PARAM_UXRCE_DDS_PTCFG=1 \
    "$px4/bin/px4" -d "$px4/etc" >sitl.log 2>&1 &
pids+=("$!")
ros2 launch "$root/deployment/cm5/launch/onboard.launch.py" >stack.log 2>&1 &
pids+=("$!")
sleep 12
python3 "$root/deployment/cm5/health.py" sitl
kill -TERM "${pids[1]}"
if python3 "$root/deployment/cm5/health.py" sitl; then
    echo 'Expected stale telemetry after stopping SITL'
    exit 1
fi
echo 'Stale telemetry correctly rejected after SITL stopped'
