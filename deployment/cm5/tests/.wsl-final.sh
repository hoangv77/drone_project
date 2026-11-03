set -eo pipefail
root=/mnt/c/dev/drone_project
release=/var/tmp/drone-phase7-ros-final/release
source /opt/ros/humble/setup.bash
source "$release/install/local_setup.bash"
export XDG_RUNTIME_DIR=/run/user/$(id -u)
export FASTRTPS_DEFAULT_PROFILES_FILE="$root/deployment/cm5/config/fastdds.xml"
for script in "$root"/deployment/cm5/*.sh; do bash -n "$script"; done
python3 "$root/deployment/cm5/schema.py" verify "$release" "$(ros2 pkg prefix px4_msgs)"
CM5_TEST_ROS=1 CM5_TEST_SYSTEMD=1 python3 -m unittest discover -s "$root/deployment/cm5/tests" -v
unset FASTRTPS_DEFAULT_PROFILES_FILE
cd "$release"
colcon test --merge-install --executor sequential --packages-select px4_interface_node safety_monitor state_machine_manager
colcon test-result --verbose
