set -eo pipefail
d=/tmp/drone-phase7-build
mkdir -p "$d/release"
cp -a /mnt/c/dev/drone_project/deployment /mnt/c/dev/drone_project/drone_project "$d"
python3 "$d/deployment/cm5/schema.py" prepare "$d/release" "$HOME/PX4-Autopilot" 97a3c7f01793e2b0b9e5a30425b8e88e69741f27
source /opt/ros/humble/setup.bash
cd "$d/release"
export MAKEFLAGS=-j2
colcon build --merge-install --executor sequential --base-paths px4_msgs ../drone_project/px4_interface_node ../drone_project/safety_monitor ../drone_project/state_machine_manager --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select px4_interface_node safety_monitor state_machine_manager
colcon test-result --verbose
