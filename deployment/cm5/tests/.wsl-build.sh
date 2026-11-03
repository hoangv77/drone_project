set -eo pipefail
d=/var/tmp/drone-phase7-ros-final
mkdir "$d"
echo "$d"
mkdir -p "$d/release" "$d/drone_project"
root=/mnt/c/dev/drone_project
cp -r "$root/deployment" "$d"
for name in px4_interface_node safety_monitor state_machine_manager; do
    cp -r "$root/drone_project/$name" "$d/drone_project"
done
mkdir -p "$d/drone_project/simulation/tools"
cp "$root/drone_project/simulation/tools/prepare_px4_msgs.py" "$d/drone_project/simulation/tools/"
python3 "$d/deployment/cm5/schema.py" prepare "$d/release" "$HOME/PX4-Autopilot" 97a3c7f01793e2b0b9e5a30425b8e88e69741f27
source /opt/ros/humble/setup.bash
cd "$d/release"
export MAKEFLAGS=-j8
colcon build --merge-install --executor sequential --base-paths px4_msgs ../drone_project/px4_interface_node ../drone_project/safety_monitor ../drone_project/state_machine_manager --cmake-args -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG'
source install/local_setup.bash
bash "$root/deployment/cm5/tests/.wsl-final.sh"
