#!/usr/bin/env bash
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
. "$here/lib.sh"

require_platform
umask 022
(( EUID != 0 )) || die "Run as the deployment operator with sudo access."
(( $# <= 1 )) || die "Usage: bash install.sh [path/to/ros2-apt-source.deb]"
need sudo
need flock
require_resources /opt
sudo install -d -m 0755 /opt/drone /etc/drone
exec 9< /opt/drone
flock -n 9 || die "Another install, build or update is running."

if ! dpkg-query -W -f='${db:Status-Status}' ros2-apt-source 2>/dev/null | grep -qx installed; then
    [[ $# == 1 && -f $1 ]] || die "Supply the official Jammy ros2-apt-source .deb (see README)."
    [[ $(dpkg-deb -f "$1" Package) == ros2-apt-source ]] || die "Expected ros2-apt-source package."
    sudo apt-get install -y "$(realpath "$1")"
fi
sudo apt-get update
sudo apt-get install -y git build-essential cmake python3-colcon-common-extensions \
    python3-rosdep python3-yaml libspdlog-dev libssl-dev libtinyxml2-dev ros-humble-ros-base \
    ros-humble-fastrtps ros-humble-fastcdr ros-humble-rmw-fastrtps-cpp
if [[ ${DRONE_WITH_VISION:-0} == 1 ]]; then
    sudo apt-get install -y ffmpeg ros-humble-camera-calibration
fi
if ! sudo test -f /etc/ros/rosdep/sources.list.d/20-default.list; then
    sudo rosdep init
fi
rosdep update

if ! id drone-runtime >/dev/null 2>&1; then
    sudo useradd --system --user-group --home-dir /var/lib/drone-runtime \
        --shell /usr/sbin/nologin drone-runtime
fi
sudo test -d /opt/drone/releases ||
    sudo install -d -m 0755 -o "$(id -un)" /opt/drone/releases
# The operator can publish a symlink and append deployment records here.
sudo test -d /opt/drone/state ||
    sudo install -d -m 0755 -o "$(id -un)" /opt/drone/state
install_default "$here/config/drone.env.example" /etc/drone/drone.env
install_default "$here/config/fastdds.xml" /etc/drone/fastdds.xml
for unit in drone-agent.service drone-stack.service; do
    destination=/etc/systemd/system/$unit
    install_default "$here/systemd/$unit" "$destination"
    cmp -s "$here/systemd/$unit" "$destination" ||
        die "Existing $destination differs. Review and install it manually."
done
sudo systemctl daemon-reload

agent_commit=57d086216d01ec43121845d385894a25987f8a2c
agent=/opt/drone/agent-v2.4.2
if [[ ! -x $agent/bin/MicroXRCEAgent ]]; then
    work=$(mktemp -d /opt/drone/releases/.agent-build.XXXXXXXX)
    trap 'rm -rf -- "$work"' EXIT
    git init -q "$work/source"
    git -C "$work/source" fetch --depth=1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$agent_commit"
    git -C "$work/source" checkout --detach "$agent_commit"
    cmake -S "$work/source" -B "$work/build" -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_PREFIX_PATH=/opt/ros/humble -DCMAKE_INSTALL_PREFIX="$work/install" \
        -DUAGENT_SUPERBUILD=OFF -DUAGENT_USE_SYSTEM_FASTDDS=ON \
        -DUAGENT_USE_SYSTEM_FASTCDR=ON -DUAGENT_USE_SYSTEM_LOGGER=ON \
        -DUAGENT_P2P_PROFILE=OFF -DBUILD_SHARED_LIBS=OFF
    cmake --build "$work/build" --parallel 1
    cmake --install "$work/build"
    source_ros
    dependencies=$(ldd "$work/install/bin/MicroXRCEAgent")
    [[ $dependencies != *'not found'* ]] || die "Agent library dependencies unresolved: $dependencies"
    sudo test ! -e "$agent" || die "Incomplete agent prefix $agent. Inspect it before retrying."
    sudo mv -T "$work/install" "$agent"
    sudo chown -R root:root "$agent"
fi
[[ -x $agent/bin/MicroXRCEAgent ]] || die "Agent installation incomplete."
printf 'Installed dependencies and unit files. Configure /etc/drone/drone.env, then build and activate a release.\n'
