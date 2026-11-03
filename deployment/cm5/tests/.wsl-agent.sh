set -eo pipefail
d=/var/tmp/drone-phase7-agent-final
mkdir "$d"
git clone -q --depth 1 --branch v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$d/source"
cmake -S "$d/source" -B "$d/build" -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH=/opt/ros/humble -DCMAKE_INSTALL_PREFIX="$d/install" \
    -DUAGENT_SUPERBUILD=OFF -DUAGENT_USE_SYSTEM_FASTDDS=ON \
    -DUAGENT_USE_SYSTEM_FASTCDR=ON -DUAGENT_USE_SYSTEM_LOGGER=ON \
    -DUAGENT_P2P_PROFILE=OFF -DBUILD_SHARED_LIBS=OFF
cmake --build "$d/build" --parallel 3
cmake --install "$d/build"
LD_LIBRARY_PATH=/opt/ros/humble/lib ldd "$d/install/bin/MicroXRCEAgent"
echo "$d"
