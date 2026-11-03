set -eo pipefail
root=/mnt/c/dev/drone_project
for script in "$root"/deployment/cm5/*.sh; do bash -n "$script"; done
export XDG_RUNTIME_DIR=/run/user/$(id -u)
CM5_TEST_SYSTEMD=1 python3 -m unittest discover -s "$root/deployment/cm5/tests" -v
agent=/tmp/drone-phase7-agent.arg436BA/install/bin/MicroXRCEAgent
export LD_LIBRARY_PATH=/opt/ros/humble/lib
export FASTRTPS_DEFAULT_PROFILES_FILE="$root/deployment/cm5/config/fastdds.xml"
if timeout 2 "$agent" udp4 -p 28888; then
    echo 'Agent exited before timeout'
    exit 1
else
    test "$?" -eq 124
fi
echo 'Agent UDP startup stayed active'
cd "$root"
python3 -m unittest discover -s drone_project/simulation/test -v
