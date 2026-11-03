# Shared by deployment entry points.
die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
need() {
    for command in "$@"; do
        command -v "$command" >/dev/null || die "Missing $command. Run install.sh first."
    done
}
require_platform() {
    local ID VERSION_ID
    . /etc/os-release
    [[ $ID == ubuntu && $VERSION_ID == 22.04 ]] || die "Expected Ubuntu 22.04 userspace (native or container)."
    [[ $(uname -m) == aarch64 && $(dpkg --print-architecture) == arm64 ]] ||
        die "Expected ARM64 kernel and Ubuntu userspace."
}
require_resources() {
    local free memory
    free=$(df -Pk "$1" | awk 'NR == 2 {print $4}')
    memory=$(awk '/^(MemTotal|SwapTotal):/ {sum += $2} END {print sum}' /proc/meminfo)
    (( free >= 8 * 1024 * 1024 )) || die "Need 8 GiB free in $1."
    (( memory >= 3 * 1024 * 1024 )) || die "Need 3 GiB RAM plus swap. Prefer at least 4 GB RAM."
}
source_ros() {
    [[ -r /opt/ros/humble/setup.bash ]] || die "ROS 2 Humble is missing."
    unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
    unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
    set +u
    . /opt/ros/humble/setup.bash
    set -u
}
source_overlay() {
    [[ -r $1/install/local_setup.bash ]] || die "No installed overlay at $1."
    set +u
    . "$1/install/local_setup.bash"
    set -u
}
load_runtime_config() {
    # The operator-owned file uses assignments compatible with both bash and systemd.
    local file=/etc/drone/drone.env
    [[ -r $file ]] || die "Missing $file. Run install.sh and configure the transport."
    set -a
    . "$file"
    set +a
    [[ ${ROS_DOMAIN_ID:-} =~ ^(0|[1-9][0-9]?)$ ]] || die "ROS_DOMAIN_ID must be 0–99."
    [[ -r /etc/drone/fastdds.xml ]] || die "Missing the Fast DDS loopback profile."
    export ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/drone/fastdds.xml
}
# Preserve existing operator configuration. Unit files also require comparison.
install_default() {
    if ! sudo test -e "$2"; then
        sudo install -m 0644 "$1" "$2"
    fi
}
