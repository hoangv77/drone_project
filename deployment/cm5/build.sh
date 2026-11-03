#!/usr/bin/env bash
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
. "$here/lib.sh"

[[ $# == 2 ]] || die "Usage: bash build.sh PX4_SOURCE_REPO FULL_PX4_COMMIT"
require_platform
umask 022
need git colcon rosdep cmake flock sudo
[[ -w /opt/drone/releases ]] || die "Run install.sh as this operator first."
exec 9< /opt/drone
flock -n 9 || die "Another install, build or update is running."
require_resources /opt/drone/releases
source_ros
repo=$(git -C "$here" rev-parse --show-toplevel)
[[ -z $(git -C "$repo" status --porcelain) ]] || die "Commit or set aside local changes before building."
commit=$(git -C "$repo" rev-parse HEAD)
release=$(mktemp -d "/opt/drone/releases/$commit.XXXXXXXX")
trap 'printf "Build failed; logs and sources retained in %s\n" "$release" >&2' ERR
chmod 0755 "$release"
mkdir "$release/source"
git -C "$repo" archive "$commit" | tar -xf - -C "$release/source"
printf '%s\n' "$commit" > "$release/commit"
scripts=$release/source/deployment/cm5
packages=$release/source/drone_project
python3 "$scripts/schema.py" prepare "$release" "$1" "$2"
paths=("$release/px4_msgs" "$packages/px4_interface_node" "$packages/safety_monitor"
       "$packages/state_machine_manager")
rosdep install --from-paths "${paths[@]}" --ignore-src --rosdistro humble -y
export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
cd "$release"
colcon --log-base log build --base-paths "${paths[@]}" \
    --merge-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
bash "$scripts/verify.sh" build "$release"
touch "$release/verified"
printf 'Verified release: %s\n' "$release"
