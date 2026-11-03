#!/usr/bin/env bash
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
. "$here/lib.sh"
require_platform
need git sudo flock systemctl
state=/opt/drone/state
[[ -w $state ]] || die "Run install.sh as this operator first."
umask 022
exec 8>"$state/update.lock"
flock -n 8 || die "Another update is running."

log() {
    printf '%s %s\n' "$(date -u +%FT%TZ)" "$*" | tee -a "$state/deployments.log"
}
if [[ $# == 2 && $1 == --activate ]]; then
    candidate=$(realpath -e "$2")
elif [[ $# == 3 ]]; then
    ref=$1
    [[ $ref != -* && -n $ref ]] || die "Supply an explicit commit or approved tag."
    repo=$(git -C "$here" rev-parse --show-toplevel)
    [[ -z $(git -C "$repo" status --porcelain) ]] ||
        die "Local changes detected. Commit or set them aside before updating."
    git -C "$repo" fetch --no-tags origin "$ref"
    commit=$(git -C "$repo" rev-parse 'FETCH_HEAD^{commit}')
    log "build requested ref=$ref commit=$commit firmware=$3"
    scratch=$(mktemp -d /opt/drone/releases/.update.XXXXXXXX)
    cleanup_source() {
        if [[ -f $scratch/source/.git ]]; then
            git -C "$repo" worktree remove "$scratch/source"
        fi
        rm -f -- "$scratch/build.log"
        rmdir -- "$scratch"
    }
    trap cleanup_source EXIT
    git -C "$repo" worktree add --detach "$scratch/source" "$commit"
    if [[ -z ${DRONE_WITH_VISION+x} && -f $state/current/vision.enabled ]]; then
        export DRONE_WITH_VISION=1
    fi
    if ! bash "$scratch/source/deployment/cm5/build.sh" "$2" "$3" | tee "$scratch/build.log"; then
        log "build failed commit=$commit; active release untouched"
        exit 1
    fi
    candidate=$(sed -n 's/^Verified release: //p' "$scratch/build.log" | tail -n1)
    cleanup_source
    trap - EXIT
else
    die "Usage: bash update.sh GIT_COMMIT_OR_TAG PX4_SOURCE_REPO FULL_PX4_COMMIT | --activate RELEASE"
fi

[[ $(dirname "$candidate") == /opt/drone/releases && -f $candidate/verified ]] ||
    die "Expected a verified release directly under /opt/drone/releases."
exec 9< /opt/drone
flock -n 9 || die "Another install, build or update is running."
scripts=$candidate/source/deployment/cm5
bash "$scripts/verify.sh" build "$candidate"
commit=$(cat "$candidate/commit")
old=$(readlink "$state/current" || true)
if [[ $old == "$candidate" ]]; then
    log "already active commit=$commit release=$candidate"
    exit 0
fi

# Capture running/starting services. Enablement is managed separately by the operator.
running=()
for unit in drone-agent drone-stack; do
    case $(systemctl show -p ActiveState --value "$unit") in
        active|activating|reloading) running+=("$unit") ;;
    esac
done

point_current() {
    local staging
    staging=$(mktemp -d "$state/.activate.XXXXXXXX")
    ln -s -- "$1" "$staging/current"
    mv -Tf -- "$staging/current" "$state/current"
    rmdir "$staging"
}
stop_running() {
    if (( ${#running[@]} )); then
        # Starting the stack can pull in the Agent via Wants=.
        sudo systemctl stop drone-stack drone-agent
    fi
}
start_running() {
    if (( ${#running[@]} )); then
        sudo systemctl start "${running[@]}"
    fi
}

rollback() {
    local code=$?
    trap - EXIT HUP INT TERM
    if (( code == 0 )); then return; fi
    log "activation failed commit=$commit; restoring release=${old:-none}"
    stop_running || log "rollback: stopping candidate services failed"
    if [[ -n $old ]]; then
        point_current "$old" || die "Rollback could not restore $state/current."
        start_running || die "Rollback restored the pointer but failed to restart previous services."
    else
        rm -f -- "$state/current"
    fi
    log "rollback completed"
    exit "$code"
}
trap rollback EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM
log "activating commit=$commit release=$candidate previous=${old:-none}"
stop_running
point_current "$candidate"
start_running
if [[ " ${running[*]} " == *' drone-stack '* ]]; then
    bash "$scripts/verify.sh" passive "$candidate"
fi
if [[ " ${running[*]} " == *' drone-agent '* ]]; then
    systemctl is-active --quiet drone-agent
fi
log "active commit=$commit release=$candidate previous=${old:-none}"
trap - EXIT HUP INT TERM
