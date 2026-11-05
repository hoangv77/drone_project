#!/usr/bin/env bash
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
. "$here/lib.sh"
require_platform
need git sudo flock systemctl python3
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
activation=/opt/drone/recovery/activation.py
[[ -f $activation ]] || die "Install the Phase 8.5 recovery helper and service units first."
for unit in drone-agent drone-stack; do
    [[ $(systemctl show -p ExecStartPre --value "$unit") == *"$activation boot"* ]] ||
        die "Install and reload the Phase 8.5 unit for $unit before activation."
done
running=()
for unit in drone-agent drone-stack; do
    case $(systemctl show -p ActiveState --value "$unit") in
        active|activating|reloading) running+=("$unit") ;;
    esac
done

manage() {
    # Children must not keep the updater lock alive after updater termination.
    sudo systemctl "$@" 8>&- 9>&-
}
journal() {
    python3 -I "$activation" "$@" 8>&- 9>&-
}
restore_running() {
    if (( ${#running[@]} )); then
        manage start "${running[@]}"
    fi
}
rollback() {
    local code=$?
    trap - EXIT HUP INT TERM
    if (( code == 0 )); then return; fi
    manage stop drone-stack drone-agent || true
    if journal rollback; then
        restore_running || log "previous release needs operator restart"
        log "rollback completed"
    else
        log "rollback incomplete. Services require operator intervention."
    fi
    exit "$code"
}

journal prepare "$candidate"
trap rollback EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM
log "activating commit=$commit release=$candidate"
manage stop drone-stack drone-agent
journal switch
manage start drone-stack
bash "$scripts/verify.sh" passive "$candidate" 8>&- 9>&-
for unit in drone-stack drone-agent; do
    systemctl is-active --quiet "$unit" 8>&- 9>&-
done
journal commit
trap - EXIT HUP INT TERM
for unit in drone-stack drone-agent; do
    if [[ " ${running[*]} " != *" $unit "* ]]; then manage stop "$unit"; fi
done
log "active commit=$commit release=$candidate"
