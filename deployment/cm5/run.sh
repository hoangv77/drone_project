#!/usr/bin/env bash
set -eo pipefail
set -u
here=$(cd "$(dirname "$0")" && pwd)
. "$here/lib.sh"
load_runtime_config
source_ros

case ${1:-} in
    agent)
        agent=/opt/drone/agent-v2.4.2/bin/MicroXRCEAgent
        [[ -x $agent ]] || die "Agent missing at $agent. Run install.sh."
        case ${PX4_TRANSPORT:-} in
            serial|usb)
                device=${PX4_SERIAL_DEVICE:-}
                [[ $device == /dev/* && -c $device ]] || die "Serial device missing: $device"
                [[ -r $device && -w $device ]] || die "Serial access denied: $device"
                [[ ${PX4_BAUD_RATE:-} =~ ^[1-9][0-9]{2,6}$ ]] || die "Invalid PX4_BAUD_RATE."
                exec "$agent" serial -D "$device" -b "$PX4_BAUD_RATE"
                ;;
            udp)
                port=${PX4_AGENT_UDP_PORT:-}
                [[ $port =~ ^[1-9][0-9]{0,4}$ ]] && (( port >= 1024 && port <= 65535 )) ||
                    die "PX4_AGENT_UDP_PORT must be 1024–65535."
                exec "$agent" udp4 -p "$port"
                ;;
            *) die "PX4_TRANSPORT must be serial, usb or udp." ;;
        esac
        ;;
    stack)
        source_overlay "$here/../../.."
        exec ros2 launch "$here/launch/onboard.launch.py" "${@:2}"
        ;;
    vision)
        export PYTHONNOUSERSITE=1
        source_overlay "$here/../../.."
        exec ros2 launch drone_vision vision.launch.py "${@:2}"
        ;;
    recorder)
        export PYTHONNOUSERSITE=1
        source_overlay "$here/../../.."
        exec ros2 run drone_vision recorder "${@:2}"
        ;;
    *) die "Usage: bash run.sh agent|stack|vision|recorder [arguments]" ;;
esac
