#!/bin/bash
# Runs rover-mission in a container: rover = rover_prod on the Jetson over ssh, local = your
# dev container (simulation). Inside a container call rover-mission directly - it is on PATH
# (docker/Dockerfile).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

# IMPORT SHARED FUNCTIONS
source "${SCRIPT_DIR}/utils.sh"

# DEFAULT VARIABLES
JETSON_USER="indomitus-rover"
JETSON_HOTSPOT_IP="10.42.0.1"
JETSON_ETHERNET_IP="indomitus-rover-computer.local"
JETSON_IP="${JETSON_HOTSPOT_IP}"
ROVER_CONTAINER="rover_prod"
LOCAL_CONTAINER="rover_dev"
CONTAINER_NAME=""
WIFI_SSID="ERC_UCUSpaceRobotics_A"
WIFI_PASS="19283746"
USE_ETH=false
NO_WIFI=false
WHERE=""
MISSION_ARGS=()

error() { echo -e "\e[31m[ERROR]\e[0m $1"; exit 1; }

show_help() {
    cat << EOF
Usage: $0 {rover,local} COMMAND [OPTIONS]

Starts / stops the rover's tmux sessions per mission (rover-mission in the container).
  rover   ${ROVER_CONTAINER} on the Jetson (ssh)
  local   ${LOCAL_CONTAINER} here - simulation missions (*-sim)

Commands:
  start MISSION [--default] [--force]
                         start it, stop the other rover sessions; --default: also at boot;
                         --force: also replace commands typed by hand
  stop [MISSION]         stop its sessions / every rover session
  restart MISSION        stop its sessions, start them fresh
  default                start the default mission
  enter SESSION [-r]     attach (-r: read-only); detach with Ctrl-b d
  status                 what runs now
  list                   missions and their sessions

Options:
  --name NAME      Container (rover: ${ROVER_CONTAINER}, local: ${LOCAL_CONTAINER})
  rover only:
  --eth            Ethernet instead of the hotspot (${JETSON_ETHERNET_IP})
  -n, --no-wifi    No Wi-Fi auto-connect, ssh directly
  --user USER      SSH user (${JETSON_USER})
  --ip IP          Jetson IP (${JETSON_IP})
  --ssid SSID      Hotspot SSID (${WIFI_SSID})
  --pass PASS      Hotspot password (${WIFI_PASS})
  -h, --help       Show this help

Examples:
  $0 rover start nav --default
  $0 rover restart arm
  $0 local start arm-sim
  $0 local enter core -r
EOF
}

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --eth) USE_ETH=true; JETSON_IP="${JETSON_ETHERNET_IP}"; shift 1;;
        --no-wifi|-n) NO_WIFI=true; shift 1;;
        --user) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_USER="$2"; shift 2;;
        --ip)   [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_IP="$2"; shift 2;;
        --name) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; CONTAINER_NAME="$2"; shift 2;;
        --ssid) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_SSID="$2"; shift 2;;
        --pass) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_PASS="$2"; shift 2;;
        -h|--help) show_help; exit 0;;
        *) if [[ -z "$WHERE" ]]; then WHERE="$1"; else MISSION_ARGS+=("$1"); fi; shift 1;;
    esac
done
[[ "${#MISSION_ARGS[@]}" -gt 0 ]] || { show_help; exit 1; }

# -t (a terminal) for `enter`, which attaches to tmux; without one (scripts) plain -i.
TTY=(-i); [[ -t 0 ]] && TTY=(-it)
case "$WHERE" in
    local)
        CONTAINER_NAME="${CONTAINER_NAME:-$LOCAL_CONTAINER}"
        [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null)" == true ]] \
            || error "Container '${CONTAINER_NAME}' isn't running (start it: scripts/enter_container.sh local)"
        exec docker exec "${TTY[@]}" "$CONTAINER_NAME" rover-mission "${MISSION_ARGS[@]}";;
    rover)
        CONTAINER_NAME="${CONTAINER_NAME:-$ROVER_CONTAINER}"
        TARGET="${JETSON_USER}@${JETSON_IP}"
        if [ "$NO_WIFI" = false ]; then
            ensure_wifi_connection "$WIFI_SSID" "$WIFI_PASS" "$USE_ETH"
        fi
        wait_for_ssh "$TARGET" 30 "$USE_ETH"
        ssh -t -q "$TARGET" "docker exec -it '${CONTAINER_NAME}' rover-mission $(printf '%q ' "${MISSION_ARGS[@]}")";;
    *)  error "First argument: rover | local (got '${WHERE}')\nRun '$0 --help' for usage.";;
esac
