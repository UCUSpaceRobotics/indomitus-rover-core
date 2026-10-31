#!/bin/bash
# Runs rover-mission (system/rover-mission.sh, installed on the Jetson by
# setup_host.sh) over ssh. On the Jetson itself, call rover-mission directly.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

# IMPORT SHARED FUNCTIONS
source "${SCRIPT_DIR}/utils.sh"

# DEFAULT VARIABLES
JETSON_USER="indomitus-rover"
JETSON_HOTSPOT_IP="10.42.0.1"
JETSON_ETHERNET_IP="indomitus-rover-computer.local"
JETSON_IP="${JETSON_HOTSPOT_IP}"
WIFI_SSID="ERC_UCUSpaceRobotics_A"
WIFI_PASS="19283746"
USE_ETH=false
NO_WIFI=false
MISSION_ARGS=()

error() { echo -e "\e[31m[ERROR]\e[0m $1"; exit 1; }

show_help() {
    cat << EOF
Usage: $0 [OPTIONS] COMMAND

Starts / stops rover subsystems per mission on the Jetson.

Commands (rover-mission on the Jetson):
  MISSION [--default]    drive | nav | arm | all; --default: also at boot
  stop                   stop every subsystem
  restart SUBSYSTEM...   core | cameras | nav | arm
  status                 what runs now
  list                   missions and their subsystems

Connection:
  --eth            Ethernet instead of the hotspot (${JETSON_ETHERNET_IP})
  -n, --no-wifi    No Wi-Fi auto-connect, ssh directly
  --user USER      SSH user (${JETSON_USER})
  --ip IP          Jetson IP (${JETSON_IP})
  --ssid SSID      Hotspot SSID (${WIFI_SSID})
  --pass PASS      Hotspot password (${WIFI_PASS})
  -h, --help       Show this help

Examples:
  $0 nav
  $0 -n arm --default
  $0 restart cameras
  $0 status
EOF
}

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --eth) USE_ETH=true; JETSON_IP="${JETSON_ETHERNET_IP}"; shift 1;;
        --no-wifi|-n) NO_WIFI=true; shift 1;;
        --user) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_USER="$2"; shift 2;;
        --ip)   [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_IP="$2"; shift 2;;
        --ssid) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_SSID="$2"; shift 2;;
        --pass) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_PASS="$2"; shift 2;;
        -h|--help) show_help; exit 0;;
        *) MISSION_ARGS+=("$1"); shift 1;;
    esac
done
[[ "${#MISSION_ARGS[@]}" -gt 0 ]] || { show_help; exit 1; }

TARGET="${JETSON_USER}@${JETSON_IP}"
if [ "$NO_WIFI" = false ]; then
    ensure_wifi_connection "$WIFI_SSID" "$WIFI_PASS" "$USE_ETH"
fi
wait_for_ssh "$TARGET" 30 "$USE_ETH"

# -t: rover-mission re-runs itself under sudo, which asks for the password.
ssh -t -q "$TARGET" "rover-mission $(printf '%q ' "${MISSION_ARGS[@]}")"
