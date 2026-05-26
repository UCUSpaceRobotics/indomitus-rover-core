#!/bin/bash

set -euo pipefail

# PATH RESOLUTION
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT" || { echo -e "\e[31m[ERROR]\e[0m Failed to navigate to repository root."; exit 1; }

# DEFAULT CONFIGURATION
JETSON_USER="ros"
JETSON_IP="10.42.0.1"
REMOTE_DIR="/home/ros/Indomitus/indomitus-rover-core/"
CONTAINER_NAME="indomitus_rover_prod"
COMPOSE_FILE="docker/docker-compose.prod.yaml"
WIFI_SSID="JetsonRosIndomitus"
WIFI_PASS="jetson1234"

show_help() {
  cat << EOF
Usage: $0 [OPTIONS]

Connects to the Jetson Nano, starts the production Docker container (if needed),
and opens an interactive terminal inside the container for manual execution.

Options:
  -i, --ip IP           Jetson IP address (Default: ${JETSON_IP})
  -u, --user USER       Jetson SSH username (Default: ${JETSON_USER})
  -d, --dir DIR         Remote deployment directory (Default: ${REMOTE_DIR})
  -n, --name NAME       Docker container name (Default: ${CONTAINER_NAME})
  -c, --compose FILE    Path to Compose file (Default: ${COMPOSE_FILE})
  -w, --ssid SSID       Wi-Fi SSID of the Jetson hotspot (Default: ${WIFI_SSID})
  -p, --pass PASS       Wi-Fi password for the hotspot (Default: ${WIFI_PASS})
  -h, --help            Display this help message and exit
EOF
}

# HELPER FUNCTIONS
success() { echo -e "\e[32m[SUCCESS]\e[0m $1"; }
error()   { echo -e "\e[31m[ERROR]\e[0m $1"; exit 1; }
step()    { echo -e "\n\e[33m>>> $1\e[0m"; }

# PARSE TERMINAL ARGUMENTS
while [[ "$#" -gt 0 ]]; do
  case $1 in
    -u|--user) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_USER="$2"; shift 2;;
    -i|--ip) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_IP="$2"; shift 2;;
    -d|--dir) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; REMOTE_DIR="$2"; shift 2;;
    -n|--name) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; CONTAINER_NAME="$2"; shift 2;;
    -c|--compose) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; COMPOSE_FILE="$2"; shift 2;;
    -w|--ssid) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_SSID="$2"; shift 2;;
    -p|--pass) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_PASS="$2"; shift 2;;
    -h|--help) show_help; exit 0;;
    *) show_help; exit 1;;
  esac
done

TARGET="${JETSON_USER}@${JETSON_IP}"
REMOTE_COMPOSE_FILE="$(basename "${COMPOSE_FILE}")"

# ===========================================================================
# 1. PRE-FLIGHT CHECKS
# ===========================================================================
step "Running Pre-Flight Checks..."

if ! command -v ssh >/dev/null 2>&1; then error "SSH client is not installed."; fi
if [ ! -f "$COMPOSE_FILE" ]; then error "Compose file not found: $COMPOSE_FILE"; fi


# ===========================================================================
# 2. WAIT FOR JETSON CONNECTION (Smart Reconnect)
# ===========================================================================
step "Verifying Network Connection..."

if [ -n "$WIFI_SSID" ]; then
  if command -v nmcli >/dev/null 2>&1; then
    CURRENT_SSID=$(nmcli -t -f active,ssid dev wifi 2>/dev/null | grep '^yes' | cut -d: -f2 || true)
    if [ "$CURRENT_SSID" = "$WIFI_SSID" ]; then
      echo -e "\e[32m[INFO]\e[0m Already connected to network: ${WIFI_SSID}"
    else
      echo "Attempting to automatically connect to Wi-Fi network: ${WIFI_SSID}..."
      if [ -n "$WIFI_PASS" ]; then
        nmcli device wifi connect "$WIFI_SSID" password "$WIFI_PASS" >/dev/null 2>&1 || true
      else
        nmcli device wifi connect "$WIFI_SSID" >/dev/null 2>&1 || true
      fi
    fi
  elif command -v networksetup >/dev/null 2>&1; then
    WIFI_IFACE=$(networksetup -listallhardwareports | awk '/Hardware Port: Wi-Fi/{getline; print $2}')
    if [ -n "$WIFI_IFACE" ]; then
      CURRENT_SSID=$(networksetup -getairportnetwork "$WIFI_IFACE" 2>/dev/null | awk -F': ' '{print $2}' || true)
      if [ "$CURRENT_SSID" = "$WIFI_SSID" ]; then
        echo -e "\e[32m[INFO]\e[0m Already connected to network: ${WIFI_SSID}"
      else
        echo "Attempting to automatically connect to Wi-Fi network: ${WIFI_SSID}..."
        if [ -n "$WIFI_PASS" ]; then
          networksetup -setairportnetwork "$WIFI_IFACE" "$WIFI_SSID" "$WIFI_PASS" >/dev/null 2>&1 || true
        else
          networksetup -setairportnetwork "$WIFI_IFACE" "$WIFI_SSID" >/dev/null 2>&1 || true
        fi
      fi
    fi
  else
    echo -e "\e[33m[WARNING]\e[0m OS not supported for auto-connect. Please switch to '${WIFI_SSID}' manually."
  fi
else
  echo -e "Please switch your Wi-Fi network to the Jetson Nano hotspot now."
fi

echo -n "Waiting for SSH connection to ${TARGET}..."

MAX_RETRIES=60
RETRY_COUNT=0
while ! ssh -q -o BatchMode=yes -o ConnectTimeout=2 -o StrictHostKeyChecking=accept-new "${TARGET}" "echo 'SSH Ready'" > /dev/null 2>&1; do
  sleep 2
  echo -n "."
  RETRY_COUNT=$((RETRY_COUNT+1))
  if [ "$RETRY_COUNT" -ge "$MAX_RETRIES" ]; then
    echo ""
    error "Timeout: Could not connect to Jetson Nano at ${TARGET} after 2 minutes."
  fi
done
echo ""
success "Connection established."


# ===========================================================================
# 3. START CONTAINER
# ===========================================================================
step "Starting Docker Container on Jetson Nano..."

ssh -q "${TARGET}" \
  "REMOTE_DIR='${REMOTE_DIR}' REMOTE_COMPOSE_FILE='${REMOTE_COMPOSE_FILE}' CONTAINER_NAME='${CONTAINER_NAME}' bash -s" << 'EOF'
set -euo pipefail

cd "${REMOTE_DIR}"

CONTAINER_STATE="$(docker inspect -f '{{.State.Running}}' "${CONTAINER_NAME}" 2>/dev/null || echo missing)"
if [ "${CONTAINER_STATE}" != "true" ]; then
  echo "[JETSON] INFO: Container '${CONTAINER_NAME}' is not running. Starting it..."
  if command -v docker-compose >/dev/null 2>&1; then
    docker-compose -f "${REMOTE_COMPOSE_FILE}" up -d
  else
    docker compose -f "${REMOTE_COMPOSE_FILE}" up -d
  fi
else
  echo "[JETSON] INFO: Container '${CONTAINER_NAME}' is already running."
fi
EOF


# ===========================================================================
# 4. INTERACTIVE TERMINAL
# ===========================================================================
step "Opening Interactive Terminal..."
echo -e "\e[90m──────────────────────────────────────────────\e[0m"
echo -e "\e[32mYou are now inside the Docker container.\e[0m"
echo -e "To start the rover, run a command like:"
echo -e "  \e[36mros2 launch indomitus_rover_bringup rover.launch.py\e[0m"
echo -e ""
echo -e "Press \e[31mCtrl+C\e[0m to stop the node gracefully, then type \e[33mexit\e[0m to leave."
echo -e "\e[90m──────────────────────────────────────────────\e[0m"

ssh -t -q "${TARGET}" "docker exec -it '${CONTAINER_NAME}' bash"

echo -e "\n\e[32m[DONE]\e[0m Session closed."