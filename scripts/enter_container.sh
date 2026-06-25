#!/bin/bash

set -euo pipefail

# PATH RESOLUTION
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT" || { echo -e "\e[31m[ERROR]\e[0m Failed to navigate to repository root."; exit 1; }

# HELPER FUNCTIONS
success() { echo -e "\e[32m[SUCCESS]\e[0m $1"; }
error()   { echo -e "\e[31m[ERROR]\e[0m $1"; exit 1; }
step()    { echo -e "\n\e[33m>>> $1\e[0m"; }

# ===========================================================================
# MAIN HELP MENU
# ===========================================================================
show_main_help() {
  cat << EOF
Usage: $0 {local|rover} [OPTIONS]

Connects to a Docker container interactively, depending on the environment.

Commands:
  local       Ensures the local container is running and opens an interactive terminal.
  rover       Connects to the rover computer, starts the container, and opens a terminal.

Use '$0 local --help' or '$0 rover --help' for command-specific options.
EOF
}

# ===========================================================================
# LOCAL CONTAINER LOGIC
# ===========================================================================
run_local() {
  # Defaults
  local CONTAINER_NAME="rover_dev"
  local COMPOSE_FILE="docker-compose.yaml"
  local ROS_DISTRO="humble"
  local WORKSPACE_DIR="/opt/ws"

  show_local_help() {
    cat << EOF
Usage: $0 local [OPTIONS]

Ensures the local development container is running and opens an interactive terminal.

Options:
  -n, --name NAME         Docker container name (Default: ${CONTAINER_NAME})
  -c, --compose FILE      Path to Compose file, relative to repo root (Default: ${COMPOSE_FILE})
  -r, --ros-distro DIST   ROS 2 distribution name (Default: ${ROS_DISTRO})
  -w, --workspace DIR     ROS 2 workspace path inside container (Default: ${WORKSPACE_DIR})
  -h, --help              Display this help message and exit
EOF
  }

  while [[ "$#" -gt 0 ]]; do
    case $1 in
      -n|--name)       [[ "$#" -ge 2 ]] || error "$1 requires an argument."; CONTAINER_NAME="$2"; shift 2;;
      -c|--compose)    [[ "$#" -ge 2 ]] || error "$1 requires an argument."; COMPOSE_FILE="$2"; shift 2;;
      -r|--ros-distro) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; ROS_DISTRO="$2"; shift 2;;
      -w|--workspace)  [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WORKSPACE_DIR="$2"; shift 2;;
      -h|--help)       show_local_help; exit 0;;
      *)               error "Unknown option: $1\nRun '$0 local --help' for usage." ;;
    esac
  done

  step "Running Pre-Flight Checks (Local)..."
  if ! docker info > /dev/null 2>&1; then error "Docker is not running."; fi
  if [ ! -f "$COMPOSE_FILE" ]; then error "Compose file not found: $COMPOSE_FILE"; fi
  success "Pre-flight checks passed."

  step "Checking Container State (${CONTAINER_NAME})..."
  local CONTAINER_STATUS
  CONTAINER_STATUS=$(docker inspect --format '{{.State.Status}}' "$CONTAINER_NAME" 2>/dev/null || true)

  if [ "$CONTAINER_STATUS" = "running" ]; then
    success "Container '${CONTAINER_NAME}' is already running."
  elif [ "$CONTAINER_STATUS" = "exited" ] || [ "$CONTAINER_STATUS" = "created" ]; then
    echo "Container '${CONTAINER_NAME}' is stopped. Starting it..."
    docker compose -f "$COMPOSE_FILE" start
    success "Container started."
  elif [ -z "$CONTAINER_STATUS" ]; then
    echo "Container '${CONTAINER_NAME}' does not exist. Bringing it up..."
    docker compose -f "$COMPOSE_FILE" up -d
    success "Container created and started."
  else
    error "Container '${CONTAINER_NAME}' is in an unexpected state: '${CONTAINER_STATUS}'. Aborting."
  fi

  step "Opening Interactive Terminal..."
  echo -e "\e[90m──────────────────────────────────────────────\e[0m"
  echo -e "\e[32mYou are now inside the Docker container.\e[0m"
  echo -e "The ROS environment and workspace are sourced automatically."
  echo -e "Type \e[33mexit\e[0m to leave."
  echo -e "\e[90m──────────────────────────────────────────────\e[0m"

  docker exec -it "$CONTAINER_NAME" bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WORKSPACE_DIR}/install/setup.bash && exec bash -i"

  echo -e "\n\e[32m[DONE]\e[0m Session closed."
}

# ===========================================================================
# ROVER CONTAINER LOGIC
# ===========================================================================
run_rover() {
  # Defaults
  local JETSON_USER="indomitus-rover"
  local JETSON_IP="10.42.0.1"
  local REMOTE_DIR="/home/indomitus-rover/indomitus-rover-core/"
  local CONTAINER_NAME="rover_prod"
  local COMPOSE_FILE="docker-compose.prod.yaml"
  local WIFI_SSID="IndomitusRover"
  local WIFI_PASS="12345678"

  show_rover_help() {
    cat << EOF
Usage: $0 rover [OPTIONS]

Connects to the rover computer, starts the production Docker container (if needed),
and opens an interactive terminal inside the remote container.

Options:
  -u, --user USER       Jetson SSH username (Default: ${JETSON_USER})
  -i, --ip IP           Jetson IP address (Default: ${JETSON_IP})
  -d, --dir DIR         Remote deployment directory (Default: ${REMOTE_DIR})
  -n, --name NAME       Docker container name (Default: ${CONTAINER_NAME})
  -c, --compose FILE    Path to Compose file (Default: ${COMPOSE_FILE})
  -w, --ssid SSID       Wi-Fi SSID of the Jetson hotspot (Default: ${WIFI_SSID})
  -p, --pass PASS       Wi-Fi password for the hotspot (Default: ${WIFI_PASS})
  -h, --help            Display this help message and exit
EOF
  }

  while [[ "$#" -gt 0 ]]; do
    case $1 in
      -u|--user)    [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_USER="$2"; shift 2;;
      -i|--ip)      [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_IP="$2"; shift 2;;
      -d|--dir)     [[ "$#" -ge 2 ]] || error "$1 requires an argument."; REMOTE_DIR="$2"; shift 2;;
      -n|--name)    [[ "$#" -ge 2 ]] || error "$1 requires an argument."; CONTAINER_NAME="$2"; shift 2;;
      -c|--compose) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; COMPOSE_FILE="$2"; shift 2;;
      -w|--ssid)    [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_SSID="$2"; shift 2;;
      -p|--pass)    [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_PASS="$2"; shift 2;;
      -h|--help)    show_rover_help; exit 0;;
      *)            error "Unknown option: $1\nRun '$0 rover --help' for usage." ;;
    esac
  done

  local TARGET="${JETSON_USER}@${JETSON_IP}"
  local REMOTE_COMPOSE_FILE="$(basename "${COMPOSE_FILE}")"

  step "Running Pre-Flight Checks (Rover)..."
  if ! command -v ssh >/dev/null 2>&1; then error "SSH client is not installed."; fi

  step "Verifying Network Connection..."
  if [ -n "$WIFI_SSID" ]; then
    if command -v nmcli >/dev/null 2>&1; then
      local CURRENT_SSID
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
      local WIFI_IFACE CURRENT_SSID
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
    echo -e "Please switch your Wi-Fi network to the rover computer hotspot now."
  fi

  echo -n "Waiting for SSH connection to ${TARGET}..."
  local MAX_RETRIES=60
  local RETRY_COUNT=0
  while ! ssh -q -o BatchMode=yes -o ConnectTimeout=2 -o StrictHostKeyChecking=accept-new "${TARGET}" "echo 'SSH Ready'" > /dev/null 2>&1; do
    sleep 2
    echo -n "."
    RETRY_COUNT=$((RETRY_COUNT+1))
    if [ "$RETRY_COUNT" -ge "$MAX_RETRIES" ]; then
      echo ""
      error "Timeout: Could not connect to rover computer at ${TARGET} after 2 minutes."
    fi
  done
  echo ""
  success "Connection established."

  step "Starting Docker Container on rover computer..."
  ssh -q "${TARGET}" \
    "REMOTE_DIR='${REMOTE_DIR}' REMOTE_COMPOSE_FILE='${REMOTE_COMPOSE_FILE}' CONTAINER_NAME='${CONTAINER_NAME}' bash -s" << 'EOF'
  set -euo pipefail
  cd "${REMOTE_DIR}"
  CONTAINER_STATE="$(docker inspect -f '{{.State.Running}}' "${CONTAINER_NAME}" 2>/dev/null || echo missing)"
  if [ "${CONTAINER_STATE}" != "true" ]; then
    echo "[JETSON] INFO: Container '${CONTAINER_NAME}' is not running. Starting it..."
    if command -v docker-compose >/dev/null 2>&1; then
      docker-compose up -d
    else
      docker compose up -d
    fi
  else
    echo "[JETSON] INFO: Container '${CONTAINER_NAME}' is already running."
  fi
EOF

  step "Opening Interactive Terminal..."
  echo -e "\e[90m──────────────────────────────────────────────\e[0m"
  echo -e "\e[32mYou are now inside the remote Docker container.\e[0m"
  echo -e "Type \e[33mexit\e[0m to leave."
  echo -e "\e[90m──────────────────────────────────────────────\e[0m"

  ssh -t -q "${TARGET}" "docker exec -it '${CONTAINER_NAME}' bash"

  echo -e "\n\e[32m[DONE]\e[0m Session closed."
}

# ===========================================================================
# COMMAND ROUTING
# ===========================================================================
if [ $# -eq 0 ]; then
  show_main_help
  exit 1
fi

COMMAND=$1
shift

case "$COMMAND" in
  local)
    run_local "$@"
    ;;
  rover)
    run_rover "$@"
    ;;
  -h|--help)
    show_main_help
    exit 0
    ;;
  *)
    error "Unknown command: '$COMMAND'. Must be 'local' or 'rover'.\nRun '$0 --help' for usage."
    ;;
esac