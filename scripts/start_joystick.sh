#!/bin/bash

set -euo pipefail

# PATH RESOLUTION
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT" || { echo -e "\e[31m[ERROR]\e[0m Failed to navigate to repository root."; exit 1; }

# DEFAULT CONFIGURATION
COMPOSE_FILE="docker-compose.yml"
SERVICE_NAME="indomitus_rover_dev"
WORKDIR="/work"
LAUNCH_FILE="joy.launch.py"
JOYSTICK_DEV="/dev/input/js0"

show_help() {
  cat << EOF
Usage: $0 [OPTIONS]

Starts the local Docker Compose stack, enters the running container,
builds the joystick stack, and launches it interactively.

Options:
  -c, --compose FILE    Path to Compose file (Default: ${COMPOSE_FILE})
  -s, --service NAME    Docker Compose service name (Default: ${SERVICE_NAME})
  -w, --workdir DIR     Workspace directory inside the container (Default: ${WORKDIR})
  -l, --launch FILE     Bringup launch file to run (Default: ${LAUNCH_FILE})
  -j, --joystick DEV    Joystick device path passed to joy.launch.py (Default: ${JOYSTICK_DEV})
  -h, --help            Display this help message and exit
EOF
}

# HELPER FUNCTIONS
success() { echo -e "\e[32m[SUCCESS]\e[0m $1"; }
error()   { echo -e "\e[31m[ERROR]\e[0m $1"; exit 1; }
step()    { echo -e "\n\e[33m>>> $1\e[0m"; }

compose() {
  if docker compose version >/dev/null 2>&1; then
    docker compose "$@"
  elif command -v docker-compose >/dev/null 2>&1; then
    docker-compose "$@"
  else
    error "Docker Compose is not installed."
  fi
}

# PARSE TERMINAL ARGUMENTS
while [[ "$#" -gt 0 ]]; do
  case $1 in
    -c|--compose) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; COMPOSE_FILE="$2"; shift 2;;
    -s|--service) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; SERVICE_NAME="$2"; shift 2;;
    -w|--workdir) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WORKDIR="$2"; shift 2;;
    -l|--launch) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; LAUNCH_FILE="$2"; shift 2;;
    -j|--joystick) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JOYSTICK_DEV="$2"; shift 2;;
    -h|--help) show_help; exit 0;;
    *) show_help; exit 1;;
  esac
done

# ==========================================================================
# 1. PRE-FLIGHT CHECKS
# ==========================================================================
step "Running Pre-Flight Checks..."

if ! command -v docker >/dev/null 2>&1; then error "Docker is not installed."; fi
if ! docker info >/dev/null 2>&1; then error "Docker daemon is not running."; fi
if [ ! -f "$COMPOSE_FILE" ]; then error "Compose file not found: $COMPOSE_FILE"; fi


# ==========================================================================
# 2. START CONTAINER
# ==========================================================================
step "Starting Docker Compose Stack..."

compose -f "$COMPOSE_FILE" up -d


# ==========================================================================
# 3. BUILD AND LAUNCH JOYSTICK STACK
# ==========================================================================
step "Building and Launching Joystick Stack..."
echo -e "\e[90m──────────────────────────────────────────────\e[0m"
echo -e "\e[32mYou are now inside the Docker container.\e[0m"
echo -e "The script will build the joystick stack and then launch it."
echo -e "Press \e[31mCtrl+C\e[0m to stop the node gracefully."
echo -e "\e[90m──────────────────────────────────────────────\e[0m"

compose -f "$COMPOSE_FILE" exec -it "$SERVICE_NAME" bash -s <<EOF
set -euo pipefail

source /opt/ros/humble/setup.bash
cd "${WORKDIR}"
colcon build --packages-select indomitus_rover_control indomitus_rover_bringup --symlink-install
source install/setup.bash
exec ros2 launch indomitus_rover_bringup "${LAUNCH_FILE}" joy_dev:="${JOYSTICK_DEV}"
EOF

echo -e "\n\e[32m[DONE]\e[0m Session closed."