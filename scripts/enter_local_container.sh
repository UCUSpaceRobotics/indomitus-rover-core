#!/bin/bash

set -euo pipefail

# PATH RESOLUTION
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT" || { echo -e "\e[31m[ERROR]\e[0m Failed to navigate to repository root."; exit 1; }

# DEFAULT CONFIGURATION
CONTAINER_NAME="rover_dev"
COMPOSE_FILE="docker-compose.yaml"
ROS_DISTRO="humble"
WORKSPACE_DIR="/opt/ws"
PACKAGES="rover_control rover_bringup"
LAUNCH_PACKAGE="rover_bringup"
LAUNCH_FILE="joy.launch.py"
BUILD=false

show_help() {
  cat << EOF
Usage: $0 [OPTIONS]

Ensures the production container is running, optionally builds the joystick
packages, and opens an interactive terminal with a ready-to-use launch command.

Options:
  -n, --name NAME         Docker container name (Default: ${CONTAINER_NAME})
  -c, --compose FILE      Path to Compose file, relative to repo root (Default: ${COMPOSE_FILE})
  -r, --ros-distro DIST   ROS 2 distribution name (Default: ${ROS_DISTRO})
  -w, --workspace DIR     ROS 2 workspace path inside the container (Default: ${WORKSPACE_DIR})
      --build             Build joystick packages before opening the terminal
  -h, --help              Display this help message and exit
EOF
}

# HELPER FUNCTIONS
success() { echo -e "\e[32m[SUCCESS]\e[0m $1"; }
error()   { echo -e "\e[31m[ERROR]\e[0m $1"; exit 1; }
step()    { echo -e "\n\e[33m>>> $1\e[0m"; }

# PARSE TERMINAL ARGUMENTS
while [[ "$#" -gt 0 ]]; do
  case $1 in
    -n|--name)       [[ "$#" -ge 2 ]] || error "$1 requires an argument."; CONTAINER_NAME="$2"; shift 2;;
    -c|--compose)    [[ "$#" -ge 2 ]] || error "$1 requires an argument."; COMPOSE_FILE="$2"; shift 2;;
    -r|--ros-distro) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; ROS_DISTRO="$2"; shift 2;;
    -w|--workspace)  [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WORKSPACE_DIR="$2"; shift 2;;
    --build) BUILD=true; shift;;
    -h|--help) show_help; exit 0;;
    *) show_help; exit 1;;
  esac
done


# ===========================================================================
# 1. PRE-FLIGHT CHECKS
# ===========================================================================
step "Running Pre-Flight Checks..."

if ! docker info > /dev/null 2>&1; then error "Docker is not running."; fi
if [ ! -f "$COMPOSE_FILE" ]; then error "Compose file not found: $COMPOSE_FILE"; fi
success "Pre-flight checks passed."


# ===========================================================================
# 2. ENSURE CONTAINER IS RUNNING
# ===========================================================================
step "Checking Container State (${CONTAINER_NAME})..."

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


# ===========================================================================
# 3. BUILD PACKAGES (optional, --build flag)
# ===========================================================================
if [ "$BUILD" = true ]; then
  step "Building ROS 2 Packages (${PACKAGES})..."

  BUILD_CMD="source /opt/ros/${ROS_DISTRO}/setup.bash && \
      cd ${WORKSPACE_DIR} && \
      colcon build \
          --packages-select ${PACKAGES} \
          --symlink-install \
          --cmake-args -DCMAKE_BUILD_TYPE=Release \
          2>&1"

  if ! docker exec -it "$CONTAINER_NAME" bash -c "$BUILD_CMD"; then
    error "Package build failed. See output above for details."
  fi
  success "Packages built successfully."
fi


# ===========================================================================
# 4. INTERACTIVE TERMINAL
# ===========================================================================
step "Opening Interactive Terminal..."
echo -e "\e[90m──────────────────────────────────────────────\e[0m"
echo -e "\e[32mYou are now inside the Docker container.\e[0m"
echo -e "The ROS environment and workspace are sourced automatically."
echo -e "To start the joystick node, run:"
echo -e "  \e[36mros2 launch ${LAUNCH_PACKAGE} ${LAUNCH_FILE}\e[0m"
echo -e ""
echo -e "Press \e[31mCtrl+C\e[0m to stop the node gracefully, then type \e[33mexit\e[0m to leave."
echo -e "\e[90m──────────────────────────────────────────────\e[0m"

docker exec -it "$CONTAINER_NAME" bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WORKSPACE_DIR}/install/setup.bash && exec bash -i"

echo -e "\n\e[32m[DONE]\e[0m Session closed."