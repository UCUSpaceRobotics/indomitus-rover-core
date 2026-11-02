#!/bin/bash

set -euo pipefail

# PATH RESOLUTION
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT" || { echo -e "\e[31m[ERROR]\e[0m Failed to navigate to repository root."; exit 1; }

# IMPORT SHARED FUNCTIONS
source "${SCRIPT_DIR}/utils.sh"

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

Opens a shell in the local or the rover container.

Commands:
  local   Local dev container (started if needed)
  rover   rover_prod on the Jetson, over ssh

Per command: $0 local -h
             $0 rover -h

Examples:
  $0 local          # dev container
  $0 local sim      # tmux session in it
  $0 rover          # via hotspot
  $0 rover -n       # already on LAN
  $0 rover --eth    # over Ethernet
  $0 rover core     # tmux session
  $0 rover arm -r   # read-only
EOF
}

# Attach to (or create) tmux session $S inside the container; $1 = read-only.
# S is passed via docker exec -e so the script needs no local quoting/expansion.
tmux_script() {
  if [ "$1" = true ]; then
    echo 'exec tmux attach-session -r -t "$S"'
  else
    echo 'if ! tmux has-session -t "$S" 2>/dev/null; then echo "[tmux] Session $S does not exist yet - creating it. Existing: $(tmux ls -F "#S" 2>/dev/null | tr "\n" " ")"; fi; exec tmux new-session -A -s "$S"'
  fi
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
  local HARDWARE_WORKSPACE_DIR="/opt/hw_ws"
  local SESSION=""
  local READ_ONLY=false

  show_local_help() {
    cat << EOF
Usage: $0 local [SESSION] [OPTIONS]

Shell in the local dev container (started if needed).

SESSION            tmux session to attach (sim, nav, ...),
                   created if missing. Detach: Ctrl-b d
                   ('exit' closes it). None: plain shell.

Options:
  -r, --read-only    With SESSION: watch only
                     (session must exist)
  --name NAME        Container (${CONTAINER_NAME})
  --compose FILE     Compose file, from repo root
                     (${COMPOSE_FILE})
  --ros-distro DIST  ROS 2 distro (${ROS_DISTRO})
  --workspace DIR    Workspace in container
                     (${WORKSPACE_DIR})
  -h, --help         Show this help
EOF
  }

  while [[ "$#" -gt 0 ]]; do
    case $1 in
      --name)       [[ "$#" -ge 2 ]] || error "$1 requires an argument."; CONTAINER_NAME="$2"; shift 2;;
      --compose)    [[ "$#" -ge 2 ]] || error "$1 requires an argument."; COMPOSE_FILE="$2"; shift 2;;
      --ros-distro) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; ROS_DISTRO="$2"; shift 2;;
      --workspace)  [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WORKSPACE_DIR="$2"; shift 2;;
      -r|--read-only)  READ_ONLY=true; shift 1;;
      -h|--help)       show_local_help; exit 0;;
      -*)              error "Unknown option: $1\nRun '$0 local --help' for usage." ;;
      *)
        [[ -z "$SESSION" ]] || error "Only one SESSION can be given (got '$SESSION' and '$1')."
        [[ "$1" =~ ^[A-Za-z0-9_-]+$ ]] || error "Invalid session name: '$1' (letters, digits, _ and - only)."
        SESSION="$1"; shift 1;;
    esac
  done

  [[ "$READ_ONLY" = false || -n "$SESSION" ]] || error "--read-only needs a SESSION."

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

  if [ -n "$SESSION" ]; then
    step "Attaching to tmux session '${SESSION}'..."
    echo -e "Detach with \e[33mCtrl-b d\e[0m - typing exit closes the session."
    docker exec -it -e S="$SESSION" "$CONTAINER_NAME" sh -c "$(tmux_script "$READ_ONLY")"
    echo -e "\n\e[32m[DONE]\e[0m Detached from '${SESSION}'."
    return
  fi

  step "Opening Interactive Terminal..."
  echo -e "\e[90m──────────────────────────────────────────────\e[0m"
  echo -e "\e[32mYou are now inside the Docker container.\e[0m"
  echo -e "The ROS environment and workspace are sourced automatically."
  echo -e "Type \e[33mexit\e[0m to leave."
  echo -e "\e[90m──────────────────────────────────────────────\e[0m"

  docker exec -it "$CONTAINER_NAME" bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WORKSPACE_DIR}/install/setup.bash && source ${HARDWARE_WORKSPACE_DIR}/install/setup.bash && exec bash -i"

  echo -e "\n\e[32m[DONE]\e[0m Session closed."
}

# ===========================================================================
# ROVER CONTAINER LOGIC
# ===========================================================================
run_rover() {
  # Defaults
  local JETSON_USER="indomitus-rover"
  local JETSON_HOTSPOT_IP="10.42.0.1"
  local JETSON_ETHERNET_IP="indomitus-rover-computer.local"
  local JETSON_IP="${JETSON_HOTSPOT_IP}"
  local REMOTE_DIR=""
  local CONTAINER_NAME="rover_prod"
  local COMPOSE_FILE="docker-compose.prod.yaml"
  local WORKSPACE_DIR="/opt/ws"
  local HARDWARE_WORKSPACE_DIR="/opt/hw_ws"
  local WIFI_SSID="ERC_UCUSpaceRobotics_A"
  local WIFI_PASS="19283746"
  local USE_ETH=false
  local NO_WIFI=false
  local SESSION=""
  local READ_ONLY=false

  show_rover_help() {
    cat << EOF
Usage: $0 rover [SESSION] [OPTIONS]

Shell in rover_prod on the Jetson (started if needed).

SESSION            tmux session to attach (core, arm, ...),
                   created if missing. Detach: Ctrl-b d
                   ('exit' closes it). None: plain shell.

Options:
  -r, --read-only  With SESSION: watch only
                   (session must exist)
  --eth            Ethernet instead of the hotspot
                   (${JETSON_ETHERNET_IP})
  -n, --no-wifi    No Wi-Fi auto-connect, ssh directly
  --user USER      SSH user (${JETSON_USER})
  --ip IP          Jetson IP (${JETSON_IP})
  --dir DIR        Repo dir on the Jetson
                   (~USER/indomitus-rover-core/)
  --name NAME      Container (${CONTAINER_NAME})
  --compose FILE   Compose file (${COMPOSE_FILE})
  --workspace DIR  Workspace in container
                   (${WORKSPACE_DIR})
  --ssid SSID      Hotspot SSID (${WIFI_SSID})
  --pass PASS      Hotspot password (${WIFI_PASS})
  -h, --help       Show this help
EOF
  }

  while [[ "$#" -gt 0 ]]; do
    case $1 in
      --eth) USE_ETH=true; JETSON_IP="${JETSON_ETHERNET_IP}"; shift 1;;
      --no-wifi|-n) NO_WIFI=true; shift 1;;
      --user) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_USER="$2"; shift 2;;
      --ip) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_IP="$2"; shift 2;;
      --dir) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; REMOTE_DIR="$2"; shift 2;;
      --name) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; CONTAINER_NAME="$2"; shift 2;;
      --compose) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; COMPOSE_FILE="$2"; shift 2;;
      --workspace) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WORKSPACE_DIR="$2"; shift 2;;
      --ssid) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_SSID="$2"; shift 2;;
      --pass) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_PASS="$2"; shift 2;;
      -r|--read-only) READ_ONLY=true; shift 1;;
      -h|--help) show_rover_help; exit 0;;
      -*) error "Unknown option: $1\nRun '$0 rover --help' for usage." ;;
      *)
        [[ -z "$SESSION" ]] || error "Only one SESSION can be given (got '$SESSION' and '$1')."
        [[ "$1" =~ ^[A-Za-z0-9_-]+$ ]] || error "Invalid session name: '$1' (letters, digits, _ and - only)."
        SESSION="$1"; shift 1;;
    esac
  done

  [[ "$READ_ONLY" = false || -n "$SESSION" ]] || error "--read-only needs a SESSION."
  REMOTE_DIR="${REMOTE_DIR:-/home/${JETSON_USER}/indomitus-rover-core/}"

  local TARGET="${JETSON_USER}@${JETSON_IP}"
  local REMOTE_COMPOSE_FILE="$(basename "${COMPOSE_FILE}")"

  step "Running Pre-Flight Checks (Rover)..."
  if ! command -v ssh >/dev/null 2>&1; then error "SSH client is not installed."; fi

  step "Verifying Network Connection..."
  if [ "$NO_WIFI" = true ]; then
    echo "--no-wifi: skipping Wi-Fi auto-connect, ssh'ing directly"
  else
    ensure_wifi_connection "$WIFI_SSID" "$WIFI_PASS" "$USE_ETH"
  fi
  wait_for_ssh "$TARGET" 30 "$USE_ETH"

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

  if [ -n "$SESSION" ]; then
    step "Attaching to tmux session '${SESSION}'..."
    echo -e "Detach with \e[33mCtrl-b d\e[0m - typing exit closes the session."
    ssh -t -q "${TARGET}" "docker exec -it -e S='${SESSION}' '${CONTAINER_NAME}' sh -c '$(tmux_script "$READ_ONLY")'"
    echo -e "\n\e[32m[DONE]\e[0m Detached from '${SESSION}'."
    return
  fi

  step "Opening Interactive Terminal..."
  echo -e "\e[90m──────────────────────────────────────────────\e[0m"
  echo -e "\e[32mYou are now inside the remote Docker container.\e[0m"
  echo -e "Type \e[33mexit\e[0m to leave."
  echo -e "\e[90m──────────────────────────────────────────────\e[0m"

  ssh -t -q "${TARGET}" "docker exec -it '${CONTAINER_NAME}' bash -lc 'source /opt/ros/humble/setup.bash && source ${WORKSPACE_DIR}/install/setup.bash && source ${HARDWARE_WORKSPACE_DIR}/install/setup.bash && exec bash -i'"

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