#!/bin/bash

set -euo pipefail

# PATH RESOLUTION
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT" || { echo -e "\e[31m[ERROR]\e[0m Failed to navigate to repository root."; exit 1; }

JETSON_USER="ros"
JETSON_IP="10.42.0.1"
COMPOSE_PATH="/home/ros/Indomitus/indomitus-rover-core/docker-compose.prod.yaml"
COMPOSE_SERVICE="indomitus_rover_prod"
ROS_DISTRO="humble"
LAUNCH_PACKAGE="indomitus_rover_bringup"
LAUNCH_FILE="rover.launch.py"
WIFI_SSID="JetsonRosIndomitus"
WIFI_PASS="jetson1234"

show_help() {
    cat << EOF
Usage: $0 [OPTIONS]

Connects to the Jetson hotspot if needed, brings up the production container,
and starts the rover launch file inside the container.

NOTE: You can run this script from any folder on your computer.

Options:
    -i, --ip IP           Jetson IP address (Default: ${JETSON_IP})
    -u, --user USER       Jetson SSH username (Default: ${JETSON_USER})
    -c, --compose FILE    Full path to the compose file on the Jetson
                                                (Default: ${COMPOSE_PATH})
    -s, --service NAME    Docker Compose service name to start (Default: ${COMPOSE_SERVICE})
    -r, --ros-distro VER  ROS 2 distribution inside the container (Default: ${ROS_DISTRO})
    -l, --launch FILE     Launch file to start inside the container (Default: ${LAUNCH_FILE})
    -g, --package NAME    ROS 2 package containing the launch file
                                                (Default: ${LAUNCH_PACKAGE})
    -w, --ssid SSID       Wi-Fi SSID of the Jetson hotspot to auto-connect (Default: ${WIFI_SSID})
    -p, --pass PASS       Wi-Fi password for the Jetson hotspot (Default: ${WIFI_PASS})
  -h, --help            Display this help message and exit
EOF
}

# HELPER FUNCTIONS
success() { echo -e "\e[32m[SUCCESS]\e[0m $1"; }
error()   { echo -e "\e[31m[ERROR]\e[0m $1"; exit 1; }
step()    { echo -e "\n\e[33m>>> $1\e[0m"; }

connect_wifi() {
    if [ -z "$WIFI_SSID" ]; then
        echo -e "Please switch your Wi-Fi network to the Jetson hotspot now."
        return
    fi

    echo "Attempting to automatically connect to Wi-Fi network: ${WIFI_SSID}..."
    echo "(Note: You can also switch to this network manually right now if you prefer.)"

    if command -v nmcli >/dev/null 2>&1; then
        if [ -n "$WIFI_PASS" ]; then
            nmcli device wifi connect "$WIFI_SSID" password "$WIFI_PASS" >/dev/null 2>&1 || true
        else
            nmcli device wifi connect "$WIFI_SSID" >/dev/null 2>&1 || true
        fi
    elif command -v networksetup >/dev/null 2>&1; then
        WIFI_IFACE=$(networksetup -listallhardwareports | awk '/Hardware Port: Wi-Fi/{getline; print $2}')
        if [ -n "$WIFI_IFACE" ]; then
            if [ -n "$WIFI_PASS" ]; then
                networksetup -setairportnetwork "$WIFI_IFACE" "$WIFI_SSID" "$WIFI_PASS" >/dev/null 2>&1 || true
            else
                networksetup -setairportnetwork "$WIFI_IFACE" "$WIFI_SSID" >/dev/null 2>&1 || true
            fi
        fi
    else
        echo -e "\e[33m[WARNING]\e[0m OS not supported for auto-connect. Please switch to '${WIFI_SSID}' manually."
    fi
}

wait_for_ssh() {
    local target=$1
    local max_retries=60
    local retry_count=0

    echo -n "Waiting for SSH connection to ${target}..."
    while ! ssh -q -o BatchMode=yes -o ConnectTimeout=2 -o StrictHostKeyChecking=accept-new "$target" "echo 'SSH Ready'" >/dev/null 2>&1; do
        sleep 2
        echo -n "."
        retry_count=$((retry_count + 1))
        if [ "$retry_count" -ge "$max_retries" ]; then
            echo ""
            error "Timeout: Could not connect to Jetson at ${target} after 2 minutes."
        fi
    done
    echo ""
    success "Connection established."
}

# PARSE TERMINAL ARGUMENTS
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -u|--user) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_USER="$2"; shift 2;;
        -i|--ip) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_IP="$2"; shift 2;;
        -c|--compose) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; COMPOSE_PATH="$2"; shift 2;;
        -s|--service) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; COMPOSE_SERVICE="$2"; shift 2;;
        -r|--ros-distro) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; ROS_DISTRO="$2"; shift 2;;
        -l|--launch) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; LAUNCH_FILE="$2"; shift 2;;
        -g|--package) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; LAUNCH_PACKAGE="$2"; shift 2;;
        -w|--ssid) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_SSID="$2"; shift 2;;
        -p|--pass) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_PASS="$2"; shift 2;;
        -h|--help) show_help; exit 0;;
        *) show_help; exit 1;;
    esac
done

TARGET="${JETSON_USER}@${JETSON_IP}"

if [[ "$LAUNCH_FILE" != *.py ]]; then
    error "Launch file must be a Python launch file (*.py)."
fi

# 1. PRE-FLIGHT CHECKS
step "Running Pre-Flight Checks..."

if ! command -v ssh >/dev/null 2>&1; then error "SSH client is not available on this machine."; fi

# 2. WAIT FOR JETSON CONNECTION
step "Verifying Jetson Connection..."
connect_wifi
wait_for_ssh "$TARGET"

# 3. START ROVER
step "Starting Rover Launch on Jetson..."
echo "Ensuring container '${COMPOSE_SERVICE}' is running and launching ${LAUNCH_PACKAGE}/${LAUNCH_FILE}..."

ssh -tt "$TARGET" "COMPOSE_PATH=\"${COMPOSE_PATH}\" COMPOSE_SERVICE=\"${COMPOSE_SERVICE}\" ROS_DISTRO=\"${ROS_DISTRO}\" LAUNCH_PACKAGE=\"${LAUNCH_PACKAGE}\" LAUNCH_FILE=\"${LAUNCH_FILE}\" bash -s" <<'REMOTE_EOF'
set -euo pipefail

STOP_CONTAINER_ON_EXIT=0
CLEANUP_DONE=0

graceful_shutdown_container() {
    docker compose -f "$COMPOSE_PATH" exec -T "$COMPOSE_SERVICE" bash -lc "
pid=\$(ps -eo pid,args | grep -F 'ros2 launch ${LAUNCH_PACKAGE} ${LAUNCH_FILE}' | grep -v grep | awk '{print \$1}' | head -n1)
if [ -n \"\$pid\" ]; then
    kill -INT \"\$pid\" || true
fi
" >/dev/null 2>&1 || true

    for _ in {1..10}; do
        if ! docker compose -f "$COMPOSE_PATH" exec -T "$COMPOSE_SERVICE" bash -lc "ps -eo args | grep -F 'ros2 launch ${LAUNCH_PACKAGE} ${LAUNCH_FILE}' | grep -v grep" >/dev/null 2>&1; then
            break
        fi
        sleep 1
    done

    docker compose -f "$COMPOSE_PATH" stop "$COMPOSE_SERVICE" >/dev/null 2>&1 || true
}

cleanup_on_error() {
    if [ "$CLEANUP_DONE" -eq 1 ]; then
        return
    fi
    CLEANUP_DONE=1

    local exit_code=$?
    if [ "$STOP_CONTAINER_ON_EXIT" -eq 1 ]; then
        graceful_shutdown_container
    fi
    return "$exit_code"
}

trap cleanup_on_error EXIT

if ! command -v docker >/dev/null 2>&1; then
    echo -e "\e[31m[ERROR]\e[0m Docker is not installed on the Jetson."
    exit 1
fi

if ! docker compose version >/dev/null 2>&1; then
    echo -e "\e[31m[ERROR]\e[0m Docker Compose v2 is not available on the Jetson."
    exit 1
fi

if [ ! -f "$COMPOSE_PATH" ]; then
    echo -e "\e[31m[ERROR]\e[0m Compose file not found on the Jetson: $COMPOSE_PATH"
    exit 1
fi

docker compose -f "$COMPOSE_PATH" up -d "$COMPOSE_SERVICE"

if docker compose -f "$COMPOSE_PATH" exec -T "$COMPOSE_SERVICE" bash -lc "ps -eo args | grep -F 'ros2 launch ${LAUNCH_PACKAGE} ${LAUNCH_FILE}' | grep -v grep" >/dev/null 2>&1; then
    echo -e "\e[33m[WARNING]\e[0m Rover launch is already running inside the container."
    docker compose -f "$COMPOSE_PATH" logs -f --tail 50 "$COMPOSE_SERVICE"
    exit 0
fi

STOP_CONTAINER_ON_EXIT=1
docker compose -f "$COMPOSE_PATH" exec "$COMPOSE_SERVICE" bash -lc "source /opt/ros/$ROS_DISTRO/setup.bash && if [ -f /opt/ws/install/setup.bash ]; then source /opt/ws/install/setup.bash; fi && exec ros2 launch $LAUNCH_PACKAGE $LAUNCH_FILE"
REMOTE_EOF

success "Rover launch session exited cleanly."