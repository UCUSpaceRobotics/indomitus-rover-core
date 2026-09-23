#!/bin/bash

set -e
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
HOST_CONFIG_DIR="$REPO_ROOT/system"

# IMPORT SHARED FUNCTIONS
source "${SCRIPT_DIR}/utils.sh"

# DEFAULT VARIABLES
JETSON_USER="indomitus-rover"
JETSON_IP="10.42.0.1"
WIFI_SSID="ERC_UCUSpaceRobotics_A"
WIFI_PASS="19283746"
NO_WIFI=false
TARGET_MODE=""   # Must be set via positional arg: rover|local

# Flags to forward to setup.sh
RUN_CAN=false
RUN_SERVICE=false
RUN_JOYSTICK_LED=false

# HELPER FUNCTIONS
success() { echo -e "\e[32m[SUCCESS]\e[0m $1"; }
error()   { echo -e "\e[31m[ERROR]\e[0m $1"; exit 1; }
step()    { echo -e "\n\e[33m>>> $1\e[0m"; }

show_help() {
    cat << EOF
Usage: $0 {rover,local} [OPTIONS]

Deploys system configuration files to either the Jetson (rover) or the local host.

Targets:
    rover             Deploy over SSH to the Jetson Nano (Wi-Fi connect + SCP + remote setup)
    local             Deploy files directly to THIS machine (skips SSH/Wi-Fi)

Options:
    --can             Deploy/configure CAN rules only
    --service         Deploy/configure Rover systemd service only
    --joystick-led    Deploy/configure PlayStation light bar rules only
    -i, --ip IP       Jetson IP address (Default: ${JETSON_IP})
    -u, --user USER   Jetson SSH username (Default: ${JETSON_USER})
    -w, --ssid SSID   Wi-Fi SSID (Default: ${WIFI_SSID})
    -p, --pass PASS   Wi-Fi password (Default: ${WIFI_PASS})
    --no-wifi, -n     Skip the Wi-Fi auto-connect and ssh straight to the Jetson
                      (already on the network, or connected some other way).
    -h, --help        Display this help message and exit

Examples:
    $0 rover --can --service
    $0 local --can
    $0 local --joystick-led
EOF
}

# PARSE ARGUMENTS
while [[ "$#" -gt 0 ]]; do
    case $1 in
        rover|local)
            [[ -n "$TARGET_MODE" ]] && error "Target already set to '$TARGET_MODE'; cannot also set '$1'."
            TARGET_MODE="$1"; shift;;
        --can)          RUN_CAN=true; shift;;
        --service)      RUN_SERVICE=true; shift;;
        --joystick-led) RUN_JOYSTICK_LED=true; shift;;
        -i|--ip)   [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_IP="$2"; shift 2;;
        -u|--user) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_USER="$2"; shift 2;;
        -w|--ssid) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_SSID="$2"; shift 2;;
        -p|--pass) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_PASS="$2"; shift 2;;
        --no-wifi|-n) NO_WIFI=true; shift 1;;
        -h|--help) show_help; exit 0;;
        *) show_help; exit 1;;
    esac
done

# Require an explicit target
if [[ -z "$TARGET_MODE" ]]; then
    error "Missing target. Usage: $0 {rover,local} [OPTIONS]  (run with -h for help)"
fi

# Build the argument string to pass down to setup.sh
SETUP_ARGS=""
[ "$RUN_CAN" = true ] && SETUP_ARGS="$SETUP_ARGS --can"
[ "$RUN_SERVICE" = true ] && SETUP_ARGS="$SETUP_ARGS --service"
[ "$RUN_JOYSTICK_LED" = true ] && SETUP_ARGS="$SETUP_ARGS --joystick-led"

# PRE-FLIGHT CHECKS
step "Pre-Flight Checks..."
[[ -f "$HOST_CONFIG_DIR/rules.d/80-can.rules" ]]  || error "Not found: $HOST_CONFIG_DIR/rules.d/80-can.rules"
[[ -f "$HOST_CONFIG_DIR/systemd/rover.service" ]] || error "Not found: $HOST_CONFIG_DIR/systemd/rover.service"
[[ -f "$HOST_CONFIG_DIR/rules.d/99-playstation-led.rules" ]] || error "Not found: $HOST_CONFIG_DIR/rules.d/99-playstation-led.rules"
[[ -f "$HOST_CONFIG_DIR/setup.sh" ]]              || error "Not found: $HOST_CONFIG_DIR/setup.sh"
success "All configuration assets verified."


# ==========================================
# 1. LOCAL DEPLOYMENT ROUTINE
# ==========================================
if [ "$TARGET_MODE" = "local" ]; then
    step "Deploying LOCALLY..."
    # Execute setup.sh locally with the parsed arguments
    bash "$HOST_CONFIG_DIR/setup.sh" $SETUP_ARGS
    success "Local host configuration deployed successfully."
    exit 0
fi


# ==========================================
# 2. REMOTE (ROVER/JETSON) DEPLOYMENT ROUTINE
# ==========================================
read -rsp "Jetson sudo password: " SUDO_PASS; echo ""
TARGET="${JETSON_USER}@${JETSON_IP}"

# Connection Hook
step "Connecting to Jetson..."
if [ "$NO_WIFI" = true ]; then
    echo "--no-wifi: skipping Wi-Fi auto-connect, ssh'ing directly"
else
    ensure_wifi_connection "$WIFI_SSID" "$WIFI_PASS" "false"
fi
wait_for_ssh "$TARGET" 30

# Copy everything from system/ directory to a temp folder
step "Copying files to Jetson..."
ssh -q "${TARGET}" "mkdir -p /tmp/host_config"
scp -r "$HOST_CONFIG_DIR/"* "${TARGET}:/tmp/host_config/"
success "Files copied."

# Run setup.sh on the Jetson with parsed arguments forwarded
step "Installing on Jetson..."
ssh -t -q "${TARGET}" "SUDO_PASS='$SUDO_PASS' bash /tmp/host_config/setup.sh $SETUP_ARGS"

# Cleanup
ssh -q "${TARGET}" "rm -rf /tmp/host_config"
success "Rover deployment complete."