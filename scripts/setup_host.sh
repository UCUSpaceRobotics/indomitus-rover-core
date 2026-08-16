#!/bin/bash

set -e
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
HOST_CONFIG_DIR="$REPO_ROOT/system"

# DEFAULT VARIABLES
JETSON_USER="indomitus-rover"
JETSON_IP="10.42.0.1"
WIFI_SSID="IndomitusRover"
WIFI_PASS="12345678"
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
if [ -n "$WIFI_SSID" ]; then
    if command -v nmcli >/dev/null 2>&1; then
        nmcli device wifi connect "$WIFI_SSID" password "$WIFI_PASS" >/dev/null 2>&1 || true
    elif command -v networksetup >/dev/null 2>&1; then
        WIFI_IFACE=$(networksetup -listallhardwareports | awk '/Hardware Port: Wi-Fi/{getline; print $2}')
        [ -n "$WIFI_IFACE" ] && networksetup -setairportnetwork "$WIFI_IFACE" "$WIFI_SSID" "$WIFI_PASS" >/dev/null 2>&1 || true
    else
        echo -e "\e[33m[WARNING]\e[0m Auto-connect not supported. Connect to '${WIFI_SSID}' manually."
    fi
fi

echo -n "Waiting for SSH connection to ${TARGET}..."
MAX_RETRIES=15
RETRY_COUNT=0
while ! ssh -q -o BatchMode=yes -o ConnectTimeout=2 -o StrictHostKeyChecking=accept-new "${TARGET}" "echo ok" > /dev/null 2>&1; do
    sleep 2; echo -n "."; RETRY_COUNT=$((RETRY_COUNT+1))
    [ "$RETRY_COUNT" -ge "$MAX_RETRIES" ] && echo "" && error "Timeout: Could not connect to ${TARGET}."
done
echo ""
success "Connection established."

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
