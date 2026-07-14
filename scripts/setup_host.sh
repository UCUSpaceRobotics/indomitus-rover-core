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
TARGET_MODE="remote"  # Default is remote (Jetson)

# Flags to forward to setup.sh
RUN_CAN=false
RUN_SERVICE=false

# HELPER FUNCTIONS
success() { echo -e "\e[32m[SUCCESS]\e[0m $1"; }
error()   { echo -e "\e[31m[ERROR]\e[0m $1"; exit 1; }
step()    { echo -e "\n\e[33m>>> $1\e[0m"; }

show_help() {
    cat << EOF
Usage: $0 [OPTIONS]

Deploys system configuration files to either the Jetson Nano or the local host.

Options:
    --local           Deploy files directly to THIS machine (skips SSH/Wi-Fi)
    --can             Deploy/configure CAN rules only
    --service         Deploy/configure Rover systemd service only
    -i, --ip IP       Jetson IP address (Default: ${JETSON_IP})
    -u, --user USER   Jetson SSH username (Default: ${JETSON_USER})
    -w, --ssid SSID   Wi-Fi SSID (Default: ${WIFI_SSID})
    -p, --pass PASS   Wi-Fi password (Default: ${WIFI_PASS})
    -h, --help        Display this help message and exit
EOF
}

# PARSE ARGUMENTS
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --local)   TARGET_MODE="local"; shift;;
        --can)     RUN_CAN=true; shift;;
        --service) RUN_SERVICE=true; shift;;
        -i|--ip)   [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_IP="$2"; shift 2;;
        -u|--user) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_USER="$2"; shift 2;;
        -w|--ssid) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_SSID="$2"; shift 2;;
        -p|--pass) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_PASS="$2"; shift 2;;
        -h|--help) show_help; exit 0;;
        *) show_help; exit 1;;
    esac
done

# Build the argument string to pass down to setup.sh
SETUP_ARGS=""
[ "$RUN_CAN" = true ] && SETUP_ARGS="$SETUP_ARGS --can"
[ "$RUN_SERVICE" = true ] && SETUP_ARGS="$SETUP_ARGS --service"

# PRE-FLIGHT CHECKS
step "Pre-Flight Checks..."
[[ -f "$HOST_CONFIG_DIR/rules.d/80-can.rules" ]]  || error "Not found: $HOST_CONFIG_DIR/rules.d/80-can.rules"
[[ -f "$HOST_CONFIG_DIR/systemd/rover.service" ]] || error "Not found: $HOST_CONFIG_DIR/systemd/rover.service"
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
# 2. REMOTE (JETSON) DEPLOYMENT ROUTINE
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
success "Host configuration deployed successfully to Jetson."