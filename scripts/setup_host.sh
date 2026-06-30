#!/bin/bash

read -rsp "Jetson sudo password: " SUDO_PASS
echo ""

set -e
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

# DEFAULT VARIABLES
JETSON_USER="indomitus-rover"
JETSON_IP="10.42.0.1"
REMOTE_DIR="/home/indomitus-rover/indomitus-rover-core"
WIFI_SSID="IndomitusRover"
WIFI_PASS="12345678"
HOST_CONFIG_DIR="$REPO_ROOT/host_config"

# HELPER FUNCTIONS
success() { echo -e "\e[32m[SUCCESS]\e[0m $1"; }
error()   { echo -e "\e[31m[ERROR]\e[0m $1"; exit 1; }
step()    { echo -e "\n\e[33m>>> $1\e[0m"; }

show_help() {
    cat << EOF
Usage: $0 [OPTIONS]

Deploys systemd services and udev rules to the Jetson Nano.

Options:
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
        -i|--ip)   [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_IP="$2"; shift 2;;
        -u|--user) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_USER="$2"; shift 2;;
        -w|--ssid) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_SSID="$2"; shift 2;;
        -p|--pass) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_PASS="$2"; shift 2;;
        -h|--help) show_help; exit 0;;
        *) show_help; exit 1;;
    esac
done

TARGET="${JETSON_USER}@${JETSON_IP}"

# PRE-FLIGHT CHECKS
step "Pre-Flight Checks..."
[[ -f "$HOST_CONFIG_DIR/rules.d/80-can.rules" ]]  || error "Not found: $HOST_CONFIG_DIR/rules.d/80-can.rules"
[[ -f "$HOST_CONFIG_DIR/systemd/rover.service" ]] || error "Not found: $HOST_CONFIG_DIR/systemd/rover.service"
success "All files found."

# CONNECTION
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

# COPY FILES
step "Copying files to Jetson..."
ssh -q "${TARGET}" "mkdir -p /tmp/host_config/rules.d /tmp/host_config/systemd"
scp "$HOST_CONFIG_DIR/rules.d/80-can.rules"  "${TARGET}:/tmp/host_config/rules.d/"
scp "$HOST_CONFIG_DIR/systemd/rover.service" "${TARGET}:/tmp/host_config/systemd/"
success "Files copied."

# INSTALL ON JETSON
step "Installing on Jetson..."
ssh -q "${TARGET}" 'bash -s' << EOF
  set -e

  if systemctl is-enabled rover.service 2>/dev/null | grep -q "enabled"; then
    ROVER_WAS_ENABLED=true
  else
    ROVER_WAS_ENABLED=false
  fi

  echo "${SUDO_PASS}" | sudo -S cp /tmp/host_config/rules.d/80-can.rules  /etc/udev/rules.d/80-can.rules
  echo "${SUDO_PASS}" | sudo -S cp /tmp/host_config/systemd/rover.service /etc/systemd/system/rover.service

  echo "${SUDO_PASS}" | sudo -S udevadm control --reload-rules
  echo "${SUDO_PASS}" | sudo -S udevadm trigger --subsystem-match=net

  echo "${SUDO_PASS}" | sudo -S systemctl daemon-reload

  if [ "\$ROVER_WAS_ENABLED" = true ]; then
    echo "${SUDO_PASS}" | sudo -S systemctl enable rover.service
    echo "${SUDO_PASS}" | sudo -S systemctl restart rover.service
  else
    echo "${SUDO_PASS}" | sudo -S systemctl disable rover.service 2>/dev/null || true
  fi

  rm -rf /tmp/host_config
EOF
success "Installed successfully."

# STATUS
step "Rover Service Status..."
ssh -q "${TARGET}" "systemctl status rover.service --no-pager"

echo -e "\n\e[32m[DONE]\e[0m Host configuration deployed successfully."