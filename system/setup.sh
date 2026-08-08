#!/bin/bash
# system/setup.sh - Runs ON the targeted system (Local Host or Jetson)
set -e

SUDO_PASS="${SUDO_PASS:-}"
run_sudo() {
    if [ -n "$SUDO_PASS" ]; then
        echo "$SUDO_PASS" | sudo -S "$@"
    else
        sudo "$@"
    fi
}

CONFIG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

# Parse Flags
INSTALL_CAN=false
INSTALL_SERVICE=false

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --can)     INSTALL_CAN=true; shift;;
        --service) INSTALL_SERVICE=true; shift;;
        *) echo "Unknown option: $1"; exit 1;;
    esac
done

# ==========================================
# 1. INSTALL CAN BUS RULES
# ==========================================
if [ "$INSTALL_CAN" = true ]; then
    echo ">>> Copying udev rules..."
    run_sudo cp "$CONFIG_DIR/rules.d/80-can.rules" /etc/udev/rules.d/80-can.rules

    echo ">>> Reloading udev rules..."
    run_sudo udevadm control --reload-rules
    run_sudo udevadm trigger --subsystem-match=net
    echo "[SUCCESS] udev rules applied."
fi

# ==========================================
# 2. INSTALL SYSTEMD ROVER SERVICE
# ==========================================
if [ "$INSTALL_SERVICE" = true ]; then
    echo ">>> Copying systemd service..."
    run_sudo cp "$CONFIG_DIR/systemd/rover.service" /etc/systemd/system/rover.service

    echo ">>> Reloading systemd daemon..."
    run_sudo systemctl daemon-reload

    echo ">>> Resetting rover service status (always disabled and stopped)..."
    run_sudo systemctl stop rover.service 2>/dev/null || true
    run_sudo systemctl disable rover.service 2>/dev/null || true
    echo "[SUCCESS] Rover service installed and disabled."
fi