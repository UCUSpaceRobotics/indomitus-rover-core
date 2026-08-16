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

# udev and systemd both live on the host. Inside a container this script writes
# to the container's own /etc, where nothing will ever read it.
if [ -f /run/.containerenv ] || [ -f /.dockerenv ]; then
    echo "[WARNING] Running inside a container — udev rules and systemd units belong to the host."
    echo "          Run this from the host shell instead (distrobox: 'distrobox-host-exec')."
fi

# Parse Flags
INSTALL_CAN=false
INSTALL_SERVICE=false
INSTALL_JOYSTICK_LED=false

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --can)          INSTALL_CAN=true; shift;;
        --service)      INSTALL_SERVICE=true; shift;;
        --joystick-led) INSTALL_JOYSTICK_LED=true; shift;;
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

# ==========================================
# 3. INSTALL PLAYSTATION LIGHT BAR RULES
# ==========================================
# Hands the DualSense light bar to group 'plugdev' so joystick_interpreter can
# paint it with the drive state without running as root.
if [ "$INSTALL_JOYSTICK_LED" = true ]; then
    echo ">>> Copying udev rules..."
    run_sudo cp "$CONFIG_DIR/rules.d/99-playstation-led.rules" /etc/udev/rules.d/99-playstation-led.rules

    echo ">>> Reloading udev rules..."
    run_sudo udevadm control --reload-rules
    # Re-emits 'add' for LEDs that are already present, so an
    # already-connected controller does not need to be re-paired.
    run_sudo udevadm trigger --subsystem-match=leds --action=add
    echo "[SUCCESS] udev rules applied."
fi