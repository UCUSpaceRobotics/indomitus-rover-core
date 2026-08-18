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
# to the container's own /etc, where nothing will ever read it — and every step
# still reports success, so the operator walks away believing the rover is
# configured. Refuse instead; a wrong environment is worse than no run at all.
if [ -f /run/.containerenv ] || [ -f /.dockerenv ]; then
    if [ -n "$ROVER_SETUP_ALLOW_CONTAINER" ]; then
        echo "[WARNING] Running inside a container — ROVER_SETUP_ALLOW_CONTAINER is set, continuing."
        echo "          udev rules and systemd units written here affect the container only."
    else
        echo "[ERROR] Running inside a container. udev rules and systemd units belong to the"
        echo "        host — installing them here would silently configure the wrong system."
        echo ""
        echo "        Run this from the host shell instead. From a distrobox container:"
        echo "            distrobox-host-exec ./scripts/setup_host.sh local $*"
        echo ""
        echo "        Set ROVER_SETUP_ALLOW_CONTAINER=1 to override (container-only testing)."
        exit 1
    fi
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
# Hands the controller light bar (DualSense or DualShock 4) to group 'plugdev'
# so joystick_interpreter can paint it with the drive state without running as
# root. The rule sets the group; the user must already be a member of plugdev.
if [ "$INSTALL_JOYSTICK_LED" = true ]; then
    echo ">>> Copying udev rules..."
    run_sudo cp "$CONFIG_DIR/rules.d/99-playstation-led.rules" /etc/udev/rules.d/99-playstation-led.rules

    echo ">>> Reloading udev rules..."
    run_sudo udevadm control --reload-rules

    # Re-emit 'add' for a light bar that is already connected, so it does not
    # have to be re-paired. Scoped to the LEDs actually owned by the
    # hid-playstation driver: a bare --subsystem-match=leds would replay every
    # unrelated LED rule on the host (keyboard backlight, capslock, rfkill, ...).
    echo ">>> Re-triggering PlayStation light bar devices..."
    TRIGGERED=0
    for led in /sys/class/leds/*; do
        [ -e "$led/device/driver" ] || continue
        [ "$(basename "$(readlink -f "$led/device/driver")")" = "playstation" ] || continue
        run_sudo udevadm trigger --action=add "$led"
        TRIGGERED=$((TRIGGERED + 1))
    done

    if [ "$TRIGGERED" -eq 0 ]; then
        echo "[NOTE] No PlayStation light bar connected right now — the rule will"
        echo "       apply the next time a controller is plugged in or paired."
    fi
    echo "[SUCCESS] udev rules applied."
fi
