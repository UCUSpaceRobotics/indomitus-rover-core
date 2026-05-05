#!/usr/bin/env bash
# Configures a SocketCAN interface on the host.
# Usage: ./scripts/can_setup.sh [interface] [bitrate]
# Example: ./scripts/can_setup.sh can0 1000000

set -euo pipefail

INTERFACE="${1:-can0}"
BITRATE="${2:-1000000}"

echo "[CAN] Configuring ${INTERFACE} at ${BITRATE} bit/s..."

if ip link show "${INTERFACE}" | grep -q "UP"; then
    echo "[CAN] ${INTERFACE} is UP — bringing down to reconfigure..."
    sudo ip link set "${INTERFACE}" down
fi

sudo ip link set "${INTERFACE}" up type can bitrate "${BITRATE}"

echo "[CAN] ${INTERFACE} is UP at ${BITRATE} bit/s"
ip -details link show "${INTERFACE}"
