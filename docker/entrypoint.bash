#!/usr/bin/bash

set -e

TARGET_IFACE="${CAN_INTERFACE:-can0}"
TARGET_BITRATE="${CAN_BITRATE:-1000000}"
TARGET_QUEUE_SIZE="${CAN_TRANSMIT_QUEUE_SIZE:-1000}"

source_if_exists() {
    local script_path="$1"

    if [ -f "$script_path" ]; then
        # shellcheck disable=SC1090
        source "$script_path"
    fi
}

# -------------------- CAN Interface Setup --------------------

echo "[CAN] INFO: Checking for CAN hardware on interface: ${TARGET_IFACE}"

if ip link show "${TARGET_IFACE}" > /dev/null 2>&1; then
    echo "[CAN] INFO: Hardware found. Configuring ${TARGET_IFACE} at ${TARGET_BITRATE} bps with queue size ${TARGET_QUEUE_SIZE}..."

    ip link set "${TARGET_IFACE}" down 2>/dev/null || true    
    ip link set "${TARGET_IFACE}" type can bitrate "${TARGET_BITRATE}"    
    ip link set "${TARGET_IFACE}" txqueuelen "${TARGET_QUEUE_SIZE}"
    ip link set "${TARGET_IFACE}" up

    echo "[CAN] SUCCESS: Interface '${TARGET_IFACE}' configured at ${TARGET_BITRATE} bps."
else
    echo "[CAN] WARNING: Interface '${TARGET_IFACE}' not found. Skipping setup."
fi


# -------------------- ROS2 Workspace Setup --------------------

source_if_exists "/opt/ros/${ROS_DISTRO}/setup.bash"
source_if_exists "/opt/ws/install/setup.bash"

echo "[ROS] SUCCESS: Environment ready (${ROS_DISTRO})."

# -------------------- Launch ROS2 nodes --------------------
if [ "${1}" = "autolaunch" ]; then
    echo "[ROVER] Starting launch file 1..."
    ros2 launch rover_bringup rover.launch.py &
    PID1=$!

    export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra:$LD_LIBRARY_PATH

    echo "[ROVER] Starting launch file 2..."
    ros2 launch rover_bringup joy.launch.py &
    PID2=$!

    # wait -n $PID1
    wait -n $PID1 $PID2
    exit $?
fi

exec "$@"