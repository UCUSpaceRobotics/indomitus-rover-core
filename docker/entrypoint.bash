#!/usr/bin/bash

set -e

TARGET_IFACE="${CAN_INTERFACE:-can0}"
TARGET_BITRATE="${CAN_BITRATE:-1000000}"


# -------------------- CAN Interface Setup --------------------

echo "[CAN] INFO: Checking for CAN hardware on interface: ${TARGET_IFACE}"

if ip link show "${TARGET_IFACE}" > /dev/null 2>&1; then
    echo "[CAN] INFO: Hardware found. Configuring ${TARGET_IFACE} at ${TARGET_BITRATE} bps..."

    ip link set "${TARGET_IFACE}" down 2>/dev/null || true
    ip link set "${TARGET_IFACE}" txqueuelen 1000 type can bitrate "${TARGET_BITRATE}"
    ip link set "${TARGET_IFACE}" up

    echo "[CAN] SUCCESS: Interface '${TARGET_IFACE}' configured at ${TARGET_BITRATE} bps."
else
    echo "[CAN] WARNING: Interface '${TARGET_IFACE}' not found. Skipping setup."
fi


# -------------------- ROS2 Workspace Setup --------------------

source /opt/ros/${ROS_DISTRO}/setup.bash

if [ -d /opt/ws/src ] && [ "$(ls -A  /opt/ws/src 2> /dev/null)" ]; then
    if [ ! -f /opt/ws/install/setup.bash ] || [ /opt/ws/src -nt /opt/ws/install/setup.bash ]; then
        echo "[ROS] INFO: Building workspace /opt/ws..."

        rosdep install --from-paths src  --ignore-src -r -y || true

        colcon build --symlink-install --packages-skip indomitus_rover_sim
    fi
fi

if [ -f /opt/ws/install/setup.bash ]; then
    source /opt/ws/install/setup.bash
fi

echo "[ROS] SUCCESS: Environment ready (${ROS_DISTRO})."

exec "$@"