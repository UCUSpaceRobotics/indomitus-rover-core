#!/usr/bin/bash

set -e

source_if_exists() {
    local script_path="$1"

    if [ -f "$script_path" ]; then
        # shellcheck disable=SC1090
        source "$script_path"
    fi
}

# -------------------- ROS2 Workspace Setup --------------------

HW_WS="/opt/hw_ws"
WS="/opt/ws"

source_if_exists "/opt/ros/${ROS_DISTRO}/setup.bash"
source_if_exists "${WH_WS}/install/setup.bash"
source_if_exists "${WS}/install/setup.bash"

echo "[ROS] SUCCESS: Environment ready (${ROS_DISTRO})."

# -------------------- Launch ROS2 nodes --------------------
if [ "${1}" = "autolaunch" ]; then
    echo "[ROVER] Starting launch file 1..."
    ros2 launch rover_bringup rover.launch.py &
    PID1=$!

    export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra:$LD_LIBRARY_PATH

    echo "[ROVER] Starting launch file 2..."
    ros2 launch rover_teleop joy.launch.py &
    PID2=$!

    # wait -n $PID1
    wait -n $PID1 $PID2
    exit $?
fi

exec "$@"