#!/usr/bin/bash

set -e

source_if_exists() {
    local script_path="$1"

    if [ -f "$script_path" ]; then
        # shellcheck disable=SC1090
        source "$script_path"
    fi
}

# -------------------- Fast DDS profile --------------------
#
# Regenerate the Fast DDS profile on every start, so it can never describe a
# topology that is no longer plugged in. The generator picks 'linked' mode when
# an address in the rover link subnet exists and 'local' mode when it does not;
# a profile left over from the other mode is the difference between a working
# `ros2 topic list` and one that returns nothing but /parameter_events and
# /rosout.
#
# Armed by DDS_PROFILE_OUT, not by FASTRTPS_DEFAULT_PROFILES_FILE, and that
# distinction is deliberate: docker-compose.dev.gs.yaml runs this container on
# the ground-station laptop and points it at the GS's own generated profile
# through a READ-ONLY mount. It sets the profile variable but not this one, so
# nothing here tries to overwrite a file it does not own. Only
# docker-compose.prod.yaml — the rover, which has no GS checkout to borrow
# from — sets DDS_PROFILE_OUT and gets a profile of its own.
#
# When it is armed it is also mandatory: a FASTRTPS_DEFAULT_PROFILES_FILE
# pointing at a missing file does not stop Fast DDS. It logs one XMLPARSER line
# and falls back to plain multicast, which cannot reach the ground station
# across the mast Pi. Every symptom then points at the network instead of at
# this file, so fail here where the cause is still visible.
if [ -n "${DDS_PROFILE_OUT:-}" ]; then
    if [ ! -x /usr/local/bin/gen-dds-profile.sh ]; then
        echo "[DDS] error: /usr/local/bin/gen-dds-profile.sh is missing or not" >&2
        echo "      executable; the image is older than this entrypoint." >&2
        exit 1
    fi
    if ! /usr/local/bin/gen-dds-profile.sh; then
        echo "[DDS] error: could not generate the DDS profile; refusing to start" >&2
        echo "      with discovery that cannot reach the ground station." >&2
        exit 1
    fi
    if [ "${DDS_PROFILE_OUT}" != "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then
        echo "[DDS] warning: generated ${DDS_PROFILE_OUT} but Fast DDS is pointed" >&2
        echo "      at '${FASTRTPS_DEFAULT_PROFILES_FILE:-<unset>}' — the profile" >&2
        echo "      just written will not be read." >&2
    fi
fi

# -------------------- ROS2 Workspace Setup --------------------

TARGET_ROS_DISTRO="${TARGET_ROS_DISTRO:-humble}"
WS="${WS:-/opt/ws}"

source_if_exists "/opt/ros/${TARGET_ROS_DISTRO}/setup.bash"
source_if_exists "${WS}/install/setup.bash"

echo "[ROS] SUCCESS: Environment ready (${TARGET_ROS_DISTRO})."

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
