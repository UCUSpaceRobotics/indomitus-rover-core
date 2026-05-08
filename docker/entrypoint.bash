#!/usr/bin/bash

set -e

source /opt/ros/${ROS_DISTRO}/setup.bash

if [ -d /opt/ws/src ] && [ "$(ls -A  /opt/ws/src 2> /dev/null)" ]; then
    if [ ! -f /opt/ws/install/setup.bash ] || [ /opt/ws/src -nt /opt/ws/install/setup.bash ]; then
        echo "Building workspace..."

        rosdep install --from-paths src  --ignore-src -r -y || true

        colcon build --symlink-install --packages-skip indomitus_rover_sim
    fi
fi

if [ -f /opt/ws/install/setup.bash ]; then
    source /opt/ws/install/setup.bash
fi

echo "ROS ${ROS_DISTRO} ready. Workspace: /opt/ws"

exec "$@"
