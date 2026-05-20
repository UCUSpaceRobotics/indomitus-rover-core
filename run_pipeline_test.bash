#!/bin/bash
source /opt/ros/humble/setup.bash
# source old workspace packages (can_bridge, sim, description)
[ -f /opt/ws/install/setup.bash ] && source /opt/ws/install/setup.bash
# source our new packages (damiao_driver, indomitus_interfaces, rover_control)
source /work/install/local_setup.bash

ros2 run indomitus_rover_control rover_kinematics_node > /tmp/kin.txt 2>&1 &
KIN_PID=$!

ros2 run damiao_driver damiao_driver_node \
  --ros-args --params-file /work/src/damiao_driver/config/damiao_driver.yaml \
  > /tmp/driver.txt 2>&1 &
DRIVER_PID=$!

sleep 2

python3 /work/test_pipeline.py

kill $KIN_PID $DRIVER_PID 2>/dev/null
