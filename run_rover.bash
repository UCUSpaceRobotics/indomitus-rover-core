#!/bin/bash
# Start the full rover stack:
#   rover_kinematics_node  (cmd_vel -> wheel_targets)
#   damiao_driver_node     (wheel_targets -> /to_can_bus)
#   can_hw_bridge_node     (/to_can_bus -> cansend -> hardware)
#
# Ctrl+C triggers clean stop: zero commands -> 1.5s settle -> disable all motors

source /opt/ros/humble/setup.bash
[ -f /opt/ws/install/setup.bash ] && source /opt/ws/install/setup.bash
source /work/install/local_setup.bash

pkill -f damiao_driver_node    2>/dev/null
pkill -f rover_kinematics_node 2>/dev/null
pkill -f can_hw_bridge_node    2>/dev/null
sleep 0.3

echo "[rover] Starting rover_kinematics_node..."
ros2 run indomitus_rover_control rover_kinematics_node &
KIN_PID=$!

echo "[rover] Starting damiao_driver_node..."
ros2 run damiao_driver damiao_driver_node \
    --ros-args --params-file /work/src/damiao_driver/config/damiao_driver.yaml &
DRIVER_PID=$!

echo "[rover] Starting can_hw_bridge_node..."
ros2 run can_hw_bridge can_hw_bridge_node &
BRIDGE_PID=$!

echo "[rover] All nodes running. Ctrl+C to stop."
wait $KIN_PID $DRIVER_PID $BRIDGE_PID
