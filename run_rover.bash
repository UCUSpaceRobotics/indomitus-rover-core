#!/bin/bash
# Start the full rover stack:
#   can_bridge              (ros2_socketcan: /to_can_bus <-> can0 hardware, bidirectional)
#   rover_kinematics_node   (cmd_vel -> wheel_targets)
#   chassis_driver_node     (wheel_targets -> /to_can_bus; handles motor init + shutdown)
#
# Before running this script, set up CAN on the HOST (not inside Docker):
#   sudo ip link set can0 up type can bitrate 1000000
#   sudo ip link set can0 txqueuelen 1000
#
# Motor enable is sent by chassis_driver_node 3s after startup.
# Ctrl+C causes chassis_driver_node to zero commands, wait 1.5s, then disable all motors.

source /opt/ros/humble/setup.bash
[ -f /opt/ws/install/setup.bash ] && source /opt/ws/install/setup.bash
source /work/install/local_setup.bash

# Kill any leftover nodes from previous runs
pkill -f chassis_driver_node   2>/dev/null || true
pkill -f rover_kinematics_node 2>/dev/null || true
pkill -f socket_can_sender     2>/dev/null || true
pkill -f socket_can_receiver   2>/dev/null || true
sleep 0.3

echo "[rover] Starting can_bridge (ros2_socketcan on can0)..."
ros2 launch can_bridge can_bridge.launch.py interface:=can0 &
BRIDGE_PID=$!

# Give socketcan lifecycle nodes time to configure and activate
sleep 1.5

echo "[rover] Starting rover_kinematics_node..."
ros2 run indomitus_rover_control rover_kinematics_node &
KIN_PID=$!

echo "[rover] Starting chassis_driver_node..."
ros2 run chassis_driver chassis_driver_node \
    --ros-args --params-file /work/src/chassis_driver/config/chassis_driver.yaml &
DRIVER_PID=$!

echo "[rover] All nodes running. Ctrl+C to stop."
wait $KIN_PID $DRIVER_PID $BRIDGE_PID
