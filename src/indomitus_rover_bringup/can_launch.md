## ROS 2 SocketCAN Docker Bridge

### Hardware Initialization Sequence

The physical CAN interface is automatically configured exactly when the Docker container boots. The container's `entrypoint.sh` script executes before any ROS 2 processes start. It reads the environment variables to safely configure and bring the hardware interface `UP` via native Linux networking commands. Afterwards, the ROS 2 launch file runs a pre-flight validation check to guarantee the interface is active before allocating memory for the nodes.

### Docker Configuration

Because the container manages physical hardware, it requires host networking and specific Linux kernel capabilities. Add the following to your docker-compose.yml:

```yaml
network_mode: host
    cap_add:
      - NET_ADMIN
      - NET_RAW
    environment:
      - CAN_INTERFACE=can0
      - CAN_BITRATE=1000000
```

> **NOTE:** This configuration is Linux-only and will not work natively on macOS or Windows.

### Environment Variables

These variables configure the physical hardware during the container's boot sequence before ROS 2 starts:

* CAN_INTERFACE (Default: can0): The physical or virtual CAN interface name.
* CAN_BITRATE (Default: 1000000): The baud rate applied to the interface.

### Usage

Start the sender and receiver lifecycle nodes using the provided launch script:

`bash ros2 launch indomitus_rover_bringup can_bridge.launch.py interface:=can0 `

#### Launch Arguments

* interface (Default: can0): The SocketCAN interface the ROS nodes should bind to.
* sender_timeout_sec (Default: 0.01): Hardware timeout wait time before retrying.
* receiver_interval_sec (Default: 0.01): Polling interval for the receiver socket.