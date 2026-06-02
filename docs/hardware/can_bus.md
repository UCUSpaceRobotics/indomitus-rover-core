# CAN Bus

## Wiring

### Main Rover CAN Bus
|  Signal  | Color |
|----------|-------|
| CAN HIGH | BLACK |
| CAN LOW  | RED   |

### Arm CAN Bus
|  Signal  | Color |
|----------|-------|
| CAN HIGH | RED   |
| CAN LOW  | BLACK |

## ROS 2 SocketCAN Bridge

CAN interface is configured automatically when the Docker container boots via `entrypoint.sh`.
Launch file runs a pre-flight check to verify the interface is active before starting nodes.

### Docker Configuration

```yaml
network_mode: host
cap_add:
  - NET_ADMIN
  - NET_RAW
environment:
  - CAN_INTERFACE=can0
  - CAN_BITRATE=1000000
```

> **Note:** Linux-only, will not work on macOS or Windows.

| Variable | Default | Description |
|----------|---------|-------------|
| `CAN_INTERFACE` | `can0` | Physical or virtual CAN interface name |
| `CAN_BITRATE` | `1000000` | Baud rate applied to the interface |

### Launch

```bash
sudo ip link set can0 up type can bitrate 1000000
```

```bash
ros2 launch rover_bringup can.launch.py interface:=can0
```

| Argument | Default | Description |
|----------|---------|-------------|
| `interface` | `can0` | SocketCAN interface for ROS nodes |
| `sender_timeout_sec` | `0.01` | Hardware timeout before retry |
| `receiver_interval_sec` | `0.01` | Polling interval for receiver socket |
