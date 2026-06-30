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

## On-Rover Setup (Jetson)

CAN interface configuration and container startup are fully automated on the Jetson via udev rules and a systemd service.

### udev Rule

Located at [`/etc/udev/rules.d/can.rules`](../system/rules.d/can.rules).

Automatically configures and brings up `can0` when the CAN adapter is detected.

### systemd Service

Located at [`/etc/systemd/system/rover.service`](../system/systemd/rover.service).

Starts the production Docker container once both Docker and `can0` are available.

## Manual / Laptop Setup

When running from a laptop with a CAN-to-USB adapter, the interface must be configured manually:

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
```

Then launch the CAN bridge:

```bash
ros2 launch rover_bringup can.launch.py interface:=can0
```

| Argument | Default | Description |
|----------|---------|-------------|
| `interface` | `can0` | SocketCAN interface for ROS nodes |
| `sender_timeout_sec` | `0.01` | Hardware timeout before retry |
| `receiver_interval_sec` | `0.01` | Polling interval for receiver socket |

> **Note:** SocketCAN is Linux-only — will not work on macOS or Windows.
