# CAN Bus

## Wiring

This wiring color scheme applies to both the rover and the robotic arm.

|  **Signal**  | **Color** |
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

## CAN Address Space

The rover uses the **standard 11-bit CAN identifier** space (`0x000–0x7FF`).

To simplify SocketCAN filtering and avoid ID collisions, CAN identifiers are reserved by prefix whenever possible.

| CAN ID Range | Purpose |
|--------------|---------|
| `0x00A–0x011` | Motor ESC IDs |
| `0x10A–0x111` | Motor position / angle communication |
| `0x20A–0x210` | Damiao velocity commands |
| `0x300–0x30F` | ESP32 lighting controller |
| `0x7FF` | Damiao register read/write service |

### Notes

> ⚠️ Do **not** allocate IDs inside any reserved range!!! ⚠️

### Helper

Set motors to 0:
```bash
# Set origin for Motor ID 11 (0x0B)      
cansend can0 00B#B1

# Set origin for Motor ID 13 (0x0D)
cansend can0 00D#B1

# Set origin for Motor ID 15 (0x0F)
cansend can0 00F#B1

# Set origin for Motor ID 17 (0x11)
cansend can0 011#B1
```