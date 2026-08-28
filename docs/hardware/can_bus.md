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
| `0x014–0x016` | Arm: Steadywin joint motors (mount/shoulder/elbow) |
| `0x017–0x019` | Arm: Damiao wrist motors |
| `0x01A–0x01B` | Arm end-effector: jaw gripper cmd / reply (ESP32 SAFE gripper firmware) |
| `0x01C–0x01D` | Arm end-effector: astro-bio gripper cmd / reply (reserved, no firmware yet) |
| `0x01E–0x01F` | Arm end-effector: drill_sampling gripper cmd / reply (ESP32 claw+drill+lock firmware) |
| `0x10A–0x111` | Motor position / angle communication |
| `0x20A–0x210` | Damiao velocity commands |
| `0x300–0x30F` | ESP32 lighting controller |
| `0x7FF` | Damiao register read/write service |

### Notes

> ⚠️ Do **not** allocate IDs inside any reserved range!!! ⚠️

### Arm

Joint motor CAN IDs (`arm_hardware_interface`):

| Joint | Motor Type | CAN ID (dec) | CAN ID (hex) |
|-------|------------|--------------|--------------|
| `arm_mount_base_joint` | Steadywin | 20 | `0x14` |
| `arm_base_shoulder_joint` | Steadywin | 21 | `0x15` |
| `arm_shoulder_forearm_joint` | Steadywin | 22 | `0x16` |
| `arm_forearm_wrist_1_joint` | Damiao | 23 | `0x17` |
| `arm_wrist_1_wrist_2_joint` | Damiao | 24 | `0x18` |
| `arm_wrist_2_end_effector_joint` | Damiao | 25 | `0x19` |

Damiao wrist motors re-flashed with their own Master ID additionally answer feedback on `0x400 | motor_id` (e.g. motor 23 -> `0x417`) — see `scripts/arm/check_calibration.py`'s `DM_MASTER_IDS`.

#### End-effector grippers

Only one tool is physically mounted at a time, so only one cmd/reply pair below is ever live on the bus — all three are reserved up front so swapping tools never needs re-numbering.

| Tool | Cmd ID | Reply ID | Status |
|------|--------|----------|--------|
| jaw | `0x1A` | `0x1B` | ESP32 SAFE gripper firmware — see `arm_peripherals/end_effector_can_node.py` |
| astro-bio | `0x1C` | `0x1D` | Reserved, no firmware yet |
| drill_sampling | `0x1E` | `0x1F` | ESP32 claw+drill+lock firmware — see `arm_peripherals/end_effector_can_node.py` |

The reply ID carries every reply kind for that tool (ACKs and READ_* data) with **no tag byte** identifying which — the protocol is strictly one-outstanding-request-at-a-time, so the requester already knows which layout to expect from what it just sent. A client that can't rely on that has to fall back on DLC to tell replies apart (see `arm_peripherals/end_effector_can_node.py`'s `_on_can_frame()` for the caveat that comes with doing that). See the gripper firmware's own CAN API doc for the full command set (`SAFE_SET_SPREAD/ANGLE`, `SAFE_OPEN/CLOSE`, `START/STOP_SAFE_HOLD`, `READ_*`) and each reply's exact byte layout.

```bash
# jaw: SAFE_OPEN / SAFE_CLOSE
cansend can0 01A#03
cansend can0 01A#04
```

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