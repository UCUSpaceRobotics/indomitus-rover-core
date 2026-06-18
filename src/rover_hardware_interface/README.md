# rover_hardware_interface

ROS2 `hardware_interface::SystemInterface` for all rover chassis motors:
- **Steadywin** — steer motors (V3.06b0 custom CAN protocol)
- **Damiao** — drive motors (MIT-style CAN protocol)

Communicates directly with `can0` via SocketCAN.

## Architecture

```
ros2_control
    ↓
RoverHardware (SystemInterface)
    ↙               ↘
Steadywin         Damiao
steer motors      drive motors
    ↘               ↙
        can0
```

## State & Command Interfaces

| Joint | Command | State |
|---|---|---|
| `*_steer` | `position` | `position` |
| `*_drive` | `velocity` | `velocity`, `position` |

Joint names follow URDF: `fl_steer`, `fr_steer`, `rl_steer`, `rr_steer`, `fl_drive`, `fr_drive`, `rl_drive`, `rr_drive`.

## Motor IDs (`config/chassis_driver.yaml`)

| Role | Type | IDs [FL, FR, RL, RR] |
|---|---|---|
| Steer | Steadywin | 11, 13, 15, 17 |
| Drive | Damiao | 10, 12, 14, 16 |

## CAN Protocol Notes

- **Steadywin steer** — absolute position via `0xC2` (counts = `angle_rad × 16384 / 2π`). Init: clear fault (`0xAF`) → position=0 (`0xC2`)
- **Damiao drive** — velocity via `0x200+id` with IEEE 754 float (rad/s). Init: set velocity mode (`0x7FF`, extended frame) → enable (`0xFC`)
- `0x7FF` must be sent as extended frame due to a ros2_socketcan bug on Humble

## Lifecycle

Motors are **disabled by default** on activation. Enable/disable is controlled via:

```
/controller_manager/set_hardware_component_state
```

Called from `joystick_interpreter_node` on button press. On deactivation — zero commands are sent and motors are disabled gracefully.

## Known Issues

- Damiao voltage/current not available via CAN (UART debug interface only)
- No CAN bus loss watchdog — if `can0` drops mid-operation, motors hold last command until hardware timeout
