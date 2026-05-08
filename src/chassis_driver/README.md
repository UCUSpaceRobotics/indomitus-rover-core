# chassis_driver

ROS2 driver for all rover chassis motors:
- **Steadywin** — steer/rotation motors (V3.06b0 custom CAN protocol)
- **Damiao** — drive motors (MIT-style CAN protocol)

## Architecture

```
/cmd_vel  →  rover_kinematics_node  →  /wheel_targets
                                             ↓
                                    chassis_driver_node
                                       ↙           ↘
                              /to_can_bus       /from_can_bus
                                   ↕                  ↕
                            ros2_socketcan  ↔  can0 hardware
```

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/wheel_targets` | `indomitus_msgs/WheelTargets` | in | Per-wheel steer angles + drive speeds |
| `/to_can_bus` | `can_msgs/Frame` | out | CAN frames to hardware |
| `/from_can_bus` | `can_msgs/Frame` | in | CAN frames from hardware |
| `/joint_states` | `sensor_msgs/JointState` | out | Motor positions and velocities |
| `/chassis/motor_states` | `indomitus_msgs/ChassisStatus` | out | Full motor status (10 Hz) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | out | Health status (1 Hz) |

## Motor IDs (config: `config/chassis_driver.yaml`)

| Role | Motor type | IDs [FL, FR, RL, RR] |
|---|---|---|
| Steer | Steadywin | 11, 13, 15, 17 |
| Drive | Damiao | 10, 12, 14, 16 |

## Running

```bash
# On host — set up CAN interface first
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000

# Inside Docker
bash /work/run_rover.bash
```

Motors enable automatically 3 seconds after startup.

---

## Status

### ✅ Working

- **Command translation** — `WheelTargets` → correct CAN frames per motor type
  - Steadywin steer: absolute position via `0xC2` (counts = angle_rad × 16384 / 2π)
  - Damiao drive: velocity via `0x200+id` with IEEE 754 float (rad/s)
- **Motor init sequence**
  - Steadywin: clear fault (`0xAF`) → position=0 (`0xC2`) to activate
  - Damiao: setMode velocity mode (`0x7FF`, extended frame workaround) → enable (`0xFC`)
- **Graceful shutdown** — zero commands → 1.5s settle → disable all motors on Ctrl+C
- **Steadywin feedback** — `0xAE` status (voltage, current, temp, mode, fault) + `0xA3` position (multi-turn) polled at 1 Hz, all 4 motors confirmed working
- **Damiao feedback** — all 4 drive motors confirmed: position, velocity, torque, MOS/rotor temp, ERR code via MIT feedback at 1 Hz poll
  - Motors respond at their own ESC_ID (not MST_ID=0); `onCanFrame` handles both cases
- **`/diagnostics`** — all 8 motors reporting correctly at 1 Hz
- **`/joint_states`** — position and velocity publishing for all connected motors
- **CAN frame routing** — ros2_socketcan bidirectional; feedback dispatched by CAN ID

### ⚠️ Needs testing

- **`/chassis/motor_states`** — publishes at 10 Hz, data correct, not validated under motion load
- **Kinematics accuracy** — Ackermann geometry pipeline works end-to-end; physical accuracy (turning radius, wheel slip) not yet measured
- **Error recovery** — fault codes reported in diagnostics; no tested re-enable procedure after fault

### ❌ Not yet implemented

- **CAN bus loss detection** — no watchdog; if CAN drops mid-operation, motors keep last command until timeout
- **Damiao voltage/current** — not available via CAN protocol (UART debug interface only)

### Known issues

- `ros2_socketcan` rejects standard CAN ID `0x7FF` on the installed Humble version (likely `>=` bug in the sender). **Workaround**: `is_extended = true` on `0x7FF` frames. Confirmed working — Damiao motors accept extended-flag frames at this address.
- Damiao motor 16 (decimal) has ESC ID `0x10`; the MIT feedback stores the motor ID in the low nibble of `data[0]` (4 bits, max 15). Motor 16 will never match in `parseFeedback()`. Needs investigation — possible register-level ID remapping.
