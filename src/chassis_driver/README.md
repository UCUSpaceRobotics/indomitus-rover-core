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
  - Damiao: setMode register write (`0x7FF`, extended frame workaround) → enable (`0xFC`)
- **Graceful shutdown** — zero commands → 1.5s settle → disable all motors on Ctrl+C
- **Steadywin status feedback** — `0xAE` polled at 1 Hz; motor 13 confirmed responding:
  - Voltage, current, temperature, operation mode, fault code parsed correctly
- **CAN frame routing** — ros2_socketcan bidirectional; `from_can_bus` dispatched to correct motor by CAN ID

### ⚠️ Partially working

- **Diagnostics (Steadywin)** — works for connected motors; unconnected motors show "No status received" (expected). Not fully tested with all 4 steer motors.
- **Damiao status polling** — read register `0x33` queries are sent at 1 Hz via `0x7FF` extended frame. No response yet. Response parsing via `parseFeedback()` at `mst_id=0x000` is implemented but untested with real hardware.

### ❌ Not yet tested / To implement

- **`/chassis/motor_states`** — published at 10 Hz but not validated end-to-end with real motor data
- **`/joint_states`** — Steadywin position from `0xC2`/`0xA3` response; Damiao pos/vel from MIT feedback. Not validated against known positions.
- **All 4 steer motors** — only motor 13 connected during development
- **All 4 drive motors** — no Damiao motors connected during development
- **Kinematics accuracy** — wheel targets from `rover_kinematics_node` not yet validated against expected physical motion (Ackermann geometry, turning radius, spin-in-place)
- **MIT feedback parsing** — `parseFeedback()` ported from embedded code but untested with real Damiao response frames. Motor ID nibble encoding limits IDs to 0–15; motor 16 (0x10) has nibble 0x0 — may need investigation.
- **Steadywin position feedback** — `0xA3`/`0xC2` response parsing implemented but no motor has returned a position response yet
- **Error recovery** — fault code handling exists in diagnostics but no tested recovery procedure (re-enable after fault)
- **CAN bus loss detection** — no watchdog; if CAN goes down mid-operation, no automatic stop

### Known issues

- `ros2_socketcan` rejects standard CAN ID `0x7FF` on the installed Humble version (likely `>=` bug in the sender). **Workaround**: `is_extended = true` on `0x7FF` frames. Confirmed working — Damiao motors accept extended-flag frames at this address.
- Damiao motor 16 (decimal) has ESC ID `0x10`; the MIT feedback stores the motor ID in the low nibble of `data[0]` (4 bits, max 15). Motor 16 will never match in `parseFeedback()`. Needs investigation — possible register-level ID remapping.
