# Motor CAN Enable/Disable Commands

This document lists `cansend` commands to enable and disable the Damiao and SteadyWin motors on the bus. All commands assume the CAN interface is named `can0` — replace with your actual interface name if different.

---

## Damiao Motors (IDs 10, 12, 16, 14)

### MIT Mode (primary method)

In MIT mode, the CAN frame ID equals the motor's configured CAN ID directly. Damiao's standard MIT-mode convention (compatible with the original MIT Cheetah protocol) uses these special 8-byte frames:

- **Enable**: `FF FF FF FF FF FF FF FC`
- **Disable**: `FF FF FF FF FF FF FF FD`

| Motor ID (dec) | Motor ID (hex) | Enable command | Disable command |
|---|---|---|---|
| 10 | 0x0A | `cansend can0 00A#FFFFFFFFFFFFFFFC` | `cansend can0 00A#FFFFFFFFFFFFFFFD` |
| 12 | 0x0C | `cansend can0 00C#FFFFFFFFFFFFFFFC` | `cansend can0 00C#FFFFFFFFFFFFFFFD` |
| 16 | 0x10 | `cansend can0 010#FFFFFFFFFFFFFFFC` | `cansend can0 010#FFFFFFFFFFFFFFFD` |
| 14 | 0x0E | `cansend can0 00E#FFFFFFFFFFFFFFFC` | `cansend can0 00E#FFFFFFFFFFFFFFFD` |

```bash
# Enable
cansend can0 00A#FFFFFFFFFFFFFFFC
cansend can0 00C#FFFFFFFFFFFFFFFC
cansend can0 010#FFFFFFFFFFFFFFFC
cansend can0 00E#FFFFFFFFFFFFFFFC

# Disable
cansend can0 00A#FFFFFFFFFFFFFFFD
cansend can0 00C#FFFFFFFFFFFFFFFD
cansend can0 010#FFFFFFFFFFFFFFFD
cansend can0 00E#FFFFFFFFFFFFFFFD
```

### Alternative: Custom CAN Protocol V3.06b0 (Dev_addr-based)

If the motors are instead configured for the "Custom CAN communication protocol" (not MIT mode), the frame ID is the device address (`Dev_addr`) itself, not offset.

- **Disable** — command code `0xCF`, DLC = 1:

```bash
cansend can0 00A#CF   # Dev_addr 10
cansend can0 00C#CF   # Dev_addr 12
cansend can0 010#CF   # Dev_addr 16
cansend can0 00E#CF   # Dev_addr 14
```

- **Enable** — this protocol has no dedicated enable command. Sending any control command (e.g. `0xC0` torque, `0xC1` speed, `0xC2`/`0xC3` position) automatically re-engages the motor after it has been disabled with `0xCF`.

> ⚠️ Confirm which protocol/mode your motors are actually configured for before using these commands — sending MIT-mode frames to a motor expecting the custom protocol (or vice versa) will not work as intended.

---

## SteadyWin Motors (IDs 11, 13, 17, 15)

SteadyWin's GIM series drives use an ODrive-derived CAN protocol:

```
CAN ID = (node_id << 5) + cmd_id
```

`Set_Axis_State` uses `cmd_id = 0x007`. Relevant states:
- `1` = IDLE (disabled)
- `8` = CLOSED_LOOP_CONTROL (enabled)

Data frame: 8 bytes, little-endian `uint32` state value in the first 4 bytes, remaining 4 bytes zero-padded.

| Motor node_id (dec) | CAN ID (hex) | Enable command | Disable command |
|---|---|---|---|
| 11 | 0x167 | `cansend can0 167#0800000000000000` | `cansend can0 167#0100000000000000` |
| 13 | 0x1A7 | `cansend can0 1A7#0800000000000000` | `cansend can0 1A7#0100000000000000` |
| 17 | 0x227 | `cansend can0 227#0800000000000000` | `cansend can0 227#0100000000000000` |
| 15 | 0x1E7 | `cansend can0 1E7#0800000000000000` | `cansend can0 1E7#0100000000000000` |

```bash
# Enable (closed-loop control)
cansend can0 167#0800000000000000
cansend can0 1A7#0800000000000000
cansend can0 227#0800000000000000
cansend can0 1E7#0800000000000000

# Disable (idle)
cansend can0 167#0100000000000000
cansend can0 1A7#0100000000000000
cansend can0 227#0100000000000000
cansend can0 1E7#0100000000000000
```

---

## Sources

- `Custom_CAN_communication_protocol_V3_06b0.pdf` (Damiao custom protocol, `0xCF` disable command)
- `DAMIAO-DM-J10010L-2EC-User_Manual` (MIT mode control frame format)
- SteadyWin GIM6010-8 Instruction Manual (ODrive-based CAN protocol, `Set_Axis_State` message)