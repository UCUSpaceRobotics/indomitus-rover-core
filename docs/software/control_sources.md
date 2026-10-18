# Control sources

- **Source** = driver + its own interpreter. It sends only a `Twist` on its own `cmd_vel_<source>`
  and `SetBool` / `.../toggle` requests to state owners. Sources know nothing about each other.
- **twist_mux alone picks who drives** (`rover_drive/config/twist_mux.yaml`). Keep
  `drive_source_lamp_node`'s `DEFAULT_SOURCES` in sync by hand.
- **Owners hold state** (`drive_manager`, lights, cameras) and publish it latched; sources read it
  back, never keep a copy.
- **Rule A:** each source has its own raw topic and node names (two pads on one `joy` → a toggle
  fires twice and undoes itself).
- **Rule B:** one publisher per twist_mux input.

| source | raw input | twist_mux | priority |
|---|---|---|---|
| rover gamepad (wired) | `/rover/joy` | `cmd_vel_joy` | 100 |
| ground station | `/gs/joy`, `/gs/switches` | `cmd_vel_gs` | 50 |
| laptop gamepad **or** keyboard (one at a time) | `/rover/remote/joy`, `/rover/remote/keyboard/joy` | `cmd_vel_remote` | 30 |
| Nav2 | — | `cmd_vel_nav` | 20 |
| external | — | `cmd_vel_ext` | 10 |
| LoRa fallback | radio | `cmd_vel_lora` | 5 |

- Laptop nodes stay in `/rover` with a `remote_` prefix, so relative service names hit the rover.
- Start: rover pad — `rover.launch.py` (`source:=rover`); laptop pad —
  `ros2 launch rover_teleop gamepad.launch.py` (default `source:=remote`); keyboard —
  `ros2 run rover_teleop keyboard_teleop_node`.
- New source: own topics/names → `twist_mux.yaml` + `DEFAULT_SOURCES` → requests only → a row above.
- Open: GS can set the traffic lamps directly, but `drive_source_lamp_node` / `gs_link_lamp_node`
  overwrite them on the next change.
