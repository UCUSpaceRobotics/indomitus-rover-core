# Control sources

How everything that can drive the rover fits together, so that a new one can be
added without fighting the existing ones or needing a manager of its own.

## Three roles

```
 SOURCES (any number)              MOTION ARBITER           STATE OWNERS
 driver -> interpreter --Twist-->  twist_mux --cmd_vel-->   swerve controller
                       --service calls----------------->   drive_manager, lights, arm_power
                       <--latched state----------------    drive/state, lights/state, …
```

1. **A source** is a driver (gamepad, console boards, keyboard, radio) plus its
   own interpreter. It hands the rest of the system exactly two things:
   - a `Twist` on **its own** twist_mux input, `cmd_vel_<source>`;
   - requests to the state owners — `SetBool` for a latching switch, the
     `.../toggle` `Trigger` for a momentary button.

   Sources know nothing about each other.
2. **Only twist_mux decides who drives**, by priority, from
   `rover_drive/config/twist_mux.yaml`. `drive_source_lamp_node` reads the same
   file, so the tower lamp follows a new input without code changes.
3. **Only the owners hold state** (`drive_manager` for motors, controller and
   compact mode; the lighting node for lights; `arm_power` for the arm). They
   publish it latched and every source reads it back from there instead of
   keeping a copy, so two operators cannot drift apart: the last request wins
   and both see the result.

## Two rules that keep sources from colliding

- **A. A source's raw input and node names are its own.** Two gamepads must not
  share a `joy` topic: both interpreters would act on every press, and a
  `.../toggle` sent twice undoes itself.
- **B. One publisher per twist_mux input.** Two sources on one input interleave
  their messages and the rover jitters between them.

## The sources

| source | runs on | raw input | twist_mux input | priority |
|---|---|---|---|---|
| rover gamepad (wired) | rover | `/rover/joy` | `cmd_vel_joy` | 100 |
| ground station | ground station | `/gs/joy`, `/gs/switches` | `cmd_vel_gs` | 50 |
| operator laptop (gamepad **or** keyboard) | a laptop | `/rover/remote/joy`, `/rover/remote/keyboard/joy` | `cmd_vel_remote` | 30 |
| Nav2 | rover | — | `cmd_vel_nav` | 20 |
| external | — | — | `cmd_vel_ext` | 10 |
| LoRa fallback (relays the ground station's `cmd_vel_gs`) | rover | radio | `cmd_vel_lora` | 5 |

The ground station is its own system (`../indomitus-ground-station`, everything
under `/gs`), not "a laptop". Laptops are plain operator stations: any number may
be connected, but **only one drives at a time** (gamepad or keyboard, not both
on different laptops) — they share `cmd_vel_remote`.

Laptop nodes stay in the `/rover` namespace with a `remote_` prefix
(`/rover/remote_gamepad_interpreter`, `/rover/remote_keyboard_interpreter`): a
namespace here means the subsystem being controlled, and the interpreter's
relative service names (`drive/power/toggle`, `lights/…`) must resolve to the
rover's.

## Starting each one

```bash
# rover gamepad: started by rover_bringup/rover.launch.py (use_joy:=true),
# which passes source:=rover explicitly
# laptop gamepad (source:=remote is the default, on purpose: a forgotten
# argument typed on a laptop must not land on the rover gamepad's topics)
ros2 launch rover_teleop gamepad.launch.py
# laptop keyboard
ros2 run rover_teleop keyboard_teleop_node
```

## Adding a source

1. An interpreter whose raw input and node names are its own (rule A).
2. A new entry in `twist_mux.yaml` with its own topic and a priority (rule B).
   If it is autonomous, add its topic to `drive_source_lamp_node`'s
   `autonomous_topics`.
3. State changes only as requests to the owners; read their state back.
4. A row in the table above.

## Open

- **Traffic lamp has several writers.** The ground station can set
  `lights/traffic_red|green|blue` directly, while `drive_source_lamp_node`
  (red/green) and `gs_link_lamp_node` (blue) own them too; the next change of
  driving source overwrites the manual value. Either drop those ground-station
  functions or give the lamps a manual-override mode.
- **The arm has no arbiter.** `gamepad_servo_node` listens to one Joy topic, so
  only one source drives the arm, chosen by remapping at launch. A second arm
  source would need a mux of its own (twist_mux supports `TwistStamped` via
  `use_stamped`).
