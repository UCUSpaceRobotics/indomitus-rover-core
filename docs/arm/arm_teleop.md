# Arm control (hardware + visualization)

> Arm packages live in `src/arm/`. Build them inside the Docker workspace:
> ```bash
> cd /opt/ws
> colcon build --symlink-install --packages-select-regex "^arm_"
> source install/setup.bash
> ```

## Modes

| Mode | Launch | Use case |
|---|---|---|
| GUI-only RViz | `arm_bringup/arm_standalone.launch.py gui_only:=true` | Inspect URDF / joint sliders — **no** motors, **no** ros2_control |
| MoveIt stack | `arm_moveit_config/demo.launch.py` | Plan&Execute + `servo_node` |
| Cartesian teleop | `ros2 run arm_tasks keyboard_servo_node` | After demo is up; gamepad: `joy_node` + `gamepad_servo_node` |

On the host (for RViz):

```bash
xhost +local:docker
```

## Hardware Cartesian teleop

### 1. CAN

```bash
# host
./scripts/setup_host.sh local --can   # once
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
```

### 2. Stack, then input (container)

Terminal 1 — MoveIt + controllers + Servo:

```bash
ros2 launch arm_moveit_config demo.launch.py use_fake_hardware:=false
```

Terminal 2 — keyboard (wait until spawners / `servo_node` are up):

```bash
ros2 run arm_tasks keyboard_servo_node
```

Gamepad instead of keyboard:

```bash
ros2 run joy joy_node
ros2 run arm_tasks gamepad_servo_node
```

Do not run keyboard and gamepad together — both publish `/servo_node/delta_twist_cmds`.

### 3. Controls

Press **r** (keyboard) or **A** (gamepad) — move to named **`home`**, then start Servo (switches to the streaming controller). **x** / **X** / Esc — stop Servo and give joints back to JTC.

**Keyboard** — two independent translation sets plus rotation about `arm_tcp_link` (everything is rotated into mount before publish):

| Key | Action |
|---|---|
| W / S | EEF +X / −X (mount) |
| A / D | EEF +Y / −Y (mount) |
| Q / E | EEF +Z / −Z (mount) |
| ↑ / ↓ | forward / back (camera) |
| ← / → | left / right (camera) |
| T / G | up / down (camera) |
| I / K | pitch (TCP) |
| U / O | yaw (TCP) |
| J / L | roll (TCP) |
| r | home + start Servo |
| Esc / x | exit |

WASD/QE move along **fixed mount axes** — they mean the same thing no matter how the arm is posed. The arrow block moves along the **camera's** axes, so "forward" is always where the camera and gripper are pointing right now.

The frame is `arm_camera_link` (parameter `view_frame`) rather than `arm_tcp_link` because the camera link is REP-103 — `+X` forward, `+Y` left, `+Z` up — which is the same sign convention the mount keys already use. The camera is bolted to `arm_end_effector_link` exactly like the gripper, so both point the same way (verified: camera `+X` · gripper direction = 1.0). `arm_tcp_link` inherits the EEF axes, where the gripper points along `+Z`, so it is **not** a drop-in value for `view_frame`.

Both sets are summed, so holding W and ↑ together gives the sum of the two motions. The view transform is resolved through TF on every publish, not latched at key-press, so a long arrow-key move keeps curving with the camera. If TF for the view frame does not resolve, the arrow keys are ignored and WASD keeps working.

View-relative keys are **keyboard only** — the gamepad mapping is unchanged.

**Gamepad** (e.g. Stadia) — translation is **entirely view-relative** (camera frame); rotation is about TCP, same as the keyboard:

| Input | Action |
|---|---|
| Left stick ←→ | left / right (camera) |
| Left stick ↑↓ | forward / back (camera) |
| Right stick ↑↓ | up / down (camera) |
| Right stick ←→ | yaw (TCP) |
| **R1** + right stick ↑↓ | pitch (TCP) |
| **R1** + right stick ←→ | roll (TCP) |
| A | home + start Servo |
| X | exit |

R1 is a hold-to-shift modifier: while it is down the right stick rotates instead of translating, so the two never mix. L2/R2 and LB are unmapped in this layout.

The default shift button index is **5** (R1/RB on most layouts and pads). Button numbering is not portable across controller models, or even across USB vs Bluetooth on the same pad — a Google Stadia controller over Bluetooth tested 5 as a dead slot, with R1 actually surfacing at button index 10 instead.

If the shift button does nothing on your pad, the node logs the complete `/joy` message on every button change:

```
/joy raw (button(s) [10] changed; shift configured as 5) — axes[6]: {...}  buttons[17]: {..., 10:1, ...}
```

Press the physical R1 alone, read which index actually flips in the `buttons[...]` list (not just whatever this doc or the default says), and pass it:

```bash
ros2 run arm_tasks gamepad_servo_node --ros-args -p gamepad_shift_button:=10
```

**Rotation naming is from the camera's point of view, not the TCP axis letters.** TCP `+X` is the camera's left-right axis → **pitch** (`wx`), TCP `+Y` its vertical axis → **yaw** (`wy`), TCP `+Z` its line of sight → **roll** (`wz`).

Unlike the keyboard, the gamepad has no mount-frame (absolute) translation — the operator is looking through the camera, so every stick axis follows it.

Gamepad mapping stays in `gamepad_servo_node`, not `teleop_twist_joy`: that package publishes all six twist axes in **one** frame, which would break camera-XYZ + TCP-ω.

### Architecture

```
keyboard / gamepad → TwistStamped (frame: arm_mount_link)
         → MoveIt Servo (inverse Jacobian, ~33 Hz)
         → Float64MultiArray positions
         → JointGroupPositionController
         → ArmCanSystem (MIT)
```

Home / Plan&Execute still use **JTC** (`FollowJointTrajectory`). Controllers are mutually exclusive; the teleop node switches them around **r** / **A** / exit.

Servo `publish_period` is **0.03 s (~33 Hz)** so each joint step stays above MIT stiction (~0.02 rad). Keyboard publishes twists at **50 Hz**; extra messages are overwritten. `controller_manager` / CAN stay at **100 Hz**.

Cartesian Servo is open-loop `J⁺ · twist`. Pure XYZ (`ω = 0`) still lets TCP attitude walk (Q/E especially — shoulder/elbow fold). The keyboard node adds a light **orientation hold** on unused rotation axes and does **not** slow XYZ. That is not industrial Cartesian pose-IK; drift can remain on a 6-DOF arm near the home wrist.

Servo collision checking is **off** (must be off at process start). Plan&Execute in `move_group` still checks collisions. Singularity deceleration thresholds are set very high (NHWA-style) so Jacobian condition number does not freeze +X from home.

Before **Plan & Execute** in RViz: stop teleop (exit the input node) so JTC owns the joints.

## Fake hardware (no CAN)

```bash
ros2 launch arm_moveit_config demo.launch.py use_fake_hardware:=true
ros2 run arm_tasks keyboard_servo_node
```

## GUI-only (no control)

```bash
ros2 launch arm_bringup arm_standalone.launch.py gui_only:=true
```

## Named poses

SRDF group states: `home`, `ready`, `experiment` (RViz Goal State).

Also `src/arm/arm_tasks/poses.json` — used by **r** / **A** and:

```bash
ros2 run arm_tasks teach_poses goto home
```

## Fake vs real hardware

| `use_fake_hardware` | Plugin | Behavior |
|---|---|---|
| `true` (default) | `mock_components/GenericSystem` | Instant joint echo — no CAN |
| `false` | `arm_hardware_interface/ArmCanSystem` | Real MIT motors on `can0` |
