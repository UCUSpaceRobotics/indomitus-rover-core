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

**Keyboard** — translation in `arm_mount_link`, rotation about `arm_tcp_link` (ω is rotated into mount before publish):

| Key | Action |
|---|---|
| W / S | EEF +X / −X (mount) |
| A / D | EEF +Y / −Y (mount) |
| Q / E | EEF +Z / −Z (mount) |
| I / K | roll (TCP) |
| U / O | pitch (TCP) |
| J / L | yaw (TCP) |
| r | home + start Servo |
| Esc / x | exit |

**Gamepad** (e.g. Stadia):

| Input | Action |
|---|---|
| Right stick | EEF X / Y (mount) |
| Left stick | EEF roll / pitch (TCP → mount) |
| L2 / R2 | EEF yaw (TCP → mount) |
| LB / RB | EEF +Z / −Z (mount) |
| A | home + start Servo |
| X | exit |

Gamepad mapping stays in `gamepad_servo_node`, not `teleop_twist_joy`: that package publishes all six twist axes in **one** frame, which would break mount-XYZ + TCP-ω.

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
