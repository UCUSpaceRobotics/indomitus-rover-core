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
| GUI-only RViz | `arm_bringup/arm_standalone.launch.py gui_only:=true` | Inspect URDF / move joint sliders — **no** motors, **no** ros2_control |
| MoveIt + teleop | `arm_moveit_config/demo.launch.py` | Plan&Execute **and** joystick Servo (shared stack) |

On the host (for RViz):

```bash
xhost +local:docker
```

## Hardware joystick teleop

### 1. CAN

```bash
# host
./scripts/setup_host.sh local --can   # once
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
```

### 2. Stack (container)

```bash
ros2 launch arm_moveit_config demo.launch.py use_fake_hardware:=false
```

This starts:

- `ros2_control` + **JTC** (`indomitus_arm_controller`) for home / Plan&Execute
- **forward position** controller (inactive until teleop) for Servo streaming
- `move_group` + RViz
- `servo_node` (Cartesian twist → joint positions as `Float64MultiArray`)

### 3. Joystick (two more terminals)

```bash
source /opt/ws/install/setup.bash
ros2 run joy joy_node
```

```bash
source /opt/ws/install/setup.bash
ros2 run arm_tasks gamepad_servo_node
```

### 4. Controls

1. Press **A** — move to named **`home`**, then start Servo (switches to streaming controller).
2. Sticks / triggers — EEF XYZ in mount frame; roll/pitch/yaw about TCP (rotated into mount for Servo).
3. **X** — exit gamepad node (Servo stop → JTC active again).

| Input | Action |
|---|---|
| Right stick | EEF X / Y (mount) |
| Left stick | EEF roll / pitch (TCP → mount) |
| L2 / R2 | EEF yaw (TCP → mount) |
| LB / RB | EEF +Z / −Z (mount) |
| A | home + start Servo |
| X | exit |

### Architecture (why streaming)

```
joy → gamepad_servo_node → TwistStamped
         → MoveIt Servo (diff IK)
         → Float64MultiArray positions
         → JointGroupPositionController
         → ArmCanSystem (MIT)
```

Home / Plan&Execute still use **JTC** (`FollowJointTrajectory`). Controllers are mutually exclusive; `gamepad_servo_node` switches them around **A** / stop.

Servo publishes joint positions at **~100 Hz** into the forward controller. The hardware interface adds MIT **velocity feedforward** from the position step when JTC is not active, so motors do not “arrive and stop” between Servo ticks.

Before **Plan & Execute** in RViz: stop teleop (exit gamepad or ensure Servo stopped) so JTC owns the joints.

## Fake hardware (no CAN)

```bash
ros2 launch arm_moveit_config demo.launch.py use_fake_hardware:=true
# then joy + gamepad_servo_node as above
```

## GUI-only (no control)

```bash
ros2 launch arm_bringup arm_standalone.launch.py gui_only:=true
```

## Named poses

SRDF group states: `home`, `ready`, `experiment` (RViz Goal State).

Also `src/arm/arm_tasks/poses.json` — used by **A** and:

```bash
ros2 run arm_tasks teach_poses goto home
```

## Fake vs real hardware

| `use_fake_hardware` | Plugin | Behavior |
|---|---|---|
| `true` (default) | `mock_components/GenericSystem` | Instant joint echo — no CAN |
| `false` | `arm_hardware_interface/ArmCanSystem` | Real MIT motors on `can0` |
