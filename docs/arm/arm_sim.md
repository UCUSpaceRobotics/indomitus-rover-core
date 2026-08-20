# Arm Simulation / visualization

For **hardware + joystick teleop** (streaming Servo), see **[arm_teleop.md](arm_teleop.md)**.

> **Note:** Arm packages are in `src/arm/`. Build:
> ```bash
> colcon build --symlink-install --packages-select-regex "^arm_"
> source install/setup.bash
> ```

## Starting the Arm in Simulation (Laptop)

| Mode | Launch file | Use case |
|---|---|---|
| Standalone visualization | `arm_bringup/arm_standalone.launch.py` | Quick URDF/mesh checks, manual joint testing via GUI |
| MoveIt stack | `arm_moveit_config/demo.launch.py` | Planning, Execute, Servo node |
| Cartesian teleop | `ros2 run arm_tasks keyboard_servo_node` | After demo; see [arm_teleop.md](arm_teleop.md) |

### Standalone Visualization

1. On the **host**: `xhost +local:docker`
2. In the container:
   ```bash
   ros2 launch arm_bringup arm_standalone.launch.py gui_only:=true
   ```

Sliders only — no ros2_control / CAN. For control without planning:
```bash
ros2 launch arm_bringup arm_standalone.launch.py use_fake_hardware:=true
```

### MoveIt demo

```bash
ros2 launch arm_moveit_config demo.launch.py          # fake hardware
ros2 run arm_tasks keyboard_servo_node
```

Servo teleop streams positions to `indomitus_arm_forward_position_controller`
at ~33 Hz (`publish_period: 0.03`). Press **r** (keyboard) or **A** (gamepad)
to go **home** and start Servo (controller switch is in `arm_tasks`).

If teleop still feels like each joint “steps then stops”, rebuild
`arm_hardware_interface` (MIT velocity feedforward from position Δ) and
restart `demo.launch`.

#### Fake Hardware vs. Real Hardware

| Value | Plugin | Behavior |
|---|---|---|
| `true` (default) | `mock_components/GenericSystem` | Instant echo — no physics/CAN |
| `false` | `arm_hardware_interface/ArmCanSystem` | Real actuators on CAN |
