# Arm Simulation

> **Note:** Arm packages are located in `src/arm/`. Before doing anything you need to build them. Run the following command to build only the arm subsystem:
> ```bash
> colcon build --symlink-install --packages-select-regex "^arm_"
> ```

## Starting the Arm in Simulation (Laptop)

There are two simulation modes available locally. Choose based on what you need:

| Mode | Launch file | Use case |
|---|---|---|
| Standalone visualization | `arm_bringup/arm_standalone.launch.py` | Quick URDF/mesh checks, manual joint testing via GUI, no planning needed |
| MoveIt planning simulation | `arm_moveit_config/demo.launch.py` | Testing trajectories, kinematics, motion planning, task development |

### Standalone Visualization

Starts RViz with the Joint State Publisher GUI but no motion planning stack. Useful for quickly
inspecting the URDF model, meshes, and TF tree without loading ros2_control or the full MoveIt overhead.

1. **Allow GUI access:** Run the following command on your **host machine** terminal (not inside Docker) before launching:
   ```bash
   xhost +local:docker
   ```
2. **Build and start the container:** Please refer to the [Docker](../../README.md#docker) section in the **README** to complete this step.
3. **Run the standalone launch file:** From the bash terminal inside your Docker container:
   ```bash
   ros2 launch arm_bringup arm_standalone.launch.py
   ```

By default this runs with `use_fake_hardware:=true`, which loads `mock_components/GenericSystem`
(see [Fake Hardware vs. Real Hardware](#fake-hardware-vs-real-hardware) below). To attempt loading
the real CAN hardware interface instead:
```bash
ros2 launch arm_bringup arm_standalone.launch.py use_fake_hardware:=false
```

### MoveIt Planning Simulation

Starts the full MoveIt 2 stack with motion planning, collision checking, and trajectory
execution using Fake Hardware. This is the primary mode for developing and testing arm
movements locally.

1. **Allow GUI access:** Run the following command on your **host machine** terminal (not inside Docker) before launching:
   ```bash
   xhost +local:docker
   ```
2. **Build and start the container:** Please refer to the [Docker](../../README.md#docker) section in the **README** to complete this step.
3. **Run the MoveIt demo launch file:** From the bash terminal inside your Docker container:
   ```bash
   ros2 launch arm_moveit_config demo.launch.py
   ```

In RViz, use the **MotionPlanning** panel to set a goal pose for the end-effector and click
**Plan & Execute** to run a full plan-and-execute cycle.

#### Fake Hardware vs. Real Hardware

The `arm_macro.xacro` model exposes a `use_fake_hardware` xacro argument that controls which
`ros2_control` hardware plugin gets loaded:

| Value | Plugin | Behavior |
|---|---|---|
| `true` (default) | `mock_components/GenericSystem` | Joint commands are written directly into the joint state and read back immediately — no physics, no motor, no delay. Useful for testing planning logic, SRDF groups, and the MoveIt API without any physical or simulated dynamics. |
| `false` | `arm_hardware_interface/ArmCanSystem` | Sends commands over the real CAN bus to the physical actuators. Requires the Jetson and a working `arm_hardware_interface` build. |

Because `mock_components/GenericSystem` reports back whatever position it was just told to move
to, it does **not** validate motor dynamics, CAN latency, encoder noise, or mechanical limits like
backlash or sag — only the kinematic/geometric correctness of a trajectory is verified.
