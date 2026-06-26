# Graph

> *not full, needs improvements*

![Graph of nodes](../assets/ros2_graph.png)

# Packages

## 1. `indomitus_interfaces`
Custom ROS2 messages, services, and actions used across all packages.

**Messages:**
- **`WheelTargets`** — commands for all 8 actuators (4 steer + 4 drive)
  - `fl_speed`, `fr_speed`, `rl_speed`, `rr_speed` (rad/s) — drive motor speeds
  - `fl_angle`, `fr_angle`, `rl_angle`, `rr_angle` (rad) — steer motor positions
- **`ChassisStatus`** — complete status report from all motors
- **`MotorStatus`** — individual motor state (kinematic and electrical data)

**Services:**
- `GetWeight`, `SetSteerZero`, `SetTrafficLight`

**Actions:**
- `ContainerLid`

---

## 2. `rover_hardware_interface` (C++)
ros2_control `SystemInterface` plugin responsible for low-level communication with motors over CAN bus.

Communicates with two motor types:
- **Steer Motors (Steadywin V3.06b0):** 4 motors (IDs: 11, 13, 15, 17)
- **Drive Motors (Damiao MIT):** 4 motors (IDs: 10, 12, 14, 16)

**Protocols:** `damiao_protocol.hpp`, `steadywin_protocol.hpp`

Loaded as a ros2_control plugin via `ros2_control_real.xacro` — no standalone node or launch file.

---

## 3. `rover_controller` (C++)
ros2_control controller plugins implementing swerve-drive kinematics and odometry.

- **`swerve_controller`** — receives `geometry_msgs/Twist` on `/cmd_vel` and computes per-wheel velocity and steering angle targets
- **`odometry_controller`** — reads wheel feedback and publishes odometry
- **`ackermann_controller`** *(planned)* — Ackermann steering with fixed rear wheels, only front wheels steer
- **`dual_ackermann_controller`** *(planned)* — Ackermann steering with symmetric front and rear wheel steering

Loaded as plugins via `controllers.yaml`, not launched directly.

---

## 4. `rover_bringup`
Main launch and configuration package.

**Launch files:**
- `rover.launch.py` — top-level bringup (loads description, ros2_control, controllers)
- `can.launch.py` — configures and brings up the CAN bus interface
- `joy.launch.py` — launches joystick input stack
- `container.launch.py` — launches container peripheral node

**Config:**
- `controllers.yaml` — ros2_control controller parameters
- `twist_mux.yaml` — velocity command multiplexer config
- `joy.yaml` — joystick driver config

**URDF:**
- `rover_real.urdf.xacro` — top-level xacro for the real robot (includes hardware interface)

---

## 5. `rover_description`
URDF/xacro robot description files and 3D meshes.

**URDF:**
- `rover.urdf.xacro` — base robot description
- `ros2_control_real.xacro` — hardware interface definition for the real robot
- `ros2_control_sim.xacro` — hardware interface definition for simulation
- `camera.xacro` — camera link/sensor definition

**Meshes:** suspension components (`rocker`, `wheel`, `wheel_mount`, `central_axii`), plus arm, navigation, and science subfolders.

---

## 6. `rover_teleop` (Python)
Teleoperation nodes.

- **`joystick_interpreter_node`** — maps raw joystick input (`sensor_msgs/Joy`) to `Twist` commands on `/cmd_vel`

---

## 7. `rover_peripherals` (Python)
Nodes for non-drivetrain devices mounted to the rover body.

- **`rover_container_node`** — controls the sample container mechanism (communicates over CAN, config: `container_can.yaml`)
- **`rover_lighting_node`** — controls rover lighting

---

## 8. `rover_sim` (C++)
Gazebo simulation support.

**Launch files:**
- `launch/sim_gz.launch.py`

**World:**
- `worlds/world_demo.sdf`

**Nodes:**
- **`sim_diff_bar_node`** — simulates passive differential bar suspension behaviour (the simulated hardware interface is handled by `ros2_control_sim.xacro` instead of a dedicated driver node)

---

## 9. `rover_viz`
RViz visualization config and launch.

**Launch files:**
- `launch/rviz.launch.py`

**Config:**
- `rviz/robot.rviz`