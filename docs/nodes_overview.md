# Packages

## 1. `indomitus_rover_chassis_driver` (C++)

This package implements the communication with motors. It communicates with two types of motor controllers:
- **Steer Motors (Steadywin V3.06b0):** 4 motors (IDs: 11, 13, 15, 17) for steering the wheels
- **Drive Motors (Damiao MIT):** 4 motors (IDs: 10, 12, 14, 16) for rotating the wheels

**Node:** 
- `chassis_driver_node`
    - Subscribes to `/wheel_targets` (motor commands from kinematics controller) and sends commands for motors to `/to_can_bus`
    - TODO: Publishes motor feedback via CAN frames

**Launch files:**
- `chassis_driver.launch.py` - launches the `chassis_driver` node (package: `indomitus_rover_chassis_driver`)

---

## 2. `indomitus_interfaces` (Custom Messages)

**Messages:**

- **`WheelTargets`**: Commands for wheel steering and drive speeds
  - `fl_speed`, `fr_speed`, `rl_speed`, `rr_speed` (rad/s) - drive motor speeds
  - `fl_angle`, `fr_angle`, `rl_angle`, `rr_angle` (rad) - steer motor positions

- **`ChassisStatus`**: Complete status report from all motors

- **`MotorStatus`**: Individual motor state (kinematic and electrical data)

---

## 3. `indomitus_rover_bringup` (Launch & Configuration)

**Package Type:** Launch/configuration package

**Launch Files:**
- `launch/can.launch.py` - Configures and launches CAN bus interface


---

## 4. `indomitus_rover_viz` (RViz Visualization)

**Package Type:** RViz visualization package

**Launch Files:**
- `launch/rviz.launch.py` - rover visualization

**Visualization Config:**
- `rviz/robot.rviz`


---

## 5. `indomitus_rover_control` (Python)

**Nodes:** 
- `rover_controller`
    - Implements kinematics calculations for 4-wheel steering rover
    - Subscribes to `/cmd_vel` (geometry_msgs/Twist) - linear and angular velocity commands
    - Publishes `/wheel_targets` (indomitus_interfaces/WheelTargets) - motor commands


**Launch files:**
- rover_kinematics.launch.py - launch rover_controller node

---

## 6. `indomitus_rover_description` (URDF & Configuration)

Containes all urdf files describing rover, 3D rover models, geometry config files.

---

## 7. `indomitus_rover_sim` (Gazebo Simulation)

Simulation of a rover on a mars yard.

**Launch Files:**
- `launch/sim_gz_urdf.launch.py`

**Simulation Nodes:**
- `sim_chassis_driver_node` - Simulated motor driver (converts commands to joint velocities)
- `sim_diff_bar_node` - Simulated differential bar suspension
