# Rover Quickstart

## 1. ROS2 Packages Overview

- `indomitus_interfaces` - package with all custom messages, services, actions
- `rover_bringup` - package with main launch files and configs
- `rover_description` - packages with meshes and everythin related to rover geometry, form, so on
- `rover_chassis_driver` - package with nodes resposible for communication with motors via CAN bus
    - `chassis_driver_node` - transforms WheelTargets msg into CAN bus frames for motors. Also it collects data from each motor about voltage, current, tempreture, so on.
- `rover_control` - package with kinematics_node and everything that is related to movement control
    - `rover_kinematics_node`
    - `joystick_interpreter_node`
    - `rover_odometry_node` (TODO)
- `rover_peripherals` - package with nodes communicating with devices mounted to rover body
    - `rover_container_node`
    - `rover_lighting_node`
- `rover_sim` - package with all stuff that is related to simulation
    - `sim_chassis_driver_node` - takes data from /wheel_targets topic and moves wheels via ros2_control
    - `sim_diff_bar_node` - nodes that simulates differential bar work
- `rover_viz` - package with all stuff that is related to visualizations and rviz

---

## 2. Build and start Docker

Navigate to the root of the repository **indomitus-rover-core**

Build image and start container:

```bash
docker compose up --build -d
```

Once the container is running, build the ROS2 workspace:

```bash
docker exec -it rover_dev bash
colcon build --symlink-install
source install/setup.bash
```

If you need to rebuild after code changes:

```bash
docker exec -it rover_dev bash
colcon build --symlink-install
source install/setup.bash
exit
```

---

## 3. Host — CAN interface setup

Run once before starting Docker (requires physical CAN adapter connected):

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
```

Verify it's up:

```bash
ip link show can0
# should say: UP LOWER_UP
```

---

## 4. Bring up the rover

Open a shell inside the container:

```bash
docker exec -it rover_dev bash
```

Launch everything (CAN bridge + kinematics + chassis driver):

```bash
ros2 launch rover_bringup rover.launch.py
```

What starts:

* `socket_can_sender` + `socket_can_receiver` — CAN ↔ ROS2 bridge
* `rover_kinematics_node` — Ackermann geometry (starts 1.5s after CAN bridge)
* `chassis_driver_node` — motor driver (starts 1.5s after CAN bridge, enables motors after 3s)

Leave this terminal open. Open a second terminal for testing.

### Reloading a single node without restarting everything

Useful when you changed `chassis_driver` or `rover_kinematics_node` and don't want to drop the CAN bridge.

Open a second terminal inside the container:

```bash
docker exec -it rover_dev bash
```

Kill only the node you want to restart (by name):

```bash
ros2 node kill /chassis_driver
# or
ros2 node kill /rover_controller
```

```bash
colcon build --symlink-install --packages-select rover_chassis_driver
source install/setup.bash
```

Then start it manually:

```bash
# rover_chassis_driver
ros2 run rover_chassis_driver chassis_driver_node \
  --ros-args --params-file /opt/ws/install/rover_chassis_driver/share/rover_chassis_driver/config/chassis_driver.yaml

# rover_kinematics_node
ros2 run rover_control rover_kinematics_node \
  --ros-args --params-file /opt/ws/install/rover_description/share/rover_description/config/rover_geometry.yaml
```

> `socket_can_sender` and `socket_can_receiver` keep running — motors stay powered and CAN stays up.

---

## 5. Monitor topics (optional, separate terminal)

```bash
docker exec -it rover_dev bash

# Wheel targets from kinematics
ros2 topic echo /wheel_targets

# Motor health (1 Hz)
ros2 topic echo /diagnostics

# Full motor state (10 Hz)
ros2 topic echo /chassis/motor_states

# Joint positions and velocities
ros2 topic echo /joint_states

# Raw CAN frames going out
ros2 topic echo /to_can_bus

# Raw CAN frames coming in
ros2 topic echo /from_can_bus
```

---

## 6. Stop everything

```bash
# In the launch terminal: Ctrl+C
# Motors will be disabled gracefully (zero → 1.5s → disable frames)

# Stop the container
docker compose down
```