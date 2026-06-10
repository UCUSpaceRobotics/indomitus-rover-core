# Rover Quickstart

## 1. Host — CAN interface setup

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

## 2. Build and start Docker

```bash
cd ~/Desktop/indomitus-rover-core

# Build image and start container
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

## 3. Bring up the rover

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

## 4. Run test_pipeline

Open a second shell inside the container:

```bash
docker exec -it rover_dev bash
```

Run the test tool:

```bash
python3 /work/test_pipeline.py
```

### Controls

| Key | Action |
| --- | --- |
| `e` | Enable all motors (send init frames) |
| `d` | Disable all motors (zero → wait 1.5s → disable) |
| `1` | Straight forward 0.5 m/s |
| `2` | Spin in place left |
| `3` | Turn left (Ackermann) |
| `4` | All wheels max steer angle, no drive |
| `s` | Stop drive (send zero cmd_vel) |
| `f` | Print latest motor feedback |
| `q` | Quit |

> Note: motors are enabled automatically by `chassis_driver_node` 3 seconds after launch.
> Press `e` only if you need to re-enable after a fault.

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