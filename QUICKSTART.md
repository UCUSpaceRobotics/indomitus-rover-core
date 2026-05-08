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

The container auto-builds the ROS2 workspace on first start (skips Gazebo sim package).
Watch build progress:

```bash
docker logs -f indomitus_rover_dev
# Wait for: "ROS humble ready. Workspace: /opt/ws"
```

If you need to rebuild manually (e.g. after code changes):

```bash
docker exec -it indomitus_rover_dev bash
colcon build --symlink-install --packages-skip indomitus_rover_sim
source install/setup.bash
exit
```

---

## 3. Bring up the rover

Open a shell inside the container:

```bash
docker exec -it indomitus_rover_dev bash
```

Launch everything (CAN bridge + kinematics + chassis driver):

```bash
ros2 launch indomitus_rover_bringup rover.launch.py
```

What starts:
- `socket_can_sender` + `socket_can_receiver` — CAN ↔ ROS2 bridge
- `rover_kinematics_node` — Ackermann geometry (starts 1.5s after CAN bridge)
- `chassis_driver_node` — motor driver (starts 1.5s after CAN bridge, enables motors after 3s)

Leave this terminal open. Open a second terminal for testing.

---

## 4. Run test_pipeline

Open a second shell inside the container:

```bash
docker exec -it indomitus_rover_dev bash
```

Run the test tool:

```bash
python3 /work/test_pipeline.py
```

### Controls

| Key | Action |
|-----|--------|
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
docker exec -it indomitus_rover_dev bash

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
