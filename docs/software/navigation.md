# Navigation Stack

How to launch autonomous navigation (LiDAR + SLAM + Nav2) — on the physical rover, and separately in Gazebo simulation.

## 1. What needs to be connected

* **RPLIDAR S2** — connected via USB, recognized at `/dev/rplidar-s2` (udev rule, see [`rplidar_s2.md`](../../src/rover_sensors/docs/rplidar_s2.md)).
* **ZED2i stereo camera** — connected via USB, host udev rule (`99-slabs.rules`) and GPU passthrough set up (see [`zed2i.md`](../../src/rover_sensors/docs/zed2i.md)). Must already be running in `nav` mode before `navigation.launch.py`'s waits will succeed — it does not start the camera itself.

## 2. Launch commands

Start the camera (and rest of the rover) first:

```bash
ros2 launch rover_bringup rover.launch.py zed2i_mode:=nav
```

Then start navigation:

```bash
ros2 launch rover_bringup navigation.launch.py
```

`navigation.launch.py` retries until real sensor data shows up, so it's fine to start it before the camera is fully up — it will just wait.

## 3. Simulation (Gazebo)

Separate from the above — no hardware needed. Spawns the rover in Gazebo with simulated LiDAR + depth camera, then starts SLAM/Nav2 against that simulated data:

```bash
ros2 launch rover_sim sim_gz_nav2.launch.py world_name:=mars_yard
```

Other `world_name` option: `nav2_test_world`. Also accepts `map_year`, `map_resolution`, `model_name`, `spawn_x`/`spawn_y`/`spawn_z`, and the same `scan_filter_params_file`/`nav2_params_file`/`slam_params_file` overrides as real navigation. Unlike `navigation.launch.py`, it doesn't wait on real sensor topics — it starts the scan filter, SLAM, and Nav2 (all with `use_sim_time:=true`) after a fixed 10s delay for the simulated robot to spawn.

## 4. What gets launched

**`rover.launch.py`** — CAN bus, `robot_state_publisher` (TF tree), drivetrain control, `twist_mux`, LoRa fallback, fault logger, lighting, power monitor, EKF (`/wheels/odom` + `/zed2i/odom` → `/odom`), and — because of `zed2i_mode:=nav` — the ZED2i camera in nav mode.

**`navigation.launch.py`**, in order:

1. **RPLIDAR driver** (`sllidar_node`, auto-respawns on disconnect) + **scan filter chain** → publishes `/rplidar/scan_filtered`.
2. Waits until `/rplidar/scan_filtered` is actually publishing.
3. Waits until `/zed2i/points` and `/zed2i/odom` are actually publishing.
4. Once both are confirmed live, starts:
   * **SLAM** (`slam_toolbox`, async) — publishes `map -> odom` TF.
   * **Nav2** (`planner_server`, `controller_server`, `bt_navigator`, `behavior_server`, `waypoint_follower`, `lifecycle_manager`) — publishes velocity commands to `cmd_vel_nav`, routed into `/cmd_vel` by `twist_mux` (priority 50 — joystick teleop at priority 100 can always override it).

If either sensor wait is killed or fails, SLAM/Nav2 are never started.

## 5. Configs used

| Component | Config file |
| --- | --- |
| RPLIDAR driver | `rover_sensors/config/rplidar_s2.yaml` |
| Scan filter | `rover_sensors/config/scan_filter.yaml` |
| ZED2i camera (`nav` mode) | `rover_sensors/config/zed2i/zed2i_common.yaml` + `zed2i_nav.yaml` (merged at launch) |
| SLAM | `rover_localization/config/slam_toolbox_params.yaml` |
| Nav2 | `rover_navigation/config/nav2_params.yaml` |
| EKF (`/odom`) | `rover_localization/config/ekf.yaml` |
| `cmd_vel` routing | `rover_bringup/config/twist_mux.yaml` |

Each can be overridden via launch arguments: `rplidar_params_file`, `scan_filter_params_file`, `nav2_params_file`, `slam_params_file` on `navigation.launch.py`; `config_path` on `zed2i.launch.py` (via `rover.launch.py`).
