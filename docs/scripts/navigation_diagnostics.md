# scripts/navigation/

Ad hoc scripts used to measure real localization error (position + yaw)
against ground truth, developed while investigating a `slam_toolbox`
translational-correction accuracy issue on Mars Yard terrain. Not part of
any ROS package — run directly with `python3`.

Both scripts take `--target-frame`/`--source-frame` (default `map` /
`base_footprint`) so the same tool can measure the SLAM-corrected estimate
(`map`) or raw dead-reckoning (`odom`) — see each script's `--help`.

## `compare_pose.py` — sim only

Takes one synchronized snapshot of RViz's estimated pose (`tf2_echo`) and
Gazebo's ground truth (`ign topic`), and prints the position/yaw error
between them. Requires a running Gazebo sim.

### Usage

```bash
python3 scripts/navigation/compare_pose.py
```

### Options

| Flag | Description |
|---|---|
| `--target-frame` | `tf2_echo` target frame, e.g. `map` or `odom` (default `map`) |
| `--source-frame` | `tf2_echo` source frame (default `base_footprint`) |
| `--world` | Gazebo world name, e.g. `mars_yard` or `nav2_test_world` (default `mars_yard`) |
| `--model` | Gazebo model name to match in `/world/<world>/pose/info` (default `indomitus_rover`) |
| `--tf-duration` | Seconds to let `tf2_echo` stream before reading the last sample (default `3.0`) |
| `--gz-duration` | Max seconds to wait for the Gazebo pose snapshot (default `5.0`) |

### Examples

```bash
python3 scripts/navigation/compare_pose.py --target-frame odom   # raw EKF, no SLAM
python3 scripts/navigation/compare_pose.py --world nav2_test_world
```

## `track_pose_drift.py` — sim or real hardware

No ground truth needed. Saves the pose as soon as the transform first
appears, then reports drift from that baseline every `--interval` seconds.
For field tests: drive out and back to the same physical spot, and the
drift reading at that moment is the real localization error.

### Usage

```bash
python3 scripts/navigation/track_pose_drift.py
```

### Options

| Flag | Description |
|---|---|
| `--target-frame` | tf frame treated as the fixed/reference frame, e.g. `map` or `odom` (default `map`) |
| `--source-frame` | tf frame whose pose is tracked (default `base_footprint`) |
| `--interval` | Seconds between drift reports (default `10.0`) |

### Examples

```bash
python3 scripts/navigation/track_pose_drift.py --target-frame odom --interval 5
```

## Requirements

Neither script has real `pip install`-able dependencies:

- `compare_pose.py` uses only the Python standard library, plus the `ros2`
  and `ign`/`gz` CLIs on `PATH`.
- `track_pose_drift.py` imports `rclpy`/`tf2_ros`, which come from a
  sourced ROS 2 install (`source /opt/ros/<distro>/setup.bash`), not PyPI.

Both just need to run inside this repo's dev container (or any shell with
ROS 2 + Gazebo sourced) — no extra setup.
