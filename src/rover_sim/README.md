# ROVER SIM

This package contains all the assets and configurations required to run the Gazebo simulation.

## Git LFS

> ⚠️ **Important** Do this before working with simulation!

This package includes large mesh assets tracked with [Git LFS](https://git-lfs.github.com/). Install it once per machine before cloning/pulling, otherwise large mesh files will check out as tiny pointer stubs instead of the real content:

**Linux**
```bash
sudo apt update && sudo apt install git-lfs   
git lfs install
```

**Mac OS**
```bash
brew install git-lfs
```

If you already cloned the repo before installing Git LFS, pull the real files with:

```bash
git lfs pull
```

## Worlds & maps

| World | Map years | Resolutions |
|---|---|---|
| `mars_yard` | `2025`, `2026` | `low`, `medium`, `high` (2025 only; ignored for 2026) |
| `nav2_test_world` | - | - |

## `sim_gz.launch.py`

Launches the Gazebo world, spawns the rover, and starts its base stack: `ros_gz_bridge`, `robot_state_publisher`, EKF, `twist_mux`, and the base + swerve controllers.

| Argument | Default | Description |
|---|---|---|
| `swerve_controller` | `swerve_controller_test` | Which swerve controller comes up active (`swerve_controller`, `swerve_controller_test`). |
| `world_name` | `mars_yard` | World to load: `mars_yard`, `nav2_test_world`. |
| `map_year` | `2026` | `mars_yard` map year: `2025`, `2026`. |
| `map_resolution` | `high` | `mars_yard` (2025) mesh resolution: `low`, `medium`, `high`. Ignored for 2026/other worlds. |
| `model_name` | `indomitus_rover` | Name the robot model is spawned under. |
| `spawn_delay` | `5.0` | Seconds to wait before spawning the robot. |
| `extra_xacro_args` | *(empty)* | Extra flags passed to the URDF xacro compiler. |
| `spawn_x` / `spawn_y` / `spawn_z` | `0.0` / `0.0` / `0.5` | Initial spawn coordinates (m). |

**Example:**

```bash
ros2 launch rover_sim sim_gz.launch.py world_name:=mars_yard map_year:=2025 map_resolution:=high
```

## `sim_gz_nav2.launch.py`

Includes `sim_gz.launch.py` (with LiDAR/depth simulation xacro args enabled), then after a 10s delay starts `scan_filter`, SLAM, and Nav2.

All world/map/model/spawn arguments below are forwarded as-is to `sim_gz.launch.py` — leave them empty to use its defaults.

| Argument | Default | Description |
|---|---|---|
| `world_name` | *(empty → sim_gz default)* | World to load. |
| `map_year` | *(empty → sim_gz default)* | `mars_yard` map year. |
| `map_resolution` | *(empty → sim_gz default)* | `mars_yard` (2025) mesh resolution. |
| `model_name` | *(empty → sim_gz default)* | Robot model name. |
| `spawn_x` / `spawn_y` / `spawn_z` | *(empty → sim_gz default)* | Initial spawn coordinates. |
| `scan_filter_params_file` | *(empty → package default)* | Scan filter params override. |
| `nav2_params_file` | *(empty → package default)* | Nav2 params override. |
| `slam_params_file` | *(empty → package default)* | SLAM params override. |

**Example:**

```bash
ros2 launch rover_sim sim_gz_nav2.launch.py map_year:=2025
```

## `sim_gz_full.launch.py`

Launches a combined rover + arm robot in Gazebo (single `robot_description`, arm-aware bridge/controllers), and optionally spawns the switch panel task board.

| Argument | Default | Description |
|---|---|---|
| `world_name` | `world_demo` | World to load. |
| `model_name` | `indomitus_rover` | Name the robot model is spawned under. |
| `arm_camera` | `true` | Enable the arm wrist camera render/bridge. |
| `spawn_panel` | `true` | Also spawn the switch panel task board. |
| `panel_x` / `panel_y` / `panel_z` / `panel_yaw` | `2.0` / `0.0` / `0.5` / `3.14159` | Panel spawn pose. |

**Example:**

```bash
ros2 launch rover_sim sim_gz_full.launch.py spawn_panel:=false
```
