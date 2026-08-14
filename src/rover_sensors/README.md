# rover_sensors

`rover_sensors` is a ROS 2 package for running sensors used on the rover.

The package should contain:

* nodes source code
* sensor node source code
* launch files for running sensors separately
* launch files for running multiple sensors together
* simple documentation for every supported sensor

Currently supported sensors:

| Sensor     | Node           | Launch file            | Documentation |
| ---------- | -------------- | ---------------------- | ---------------------- |
| RPLIDAR S2 | `rplidar_node` | `rplidar_s2.launch.py` | [rplidar_s2.md](docs/rplidar_s2.md) |
| ZED2i Stereo Camera | `zed_camera` | `zed2i.launch.py` | [zed2i.md](docs/zed2i.md) |

Additional nodes:
| Node               | Launch file             | Description |
| ------------------ | ----------------------- | ----------- |
| `laser_filter_node` | `scan_filter.launch.py` | Node for filtering out rover parts seen by the LiDAR |

---

## Build

From the ROS 2 workspace root:

```bash
colcon build --packages-select rover_sensors
source install/setup.bash
```