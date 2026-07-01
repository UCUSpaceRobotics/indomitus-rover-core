# rover_sensors

`rover_sensors` is a ROS 2 package for running sensors used on the rover.

The package should contain:

* sensor node source code
* launch files for running sensors separately
* launch files for running multiple sensors together
* simple documentation for every supported sensor

Currently supported sensors:

| Sensor     | Node           | Launch file            | Status      |
| ---------- | -------------- | ---------------------- | ----------- |
| RPLIDAR S2 | `rplidar_node` | `rplidar_s2.launch.py` | Implemented |

---

## Build

From the ROS 2 workspace root:

```bash
colcon build --packages-select rover_sensors
source install/setup.bash
```