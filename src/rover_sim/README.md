This package contains all the assets and configurations required to run the Gazebo simulation.

## `sim_gz.launch.py`

This script launches the Gazebo simulation environment.

You can dynamically set the 3D map quality using the `map_resolution` launch argument. The following options are supported: `low`, `medium` and `high`.

**Example Usage:**

```bash
ros2 launch rover_sim sim_gz.launch.py map_resolution:=high
```