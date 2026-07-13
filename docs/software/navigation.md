# Nav2 -- Indomitus Rover

Autonomous navigation for the Indomitus rover in Gazebo simulation using Nav2.

Localization is done with live SLAM. There is no static map anymore.
The map is built while the rover drives.

---

## Prerequisites

The simulation must be running before launching navigation.
Follow the Docker and simulation setup in `QUICKSTART.md` first.

---

## Dependencies

```bash
sudo apt install -y \
  ros-${ROS_DISTRO}-slam-toolbox \
  ros-${ROS_DISTRO}-nav2-smac-planner \
  ros-${ROS_DISTRO}-nav2-planner \
  ros-${ROS_DISTRO}-nav2-controller \
  ros-${ROS_DISTRO}-nav2-bt-navigator \
  ros-${ROS_DISTRO}-nav2-behaviors \
  ros-${ROS_DISTRO}-nav2-waypoint-follower \
  ros-${ROS_DISTRO}-nav2-lifecycle-manager \
  ros-${ROS_DISTRO}-nav2-mppi-controller
```

---

## Build

```bash
cd /opt/ws
colcon build --packages-select rover_navigation
source install/setup.bash
```

---

## Running

Open four terminals inside the container.

**Terminal 1 -- Simulation**
```bash
ros2 launch rover_sim sim_gz_nav2.launch.py
```

Wait until you see `Configured and activated swerve_controller` before proceeding.

**Terminal 2 -- SLAM**
```bash
ros2 launch rover_navigation slam.launch.py use_sim_time:=true
```

This builds the map live from lidar data.
It also publishes the `map -> odom` transform.

Wait a few seconds for it to start before going to the next step.

**Terminal 3 -- Navigation stack**
```bash
ros2 launch rover_navigation navigation.launch.py
```

**Terminal 4 -- Visualisation**
```bash
ros2 launch rover_viz rviz.launch.py use_sim:=true use_nav:=true
```

---

## Sending a Goal

1. In RViz set **Global Options -- Fixed Frame** to `map`
2. Wait for the map to appear
3. Click **2D Goal Pose** in the toolbar and click a destination on the map

> **Tip:** Areas the rover has not seen yet look empty on the map.
> The rover can still plan through them, but it may need to replan
> once it sees a real obstacle there.

---

## Sending Multiple Goals (Navigate Through Poses)

1. In the **Navigation 2** panel, click **Waypoint / Nav Through Poses Mode**
2. Click **Nav2 Goal** and place each waypoint on the map
3. Click **Start Nav Through Poses** to run the full sequence