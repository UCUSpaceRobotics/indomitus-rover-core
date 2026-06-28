# Nav2 -- Indomitus Rover

Autonomous navigation for the Indomitus rover in Gazebo simulation using Nav2.

---

## Prerequisites

The simulation must be running before launching navigation.
Follow the Docker and simulation setup in `QUICKSTART.md` first.

---

## Dependencies

```bash
sudo apt install -y \
  ros-${ROS_DISTRO}-nav2-amcl \
  ros-${ROS_DISTRO}-nav2-map-server \
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

Open three terminals inside the container.

**Terminal 1 -- Simulation**
```bash
ros2 launch rover_sim sim_gz_nav2.launch.py
```

Wait until you see `Configured and activated swerve_controller` before proceeding.

**Terminal 2 -- Navigation stack**
```bash
ros2 launch rover_navigation navigation.launch.py
```

**Terminal 3 -- Visualisation**
```bash
ros2 launch rover_viz rviz.launch.py use_sim:=true
```

---

## Sending a Goal

1. In RViz set **Global Options -- Fixed Frame** to `map`
2. Wait for the map to appear and the particle cloud to stabilise
3. Click **2D Nav Goal** in the toolbar and click a destination on the map
