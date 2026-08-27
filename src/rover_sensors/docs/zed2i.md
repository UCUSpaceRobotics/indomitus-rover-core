# ZED2i Camera Configuration & Launch Guide

This document outlines the configuration, launch instructions, and data capabilities of the ZED2i stereo camera used on the rover.

**Official Documentation Resources:**

* [ZED2I Datasheet](https://cdn.sanity.io/files/s18ewfw4/staging/f6eb2a45caad4faa16149f747b765864a337ae91.pdf/ZED-2i-Datasheet.pdf)
* [ZED ROS 2 Integration Docs](https://docs.stereolabs.com/docs/integrations/ros-2)
* [ZED ROS 2 Wrapper Repository](https://github.com/stereolabs/zed-ros2-wrapper)


---


## 1. Launching the Camera

The camera is initialized using a custom launch file that wraps the official `zed_camera.launch.py` and applies specific topic remappings to fit the rover's architecture.

The launch file takes a `mode` argument that picks which config to load. Every mode always loads `rover_sensors/config/zed2i/zed2i_common.yaml` first (the `general`/`video`/`sensors` settings shared by both modes), then merges the mode-specific file on top at launch time, so the shared settings aren't duplicated between mode files:

* `mode:=rgb` *(default)* — only the rectified color feed is enabled, for the operator view. Merges in `rover_sensors/config/zed2i/zed2i_rgb.yaml`.
* `mode:=nav` — additionally enables the point cloud and positional tracking (VIO) used by the navigation stack. Merges in `rover_sensors/config/zed2i/zed2i_nav.yaml`.

```bash
# RGB feed only (operator view)
ros2 launch rover_sensors zed2i.launch.py mode:=rgb

# Point cloud + VIO for navigation
ros2 launch rover_sensors zed2i.launch.py mode:=nav
```

On the real rover, this launch file is normally started via `rover_bringup/launch/rover.launch.py`'s `zed2i_mode` argument rather than invoked directly — see [`nodes_overview.md`](../../../docs/software/nodes_overview.md). `config_path` can still be set explicitly to override the default file chosen by `mode` - in that case the file is used as-is and `zed2i_common.yaml` is not merged in.


---


## 2. TF Publishing

**Transform (TF) publishing from the ZED node must remain disabled.**
The rover's `robot_state_publisher`, `ekf_node`, and `slam_toolbox` are strictly responsible for the TF tree. To prevent conflicts, the following launch arguments and YAML parameters default to `false`:

* `publish_tf`: Disabled in both launch arguments and YAML (`pos_tracking.publish_tf`).
* `publish_map_tf`: Disabled in both launch arguments and YAML (`pos_tracking.publish_map_tf`).
* `publish_urdf_tf`: Disabled in launch arguments.

**Exception: `publish_imu_tf` is enabled.** `rover_description`'s URDF never defines
a `zed2i_imu_link` (the vendored `zed_description` package doesn't include an IMU
frame at all), so `robot_state_publisher` has no way to provide
`zed2i_left_camera_frame -> zed2i_imu_link`. The ZED node is the only source for
this transform (it uses the camera's factory-calibrated extrinsic), and since it
doesn't overlap any frame `robot_state_publisher` owns, enabling it here doesn't
violate the "ZED node owns no TF" rule above. It's forwarded as the `publish_imu_tf`
launch argument (default `true`) — the upstream `zed_camera.launch.py` otherwise
defaults it to `false` and silently overrides the YAML value.


---


## 3. Published Data

The camera captures data at an internal resolution of `HD720` at 30 FPS. The active data streams are remapped in the launch file to match the rover's namespace.

Depth, point cloud, and positional tracking (VIO) are only enabled in `mode:=nav` (`zed2i_nav.yaml`); `mode:=rgb` (`zed2i_rgb.yaml`) disables depth extraction entirely (`depth.depth_mode: 'NONE'`) to save compute, since those consumers don't need it. Where depth is enabled, it uses the `NEURAL_LIGHT` mode, and visual odometry operates in `two_d_mode: true`, forcing navigation logic onto a flat plane (Z is fixed to 0.0, roll and pitch are zeroed).

| Data Type | Published Topic | Enabling YAML Parameter | Available in |
| --- | --- | --- | --- |
| **RGB Image (Rectified)** | `/zed2i/rgb/image_rect_color`<br> | `video.publish_rgb: true`<br> | `rgb`, `nav` |
| **RGB Camera Info** | `/zed2i/rgb/camera_info`<br> | `video.publish_rgb: true`<br> | `rgb`, `nav` |
| **IMU Data** | `/zed2i/imu/data`<br> | `sensors.publish_imu: true`<br> | `rgb`, `nav` |
| **Node Status** | *(Internal status topics)* | `general.publish_status: true`<br> | `rgb`, `nav` |
| **Depth Map** | `/zed2i/depth/depth_registered`<br> | `depth.publish_depth_map: true`<br> | `nav` only |
| **Depth Camera Info** | `/zed2i/depth/camera_info`<br> | `depth.publish_depth_map: true`<br> | `nav` only |
| **3D Point Cloud** | `/zed2i/points`<br> | `depth.publish_point_cloud: true`<br> | `nav` only |
| **Visual Odometry** | `/zed2i/odom`<br> | `pos_tracking.pos_tracking_enabled: true`, `publish_odom_pose: true`<br> | `nav` only |
| **Camera Pose** | `/zed2i/pose`<br> | `pos_tracking.pos_tracking_enabled: true`, `publish_odom_pose: true`<br> | `nav` only |


---


## 4. Available But Disabled Data

The ZED SDK and ROS 2 wrapper support extracting significantly more data, which is currently turned off in `zed2i_common.yaml`, `zed2i_rgb.yaml`, and `zed2i_nav.yaml` to save bandwidth and compute resources. These can be enabled if required for future features.

**Video & Imaging:**

* `publish_left_right`: Independent left and right camera images.
* `publish_raw`: Unrectified (raw, distorted) images.
* `publish_gray`: Grayscale versions of the images.
* `publish_stereo`: Side-by-side stereo composite image.

**Depth & 3D:**
* `publish_depth_info`: Specific depth intrinsic information.
* `publish_depth_confidence`: Confidence map evaluating the reliability of each depth pixel.
* `publish_disparity`: Disparity map image.
* `mapping_enabled`: Fused spatial mapping and large-scale point cloud generation.

**Sensors & Internal State:**
* `publish_cam_imu_transf`: The static transformation between the camera center and IMU.
* `publish_temp`: Camera temperature diagnostics.
* `publish_roi_mask`: Visual mask of the configured Region of Interest.

**Tracking & Pose:**
* `publish_3d_landmarks`: 3D visual features used by the positional tracking algorithm.
* `publish_pose_cov`: Pose data including covariance matrices.
* `publish_cam_path`: Historical trajectory path of the camera.


---


## 5. Troubleshooting

### udev Rule (sensor MCU permission denied)

The ZED SDK installer bakes a udev rule (`/etc/udev/rules.d/99-slabs.rules`) into the Docker image at build time, granting non-root access to the camera's USB/HID device nodes. If the camera opens but the sensor module doesn't, you'll see:

```text
[ZED][MCU] Permissions denied : can't open device. Make sure you have installed udev rules or use sudo
[ZED] A ZED camera was detected, but the sensors returned an invalid serial number.
...
NOT VALID SERIAL NUMBER FOR SENSORS MODULE MCU
```

**This rule must be installed on the host, not just present in the container image — and that's true for any container backend, distrobox included.** `/dev` is bind-mounted from the host into the container in both distrobox and `docker compose` (see `docker/docker-compose.dev.example.yaml`, `docker/docker-compose.prod.yaml`), so device node permissions are applied by the **host's** udevd. It never reads a rules file that only exists inside the container's own `/etc/udev/rules.d` — the container backend doesn't change that, only whether `/dev` is host-shared (it is, in both of our setups).

Install it manually on the host:

Pull the rule out of the running container and install it on the host
```bash
distrobox enter <container> -- cat /etc/udev/rules.d/99-slabs.rules | sudo tee /etc/udev/rules.d/99-slabs.rules >/dev/null
```

or, for a plain docker-compose container:
```bash
docker exec <container> cat /etc/udev/rules.d/99-slabs.rules | sudo tee /etc/udev/rules.d/99-slabs.rules >/dev/null
```

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then unplug/replug the camera (or just relaunch) — no container restart needed, since this only touches the host's udev state.

### GPU Passthrough (`libcuda.so.1` missing)

The ZED component container also needs the host's CUDA driver library, which is provided by mounting the NVIDIA driver in — not by anything baked into the image:

```
Failed to load library: Could not load library dlopen error: libcuda.so.1: cannot open shared object file: No such file or directory
```

* **`docker compose`**: needs the `deploy.resources.reservations.devices` (`driver: nvidia`) block present in the compose file, and the NVIDIA Container Toolkit installed on the host.
* **distrobox**: the box must be created with `--nvidia`. This can't be toggled on an existing box — it must be recreated (`distrobox rm` + `distrobox create --nvidia`). Note `/opt/ws` and `/opt/hw_ws` aren't bind-mounted, so recreating a box wipes the ROS build inside it — `colcon build` again afterward.