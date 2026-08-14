# ZED2i Camera Configuration & Launch Guide

This document outlines the configuration, launch instructions, and data capabilities of the ZED2i stereo camera used on the rover.

**Official Documentation Resources:**

* [ZED2I Datasheet](https://cdn.sanity.io/files/s18ewfw4/staging/f6eb2a45caad4faa16149f747b765864a337ae91.pdf/ZED-2i-Datasheet.pdf)
* [ZED ROS 2 Integration Docs](https://docs.stereolabs.com/docs/integrations/ros-2)
* [ZED ROS 2 Wrapper Repository](https://github.com/stereolabs/zed-ros2-wrapper)


---


## 1. Launching the Camera

The camera is initialized using a custom launch file that wraps the official `zed_camera.launch.py` and applies specific topic remappings to fit the rover's architecture.

To launch the camera with the default configuration:

```bash
ros2 launch rover_sensors zed2i.launch.py
```

This automatically loads the parameters defined in `rover_sensors/config/zed2i.yaml`.


---


## 2. TF Publishing

**Transform (TF) publishing from the ZED node must remain disabled.**
The rover's `robot_state_publisher`, `ekf_node`, and `slam_toolbox` are strictly responsible for the TF tree. To prevent conflicts, the following launch arguments and YAML parameters default to `false`:

* `publish_tf`: Disabled in both launch arguments and YAML (`pos_tracking.publish_tf`).
* `publish_map_tf`: Disabled in both launch arguments and YAML (`pos_tracking.publish_map_tf`).
* `publish_urdf_tf`: Disabled in launch arguments.
* `publish_imu_tf`: Disabled in YAML (`sensors.publish_imu_tf`).


---


## 3. Published Data

The camera captures data at an internal resolution of `HD720` at 60 FPS. The active data streams are remapped in the launch file to match the rover's namespace.

Note: Visual odometry operates in `two_d_mode: true`, forcing navigation logic onto a flat plane (Z is fixed to 0.0, roll and pitch are zeroed). Depth is calculated using the `NEURAL_LIGHT` mode.

| Data Type | Published Topic | Enabling YAML Parameter |
| --- | --- | --- |
| **RGB Image (Rectified)** | `/zed2i/rgb/image_rect_color`<br> | `video.publish_rgb: true`<br> |
| **RGB Camera Info** | `/zed2i/rgb/camera_info`<br> | `video.publish_rgb: true`<br> |
| **Depth Map** | `/zed2i/depth/depth_registered`<br> | `depth.publish_depth_map: true`<br> |
| **Depth Camera Info** | `/zed2i/depth/camera_info`<br> | `depth.publish_depth_map: true`<br> |
| **3D Point Cloud** | `/zed2i/points`<br> | `depth.publish_point_cloud: true`<br> |
| **IMU Data** | `/zed2i/imu/data`<br> | `sensors.publish_imu: true`<br> |
| **Visual Odometry** | `/zed2i/odom`<br> | `pos_tracking.publish_odom_pose: true`<br> |
| **Camera Pose** | `/zed2i/pose`<br> | `pos_tracking.publish_odom_pose: true`<br> |
| **Node Status** | *(Internal status topics)* | `general.publish_status: true`<br> |


---


## 4. Available But Disabled Data

The ZED SDK and ROS 2 wrapper support extracting significantly more data, which is currently turned off in `zed2i.yaml` to save bandwidth and compute resources. These can be enabled if required for future features.

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