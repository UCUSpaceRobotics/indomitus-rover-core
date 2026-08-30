# Rover Monocular Camera Calibration & ArUco Tracking Guide

This guide outlines how to configure, launch, and verify the ArUco marker tracking system for rover localization, as well as perform offline intrinsic calibration for a monocular camera. Accurate intrinsic calibration (focal length, optical center, and lens distortion) is required for precise marker pose and distance estimations.

## 1. Launch Configurations

Depending on your environment, you can run the tracker standalone (for production) or alongside a local camera node (for debugging).

### Production Launch

Starts only the `aruco_tracker` node. A separate camera driver must already be running and publishing image and calibration topics.

```bash
ros2 launch rover_localization aruco.launch.py
```

### Debug Launch

Starts a local USB camera node alongside the `aruco_tracker`. This is primarily used for laptop or bench testing when no external driver is running.

```bash
ros2 launch rover_aruco aruco_debug.launch.py
```

## 2. Node Parameters & Topics

### The `aruco_tracker` Node

Configured via `config/aruco_params.yaml`.

* **`cam_base_topic`:** Image topic base used by the tracker (Default: `camera/image_raw`, resolved under the pushed `rover` namespace).
* **`marker_size`:** Physical marker side length in meters (Default: `0.05`).
* **`marker_dict`:** OpenCV ArUco dictionary used by the printed marker (Default: `4X4_50`).
* **`publish_tf`:** Publishes detected marker poses to TF when enabled (Default: `true`).

### The `/rover/camera/usb_cam` Node (Debug Only)

Configured via `config/usb_cam_params.yaml`.

* **Device:** `/dev/video0`.
* **Stream:** `640x480` resolution, `30.0` FPS, `yuyv2rgb` pixel format, and `mmap` I/O method.
* **Identifiers:** `laptop_camera` for the camera name, and `camera` for the frame ID.
* **Calibration:** Defaults to `config/approx_laptop_camera.yaml`. Because its distortion coefficients are zero, it is only suitable for basic pipeline testing.

### System Topics

All topics below are relative and pushed under the `rover` namespace (override with the `rover_namespace` launch argument or `ROVER_NAMESPACE` env var), e.g. `/rover/camera/image_raw`.

* **`camera/image_raw` (Input):** Camera image stream. The image header must have a valid `frame_id`.
* **`camera/camera_info` (Input):** Camera calibration data from the same camera namespace.
* **`aruco_detections` (Output):** Detected marker IDs and poses.
* **`aruco_tracker/debug` (Output):** Debug image overlaying detected marker axes on top of the feed.
* **`/tf` (Output):** Marker transforms (only if `publish_tf` is true) — stays global regardless of the pushed rover namespace.

---

## 3. Pipeline Verification Checklist

After starting your preferred launch file, point the camera at a configured marker and verify the data flow:

* **Verify image stream:** Run `ros2 topic hz /rover/camera/image_raw`.
* **Verify camera calibration:** Run `ros2 topic echo /rover/camera/camera_info --once`.
* **Verify marker detections:** Run `ros2 topic echo /rover/aruco_detections --once`.
* **Verify TF broadcasts:** Run `ros2 topic echo /tf --once` (if `publish_tf` is enabled).
* **Visual validation:** Run `rviz2`, then select **Add > By topic > /rover/aruco_detections/camera** (Note: this image path does not appear in standard topic lists). A working setup shows the webcam image and a bounding box over the detected marker. A **No Image** warning indicates a problem in the camera or detection pipeline.

---

## 4. ArUco Camera Calibration Guide

The package provides `calibrate_camera.py`, an offline utility that reads saved chessboard images and generates OpenCV parameters. It does not capture images or configure `usb_cam` automatically.

### Step 1: Prepare the Setup

* Clean the lens and mount the production camera rigidly to avoid cable strain.
* Set the camera to its exact production resolution and field-of-view mode. Disable autofocus, establish the final lens focus, and prefer fixed exposure and white balance.
* Count the *inner corners* of your flat chessboard target (e.g., a 10x7 square board has 9 columns and 6 rows of inner corners).
* Measure several printed squares, divide to find the average, and record the side length in meters.
* Install dependencies: `rosdep install --from-paths src --ignore-src --rosdistro humble -r -y`.

### Step 2: Collect the Dataset

Save **25 to 40 unedited, lossless frames (PNG or BMP)** from the production stream into a dedicated directory (e.g., `media/calibration/rover_front_2026-07-19`). Reject any frames with motion blur, glare, or heavy occlusion. Ensure the dataset includes:

* **Full Image Coverage:** Board placed centered, at all four edges, and in all four corners. Edge observations are critical for estimating distortion.
* **Varied Tilt:** Board parallel to the sensor, and tilted 15 to 30 degrees toward/away on all four sides.
* **Varied Distance:** Close (board fills the frame), medium (fills half), and far (fills 1/4 to 1/3).
* **Varied Rotation:** Normal orientation, and 20 to 45 degrees clockwise and counter-clockwise.

### Step 3: Run the Calibration Utility

Build the workspace and run the utility, replacing the arguments with your measured values and paths:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rover_aruco
source install/setup.bash

ros2 run rover_aruco calibrate_camera.py \
  --image-dir media/calibration/rover_front_2026-07-19 \
  --output media/calibration/rover_front_2026-07-19.npz \
  --cols 9 \
  --rows 6 \
  --square-size-m 0.0245 \
  --show
```

> **Note:** The `--show` flag briefly displays each accepted image; press `q` or `Esc` to skip. A good calibration typically produces a sub-pixel RMS reprojection error below `0.5`, but do not rely on RMS alone if edge coverage is poor.

### Step 4: Convert to ROS YAML

The utility outputs an OpenCV `.npz` archive. You must manually map this to a ROS YAML file:

* Map `image_size` to `image_width` and `image_height`.
* Transfer `camera_matrix` directly.
* Map `dist_coeffs` to `distortion_coefficients` using the `plumb_bob` model.
* Use an identity 3x3 matrix for the `rectification_matrix`.
* Transfer intrinsic values to the 3x4 `projection_matrix`.
* Ensure `camera_name` matches your driver configuration.

Store the YAML in `src/rover_aruco/config/` and update your camera launch files to point to it.

### Step 5: Post-Calibration Validation

* **Intrinsics:** Run `ros2 topic echo /rover/camera/camera_info --once` to confirm the new metrics are publishing.
* **Rectification:** Use `image_proc` to rectify the image and verify that straight physical lines remain straight near the image boundaries.
* **ArUco Pose:** Test a marker matching your configured `marker_size`. Confirm that the pose does not jump erratically and that the depth estimate aligns with physical tape measurements.

### Step 6: Record-Keeping and Recalibration

When submitting calibration data, document the camera hardware, device path, driver settings, chessboard metrics, accepted observation count, RMS, and validation evidence.

**When to Recalibrate:**
A calibration profile belongs to a specific physical camera and lens state. You must recalibrate if you change:

* The camera module, lens, or lens position (e.g., after an impact or disassembly).
* Manual focus settings.
* Image resolution, binning, crop, digital zoom, or field-of-view modes.

*(Note: Altering the frame rate generally does not invalidate intrinsics).*