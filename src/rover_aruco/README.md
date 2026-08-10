# Monocular Camera Calibration & ArUco Debug Guide

This guide outlines how to use the local debug launch setup and perform offline intrinsic calibration for a monocular camera. Intrinsic calibration estimates focal lengths, the optical center, and lens distortion, which are required for accurate ArUco marker pose and distance estimations.

## ArUco Debug Launch Configuration

The `aruco_debug.launch.py` file starts a local USB camera node and the ArUco tracker, primarily used for laptop or bench testing when no external driver is running. Start it using:

```bash
ros2 launch rover_aruco aruco_debug.launch.py
```

### Nodes and Parameters

* **`/camera/usb_cam`**: Configured via `config/usb_cam_params.yaml`.
* **Device:** `/dev/video0`.
* **Stream:** `640x480` resolution, `30.0` FPS, `yuyv2rgb` pixel format, and `mmap` I/O method.
* **Identifiers:** `laptop_camera` for the camera name, and `camera` for the frame ID.
* **`/aruco_tracker`**: Configured via `config/aruco_params.yaml`.
* **Targeting:** Subscribes to `/camera/image_raw`.
* **Detection:** Uses a physical `marker_size` of `0.05` meters and the `4X4_50` `marker_dict`.
* **Output:** `publish_tf` is set to `true` to broadcast detected marker poses.


### Expected Topics

* `/camera/image_raw`: Image stream.
* `/camera/camera_info`: Camera calibration data.
* `/aruco_detections`: Detected marker IDs and poses.
* `/aruco_tracker/debug`: Debug images overlaying marker axes.


> **Calibration Note:** By default, the launch uses `config/approx_laptop_camera.yaml`. Because its distortion coefficients are all zero, it is only suitable for basic pipeline testing and must be replaced with measured calibration before evaluating pose accuracy.


---

## ArUco Calibration

### 1. Prepare for Calibration

The package provides `calibrate_camera.py`, an offline utility that reads saved chessboard images and generates OpenCV parameters. It does not capture images or configure `usb_cam` automatically.

#### Equipment & Dependencies

* Final rover camera, lens, and a rigid, flat chessboard target.
* Measuring tools (calipers/ruler) to measure a printed square.
* Install dependencies: `rosdep install --from-paths src --ignore-src --rosdistro humble -r -y`.


#### Chessboard Configuration

* **Grid Size:** Count the *inner corners*, not the printed squares (e.g., a 10x7 square board has 9 columns and 6 rows of inner corners).
* **Square Size:** Measure several printed squares, divide to find the average, and record the side length in meters.


#### Lock the Setup

* Clean the lens and mount the camera rigidly to avoid cable strain.
* Set the camera to its exact production resolution and field-of-view mode.
* Disable autofocus, establish final lens focus, and prefer fixed exposure and white balance.

---

### 2. Collect the Dataset

Save unedited, lossless frames (PNG or BMP) from the production stream into a dedicated directory, such as `media/calibration/rover_front_2026-07-19`.

Capture **25 to 40 sharp, diverse observations**, ensuring the entire chessboard is visible in every frame. Reject frames with motion blur, glare, or heavy occlusion. Ensure the dataset includes:

* **Full Image Coverage:** Board placed centered, at all four edges, and in all four corners. Edge observations are critical for estimating distortion.
* **Varied Tilt:** Board parallel to the sensor, and tilted 15 to 30 degrees toward/away on all four sides.
* **Varied Distance:** Close (board fills the frame), medium (fills half), and far (fills 1/4 to 1/3).
* **Varied Rotation:** Normal orientation, and 20 to 45 degrees clockwise and counter-clockwise.

---

### 3. Run the Calibration Utility

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

* The `--show` flag briefly displays each accepted image; press `q` or `Esc` to skip.
* The script requires exactly one consistent image resolution and a minimum of five accepted images to output the `.npz` archive.
* A good calibration typically produces a sub-pixel RMS reprojection error below `0.5`, but do not rely on RMS alone if edge coverage is poor.

---

### 4. Integrate & Validate Calibration

#### Convert to ROS YAML

The utility outputs an OpenCV `.npz` archive. You must manually map this to a ROS YAML file:

* Map `image_size` to `image_width` and `image_height`.
* Transfer `camera_matrix` directly.
* Map `dist_coeffs` to `distortion_coefficients` using the `plumb_bob` model.
* Use an identity 3x3 matrix for the `rectification_matrix`.
* Transfer intrinsic values to the 3x4 `projection_matrix`.
* Ensure `camera_name` matches your driver configuration.

Store the YAML in `src/rover_aruco/config/` and update your launch files to point to it.


#### Validation Checks

* **Intrinsics:** Run `ros2 topic echo /camera/camera_info --once` to confirm the new metrics are publishing.
* **Rectification:** Use `image_proc` to rectify the image and verify that straight physical lines remain straight near the image boundaries.
* **ArUco Pose:** Test a marker matching your configured `marker_size`. Confirm that the pose does not jump erratically and that the depth estimate aligns with physical tape measurements.

---

### 5. Record-Keeping and Recalibration

When submitting calibration data, document the camera hardware, device path, driver settings, chessboard metrics, accepted observation count, RMS, and validation evidence.

**When to Recalibrate:**
A calibration profile belongs to a specific physical camera and lens state. You must recalibrate if you change:

* The camera module, lens, or lens position (e.g., after an impact or disassembly).
* Manual focus settings.
* Image resolution, binning, crop, digital zoom, or field-of-view modes.
(Note: Altering the frame rate generally does not invalidate intrinsics).