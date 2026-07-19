# Monocular Camera Calibration Guide

This guide describes how to create intrinsic calibration for a monocular camera. Intrinsic calibration
estimates focal lengths, optical center, and lens distortion. Incorrect values
can produce inaccurate marker position, distance, and TF output even when marker
detection itself appears correct.

This package provides `calibrate_camera.py`, an offline utility that reads saved
chessboard images and writes OpenCV calibration parameters to an `.npz` file.
It does not capture images, subscribe to ROS topics, or automatically configure
`usb_cam`; those integration steps remain manual.

## Current Rover Configuration

The bench-test configuration currently uses:

- Camera driver: `usb_cam`.
- Device: `/dev/video0`.
- Resolution: `640x480`. # Highly debatable. Needs additional discussion.
- Frame rate: `30` FPS.
- Pixel format: `yuyv2rgb`.
- Camera name: `laptop_camera`.
- Camera frame: `camera`.
- Image topic: `/camera/image_raw`.
- Camera information topic: `/camera/camera_info`.
- Approximate calibration: `config/approx_laptop_camera.yaml`.

`aruco_debug.launch.py` loads `approx_laptop_camera.yaml`. Its distortion
coefficients are all zero, so it is suitable only for basic pipeline testing.
Do not use it as measured calibration or for evaluating ArUco pose accuracy.

## Required Equipment

- Final rover camera and lens.
- Flat, rigid chessboard target with accurately known square size.
- Ruler or calipers for measuring printed square size.

`python3-opencv` and `python3-numpy` are declared runtime dependencies. Install
workspace dependencies through rosdep from the workspace root:

```bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

## Prepare the Chessboard

Use a conventional black-and-white chessboard printed without page scaling.
Mount paper targets to rigid flat
surface. A bent or wavy target produces invalid observations.

Two measurements must be known:

1. Inner-corner grid size. `--cols` and `--rows` mean detectable inner corners,
   not printed squares. For example, a board containing 10 by 7 squares has 9
   columns by 6 rows of inner corners, so use `--cols 9 --rows 6`.
2. Physical side length of one square, in meters. Measure the printed target;
   do not trust the nominal printer setting. For a measured 24.5 mm square, use
   `--square-size-m 0.0245`.

Measure several squares across the board and divide by the number of squares.
This reduces single-edge measurement error. Record dimensions and keep the same
target until calibration is accepted.

## Lock the Camera Setup

Before collecting images:

1. Clean the lens and target.
2. Select the exact production resolution and field-of-view mode. Current bench
   configuration is `640x480`.
3. Set final lens focus. Disable autofocus if the camera permits it and rover
   operation uses fixed focus.
4. Prefer fixed exposure and white balance while collecting images if supported.
   At minimum, wait for automatic controls to settle before moving the board.
5. Mount the camera rigidly. Avoid cable strain that can move the lens or camera.
6. Confirm every chessboard corner can become sharp at intended working
   distances.

For rover deployment, calibrate the actual rover camera, not a laptop webcam.
Use `laptop_camera` calibration only for local debug hardware.

## Start the Camera

Build and source the workspace from its root first:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rover_localization
source install/setup.bash
```

The current debug launch starts both `usb_cam` and the ArUco tracker:

```bash
ros2 launch rover_localization aruco_debug.launch.py
```

Select another camera device when needed:

```bash
ros2 launch rover_localization aruco_debug.launch.py video_device:=/dev/video2
```

Before calibration, verify the stream is stable:

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/camera_info --once
```

Expected image dimensions are `640x480` for the current configuration. Stop and
correct the driver configuration if the published dimensions differ from the
dimensions intended for deployment.

## Save Calibration Images

`calibrate_camera.py` processes existing image files; it does not capture them.
Use a camera application, ROS image-saving tool, or a future capture utility to
save frames from the production stream. Preserve the camera's native resolution
and do not resize, crop, rotate, or undistort saved images.

The default input directory, relative to the current working directory, is:

```text
media/calibration/
```

Create a separate directory for each physical camera and calibration attempt.
For example:

```bash
mkdir -p media/calibration/rover_front_2026-07-19
```

The utility reads `.jpg`, `.jpeg`, `.png`, and `.bmp` files directly inside the
selected directory. Extension matching is case-insensitive. It does not search
subdirectories. Remove unrelated images from the directory before running it.

Use lossless PNG or BMP when possible. JPEG is supported, but heavy compression
can shift corner positions. Do not commit calibration photos or generated `.npz`
files unless maintainers explicitly request them.

## Collect a Complete Dataset

Capture at least 25 to 40 sharp, meaningfully different observations. More are
useful only when they add new board positions, tilts, rotations, or distances.
Many nearly identical center-facing images do not improve calibration.

Move slowly and pause until the board is sharp before allowing an observation.
Keep the entire chessboard visible, including all outer squares and all inner
corners. Reject frames with motion blur, glare, shadows hiding corners, partial
occlusion, or the board almost edge-on.

### Cover the Full Image

Collect observations with the board:

- Centered.
- Near the top edge.
- Near the bottom edge.
- Near the left edge.
- Near the right edge.
- Near the top-left corner.
- Near the top-right corner.
- Near the bottom-left corner.
- Near the bottom-right corner.

Near an edge or corner means the detected inner corners should approach that
part of the image while the full target remains visible. Edge observations are
important because lens distortion is usually strongest away from image center.

### Vary Board Tilt

At several image positions, include:

- Board approximately parallel to the camera sensor.
- Top tilted away or toward the camera by about 15 to 30 degrees.
- Bottom tilted away or toward the camera by about 15 to 30 degrees.
- Left side tilted away or toward the camera by about 15 to 30 degrees.
- Right side tilted away or toward the camera by about 15 to 30 degrees.

Avoid extreme tilt where corner locations become ambiguous or the pattern is
barely visible. Tilt observations are necessary to estimate focal length and
distortion independently.

### Vary Distance and Apparent Size

Include all of these distances:

- Close: target fills most of the frame, but every corner remains visible.
- Medium: target fills roughly half the frame.
- Far: target fills roughly one quarter to one third of the frame.

Use distances representative of expected ArUco detection on the rover. Ensure
the far target remains sharp enough for reliable corner detection.

### Vary In-Plane Rotation

Include:

- Normal horizontal or vertical orientation.
- Approximately 20 to 45 degrees clockwise.
- Approximately 20 to 45 degrees counter-clockwise.

Combine rotation with several positions and distances instead of collecting all
rotated observations at image center.

### Suggested Minimum Capture Plan

A practical 35-observation set is:

- 9 positions: center, four edges, and four corners.
- 10 tilted observations distributed across those positions.
- 6 close and 6 far observations at different positions.
- 4 rotated observations, split between clockwise and counter-clockwise.

Categories may overlap. For example, a close, clockwise-rotated board near the
top-left corner can satisfy several categories. Diversity matters more than an
exact image count.

## Run the Offline Calibration Utility

Build and source `rover_localization` so CMake installs the utility as a package
executable:

```bash
colcon build --symlink-install --packages-select rover_localization
source install/setup.bash
```

Run it from the workspace root. Replace all example measurements and paths with
the actual target and capture set:

```bash
ros2 run rover_localization calibrate_camera.py \
  --image-dir media/calibration/rover_front_2026-07-19 \
  --output media/calibration/rover_front_2026-07-19.npz \
  --cols 9 \
  --rows 6 \
  --square-size-m 0.0245
```

Arguments:

- `--image-dir`: directory containing one calibration attempt. Default:
  `media/calibration`.
- `--output`: output file path. Parent directories are created automatically.
  Default: `camera_calibration.npz`.
- `--cols`: number of inner corners in each chessboard row. Required; minimum
  value is `3`.
- `--rows`: number of inner corners in each chessboard column. Required; minimum
  value is `3`.
- `--square-size-m`: measured square side length in meters. Required.
- `--show`: briefly display every accepted image with detected corners. Press
  `q` or `Esc` to stop processing early.

The utility prints one status line per file:

- `accepted`: complete corner grid was found and refined.
- `no chessboard corners`: file was readable, but the requested grid was not
  detected.
- `skipped unreadable image`: OpenCV could not decode the file.

All accepted images must have exactly one resolution. The utility stops with an
error instead of combining different image sizes. It also stops without writing
output when fewer than five images are accepted. Five is only a mathematical
minimum; use the recommended 25 to 40 diverse images for rover calibration.

For quick corner-detection review, add `--show`:

```bash
ros2 run rover_localization calibrate_camera.py \
  --image-dir media/calibration/rover_front_2026-07-19 \
  --output /tmp/rover_front_camera.npz \
  --cols 9 --rows 6 --square-size-m 0.0245 --show
```

The output is a NumPy `.npz` archive containing:

- `camera_matrix`: `3x3` intrinsic matrix in OpenCV order.
- `dist_coeffs`: OpenCV lens-distortion coefficients.
- `image_size`: `[width, height]` used by calibration.
- `rms`: overall RMS reprojection error in pixels.
- `rvecs` and `tvecs`: estimated chessboard pose for every accepted image.
- `cols`, `rows`, and `square_size_m`: target metadata supplied on the command
  line.

Inspect an output file without modifying it:

```bash
python3 - <<'PY'
import numpy as np

with np.load("media/calibration/rover_front_2026-07-19.npz") as calibration:
    for key in calibration.files:
        print(key, calibration[key])
PY
```

As a practical heuristic, sub-pixel RMS reprojection error is expected from a
good, sharp dataset. Values below approximately `0.5` pixels are usually strong;
values above `1.0` pixel should trigger image inspection and normally a new
capture. Do not accept calibration based on RMS alone. Poor edge coverage can
produce low aggregate error but inaccurate correction near image boundaries.

## Integrate the Result with ROS

The generated `.npz` file is an OpenCV calibration archive. `usb_cam` and
`camera_info_manager` do not load this format directly; they expect a ROS camera
calibration YAML file. Automatic `.npz`-to-YAML conversion is not implemented
yet.

A future converter must map:

- `image_size[0]` and `image_size[1]` to `image_width` and `image_height`.
- `camera_matrix` to the YAML `camera_matrix` data.
- `dist_coeffs` to `distortion_coefficients` using `plumb_bob` for this standard
  OpenCV calibration model.
- Identity `3x3` matrix to `rectification_matrix` for a monocular camera.
- Intrinsic values to the `3x4` `projection_matrix`.
- Driver camera name to `camera_name`; it is `laptop_camera` in the current
  bench configuration.

Store reviewed YAML under `src/rover_localization/config/` with a physical-camera
name such as `rover_front_camera.yaml`. Verify its dimensions match the capture
set. Do not copy calibration between rover and laptop cameras.

The current `aruco_debug.launch.py` hard-codes
`config/approx_laptop_camera.yaml`. Integrating measured calibration therefore
also requires changing that path or adding a calibration-file launch argument.
Merely generating `.npz` or adding YAML to `config/` does not activate it.

## Validate the Result

### Confirm Published Intrinsics

Start the camera with the measured calibration and inspect the publication:

```bash
ros2 topic echo /camera/camera_info --once
```

Confirm width, height, distortion model, `d`, `k`, `r`, and `p` match the new
YAML. If values still match `approx_laptop_camera.yaml`, the launch file or
`camera_info_url` still points at the placeholder calibration.

### Inspect Rectification

Use the ROS image pipeline to rectify an image, then inspect straight physical
edges near image boundaries. Lines that are straight in the scene should remain
straight in the rectified image. Excessive bending, stretching, or cropped
output suggests wrong dimensions, wrong YAML, or poor calibration coverage.

`image_proc`/`image_view` may require the `ros-humble-image-pipeline` package;
they are not currently declared dependencies of `rover_localization`.

### Validate ArUco Pose

Run the ArUco debug pipeline and use a marker whose physical side length matches
the configured `marker_size` (`0.05` meters currently):

```bash
ros2 launch rover_localization aruco_debug.launch.py
```

Check:

- `/aruco_detections` publishes while the marker is visible.
- RViz **Add** > **By topic** > `/aruco_detections/camera` shows the webcam image
  and marker bounding box.
- A stationary marker produces a stable pose instead of large depth or angle
  jumps.
- Estimated marker distance is plausible when compared with a tape measure.
- TF marker output is present when `publish_tf` is enabled.

Intrinsic calibration and ArUco marker size affect different scale inputs.
Validate both: good intrinsics cannot compensate for an incorrectly measured
`marker_size`.

## Record Calibration Metadata

Include this information in the pull request or calibration record:

- Camera manufacturer, model, serial number, and rover mounting position.
- Device path used during capture.
- Resolution, pixel format, frame rate, focus, and field-of-view mode.
- Chessboard printed square count and inner-corner count.
- Measured square size in meters and measurement method.
- Number of accepted observations.
- Calibration date and operator.
- Reported reprojection error.
- Names of the generated `.npz` archive and reviewed ROS YAML file.
- Validation evidence, including rectified-image and ArUco pose checks.

## Acceptance Checklist

- [ ] Final physical camera and lens were calibrated.
- [ ] Production resolution and field-of-view mode were used.
- [ ] Target is flat and square size was measured in meters.
- [ ] `--cols` and `--rows` use inner corners, not printed squares.
- [ ] At least 25 to 40 sharp, diverse observations were collected.
- [ ] Center, edges, and all four image corners were covered.
- [ ] Multiple tilts, distances, and in-plane rotations were included.
- [ ] Entire chessboard is visible in every accepted observation.
- [ ] Utility accepted one consistent image resolution.
- [ ] Generated `.npz` metadata matches target measurements.
- [ ] Reprojection error and rectified output were reviewed.
- [ ] YAML dimensions and `camera_name` match driver configuration.
- [ ] Launch configuration actually loads the measured YAML.
- [ ] `/camera/camera_info` publishes the measured values.
- [ ] ArUco bounding box, pose, distance, and optional TF output were validated.





Calibration belongs to one physical camera and lens configuration. Recalibrate
when any of these change:

- Camera module or lens.
- Lens focus, if focus is manually adjustable.
- Image resolution, crop, binning, digital zoom, or aspect ratio.
- Driver mode that changes the camera's field of view.
- Lens position after impact, repair, or disassembly.

Changing only frame rate normally does not invalidate intrinsics, provided the
camera uses the same resolution and field of view. Camera-to-rover mounting TF
is extrinsic calibration and is outside the scope of this guide.


