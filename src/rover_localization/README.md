# Rover Localization: ArUco Launch

`aruco.launch.py` starts only the ArUco marker tracker. It does not start a
camera driver, so a camera node must already be publishing image and calibration
topics.

```bash
ros2 launch rover_localization aruco.launch.py
```

## Node

- Name: `aruco_tracker`
- Package: `aruco_opencv`
- Executable: `aruco_tracker_autostart`
- Parameters: `config/aruco_params.yaml`
- `default_param_file`: package default parameter file resolved by the launch
  file; current value is `config/aruco_params.yaml`.

## Input Topics

- `/camera/image_raw`: camera image stream used by the tracker.
- `/camera/camera_info`: camera calibration from the same camera namespace.

## Parameters

- `cam_base_topic`: image topic base used by the tracker. Current value:
  `/camera/image_raw`.
- `marker_size`: physical marker side length in meters. Current value: `0.05`.
- `marker_dict`: OpenCV ArUco dictionary used by the printed marker. Current
  value: `4X4_50`.
- `publish_tf`: publishes detected marker poses to TF when enabled. Current
  value: `true`.

## Output Topics

- `/aruco_detections`: detected marker IDs and poses.
- `/aruco_tracker/debug`: debug image with detected marker axes drawn on top.
- TF marker frames when `publish_tf` is `true`.

## Notes

Camera calibration quality affects pose accuracy. The camera image header must
have a valid `frame_id`.

For local USB camera testing, see `DEBUG_TEST.md`.

## Verification Checklist

After starting the camera and ArUco tracker, point the camera at a configured
marker:

- [ ] Verify images arrive: `ros2 topic hz /camera/image_raw`
- [ ] Verify camera calibration: `ros2 topic echo /camera/camera_info --once`
- [ ] Verify marker detections: `ros2 topic echo /aruco_detections --once`
- [ ] If `publish_tf` is enabled, verify `/tf` contains a marker transform:
  `ros2 topic echo /tf --once`
- [ ] Run `rviz2`, then select **Add** > **By topic** >
  `/aruco_detections/camera`. This image path does not appear in
  `ros2 topic list`. A working setup shows the webcam image and, when a marker
  is detected, its bounding box. **No Image** indicates a problem in the camera
  or detection pipeline.
