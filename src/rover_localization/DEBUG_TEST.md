# ArUco Debug Launch

`aruco_debug.launch.py` starts a local USB camera node and the ArUco tracker.
Use it for laptop or bench testing when no external camera driver is already
running.

```bash
ros2 launch rover_localization aruco_debug.launch.py
```

## Nodes

- `/camera/usb_cam`
  - Package: `usb_cam`
  - Executable: `usb_cam_node_exe`
  - Parameters: `config/usb_cam_params.yaml`
- `/aruco_tracker`
  - Package: `aruco_opencv`
  - Executable: `aruco_tracker_autostart`
  - Parameters: `config/aruco_params.yaml`
  - `default_param_file`: package default ArUco parameter file resolved by the
    launch file; current value is `config/aruco_params.yaml`.

## USB Camera Parameters

- `video_device`: camera device path. Current value: `/dev/video0`.
- `image_width`: image width in pixels. Current value: `640`.
- `image_height`: image height in pixels. Current value: `480`.
- `framerate`: target capture rate in frames per second. Current value: `30.0`.
- `pixel_format`: camera pixel format conversion. Current value: `yuyv2rgb`.
- `io_method`: capture I/O method. Current value: `mmap`.
- `camera_name`: name matched with camera calibration data. Current value:
  `laptop_camera`.
- `frame_id`: frame used in published image headers. Current value: `camera`.

## Camera Calibration

The launch file passes `config/approx_laptop_camera.yaml` as `camera_info_url`.
This is approximate laptop-camera calibration and is useful for quick testing.
Replace it with real camera calibration for accurate marker pose estimates.

## ArUco Parameters

- `cam_base_topic`: image topic base used by the tracker. Current value:
  `/camera/image_raw`.
- `marker_size`: physical marker side length in meters. Current value: `0.05`.
- `marker_dict`: OpenCV ArUco dictionary used by the printed marker. Current
  value: `4X4_50`.
- `publish_tf`: publishes detected marker poses to TF when enabled. Current
  value: `true`.

## Expected Topics

- `/camera/image_raw`: image stream from `usb_cam`.
- `/camera/camera_info`: calibration from `usb_cam`.
- `/aruco_detections`: detected marker IDs and poses.
- `/aruco_tracker/debug`: debug image with detected marker axes drawn on top.
- TF marker frames when `publish_tf` is `true`.
