# rover_cameras

- One `camera_node` process per camera in `config/cameras.yaml` (GStreamer `v4l2src`); a wedged or
  unplugged camera takes down only its own feed and is reopened.
- **GS video**: MJPEG over HTTP, never DDS — `http://10.42.0.1:<port>/<name>?action=stream`,
  `/?action=snapshot`, `/health`. Ports are GS UI tile slots (see `cameras.yaml`).
- **On the rover**: `/rover/cameras/<name>/image_raw` + `camera_info` at `ros_fps` / `ros_res`,
  stamped with the capture time. Not for the GS over Wi-Fi.
- **Dataset** (ROS services only), saved to `save_dir/<date>/<name>/`:
  - all cameras (`dataset_node`): `/rover/cameras/capture` (Trigger), `/rover/cameras/record`
    (SetBool), `/rover/cameras/record/toggle` (Trigger), `/rover/cameras/dataset_state` (latched);
  - one camera: `/rover/cameras/<name>/capture`, `…/record`, `…/dataset_state`;
  - gamepad buttons 14 (capture) / 15 (record toggle).
- **Run**: `ros2 launch rover_cameras cameras.launch.py [cameras:=arducam-mast,arm-camera]`; stop the
  GS's `start-cameras.sh` first (a camera serves one process). Test without hardware:
  `device: videotestsrc`.
- **Calibration**: `config/calibration/<name>.yaml`; without it `camera_info` has K = 0.
- **Not done**: not in `rover.launch.py`; no camera frames in the URDF; exposure / white balance on
  auto; not run on the Jetson yet.
