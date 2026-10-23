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
- **Run**: `ros2 launch rover_cameras cameras.launch.py [cameras:=arducam-mast,arm-camera]
  [include:=zed2i] [exclude:=arm-camera]`; stop the GS's `start-cameras.sh` first (a camera serves one process). Test
  without hardware: `device: videotestsrc`.
- **Which cameras start**: see [below](#which-cameras-start).
- **Stereo** (ZED 2i as UVC, sensors side by side): `stereo: left|right` keeps one sensor for every
  output, `both` (default) the joined frame. `res` stays the joined size; `ros_res` and the
  calibration are for what is kept.
- **ZED 2i: this or `rover_sensors/zed2i.launch.py`, never both.** Both open the same device and the
  second one fails. So `zed2i` has `autostart: false` and starts only when asked for
  (`include:=zed2i`, `cameras:=zed2i` or `cameras:=all`); don't while the ZED SDK runs (nav, VIO,
  depth); see
  [zed2i.md](../rover_sensors/docs/zed2i.md#zed-2i-already-in-use-by-rover_cameras).
- **Calibration**: `config/calibration/<name>.yaml`; without it `camera_info` has K = 0.
- **Not done**: not in `rover.launch.py`; no camera frames in the URDF; exposure / white balance on auto.

## Which cameras start

**started = (base set + `include`) − `exclude`**, in `cameras.yaml` order. `cameras` sets the base:

| `cameras:=` | Base set |
|---|---|
| *(empty, default)* | every camera with `autostart: true` (the default for each camera) |
| `a,b` | exactly `a`, `b`; `autostart` ignored |
| `all` | every camera; alone only, and no camera may be called `all` |

`exclude` wins over `include`; overlaps and repeats are fine. Unknown names in any of the three
fail the launch, so a typo never silently skips a camera.

| Arguments | Starts |
|---|---|
| *(none)* | all but `zed2i` (`autostart: false`) |
| `include:=zed2i` | the defaults + `zed2i` |
| `cameras:=zed2i` | only `zed2i` |
| `cameras:=all exclude:=thermal-camera-front` | everything but the thermal camera |
| `cameras:=arducam-mast include:=zed2i` | `arducam-mast`, `zed2i` (same as `cameras:=arducam-mast,zed2i`) |
| `include:=zed2i exclude:=zed2i` | the defaults (`exclude` wins) |
