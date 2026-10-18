# rover_cameras

The rover's USB cameras. One GStreamer pipeline per camera, three branches:

```
v4l2src ─ tee ─┬─ valve ─ jpegenc ─ HTTP MJPEG          → ground station (operators)
               ├─ valve ─ jpegenc ─ ~/capture, ~/record → dataset on the rover (ROS services)
               └─ rate limit ─ convert/scale ─ ROS      → cameras/<name>/image_raw, camera_info
```

- **Video to the ground station never goes through DDS.** It is plain MJPEG over HTTP, with
  the same video URLs and ports as `indomitus-ground-station/cameras/camera_mjpeg_server.py`,
  so UI tiles work unchanged. Dataset capture is **not** HTTP any more: see [Dataset](#dataset).
- **The ROS topics are for nodes on the rover** (perception), at `ros_fps` / `ros_res`, stamped
  with the capture time. Do not subscribe to them from the GS over Wi-Fi.
- JPEG branches encode only while someone watches or records (valves); the ROS branch always runs,
  so the cameras are always open.
- MJPG cameras (arm, thermal) pass their JPEGs through untouched; only the ROS branch decodes.
- `jpeg_encoder` is an element name: CPU `jpegenc` today, a hardware encoder later. H.264/WebRTC can
  be added as another branch when the board has an encoder.

## Run

```bash
# on the rover; stop the GS's servers first: a camera streams to one process only
./cameras/start-cameras.sh --stop          # in indomitus-ground-station
ros2 launch rover_cameras cameras.launch.py
ros2 launch rover_cameras cameras.launch.py cameras:=arducam-front,arm-camera
```

Then `http://10.42.0.1:<port>/<name>?action=stream` as before, and on the rover
`/rover/cameras/<name with - as _>/image_raw`.

`config/cameras.yaml` lists every camera that may be mounted (keys documented there). A camera that
is not plugged in waits for its device; one that errors or stops sending frames (`stall_timeout`)
is reopened. One process per camera, so a wedged camera takes down only its own feed.

Test without hardware: `device: videotestsrc`.

## Dataset

Frames are saved on the rover to `save_dir/<date>/<name>/<name>_<time>.jpg` at `save_quality`,
only on request, over ROS services. Every camera has its own; `dataset_node` (started by the
same launch) calls all of them at once.

| | type | |
|---|---|---|
| `/rover/cameras/capture` | `std_srvs/Trigger` | one frame from every capturing camera; message lists the paths |
| `/rover/cameras/record` | `std_srvs/SetBool` | save one frame every `record_interval` s on every camera, or stop |
| `/rover/cameras/record/toggle` | `std_srvs/Trigger` | on unless any camera already records |
| `/rover/cameras/dataset_state` | `indomitus_interfaces/DatasetState` | latched: `recording`, `cameras_live`, `saved` |
| `/rover/cameras/<name>/capture`, `…/record`, `…/dataset_state` | same | the same, one camera |

A camera that is not plugged in answers `not capturing` and is reported as skipped, not failed.
Recording lives in the cameras (a respawned camera comes back not recording); `dataset_state`
is a read-back. Gamepad: `dataset_capture_button` (14) and `dataset_record_button` (15) in
`rover_teleop/config/joy.yaml`.

```bash
ros2 service call /rover/cameras/capture std_srvs/srv/Trigger
ros2 service call /rover/cameras/arm_camera/capture std_srvs/srv/Trigger
ros2 service call /rover/cameras/record std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /rover/cameras/dataset_state
```

## HTTP

| path | |
|---|---|
| `/<name>?action=stream` | MJPEG (multipart/x-mixed-replace) |
| `/?action=snapshot` | one JPEG |
| `/health` | `ok name=… mode=… state=capturing\|waiting … frames=… recording=… saved=…` |

## Calibration

`config/calibration/<name>.yaml` (see the README there). Without it `camera_info` has K = 0.

## Not done yet

- Camera frames (`<name>_optical_frame`) in the URDF.
- Exposure / white balance are left on the camera's auto defaults (no `v4l2-ctl --set-ctrl`).
- Blank-frame detection of the GS server (only the first `warmup_frames` are dropped).
- Not in `rover.launch.py` yet; `save_dir` is inside the container unless mounted.
- Not run on the Jetson or the Arducams yet; CPU cost of `jpegenc` on the Orin Nano unmeasured.
