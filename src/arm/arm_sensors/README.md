# arm_sensors

Driver bringup for the arm's onboard sensors — currently the OV5693 5MP
camera. One launch file, two backends, switched by the `backend`
argument:

- `usb` (default) — plain USB/UVC bridge board, for testing on a regular
  dev machine (laptop). This is the only backend actually verified live
  so far.
- `csi` — native MIPI CSI-2 on a Jetson, the real deployment target.
  **Not yet tested on real Jetson hardware** — `nvarguscamerasrc` can't
  even be tested on a laptop (it's a Jetson-only GStreamer element), so
  treat this path as unverified until it's actually run there.

Both publish to `/camera/image_raw` + `/camera/camera_info` with
`frame_id=arm_camera_optical_frame` — the same topics/frame the
simulated wrist camera uses (`arm_sim`'s `arm_gazebo.launch.py`), so
`aruco_tracker`, `panel_pose_fuser_node`, and anything else downstream
in the CV pipeline need no changes to run against this real driver
instead of the sim one.

## On the laptop (dev machine, USB)

The camera must be plugged in via its USB/UVC bridge board, and the
container needs `/dev/v4l` mounted (already set up in
`docker-compose.yaml` — if the container was created before that mount
existed, recreate it with `docker compose up -d rover_dev`, not
`docker compose restart`, which doesn't pick up new volume mounts).

```bash
docker exec -it rover_dev bash
source /opt/ros/humble/setup.bash
source /opt/ws/install/setup.bash
ros2 launch arm_sensors camera.launch.py
```

That's it — `backend` defaults to `usb`. Focus (fixed lens position,
autofocus off by default is available but currently left *on* by
choice — see the launch file's `disable_autofocus` argument),
brightness, and backlight compensation are all pre-tuned defaults, no
flags needed.

To view the live image:

```bash
# separate terminal, same container
docker exec -it rover_dev bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

## On the Jetson (real robot, CSI)

```bash
ros2 launch arm_sensors camera.launch.py backend:=csi
```

This is **unverified** — it's never been run against real CSI hardware.
Before trusting it on the robot:

- Confirm `nvarguscamerasrc` actually works standalone first:
  `gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! nvvidconv ! fakesink`
- The `csi_sensor_id` argument (default `0`) may need to change
  depending on which CSI port the camera is physically wired to.
- `csi_width`/`csi_height` default to the OV5693's native 2592x1944 —
  CSI isn't USB-bandwidth-limited the way the laptop's UVC bridge is,
  but this hasn't been load-tested at that resolution either.
- Focus and exposure/brightness controls (`focus_absolute`,
  `brightness`, `backlight_compensation`) are **USB-backend only** —
  they're plain V4L2 controls (`v4l2-ctl`), and `nvarguscamerasrc`
  doesn't expose the camera as a V4L2 device at all. Whatever
  focus/exposure behavior the Jetson's Argus ISP defaults to is what
  you get, until someone works out the CSI-side equivalent (likely
  Argus-specific GStreamer element properties, not `v4l2-ctl`).

## Tuning focus / brightness (USB backend)

Both depend on the camera's actual mounted position and lighting once
it's on the arm — the current defaults were tuned on a laptop at a
different distance/lighting, so re-tune once mounted:

```bash
# camera already running via camera.launch.py in another terminal
v4l2-ctl -d /dev/video4 -c focus_absolute=<0-1023>
v4l2-ctl -d /dev/video4 -c brightness=<0-255>,backlight_compensation=<0-2>
```

(Substitute the actual `/dev/videoN` node, or use the same by-id path
the launch file uses — see `video_device`'s description via
`ros2 launch arm_sensors camera.launch.py --show-args`.) Watch
`rqt_image_view` while adjusting, then once happy, set those values as
the new defaults in `launch/camera.launch.py`.

To re-enable autofocus/pin a manual focus, see the `disable_autofocus`
launch argument.

## Calibration

Not done yet — `camera_info_url` defaults to empty (uncalibrated,
all-zero distortion/intrinsics). See `config/README.md` for how to run
a real checkerboard calibration once the camera is in its final mounted
position (calibrate separately per backend/resolution — different
effective intrinsics).

## Known gotchas

- **USB backend uses raw YUYV, not this camera's MJPEG mode.** MJPEG
  decoding intermittently corrupted frames on this specific
  camera/USB stack (occasionally fatal); YUYV never did across repeated
  tests. The tradeoff is framerate, not a settings knob: ~29fps at
  640x480 (the default), only ~5-6fps at 1280x720 in practice (USB2.0
  bandwidth for raw video is much heavier than compressed MJPEG).
- **`video_device` defaults to a stable by-id path**
  (`/dev/v4l/by-id/usb-HZ_USB_Camera_20220301104-video-index0`), not
  `/dev/videoN` — that node number isn't stable across
  reboots/replugs/other USB devices connecting, the by-id symlink
  (keyed on this camera's USB serial, auto-created by Ubuntu's own
  `60-persistent-v4l.rules`) is. This only matches *this specific
  physical camera unit* — override `video_device` if it's ever swapped
  for a different one.
- Dev tools used for diagnosing the above (`v4l2-ctl`,
  `gst-launch-1.0`/`gst-inspect-1.0`, `rqt_image_view`) are installed in
  the Dockerfile's `dev` stage, not this package's `package.xml` — they
  aren't runtime dependencies of the camera driver itself.
