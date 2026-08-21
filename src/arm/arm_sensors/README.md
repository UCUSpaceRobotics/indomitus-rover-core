# arm_sensors

Driver bringup for the arm's onboard sensors — currently the OV5693 5MP
camera. One launch file, two backends, switched by the `backend`
argument:

- `usb` (default) — plain USB/UVC bridge board. **This is the only
  backend that has actually been run and confirmed working** (dev
  machine / laptop testing).
- `csi` — native MIPI CSI-2 on a Jetson, the real deployment target.
  **EXPERIMENTAL. Never once run against real CSI hardware** —
  `nvarguscamerasrc` is a Jetson-only GStreamer element and can't even
  be tested on a laptop, so this whole backend has only been reviewed,
  not verified. Do not treat it as production-ready and do not use it
  on the real arm until someone has actually run and validated it on
  the target Jetson.

Both publish to `/camera/image_raw` + `/camera/camera_info` with
`frame_id=arm_camera_optical_frame` — the same topics/frame the
simulated wrist camera uses (`arm_sim`'s `arm_gazebo.launch.py`), so
`aruco_tracker`, `panel_pose_fuser_node`, and anything else downstream
in the CV pipeline need no changes to run against this real driver
instead of the sim one. **This also means the two are interchangeable,
not simultaneous** — see "Real camera vs. simulation" below.

## On the laptop (dev machine, USB)

The camera must be plugged in via its USB/UVC bridge board. No extra
Docker configuration is needed for this: the repo's example dev
container config (`docker/docker-compose.dev.example.yaml`) already
mounts `/dev:/dev:rw`, which covers the camera's device node and its
stable by-id symlink (see "Known gotchas" below) with no changes.
(If a container was created before the camera was ever physically
plugged in, recreate it with `docker compose up -d rover_dev` — not
`docker compose restart`, which reuses the existing container and
won't pick up devices that weren't present at creation time.)

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

> **⚠ EXPERIMENTAL — not validated on real hardware.** This backend has
> never been run against an actual Jetson/CSI camera; everything below
> is reasoned from GStreamer/Argus documentation, not confirmed live.
> Treat it as a starting point for bringup, not something to trust on
> the real arm until it's actually been tested and validated on the
> target Jetson. `usb` above is the only backend with any live
> confirmation behind it.

Before trusting it on the robot:

- Confirm `nvarguscamerasrc` actually works standalone first:
  `gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! nvvidconv ! fakesink`
- The `csi_sensor_id` argument (default `0`) may need to change
  depending on which CSI port the camera is physically wired to.
- `csi_width`/`csi_height` default to 1280x720 — a practical resolution
  for the CV pipeline, not a hardware ceiling. The pipeline requests
  `format=NV12` explicitly (what `nvarguscamerasrc`'s NVMM buffers
  actually are on Jetson) rather than leaving it to negotiate. Raise
  the resolution up to the OV5693's native 2592x1944 (CSI isn't
  USB-bandwidth-limited the way the laptop's UVC bridge is) if the full
  5MP is actually needed, but the exact supported resolution/framerate
  combinations still need verifying on real hardware — nothing here has
  been load-tested at any resolution over CSI.
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

**Not done yet, and required before trusting any pose output from this
camera.** `camera_info_url` defaults to empty (uncalibrated, all-zero
distortion/intrinsics) — fine for initial bringup and just looking at
the image, but ArUco/panel pose estimation (`aruco_tracker`,
`panel_pose_fuser_node`) needs real intrinsics to produce a correct
pose. Uncalibrated numbers don't fail loudly — they silently produce a
*wrong* pose, not an error. Deliberately not included in this PR:
calibration only means something done with the camera in its final
mounted position at production resolution, and that hasn't happened
yet. See `config/README.md` for how to run a real checkerboard
calibration once it has (calibrate separately per backend/resolution —
different effective intrinsics).

## Real camera vs. simulation

**Do not launch this at the same time as the simulated wrist camera**
(`arm_sim`'s `arm_gazebo.launch.py`, `camera:=true`, the default). Both
intentionally publish to the exact same topics —
`/camera/image_raw` + `/camera/camera_info` — precisely so downstream
consumers (`aruco_tracker`, `panel_pose_fuser_node`, ...) don't need any
configuration change to run against real hardware instead of Gazebo.
That convenience is also the hazard: running both at once means two
publishers on the same topics, with whichever one happens to publish
last "winning" on any given tick — silently wrong/flickering data, not
an error. Run one or the other, never both.

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
