# Arducam Camera Configuration & Launch Guide

This document outlines the configuration, launch instructions, and udev setup for the Arducam USB cameras used on the rover (mast, rear, container).

**Each camera and its USB hub port are numbered with a physical sticker:**
* `1` = mast
* `2` = rear
* `3` = container

These numbers are what the udev rule setup below is keyed off of — always plug camera `N` into hub port `N`.

**Official Documentation Resources:**

* [v4l2_camera ROS 2 package docs](https://docs.ros.org/en/rolling/p/v4l2_camera/)
* [v4l2_camera source (ros2_v4l2_camera)](https://gitlab.com/boldhearts/ros2_v4l2_camera)
* [topic_tools (throttle) docs](https://github.com/ros-tooling/topic_tools)

---

## 1. Launching a Camera

The camera driver comes from the upstream `v4l2_camera` package. The rover package keeps the rover-specific launch file and per-camera default parameter files:

```text
src/rover_sensors/launch/arducam.launch.py
src/rover_sensors/config/arducam/mast.yaml
src/rover_sensors/config/arducam/rear.yaml
src/rover_sensors/config/arducam/container.yaml
```

The launch file starts three nodes: the `v4l2_camera_node` driver (named `arducam_node`) and two `topic_tools` `throttle` nodes that republish rate-limited copies of `image_raw` and `image_raw/compressed`.

```bash
ros2 launch rover_sensors arducam.launch.py camera_name:=mast camera_path:=/dev/arducam-mast
```

### Launch arguments

| Argument | Default | Description |
| --- | --- | --- |
| `camera_name` | `mast` | Selects the default config file: `config/arducam/<camera_name>.yaml`. One of `mast`, `rear`, `container`. Ignored if `config_path` is set. |
| `camera_path` | `/dev/video0` | Path to the camera device. Use the stable udev symlink (e.g. `/dev/arducam-mast`) once the rule below is installed, not the raw `/dev/videoN` index. |
| `camera_info_url` | `""` | URL to the camera calibration file (e.g. `file:///path/to/cal.yaml`). |
| `camera_frame_id` | `camera` | TF frame ID attached to the published image headers (e.g. `rear_arducam_optical_frame`). |
| `namespace` | `camera` | Namespace for the camera and throttle nodes. |
| `config_path` | `""` | Full path to a YAML file, overriding `camera_name`'s default config entirely. |
| `throttle_rate` | `""` | Overrides `msgs_per_sec` for both throttle nodes. Leave unset to use the value from the selected config file. |

Since the rover has three physical cameras, launch each one with a distinct `namespace`/`camera_frame_id`/`camera_path`, e.g.:

```bash
ros2 launch rover_sensors arducam.launch.py camera_name:=mast   camera_path:=/dev/arducam-mast   namespace:=mast_arducam   camera_frame_id:=mast_arducam_optical_frame
ros2 launch rover_sensors arducam.launch.py camera_name:=rear   camera_path:=/dev/arducam-rear   namespace:=rear_arducam   camera_frame_id:=rear_arducam_optical_frame
ros2 launch rover_sensors arducam.launch.py camera_name:=container camera_path:=/dev/arducam-container namespace:=container_arducam camera_frame_id:=container_arducam_optical_frame
```

---

## 2. Configure Arducam Udev Rules

Setting up udev rules ensures each camera is consistently recognized at the same device path (`/dev/arducam-mast`, `/dev/arducam-rear`, `/dev/arducam-container`) regardless of which raw `/dev/videoN` index it enumerates on or the order the cameras power up in, and is automatically granted read/write permissions on connection.

**Why matching is done by USB port, not serial number:** all three units are the same model (`Arducam B0495`), so `idVendor`/`idProduct` alone match all of them identically. Normally the per-unit USB serial number (`iSerial`) would disambiguate them, but on this hardware **all three units report the same serial** (a firmware quirk of the Cypress USB3 controller they use) — so serial-based rules don't work here. Instead, the rule matches on the physical USB hub port the camera is plugged into (`KERNELS`), which is exactly why the sticker numbering at the top of this doc matters: as long as camera `N` stays in hub port `N`, its `KERNELS` path stays stable.

Because `KERNELS` is a physical-topology match, do this **one camera at a time** — plug in only the camera you're currently configuring.

**1. With only that camera plugged in, list its device nodes:**

```bash
v4l2-ctl --list-devices | grep -A2 Arducam
```

This lists two `/dev/videoN` nodes (capture + metadata).

**2. Find which node is the capture node, and its USB port path:**

```bash
udevadm info -a -n /dev/videoN | grep -E "looking at device|ATTR\{index\}"
```

Run for both nodes from step 1. The one printing `ATTR{index}=="0"` is the capture node — use it (the other, `ATTR{index}=="1"`, is a metadata-only node and must not be used). Note the devpath segment right before `:1.0` in the `looking at device` line, e.g.:

```text
.../usb2/2-1/2-1.3/2-1.3.1/2-1.3.1:1.0/video4linux/video2
```

Here the port path is `2-1.3.1` — that's the value for `KERNELS`.

**3. Write (or append) the rule line**, replacing `<PORT>` and `<NAME>`:

```bash
sudoedit /etc/udev/rules.d/99-arducam.rules
```

```text
SUBSYSTEM=="video4linux", KERNELS=="<PORT>", ATTR{index}=="0", MODE:="0666", SYMLINK+="arducam-<NAME>"
```

**4. Apply the changes:**

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**5. Verify:**

```bash
ls -l /dev/arducam-<NAME>
```

> **Success:** The symlink should point to the active capture node (e.g. `lrwxrwxrwx ... /dev/arducam-mast -> video2`).

**6. Repeat steps 1-5** for the other two cameras, appending a new line to the same rule file each time (don't overwrite the previous lines).

This rover's current rule (`/etc/udev/rules.d/99-arducam.rules`), matching the sticker numbering (`1`=mast, `2`=rear, `3`=container) on this specific hub:

```text
SUBSYSTEM=="video4linux", KERNELS=="2-1.3.1",   ATTR{index}=="0", MODE:="0666", SYMLINK+="arducam-mast"
SUBSYSTEM=="video4linux", KERNELS=="2-1.3.2",   ATTR{index}=="0", MODE:="0666", SYMLINK+="arducam-rear"
SUBSYSTEM=="video4linux", KERNELS=="2-1.3.3.1", ATTR{index}=="0", MODE:="0666", SYMLINK+="arducam-container"
```

**Caveat:** `KERNELS` ties the rule to this exact physical port on this exact hub/host. If a camera or the hub itself is ever rewired to a different port, its `KERNELS` value changes and the corresponding line must be updated to match — re-run steps 1-2 for that camera to get its new port path.

---

## 3. Arducam parameters (`config/arducam/<camera_name>.yaml`)

| Parameter | Default value | Description |
| --- | --- | --- |
| `image_size` | `[1920, 1200]` | Resolution requested from the camera. |
| `pixel_format` | `YUYV` | Raw pixel format requested from the V4L2 hardware. Must be one of the formats the sensor natively advertises. |
| `output_encoding` | `rgb8` | Image encoding published on `/image_raw`. Since the sensor only captures `YUYV`, the driver performs a software conversion to this encoding on every frame. |
| `brightness` | `0` | Hardware image brightness. |
| `contrast` | `10` | Hardware image contrast. |
| `saturation` | `10` | Hardware color saturation. |
| `gain` | `168` | Hardware sensor gain (ISO equivalent) for low light. |
| `white_balance_automatic` | `true` | Toggles auto white balance. |
| `white_balance_temperature` | `4500` | Manual white balance in Kelvin (requires `white_balance_automatic: false`). |
| `auto_exposure` | `0` | Toggles automatic exposure (values depend on UVC spec, usually 3=auto, 1=manual). |
| `exposure_time_absolute` | `5` | Manual exposure time (requires `auto_exposure` set to manual mode). |
| `power_line_frequency` | `0` | Anti-flicker setting for indoor lighting (0=disabled, 1=50Hz, 2=60Hz). |
| `msgs_per_sec` (`throttle_raw` / `throttle_compressed`) | `30.0` | Rate limit applied to the throttled `image_raw_slow` / `image_raw_slow/compressed` output topics. |

The three camera config files are currently identical placeholders — tune each one's hardware parameters once the real per-camera differences (mounting, lighting, lens) are known.

---

## 4. Check if a camera is working

```bash
ros2 topic list
```

You should see, under the chosen `namespace`:

```text
/<namespace>/image_raw
/<namespace>/camera_info
/<namespace>/image_raw_slow
/<namespace>/image_raw_slow/compressed
```

```bash
ros2 topic echo /<namespace>/camera_info
```

Expected message type:

```text
sensor_msgs/msg/Image
```

---

## 5. Troubleshooting

### Permission denied on a specific control ("Camera Controls")

```text
[v4l2_camera]: Failed getting value for control 10092545: Permission denied (13); returning 0!
```

This is a UVC extension-unit control that isn't covered by the `video` group / standard V4L2 permissions. It's a benign warning — the driver logs it and continues; it does not affect `image_raw`.

### `Control type not currently supported: 6, for control: Camera Controls`

The `v4l2_camera` driver doesn't know how to expose this particular extension-unit control as a ROS parameter. Harmless, upstream limitation — not something fixable from this package.

### `Image encoding not the same as requested output ... yuv422_yuy2 => rgb8`

Expected: the Arducam sensor only captures `YUYV`, and `output_encoding: "rgb8"` asks the driver to convert every frame in software. This is a performance note, not an error — see `pixel_format` in the parameters table above.

### Camera calibration file not found

```text
[camera_calibration_parsers]: Unable to open camera calibration file [...]
```

Expected when `camera_info_url` isn't set — the driver falls back to a default path that doesn't exist yet. Pass `camera_info_url:=file:///path/to/cal.yaml` once a calibration file exists for that camera.

### Camera doesn't recover after a USB replug

`v4l2_camera_node` has `respawn=True` set (it restarts automatically if the *process* dies), but on a USB disconnect the driver doesn't crash — its capture loop just logs `Error dequeueing buffer: No such device (19)` every 10ms forever on the same now-invalid file descriptor, and never reopens the device even after it reappears at a new node. Because the process never exits, `respawn` never triggers. There is currently no automatic recovery for this case — the node must be manually restarted after a replug. (A udev-driven watchdog that kills the node on device-remove, letting `respawn` restart it against the fresh device node, would fix this; not implemented yet.)

### `topic_tools` throttle node exits immediately ("Statically typed parameter 'msgs_per_sec' must be initialized")

Means `msgs_per_sec` wasn't available from either the resolved config file or `throttle_rate`. Check that `config_path` (if set) is an **absolute path** — a relative path silently fails to load (`launch_ros` logs `Parameter file path is not a file`), leaving the throttle nodes with no `msgs_per_sec` value and the camera node running on hardcoded driver defaults instead of the selected yaml.
