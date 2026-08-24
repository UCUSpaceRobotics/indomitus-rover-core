# Camera calibration

`ov5693_usb.yaml` is this camera unit's real checkerboard calibration
(`backend:=usb`), done with it mounted in its final position on the arm
— `camera.launch.py`'s `camera_info_url` defaults to it. Pass
`camera_info_url:=""` to go back to uncalibrated (all-zero distortion/
intrinsics) for quick bringup/eyeballing the image.

If this camera unit or its mounting ever changes, that calibration is
no longer valid — redo it:

```
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
  image:=/camera/image_raw camera:=/camera
```

Save the resulting `ost.yaml` here (e.g. `ov5693_usb.yaml` /
`ov5693_csi.yaml` — calibrate separately per backend, the two pipelines
have different resolutions and likely different effective intrinsics),
then pass it to the launch file:

```
ros2 launch arm_sensors camera.launch.py \
  camera_info_url:=file://$(pwd)/ov5693_usb.yaml
```
