# Camera calibration

No calibration file exists yet — `camera.launch.py`'s `camera_info_url`
defaults to empty (uncalibrated: `sensor_msgs/CameraInfo` with all-zero
distortion/intrinsics).

Once the OV5693 is mounted in its final position on the arm, run a real
checkerboard calibration:

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
