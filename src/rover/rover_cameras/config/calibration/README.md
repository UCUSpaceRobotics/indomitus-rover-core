# Camera calibrations

One `camera_calibration` YAML per camera, named `<camera name>.yaml` (the name from
`../cameras.yaml`, e.g. `arducam-front.yaml`); `cameras.launch.py` picks it up by name.
Calibrate at the capture resolution in `cameras.yaml`; the node scales K and P to `ros_res`.

    ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
        --ros-args -r image:=/rover/cameras/arducam_front/image_raw

Without a file the camera publishes `camera_info` with K = 0 (uncalibrated).
