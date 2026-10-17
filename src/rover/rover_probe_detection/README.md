# rover_probe_detection

ERC probes: detected in each camera's image, placed in 3D from several cameras.

```
rover_cameras ── cameras/<cam>/image_raw ──► probe_detector_node ── cameras/<cam>/probe_detections ─┐
              └─ cameras/<cam>/camera_info ──────────────────────────────────────────────────────────┤
robot_state_publishers ── /tf (camera optical frames) ───────────────────────────────────────────────┤
                                                                                                      ▼
                                                     probe_triangulator_node ── probes (Detection3DArray)
                                                                             └─ probes/markers (RViz)
```

- **Detector: not implemented yet.** The model comes from the course project
  (`ai-course-erc-sample-pose`); the plan and the model contract are in
  [detector.py](rover_probe_detection/detector.py). With `backend: none` the node only warns.
- **Triangulator.** Each detection (its bbox centre for now) becomes a ray from its camera, using
  `camera_info` (K, D; must be calibrated) and TF at the frame's capture stamp. Rays of 2+ cameras
  meeting within `max_ray_gap` are one probe, placed by least squares. A probe only one camera sees
  is placed on the ground plane (`single_view: ground`). Parameters: [config/probe_detection.yaml](config/probe_detection.yaml).
- **Output**: `vision_msgs/Detection3DArray` in `fixed_frame`; per probe `bbox.center` = position,
  `results[0]` = class, mean score, position covariance. Standard messages, so `arm_*` can consume
  them without depending on this package. In RViz the markers are green when triangulated, yellow
  when one camera + ground plane.

## Run

```bash
ros2 launch rover_probe_detection probe_detection.launch.py
# without a model: fake detections of known probes (x,y,z in base_footprint)
ros2 launch rover_probe_detection probe_detection.launch.py fake_probes:="1.5,0.3,0;2.2,-0.4,0"
```

Needs rover_cameras with calibrated cameras (`rover_cameras/config/calibration/`) and the
robot_state_publishers (camera frames). Cameras: `arducam_mast`, `arducam_fr_bottom`, `arm_camera`.

## Known limits / next

- `fixed_frame: base_footprint` is only right while the rover holds still within `max_age`;
  switch to `odom` once odom -> base_footprint is published (rover_localization's EKF).
- One frame's estimate per tick: no tracking or averaging over time yet.
- Keypoints (top/bottom) not used yet; triangulating both gives the probe's 3D axis.
- `point_height`: the bbox centre is ~ the probe's mid-height, which matters for the ground-plane
  fallback; set it once the probe's size is known.

Tests: `python3 -m pytest -q src/rover/rover_probe_detection/test`.
