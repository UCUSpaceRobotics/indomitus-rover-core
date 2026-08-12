# TF Ownership

This document lists every transform published in the system and which single node is responsible for broadcasting it. **Each transform must have exactly one publisher.** If you add a new node or controller that could plausibly publish TF, check this table first and update it if ownership changes.

## Current tree

```text
map
 └── odom
      └── base_footprint
           └── base_link
                ├── suspension_base_axii_link
                │    └── main_body_link
                │         ├── zed2i_camera_link
                │         └── laser_link
                ├── fl_wheel_mount_link -> fl_wheel_link
                ├── fr_wheel_mount_link -> fr_wheel_link
                ├── bl_wheel_mount_link -> bl_wheel_link
                ├── br_wheel_mount_link -> br_wheel_link
                ├── l_rocker_link
                └── r_rocker_link

```

## Ownership table

Here is the combined table keeping everything from `HEAD` and only the `world -> panel/panel_base_link` row from `develop`:

| Transform | Owning Node | Package | Config / Source |
| --- | --- | --- | --- |
| `map -> odom` | `async_slam_toolbox_node` | `slam_toolbox` | Launched via `rover_localization/launch/slam.launch.py` using `slam_toolbox_params.yaml`. |
| `odom -> base_footprint` | `ekf_filter_node` (`robot_localization`) | `rover_localization` | **Real Hardware:** `config/ekf.yaml` (fuses `/wheels/odom` + `/zed2i/odom`). **Simulation:** `config/ekf_sim.yaml` (fuses only `/wheels/odom`). Both configurations set `publish_tf: true`, `odom_frame: odom`, `base_link_frame: base_footprint`. |
| `base_footprint -> *` *(all static & dynamic links)* | `robot_state_publisher` | `rover_description` | Driven by `/joint_states` + `rover.urdf.xacro`. Launched via `robot_state_publisher.launch.py`. |
| `world -> panel/panel_base_link -> *` (panel base + switch/breaker links) | `robot_state_publisher` | `panel_description` | driven by `/joint_states` + `panel_standalone.urdf.xacro`; verified via `panel_bringup/panel_standalone.launch.py` (there, unprefixed: `world -> panel_base_link`, since standalone has no namespace to collide with). When spawned inside `rover_sim/sim_gz_full.launch.py` it runs under a `panel` namespace with `frame_prefix:='panel/'` set (so frame IDs are actually `panel/panel_base_link` etc., not just the topics) and `tf`/`tf_static` remapped back onto the global `/tf`/`/tf_static` topics, so the prefixed frames land on the one shared TF tree. The `world -> panel/panel_base_link` pose is generated from the same `panel_x`/`panel_y`/`panel_z`/`panel_yaw` launch args passed to the Gazebo spawn, so the two stay in sync -- **not yet verified end-to-end in Gazebo** (blocked on the Gazebo version issue in [`panel_sim.md`](https://www.google.com/search?q=../panel/panel_sim.md)), but the frame-naming and pose-sync code path is in place. |

## Non-owners (explicitly verified)

* **`odometry_controller`** (`rover_controller`) — Publishes `nav_msgs/Odometry` on `/wheels/odom` only. It does **not** broadcast any TF transform — confirmed by inspecting `publish_odom()`: no `tf2_ros::TransformBroadcaster` exists in this controller. Its topic is consumed as the `odom0` input to `ekf_node` (used in both real and simulation configurations). This separation is intentional — do not add a TF broadcast here, since that would duplicate `ekf_node`'s ownership.
* **`zed_node`** (`zed_wrapper`) — Visual odometry TF broadcasting is intentionally disabled. Verified in `config/zed2i.yaml` that `pos_tracking.publish_tf: false`, `pos_tracking.publish_map_tf: false`, and `sensors.publish_imu_tf: false`. It provides the `/zed2i/odom` topic consumed as `odom1` by the EKF **in the real hardware configuration only**, ensuring it does not conflict with `ekf_filter_node` or `slam_toolbox`.

## Rules for future changes

1. Before adding any new node that fuses odometry, publishes pose, or integrates sensor data into a frame already listed above, check this table — do not enable a second `publish_tf`-style flag for an already-owned transform.
2. If ownership of any transform changes, update this table in the same PR.