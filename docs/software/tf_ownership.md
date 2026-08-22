# TF Ownership

This document lists every transform published in the system and which single node is responsible for broadcasting it. **Each transform must have exactly one publisher.** If you add a new node or controller that could plausibly publish TF, check this table first and update it if ownership changes.

## Current tree

```text
map
 └── odom
      └── base_footprint  (flat: z=0, roll=pitch=0)
           └── base_link  (real, tilted 6-DOF pose)
                ├── base_link_ground_ref  (tilts with base_link, sits at ground height)
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

`base_footprint -> base_link` is **not** a URDF joint (a `fixed` joint can't
vary at runtime, and `base_footprint` is deliberately not declared as a URDF
link at all -- a URDF can only have one root link, and it can't parse with
`base_footprint` sitting disconnected next to `base_link`). It's a live TF
edge published by a second `robot_localization` instance, `ekf_tilt_node`,
carrying real roll/pitch from the IMU while `base_footprint` itself stays
flat for nav2/`slam_toolbox`. See the ownership table below and the
rationale in `rover.urdf.xacro`.

## Ownership table

Here is the combined table keeping everything from `HEAD` and only the `world -> panel/panel_base_link` row from `develop`:

| Transform | Owning Node | Package | Config / Source |
| --- | --- | --- | --- |
| `map -> odom` | `async_slam_toolbox_node` | `slam_toolbox` | Launched via `rover_localization/launch/slam.launch.py` using `slam_toolbox_params.yaml`. |
| `odom -> base_footprint` | `ekf_filter_node` (`robot_localization`) | `rover_localization` | **Real Hardware:** `config/ekf.yaml` (fuses `/wheels/odom` + `/zed2i/odom`; `two_d_mode: true`). **Simulation:** `config/ekf_sim.yaml` (fuses only `/wheels/odom`; `two_d_mode: true`). Both configurations set `publish_tf: true`, `odom_frame: odom`, `base_link_frame: base_footprint`. Deliberately no IMU/tilt input — this filter only ever tracks `(x, y, yaw)`, keeping `base_footprint` flat per REP 105/120. |
| `base_footprint -> base_link` | `ekf_tilt_node` (`robot_localization`) | `rover_localization` | `config/ekf_tilt.yaml`, shared by real and sim (fuses only `/zed2i/imu/data`, identical in both). Launched alongside `ekf_filter_node` via `launch/ekf.launch.py`. Sets `world_frame: base_footprint`, `odom_frame: base_footprint` (aliased to satisfy `robot_localization`'s world_frame-must-equal-odom_frame-or-map_frame constraint), `base_link_frame: base_link`, `two_d_mode: false`. Fuses roll/pitch only from the IMU (yaw dropped — `base_footprint`'s own filter already owns heading; safe because roll/pitch are yaw-invariant). `x`/`y`/`z` are never observed, so they hold at `initial_state`'s seed value `(0, 0, 0.4597)` forever — this must be kept in sync by hand with `base_link_ground_height` in `rover_description/urdf/properties.xacro` (derived from the suspension/wheel stack-up: `wheel_radius - rocker_hub_z + wheel_mount_drop`; robot_localization loads plain YAML, so it can't share that xacro property directly). `odometry/filtered` is remapped to `/tilt_ekf/odometry_filtered` and unused — only the TF broadcast matters. |
| `base_link -> *` *(all static & dynamic links)* | `robot_state_publisher` | `rover_description` | Driven by `/joint_states` + `rover.urdf.xacro`. Launched via `robot_state_publisher.launch.py`. `base_link` is the sole URDF root — `base_footprint` is intentionally not declared as a URDF link (see tree note above). |
| `world -> panel/panel_base_link -> *` (panel base + switch/breaker links) | `robot_state_publisher` | `panel_description` | driven by `/joint_states` + `panel_standalone.urdf.xacro`; verified via `panel_bringup/panel_standalone.launch.py` (there, unprefixed: `world -> panel_base_link`, since standalone has no namespace to collide with). When spawned inside `rover_sim/sim_gz_full.launch.py` it runs under a `panel` namespace with `frame_prefix:='panel/'` set (so frame IDs are actually `panel/panel_base_link` etc., not just the topics) and `tf`/`tf_static` remapped back onto the global `/tf`/`/tf_static` topics, so the prefixed frames land on the one shared TF tree. The `world -> panel/panel_base_link` pose is generated from the same `panel_x`/`panel_y`/`panel_z`/`panel_yaw` launch args passed to the Gazebo spawn, so the two stay in sync -- **not yet verified end-to-end in Gazebo** (blocked on the Gazebo version issue in [`panel_sim.md`](https://www.google.com/search?q=../panel/panel_sim.md)), but the frame-naming and pose-sync code path is in place. |

## Non-owners (explicitly verified)

* **`odometry_controller`** (`rover_controller`) — Publishes `nav_msgs/Odometry` on `/wheels/odom` only. It does **not** broadcast any TF transform — confirmed by inspecting `publish_odom()`: no `tf2_ros::TransformBroadcaster` exists in this controller. Its topic is consumed as the `odom0` input to `ekf_node` (used in both real and simulation configurations). This separation is intentional — do not add a TF broadcast here, since that would duplicate `ekf_node`'s ownership.
* **`zed_node`** (`zed_wrapper`) — Visual odometry TF broadcasting is intentionally disabled. Verified in `config/zed2i.yaml` that `pos_tracking.publish_tf: false`, `pos_tracking.publish_map_tf: false`, and `sensors.publish_imu_tf: false`. It provides the `/zed2i/odom` topic consumed as `odom1` by the EKF **in the real hardware configuration only**, ensuring it does not conflict with `ekf_filter_node` or `slam_toolbox`. Its `/zed2i/imu/data` topic is consumed by `ekf_tilt_node` instead, a separate `robot_localization` instance owning a different TF edge — not a conflict.
* **`robot_state_publisher`** — no longer publishes `base_footprint -> base_link`. That edge used to be a static `fixed` joint (`base_footprint_joint`) in `rover.urdf.xacro`; it was removed because a fixed joint can never vary at runtime, and the whole point of this change was to let that edge carry real, time-varying tilt. `base_footprint` is not declared as a URDF link at all anymore — see the tree note above.
* **`ekf_filter_node`** — does not fuse IMU orientation and does not touch `base_link`. It only ever publishes `odom -> base_footprint`, and only ever tracks `(x, y, yaw)` (`two_d_mode: true` in both real and sim configs). Tilt is intentionally out of scope for this filter — see `ekf_tilt_node`'s row above.

## Rules for future changes

1. Before adding any new node that fuses odometry, publishes pose, or integrates sensor data into a frame already listed above, check this table — do not enable a second `publish_tf`-style flag for an already-owned transform.
2. If ownership of any transform changes, update this table in the same PR.
3. `base_footprint` must stay flat (REP 105/120: z=0, roll=pitch=0) — it's what nav2 (`robot_base_frame`) and `slam_toolbox` (`base_frame`) are built assuming. Any consumer that needs the rover's *real* tilted pose should read `base_link` (body-height origin) or `base_link_ground_ref` (same tilt, but referenced to true ground height — use this one for height-band filters, e.g. `rover_costmap_plugins::SlopeLayer`'s `base_frame` param, so `min_height`/`max_height` don't need a manual per-consumer offset). Never read `base_footprint` for this. Do not fuse IMU orientation into `ekf_filter_node` as a shortcut to get tilt onto `base_footprint` — that breaks the flat-frame guarantee for every other consumer of it. Two `ekf_node` instances publishing TF is fine as long as each owns a distinct edge (see `ekf_filter_node` vs `ekf_tilt_node` above) — it's only a conflict when two nodes claim the *same* edge.