# TF Ownership

This document lists every transform published in the system and which single
node is responsible for broadcasting it. **Each transform must have exactly
one publisher.** If you add a new node or controller that could plausibly
publish TF, check this table first and update it if ownership changes.

## Current tree

```
map            (not yet published — reserved for future Nav2/AMCL)
 └── odom
      └── base_link
           ├── fl_wheel_mount_link -> fl_wheel_link
           ├── fr_wheel_mount_link -> fr_wheel_link
           ├── bl_wheel_mount_link -> bl_wheel_link
           ├── br_wheel_mount_link -> br_wheel_link
           ├── l_rocker_link
           ├── r_rocker_link
           ├── suspension_base_axii_link
           └── main_body_link
```

## Ownership table

| Transform | Owning Node | Package | Config |
|---|---|---|---|
| `odom -> base_link` | `ekf_node` (`robot_localization`) | `rover_localization` | `config/ekf_filter.yaml` — `publish_tf: true`, `world_frame: odom` |
| `base_link -> *` (all links derived from URDF, including the arm when `mount_arm:=true`) | `robot_state_publisher` | `rover_description` | driven by `/joint_states` + URDF kinematics; launched via `robot_state_publisher.launch.py` |
| `map -> odom` | *not yet implemented* | — | Reserved for a future localization node (e.g. Nav2's AMCL or a second EKF instance). Do **not** enable `publish_tf` on more than one node targeting this transform. |
| `world -> panel/panel_base_link -> *` (panel base + switch/breaker links) | `robot_state_publisher` | `panel_description` | driven by `/joint_states` + `panel_standalone.urdf.xacro`; verified via `panel_bringup/panel_standalone.launch.py` (there, unprefixed: `world -> panel_base_link`, since standalone has no namespace to collide with). When spawned inside `rover_sim/sim_gz_full.launch.py` it runs under a `panel` namespace with `frame_prefix:='panel/'` set (so frame IDs are actually `panel/panel_base_link` etc., not just the topics) and `tf`/`tf_static` remapped back onto the global `/tf`/`/tf_static` topics, so the prefixed frames land on the one shared TF tree. The `world -> panel/panel_base_link` pose is generated from the same `panel_x`/`panel_y`/`panel_z`/`panel_yaw` launch args passed to the Gazebo spawn, so the two stay in sync -- **not yet verified end-to-end in Gazebo** (blocked on the Gazebo version issue in [`panel_sim.md`](../panel/panel_sim.md)), but the frame-naming and pose-sync code path is in place. |

The panel's TF tree is **not** connected to the rover's (`panel_base_link`
has no common ancestor with `base_link`). This is intentional: on the real
field the panel is a stationary object the rover drives up to, not a rigid
part of it, so its pose relative to the rover is something perception
(camera/fiducials) will need to establish at runtime -- not a fixed static
transform baked into the URDF.

## Non-owners (explicitly verified)

- **`odometry_controller`** (`rover_controller`) — publishes `nav_msgs/Odometry`
  on `/odometry/wheels` only. It does **not** broadcast any TF transform —
  confirmed by inspecting `publish_odom()`: no `tf2_ros::TransformBroadcaster`
  exists in this controller. Its `/odometry/wheels` topic is consumed as the
  `odom0` input to `ekf_node`, not published to TF directly. This separation
  is intentional — do not add a TF broadcast here, since that would duplicate
  `ekf_node`'s ownership of `odom -> base_link`.

## Rules for future changes

1. Before adding any new node that fuses odometry, publishes pose, or
   integrates sensor data into a frame already listed above, check this table
   — do not enable a second `publish_tf`-style flag for an already-owned
   transform.
2. When integrating Nav2: AMCL (or an equivalent global localization node)
   will own `map -> odom`. Confirm no other node is configured to publish it
   before enabling.
3. If ownership of any transform changes, update this table in the same PR.
