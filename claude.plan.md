# Rover + arm in one TF tree, runnable together and separately (Gazebo, ROS 2 Jazzy)

Repo: `/home/yurifi/ERC/indomitus-rover-core-ai` (fork of indomitus-rover-core, branch `develop`).
Environment: **ROS 2 Jazzy only**. Build/run inside `distrobox enter ubuntu24` (Ubuntu 24.04, Jazzy, Gazebo Harmonic gz-sim 8, gz_ros2_control and ros2_controllers from apt). Ignore Humble; the repo's CI and Dockerfile still say Humble, so leave them alone.

## Context

The rover (`src/rover_*`) and the arm (`src/arm/arm_*`) are separate subsystems in one repo. Today they are integrated in an ad-hoc way:

- **Combined sim** (`src/rover_sim/launch/sim_gz_full.launch.py`): `rover_description/urdf/rover.urdf.xacro` includes the arm macro when `mount_arm:=true` (lines ~11 and ~147–153, placeholder mount pose `xyz="0.2 0 0.1"` on `main_body_link`). This yields ONE URDF, ONE robot_state_publisher, and ONE controller_manager under `/rover`, so the arm's controllers end up under `/rover`. The controller config `rover_sim/config/controllers_with_arm.yaml` is a hand-merged copy of the rover and arm configs. `arm_macro.xacro` has a `standalone_gz_plugin` arg that exists only to work around this.
- **Arm-only sim** (`src/arm/arm_sim/launch/arm_gazebo.launch.py`): uses `arm_standalone.urdf.xacro` (root `world`). Its gz_ros2_control plugin lives inside the macro (`arm_macro.xacro` ~line 630). Its RSP, controller_manager and move_group all run in the **global** namespace.
- **Arm on hardware** (`arm_bringup/launch/arm.launch.py`) runs under `/arm`. Rover-only sim (`rover_sim/launch/sim_gz.launch.py`) runs under `/rover`.
- `rover_description` depends on `arm_description` (bad direction).

**Goal.**
1. One shared TF tree, filled by several publishers: rover RSP → `base_link … main_body_link`, arm RSP → `main_body_link → arm_mount_link → …`.
2. Separate controller_managers: `/rover/controller_manager` and `/arm/controller_manager`.
3. Three ways to run, built from the same pieces:
   - rover-only sim;
   - arm-only sim;
   - arm mounted on the rover in one Gazebo model.
4. The rover never depends on the arm, and the arm never depends on the rover.

**Scope is deliberately minimal.**
- No package moves or renames beyond the two new packages below.
- No custom hardware plugins.
- No changes to real-hardware drivers.

## Decisions already made (do not revisit)

- **Two `gz_ros2_control` plugins in one Gazebo model**, each with its own `<ros><namespace>` (`/rover`, `/arm`) and its own `<parameters>` yaml. This was verified on Jazzy with a minimal model and gives two independent controller_managers:
  - each one claims only the joints of its own `<ros2_control>` block;
  - commands reached physics (a velocity joint ran at 3.0 rad/s, a position joint went to 0.8 rad);
  - no `--param-file` was needed on spawners.
- **How each controller_manager finds its URDF (Jazzy):** it reads `robot_description` from the **topic in its own namespace**, which is published by that subsystem's robot_state_publisher. The spawned Gazebo model needs the joints and the plugins, but it does **not** need `<ros2_control>` blocks. Each subsystem's RSP URDF carries only that subsystem's `<ros2_control>` block. Extra `<gazebo>` tags inside an RSP URDF are harmless, because nothing spawns from it.
- **Topic and frame names.**
  - `/tf` and `/tf_static` stay global, with no `frame_prefix`.
  - `joint_states` stays namespaced per subsystem (`/rover/joint_states`, `/arm/joint_states`); each RSP reads its own.
  - MoveIt keeps its **arm-only** URDF in this scope (making MoveIt collide with the rover body is a later step).
- **Arm-only sim moves under `/arm`**, the same as arm hardware, so that the arm-only and combined sims use the same arm launch stack.

Verified minimal pattern for the spawned model (adapt it, don't copy it blindly):

```xml
<gazebo>
  <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <ros><namespace>/rover</namespace></ros>
    <parameters>ABS_PATH/rover_controllers.yaml</parameters>
  </plugin>
  <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <ros><namespace>/arm</namespace></ros>
    <parameters>ABS_PATH/arm_controllers.yaml</parameters>
  </plugin>
</gazebo>
```

## Steps

### 0. Make the fork build and run on Jazzy (do this first, keep fixes minimal)
- Run `colcon build --symlink-install` in the distrobox. Fix only what breaks on Jazzy. Known suspects:
  - the moveit_servo executable name (`servo_node_main` in `arm_gazebo.launch.py`/`arm.launch.py`; on Jazzy it is `servo_node`);
  - deprecated ROS 2 APIs in `rover_controller` and the hardware interfaces.
- `src/rover_localization` has `COLCON_IGNORE`/`AMENT_IGNORE`, but `sim_gz_full.launch.py` includes `rover_localization/ekf.launch.py`. Guard or remove that include so the sim doesn't crash. Note that without the EKF and the tilt broadcaster there is no `odom → base_footprint → base_link` in sim, so the TF root will be `base_link`. That is acceptable for this task.
- Check that `ros2 launch rover_sim sim_gz.launch.py` (rover only) and `ros2 launch arm_sim arm_gazebo.launch.py` (arm only) start as they do today. This is the baseline.

### 1. Arm description: take Gazebo and mount concerns out of the macro
File: `src/arm/arm_description/urdf/arm_macro.xacro`.
- Remove the gz_ros2_control `<plugin>` block (~line 630) and the `standalone_gz_plugin` arg from the macro. The `<ros2_control name="ArmSystem">` block stays, with its sim / fake / real branching (`sim` → `gz_ros2_control/GazeboSimSystem`).
- `arm_standalone.urdf.xacro` becomes the arm-only assembly. Under `<xacro:if value="$(arg sim)">` it adds the gz_ros2_control plugin with `<ros><namespace>/arm</namespace>` and `<parameters>$(find arm_sim)/config/ros2_controllers.yaml</parameters>`. Keep the `world` root, because it fixes the model in Gazebo.
- Rule to preserve: arm code and configs reference only `arm_*` frames. `servo.yaml` and `panel_align_node.py` already use `arm_mount_link`.

### 2. Rover description: drop knowledge of the arm
- `src/rover_description/urdf/rover.urdf.xacro`: remove the `mount_arm` include and the `indomitus_arm` block.
- `src/rover_description/urdf/rover.xacro`: remove the `mount_arm` arg. Keep its own `use_sim` gz_ros2_control plugin (namespace `rover_namespace`); it is still needed for rover-only sim.
- `src/rover_description/package.xml`: remove `<exec_depend>arm_description</exec_depend>`.

### 3. New package `src/indomitus_description` (ament_cmake, data only, installs `urdf/`)
- `urdf/mount.xacro`: the **single source** of the arm mount pose on `main_body_link`. Use a macro or properties so both files below use it. Copy the current placeholder `0.2 0 0.1` and add a TODO: the real pose must come from CAD (MR_ARM_ATT), and the placeholder sits inside the body box.
- `urdf/arm_on_rover.urdf.xacro`: root link `main_body_link` (empty), then the arm macro with the mount pose. It also carries the arm `<ros2_control>`, which already comes from the macro. This is the URDF for the `/arm` RSP when the arm is mounted. Its `main_body_link → arm_mount_link …` edges join the rover tree.
- `urdf/indomitus_gz.urdf.xacro`: the **only** file spawned into Gazebo for the combined sim.
  - Include `rover_description/urdf/rover.urdf.xacro` (rover geometry and sensor `<gazebo>` tags). Declare the same xacro args that `rover.xacro` declares, because `rover.urdf.xacro` reads them.
  - Include the arm macro with the mount pose.
  - Add the two gz_ros2_control plugins shown above. Parameters: rover → `$(find rover_sim)/config/controllers.yaml`, arm → `$(find arm_sim)/config/ros2_controllers.yaml`.
  - Add the gz `JointStatePublisher` system that `rover.xacro` adds. Check whether it is actually used (bridge configs) before keeping it.
  - Do **not** add `<ros2_control>` blocks here.
- `package.xml` exec_depends: `rover_description`, `arm_description`, `xacro`.

### 4. Controller configs: one per subsystem, namespace-agnostic
- `src/arm/arm_sim/config/ros2_controllers.yaml` uses bare top-level keys (`controller_manager:`, `indomitus_arm_controller:` …). Change them to `/**/controller_manager:` and `/**/<controller>:` so the file works under `/arm`. `rover_sim/config/controllers.yaml` already uses `/**/`.
- Diff `rover_sim/config/controllers_with_arm.yaml` against the two per-subsystem files. Make sure nothing rover-specific exists only in the merged file (e.g. `diff_bar_effort_controller` / `RockerDiffController`, `hardware_components_initial_state`). Then **delete** `controllers_with_arm.yaml` and `bridge_gz_with_arm_camera.yaml` once their contents are covered (the arm camera bridge should come from the arm side: `arm_sim/config/gz_bridge.yaml`).

### 5. Arm launch stack reusable for arm-only and mounted sim, under `/arm`
Split `src/arm/arm_sim/launch/arm_gazebo.launch.py` (currently a ~430-line monolith in the global namespace) into two files.

**(a) `arm_sim/launch/arm_stack_sim.launch.py`** does **not** start Gazebo and does **not** spawn. It runs everything under `PushRosNamespace('arm')`, pushed once, in this file only:
- the `/arm` RSP (`use_sim_time: true`) from a `description_file` arg: `arm_standalone.urdf.xacro` (arm-only) or `indomitus_description/urdf/arm_on_rover.urdf.xacro` (mounted), plus `sim:=true camera:=… end_effector:=…`;
- controller spawners (existing order and event-handler chaining: JTC, forward controller `--inactive`, gripper), targeting `/arm/controller_manager`;
- move_group and servo, as `arm.launch.py` already does under `/arm`. Reuse its approach: `MoveItConfigsBuilder` with plain-str mappings.

MoveIt's `robot_description` must use the **same** `description_file` as the RSP, so that its root link exists in TF. Arm-only uses `world`; mounted uses `main_body_link`. Planning and servo frames stay `arm_mount_link`.

Watch the existing double-namespace trap documented in `arm.launch.py` (~lines 314–323): `gamepad_servo.launch.py` hardcodes `namespace='arm'`. Never combine that with the pushed namespace.

**(b) `arm_gazebo.launch.py`** becomes a thin arm-only entry point:
- Gazebo world;
- `/clock` bridge plus `arm_sim/config/gz_bridge.yaml`;
- spawn `arm_standalone` (`sim:=true`) **from a file or string, not from a second RSP**;
- include `arm_stack_sim.launch.py` with `description_file:=arm_standalone`;
- keep the optional panel as today, including its `world → panel/world` static TF.

### 6. New package `src/indomitus_bringup` with the combined entry point
`launch/sim_rover_arm.launch.py`. Reuse helpers and patterns from `rover_sim/launch/sim_gz.launch.py`: world selection, `GZ_SIM_RESOURCE_PATH`, generating the bridge yaml into a temp dir, spawn delay. The order matters.

1. Gazebo with the rover world (`rover_sim/worlds/mars_yard.sdf` by default); `/clock` bridge plus rover bridge yaml plus arm camera bridge.
2. `/rover` RSP: `rover_description/launch/robot_state_publisher.launch.py` with `rover.xacro use_sim:=true …`, `use_sim_time: true`, inside `PushRosNamespace('rover')`.
3. `/arm` stack: include `arm_sim/launch/arm_stack_sim.launch.py` with `description_file:=…/arm_on_rover.urdf.xacro`.
4. **Only after both RSPs are up**, spawn `indomitus_gz.urdf.xacro` (rendered to a temp file or `-string`). Each gz_ros2_control plugin blocks Gazebo until `robot_description` appears on its namespace topic.
   - **Never** start a robot_state_publisher for the full Gazebo model: that would duplicate TF edges.
5. Rover controllers: include `rover_bringup/launch/control.launch.py` with `use_sim:=true` inside `PushRosNamespace('rover')`, with the same controller list as `sim_gz.launch.py`. Its spawners resolve to `/rover/controller_manager`. Then include the twist_mux and drive_power launches, as `sim_gz_full.launch.py` does today.
6. Optional `spawn_panel` arg: port the panel block from `sim_gz_full.launch.py` unchanged.

Then:
- **delete** `rover_sim/launch/sim_gz_full.launch.py` (its role moves here), and update docs that reference it (`src/rover_sim/README.md`, `docs/panel/panel_sim.md`, `docs/software/tf_ownership.md`);
- `indomitus_bringup/package.xml` exec_depends: `indomitus_description`, `rover_sim`, `rover_bringup`, `arm_sim`, `ros_gz_sim`, `ros_gz_bridge`.

### 7. Docs
- `docs/software/tf_ownership.md`: add rows for the `/arm` RSP (`main_body_link → arm_*` when mounted, `world → arm_*` standalone), and state that two RSPs share `/tf`.
- Add a short section on the three entry points (rover-only, arm-only, combined) and the rules:
  - the namespace is pushed only in entry points;
  - modules use relative names;
  - joint names must be unique robot-wide (arm joints are `arm_*`);
  - controller yaml keys use `/**/`.

## Pitfalls

- **Launch argument leakage.** `IncludeLaunchDescription` applies its `launch_arguments` with `SetLaunchConfiguration` in the **enclosing** scope, so they leak to later actions in the same group. Wrap includes that pass overriding values in their own `GroupAction`.
- **Startup order.** Each gz_ros2_control plugin waits (blocking Gazebo) for its namespace's `robot_description`. Start the RSPs first.
- **Duplicate TF publishers.** Exactly two RSPs in the combined sim. `ros2 topic info /tf -v` must not show a third robot_state_publisher.
- **No new cross-dependencies.** Nothing in `rover_*` may reference `arm_*` or `indomitus_*`, and nothing in `arm_*` may reference `rover_*`.
- **Real hardware is out of scope.** Don't touch `rover_bringup/launch/rover.launch.py`, `arm_bringup/launch/arm.launch.py` (beyond the Jazzy servo executable fix, if needed) or the hardware interfaces. A later follow-up would switch arm hardware to `arm_on_rover` when mounted.

## Verification (in the distrobox, after `colcon build --symlink-install && source install/setup.bash`)

1. **URDFs render.** `xacro` renders `rover.xacro use_sim:=true`, `arm_standalone.urdf.xacro sim:=true`, `arm_on_rover.urdf.xacro sim:=true` and `indomitus_gz.urdf.xacro`. `check_urdf` passes on each. `grep -c gz_ros2_control-system` on the `indomitus_gz` output gives 2.
2. **Rover-only:** `ros2 launch rover_sim sim_gz.launch.py`.
   - `ros2 control list_controllers -c /rover/controller_manager` shows all active.
   - Publishing a `geometry_msgs/Twist` on `/rover/cmd_vel` (or the twist_mux input it uses) moves the rover.
3. **Arm-only:** `ros2 launch arm_sim arm_gazebo.launch.py`.
   - `ros2 control list_controllers -c /arm/controller_manager` shows the JTC active and the forward controller inactive.
   - A MoveIt plan & execute (or a servo command) moves the arm.
   - `ros2 node list` shows no `/arm/arm/...`.
4. **Combined:** `ros2 launch indomitus_bringup sim_rover_arm.launch.py`.
   - Both controller_managers exist. `ros2 control list_hardware_interfaces -c /rover/controller_manager` shows only rover joints, and `-c /arm/...` only `arm_*` joints.
   - `ros2 run tf2_tools view_frames` shows ONE tree, `base_link → … → main_body_link → arm_mount_link → … → arm_tcp_link`.
   - Rover drives, arm moves (MoveIt plan in `/arm`), and both work at the same time.
   - No `/rover/rover` or `/arm/arm` in `ros2 node list`.
5. **Dependency rule.** `grep -rn "arm_" src/rover_*/package.xml src/rover_*/launch` and `grep -rn "rover_" src/arm/*/package.xml src/arm/*/launch` return nothing new.
