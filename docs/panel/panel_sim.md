# Panel Simulation

> **Note:** Panel packages are located in `src/panel/`. Before doing anything you need to build them. Run the following command to build only the panel subsystem:
> ```bash
> colcon build --symlink-install --packages-select-regex "^panel_"
> ```

The panel is the ERC equipment/switch task board -- a base plate
(`panel.stl`) with a main switch, a toggle switch and a single-pole circuit
breaker (`panel_main_switch_link`, `panel_toggle_switch_link`,
`panel_breaker_link`) -- modeled as a **passive** object: the arm interacts
with it via contact physics in Gazebo, there is no `ros2_control` hardware
on the panel itself.

> **Note:** the mount position/orientation of each switch on the board in
> `panel_macro.xacro` is a placeholder inferred from the STL bounding boxes,
> not the real hole layout (no source CAD assembly was available). See
> [`panel_description/meshes/README.md`](../../src/panel/panel_description/meshes/README.md)
> for details and verify visually before relying on exact positions.

## Standalone Visualization (verified)

1. **Allow GUI access** (host terminal, not inside Docker):
   ```bash
   xhost +local:docker
   ```
2. **Build and start the container:** see the [Docker](../../README.md#docker) section of the README.
3. **Run the launch file:**
   ```bash
   ros2 launch panel_bringup panel_standalone.launch.py
   ```
   Use the Joint State Publisher GUI sliders to move
   `panel_main_switch_1_joint`, `panel_main_switch_2_joint`,
   `panel_toggle_switch_joint` and `panel_breaker_joint`, and confirm the
   meshes/joint limits look right in RViz.

## Gazebo simulation (not yet verified)

`panel_sim/panel_gazebo.launch.py` (panel alone) and
`rover_sim/sim_gz_full.launch.py` (rover + arm + panel together) exist in
the codebase for physics testing, but neither has been confirmed working
end-to-end yet: every attempt so far in this environment has hit spawn
failures caused by an outdated Gazebo install (server reports **Ignition
Gazebo Fortress v6.18.0**, not the Gazebo Harmonic this project's
[software setup docs](../software/workflows.md) expect). Before relying on
either launch file, check your container's Gazebo version:

```bash
gz sim --version
```

If it doesn't say Harmonic, pull/rebuild a current image (see
[Image Tags](../../README.md#image-tags)) and retest before trusting these
launch files or documenting their behavior here.
