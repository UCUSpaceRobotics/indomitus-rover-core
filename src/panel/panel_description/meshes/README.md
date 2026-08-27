# Panel Meshes

- `panel.stl` — base plate / board (330 x 72 x 450 mm)
- `main_switch.stl` — main toggle/rotary switch (~65 x 65 x 29 mm)
- `switch.stl` — toggle switch (~38 x 46 x 29 mm)
- `MCB_1P.stl` — single-pole miniature circuit breaker (~14 x 16 x 16 mm)
- `aruco_tags/panel_marker_<suffix>.obj` + `.mtl` + `.png` — textured 0.04x0.04m
  decal quads for the 3 ArUco marker mounts (see `aruco_tags/README.md`),
  layered as a second `<visual>` on top of each `panel_marker_*_link`'s
  black mount box.

Used by [`../urdf/panel_macro.xacro`](../urdf/panel_macro.xacro) (link names
`panel_base_link`, `panel_main_switch_link`, `panel_toggle_switch_link`,
`panel_breaker_link`). Collision boxes there were sized from each mesh's own
bounding box, but the mount positions/orientations on the board are a
placeholder — there was no source CAD assembly to read exact hole positions
from. Verify the layout visually (`ros2 launch panel_bringup
panel_standalone.launch.py`) and adjust the `<origin>` of each
`panel_*_joint` to match the real board.
