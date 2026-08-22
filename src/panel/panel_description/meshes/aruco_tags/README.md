# Panel ArUco Tag Decals

Textured quads (`marker_<id>.obj` + `.mtl` + `.png`) for the panel's 3
marker mounts in
[`../../urdf/panel_macro.xacro`](../../urdf/panel_macro.xacro)
(`panel_marker_top_left/top_right/bottom_left`), layered as a second
`<visual>` mesh directly on top of each mount's black box.

Named by ArUco **ID**, not by mount — which ID goes on which mount is a
launch-time choice (`marker_id_top_left`/`top_right`/`bottom_left`
xacro:args in `panel_macro.xacro`), not fixed per mesh, since per
competition rules the panel may use any 3 of IDs `{11,13,14,15}` in any
of the 3 mount positions. `panel_perception`'s `panel_pose_fuser_node`
has the matching `marker_id_top_left`/`top_right`/`bottom_left` ROS
parameters — keep both in sync with whatever the real/competition panel
actually uses.

A first attempt applied the PNG as a `<gazebo><material><pbr><albedo_map>`
override directly on the box primitive — this resolved fine (file path
correct) but rendered as flat black in gz-sim; textured meshes are the
reliable path, so this replaced it.

Dictionary `ARUCO_ORIGINAL` (the original ArUco dictionary — per
competition rules, NOT `4X4_50`), matching
`rover_aruco/config/aruco_params.yaml`'s `marker_dict`. Pre-generated
IDs: `11, 13, 14, 15` — the full competition set (any 3 of these 4 may
be mounted on a given panel, in any of its 3 positions).

Regenerate a PNG with (this workspace's opencv is 4.9, whose
`cv2.aruco` API uses `generateImageMarker`, not the older
`drawMarker`/`Dictionary_get`):

```python
import cv2
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
img = cv2.aruco.generateImageMarker(d, marker_id, 400)
cv2.imwrite(f"marker_{marker_id}.png", img)
```

The `.obj`/`.mtl` pair for each ID is a flat 0.04x0.04m quad: a single
face at `y=-0.0011` (just proud of the mount box's front face) spanning
`x,z ∈ [-0.02, 0.02]`, normal facing `-Y` (the panel's front, per the
mount box's own convention), UV-mapped `0..1` straight onto the PNG.
Regenerate the `.obj`/`.mtl` files (not the PNGs) with
`generate_marker_meshes.py` in this directory if the marker size ever
changes, or a new ID needs a mesh added to its `MARKER_IDS` list.
