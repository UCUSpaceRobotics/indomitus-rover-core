# Panel ArUco Tag Decals

Textured quads (`panel_marker_<suffix>.obj` + `.mtl` + `.png`) for the
three marker mounts in
[`../../urdf/panel_macro.xacro`](../../urdf/panel_macro.xacro)
(`panel_marker_top_left/top_right/bottom_left`), layered as a second
`<visual>` mesh directly on top of each mount's black box.

A first attempt applied the PNG as a `<gazebo><material><pbr><albedo_map>`
override directly on the box primitive — this resolved fine (file path
correct) but rendered as flat black in gz-sim; textured meshes are the
reliable path, so this replaced it.

Dictionary `4X4_50`, matching `rover_aruco/config/aruco_params.yaml`'s
`marker_dict`. IDs: `top_left=20`, `top_right=21`, `bottom_left=22` —
chosen arbitrarily, nothing was previously assigned. If real hardware
already uses different IDs, regenerate to match and update
`panel_perception`'s marker-ID parameters to stay in sync.

Regenerate the PNGs with (OpenCV 4.5's legacy API — this workspace's
opencv is 4.5.4, predating `generateImageMarker()`):

```python
import cv2
d = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
img = cv2.aruco.drawMarker(d, marker_id, 400)
cv2.imwrite(f"panel_marker_{suffix}.png", img)
```

The `.obj`/`.mtl` pair for each marker is a flat 0.04x0.04m quad: a
single face at `y=-0.0011` (just proud of the mount box's front face)
spanning `x,z ∈ [-0.02, 0.02]`, normal facing `-Y` (the panel's front,
per the mount box's own convention), UV-mapped `0..1` straight onto the
PNG. Regenerate with `generate_marker_meshes.py` in this directory if
the marker size ever changes.
