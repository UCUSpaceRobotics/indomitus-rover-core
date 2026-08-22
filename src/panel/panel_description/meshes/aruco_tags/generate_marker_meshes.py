#!/usr/bin/env python3
"""Regenerate the flat textured-quad OBJ+MTL for each panel ArUco marker ID.

Run from this directory. Meshes are named by ArUco ID (marker_<id>.obj/
.mtl), not by physical mount (top_left/etc.) — panel_macro.xacro picks
which ID goes on which mount at launch time (marker_id_top_left/
top_right/bottom_left xacro:args), so the mesh itself only needs to know
its ID, never its mount. Only needs to be re-run if the marker size
(0.04x0.04m, matching panel_macro.xacro's mount box) changes, or a new
ID is added to MARKER_IDS — the PNG textures themselves are separate,
see this directory's README.md for how to (re)generate those.
"""
import os

SIZE = 0.02  # half-width/height of the 0.04x0.04m marker, meters
# The competition's allowed marker ID set — see panel_macro.xacro's
# marker_id_top_left/top_right/bottom_left xacro:args for how any 3 of
# these get assigned to the panel's 3 physical mounts at launch time.
MARKER_IDS = [11, 13, 14, 15]

OBJ_TEMPLATE = """# Flat textured quad for ArUco marker decal
mtllib marker_{marker_id}.mtl
usemtl marker

v -{s} -0.0011 -{s}
v  {s} -0.0011 -{s}
v  {s} -0.0011  {s}
v -{s} -0.0011  {s}

vt 0 0
vt 1 0
vt 1 1
vt 0 1

vn 0 -1 0

f 1/1/1 2/2/1 3/3/1
f 1/1/1 3/3/1 4/4/1
"""

MTL_TEMPLATE = """newmtl marker
Ka 1.0 1.0 1.0
Kd 1.0 1.0 1.0
Ks 0.0 0.0 0.0
d 1.0
illum 1
map_Kd marker_{marker_id}.png
"""

if __name__ == "__main__":
    out_dir = os.path.dirname(os.path.abspath(__file__))
    for marker_id in MARKER_IDS:
        obj_path = os.path.join(out_dir, f"marker_{marker_id}.obj")
        mtl_path = os.path.join(out_dir, f"marker_{marker_id}.mtl")
        with open(obj_path, "w") as f:
            f.write(OBJ_TEMPLATE.format(marker_id=marker_id, s=SIZE))
        with open(mtl_path, "w") as f:
            f.write(MTL_TEMPLATE.format(marker_id=marker_id))
        print("wrote", obj_path, mtl_path)
