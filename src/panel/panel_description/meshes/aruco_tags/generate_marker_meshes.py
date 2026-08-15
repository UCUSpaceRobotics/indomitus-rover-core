#!/usr/bin/env python3
"""Regenerate the flat textured-quad OBJ+MTL for each panel ArUco marker.

Run from this directory. Only needs to be re-run if the marker size
(0.04x0.04m, matching panel_macro.xacro's mount box) or the set of
marker names changes — the PNG textures themselves are separate, see
this directory's README.md for how to regenerate those.
"""
import os

SIZE = 0.02  # half-width/height of the 0.04x0.04m marker, meters
NAMES = ["top_left", "top_right", "bottom_left"]

OBJ_TEMPLATE = """# Flat textured quad for ArUco marker decal
mtllib panel_marker_{name}.mtl
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
map_Kd panel_marker_{name}.png
"""

if __name__ == "__main__":
    out_dir = os.path.dirname(os.path.abspath(__file__))
    for name in NAMES:
        obj_path = os.path.join(out_dir, f"panel_marker_{name}.obj")
        mtl_path = os.path.join(out_dir, f"panel_marker_{name}.mtl")
        with open(obj_path, "w") as f:
            f.write(OBJ_TEMPLATE.format(name=name, s=SIZE))
        with open(mtl_path, "w") as f:
            f.write(MTL_TEMPLATE.format(name=name))
        print("wrote", obj_path, mtl_path)
