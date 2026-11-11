#!/usr/bin/env python3
"""redscreen 11-11: mesh for the PC viewer, in the frame of the measured red rectangle (viewer draws the 'board' outline = the
REFERENCE red area 302.0 x 188.75 mm at z=0, x right along the top edge, y down along the left edge, origin = top-left).
Rigid transform only (no rescale): a red patch larger than the outline = scanner too big.
The scanner mesh was written in the lens-only stand-in world = BA world + (0, 0, 1000 m) (scan_offline alignment to the stand-in).
usage: export_viewer.py MESH.ply red_measure.json OUT.ply BA_POSES VIO_POSES"""
import json, sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as Rot
sys.path.insert(0, str(Path(__file__).resolve().parent))
import measure_mesh as MM

import gzip
mesh, mj, outp, bap, viop = sys.argv[1:6]
X, C, F = MM.read_ply(mesh)
# undo scan_offline's rigid alignment BA -> stand-in (VIO + 1000 m z), recomputed exactly as scan_offline does it
ba = [json.loads(l) for l in gzip.open(bap, "rt")]
vio = {int(d["t_ns"]): np.array(d["T_world_cam0"]).reshape(4, 4) for d in map(json.loads, gzip.open(viop, "rt"))
       if d.get("type") == "pose" and d.get("T_world_cam0") and d.get("status") == "tracking"}
vt = np.array(sorted(vio)); A, B = [], []
for d in ba:
    i = int(np.argmin(np.abs(vt - int(d["t_ns"]))))
    if abs(vt[i] - int(d["t_ns"])) < 1e6:
        A.append(np.array(d["T_world_cam0"]).reshape(4, 4)[:3, 3]); B.append(vio[vt[i]][:3, 3] + [0, 0, 1000.0])
A, B = np.array(A), np.array(B); ma, mb = A.mean(0), B.mean(0)
U, S_, Vt = np.linalg.svd((B - mb).T @ (A - ma)); D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]); Ra = U @ D @ Vt; ta = mb - Ra @ ma
X = (X - ta) @ Ra                                   # stand-in world -> BA world
print("undo alignment: rot deg", np.degrees(np.arccos(np.clip((np.trace(Ra) - 1) / 2, -1, 1))), "t", ta)
x = json.loads(Path(mj).read_text())["base_B_x"]
R = Rot.from_rotvec(x[:3]).as_matrix(); c = np.array(x[3:6]); W, H = x[6], x[7]
# rectangle local: +x right (TR - TL), +y up (TL - BL) in measure_red.ECORN; viewer: origin TL, y down
TL = c + R @ np.array([-W / 2, H / 2, 0])
ex, ey_up = R[:, 0], R[:, 1]
ez = np.cross(ex, -ey_up)
M = np.stack([ex, -ey_up, ez], 0)                  # world -> screen frame rows
Y = (X - TL) @ M.T
hdr = (f"ply\nformat binary_little_endian 1.0\ncomment redscreen 11-11 202906 in red-rectangle frame (TL origin, mm-free metres)\n"
       f"element vertex {len(Y)}\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n"
       f"element face {len(F)}\nproperty list uchar int vertex_indices\nend_header\n")
vt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("r", "u1"), ("g", "u1"), ("b", "u1")])
V = np.empty(len(Y), vt); V["x"], V["y"], V["z"] = Y[:, 0], Y[:, 1], Y[:, 2]; V["r"], V["g"], V["b"] = C[:, 0], C[:, 1], C[:, 2]
ft = np.dtype([("n", "u1"), ("i", "<i4", 3)]); FF = np.empty(len(F), ft); FF["n"] = 3; FF["i"] = F
Path(outp).write_bytes(hdr.encode() + V.tobytes() + FF.tobytes())
print("wrote", outp, len(Y), "vertices; measured W H mm", W * 1e3, H * 1e3)
