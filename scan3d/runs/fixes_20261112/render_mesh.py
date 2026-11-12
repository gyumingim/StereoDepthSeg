#!/usr/bin/env python3
"""fixes 11-12: headless point-splat render of mesh.ply files (numpy z-buffer, no window, no GPU).
Usage: render_mesh.py OUT.png MESH.ply[:label] [MESH2.ply[:label] ...] [--views 3]
One row per mesh, one column per view. Same cameras for every mesh (fit to the union of their bounding boxes,
camera placed like the scanning phone: from the mean keyframe side, here simply -z of the board frame + yaw/pitch)."""
import argparse
import sys

import cv2
import numpy as np


def read_ply(p):
    with open(p, "rb") as f:
        hdr = b""
        while not hdr.endswith(b"end_header\n"):
            hdr += f.readline()
        h = hdr.decode()
        nv = int(h.split("element vertex ")[1].split()[0])
        props = [l.split()[-1] for l in h.split("element vertex")[1].split("element")[0].splitlines() if l.startswith("property")]
        types = [l.split()[1] for l in h.split("element vertex")[1].split("element")[0].splitlines() if l.startswith("property")]
        tmap = {"float": "<f4", "uchar": "u1", "double": "<f8", "int": "<i4"}
        dt = np.dtype([(n, tmap[t]) for n, t in zip(props, types)])
        v = np.frombuffer(f.read(nv * dt.itemsize), dt, nv)
    X = np.stack([v["x"], v["y"], v["z"]], 1).astype(np.float64)
    C = np.stack([v["red"], v["green"], v["blue"]], 1) if "red" in props else np.full((nv, 3), 200, np.uint8)
    return X, C


def look(eye, tgt, up=(0, 0, 1)):
    z = tgt - eye; z /= np.linalg.norm(z)
    x = np.cross(z, up); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack([x, y, z])


def render(X, C, R, eye, f, W, H):
    pc = (X - eye) @ R.T
    ok = pc[:, 2] > 0.05
    pc, Cc = pc[ok], C[ok]
    u = (pc[:, 0] / pc[:, 2] * f + W / 2).astype(int); v = (pc[:, 1] / pc[:, 2] * f + H / 2).astype(int)
    img = np.full((H, W, 3), 255, np.uint8); zb = np.full((H, W), np.inf)
    o = np.argsort(-pc[:, 2])                       # far first, near overwrites
    for du in (0, 1):
        for dv in (0, 1):
            uu, vv = u[o] + du, v[o] + dv
            m = (uu >= 0) & (uu < W) & (vv >= 0) & (vv < H)
            img[vv[m], uu[m]] = Cc[o][m][:, ::-1]
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("meshes", nargs="+")
    ap.add_argument("--W", type=int, default=640)
    ap.add_argument("--H", type=int, default=480)
    a = ap.parse_args()
    ms = []
    for s in a.meshes:
        p, _, lab = s.partition(":")
        X, C = read_ply(p)
        ms.append((X, C, lab or p))
    allX = np.concatenate([m[0] for m in ms])
    lo, hi = np.percentile(allX, 1, 0), np.percentile(allX, 99, 0)
    ctr = (lo + hi) / 2; rad = float(np.linalg.norm(hi - lo)) / 2
    # board frame: board plane z = 0, the camera looked at it from z < 0 (scan_offline world)
    views = [("front", np.array([0.0, 0.0, -1.0])), ("left-up", np.array([-0.6, -0.4, -0.7])), ("right-down", np.array([0.6, 0.5, -0.6]))]
    rows = []
    for X, C, lab in ms:
        tiles = []
        for name, d in views:
            d = d / np.linalg.norm(d)
            eye = ctr + d * rad * 2.2
            R = look(eye, ctr, up=(0, -1, 0))
            t = render(X, C, R, eye, a.W * 0.9, a.W, a.H)
            cv2.putText(t, f"{lab} | {name} | {len(X)} v", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 200), 2)
            tiles.append(t)
        rows.append(np.hstack(tiles))
    cv2.imwrite(a.out, np.vstack(rows))
    print("wrote", a.out)


if __name__ == "__main__":
    sys.exit(main())
