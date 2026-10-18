#!/usr/bin/env python3
"""Paint a 3D-consistent ArUco marker into an ARKF test dump (make_test_dump.py output) for the marker loop test.

  venv_ffs/bin/python add_test_marker.py IN.bin OUT.bin [--id 0] [--len 0.15] [--dist 0.8]
The marker (DICT_4X4_50) is placed flat, facing the first keyframe's camera, --dist m in front of it, using the TRUE
poses (IN.bin.truth.csv is not needed: the drift of make_test_dump starts at 0, so the first keyframe is true; the
others are un-drifted with --yaw-drift/--pos-drift, which must match the values used for IN.bin).
"""
import argparse
import struct

import cv2
import numpy as np

A = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)
F = np.diag([1.0, -1.0, -1.0])


def quat_to_rot(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def read(path):
    out = []
    with open(path, "rb") as f:
        while True:
            h = f.read(4)
            if len(h) < 4:
                break
            t, = struct.unpack("<q", f.read(8))
            p = struct.unpack("<7d", f.read(56))
            w, hh = struct.unpack("<ii", f.read(8))
            K = struct.unpack("<4d", f.read(32))
            g = np.frombuffer(f.read(w * hh), np.uint8).reshape(hh, w).copy()
            dw, dh = struct.unpack("<ii", f.read(8))
            d = f.read(2 * dw * dh)
            out.append(dict(t=t, p=p, w=w, h=hh, K=K, g=g, dw=dw, dh=dh, d=d))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("inp"); ap.add_argument("out")
    ap.add_argument("--id", type=int, default=0); ap.add_argument("--len", type=float, default=0.15)
    ap.add_argument("--dist", type=float, default=0.8)
    ap.add_argument("--yaw-drift", type=float, default=12.0); ap.add_argument("--pos-drift", type=float, default=1.0)
    a = ap.parse_args()
    kfs = read(a.inp)
    t0 = kfs[0]["t"]

    def true_pose(k):   # undo make_test_dump drift -> camera->world (Z-up world, OpenCV camera)
        x, y, z, qx, qy, qz, qw = k["p"]
        R = A @ quat_to_rot(qx, qy, qz, qw) @ F
        t = A @ np.array([x, y, z])
        m = (k["t"] - t0) * 1e-9 / 60.0
        yaw = np.radians(a.yaw_drift * m)
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        return Rz.T @ R, Rz.T @ (t - np.array([a.pos_drift * m, 0, 0]))

    L = a.len
    corners_m = np.array([[-L / 2, L / 2, 0], [L / 2, L / 2, 0], [L / 2, -L / 2, 0], [-L / 2, -L / 2, 0]])
    poses = [true_pose(k) for k in kfs]

    def place(i):   # marker dist m ahead of keyframe i, facing it
        Ri, ti = poses[i]
        return (Ri @ np.diag([1.0, -1.0, -1.0]) @ corners_m.T).T + ti + Ri @ np.array([0, 0, a.dist])

    def visible(Xw, k, R, t):
        xc = (R.T @ (Xw - t).T).T
        if (xc[:, 2] < 0.15).any():
            return None
        fx, fy, cx, cy = k["K"]
        uv = np.float32(np.c_[fx * xc[:, 0] / xc[:, 2] + cx, fy * xc[:, 1] / xc[:, 2] + cy])
        if (uv < 5).any() or (uv[:, 0] > k["w"] - 5).any() or (uv[:, 1] > k["h"] - 5).any():
            return None
        if cv2.arcLength(uv, True) < 4 * 35:
            return None
        return uv

    # anchor keyframe whose marker is seen both early and >= 8 s later (a marker loop), most sightings
    best, best_score = 0, -1
    for i in range(len(kfs)):
        Xw = place(i)
        vis = [j for j, k in enumerate(kfs) if visible(Xw, k, *poses[j]) is not None]
        span = (kfs[vis[-1]]["t"] - kfs[vis[0]]["t"]) * 1e-9 if vis else 0
        score = len(vis) if span >= 8 else -1
        if score > best_score:
            best, best_score = i, score
    Xw = place(best)
    print(f"marker anchored at keyframe {best}, visible in {best_score} keyframes")
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    S = 300
    mk = cv2.aruco.generateImageMarker(dic, a.id, S)
    mk = cv2.copyMakeBorder(mk, 0, 0, 0, 0, cv2.BORDER_CONSTANT)
    src = np.float32([[0, 0], [S, 0], [S, S], [0, S]])
    seen = 0
    for k, (R, t) in zip(kfs, poses):
        uv = visible(Xw, k, R, t)
        if uv is None:
            continue
        # white quiet zone around the marker so the detector finds the border
        ctr = uv.mean(0)
        quiet = np.float32(ctr + (uv - ctr) * 1.4)
        cv2.fillConvexPoly(k["g"], quiet.astype(np.int32), 255)
        H = cv2.getPerspectiveTransform(src, uv)
        warped = cv2.warpPerspective(mk, H, (k["w"], k["h"]), flags=cv2.INTER_LINEAR)
        mask = cv2.warpPerspective(np.full_like(mk, 255), H, (k["w"], k["h"]))
        k["g"][mask > 128] = warped[mask > 128]
        seen += 1
    with open(a.out, "wb") as f:
        for k in kfs:
            f.write(struct.pack("<Iq", 0x464B5241, k["t"]))
            f.write(struct.pack("<7d", *k["p"]))
            f.write(struct.pack("<ii", k["w"], k["h"]))
            f.write(struct.pack("<4d", *k["K"]))
            f.write(k["g"].tobytes())
            f.write(struct.pack("<ii", k["dw"], k["dh"]))
            f.write(k["d"])
    print(f"marker {a.id} ({L} m) painted into {seen}/{len(kfs)} keyframes -> {a.out}")


if __name__ == "__main__":
    main()
