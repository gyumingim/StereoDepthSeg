#!/usr/bin/env python3
"""Offline test input for arloop_replay: an ARKF keyframe dump made from a recorded VIO run plus injected drift.

  python3 make_test_dump.py STREAM.bin POSES.jsonl CONFIG_DIR OUT.bin [--yaw-drift DEG_PER_MIN] [--pos-drift M_PER_MIN]
STREAM.bin  = ov_bridge input stream (PROTOCOL.md records, cam0 = lens0u 640x480), e.g. ~/scanner/android/bench640/stream.bin
POSES.jsonl = ov_bridge output for that stream (pose lines with T_world_cam0), e.g. pc640.jsonl
CONFIG_DIR  = its kalibr_imucam_chain.yaml (intrinsics)
Writes OUT.bin (ARKF records, ARCore conventions: world Y-up, camera OpenGL) and OUT.truth.csv (t_ns, true x y z in the
ARCore world). The drift grows linearly with time (yaw about gravity + horizontal translation), like ARCore drift.
"""
import argparse
import json
import re
import struct

import numpy as np

A = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)   # ARCore world -> Z-up
F = np.diag([1.0, -1.0, -1.0])                               # OpenGL camera <-> OpenCV camera


def rot_to_quat(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = np.copysign(x, R[2, 1] - R[1, 2])
    y = np.copysign(y, R[0, 2] - R[2, 0])
    z = np.copysign(z, R[1, 0] - R[0, 1])
    return np.array([x, y, z, w])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stream"); ap.add_argument("poses"); ap.add_argument("cfg"); ap.add_argument("out")
    ap.add_argument("--yaw-drift", type=float, default=6.0, help="deg per minute")
    ap.add_argument("--pos-drift", type=float, default=0.6, help="m per minute (along x)")
    a = ap.parse_args()
    K = [float(v) for v in re.search(r"intrinsics:\s*\[([^\]]*)\]", open(f"{a.cfg}/kalibr_imucam_chain.yaml").read()).group(1).split(",")]
    poses = {}
    for line in open(a.poses):
        d = json.loads(line)
        if d.get("type") == "pose" and d.get("status") == "tracking" and d.get("T_world_cam0"):
            poses[d["t_ns"]] = np.array(d["T_world_cam0"], float).reshape(4, 4)
    imgs = {}
    with open(a.stream, "rb") as f:
        while True:
            h = f.read(12)
            if len(h) < 12:
                break
            _, typ, n = struct.unpack("<III", h)
            p = f.read(n)
            if typ == 2:
                t, w, hh, nc = struct.unpack("<qIIB3x", p[:20])
                if t in poses:
                    imgs[t] = (w, hh, p[20:20 + w * hh])
    ts = sorted(imgs)
    t0 = ts[0]
    out = open(a.out, "wb")
    truth = open(a.out + ".truth.csv", "w")
    truth.write("t_ns,x,y,z\n")
    last = None
    n = 0
    for t in ts:
        T = poses[t]
        R, p = T[:3, :3], T[:3, 3]
        if last is not None:
            dt = (t - last[0]) * 1e-9
            ang = np.degrees(np.arccos(np.clip((np.trace(last[1].T @ R) - 1) / 2, -1, 1)))
            if dt < 0.25 or (np.linalg.norm(p - last[2]) < 0.10 and ang < 8 and dt < 1.0):
                continue
        last = (t, R, p)
        m = (t - t0) * 1e-9 / 60.0
        yaw = np.radians(a.yaw_drift * m)
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        Rd, pd = Rz @ R, Rz @ p + np.array([a.pos_drift * m, 0, 0])
        R_gl, t_gl = A.T @ Rd @ F, A.T @ pd
        q = rot_to_quat(R_gl)
        w, hh, gray = imgs[t]
        out.write(struct.pack("<Iq", 0x464B5241, t))
        out.write(struct.pack("<7d", *t_gl, *q))
        out.write(struct.pack("<ii", w, hh))
        out.write(struct.pack("<4d", *K))
        out.write(gray)
        out.write(struct.pack("<ii", 0, 0))
        tt = A.T @ p
        truth.write(f"{t},{tt[0]},{tt[1]},{tt[2]}\n")
        n += 1
    print(f"{n} keyframes from {len(ts)} frames, span {(ts[-1] - t0) * 1e-9:.1f} s -> {a.out}")


if __name__ == "__main__":
    main()
