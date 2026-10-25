#!/usr/bin/env python3
"""Raw 1920x1080 replay stream for the phone bridge (like the arlat app feeds it: raw Y frames, bridge --remap).

  make_raw_stream.py REC OUT.bin [--secs 75] [--fps 30] [--start-off 0]
Frames: picked by time from the 240 fps recording (vio_bench_hs.fed_selection), cam5.mp4 decoded -> gray 1920x1080,
NOT undistorted (the bridge remaps with the phone remap file). Time stamps: t_mid - align dt (camera time on the IMU
clock, like vio_bench_hs) -> use the config with timeshift_cam_imu 0. IMU: raw types 16/35 (bench --imu-raw), accel
interpolated to the gyro time, no gyro rebias (the app does none). Order like the bench: IMU up to t_cam + 5 ms, then
the frame. Writes OUT.bin + OUT.json (frame t_ns list, dt, source).
"""
import argparse
import csv
import json
import struct
import sys
from pathlib import Path

import cv2
import numpy as np

SCAN3D = Path("/home/system/scanner/StereoDepthSeg/scan3d")
sys.path.insert(0, str(SCAN3D / "imu_learn"))
sys.path.insert(0, str(SCAN3D))
from vio_bench_hs import fed_selection  # noqa: E402
from imu_gt import read_imu  # noqa: E402

HDR = struct.Struct("<III")
MAGIC = 0x4F564252

ap = argparse.ArgumentParser()
ap.add_argument("rec"); ap.add_argument("out")
ap.add_argument("--secs", type=float, default=75); ap.add_argument("--fps", type=float, default=30)
ap.add_argument("--start-off", type=float, default=0.0)
a = ap.parse_args()
rec = Path(a.rec)
meta = json.load(open(rec / "meta.json"))
rows = list(csv.DictReader(open(rec / "frames_cam5.csv")))
al = json.load(open(rec / "imu_gt" / "align.json"))
dt_ns = int(al["dt_ns"])
fs = fed_selection(rec, meta, rows, a.fps, a.start_off, a.secs)
t_frames, sel, t_end = fs["t_frames"], fs["sel"], fs["t_end"]
acc, gyr = read_imu(rec / "imu.csv", raw=True)
a_i = np.stack([np.interp(gyr[:, 0], acc[:, 0], acc[:, 1 + j]) for j in range(3)], 1)
g_i = gyr[:, 1:4].copy()
t_imu = gyr[:, 0].astype(np.int64)
selected = np.zeros(len(t_frames), bool); selected[sel] = True
ii = int(np.searchsorted(t_imu, t_frames[sel[0]] - dt_ns - 2_000_000_000))
cap = cv2.VideoCapture(str(rec / "cam5.mp4"))
fed = []
with open(a.out, "wb") as f:
    for fi in range(len(rows)):
        if not cap.grab():
            break
        tc = t_frames[fi]
        if tc > t_end:
            break
        if not selected[fi]:
            continue
        tci = int(tc - dt_ns)
        while ii < len(t_imu) and t_imu[ii] <= tci + 5_000_000:
            f.write(HDR.pack(MAGIC, 1, 56) + struct.pack("<q3d3d", int(t_imu[ii]), *g_i[ii], *a_i[ii]))
            ii += 1
        _, im = cap.retrieve()
        g = np.ascontiguousarray(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY))
        h, w = g.shape
        body = struct.pack("<qIIB3x", tci, w, h, 1) + g.tobytes()
        f.write(HDR.pack(MAGIC, 2, len(body)) + body)
        fed.append(tci)
        if len(fed) % 300 == 0:
            print("frames", len(fed), flush=True)
    for _ in range(200):
        if ii >= len(t_imu):
            break
        f.write(HDR.pack(MAGIC, 1, 56) + struct.pack("<q3d3d", int(t_imu[ii]), *g_i[ii], *a_i[ii]))
        ii += 1
json.dump(dict(rec=rec.name, secs=a.secs, fps=a.fps, start_off=a.start_off, dt_ns=dt_ns, n_frames=len(fed),
               size=[int(meta["width"]), int(meta["height"])], imu="raw 16/35, no rebias", t_cam_ns=fed),
          open(a.out.replace(".bin", ".json"), "w"))
print("done", len(fed), "frames", (fed[-1] - fed[0]) / 1e9, "s")
