"""2026-10-24: motion statistics of std60 board recordings from the spline GT (camera centre path, rotation rate) and the
IMU (gyro norm, accel minus gravity), to see how much each recording excites scale. Prints one line per recording."""
import csv
import sys

import numpy as np

for n in sys.argv[1:]:
    g = np.load(f"recordings/{n}/imu_gt_spl/gt.npz", allow_pickle=True)
    ok = g["ok"].astype(bool)
    t = g["t_ns"][ok] / 1e9
    p = g["p_bc"][ok].reshape(-1, 3)
    path = float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
    ext = np.ptp(p, axis=0)
    rows = list(csv.DictReader(open(f"recordings/{n}/imu.csv")))
    gy = np.array([[float(r["v0"]), float(r["v1"]), float(r["v2"])] for r in rows if r["type"] == "16"])
    ac = np.array([[float(r["v0"]), float(r["v1"]), float(r["v2"])] for r in rows if r["type"] == "35"])
    w = np.linalg.norm(gy, axis=1)
    a = np.abs(np.linalg.norm(ac, axis=1) - np.median(np.linalg.norm(ac, axis=1)))
    print(n, dict(gt_dur_s=round(t[-1] - t[0], 1), cam_path_m=round(path, 3), extent_cm=np.round(ext * 100, 1).tolist(),
                  gyro_p50=round(float(np.median(w)), 3), gyro_p90=round(float(np.percentile(w, 90)), 3),
                  acc_dev_p90=round(float(np.percentile(a, 90)), 3)))
