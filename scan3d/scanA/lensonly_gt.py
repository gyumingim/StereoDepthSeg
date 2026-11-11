#!/usr/bin/env python3
"""Lens-only stand-in for a GT folder (redscreen 11-11; used by run_scanner.sh for recordings without GT): REC has no ChArUco GT, but kf_ba/scan_offline/make_reduced read
lens K/dist and frame times from REC/<gt>/gt.npz.  NOT a ground truth:
  t_ns  = frames_cam5.csv t_mid_ns (same as imu_gt_spl of 165506)
  K, dist = the 0.5 dpt app lens json (board calibration only)
  R_bc, p_bc = the VIO live pose of that frame, p shifted +1000 m in z, so the scan_offline alignment is a pure translation
  (scale 1) and its fake board plane (z=0) is never in view (no board scoring).  ok = frame has a VIO tracking pose within 1 ms.
usage: lensonly_gt.py REC VIO_POSES LENS_JSON OUTNAME"""
import csv, gzip, json, sys, time
from pathlib import Path
import numpy as np

rec, vio, lens, name = Path(sys.argv[1]), sys.argv[2], Path(sys.argv[3]), sys.argv[4]
t = np.array([int(r["t_mid_ns"]) for r in csv.DictReader(open(rec / "frames_cam5.csv"))], np.int64)
vt, vT = [], []
for line in gzip.open(vio, "rt"):
    d = json.loads(line)
    if d.get("type") == "pose" and d.get("T_world_cam0") and d.get("status") == "tracking":
        vt.append(int(d["t_ns"])); vT.append(np.array(d["T_world_cam0"], float).reshape(4, 4))
vt = np.array(vt, np.int64); vT = np.array(vT)
idx = np.searchsorted(vt, t).clip(1, len(vt) - 1)
idx = np.where(np.abs(vt[idx - 1] - t) < np.abs(vt[idx] - t), idx - 1, idx)
dt = np.abs(vt[idx] - t); ok = dt < 1e6
R = np.tile(np.eye(3), (len(t), 1, 1)); p = np.zeros((len(t), 3))
R[ok] = vT[idx[ok]][:, :3, :3]; p[ok] = vT[idx[ok]][:, :3, 3] + [0, 0, 1000.0]
L = json.loads(lens.read_text())
board = "/home/system/scanner/StereoDepthSeg/scan3d/config/charuco/board_laptop_5x3.json"
out = rec / name
out.mkdir(exist_ok=True)
np.savez(out / "gt.npz", t_ns=t, R_bc=R, p_bc=p, rms_px=np.zeros(len(t)), n_corners=np.zeros(len(t), np.int64), ok=ok,
         K=np.array(L["K"], float), dist=np.array(L["dist"], float), spline=False, knot=0.0, board_json=board, square_mm=60.4)
info = dict(NOT_GT="lens-only stand-in, see make_lensonly_gt.py", made=time.strftime("%Y-%m-%d %H:%M:%S"), rec=rec.name, vio=vio,
            lens=str(lens), K=L["K"], dist=L["dist"], frames=len(t), ok=int(ok.sum()), n_vio=len(vt),
            dt_ms_median=float(np.median(dt) / 1e6), board_json=board, square_mm=60.4, corner_px_median=None)
(out / "board.json").write_text(json.dumps(info, indent=1))
print(json.dumps(info))
