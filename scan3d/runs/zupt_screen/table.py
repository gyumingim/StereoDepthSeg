#!/usr/bin/env python3
"""Print compact tables from runs/zupt_screen/screen.json + per_frame.pkl."""
import json, pickle
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
S = json.load(open(ROOT / "runs/zupt_screen/screen.json"))
P = pickle.load(open(ROOT / "runs/zupt_screen/per_frame.pkl", "rb"))
print("rec | runs | frames | imu | still% | gt frames | still disp p10/50/90 | spread med/p95 (multi-run frames)")
for rec, R in S["recs"].items():
    print(rec[3:], R["n_runs"], R["n_frames"], R["imu"][:6], R["frac_imu_still"], R["gt_frames"], R["still_disp_pct"],
          R["disp_spread_med"], R["disp_spread_p95"])
print()
print("rec | thr | NEW pass: GT>2cm/s (q) | IMU-move | IMU-move&GT>2 | still gain (still pass / still n) | base pass GT>2 | w_med vmax")
for rec, R in S["recs"].items():
    for th, x in R["thr"].items():
        print(rec[3:], th, f'{x["new_gtmove"]}({x["new_gtmove_q"]})', x["new_imumove"], x["new_imumove_and_gtmove"],
              f'{x["new_still_gain"]} ({x["still_pass"]}/{x["still_n"]})', x["pass_gtmove"], x["new_imumove_w_med"],
              x["new_gtmove_v_max"])
# spread in still vs moving and near threshold
print()
print("spread of disparity across runs for frames with median disp in [0.3,1.5] (multi-run):")
for rec, d in P.items():
    m = (d["nruns"] > 1) & (d["disp"] > 0.3) & (d["disp"] < 1.5)
    if m.sum() < 10:
        continue
    print(rec[3:], int(m.sum()), "still frames", int((m & d["still"]).sum()))
