#!/usr/bin/env python3
"""fixes 11-13 unit 15b: how far apart in baseline direction can a 2nd wide partner be? (poses only, CPU, seconds; no FFS / SIFT)
For each keyframe that got a 1st wide partner (KEYFRAMES json of a far run): geometric wide candidates (same window as pick_far:
B 0.35 x (1 -+ 0.3), dt <= 30 s, motion_stereo.score_pair with the far cfg) and their baseline-direction angle to the 1st
partner (motion_stereo.dir_differs rule, sign-free). Prints the share of keyframes that still have >= 1 candidate at angle >= T
(no overlap / dy check here -> an upper bound), and the angle of the 2nd partners the old rule picked.
Usage: u15b_p2_geom.py POSES_BA.jsonl.gz KEYFRAMES.json OUT_JSON"""
import gzip
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import motion_stereo as ms  # noqa: E402

pp, kp, out = sys.argv[1:4]
rows = [json.loads(l) for l in gzip.open(pp, "rt") if '"t_ns"' in l]
T = np.array([np.array(r["T_world_cam0"], float).reshape(4, 4) for r in rows])
t_ns = np.array([r["t_ns"] for r in rows], np.int64)
kf = json.loads(Path(kp).read_text())
cfg = ms.MSConfig(); cfg.b_target_m = 0.35; cfg.b_min_m, cfg.b_max_m = 0.35 * 0.7, 0.35 * 1.3; cfg.max_rot_deg = 15.0; cfg.dt_max_s = 30.0


def ang(g0, g1):
    u0, u1 = g0["c"] / g0["B"], g1["c"] / g1["B"]
    return float(np.degrees(np.arccos(np.clip(abs(u0 @ u1), 0, 1))))


best, old = [], []
for r in kf:
    j1 = r.get("far_partner")
    if j1 is None:
        continue
    i = r["i"]
    g1 = ms.pair_geometry(*ms.rel_pose(T[i], T[j1]))
    a_max = 0.0
    for j in range(len(T)):
        if j in (i, j1) or abs(t_ns[i] - t_ns[j]) * 1e-9 > cfg.dt_max_s:
            continue
        if not (cfg.b_min_m <= np.linalg.norm(T[j][:3, 3] - T[i][:3, 3]) <= cfg.b_max_m):
            continue
        g = ms.pair_geometry(*ms.rel_pose(T[i], T[j]))
        if ms.score_pair(g, abs(t_ns[i] - t_ns[j]) * 1e-9, cfg)[0] is None:
            continue
        a_max = max(a_max, ang(g1, g))
    best.append(a_max)
    if r.get("far_partner2") is not None:
        old.append(ang(g1, ms.pair_geometry(*ms.rel_pose(T[i], T[r["far_partner2"]]))))
best, old = np.array(best), np.array(old)
res = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), poses=pp, keyframes=kp, n_kf_with_partner=int(len(best)),
           share_with_candidate_at_least={str(a): float(np.mean(best >= a)) for a in (5, 10, 15, 20, 25, 30, 40)},
           old_p2_angle_deg=dict(n=int(len(old)), median=float(np.median(old)) if len(old) else None,
                                 p90=float(np.percentile(old, 90)) if len(old) else None,
                                 frac_lt_10=float(np.mean(old < 10)) if len(old) else None))
print(json.dumps(res, indent=1))
Path(out).write_text(json.dumps(res, indent=1))
