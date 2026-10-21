#!/usr/bin/env python3
"""Logged ZUPT disparity (non-held runs) inside each IMU still window [a+1, min(b-0.5, a+11)] s, for the 4 new recordings,
to compare with img_cause.py KLT jitter. Also: share of frames with < 0.5 px and features count."""
import sys, os, pickle, json
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "imu_learn")); os.chdir(ROOT)
import i3_confirm as ic
D = pickle.load(open(ROOT / "runs/zupt_screen/frames.pkl", "rb"))
out = {}
for rec in ["hs_20261021_213341", "hs_20261021_213955", "hs_20261021_214550", "hs_20261021_215348"]:
    x = ic.inputs(rec); f0 = x["first_frame_imu_ns"]
    rows = []
    for a, b in x["stills_s"]:
        if b - a < 5:
            continue
        lo, hi = f0 + (a + 1) * 1e9, f0 + min(b - 0.5, a + 11) * 1e9
        ds, nfs, nr = [], [], 0
        for r in D["runs"]:
            if r["rec"] != rec or r["held"]:
                continue
            m = (r["t"] >= lo) & (r["t"] <= hi)
            if m.sum() > 50:
                nr += 1
                ds.append(r["disp"][m]); nfs.append(r["nf"][m])
        if not ds:
            rows.append(dict(window_s=[a + 1, min(b - 0.5, a + 11)], n_runs=0)); continue
        d = np.concatenate(ds); nf = np.concatenate(nfs)
        rows.append(dict(window_s=[round(a + 1, 2), round(min(b - 0.5, a + 11), 2)], n_runs=nr, n=int(len(d)),
                         disp_pct_10_50_90=[round(float(v), 3) for v in np.percentile(d, [10, 50, 90])],
                         frac_lt05=round(float((d < 0.5).mean()), 3), frac_lt075=round(float((d < 0.75).mean()), 3),
                         frac_lt1=round(float((d < 1.0).mean()), 3), nf_med=int(np.median(nf))))
    out[rec] = rows
    for r in rows:
        print(rec[-6:], json.dumps(r))
json.dump(out, open(ROOT / "runs/zupt_screen/still_disp.json", "w"), indent=1)
