#!/usr/bin/env python3
"""Inspect frames that pass at <0.75 px while GT speed is high (sanity of labels)."""
import pickle
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
P = pickle.load(open(ROOT / "runs/zupt_screen/per_frame.pkl", "rb"))
for rec in ["hs_20261019_131326", "hs_20261020_221052", "hs_20261021_215348", "hs_20261019_154320"]:
    d = P[rec]
    t0 = d["t"][0]
    for lo, hi in [(0, 0.5), (0.5, 0.75), (0.75, 1.0)]:
        m = (d["disp"] >= lo) & (d["disp"] < hi) & (d["nf"] > 20) & np.isfinite(d["vgt"])
        v = d["vgt"][m] * 100
        print(rec[3:], f"disp[{lo},{hi})", "n", int(m.sum()), "GT v cm/s pct50/90/99", np.round(np.percentile(v, [50, 90, 99]), 2) if m.any() else None,
              ">2", int((v > 2).sum()), ">5", int((v > 5).sum()), ">10", int((v > 10).sum()),
              "gyro|w| med", round(float(np.nanmedian(d["wn"][m])), 4) if m.any() else None)
    m = (d["disp"] < 0.75) & (d["nf"] > 20) & (d["vgt"] > 0.1)
    idx = np.where(m)[0][:6]
    for i in idx:
        print("   t=%.2f s disp=%.3f nf=%d nruns=%d v=%.3f m/s w=%.3f still=%d q=%d" % ((d["t"][i] - t0) / 1e9, d["disp"][i], d["nf"][i], d["nruns"][i], d["vgt"][i], d["wn"][i], d["still"][i], d["cq"][i]))
