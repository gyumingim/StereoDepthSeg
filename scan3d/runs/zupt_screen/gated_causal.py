#!/usr/bin/env python3
"""zupt_screen step 6 (POST-HOC design idea, exploratory): causal gyro-gated relaxed threshold.
pass = disp < 0.5  OR  (disp < thr AND causal gyro still), causal still = trailing 0.5 s gyro windows ending at t, t-0.5,
t-1.0 all have per-axis std < 0.01 rad/s and |mean| < 0.03 rad/s (i3_confirm rule, made causal, ~1.5 s).
Per run counts of new passes with GT > 2 / > 5 cm/s, and gain inside IMU stills."""
import sys, os, json, pickle, time
from pathlib import Path
from collections import defaultdict
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "imu_learn")); os.chdir(ROOT)
from imu_gt import read_imu
D = pickle.load(open(ROOT / "runs/zupt_screen/frames.pkl", "rb"))
P = pickle.load(open(ROOT / "runs/zupt_screen/per_frame.pkl", "rb"))
THRS = [0.75, 1.0, 1.5]
BIN = 4_000_000
G = {}


def causal_still(rec, t):
    if rec not in G:
        try:
            _, g = read_imu(Path("recordings") / rec / "imu.csv", raw=True)
        except ValueError:
            _, g = read_imu(Path("recordings") / rec / "imu.csv", raw=False)
        gt_ = g[:, 0].astype(np.int64); w = g[:, 1:4]
        c1 = np.concatenate([[np.zeros(3)], np.cumsum(w, 0)]); c2 = np.concatenate([[np.zeros(3)], np.cumsum(w * w, 0)])
        G[rec] = (gt_, c1, c2)
    gt_, c1, c2 = G[rec]

    def ok_at(te):
        i1 = np.searchsorted(gt_, te); i0 = np.searchsorted(gt_, te - 500_000_000)
        n = (i1 - i0)[:, None].astype(float)
        good = n[:, 0] >= 20
        n = np.maximum(n, 1)
        m = (c1[i1] - c1[i0]) / n
        v = (c2[i1] - c2[i0]) / n - m * m
        sd = np.sqrt(np.maximum(v, 0))
        return good & np.all(sd < 0.01, 1) & (np.linalg.norm(m, axis=1) < 0.03)
    return ok_at(t) & ok_at(t - 500_000_000) & ok_at(t - 1_000_000_000)


res = defaultdict(lambda: defaultdict(list))
for r in D["runs"]:
    if r["held"]:
        continue
    rec = r["rec"]; L = P[rec]
    k = np.round(r["t"] / BIN).astype(np.int64)
    idx = np.clip(np.searchsorted(L["t"] // BIN, k), 0, len(L["t"]) - 1)
    v, still = L["vgt"][idx], L["still"][idx]
    cs = causal_still(rec, r["t"])
    disp, nfok = r["disp"], r["nf"] > 20
    base = (disp < 0.5) & nfok
    for th in THRS:
        new = (disp < th) & nfok & ~base & cs
        res[rec][f"{th}_gt2"].append(int((new & (v > 0.02)).sum()))
        res[rec][f"{th}_gt5"].append(int((new & (v > 0.05)).sum()))
        res[rec][f"{th}_imumove"].append(int((new & ~still).sum()))
        res[rec][f"{th}_gain"].append(int((new & still).sum()))
out = dict(made=time.strftime("%F %T"), label="POST-HOC design idea, exploratory", recs={})
print("rec | thr: new&gated GT>2 med[max] / GT>5 max / IMU-move med[max] / gain med")
for rec in sorted(res):
    R = res[rec]; s = {}
    line = rec[3:]
    for key, a in R.items():
        a = np.array(a); s[key] = dict(med=float(np.median(a)), max=int(a.max()))
    for th in THRS:
        line += (f" | {th}: {s[f'{th}_gt2']['med']:.0f}[{s[f'{th}_gt2']['max']}] / {s[f'{th}_gt5']['max']}"
                 f" / {s[f'{th}_imumove']['med']:.0f}[{s[f'{th}_imumove']['max']}] / {s[f'{th}_gain']['med']:.0f}")
    out["recs"][rec] = s
    print(line)
json.dump(out, open(ROOT / "runs/zupt_screen/gated_causal.json", "w"), indent=1)
