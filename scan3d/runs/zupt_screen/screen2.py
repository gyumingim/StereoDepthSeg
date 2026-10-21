#!/usr/bin/env python3
"""zupt_screen step 3 (post-hoc, exploratory): per-run counts (not per-frame medians), GT speed bins, longest run of
consecutive new false passes, and a post-hoc 'gyro-gated' variant (new pass only inside an IMU still run).
Labels reuse per_frame.pkl (GT speed, IMU still, gyro) by nearest frame bin."""
import json, pickle, time
from pathlib import Path
from collections import defaultdict
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
D = pickle.load(open(ROOT / "runs/zupt_screen/frames.pkl", "rb"))
P = pickle.load(open(ROOT / "runs/zupt_screen/per_frame.pkl", "rb"))
THRS = [0.75, 1.0, 1.5]
BIN = 4_000_000


def longest(mask):
    best = cur = 0
    for x in mask:
        cur = cur + 1 if x else 0
        best = max(best, cur)
    return best


res = defaultdict(lambda: defaultdict(list))
for r in D["runs"]:
    if r["held"]:
        continue
    rec = r["rec"]; L = P[rec]
    k = np.round(r["t"] / BIN).astype(np.int64)
    idx = np.searchsorted(L["t"] // BIN, k)
    idx = np.clip(idx, 0, len(L["t"]) - 1)
    v, still, w = L["vgt"][idx], L["still"][idx], L["wn"][idx]
    disp, nfok = r["disp"], r["nf"] > 20
    base = (disp < 0.5) & nfok
    gm = np.isfinite(v) & (v > 0.02)
    res[rec]["base_gtmove"].append(int((base & gm).sum()))
    res[rec]["gtmove_n"].append(int(gm.sum()))
    for th in THRS:
        new = (disp < th) & nfok & ~base
        f = new & gm
        res[rec][f"{th}_new_gtmove"].append(int(f.sum()))
        res[rec][f"{th}_new_gt5"].append(int((new & (v > 0.05)).sum()))
        res[rec][f"{th}_new_gt10"].append(int((new & (v > 0.10)).sum()))
        res[rec][f"{th}_new_imumove"].append(int((new & ~still).sum()))
        res[rec][f"{th}_longest_false"].append(longest(f))
        res[rec][f"{th}_gain"].append(int((new & still).sum()))
        res[rec][f"{th}_gated_false_gt"].append(int((new & still & gm).sum()))
    res[rec]["runs"].append(r["run"])

out = dict(made=time.strftime("%F %T"), input=["runs/zupt_screen/frames.pkl", "runs/zupt_screen/per_frame.pkl"],
           note="post-hoc exploratory; per-run counts over non-held runs; median [min-max] across runs", recs={})
print("rec | nruns | base(0.5) GT>2 med | thr: new GT>2 med[min-max] / >5 / >10 / longest / IMU-move / gain / gated-false")
for rec in sorted(res):
    R = res[rec]
    s = dict(n_runs=len(R["runs"]), runs_sample=R["runs"][:3])
    line = f"{rec[3:]} | {len(R['runs'])} | {int(np.median(R['base_gtmove']))} of {int(np.median(R['gtmove_n']))} |"
    for key in R:
        if key == "runs":
            continue
        a = np.array(R[key])
        s[key] = dict(med=float(np.median(a)), min=int(a.min()), max=int(a.max()))
    for th in THRS:
        g = lambda kk: s[f"{th}_{kk}"]
        line += (f" {th}: {g('new_gtmove')['med']:.0f}[{g('new_gtmove')['min']}-{g('new_gtmove')['max']}]"
                 f"/{g('new_gt5')['med']:.0f}/{g('new_gt10')['med']:.0f}/{g('longest_false')['med']:.0f}"
                 f"/{g('new_imumove')['med']:.0f}/{g('gain')['med']:.0f}/{g('gated_false_gt')['med']:.0f} |")
    out["recs"][rec] = s
    print(line)
json.dump(out, open(ROOT / "runs/zupt_screen/screen_per_run.json", "w"), indent=1)
