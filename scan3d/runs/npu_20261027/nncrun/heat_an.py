#!/usr/bin/env python3
"""nncrun 2026-10-28: NPU-only heat test analysis (README 11차). mon.csv (2-3 s samples) + phases.log (epoch start/end lines)
+ app logs heat_p1/p3 (HEATWIN 10 s windows). Per phase: AP/SKIN rise rate (deg C/min = (mean of last 20 s - mean of first 20 s)
/ phase minutes), mean current minus P0 mean (mA, dumpsys 'current now', negative = discharge), NPU p50 per window.
  python3 heat_an.py HEATDIR OUT.json"""
import json, re, sys
import numpy as np
D = sys.argv[1]
M = np.genfromtxt(f"{D}/mon.csv", delimiter=",", names=True, dtype=None, encoding=None)
t = M["pc_epoch"].astype(float); ap = M["ap"].astype(float); sk = M["skin"].astype(float); cu = M["current_mA"].astype(float)
ev = {}
for ln in open(f"{D}/phases.log"):
    p = ln.split()
    m = re.match(r"(P\d) (start|end)", " ".join(p[2:4]))
    if m: ev[(m.group(1), m.group(2))] = float(p[0])
# phase boundaries: P0 start..P1 start, P1 start..P1 end, P1 end..P2 end, P3 start..P3 end, P3 end..P4 end
seg = {"P0_idle": (ev.get(("P0", "start")), ev.get(("P1", "start"))), "P1_npu_b2b": (ev.get(("P1", "start")), ev.get(("P1", "end"))),
       "P2_idle": (ev.get(("P1", "end")), ev.get(("P2", "end"))), "P3_npu_4Hz": (ev.get(("P3", "start")), ev.get(("P3", "end"))),
       "P4_idle": (ev.get(("P3", "end")), ev.get(("P4", "end")))}
res = {"samples": int(len(t)), "sample_dt_med_s": float(np.median(np.diff(t))), "phases": {}}
base_cu = None
for k, (a, b) in seg.items():
    if a is None or b is None: continue
    s = (t >= a) & (t <= b)
    if s.sum() < 3: continue
    tt = t[s]; f = tt <= tt[0] + 20; l = tt >= tt[-1] - 20
    mins = (tt[-1] - tt[0]) / 60
    r = {"dur_s": float(tt[-1] - tt[0]), "ap_start": float(ap[s][f].mean()), "ap_end": float(ap[s][l].mean()), "ap_max": float(ap[s].max()),
         "skin_start": float(sk[s][f].mean()), "skin_end": float(sk[s][l].mean()),
         "ap_rate_C_per_min": float((ap[s][l].mean() - ap[s][f].mean()) / mins), "skin_rate_C_per_min": float((sk[s][l].mean() - sk[s][f].mean()) / mins),
         "current_mean_mA": float(cu[s].mean()), "current_med_mA": float(np.median(cu[s]))}
    if k == "P0_idle": base_cu = r["current_mean_mA"]
    if base_cu is not None: r["current_delta_vs_P0_mA"] = float(r["current_mean_mA"] - base_cu)
    res["phases"][k] = r
for nm in ("p1", "p3"):
    try:
        W = [tuple(map(float, re.findall(r"t=(\d+) s n=(\d+) p50=(\d+) p95=(\d+)", ln)[0])) for ln in open(f"{D}/heat_{nm}.txt") if "HEATWIN" in ln]
        tot = [ln.strip() for ln in open(f"{D}/heat_{nm}.txt") if "HEAT total" in ln]
        res[f"npu_{nm}"] = {"windows_t_n_p50_p95": W, "total": tot}
    except (OSError, IndexError):
        pass
print(json.dumps(res, indent=1))
json.dump(res, open(sys.argv[2], "w"), indent=1)
