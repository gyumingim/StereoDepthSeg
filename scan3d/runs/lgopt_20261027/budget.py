#!/usr/bin/env python3
"""lgopt: phone-budget sims (dev + hand SEEN) on top of the grid's gates (inl 60, ratio 0, rp 2) and keyframe load stats.
-> budget.json. Phone ms = ESTIMATE from coordinator phone measurements 10-27 19:4x (XFeat+NN x4 per KF 41.6 ms 4 threads,
LG 512 88.3 / 1024 252.8 ms p50), PnP not included."""
import json, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import lg_eval as E  # noqa: E402
RD = Path(__file__).resolve().parent
G = dict(pick="x", inl=60, ratio=0.0, cells=0, rp=2.0, yaw=10.0)
PH = dict(base=41.6, lg={512: 88.3, 1024: 252.8})
VAR = [dict(), dict(max_cand=2), dict(max_cand=1), dict(kf_every=0.5, consist_kf=4), dict(kf_every=0.5, consist_kf=4, max_cand=1),
       dict(kf_every=1.0, consist_kf=6), dict(kf_every=1.0, consist_kf=6, max_cand=1)]
conds = sys.argv[1].split(",") if len(sys.argv) > 1 else ["L", "Lp", "LT", "LTp", "LTp512", "Lc"]
ref = E.ev("B60", dict(pick="b"))
out = dict(made=time.strftime("%F %T"), gates=G, phone_ms_basis=PH, ref=ref, rows=[])
for cond in conds:
    cs = E.cells_of(cond) + E.cells_of(cond, "hand")
    if not cs:
        continue
    topk = 512 if "512" in cond else 1024
    nn_tri = cond.startswith("LT")
    for v in VAR:
        g = dict(G, **v)
        t = E.ev(cond, g)
        # LG calls per KF: candidates verified (sim rule) + 1 for triangulation if LG tri
        n_kf = n_lg = 0
        per = []
        for c in cs:
            last = None
            for k in c.kfs:
                n_kf += 1
                olds = []
                for e in k.get("cx", []):
                    if e["o"] not in olds:
                        olds.append(e["o"])
                if olds and g.get("kf_every", 0) > 0:
                    if last is not None and k["t"] - last < g["kf_every"]:
                        olds = []
                    else:
                        last = k["t"]
                m = min(len(olds), g.get("max_cand", 3)) + (0 if nn_tri else 1)
                n_lg += m
                per.append(PH["base"] + m * PH["lg"][topk])
        per.sort()
        r = dict(cond=cond, var=v, good=t["good"], f3=t["false_good"], f25=t["false25"], f20=t["false20"], hand=t["hand"],
                 passes=E.passes(t, ref), lg_per_kf=round(n_lg / n_kf, 2), phone_ms_mean=round(sum(per) / len(per), 1),
                 phone_ms_p50=per[len(per) // 2], phone_ms_p90=per[int(len(per) * 0.9)])
        out["rows"].append(r)
        print(r, flush=True)
json.dump(out, open(RD / "budget.json", "w"), indent=1)
