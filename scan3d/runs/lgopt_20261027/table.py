#!/usr/bin/env python3
"""lgopt: dev table from the actual runs (score.json): false loops (3 / 2.5 / 2.0 deg, corner rule), GT-good edges,
map ATE median + paired diff vs B60 (dev 15 cells), hand loops (SEEN) loop count + start-end cm. -> table.json / table.md"""
import json, time
import numpy as np
from pathlib import Path
RD = Path(__file__).resolve().parent
S = json.load(open(RD / "score.json"))["rows"]
by = {}
for k, r in S.items():
    by.setdefault(r["cond"], {})[r["name"].rsplit("_", 1)[0]] = r
out = dict(made=time.strftime("%F %T"), score_made=json.load(open(RD / "score.json"))["made"], rows={})
b = by["B60"]
for cond, cells in by.items():
    dev = {k: r for k, r in cells.items() if r["set"] == "dev"}
    hand = {k: r for k, r in cells.items() if r["set"] == "hand"}
    e = [x for r in dev.values() for x in r.get("edges", []) if x.get("gt") and x.get("gt_good")]
    f = lambda d: sum(1 for x in e if x["err_cm"] > 10 or x["err_deg"] > d)
    ate = [r.get("map_ate_cm") for r in dev.values() if r.get("map_ate_cm") is not None]
    dif = [r["map_ate_cm"] - b[k]["map_ate_cm"] for k, r in dev.items() if k in b and r.get("map_ate_cm") is not None and b[k].get("map_ate_cm") is not None]
    ms = [r["kf_ms_p50_p95"][0] for r in dev.values() if r.get("kf_ms_p50_p95")]
    out["rows"][cond] = dict(cells=len(dev), good=len(e), f3=f(3.0), f25=f(2.5), f20=f(2.0), loops=sum(r["n_loop"] for r in dev.values()),
                             map_ate_med=round(float(np.median(ate)), 3) if ate else None,
                             dATE_med=round(float(np.median(dif)), 3) if dif else None, dATE_mean=round(float(np.mean(dif)), 3) if dif else None,
                             dATE_range=[round(min(dif), 2), round(max(dif), 2)] if dif else None,
                             hand_loops={k: r["n_loop"] for k, r in sorted(hand.items())},
                             hand_map_cm={k: r.get("map_loop_cm") for k, r in sorted(hand.items())},
                             hand_vio_cm={k: r.get("vio_loop_cm") for k, r in sorted(hand.items())},
                             pc_kf_ms_p50_med=round(float(np.median(ms)), 1) if ms else None)
json.dump(out, open(RD / "table.json", "w"), indent=1)
for c, r in out["rows"].items():
    print(c, r)
