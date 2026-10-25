# Summarise prof_pc.sh / phone runs: per run (prefix) the --prof-log stage times + --timing-log remap/feed, tracking frames
# (init=1). Prints one table per config with p50 / p95 / mean per variant (pooled over repeats and per repeat).
#   python3 prof_an.py DIR [--cfgs c4,c4n400s100c20] [--json out.json]
import argparse
import csv
import glob
import json
import os
import re
from collections import defaultdict

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("dir")
ap.add_argument("--json", default=None)
ap.add_argument("--stages", default="feed_ms,remap_ms,track,prop,select,msckf_tri,msckf_jac,msckf_comp,msckf_ekf,slam_jac,"
                "slam_ekf,delay_tri,delay_init,retri,db_clean,anchor,marg_clone,slam_marg,ekf_ma,ekf_s,ekf_k,ekf_cov,ekf_sym,ekf_dx")
a = ap.parse_args()
ST = a.stages.split(",")

runs = defaultdict(list)  # (cfg, var) -> [prefix]
for p in sorted(glob.glob(os.path.join(a.dir, "*.prof.csv"))):
    m = re.match(r"(.+)_([A-Za-z0-9]+)_r(\d+)\.prof\.csv$", os.path.basename(p))
    if m:
        runs[(m.group(1), m.group(2))].append(p[:-len(".prof.csv")])


def load(prefix):
    P = list(csv.DictReader(open(prefix + ".prof.csv")))
    T = {r["t_ns"]: r for r in csv.DictReader(open(prefix + ".tl.csv"))} if os.path.exists(prefix + ".tl.csv") else {}
    rows = []
    for r in P:
        if r["init"] != "1":
            continue
        d = {k: float(v) for k, v in r.items() if k not in ("t_ns",)}
        t = T.get(r["t_ns"])
        d["remap_ms"] = float(t["remap_ms"]) if t else float("nan")
        rows.append(d)
    return rows


def stats(v):
    v = np.asarray(v, float)
    v = v[np.isfinite(v)]
    if not len(v):
        return None
    return dict(p50=round(float(np.percentile(v, 50)), 3), p95=round(float(np.percentile(v, 95)), 3), mean=round(float(v.mean()), 3))


out = {}
for cfg in sorted({c for c, _ in runs}):
    print(f"\n### {cfg} (tracking frames; ms; pooled over repeats, per-repeat feed p50 in brackets)")
    vars_ = sorted(v for c, v in runs if c == cfg)
    hdr = "| stage | " + " | ".join(f"{v} p50 / p95 / mean" for v in vars_) + " |"
    print(hdr)
    print("|" + "---|" * (len(vars_) + 1))
    data = {v: [load(p) for p in runs[(cfg, v)]] for v in vars_}
    out[cfg] = {}
    for s in ST + ["n_slam", "cov_n", "ekf_calls", "ekf_rows", "msckf_feats", "msckf_rows"]:
        cells = []
        for v in vars_:
            pooled = [r[s] for rep in data[v] for r in rep if s in r]
            st = stats(pooled)
            out[cfg].setdefault(v, {})[s] = st
            if st is None:
                cells.append("-")
                continue
            extra = ""
            if s == "feed_ms":
                extra = " [" + ", ".join(f"{np.percentile([r[s] for r in rep], 50):.2f}" for rep in data[v]) + "]"
            cells.append(f"{st['p50']:.2f} / {st['p95']:.2f} / {st['mean']:.2f}{extra}")
        print(f"| {s} | " + " | ".join(cells) + " |")
    for v in vars_:
        out[cfg][v]["n_frames"] = [len(rep) for rep in data[v]]
        out[cfg][v]["runs"] = runs[(cfg, v)]
    # by n_slam bins (feed_ms p50/p95, pooled)
    bins = [(0, 0), (1, 25), (26, 50), (51, 75), (76, 100)]
    print("\nfeed_ms by n_slam (p50 / p95, n):")
    for lo, hi in bins:
        cells = []
        for v in vars_:
            vv = [r["feed_ms"] for rep in data[v] for r in rep if lo <= r["n_slam"] <= hi]
            cells.append(f"{v} {np.percentile(vv, 50):.2f}/{np.percentile(vv, 95):.2f} ({len(vv)})" if len(vv) > 5 else f"{v} -")
        print(f"  n_slam {lo}-{hi}: " + " | ".join(cells))
if a.json:
    json.dump(out, open(a.json, "w"), indent=1)
