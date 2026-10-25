#!/usr/bin/env python3
"""ovupd 2026-10-26: stage table from ov_bridge --prof-log files (tracking frames with n_slam > 0).
  python3 prof_an.py OUT.md GROUP=file1,file2 GROUP=... [--stages a,b,c]
Per group: pooled p50 / p95 / mean of each stage + per-file feed_ms p50/p95."""
import csv, sys
import numpy as np

STAGES = "feed_ms,track,prop,msckf_jac,msckf_comp,msckf_ekf,slam_jac,slam_ekf,delay_init,marg_clone,ekf_ma,ekf_s,ekf_k,ekf_cov,ekf_sym,ekf_dx,n_slam,cov_n,ekf_rows,sparse_used".split(",")
out, groups = sys.argv[1], []
for a in sys.argv[2:]:
    g, fs = a.split("=", 1)
    groups.append((g, fs.split(",")))

def load(p):
    rows = [r for r in csv.DictReader(open(p)) if float(r.get("n_slam", 0) or 0) > 0]
    return rows

data, per = {}, {}
for g, fs in groups:
    allr = []
    per[g] = []
    for f in fs:
        r = load(f)
        allr += r
        v = np.array([float(x["feed_ms"]) for x in r])
        per[g].append(f"{np.percentile(v,50):.2f}/{np.percentile(v,95):.2f} (n {len(v)})")
    data[g] = allr
L = ["| stage | " + " | ".join(g for g, _ in groups) + " |", "|---|" + "---|" * len(groups)]
for s in STAGES:
    cells = []
    for g, _ in groups:
        try:
            v = np.array([float(x[s]) for x in data[g]])
            cells.append(f"{np.percentile(v,50):.2f} / {np.percentile(v,95):.2f} / {v.mean():.2f}")
        except (KeyError, ValueError):
            cells.append("-")
    L.append(f"| {s} | " + " | ".join(cells) + " |")
L.append("")
for g, _ in groups:
    L.append(f"- {g} feed_ms p50/p95 per run: " + ", ".join(per[g]))
open(out, "w").write("\n".join(L) + "\n")
print("\n".join(L))
