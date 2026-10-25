# Phone replay analysis for the ovopt lists (2026-10-25). One line per run + a per-variant summary.
#   python3 phone_an.py OUT_DIR [OUT_DIR ...]     (OUT_DIR = pulled out_<list>/, one subdir per run tag)
# Per run: proc = remap_ms + feed_ms (tracking frames, init=1) p50 / p95 / max / % over 33.3 ms; feed p50 / p95;
# lag = (bridge receive - schedule) + processing, p95 / max (backlog); clock-state indicator = remap / feed p50 of the
# pre-init frames (the same work in every run); start / max SKIN, AP, cpu9 clock cap (run.txt); screen state;
# stage medians from prof.csv (ours) or ov_timing.txt (0.6.17 reference). Variant = tag without the _rN suffix.
import csv
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np


def pc(a, q):
    return round(float(np.percentile(a, q)), 2) if len(a) else None


rows = []
for od in sys.argv[1:]:
    for d in sorted(Path(od).iterdir()):
        if not d.is_dir() or not (d / "tl.csv").exists():
            continue
        T = list(csv.DictReader(open(d / "tl.csv")))
        if not T:
            continue
        f = lambda k: np.array([float(r[k]) for r in T])
        init, rm, fe = f("init"), f("remap_ms"), f("feed_ms")
        tr = init > 0
        proc = rm + fe
        tn = np.array([int(r["t_ns"]) for r in T], dtype=np.int64)
        rx = np.array([int(r["rx_boot_ns"]) for r in T], dtype=np.int64)
        lag = ((rx - rx[0]) - (tn - tn[0])) / 1e6
        lag = lag - lag.min() + proc + f("wait_ms")
        kv = {}
        if (d / "run.txt").exists():
            txt = open(d / "run.txt").read()
            for m in re.finditer(r"(\w+)=('[^']*'|\S+)", txt):
                kv[m.group(1)] = m.group(2).strip("'")
        st = {}
        if (d / "prof.csv").exists():
            P = list(csv.DictReader(open(d / "prof.csv")))
            P = [p for p in P if p["init"] == "1"]
            for k in ("track", "msckf_ekf", "slam_ekf", "msckf_comp", "msckf_jac", "retri", "ekf_ma", "ekf_cov", "ekf_k"):
                if P and k in P[0]:
                    st[k] = pc([float(p[k]) for p in P], 50)
            for k in ("msckf_comp",):
                if P and k in P[0]:
                    st[k + "_p95"] = pc([float(p[k]) for p in P], 95)
        elif (d / "ov_timing.txt").exists():
            O = np.array([[float(v) for v in l.split(",")] for l in open(d / "ov_timing.txt") if l[0] != "#" and l.strip()])
            if len(O):
                names = ["tracking", "propagation", "msckf", "slam_up", "slam_delay", "retri_marg", "total"]
                for i, k in enumerate(names):
                    st[k] = pc(O[:, 1 + i] * 1000, 50)
        po = d / "po.jsonl"
        n_slam = None
        if po.exists():
            P2 = [json.loads(x) for x in open(po) if '"pose"' in x]
            ns = [p.get("n_slam", 0) for p in P2 if p.get("status") == "tracking"]
            n_slam = float(np.median(ns)) if ns else None
        tag = d.name
        var = re.sub(r"_r\d+$", "", tag)
        rows.append(dict(dir=str(d), tag=tag, var=var, frames=len(T), trk=int(tr.sum()), proc_p50=pc(proc[tr], 50),
                         proc_p95=pc(proc[tr], 95), proc_max=round(float(proc[tr].max()), 1) if tr.any() else None,
                         over33=round(float((proc[tr] > 33.3).mean() * 100), 1) if tr.any() else None,
                         feed_p50=pc(fe[tr], 50), feed_p95=pc(fe[tr], 95), remap_p50=pc(rm[tr], 50),
                         lag_p95=pc(lag[tr], 95), lag_max=round(float(lag.max()), 1),
                         pre_remap=pc(rm[~tr], 50), pre_feed=pc(fe[~tr], 50), n_slam=n_slam,
                         skin0=kv.get("skin_start"), skinmax=kv.get("skin_max"), ap0=kv.get("ap_start"), ap1=kv.get("ap_end"),
                         cap9=kv.get("cpu9_cap_start"), screen=kv.get("screen_start"), stages=st))
print("| run | proc p50 / p95 / max | >33 ms % | feed p50 / p95 | remap p50 | lag p95 / max ms | pre-init remap / feed | n_slam | SKIN start / max | AP start / end | cpu9 cap | screen | stages p50 |")
print("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
for r in rows:
    print(f"| {r['tag']} | {r['proc_p50']} / {r['proc_p95']} / {r['proc_max']} | {r['over33']} | {r['feed_p50']} / {r['feed_p95']} | "
          f"{r['remap_p50']} | {r['lag_p95']} / {r['lag_max']} | {r['pre_remap']} / {r['pre_feed']} | {r['n_slam']} | "
          f"{r['skin0']} / {r['skinmax']} | {r['ap0']} / {r['ap1']} | {r['cap9']} | {r['screen']} | "
          f"{' '.join(f'{k} {v}' for k, v in r['stages'].items())} |")
by = defaultdict(list)
for r in rows:
    by[r["var"]].append(r)
print("\nper variant (median over runs of the per-run values):")
for v, rs in sorted(by.items()):
    med = lambda k: round(float(np.median([x[k] for x in rs if x[k] is not None])), 2) if any(x[k] is not None for x in rs) else None
    print(f"  {v}: runs {len(rs)} proc p50 {med('proc_p50')} p95 {med('proc_p95')} >33% {med('over33')} feed p50 {med('feed_p50')} "
          f"lag p95 {med('lag_p95')} | pre-init feed {med('pre_feed')}")
json.dump(rows, open(Path(sys.argv[1]) / "phone_an.json", "w"), indent=1)
