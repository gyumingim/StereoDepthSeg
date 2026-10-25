#!/usr/bin/env python3
"""Final tables from results/*/ (metrics.json, run.txt, tl.csv, samp.txt). final.py RESULTS_DIR > tables.md
Per run: proc (remap+feed) p50/p95/max, >33.3 %, late (pose lag vs on-time, ms), OV stage p50, n_slam, ATE,
speed index = pre-init remap/feed p50 (same work for all configs with the same num_pts, never backlogged),
big-core (cpu9) median current freq + cap where sampled (run_all2+)."""
import csv
import json
import sys
from pathlib import Path

import numpy as np

R = Path(sys.argv[1])
ST = ["tracking", "propagation", "msckf update", "slam update", "slam delayed", "re-tri & marg", "total"]
runs = []
for d in sorted(R.iterdir()):
    if not (d / "metrics.json").exists():
        continue
    m = json.load(open(d / "metrics.json"))
    kv = dict(x.split("=", 1) for x in open(d / "run.txt").read().split()) if (d / "run.txt").exists() else {}
    rows = list(csv.DictReader(open(d / "tl.csv")))
    pre = [x for x in rows if x["init"] == "0"][30:]
    m["pre_remap"] = float(np.median([float(x["remap_ms"]) for x in pre]))
    m["pre_feed"] = float(np.median([float(x["feed_ms"]) for x in pre]))
    if (d / "samp.txt").exists():
        L = open(d / "samp.txt").read().splitlines()
        fr = [int(l.split("freq ")[1].split()[9]) for l in L if l.startswith("t ")]
        cp = [int(l.split("cap ")[1].split()[3]) for l in L if l.startswith("t ") and " cap " in l]
        m["big_mhz"] = round(float(np.median(fr)) / 1e3) if fr else None
        m["big_cap_mhz"] = round(float(np.median(cp)) / 1e3) if cp else None
    m["kv"] = kv
    runs.append(m)
runs.sort(key=lambda m: m["kv"].get("start", ""))


def g(m, k, q="p50"):
    return m.get("ov_stage_ms", {}).get(k, {}).get(q)


print("| run | cfg | spin | thr | speed | start | SKIN start→end | proc p50 / p95 / max | >33.3 ms % | pose late p50 / max ms | "
      "tracking | msckf | slam upd | slam dly | re-tri&marg | propag. | OV total | n_slam med/max | ATE cm | pre-init remap / feed ms | big core MHz (cap) |")
print("|" + "---|" * 21)
for m in runs:
    kv, a, lt = m["kv"], m["trk"], m.get("late_ms", {})
    st = lambda k: f"{g(m, k)} / {g(m, k, 'p95')}"
    print(f"| {m['run']} | {kv.get('cfg')} | {kv.get('idle_spin')} | {kv.get('threads')} | {kv.get('speed', '1.0')}"
          f"{' ' + kv['extra'] if kv.get('extra') else ''} | {kv.get('start')} | {kv.get('skin_start')}→{kv.get('skin_end')} | "
          f"{a['proc_p50']} / {a['proc_p95']} / {a['proc_max']} | {a['over33_pct']} | {lt.get('p50')} / {lt.get('max')} | "
          f"{st('tracking')} | {st('msckf update')} | {st('slam update')} | {st('slam delayed')} | {st('re-tri & marg')} | "
          f"{g(m, 'propagation')} | {g(m, 'total')} | {m['n_slam_med']:.0f}/{m['n_slam_max']} | {m['acc'].get('ate_cm')} | "
          f"{m['pre_remap']:.2f} / {m['pre_feed']:.2f} | {m.get('big_mhz', '-')} ({m.get('big_cap_mhz', '-')}) |")
print()
print("Stage cost (p50 ms) by n_slam bin, tracking phase (bins with >= 5 frames):")
print()
print("| run | n_slam | frames | proc p50 | proc p95 | >33 % | tracking | propag. | msckf | slam upd | slam dly | re-tri&marg | OV total |")
print("|" + "---|" * 13)
for m in runs:
    for b in m.get("by_n_slam", []):
        print(f"| {m['run']} | {b['n_slam']} | {b['n']} | {b['proc_p50']} | {b['proc_p95']} | {b['over33_pct']} | {b['tracking']} | "
              f"{b['propagation']} | {b['msckf update']} | {b['slam update']} | {b['slam delayed']} | {b['re-tri & marg']} | {b['total']} |")
