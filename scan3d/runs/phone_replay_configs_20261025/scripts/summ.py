#!/usr/bin/env python3
"""Markdown tables from results/*/metrics.json (+ run.txt temps). summ.py RESULTS_DIR > table.md"""
import json
import sys
from pathlib import Path

R = Path(sys.argv[1])
rows = []
for d in sorted(R.iterdir()):
    if not (d / "metrics.json").exists():
        continue
    m = json.load(open(d / "metrics.json"))
    rt = {}
    if (d / "run.txt").exists():
        rt = dict(kv.split("=", 1) for kv in open(d / "run.txt").read().split())
    fe = (d / "feeder.txt").read_text().strip() if (d / "feeder.txt").exists() else ""
    wall = fe.split(" frames, ")[1].split(" s")[0] if " frames, " in fe else "?"
    rows.append((rt.get("start", ""), d.name, m, rt, wall))
rows.sort()
S = lambda m, k: m.get("ov_stage_ms", {}).get(k, {})
print("| run | cfg | spin | thr | start | SKIN start->end (max) | wall s | proc p50 / p95 / max ms | >33.3 ms % | remap p50 | "
      "tracking p50/p95 | msckf p50/p95 | slam upd p50/p95 | slam delayed p50/p95 | re-tri&marg p50/p95 | OV total p50 | "
      "n_slam med / max | n_msckf med | ATE cm | scale |")
print("|" + "---|" * 20)
for st, name, m, rt, wall in rows:
    a = m["trk"]
    f = lambda k: f"{S(m, k).get('p50')} / {S(m, k).get('p95')}"
    acc = m.get("acc", {})
    print(f"| {name} | {rt.get('cfg','')} | {rt.get('idle_spin','')} | {rt.get('threads','')} | {st} | "
          f"{rt.get('skin_start','')}->{rt.get('skin_end','')} ({rt.get('skin_max','')}) | {wall} | "
          f"{a['proc_p50']} / {a['proc_p95']} / {a['proc_max']} | {a['over33_pct']} | {a['remap_p50']} | {f('tracking')} | "
          f"{f('msckf update')} | {f('slam update')} | {f('slam delayed')} | {f('re-tri & marg')} | {S(m, 'total').get('p50')} | "
          f"{m['n_slam_med']} / {m['n_slam_max']} | {m['n_msckf_med']} | {acc.get('ate_cm')} | {acc.get('scale')} |")
print()
print("Per-frame stage cost (p50 ms) by n_slam bin (tracking phase):")
print()
print("| run | n_slam bin | frames | proc p50 | proc p95 | >33 % | tracking | propagation | msckf | slam upd | slam delayed | re-tri&marg | OV total |")
print("|" + "---|" * 13)
for st, name, m, rt, wall in rows:
    for b in m.get("by_n_slam", []):
        print(f"| {name} | {b['n_slam']} | {b['n']} | {b['proc_p50']} | {b['proc_p95']} | {b['over33_pct']} | {b['tracking']} | "
              f"{b['propagation']} | {b['msckf update']} | {b['slam update']} | {b['slam delayed']} | {b['re-tri & marg']} | {b['total']} |")
