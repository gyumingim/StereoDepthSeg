#!/usr/bin/env python3
"""ovacc final summary (18:40): read score_*.json in results/ (copies of /tmp/ovacc/ate) and print one table
per scoring set: div, ATE med, paired d_med/d_mean vs ref, better/worse, two-sided sign-test p, |d|>0.3 cm cells.
python3 summarize.py > summary.md"""
import json
import math
import sys
from pathlib import Path

R = Path(__file__).resolve().parent / "results"


def sign_p(k, n):
    if n == 0:
        return None
    t = sum(math.comb(n, i) for i in range(0, min(k, n - k) + 1)) / 2 ** n
    return float(f'{min(1.0, 2 * t):.2g}')


def table(f):
    d = json.load(open(f))
    ref = d["ref"]
    print(f"\n### {f.name} (made {d['made']}, ref {ref})\n")
    print("| cond | cells | div | div cells | ATE med cm | paired n | d_med cm | d_mean cm | better/worse | sign p | |d|>0.3 cm (ref->cond) |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    for c in d["conds"]:
        s = d["summary"][c]
        rows = [x for x in d["cells"] if c in x and ref in x and not x[c]["div"] and not x[ref]["div"]] if c != ref else []
        dd = [x[c]["ate_w"] - x[ref]["ate_w"] for x in rows]
        b, w = sum(v < 0 for v in dd), sum(v > 0 for v in dd)
        big = "; ".join(f"{k} {a}->{bb}" for k, a, bb in (s.get("big_changes") or []))
        print(f"| {c} | {s['n']} | {s['div']} | {' '.join(s['div_cells'])} | {s['ate_w_med']} | {s.get('paired_n', '-')} | "
              f"{s.get('d_med', '-')} | {s.get('d_mean', '-')} | {f'{b}/{w}' if c != ref else '-'} | "
              f"{sign_p(b, b + w) if c != ref else '-'} | {big} |")


for f in sorted(R.glob("score_*.json")):
    if f.name.startswith("score_std"):
        d = json.load(open(f))
        print(f"\n### {f.name} (std60 live-mode set, made {d['made']}; 173432 never initialises)\n")
        print("| cell | " + " | ".join(d["conds"]) + " |")
        print("|---" * (len(d["conds"]) + 1) + "|")
        for x in d["cells"]:
            print(f"| {x['key']} | " + " | ".join(
                (f"{x[c]['ate_w']:.2f}" if x.get(c, {}).get("ate_w") is not None else "none") for c in d["conds"]) + " |")
    else:
        table(f)
