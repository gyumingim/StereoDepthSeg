#!/usr/bin/env python3
"""xfopt dev tuning (dev cells only): offline gates (sim.py) on each logged condition, sequential factor search (PLAN 9).
Selection rule (PLAN 5): false (corner rule, 3 deg) <= B60 dev AND false at 2.5 deg <= B60 dev at 2.5 deg; then most GT-good edges.
Writes dev_eval.json + prints a table."""
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim  # noqa: E402
import xfopt_run as XR  # noqa: E402

RD = Path(__file__).resolve().parent
_cells = {}


def cells_of(cond):
    if cond not in _cells:
        L = []
        for r, off in XR.cells("dev"):
            name = f"{XR.key_of(r, off, 'dev')}_{cond}"
            if (XR.OUT / cond / f"{name}.json").exists():
                L.append(sim.Cell(cond, name))
        _cells[cond] = L
    return _cells[cond]


def ev(cond, gates):
    tot = dict(cells=0, n=0, checked=0, good=0, false_good=0, false_all=0, false25=0, false35=0)
    per_rec = {}
    for c in cells_of(cond):
        acc = sim.simulate(c, gates)
        s = sim.edges_score(c, acc)
        s25 = sim.edges_score(c, acc, deg=2.5)
        s35 = sim.edges_score(c, acc, deg=3.5)
        tot["cells"] += 1
        for k in ("n", "checked", "good", "false_good", "false_all"):
            tot[k] += s[k]
        tot["false25"] += s25["false_good"]
        tot["false35"] += s35["false_good"]
        rec = c.res["rec"]
        per_rec[rec] = per_rec.get(rec, 0) + s["false_good"]
    tot["false_by_rec"] = per_rec
    return tot


FACTORS = [   # (name, list of gate dicts) - sequential, each factor alone on top of the best so far (PLAN 9)
    ("a_inl", [dict(inl=40), dict(inl=60)]),
    ("a_ratio", [dict(ratio=0.3), dict(ratio=0.5), dict(ratio=0.6)]),
    ("a_cells", [dict(cells=10), dict(cells=16)]),
    ("b_ntri", [dict(ntri_frac=0.9), dict(ntri_frac=0.8), dict(ntri_frac=0.7)]),
    ("g_rp", [dict(rp=2.0), dict(rp=1.5), dict(rp=1.0)]),
    ("g_yaw", [dict(yaw=3.0), dict(yaw=2.0), dict(yaw=1.5), dict(yaw_a=1.0, yaw_b=0.05)]),
    ("d_consist", [dict(consist_n=2)]),
]


def J(t, ref):
    return t["good"] - 100 * max(0, t["false_good"] - ref["false_good"]) - 20 * max(0, t["false25"] - ref["false25"])


def passes(t, ref):
    return t["false_good"] <= ref["false_good"] and t["false25"] <= ref["false25"]


def main():
    conds = sys.argv[1].split(",") if len(sys.argv) > 1 else ["X1", "X1n", "X1t3", "X1m1", "X1m2"]
    ref = ev("B60", dict(pick="b"))
    out = dict(made=time.strftime("%F %T"), ref_B60=ref, conds={})
    print("B60", {k: v for k, v in ref.items() if k != "false_by_rec"})
    for cond in conds:
        if not cells_of(cond):
            continue
        for pick in ("x", "h"):
            best = dict(pick=pick)
            hist = [("base", dict(best), ev(cond, best))]
            for fname, levels in FACTORS:
                cur = hist[-1][2] if hist else None
                cand = []
                for lv in levels:
                    g = dict(best, **lv)
                    t = ev(cond, g)
                    cand.append((g, t))
                    print(f"{cond} {pick} {fname} {lv}: n {t['n']} good {t['good']} false {t['false_good']} /2.5 {t['false25']} /3.5 {t['false35']} all {t['false_all']}")
                # objective J (the first greedy rule, 13:2x, destroyed nearly all XFeat edges on X1 -> replaced; dev only):
                # J = good - 100 x excess false (3 deg) - 20 x excess false (2.5 deg), excess over B60 dev; a level is kept only if J rises
                bx = max(cand, key=lambda x: J(x[1], ref))
                if J(bx[1], ref) > J(hist[-1][2], ref):
                    best = bx[0]
                    hist.append((fname, dict(best), bx[1]))
            final = hist[-1]
            print(f"== {cond} {pick} final {final[1]} -> good {final[2]['good']} false {final[2]['false_good']} /2.5 {final[2]['false25']} pass {passes(final[2], ref)}")
            out["conds"][f"{cond}/{pick}"] = dict(history=[dict(step=h[0], gates=h[1], score=h[2]) for h in hist], pass_rule=passes(final[2], ref))
    json.dump(out, open(RD / "dev_eval.json", "w"), indent=1)


if __name__ == "__main__":
    main()
