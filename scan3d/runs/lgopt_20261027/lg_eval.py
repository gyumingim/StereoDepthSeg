#!/usr/bin/env python3
"""lgopt 2026-10-27: dev-only gate search for the LighterGlue conditions (PLAN 3) + budget factors, hand loops (SEEN) counted only.
  python3 lg_eval.py check            sim (default / run gates) == the runs' own loop lines on dev + hand
  python3 lg_eval.py grid [conds]     -> lg_eval.json, lg_eval.md
J = good - 100 * excess false(3 deg) - 20 * excess false(2.5 deg) over B60 dev (xfopt J form); grid instead of sequential (PLAN 3)."""
import itertools
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim  # noqa: E402
import lgopt_run as XR  # noqa: E402

RD = Path(__file__).resolve().parent
_cells = {}
RUN_GATES = {"FNN": dict(ratio=0.3, cells=10, rp=2.0, yaw=3.0)}       # gates inside the bridge for that run
FLG_G = dict(inl=60, ratio=0.6, rp=2.0)                                # xfopt FLG gates (dev-selected, sequential)
GRID = dict(inl=[0, 40, 60], ratio=[0.0, 0.3, 0.45, 0.6], cells=[0, 10], rp=[3.0, 2.0], yaw=[10.0, 3.0])
BUDGET = [dict(max_cand=2), dict(max_cand=1), dict(kf_every=1.0), dict(kf_every=1.0, consist_kf=6),
          dict(max_cand=1, kf_every=1.0, consist_kf=6)]


def cells_of(cond, which="dev"):
    k = (cond, which)
    if k not in _cells:
        L = []
        for r, off in XR.cells(which):
            name = f"{XR.key_of(r, off, which)}_{cond}"
            if (XR.OUT / cond / f"{name}.json").exists():
                L.append(sim.Cell(cond, name, gt=(which == "dev")))
        _cells[k] = L
    return _cells[k]


def ev(cond, gates):
    t = dict(cells=0, n=0, checked=0, good=0, false_good=0, false25=0, false20=0, hand=[])
    for c in cells_of(cond):
        acc = sim.simulate(c, gates)
        s = sim.edges_score(c, acc)
        t["cells"] += 1
        for k in ("n", "checked", "good", "false_good"):
            t[k] += s[k]
        t["false25"] += sim.edges_score(c, acc, deg=2.5)["false_good"]
        t["false20"] += sim.edges_score(c, acc, deg=2.0)["false_good"]
    for c in cells_of(cond, "hand"):
        t["hand"].append(len(sim.simulate(c, gates)))
    return t


def J(t, ref):
    return t["good"] - 100 * max(0, t["false_good"] - ref["false_good"]) - 20 * max(0, t["false25"] - ref["false25"])


def passes(t, ref):
    return t["false_good"] <= ref["false_good"] and t["false25"] <= ref["false25"]


def loose_key(g):   # PLAN 3 tie-break: looser gates first (ratio, then inl, then cells, rp, yaw)
    return (g.get("ratio", 0), g.get("inl", 0), g.get("cells", 0), -g.get("rp", 3), -g.get("yaw", 10))


def check():
    bad = 0
    for cond in sorted(p.name for p in XR.OUT.iterdir() if p.is_dir() and p.name in XR.CONDS):
        pick = "b" if cond == "B60" else "x"
        for which in ("dev", "hand"):
            for c in cells_of(cond, which):
                acc = sim.simulate(c, dict(pick=pick, **RUN_GATES.get(cond, {})))
                simr = sorted((s, cur, e["o"], e["ni"]) for s, cur, e in acc)
                act = sorted((x["session"], x["cur"], x["old"], x["n_inl"]) for x in c.loops)
                ok = simr == act
                bad += not ok
                if not ok:
                    print("DIFF", c.name, len(simr), len(act))
    print("check", "PASS" if not bad else f"FAIL {bad}")
    return bad


def grid(conds):
    ref = ev("B60", dict(pick="b"))
    out = dict(made=time.strftime("%F %T"), plan=XR.git("log", "-1", "--format=%h", "--", "runs/lgopt_20261027/PLAN.md"),
               lens_hs=XR.LENS_HS, ref_B60=ref, conds={})
    fnn = ev("FNN", dict(pick="x", **RUN_GATES["FNN"])) if cells_of("FNN") else None
    out["FNN"] = fnn
    print("B60", ref, "\nFNN", fnn)
    for cond in conds:
        if not cells_of(cond) or cond in ("B60", "FNN"):
            continue
        rows = []
        keys = list(GRID)
        for vals in itertools.product(*(GRID[k] for k in keys)):
            g = dict(pick="x", **dict(zip(keys, vals)))
            t = ev(cond, g)
            rows.append(dict(gates=g, score=t, J=J(t, ref), passes=passes(t, ref)))
        ok = [r for r in rows if r["passes"]]
        flg = ev(cond, dict(pick="x", **FLG_G))
        res = dict(base=ev(cond, dict(pick="x")), flg_gates=flg, n_pass=len(ok))
        if ok:
            jm = max(r["J"] for r in ok)
            band = [r for r in ok if r["J"] >= jm - 0.01 * ref["good"]]
            best = sorted(band, key=lambda r: (loose_key(r["gates"]), -r["J"]))[0]
            res.update(J_max=jm, band=len(band), best=best, top=sorted(ok, key=lambda r: -r["J"])[:8])
            bud = []
            for b in BUDGET:
                g = dict(best["gates"], **b)
                t = ev(cond, g)
                bud.append(dict(budget=b, score=t, J=J(t, ref), passes=passes(t, ref)))
            res["budget"] = bud
            s = best["score"]
            print(f"{cond}: pass {len(ok)}/{len(rows)} Jmax {jm} band {len(band)} best {best['gates']} -> good {s['good']} f3 {s['false_good']} "
                  f"f2.5 {s['false25']} f2 {s['false20']} hand {s['hand']} | FLG gates good {flg['good']} f3 {flg['false_good']} hand {flg['hand']}")
            for b in bud:
                print(f"   budget {b['budget']}: good {b['score']['good']} f3 {b['score']['false_good']} f2.5 {b['score']['false25']} hand {b['score']['hand']} pass {b['passes']}")
        else:
            print(f"{cond}: no gate set passes")
        res["all"] = rows
        out["conds"][cond] = res
    json.dump(out, open(RD / "lg_eval.json", "w"), indent=1)
    return out


if __name__ == "__main__":
    if sys.argv[1] == "check":
        sys.exit(1 if check() else 0)
    conds = sys.argv[2].split(",") if len(sys.argv) > 2 else ["L", "Lc", "Lp", "Lpd", "L512", "LT", "Lm", "Lbud"]
    grid(conds)
