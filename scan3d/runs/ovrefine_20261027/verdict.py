#!/usr/bin/env python3
"""ovrefine verdict (PLAN.md 4 and 5, written before the dev scores were read).
  python3 verdict.py dev  <score json of the dev set>   -> selected condition or "no test" (PLAN 4)
  python3 verdict.py test <score json of the test set> COND -> rules (1)(2) on test HS27 (PLAN 5) + reference numbers
"""
import json
import sys

DEV_CANDS = ["R10", "R5", "R10p4", "R10q", "R10m2", "R30f30"]


def dev(sc):
    d = sc["dev"]
    S, P = d["summary"], d["paired"]
    b_div = S["B60"]["div"]
    rows = []
    for c in DEV_CANDS:
        s = S.get(f"{c}.ref")
        p = P.get(f"B60->{c}.ref")
        if not s or not p:
            continue
        rej = sum((x.get(c) or {}).get("rs", {}).get("n_rej", 0) for x in d["cells"])
        rows.append(dict(cond=c, div=s["div"], d_med=p["d_med"], better=p["b_better"], n=p["n"], rej=rej, ok_div=s["div"] <= b_div))
    for r in rows:
        print(r)
    ok = sorted([r for r in rows if r["ok_div"]], key=lambda r: (r["d_med"], r["rej"]))
    if not ok:
        print("DEV: no condition with div <= B60 -> no test, not adopted")
        return None
    best = ok[0]
    if best["d_med"] >= 0:
        print(f"DEV: best {best['cond']} d_med {best['d_med']} >= 0 -> no test, not adopted (PLAN 4)")
        return None
    print(f"DEV: selected {best['cond']} (d_med {best['d_med']}, div {best['div']} vs B60 {b_div})")
    return best["cond"]


def test(sc, c):
    t = sc["test"]
    S, P = t["summary"], t["paired"]
    s, b, p = S[f"{c}.ref"], S["B60"], P[f"B60->{c}.ref"]
    r1 = s["div"] <= b["div"]
    r2 = p.get("d_med") is not None and p["d_med"] < 0
    print(f"(1) div ref {s['div']} {s['div_cells']} vs B60 {b['div']} {b['div_cells']} -> {'PASS' if r1 else 'FAIL'}")
    print(f"(2) paired d_med {p.get('d_med')} cm (better {p.get('b_better')}/{p.get('n')}, recordings better {p.get('rec_b_better')}) -> {'PASS' if r2 else 'FAIL'}")
    print("ref: rpe1", P.get(f"B60->{c}.ref rpe1"), "| ate med B60", b["ate_w_med"], "ref", s["ate_w_med"])
    print("VERDICT:", "adoptable (PC)" if r1 and r2 else "NOT adopted")
    return r1 and r2


if __name__ == "__main__":
    sc = json.load(open(sys.argv[2]))
    if sys.argv[1] == "dev":
        dev(sc)
    else:
        test(sc, sys.argv[3])
