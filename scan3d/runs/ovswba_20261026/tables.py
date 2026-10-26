#!/usr/bin/env python3
"""ovswba: summary tables from score_*.json (made by swba_run.py score). HS 42 cells (213955 phone drop excluded) with and
without the 4 dev / selection cells, std60 per cell, hand loops. Usage: tables.py score_A.json [score_B.json ...]"""
import json
import sys

import numpy as np

DEV = {"133820_o0", "224323_o0", "214138_o0", "221052_o0"}
DROP = "hs_20261021_213955"


def summ(cells, get, excl=()):
    v = [get(x) for x in cells if x["rec"] != DROP and x["key"] not in excl and get(x) is not None]
    ok = [e["ate_w"] for e in v if not e.get("div") and e.get("ate_w") is not None]
    return len(v), sum(bool(e.get("div")) for e in v), (round(float(np.median(ok)), 3) if ok else None)


def paired(cells, ga, gb, excl=()):
    d = []
    for x in cells:
        if x["rec"] == DROP or x["key"] in excl:
            continue
        a, b = ga(x), gb(x)
        if a and b and not a.get("div") and not b.get("div") and a.get("ate_w") is not None and b.get("ate_w") is not None:
            d.append(b["ate_w"] - a["ate_w"])
    d = np.array(d)
    return (len(d), round(float(np.median(d)), 3), int((d < 0).sum())) if len(d) else (0, None, 0)


rows = []
for f in sys.argv[1:]:
    r = json.load(open(f))
    conds = [c for c in r["conds"] if c != "Z60"]
    hs = r["hs"]["cells"]
    for c in conds:
        for kind in ("swba", "raw"):
            g = (lambda x, c=c, k=kind: (x.get(c) or {}).get(k))
            n, dv, med = summ(hs, g)
            n2, dv2, med2 = summ(hs, g, DEV)
            pn, pd, pb = paired(hs, lambda x: x.get("GL60") if kind == "swba" else (x.get(c) or {}).get("rawvio"), g)
            pn2, pd2, pb2 = paired(hs, lambda x: x.get("GL60") if kind == "swba" else (x.get(c) or {}).get("rawvio"), g, DEV)
            std = [(x["key"], round(((x.get(c) or {}).get(kind) or {}).get("ate_w") or -1, 2)) for x in r["std"]["cells"]
                   if x["key"].startswith("s165506")]
            rows.append((c, kind, n, dv, med, dv2, med2, f"{pd} ({pb}/{pn})", f"{pd2} ({pb2}/{pn2})", std))
    n, dv, med = summ(hs, lambda x: x.get("GL60"))
    n2, dv2, med2 = summ(hs, lambda x: x.get("GL60"), DEV)
print(f"GL60 (OpenVINS ref): HS42 div {dv} med {med} | without dev cells (38) div {dv2} med {med2}")
print("| cond | traj | n | HS42 div | HS42 med cm | no-dev div | no-dev med | paired d_med (better/n) vs GL60 (swba) or vio same frames (raw) | same, no-dev | std60 165506 o0-o2 |")
print("|---|---|---|---|---|---|---|---|---|---|")
for x in rows:
    print("| " + " | ".join(str(v) for v in x) + " |")
for f in sys.argv[1:]:
    r = json.load(open(f))
    print("\nloops", f.split("/")[-1])
    for k, v in r["loops"].items():
        print(" ", k, {c: (e.get("loop_pct"), (e.get("swba") or {}).get("loop_pct")) for c, e in v.items()})
