#!/usr/bin/env python3
"""lgopt: new bridge (build_lgopt, xf_tri_mode default 0) + lgopt runner/server with the OLD xfopt lens == xfopt results
(134550 o0: B60 vs xfopt B60, L vs xfopt X1Lt3). Compares kf / loop lines without timing fields, and poses."""
import gzip, json, sys
from pathlib import Path
A = Path("/home/system/scanner/tmp_lgopt/ident"); B = Path("/home/system/scanner/tmp_xfopt/ate")
def lines(p):
    out = []
    for x in open(p):
        d = json.loads(x)
        d.pop("ms", None)
        if "xf" in d and isinstance(d["xf"], list):
            d["xf"] = d["xf"][:3]
        out.append(d)
    return out
def poses(p):
    return [(m.get("t_ns"), m.get("T_world_cam0"), (m.get("map") or {}).get("p")) for m in (json.loads(x) for x in gzip.open(p, "rt"))]
bad = 0
for a, b in (("B60/134550_o0_B60", "B60/134550_o0_B60"), ("L/134550_o0_L", "X1Lt3/134550_o0_X1Lt3")):
    la, lb = lines(A / f"{a}.loop.jsonl"), lines(B / f"{b}.loop.jsonl")
    pa, pb = poses(A / f"{a}.poses.jsonl.gz"), poses(B / f"{b}.poses.jsonl.gz")
    same = la == lb and pa == pb
    bad += not same
    print(a, "vs", b, "IDENTICAL" if same else f"DIFF (loop lines {len(la)}/{len(lb)} eq {la == lb}, poses eq {pa == pb})")
print("RESULT", "PASS" if not bad else "FAIL")
sys.exit(bad)
