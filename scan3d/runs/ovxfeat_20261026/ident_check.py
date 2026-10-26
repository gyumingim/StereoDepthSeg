#!/usr/bin/env python3
"""ovxfeat identity checks (writes ident/ident.txt):
 (a) reference build (ovloop 0d71a86) vs this branch, B60 (no --loop-xf): poses + map + loop log (ms excluded)
 (b) VIO fields of every XFeat / tri condition == B60 (the loop closure must never change the VIO pose)
 (c) v1 (bridge cf815d62, before the tri_one refactor) vs v2 (c9a32937): loop logs of the common XFeat conditions"""
import glob
import gzip
import json
import os
import time

A = "/home/system/scanner/tmp_ovxfeat"
RD = os.path.dirname(os.path.abspath(__file__))
K = ("t", "p", "q", "v", "T_world_cam0")
out = [time.strftime("%F %T")]


def poses(f):
    return [json.loads(x) for x in gzip.open(f, "rt")]


def strip(L):
    r = []
    for x in L:
        x = {k: v for k, v in x.items() if k != "ms"}
        if "xf" in x:
            x["xf"] = x["xf"][:3]
        r.append(x)
    return r


for a in sorted(glob.glob(f"{A}/ate_ref/B60/*.poses.jsonl.gz")):
    b = a.replace("ate_ref", "ate")
    P, Q = poses(a), poses(b)
    same = len(P) == len(Q) and all(all(x.get(k) == y.get(k) for k in K + ("map",)) for x, y in zip(P, Q))
    la = strip(json.loads(x) for x in open(a.replace(".poses.jsonl.gz", ".loop.jsonl")))
    lb = strip(json.loads(x) for x in open(b.replace(".poses.jsonl.gz", ".loop.jsonl")))
    out.append(f"(a) ref vs branch B60 {os.path.basename(a)}: {len(P)} poses+map {'IDENTICAL' if same else 'DIFFERENT'}, loop log {'IDENTICAL' if la == lb else 'DIFFERENT'}")
n_ok = n_bad = 0
bad = []
for b in sorted(glob.glob(f"{A}/ate/B60/*.poses.jsonl.gz")):
    P = poses(b)
    key = os.path.basename(b).rsplit("_", 1)[0]
    for f in sorted(glob.glob(f"{A}/ate/*/{key}_*.poses.jsonl.gz")):
        if "/B60/" in f:
            continue
        Q = poses(f)
        same = len(P) == len(Q) and all(all(x.get(k) == y.get(k) for k in K) for x, y in zip(P, Q))
        n_ok += same
        n_bad += not same
        if not same:
            bad.append(f)
out.append(f"(b) VIO fields of XFeat/tri runs vs B60 (same cell): {n_ok} IDENTICAL, {n_bad} DIFFERENT {bad[:5]}")
for a in sorted(glob.glob(f"{A}/ate_v1/*/*.loop.jsonl")):
    b = a.replace("ate_v1", "ate")
    if not os.path.exists(b):
        continue
    la = strip(json.loads(x) for x in open(a))
    lb = strip(json.loads(x) for x in open(b))
    out.append(f"(c) v1 vs v2 {os.path.basename(a)}: {'IDENTICAL' if la == lb else 'DIFFERENT'}")
os.makedirs(f"{RD}/ident", exist_ok=True)
open(f"{RD}/ident/ident.txt", "w").write("\n".join(out) + "\n")
print("\n".join(out))
