#!/usr/bin/env python3
"""xfopt identity checks (writes ident/ident.txt):
 (a) reference build (cb746c4 = lens05 + ovxfeat patch) vs this branch, B60r (no log_cand, no --loop-xf): poses + map + loop log (ms excluded)
 (b) this branch B60 (log_cand=1) vs B60r: poses + map + loop lines identical (log_cand changes no decision)
 (c) VIO fields of every XFeat condition == B60 (same cell): the loop closure never changes the VIO pose"""
import glob
import gzip
import json
import os
import time

A = "/home/system/scanner/tmp_xfopt"
RD = os.path.dirname(os.path.abspath(__file__))
K = ("t", "p", "q", "v", "T_world_cam0")
out = [time.strftime("%F %T")]


def poses(f):
    return [json.loads(x) for x in gzip.open(f, "rt")]


def strip(L, drop_cx=False):
    r = []
    for x in L:
        x = {k: v for k, v in x.items() if k != "ms" and not (drop_cx and k == "cx")}
        r.append(x)
    return r


def same_poses(P, Q, keys):
    return len(P) == len(Q) and all(all(x.get(k) == y.get(k) for k in keys) for x, y in zip(P, Q))


n = [0, 0]
for a in sorted(glob.glob(f"{A}/ident/ref/B60r/*.poses.jsonl.gz")):
    b = a.replace("/ref/", "/new/")
    if not os.path.exists(b):
        continue
    sp = same_poses(poses(a), poses(b), K + ("map",))
    la = strip(json.loads(x) for x in open(a.replace(".poses.jsonl.gz", ".loop.jsonl")))
    lb = strip(json.loads(x) for x in open(b.replace(".poses.jsonl.gz", ".loop.jsonl")))
    ok = sp and la == lb
    n[0 if ok else 1] += 1
    out.append(f"(a) ref vs new B60r {os.path.basename(a)}: poses+map {'IDENTICAL' if sp else 'DIFFERENT'}, loop log {'IDENTICAL' if la == lb else 'DIFFERENT'}")
out.append(f"(a) total {n[0]} IDENTICAL, {n[1]} DIFFERENT")
n = [0, 0]
for a in sorted(glob.glob(f"{A}/ident/new/B60r/*.poses.jsonl.gz")):
    b = a.replace("/ident/new/B60r/", "/ate/B60/").replace("_B60r.", "_B60.")
    if not os.path.exists(b):
        continue
    sp = same_poses(poses(a), poses(b), K + ("map",))
    la = strip((json.loads(x) for x in open(a.replace(".poses.jsonl.gz", ".loop.jsonl"))), True)
    lb = strip((json.loads(x) for x in open(b.replace(".poses.jsonl.gz", ".loop.jsonl"))), True)
    ok = sp and la == lb
    n[0 if ok else 1] += 1
    if not ok:
        out.append(f"(b) DIFFERENT {os.path.basename(a)} poses {sp} loop {la == lb}")
out.append(f"(b) B60 (log_cand=1) vs B60r: {n[0]} IDENTICAL, {n[1]} DIFFERENT (poses+map, loop log without ms/cx)")
n_ok = n_bad = 0
bad = []
for b in sorted(glob.glob(f"{A}/ate/B60/*.poses.jsonl.gz")):
    P = poses(b)
    key = os.path.basename(b).rsplit("_", 1)[0]
    for f in sorted(glob.glob(f"{A}/ate/*/{key}_*.poses.jsonl.gz")):
        if "/B60/" in f:
            continue
        same = same_poses(P, poses(f), K)
        n_ok += same
        n_bad += not same
        if not same:
            bad.append(f)
out.append(f"(c) VIO fields of other conditions vs B60 (same cell): {n_ok} IDENTICAL, {n_bad} DIFFERENT {bad[:5]}")
os.makedirs(f"{RD}/ident", exist_ok=True)
open(f"{RD}/ident/ident.txt", "w").write("\n".join(out) + "\n")
print("\n".join(out[-6:]))
