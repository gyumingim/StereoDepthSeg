#!/usr/bin/env python3
"""ovxfeat (supervisor 21:03 #2): per keyframe work for the phone budget = 1 XFeat + matcher calls
(1 with the previous keyframe when xf_tri=1, + one per BoW candidate, max_cand 3). Distribution over HS 42 / std / loops
and keyframe rate (KF/s of recording time).  python3 cand_dist.py COND [OUT]"""
import collections
import glob
import json
import sys

import numpy as np

cond = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else "/home/system/scanner/tmp_ovxfeat/ate"
import re
TRI = re.search(r"t\d*$", cond) is not None
calls, rates = [], []
hist = collections.Counter()
for f in sorted(glob.glob(f"{OUT}/{cond}/*.loop.jsonl")):
    K = [json.loads(x) for x in open(f) if '"type":"kf"' in x]
    if len(K) < 2:
        continue
    for i, k in enumerate(K):
        n = len(k["cand"]) + (1 if i > 0 and TRI else 0)
        calls.append(n)
        hist[n] += 1
    rates.append(len(K) / max(1e-6, K[-1]["t"] - K[0]["t"]))
c = np.array(calls)
print(f"{cond}: {len(c)} keyframes; matcher calls per KF p50 {np.percentile(c, 50):.0f} p95 {np.percentile(c, 95):.0f} max {c.max()} "
      f"mean {c.mean():.2f}; hist {dict(sorted(hist.items()))}; KF rate per recording median {np.median(rates):.2f}/s max {max(rates):.2f}/s")
