#!/usr/bin/env python3
"""appnpu: VIO / control pose fields unchanged when the NPU loop path is on? Pose lines minus "map" (loop-closure map frame,
expected to differ) and minus wall-clock timing ("ms", "remap_ms") must be equal.   python vio_same.py PREFIX_A PREFIX_B"""
import json, sys


def lines(p):
    out = []
    for x in open(p):
        d = json.loads(x)
        for k in ("ms", "remap_ms", "map"):
            d.pop(k, None)
        out.append(json.dumps(d, sort_keys=True))
    return out


a, b = lines(sys.argv[1] + ".poses.jsonl"), lines(sys.argv[2] + ".poses.jsonl")
nd = sum(1 for x, y in zip(a, b) if x != y) + abs(len(a) - len(b))
print(f"{sys.argv[2]}: pose lines {len(a)} / {len(b)}, differing without map/timing {nd} ->", "VIO SAME" if nd == 0 else "VIO DIFF")
sys.exit(0 if nd == 0 else 1)
