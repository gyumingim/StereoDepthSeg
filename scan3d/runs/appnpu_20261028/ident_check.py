#!/usr/bin/env python3
"""appnpu: flags-off identity. Pose lines (every field) and loop.jsonl lines (timing fields "ms" / "cap_ms" / "xf" ms removed)
of two replays must be equal.   python ident_check.py PREFIX_A PREFIX_B"""
import json, sys


def loop_lines(p):
    out = []
    for x in open(p):
        d = json.loads(x)
        for k in ("ms", "cap_ms"):
            d.pop(k, None)
        if isinstance(d.get("xf"), list):
            d["xf"] = d["xf"][:3]
        out.append(d)
    return out


a, b = sys.argv[1], sys.argv[2]
def pose_lines(p):  # pose lines: wall-clock timing fields "ms" / "remap_ms" removed, everything else compared
    out = []
    for x in open(p):
        d = json.loads(x)
        for k in ("ms", "remap_ms"):
            d.pop(k, None)
        out.append(json.dumps(d, sort_keys=True))
    return out


pa, pb = pose_lines(a + ".poses.jsonl"), pose_lines(b + ".poses.jsonl")
la, lb = loop_lines(a + ".loop.jsonl"), loop_lines(b + ".loop.jsonl")
ndiff = sum(1 for x, y in zip(pa, pb) if x != y) + abs(len(pa) - len(pb))
ldiff = sum(1 for x, y in zip(la, lb) if x != y) + abs(len(la) - len(lb))
nmap = sum(1 for x in pa if '"map"' in x)
print(f"poses {len(pa)} / {len(pb)} lines, differing {ndiff} (lines with map {nmap}); loop {len(la)} / {len(lb)} lines, differing {ldiff}")
print("RESULT", "IDENTICAL" if ndiff == 0 and ldiff == 0 else "DIFF")
sys.exit(0 if ndiff == 0 and ldiff == 0 else 1)
