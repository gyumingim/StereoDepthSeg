#!/usr/bin/env python3
"""ovloop: check that the published map frame never jumps and that the control (VIO / pred) fields are untouched.
  python3 check_smooth.py OUT.jsonl [--v 0.10] [--w 3.0] [--ref REF.jsonl]
- per consecutive pose line (same session): |delta(p_map - Rz(dyaw) ... )| measured as the change of the map position of the
  CURRENT body caused by T_map_odom alone: |(Rz(y1) p1 + t1) - (Rz(y0) p1 + t0)| <= v * dt (+1e-6), |y1 - y0| <= w * dt
- pred lines: T_map_odom only changes at frames (between two pose lines the pred lines carry the same yaw / t)
- --ref: the VIO fields (everything except "map", ms timing) of pose lines identical to REF (loop off run)
"""
import json
import math
import sys

args = sys.argv[1:]
v, w, ref = 0.10, 3.0, None
f = args[0]
for i, a in enumerate(args):
    if a == "--v":
        v = float(args[i + 1])
    if a == "--w":
        w = float(args[i + 1])
    if a == "--ref":
        ref = args[i + 1]


def Rz(y, p):
    c, s = math.cos(y), math.sin(y)
    return [c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]]


poses, preds = [], []
for line in open(f):
    if '"type":"pose"' in line:
        poses.append(json.loads(line))
    elif '"type":"pred"' in line:
        preds.append(json.loads(line))
worst_v, worst_w, nviol, n = 0.0, 0.0, 0, 0
prev = None
for m in poses:
    mm = m.get("map")
    if not mm:
        prev = None
        continue
    if prev is not None and prev["map"]["session"] == mm["session"]:
        dt = m["t"] - prev["t"]
        p1 = m["p"]
        a = Rz(mm["yaw"], p1)
        b = Rz(prev["map"]["yaw"], p1)
        d = math.dist([a[k] + mm["t"][k] for k in range(3)], [b[k] + prev["map"]["t"][k] for k in range(3)])
        dy = abs(math.remainder(mm["yaw"] - prev["map"]["yaw"], 2 * math.pi))
        n += 1
        rv = d / max(dt, 1e-9)
        rw = math.degrees(dy) / max(dt, 1e-9)
        worst_v, worst_w = max(worst_v, rv), max(worst_w, rw)
        if d > v * dt + 1e-6 or math.degrees(dy) > w * dt + 1e-6:
            nviol += 1
    prev = m
print(f"pose lines {len(poses)} with map {sum(1 for m in poses if m.get('map'))}, steps {n}: max map-correction speed "
      f"{worst_v:.4f} m/s (limit {v}), max yaw rate {worst_w:.3f} deg/s (limit {w}), violations {nviol}")
# pred lines: T between frames constant
bad = 0
for i in range(1, len(preds)):
    a, b = preds[i - 1].get("map"), preds[i].get("map")
    if a and b and preds[i - 1]["t_cam_ns"] == preds[i]["t_cam_ns"] and (a["yaw"] != b["yaw"] or a["t"] != b["t"]):
        bad += 1
print(f"pred lines {len(preds)} with map {sum(1 for m in preds if m.get('map'))}: T changed between frames {bad}")
if ref:
    EX = {"map", "ms", "remap_ms", "wait_ms"}
    R = [json.loads(x) for x in open(ref) if '"type":"pose"' in x]
    same = sum(1 for a, b in zip(R, poses) if {k: x for k, x in a.items() if k not in EX} == {k: x for k, x in b.items() if k not in EX})
    print(f"VIO fields vs ref: {same}/{len(R)} identical (poses {len(poses)})")
sys.exit(1 if nviol or bad else 0)
