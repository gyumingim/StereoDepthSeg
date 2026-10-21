#!/usr/bin/env python3
"""Self-check for docs/setdown_failure.html: re-derive the key numbers straight from the raw logs / IMU CSV (not via
endstill_setdown.py): first ZUPT failure streak (>= 5 frames) after the still starts (117 / 132 s), mean features in MSCKF+SLAM updates after the
set-down, VIO |v| at the end, impact peak |a| - g, max |a| / |w| per axis vs range. Bridge->recording time via pose lines.

  ../venv_ffs/bin/python imu_learn/endstill_setdown_check.py
"""
import csv
import gzip
import json
import re

import numpy as np

ANSI = re.compile(r"\x1b\[[0-9;]*m")
RUNS = "runs/endstill_vio/"
CLAIM = {"224323": dict(zfail=133.0, impact=113.84, slam_zero_from=115.0, still_from=117.0),
         "223815": dict(zfail=136.0, impact=130.65, slam_zero_from=137.0, still_from=132.0)}


def imu_t0(rec):
    with open(f"recordings/{rec}/imu.csv") as f:
        for r in csv.reader(f):
            if r and r[0] in ("1", "4", "16", "35"):
                return int(r[1])


for rec6, cl in CLAIM.items():
    rec = f"hs_20261020_{rec6}"
    t0 = imu_t0(rec)
    for k in range(3):
        p = f"{RUNS}{rec6}_loose_raw_o{k}"
        P = [json.loads(x) for x in gzip.open(p + ".poses.jsonl.gz", "rt")]
        off = float(np.median([m["t"] - (m["t_ns"] - t0) / 1e9 for m in P]))
        fr, cur = [], None
        for line in gzip.open(p + ".stderr.log.gz", "rt", errors="replace"):
            line = ANSI.sub("", line)
            if "[DIAGT]" in line:
                cur = dict(t=float(line.split("]")[-1]) - off, z=None, f=0)
                fr.append(cur)
            elif cur is not None and "[ZUPT]: failed disparity" in line:
                cur["z"] = False
            elif cur is not None and "[ZUPT]: accepted" in line:
                cur["z"] = True
            elif cur is not None and ("for MSCKF update (" in line or "for SLAM update (" in line):
                cur["f"] += int(re.search(r"\((\d+) feats\)", line)[1])
        z = [f for f in fr if f["t"] > cl["still_from"]]          # search from the start of the post-set-down still
        first = next((z[i]["t"] for i in range(len(z) - 5) if all(x["z"] is False for x in z[i:i + 5])), None)
        feats_after = np.mean([f["f"] for f in fr if f["t"] > cl["slam_zero_from"]])
        v_end = float(np.linalg.norm(P[-1]["v"])) if P[-1].get("v") else None
        print(f"{rec6} o{k}: first ZUPT-fail streak {first:.2f} s (claim {cl['zfail']}), mean update feats after "
              f"{cl['slam_zero_from']} s = {feats_after:.2f}, |v| last pose {v_end:.2f} m/s")
    a, g = [], []
    with open(f"recordings/{rec}/imu.csv") as f:
        for r in csv.reader(f):
            if r and r[0] == "35" and len(r) >= 5:
                a.append([int(r[1])] + [float(x) for x in r[2:5]])
            elif r and r[0] == "16" and len(r) >= 5:
                g.append([int(r[1])] + [float(x) for x in r[2:5]])
    a, g = np.array(a), np.array(g)
    ta = (a[:, 0] - t0) / 1e9
    dev = np.linalg.norm(a[:, 1:], axis=1) - 9.80665
    m = (ta > cl["impact"] - 3) & (ta < cl["impact"] + 3)
    i = np.argmax(dev * m)
    print(f"{rec6}: impact peak {ta[i]:.2f} s |a|-g {dev[i]:.1f} m/s2 (claim {cl['impact']}); max |a| axis "
          f"{np.abs(a[:, 1:]).max():.1f} / 156.9, max |w| axis {np.abs(g[:, 1:]).max():.2f} / 34.9")
