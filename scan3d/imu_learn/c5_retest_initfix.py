#!/usr/bin/env python3
"""Recompute the c5 retest column 'init time - motion (median)' with the TRUE init time from the DEBUG log
('successful initialization' frame, c5_initwin_analyse.true_init) instead of the bench t_init_clip_s (= first visual
update). still_left_s as in c5_retest_analyse (coarse onset - start_off). Prints per-cell old/new medians as JSON.

  ../venv_ffs/bin/python imu_learn/c5_retest_initfix.py runs/c5_retest
"""
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import c5_initwin_analyse as iw  # noqa: E402
from c5_retest_analyse import NAME, STILL  # noqa: E402

d = Path(sys.argv[1])
old, new = defaultdict(list), defaultdict(list)
for f in sorted(d.glob('*.json')):
    m = NAME.match(f.stem)
    if not m:
        continue
    j = json.load(open(f))
    ms = json.load(open(STILL / f"{j['rec']}_still.json"))['motion_start_s']
    sl = ms - j['start_off']
    key = f"{'L2' if m[1] else 'L0'}|S{m[3]}|{m[4]}"
    if j.get('t_init_clip_s') is not None:
        old[key].append(j['t_init_clip_s'] - sl)
    ti = iw.true_init(str(f)[:-5])
    if ti is not None:
        new[key].append(ti - iw.first_pose_t(str(f)[:-5]) - sl)
out = {k: dict(old=round(float(np.median(old[k])), 2) if old[k] else None,
               new=round(float(np.median(new[k])), 2) if new[k] else None, n_init=len(new[k])) for k in sorted(set(old) | set(new))}
print(json.dumps(out, indent=1))
