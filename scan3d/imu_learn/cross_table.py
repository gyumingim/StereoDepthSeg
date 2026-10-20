#!/usr/bin/env python3
"""Collect runs/x_<calib>[_s<k>]_on_<ruler>_<rec>_s<eval seed>/result.json → table: calibration trained on GT A (3 training
seeds averaged) × ruler B, mean held-out 3 s position error (cm) over the held-out recordings → runs/spline_2x2.json"""
import glob
import json
import re
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent / "runs"
cell = defaultdict(lambda: defaultdict(list))        # (calib, ruler) → rec → [pos per training seed]
rot = defaultdict(lambda: defaultdict(list))
none = defaultdict(lambda: defaultdict(list))
for f in glob.glob(str(ROOT / "x_*_on_*" / "result.json")):
    m = re.match(r"x_(rs|spl|splimu)(?:_s\d)?_on_(rs|spl|rsraw)_(hs_\d+_\d+)_s\d+$", Path(f).parent.name)
    if not m:
        continue
    c, g, rec = m.groups()
    r = json.load(open(f))
    cell[(c, g)][rec].append(r["calib"]["val"]["pos_rms_cm"])
    rot[(c, g)][rec].append(r["calib"]["val"]["rot_end_deg"])
    none[g][rec].append(r["none"]["val"]["pos_rms_cm"])
out = {}
recs = sorted({rec for d in cell.values() for rec in d})
for (c, g), d in sorted(cell.items()):
    per = {rec: float(np.mean(v)) for rec, v in d.items()}
    spread = {rec: float(np.ptp(v)) for rec, v in d.items()}
    out[f"{c}@{g}"] = dict(mean=round(float(np.mean(list(per.values()))), 4), per_rec={k: round(v, 3) for k, v in per.items()},
                           seed_range={k: round(v, 3) for k, v in spread.items()}, n_seeds={k: len(v) for k, v in d.items()},
                           rot_deg=round(float(np.mean([np.mean(v) for v in rot[(c, g)].values()])), 4))
for g, d in none.items():
    out[f"none@{g}"] = dict(mean=round(float(np.mean([np.mean(v) for v in d.values()])), 4),
                            per_rec={k: round(float(np.mean(v)), 3) for k, v in d.items()})
json.dump(out, open(ROOT / "spline_2x2.json", "w"), indent=1)
print("recordings:", recs)
print(f"{'calibration \\ ruler':22s}" + "".join(f"{g:>10s}" for g in ("rs", "spl", "rsraw")))
for c in ("none", "rs", "spl", "splimu"):
    print(f"{c:22s}" + "".join(f"{out.get(f'{c}@{g}', {}).get('mean', float('nan')):10.3f}" for g in ("rs", "spl", "rsraw")))
for k, v in out.items():
    print(k, v)
