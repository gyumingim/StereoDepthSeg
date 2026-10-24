"""Is the displayed board non-square (x/y square size differ)? Scale board y by s, refit k12_notan, watch rms, fx/fy and
the tilt-group fx split. → results/aspect_scan.json (2026-10-24)"""
import json, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import calib
views, clusters, members = calib.build()
r = calib.fit(views, "k12_notan")
tilt = {}
for v, rv, tv in zip(views, r[3], r[4]):
    tilt.setdefault(v["cluster"], []).append(calib.tilt_dist(rv, tv)[0])
tilt = {c: np.mean(x) for c, x in tilt.items()}
out = []
for s in [0.990, 0.995, 0.998, 1.0, 1.002, 1.005, 1.010]:
    vs = []
    for v in views:
        o = v["obj"].copy(); o[:, 1] *= s
        vs.append(dict(v, obj=o))
    rr = calib.fit(vs, "k12_notan")
    lo = calib.fit([v for v in vs if tilt[v["cluster"]] < 45], "k12_notan")
    hi = calib.fit([v for v in vs if tilt[v["cluster"]] < 25 or tilt[v["cluster"]] >= 45], "k12_notan")
    row = dict(s=s, rms=float(rr[0]), fx=float(rr[1][0, 0]), fy=float(rr[1][1, 1]), fy_fx=float(rr[1][1, 1] / rr[1][0, 0]),
               fx_tilt_lt45=float(lo[1][0, 0]), fx_tilt_ge45=float(hi[1][0, 0]))
    out.append(row)
    print({k: round(x, 4) for k, x in row.items()})
json.dump(dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), rows=out), open(HERE / "results" / "aspect_scan.json", "w"), indent=1)
