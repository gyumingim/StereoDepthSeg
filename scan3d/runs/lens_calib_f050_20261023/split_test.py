"""Which pose property drives fx? Fit k12_notan with frontal poses (< 25 deg) + only one tilted group (by tilt, distance,
tilt axis, session). → results/split_test.json  (2026-10-24)"""
import json, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import calib
views, clusters, members = calib.build()
M = "k12_notan"
r = calib.fit(views, M)
info = {}
for v, rv, tv in zip(views, r[3], r[4]):
    t, d = calib.tilt_dist(rv, tv)
    info.setdefault(v["cluster"], []).append((t, d))
tilt = {c: np.mean([a for a, _ in x]) for c, x in info.items()}
dist = {c: np.mean([b for _, b in x]) for c, x in info.items()}
ta = json.load(open(HERE / "results" / "tiltaxis.json"))
axis = {c: ("yaw" if abs(ta[c]["yaw_deg"]) > abs(ta[c]["pitch_deg"]) else "pitch") for c in clusters if c in ta}
front = {c for c in clusters if tilt[c] < 25}
tilted = [c for c in clusters if c not in front]
groups = {
    "tilt 25-45": [c for c in tilted if tilt[c] < 45], "tilt >=45": [c for c in tilted if tilt[c] >= 45],
    "dist <0.75 m": [c for c in tilted if dist[c] < 0.75], "dist >=0.75 m": [c for c in tilted if dist[c] >= 0.75],
    "axis yaw": [c for c in tilted if axis.get(c) == "yaw"], "axis pitch": [c for c in tilted if axis.get(c) == "pitch"],
    "night 10-23": [c for c in tilted if "20261023" in c], "morning 10-24": [c for c in tilted if "20261024" in c],
}
out = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), model=M, n_front=len(front), full=calib.lens_summary(r[1], r[2]), groups={})
for g, cs in groups.items():
    vs = [v for v in views if v["cluster"] in front or v["cluster"] in cs]
    rr = calib.fit(vs, M)
    out["groups"][g] = dict(n_tilted=len(cs), **calib.lens_summary(rr[1], rr[2]), rms=float(rr[0]))
    print(f"{g:15s} n={len(cs):2d} fx {rr[1][0,0]:.1f} cx {rr[1][0,2]:.1f} cy {rr[1][1,2]:.1f} rms {rr[0]:.3f}")
# per tilted cluster: fx with frontal + that one cluster + all other tilted? -> use LOPO-like 'add one' on frontal base is too weak;
# instead residual pattern: per-pose mean residual vs tilt
json.dump(out, open(HERE / "results" / "split_test.json", "w"), indent=1)
