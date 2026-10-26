#!/usr/bin/env python3
"""gyrocheck independence check: hash of TRACKING poses (absolute t_ns + T_world_cam0 + p) per run.
Same hash = same trajectory after init (shifted start did not change the run) -> 1 vote."""
import sys, json, gzip, hashlib, glob, os
from pathlib import Path
OUT = Path("/home/system/scanner/tmp_gyrocheck/ate")
res = {}
for f in sorted(glob.glob(str(OUT / "*" / "*.poses.jsonl.gz"))):
    n = Path(f).name.replace(".poses.jsonl.gz", "")
    h = hashlib.sha256(); nt = 0; t_first = None
    for l in gzip.open(f, "rt"):
        m = json.loads(l)
        if m.get("status") != "tracking" or not m.get("T_world_cam0"):
            continue
        if t_first is None: t_first = m["t_ns"]
        h.update(json.dumps([m["t_ns"], m["T_world_cam0"], m.get("p")]).encode()); nt += 1
    res[n] = dict(track_hash=h.hexdigest()[:16] if nt else None, n_track=nt, t_first_track_ns=t_first)
json.dump(res, open(sys.argv[1], "w"), indent=1)
for n, r in res.items(): print(n, r["track_hash"], r["n_track"], r["t_first_track_ns"])
