#!/usr/bin/env python3
"""fixes 11-13 unit 15b: small-piece vertices at several component-size thresholds (far_eval rule: welded 0.1 mm), all and
beyond 1.6 m camera depth (far_eval rule, BASE kf_poses). CPU, read only. Usage: u15b_thresholds.py OUT_JSON NAME=MESH_DIR ..."""
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import far_eval as fe  # noqa: E402

TH = (500, 2000, 5000, 10000, 20000)
out = {"made": time.strftime("%Y-%m-%d %H:%M:%S"), "thresholds": TH, "rows": {}}
for arg in sys.argv[2:]:
    name, d = arg.split("=", 1)
    X, C, F = fe.read_mesh(Path(d) / "mesh.ply")
    vsz = fe.comp_size(X, F)[0]
    P = np.load(Path(d) / "kf_poses.npz")
    W, H = (int(v) for v in P["size"])
    cd = fe.cam_depth(X, P["T"], float(P["f"]), float(P["cx"]), float(P["cy"]), W, H)
    g16 = np.isfinite(cd) & (cd > 1.6)
    row = {f"lt{t}": int((vsz < t).sum()) for t in TH}
    row.update({f"gt1p6_lt{t}": int(((vsz < t) & g16).sum()) for t in TH})
    out["rows"][name] = row
    print(name, json.dumps(row), flush=True)
Path(sys.argv[1]).write_text(json.dumps(out, indent=1))
