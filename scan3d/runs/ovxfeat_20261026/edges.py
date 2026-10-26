#!/usr/bin/env python3
"""ovxfeat: loop-edge errors vs GT per condition from score.json: counts > 10 cm or > 3 deg (= 'false', ovloop rule),
borderline 3-10 cm (position) separately, and the worst edges.  python3 edges.py SET(std|hs) [conds]"""
import json
import sys
from collections import defaultdict

S = json.load(open("/home/system/scanner/tmp_ovxfeat/ate/score.json"))["rows"]
which = sys.argv[1]
conds = sys.argv[2].split(",") if len(sys.argv) > 2 else None
sys.path.insert(0, "/home/system/scanner/StereoDepthSeg-ovxfeat/scan3d/runs/ovxfeat_20261026")
DROP = set()
if which == "hs":
    import ovxfeat_run as R
    _, DROP, _ = R.hs_cells()
agg = defaultdict(lambda: dict(cells=0, loops=0, checked=0, false=0, fpos=0, frot=0, border=0, worst=[]))
for k, v in S.items():
    if v.get("set") != which or (conds and v["cond"] not in conds):
        continue
    if v["rec"].split("/")[-1] in DROP:
        continue
    a = agg[v["cond"]]
    a["cells"] += 1
    a["loops"] += v.get("n_loop", 0)
    for e in v.get("edges", []):
        if not e.get("gt"):
            continue
        a["checked"] += 1
        a["false"] += e["false"]
        a["fpos"] += e["err_cm"] > 10
        a["frot"] += e["err_deg"] > 3
        a["border"] += 3 < e["err_cm"] <= 10
        a["worst"].append((e["err_cm"], e["err_deg"], v["name"], e["cur"], e["old"], e["n_inl"]))
for c in sorted(agg):
    a = agg[c]
    w = sorted(a["worst"], key=lambda x: -max(x[0] / 10, x[1] / 3))[:3]
    print(f"{c:7s} cells {a['cells']} loops {a['loops']} checked {a['checked']} false {a['false']} (pos>10cm {a['fpos']}, rot>3deg {a['frot']}) "
          f"border 3-10cm {a['border']} worst {w}")
