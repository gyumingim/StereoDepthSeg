#!/usr/bin/env python3
"""gpuba 11-11 task 2 (README 8): repeated from-scratch kf_ba runs -> are keypoints/descriptors, points and poses bit-identical?
  python det_compare.py DIR  (DIR = runs/laptop/<rid>/<tag>, run dirs old1.. new1 fix1.. with feats.npz, ba.json, poses_ba.jsonl.gz)"""
import gzip, json, sys
from pathlib import Path
import numpy as np

D = Path(sys.argv[1])
runs = sorted(p.name for p in D.iterdir() if (p / "ba.json").exists())
groups = {}
for r in runs:
    groups.setdefault(r.rstrip("0123456789"), []).append(r)
res = {}
for g, rs in groups.items():
    info = []
    for r in rs:
        b = json.loads((D / r / "ba.json").read_text())
        f = np.load(D / r / "feats.npz")
        P = np.array([json.loads(l)["T_world_cam0"] for l in gzip.open(D / r / "poses_ba.jsonl.gz", "rt")])
        info.append(dict(run=r, f=f, P=P, sim3=b["sim3_scale"], points=b["points"], obs=b["obs"], t=b["runtime_s"] - b["timing"].get("images_s", 0),
                         t_feat=b["timing"].get("features_s"), t_match=b["timing"].get("matching_s"), t_ba=b["timing"].get("ba_s")))
    base = info[0]
    n_img_diff = []
    for o in info[1:]:
        n = 0
        for k in base["f"].files:
            if k.startswith("kp_") and not (np.array_equal(base["f"][k], o["f"][k]) and np.array_equal(base["f"]["de_" + k[3:]], o["f"]["de_" + k[3:]])):
                n += 1
        n_img_diff.append(n)
    res[g] = dict(runs=[i["run"] for i in info], n_images=sum(k.startswith("kp_") for k in base["f"].files),
                  images_differ_vs_first=n_img_diff, sim3=[i["sim3"] for i in info], points=[i["points"] for i in info], obs=[i["obs"] for i in info],
                  pose_maxdiff_vs_first=[float(np.abs(base["P"] - o["P"]).max()) if base["P"].shape == o["P"].shape else None for o in info[1:]],
                  t_s=[round(i["t"], 1) for i in info], t_feat=[i["t_feat"] for i in info], t_match=[i["t_match"] for i in info], t_ba=[i["t_ba"] for i in info])
    res[g]["bit_identical"] = len(info) > 1 and all(x == 0 for x in n_img_diff) and all(x == 0.0 for x in res[g]["pose_maxdiff_vs_first"]) \
        and len(set(res[g]["sim3"])) == 1
    print(g, json.dumps(res[g]))
(D / "det_compare.json").write_text(json.dumps(res, indent=1))
