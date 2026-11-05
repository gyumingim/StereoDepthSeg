#!/usr/bin/env python3
"""Export a board GT variant (REC/<gt>/gt.npz, ok frames) as a bridge-format poses.jsonl.gz (T_world_cam0 = T_board_cam).
  python scan3d/scanA/gt_poses.py REC GT OUT.jsonl.gz"""
import gzip, json, sys
import numpy as np

rec, gt, out = sys.argv[1:4]
g = np.load(f"{rec}/{gt}/gt.npz", allow_pickle=True)
with gzip.open(out, "wt") as f:
    for i in range(len(g["t_ns"])):
        if g["ok"][i]:
            T = np.eye(4); T[:3, :3] = g["R_bc"][i]; T[:3, 3] = g["p_bc"][i]
            f.write(json.dumps(dict(type="pose", status="tracking", t_ns=int(g["t_ns"][i]), frame=i, T_world_cam0=T.ravel().tolist())) + "\n")
print(out, int(g["ok"].sum()))
