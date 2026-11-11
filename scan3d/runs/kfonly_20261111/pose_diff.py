#!/usr/bin/env python3
"""kfonly 11-11 README 11: BA keyframe pose difference between two poses_ba.jsonl.gz (same frames, no alignment; both are in the
VIO world after kf_ba's Sim3). Prints position diff (mm) and rotation diff (deg): median, p90, max. Light."""
import gzip, json, sys
import numpy as np


def load(p):
    return {d["frame"]: np.array(d["T_world_cam0"], float).reshape(4, 4) for d in map(json.loads, gzip.open(p, "rt"))}


def main():
    a, b = load(sys.argv[1]), load(sys.argv[2])
    common = sorted(set(a) & set(b))
    dp = np.array([np.linalg.norm(a[k][:3, 3] - b[k][:3, 3]) * 1000 for k in common])
    dr = np.array([np.degrees(np.arccos(np.clip((np.trace(a[k][:3, :3].T @ b[k][:3, :3]) - 1) / 2, -1, 1))) for k in common])
    r = dict(n=len(common), n_a=len(a), n_b=len(b), pos_mm=dict(median=float(np.median(dp)), p90=float(np.percentile(dp, 90)), max=float(dp.max())),
             rot_deg=dict(median=float(np.median(dr)), p90=float(np.percentile(dr, 90)), max=float(dr.max())))
    print(json.dumps(r))


if __name__ == "__main__":
    main()
