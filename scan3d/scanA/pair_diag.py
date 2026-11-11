#!/usr/bin/env python3
"""ffs-views 11-11: why do keyframes get no FFS partner? Pose geometry only (no images, no GPU).
For every keyframe of a BA pose file, count partner keyframes that survive each score_pair test in order
(dt, baseline, perp, rotation, roll) for a given MSConfig variant, and the first test that kills all of them.
  pair_diag.py POSES.jsonl.gz [--dt-max S] [--b-min M] [--b-max M] [--min-perp DEG] [--max-rot DEG] [--json OUT]
"""
import argparse
import gzip
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import motion_stereo as ms  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("poses")
    ap.add_argument("--dt-min", type=float, default=None)
    ap.add_argument("--dt-max", type=float, default=None)
    ap.add_argument("--b-min", type=float, default=None)
    ap.add_argument("--b-max", type=float, default=None)
    ap.add_argument("--min-perp", type=float, default=None)
    ap.add_argument("--max-rot", type=float, default=None)
    ap.add_argument("--json", default="")
    a = ap.parse_args()
    cfg = ms.MSConfig()
    for k, v in (("dt_min_s", a.dt_min), ("dt_max_s", a.dt_max), ("b_min_m", a.b_min), ("b_max_m", a.b_max),
                 ("min_perp_deg", a.min_perp), ("max_rot_deg", a.max_rot)):
        if v is not None:
            setattr(cfg, k, v)
    P = [json.loads(l) for l in gzip.open(a.poses, "rt")]
    P = [d for d in P if d.get("type") == "pose" and d.get("T_world_cam0")]
    t = np.array([d["t_ns"] for d in P]) * 1e-9
    T = np.array([np.array(d["T_world_cam0"], float).reshape(4, 4) for d in P])
    n = len(P)
    tests = ("dt", "baseline", "forward", "rotation", "roll")
    surv = np.zeros((n, len(tests) + 1), int)
    geo = []
    for i in range(n):
        for j in range(n):
            if j == i:
                continue
            dt = abs(t[i] - t[j])
            g = ms.pair_geometry(*ms.rel_pose(T[i], T[j]))
            checks = (cfg.dt_min_s <= dt <= cfg.dt_max_s, cfg.b_min_m <= g["B"] <= cfg.b_max_m, g["perp_deg"] >= cfg.min_perp_deg,
                      g["rot_deg"] <= cfg.max_rot_deg, abs(g["roll_deg"]) <= cfg.max_roll_deg)
            surv[i, 0] += 1
            for k, c in enumerate(checks):
                if not c:
                    break
                surv[i, k + 1] += 1
            if g["rot_deg"] <= 25 and 0.02 <= g["B"] <= 0.40:
                geo.append((i, dt, g["B"], g["perp_deg"], g["rot_deg"]))
    have = surv[:, -1] > 0
    first_kill = {}
    for i in np.flatnonzero(~have):
        k = int(np.argmax(surv[i, 1:] == 0))
        first_kill[tests[k]] = first_kill.get(tests[k], 0) + 1
    out = dict(poses=a.poses, n_kf=n, cfg=dict(dt=[cfg.dt_min_s, cfg.dt_max_s], b=[cfg.b_min_m, cfg.b_max_m], min_perp=cfg.min_perp_deg,
                                               max_rot=cfg.max_rot_deg, max_roll=cfg.max_roll_deg),
               n_kf_with_partner=int(have.sum()), frac_with_partner=float(have.mean()),
               no_partner_first_failed_test=first_kill,
               median_survivors_after_each_test={tt: float(np.median(surv[:, k + 1])) for k, tt in enumerate(tests)},
               kf_with_any_after_each_test={tt: int((surv[:, k + 1] > 0).sum()) for k, tt in enumerate(tests)})
    G = np.array([x[1:] for x in geo]) if geo else np.zeros((0, 4))
    if len(G):   # pairs with rot <= 25 deg and B 2-40 cm: where are they in dt and perp?
        out["near_pairs_rot25_B2_40"] = dict(n=len(G), dt_p10_p50_p90=np.percentile(G[:, 0], [10, 50, 90]).round(2).tolist(),
                                             perp_p10_p50_p90=np.percentile(G[:, 2], [10, 50, 90]).round(1).tolist(),
                                             frac_perp_ge40=float((G[:, 2] >= 40).mean()), frac_dt_le2p5=float((G[:, 0] <= 2.5).mean()))
    s = json.dumps(out, indent=1)
    print(s)
    if a.json:
        Path(a.json).write_text(s)


if __name__ == "__main__":
    main()
