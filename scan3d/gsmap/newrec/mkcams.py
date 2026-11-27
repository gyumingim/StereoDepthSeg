#!/usr/bin/env python3
"""Keyframe cameras (dump_ba/cams.npz) for the FFS pair planner from a reduced scan (run_scanner --stop-after reduce) + VIO poses.
No BA (VIO poses), so it works on any new recording without 121801 paths. (Same rule as gsmap/dryrun30/dryrun30.py make_cams.)

  python mkcams.py RED_KF_DIR REC VIO OUT_RUN [--subset N --plan PLAN.json]
--subset N: keep N keyframes evenly spread over those that have >= 1 strict partner in PLAN.json (rehearsal FFS on a few keyframes).
"""
import argparse, gzip, json, time
from pathlib import Path

import numpy as np


def poses(p):
    t, T = [], []
    for line in gzip.open(p, "rt"):
        d = json.loads(line)
        if d.get("type") == "pose" and d.get("T_world_cam0") and d.get("status") == "tracking":
            t.append(int(d["t_ns"])); T.append(np.array(d["T_world_cam0"], float).reshape(4, 4))
    return np.array(t), np.array(T)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("red"); ap.add_argument("rec"); ap.add_argument("vio"); ap.add_argument("out")
    ap.add_argument("--subset", type=int, default=0); ap.add_argument("--plan", default="")
    a = ap.parse_args(); red, rec, out = Path(a.red), Path(a.rec), Path(a.out)
    man = json.loads((red / "manifest.json").read_text()); kfl = [int(x) for x in man["keyframes"]]
    if a.subset:
        pl = json.loads(Path(a.plan).read_text())["chosen"]["plan"]
        ok = [f for f in kfl if any(q[1] != "fallback" for q in pl.get(str(f), []))]
        kfl = [ok[int(i)] for i in np.linspace(0, len(ok) - 1, min(a.subset, len(ok))).round().astype(int)]
    g = np.load(rec / "lensonly" / "gt.npz"); t_ns = g["t_ns"]; K = g["K"].copy()
    tv, Tv = poses(a.vio); T = []; miss = []
    for f in kfl:
        k = int(np.argmin(np.abs(tv - t_ns[f])))
        if abs(tv[k] - t_ns[f]) < 8e6:
            T.append(Tv[k])
        else:
            miss.append(f); T.append(None)
    keep = [i for i, x in enumerate(T) if x is not None]
    kfl = [kfl[i] for i in keep]; T = np.array([T[i] for i in keep])
    (out / "dump_ba").mkdir(parents=True, exist_ok=True)
    K = K.copy(); K[:2] *= 0.5                       # 960x540 scale (= gsplat dump convention: cams K is the 960 camera, Kr = square-pixel rectified)
    Kr = np.array([[K[0, 0], 0, K[0, 2]], [0, K[0, 0], K[1, 2]], [0, 0, 1.0]])
    np.savez(out / "dump_ba" / "cams.npz", kf=np.array(kfl), T=T, t_ns=t_ns[kfl], Kr=Kr, K=K, dist=g["dist"], size=np.array([960, 540]),
             made=np.array(time.strftime("%F %T")), poses=np.array(f"VIO {a.vio} (no BA)"))
    info = dict(made=time.strftime("%F %T"), red=str(red), rec=str(rec), vio=str(a.vio), n_kf=len(kfl), dropped_no_vio_pose=miss, subset=a.subset)
    (out / "dump_ba" / "cams_info.json").write_text(json.dumps(info, indent=1)); print(json.dumps(info))


if __name__ == "__main__":
    main()
