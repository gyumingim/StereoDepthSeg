#!/usr/bin/env python3
"""lgin 2026-10-29 (exploratory, no verdict): how large are the revisits in the recordings we already have?
Reads the keyframe lines (p_v, q_v, path, t_ns) of existing PC loop logs (*.loop.jsonl of earlier runs, any condition) and, per
recording (longest run found), reports: VIO duration covered, total path, extent (largest distance between two keyframes),
largest 'revisit' = pair of keyframes >= 3 s apart, < 0.5 m apart and optical axes < 30 deg (VIO poses, drift included = estimate),
with the path travelled between them, and the largest path_between of accepted loops in that log.
  revisit_list.py OUT.json [log dirs...]"""
import glob
import json
import os
import re
import sys
import time

import numpy as np

T_IC = np.array([[0.020201228802, -0.99977773579, -0.00603236055105], [-0.999795522195, -0.0201953741747, -0.00102988426744],
                 [0.000907829582544, 0.00605193199493, -0.999981274807]])  # c4f60 T_imu_cam rotation (HS lens rotation assumed similar)


def qmat(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rec_of(path):
    b = os.path.basename(path)
    m = re.search(r"(\d{6})", b)
    return m.group(1) if m else None


def main():
    out = sys.argv[1]
    dirs = sys.argv[2:]
    best = {}
    for d in dirs:
        for f in glob.glob(os.path.join(d, "**", "*.loop.jsonl"), recursive=True):
            k = rec_of(f)
            if not k:
                continue
            kf, loops = [], []
            for s in open(f, errors="replace"):
                if not s.startswith("{"):
                    continue
                try:
                    l = json.loads(s)
                except Exception:
                    continue
                if l.get("type") == "kf" and l.get("session", 0) == 0:
                    kf.append(l)
                elif l.get("type") == "loop":
                    loops.append(l)
            if len(kf) < 5:
                continue
            dur = (kf[-1]["t_ns"] - kf[0]["t_ns"]) / 1e9
            if k not in best or dur > best[k]["dur"]:
                best[k] = dict(dur=dur, f=f, kf=kf, loops=loops)
    rows = []
    for k, b in best.items():
        kf = b["kf"]
        P = np.array([l["p_v"] for l in kf])
        A = np.array([qmat(l["q_v"]) @ T_IC[:, 2] for l in kf])
        t = np.array([l["t_ns"] for l in kf]) / 1e9
        path = np.array([float(l.get("path") or 0) for l in kf])
        D = np.linalg.norm(P[:, None] - P[None], axis=-1)
        C = np.clip(A @ A.T, -1, 1)
        dt = t[None] - t[:, None]
        ok = (dt >= 3) & (D < 0.5) & (np.degrees(np.arccos(C)) < 30)
        rp = np.where(ok, path[None] - path[:, None], -1)
        i, j = np.unravel_index(np.argmax(rp), rp.shape)
        rows.append(dict(rec=k, log=b["f"], secs=round(b["dur"], 1), n_kf=len(kf), path_m=round(float(path[-1] - path[0]), 2),
                         extent_m=round(float(D.max()), 2),
                         revisit_max_path_m=round(float(rp[i, j]), 2) if rp[i, j] > 0 else 0.0,
                         revisit_max_dt_s=round(float(dt[i, j]), 1) if rp[i, j] > 0 else 0.0,
                         n_revisit_pairs=int(ok.sum()),
                         loops=len(b["loops"]), loop_path_between_max_m=round(max([l.get("path_between") or 0 for l in b["loops"]], default=0), 2)))
    rows.sort(key=lambda r: -r["revisit_max_path_m"])
    json.dump(dict(made=time.strftime("%F %T"), dirs=dirs, rule="kf pairs dt>=3 s, |p|<0.5 m, axes<30 deg (VIO, estimate)", rows=rows),
              open(out, "w"), indent=1)
    for r in rows[:40]:
        print(r["rec"], r["secs"], r["path_m"], r["extent_m"], r["revisit_max_path_m"], r["revisit_max_dt_s"], r["loops"],
              r["loop_path_between_max_m"], os.path.relpath(r["log"], os.path.expanduser("~/scanner")))


if __name__ == "__main__":
    main()
