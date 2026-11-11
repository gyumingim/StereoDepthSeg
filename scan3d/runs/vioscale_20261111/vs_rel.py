#!/usr/bin/env python3
"""vioscale relative ruler: scale of each variant's VIO trajectory vs the baseline B, at the 324 keyframe times.

  python vs_rel.py RUNS_DIR KFMAP [--ref B] [--anchor 1.0489] > rel.json
rel = Sim3 scale (B positions -> variant positions, Umeyama) = variant length / B length. s_est = anchor x rel, anchor = s of the
B trajectory after BA on the red screen (redscreen 1.0489; BA vs VIO Sim3 0.9985 -> BA keeps the VIO scale). Why: the red ruler on raw
VIO poses is noisy (B: rms 21.9 px, halves 1.051 / 1.023 = drift + loop jumps), the relative Sim3 over 300 positions is not.
Also rel_first / rel_second (first / second half of the keyframes) to see drift.
"""
import gzip, json, sys
from pathlib import Path
import numpy as np


def poses(f):
    out = {}
    for l in gzip.open(f, "rt"):
        m = json.loads(l)
        if m.get("status") == "tracking" and m.get("T_world_cam0"):
            out[int(m["t_ns"])] = np.asarray(m["T_world_cam0"], float).reshape(4, 4)[:3, 3]
    return out


def sim3s(A, B):
    """scale of B ~ s R A + t"""
    ma, mb = A.mean(0), B.mean(0)
    U, S, Vt = np.linalg.svd((B - mb).T @ (A - ma) / len(A))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    return float(np.trace(np.diag(S) @ D) / np.mean(np.sum((A - ma) ** 2, 1)))


def main():
    runs, kfmap = Path(sys.argv[1]), sys.argv[2]
    ref = sys.argv[sys.argv.index("--ref") + 1] if "--ref" in sys.argv else "B"
    anchor = float(sys.argv[sys.argv.index("--anchor") + 1]) if "--anchor" in sys.argv else 1.0489
    kt = [json.loads(l)["t_ns"] for l in gzip.open(kfmap, "rt")]
    P0 = poses(runs / f"{ref}.poses.jsonl.gz")
    out = {}
    for f in sorted(runs.glob("*.poses.jsonl.gz")):
        c = f.name[:-len(".poses.jsonl.gz")]
        P = poses(f)
        t = [x for x in kt if x in P and x in P0]
        if len(t) < 50:
            out[c] = dict(n=len(t)); continue
        A = np.array([P0[x] for x in t]); B = np.array([P[x] for x in t])
        h = len(t) // 2
        r = sim3s(A, B)
        out[c] = dict(n=len(t), rel=round(r, 5), s_est=round(anchor * r, 4), rel_first=round(sim3s(A[:h], B[:h]), 5),
                      rel_second=round(sim3s(A[h:], B[h:]), 5))
    print(json.dumps(dict(ref=ref, anchor=anchor, kfmap=kfmap, conds=out), indent=1))


if __name__ == "__main__":
    main()
