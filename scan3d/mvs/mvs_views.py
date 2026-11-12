#!/usr/bin/env python3
"""mvs 11-12: source-view selection + depth range per keyframe from the fixed-pose sparse model (mvs_prep.py sparse960/tri).
Score of a pair = MVSNet/colmap2mvsnet rule: sum over shared 3D points of exp(-(theta - 5deg)^2 / (2 s^2)), s = 1 deg below 5 deg,
10 deg above (theta = triangulation angle of that point between the two cameras). Top N by score (score > 0).
Depth range = 1st / 99th percentile of the visible sparse points' camera depth x (0.75, 1.5), clipped to [0.15, 12] m.
Writes PREP/views.json: {name: {src, score, B_m (baseline per src), dmin, dmax, n_pts}}.
Usage: mvs_views.py PREP [--n-src 10]"""
import argparse
import json
from pathlib import Path

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("prep"); ap.add_argument("--n-src", type=int, default=10)
    a = ap.parse_args()
    import pycolmap
    P = Path(a.prep)
    r = pycolmap.Reconstruction(str(P / "sparse960" / "tri"))
    C = np.load(P / "cams.npz")
    names = [str(x) for x in C["names"]]
    cen = {n: C["T"][i][:3, 3] for i, n in enumerate(names)}
    Rwc = {n: C["T"][i][:3, :3] for i, n in enumerate(names)}
    id2n = {iid: im.name for iid, im in r.images.items()}
    vis = {n: [] for n in names}
    pts = {}
    for pid, p in r.points3D.items():
        ims = sorted({id2n[e.image_id] for e in p.track.elements})
        pts[pid] = (np.asarray(p.xyz), ims)
        for n in ims:
            vis[n].append(pid)
    idx = {n: i for i, n in enumerate(names)}
    N = len(names)
    score = np.zeros((N, N))
    th0, s1, s2 = 5.0, 1.0, 10.0
    for X, ims in pts.values():
        if len(ims) < 2:
            continue
        ii = [idx[n] for n in ims]
        rays = np.stack([X - cen[n] for n in ims]); rays /= np.linalg.norm(rays, axis=1, keepdims=True)
        th = np.degrees(np.arccos(np.clip(rays @ rays.T, -1, 1)))
        s = np.where(th <= th0, s1, s2)
        g = np.exp(-(th - th0) ** 2 / (2 * s * s))
        np.fill_diagonal(g, 0)
        score[np.ix_(ii, ii)] += g
    out = {}
    for n in names:
        i = idx[n]
        o = [j for j in np.argsort(-score[i]) if score[i, j] > 0][:a.n_src]
        Xs = np.array([pts[p][0] for p in vis[n]]) if vis[n] else np.zeros((0, 3))
        z = (Xs - cen[n]) @ Rwc[n][:, 2] if len(Xs) else np.zeros(0)
        z = z[z > 0.05]
        if len(z) >= 20:
            dmin, dmax = float(np.percentile(z, 1) * 0.75), float(np.percentile(z, 99) * 1.5)
        else:
            dmin, dmax = 0.3, 6.0
        dmin, dmax = max(0.15, dmin), min(12.0, max(dmax, dmin + 0.5))
        out[n] = dict(src=[names[j] for j in o], score=[round(float(score[i, j]), 2) for j in o],
                      B_m=[round(float(np.linalg.norm(cen[n] - cen[names[j]])), 4) for j in o], dmin=dmin, dmax=dmax, n_pts=int(len(z)))
    (P / "views.json").write_text(json.dumps(out, indent=0))
    B = np.array([np.median(v["B_m"]) for v in out.values() if v["B_m"]])
    ns = np.array([len(v["src"]) for v in out.values()])
    print(json.dumps(dict(n=len(out), n_src_median=float(np.median(ns)), n_src_lt3=int((ns < 3).sum()),
                          B_median_m=float(np.median(B)), dmin_med=float(np.median([v["dmin"] for v in out.values()])),
                          dmax_med=float(np.median([v["dmax"] for v in out.values()])),
                          dmax_p90=float(np.percentile([v["dmax"] for v in out.values()], 90)))))


if __name__ == "__main__":
    main()
