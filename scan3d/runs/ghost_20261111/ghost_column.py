#!/usr/bin/env python3
"""ghost 11-11 diagnostic: integrate the cached depth (base TSDF) and dump voxel columns (sdf, w, nobs along board z)
at ghost-vertex xy of the base mesh, plus each keyframe's contribution (val, w) to the voxel at the ghost vertex.
usage: ghost_column.py CACHE BASE_MESH_PLY OUT_JSON"""
import json, sys
import numpy as np, torch, cv2
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent)); sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import ghost_fuse as gf, tsdf_map

cache, ply, out = sys.argv[1:4]
K = gf.load(cache); R_ = gf.rays(K[0]); BW, BH = K[0]["BW"], K[0]["BH"]
b = open(ply, "rb").read(); e = b.index(b"end_header\n") + 11; h = b[:e].decode().split("\n")
nv = int([l for l in h if l.startswith("element vertex")][0].split()[-1])
V = np.frombuffer(b[e:e + nv * 15], np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("r", "u1"), ("g", "u1"), ("b", "u1")]))
mb = (V["x"] > 0.02) & (V["x"] < BW - 0.02) & (V["y"] > 0.02) & (V["y"] < BH - 0.02) & (np.abs(V["z"]) < 0.1)
gi = np.flatnonzero(mb & (V["z"] < 0.005))
rng = np.random.default_rng(0); pick = rng.choice(gi, 6, replace=False)
P = np.stack([V["x"][pick], V["y"][pick], V["z"][pick]], 1).astype(np.float64)
t = tsdf_map.TSDFMap(**gf.TS); vm = t.vm
contrib = {i: [] for i in range(len(P))}
for k in K:
    Z, S = k["Z"], k["S"]; T = k["T"]; hh, ww = Z.shape
    for pi, p in enumerate(P):
        g = np.floor(p / vm); c = (g + 0.5) * vm               # voxel centre containing the ghost vertex
        pc = np.linalg.inv(T) @ np.r_[c, 1]
        if pc[2] <= 0: continue
        u = int(round(pc[0] / pc[2] * k["f"] + k["cx"])); v = int(round(pc[1] / pc[2] * k["f"] + k["cy"]))
        if not (0 <= u < ww and 0 <= v < hh) or not np.isfinite(Z[v, u]): continue
        d = float(Z[v, u]); sg = float(S[v, u]); mu = float(np.clip(t.cfg["trunc_k"] * sg, t.cfg["trunc_min_vox"] * vm, max(t.cfg["trunc_max"], t.cfg["trunc_min_vox"] * vm)))
        sdf = d - pc[2]
        if sg > t.cfg["sigma_max"] or d > t.cfg["z_max"] or d < t.cfg["z_min"]: continue
        contrib[pi].append(dict(i=k["i"], sdf_cm=round(sdf * 100, 3), mu_cm=round(mu * 100, 3), w=round((0.01 / sg) ** 2, 3), used=bool(sdf >= -mu), zcam=round(pc[2], 3)))
    bgr = cv2.imdecode(np.frombuffer(k["jpg"], np.uint8), cv2.IMREAD_COLOR)
    t.integrate(np.where(k["V"], Z, np.nan).astype(np.float32), k["V"], bgr, k["f"], k["cx"], k["cy"], 0.08, T, sigma=np.where(k["V"], S, np.inf).astype(np.float32))
cols = []
for pi, p in enumerate(P):
    g = np.floor(p / vm).astype(np.int64); col = []
    for dz in range(-8, 9):
        gg = g + np.array([0, 0, dz]); bk = np.floor_divide(gg, 8); li = gg - bk * 8
        r = int(t._find(tsdf_map._pack(torch.as_tensor(bk[None], device=t.dev)))[0])
        if r < 0: col.append(dict(z_cm=round((gg[2] + .5) * vm * 100, 2), alloc=False)); continue
        idx = int(li[0] + 8 * li[1] + 64 * li[2])
        col.append(dict(z_cm=round((gg[2] + .5) * vm * 100, 2), sdf_cm=round(float(t.sdf[r, idx]) * 100, 3), w=round(float(t.w[r, idx]), 2), nobs=int(t.nobs[r, idx])))
    cs = contrib[pi]
    cols.append(dict(p=p.round(4).tolist(), column=col, n_contrib=len(cs), n_used=sum(c["used"] for c in cs),
                     n_neg=sum(c["used"] and c["sdf_cm"] < 0 for c in cs), contrib_neg=[c for c in cs if c["used"] and c["sdf_cm"] < 0][:30],
                     contrib_pos_sample=[c for c in cs if c["used"] and c["sdf_cm"] >= 0][:10]))
Path(out).write_text(json.dumps(cols, indent=1, default=float))
print("done")
