#!/usr/bin/env python3
"""gsmap 2026-11-25: one ruler for 121801 meshes (old vs v2), CPU only.

  ~/scanner/gsmap_env/venv/bin/python scan3d/gsmap/gsm_eval_mesh.py REF_RUN MESH(.glb|.ply) OUT.json [--cache render_cache.npz]

REF_RUN = the ORIGINAL run (gsmap_20261125_121801): its keyframe poses and its filtered FFS (fuse/moge_f, F == 1) are the
fixed reference for every mesh. Metrics (same as the 11-25 diag where possible, but my own code):
* kf views (every 6th keyframe, 3 px stride, FFS pixels <= 3 m): ok (|t - z| <= 3 cm), hole (no hit or t > z + 3 cm),
  front (t < z - 3 cm = something in front of the measured surface: front double layer / flakes).
* optional --cache: same, vs that model's own 2DGS median depth (alpha > 0.5).
* side views: keyframes every 12th rotated +-25 deg about the vertical through a point 1.2 m ahead; empty = no hit within 4 m.
* panel: keyframe 813 rect x 715..930, y 110..515 (the diag's sticker-cabinet panel): ray hits -> SVD plane on the central
  50 %, RMS / p95 of |distance| (hits within 30 cm), share within 5 mm.
* walls: 400k area-weighted surface samples, 4 RANSAC near-vertical planes (|n_y| < 0.12, 3 cm), RMS of samples within 10 cm.
"""
import json, struct, sys
from pathlib import Path

import cv2
import numpy as np


def load(p):
    import open3d as o3d
    if p.endswith(".glb"):
        b = open(p, "rb").read(); jl = struct.unpack("<I", b[12:16])[0]; j = json.loads(b[20:20 + jl]); bl = b[20 + jl + 8:]
        bv = j["bufferViews"]; g = lambda i: bl[bv[i]["byteOffset"]:bv[i]["byteOffset"] + bv[i]["byteLength"]]  # noqa: E731
        return np.frombuffer(g(0), np.float32).reshape(-1, 3).copy(), np.frombuffer(g(2), np.uint32).reshape(-1, 3).astype(np.int64)
    m = o3d.io.read_triangle_mesh(p); return np.asarray(m.vertices, np.float32), np.asarray(m.triangles, np.int64)


def main():
    import open3d as o3d
    ref, mp, outp = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
    cache = sys.argv[sys.argv.index("--cache") + 1] if "--cache" in sys.argv else None
    V, F = load(mp)
    s = o3d.t.geometry.RaycastingScene(); s.add_triangles(o3d.core.Tensor(V), o3d.core.Tensor(F.astype(np.uint32)))
    c = np.load(ref / "dump_ba" / "cams.npz"); kf, T, K = c["kf"], c["T"], c["Kr"]; W, H = [int(v) for v in c["size"]]
    uu, vv = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
    ray = np.stack([(uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones_like(uu)], -1)
    sub = np.zeros((H, W), bool); sub[::3, ::3] = True
    cast = lambda Tc, d: s.cast_rays(o3d.core.Tensor(np.concatenate([np.broadcast_to(Tc[:3, 3], d.shape), d @ Tc[:3, :3].T], 1).astype(np.float32)))["t_hit"].numpy()  # noqa: E731
    res = dict(mesh=mp, faces=int(len(F)))
    P0, P1, P2 = V[F[:, 0]].astype(np.float64), V[F[:, 1]].astype(np.float64), V[F[:, 2]].astype(np.float64)
    ar = 0.5 * np.linalg.norm(np.cross(P1 - P0, P2 - P0), axis=1); res["area_m2"] = round(float(ar.sum()), 2)
    # kf views vs fixed FFS
    tt, zz = [], []
    for ci in range(2, len(kf), 6):
        f = np.load(ref / "fuse" / "moge_f" / f"{kf[ci]:05d}.npz"); Z, Fm = f["Z"], f["F"]
        m = sub & (Fm == 1) & np.isfinite(Z) & (Z <= 3.0)
        if m.any():
            tt.append(cast(T[ci], ray[m])); zz.append(Z[m])
    t, z = np.concatenate(tt), np.concatenate(zz)
    res["kf_vs_ffs"] = dict(n_px=int(len(z)), ok=round(float((np.abs(t - z) <= 0.03).mean()), 4), hole=round(float((~(t <= z + 0.03)).mean()), 4),
                            front=round(float((t < z - 0.03).mean()), 4))
    if cache:
        d = np.load(cache); tt, zz = [], []
        for k, ci in enumerate(d["idx"]):
            ZM = d["ZM"][k].astype(np.float32); m = sub & (d["AL"][k] > 127) & (ZM > 0.05) & (ZM <= 3.0)
            tt.append(cast(d["T"][ci], ray[m])); zz.append(ZM[m])
        t, z = np.concatenate(tt), np.concatenate(zz)
        res["kf_vs_own_2dgs"] = dict(n_px=int(len(z)), ok=round(float((np.abs(t - z) <= 0.03).mean()), 4), hole=round(float((~(t <= z + 0.03)).mean()), 4),
                                     front=round(float((t < z - 0.03).mean()), 4))
    # side views
    emp, tot = 0, 0
    for ci in range(0, len(kf), 12):
        Tv = T[ci]; piv = Tv[:3, 3] + Tv[:3, 2] * 1.2
        for ang in (-25, 25):
            a = np.radians(ang); Ry = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
            Rw = Tv[:3, :3] @ Ry @ Tv[:3, :3].T; Tn = np.eye(4); Tn[:3, :3] = Rw @ Tv[:3, :3]; Tn[:3, 3] = piv + Rw @ (Tv[:3, 3] - piv)
            t = cast(Tn, ray[sub]); emp += int((~(t <= 4.0)).sum()); tot += len(t)
    res["side_empty_share"] = round(emp / tot, 4)
    # panel
    ci = int(np.nonzero(kf == 813)[0][0]); m = np.zeros((H, W), bool); m[110:516:2, 715:931:2] = True
    t = cast(T[ci], ray[m]); X = (ray[m] * t[:, None])[np.isfinite(t)]
    if len(X) > 100:
        Xw = X @ T[ci][:3, :3].T + T[ci][:3, 3]; core = Xw[np.abs(Xw - np.median(Xw, 0)).max(1) < np.percentile(np.abs(Xw - np.median(Xw, 0)).max(1), 50)]
        ctr = core.mean(0); nn = np.linalg.svd(core - ctr, full_matrices=False)[2][2]; dd = np.abs((Xw - ctr) @ nn); dd = dd[dd < 0.30]
        res["panel"] = dict(hit_share=round(float(np.isfinite(t).mean()), 3), rms_cm=round(float(np.sqrt((dd ** 2).mean()) * 100), 2),
                            p95_cm=round(float(np.percentile(dd, 95) * 100), 2), within_5mm=round(float((dd < 0.005).mean()), 3))
    # walls
    rng = np.random.default_rng(0); fi = rng.choice(len(F), 400000, p=ar / ar.sum()); w_ = rng.random((len(fi), 2)); s1 = np.sqrt(w_[:, 0])
    Pm = (1 - s1)[:, None] * P0[fi] + (s1 * (1 - w_[:, 1]))[:, None] * P1[fi] + (s1 * w_[:, 1])[:, None] * P2[fi]
    left = np.ones(len(Pm), bool); walls = []
    for _ in range(4):
        idx = np.flatnonzero(left); best = (0, None)
        for _ in range(1000):
            a, b, c_ = Pm[rng.choice(idx, 3, replace=False)]; nn = np.cross(b - a, c_ - a); L = np.linalg.norm(nn)
            if L < 1e-9:
                continue
            nn /= L
            if abs(nn[1]) > 0.12:
                continue
            k = int((np.abs((Pm[idx[::5]] - a) @ nn) < 0.03).sum())
            if k > best[0]:
                best = (k, (nn, a))
        if best[1] is None:
            break
        nn, a0 = best[1]; inl = left & (np.abs((Pm - a0) @ nn) < 0.03); ctr = Pm[inl].mean(0); nn = np.linalg.svd(Pm[inl][::5] - ctr, full_matrices=False)[2][2]
        dd = (Pm - ctr) @ nn; band = left & (np.abs(dd) < 0.10)
        walls.append(dict(n=int(band.sum()), rms_cm=round(float(np.sqrt((dd[band] ** 2).mean()) * 100), 2), within_1cm=round(float((np.abs(dd[band]) < 0.01).mean()), 3)))
        left &= ~band
    res["walls"] = walls; res["walls_rms_cm_median"] = float(np.median([w["rms_cm"] for w in walls])) if walls else None
    Path(outp).write_text(json.dumps(res, indent=1)); print(json.dumps(res), flush=True)


if __name__ == "__main__":
    main()
