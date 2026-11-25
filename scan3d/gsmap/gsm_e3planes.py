#!/usr/bin/env python3
"""gsmap 2026-11-25 E3 (plan section 3 / judge round 1): full-room ceiling + floor planes. CPU.

  GSMAP_RUN=<v2 run> python scan3d/gsmap/gsm_e3planes.py IN.ply OUT.ply [--cell 0.04]

(a) floor plane = the v3 fit (eval2/ref/room_poly.json). Ceiling plane forced PARALLEL to the floor; its height = the topmost
    supported mode of near-horizontal vertices above the cameras (1 cm histogram of the distance along the floor normal; topmost
    bin cluster with >= 20 % of the largest peak's count). (The plan's photo-consistency plane sweep is NOT implemented.)
(b) delete near-horizontal faces (normal within 25 deg of vertical) between the ceiling plane and 0.6 m below it that are
    > 4 cm off the ceiling plane (ceiling shards / chunks).
(c) fill both planes over the room polygon (eval2 ref): every --cell cell inside the polygon without mesh within 5 cm of the
    plane gets 2 flat triangles facing the room (generated).
(d) snap vertices within +-3 cm of the floor plane onto it (no component removal).
Writes OUT.ply + OUT.json (+ OUT.gen_faces.npy).
"""
import argparse, json, os, time
from pathlib import Path

import cv2
import numpy as np

R = Path(os.environ.get("GSMAP_RUN", ""))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("inp"); ap.add_argument("out"); ap.add_argument("--cell", type=float, default=0.04)
    ap.add_argument("--shard-band", type=float, default=0.6); ap.add_argument("--shard-off", type=float, default=0.04); ap.add_argument("--snap", type=float, default=0.03)
    a = ap.parse_args()
    import open3d as o3d
    t0 = time.time(); info = {}
    m = o3d.io.read_triangle_mesh(a.inp); m.compute_vertex_normals(); m.compute_triangle_normals()
    V = np.asarray(m.vertices, np.float64).copy(); F = np.asarray(m.triangles, np.int64); VN = np.asarray(m.vertex_normals); FN = np.asarray(m.triangle_normals)
    rp = json.loads((R / "eval2" / "ref" / "room_poly.json").read_text())
    n = np.array(rp["floor"]["normal"]); n /= np.linalg.norm(n); yf = rp["floor"]["y_m"]
    df = -(np.array([0, yf, 0]) @ n)          # floor plane: n.x + df = 0 ; n points down (+y)
    s = V @ n + df                             # signed distance along n: 0 at floor, negative above it
    c = np.load(R / "dump_ba" / "cams.npz"); T = c["T"]; scam = T[:, :3, 3] @ n + df
    hz_v = np.abs(VN @ n) > np.cos(np.radians(25))
    cand = hz_v & (s < scam.min() - 0.2)
    hist, edges = np.histogram(s[cand], bins=np.arange(s[cand].min(), scam.min() - 0.2 + 0.01, 0.01))
    pk = hist.max(); ok = np.nonzero(hist >= 0.2 * pk)[0]
    top = ok.min()                              # topmost (most negative distance = highest) supported bin
    j = top
    while j + 1 < len(hist) and hist[j + 1] >= hist[j]:   # climb to the local peak of that cluster
        j += 1
    sc = float((edges[j] + edges[j + 1]) / 2); dc = df - sc   # ceiling plane: n.x + dc = 0
    info["floor"] = dict(normal=n.round(4).tolist(), y_m=yf); info["ceiling"] = dict(normal=n.round(4).tolist(), dist_above_floor_m=round(-sc, 3),
                                                                                       y_m=round(float(-dc / n[1]), 3), hist_peak=int(pk), mode_count=int(hist[j]),
                                                                                       method="topmost supported mode (>= 20 % of the largest peak), parallel to floor")
    # (b) shards below the ceiling
    cen = (V[F[:, 0]] + V[F[:, 1]] + V[F[:, 2]]) / 3; sf = cen @ n + df
    hz_f = np.abs(FN @ n) > np.cos(np.radians(25))
    shard = hz_f & (sf > sc) & (sf < sc + a.shard_band) & (np.abs(sf - sc) > a.shard_off)
    ar_ = 0.5 * np.linalg.norm(np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]), axis=1)
    info["shards_removed"] = dict(faces=int(shard.sum()), area_m2=round(float(ar_[shard].sum()), 3), area_pct=round(float(ar_[shard].sum() / ar_.sum() * 100), 3))
    F = F[~shard]
    # (d) snap floor
    near = np.abs(s) < a.snap; V[near] -= s[near, None] * n[None]; info["floor_snapped_vertices"] = int(near.sum())
    # (c) fill over the room polygon
    poly = np.array(rp["poly_xz"]); lo = poly.min(0); gsz = np.ceil((poly.max(0) - lo) / a.cell).astype(int) + 1
    pm = np.zeros((gsz[1], gsz[0]), np.uint8); cv2.fillPoly(pm, [np.round((poly - lo) / a.cell).astype(np.int32)], 1)
    P0, P1, P2 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]; pts = np.concatenate([(P0 + P1 + P2) / 3, P0, (P0 + P1) / 2])
    newV, newF = [], []
    for nm, d_ in (("floor", df), ("ceiling", dc)):
        dd = np.abs(pts @ n + d_); q = pts[dd < 0.05][:, [0, 2]]; g = np.floor((q - lo) / a.cell).astype(int)
        okk = (g[:, 0] >= 0) & (g[:, 0] < gsz[0]) & (g[:, 1] >= 0) & (g[:, 1] < gsz[1]); cov = np.zeros_like(pm); cov[g[okk, 1], g[okk, 0]] = 1
        empty = (pm > 0) & (cov == 0); ys, xs = np.nonzero(empty)
        info[nm].update(coverage_before_pct=round(float((cov & pm).sum() / pm.sum() * 100), 2), filled_cells=int(len(ys)), filled_m2=round(float(len(ys) * a.cell ** 2), 2))
        b0 = len(V) + sum(len(x) for x in newV)
        towards = -n if nm == "floor" else n    # floor faces up (-n), ceiling faces down (+n)
        for k_, (y_, x_) in enumerate(zip(ys, xs)):
            x0, z0 = lo[0] + x_ * a.cell, lo[1] + y_ * a.cell
            cs = []
            for dx, dz in ((0, 0), (a.cell, 0), (a.cell, a.cell), (0, a.cell)):
                x, z = x0 + dx, z0 + dz; y = -(n[0] * x + n[2] * z + d_) / n[1]; cs.append([x, y, z])
            cs = np.array(cs); newV.append(cs); bb = b0 + 4 * k_
            tri = [[bb, bb + 1, bb + 2], [bb, bb + 2, bb + 3]]; nt = np.cross(cs[1] - cs[0], cs[2] - cs[0])
            newF += tri if nt @ towards > 0 else [t_[::-1] for t_ in tri]
    nf_old = len(F)
    if newV:
        V = np.concatenate([V] + newV); F = np.concatenate([F, np.array(newF, np.int64)])
    used = np.zeros(len(V), bool); used[F.ravel()] = True; rm = -np.ones(len(V), np.int64); rm[used] = np.arange(used.sum()); V = V[used]; F = rm[F]
    mo = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V), o3d.utility.Vector3iVector(F.astype(np.int32)))
    o3d.io.write_triangle_mesh(a.out, mo, write_ascii=False, write_vertex_normals=False)
    np.save(Path(a.out).with_suffix(".gen_faces.npy"), np.arange(nf_old, len(F)))
    info.update(made=time.strftime("%F %T"), inp=a.inp, faces_out=int(len(F)), generated_faces=int(len(F) - nf_old), args=vars(a), runtime_s=round(time.time() - t0, 1))
    Path(a.out).with_suffix(".json").write_text(json.dumps(info, indent=1)); print(json.dumps(info), flush=True)


if __name__ == "__main__":
    main()
