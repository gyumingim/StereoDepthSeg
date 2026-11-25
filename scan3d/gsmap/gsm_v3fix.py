#!/usr/bin/env python3
"""gsmap 2026-11-25 v3 (121801): mesh edit only, no retraining. CPU.

  GSMAP_RUN=<v2 run> ~/scanner/gsmap_env/venv/bin/python scan3d/gsmap/gsm_v3fix.py IN.ply OUT.ply [--cell 0.04] [--close 0.30]

1. floor / ceiling: world y = down (reframed VIO). RANSAC (2 cm) on vertices whose normal is within 25 deg of vertical,
   floor = below the cameras (y > max camera y + 0.3), ceiling = above (y < min camera y - 0.2); largest plane each.
   Footprint = 2 cm grid of plane inliers, closed with --close m (concave, follows the occupied area; NOT a convex hull),
   holes filled. Cells of --cell m inside the footprint with no mesh surface within 5 cm of the plane -> 2 flat triangles
   (generated). No component cleanup of the old kind.
2. floating islands removed only if ALL: (area < 0.05 m2 or < 200 triangles) and > 5 cm from floor and ceiling plane and
   seen by <= 2 keyframes (centroid view count, ray-cast depth agreement at 640x360).
3. floor coverage (before / after): share of the floor footprint's 4 cm cells that have mesh within 5 cm of the plane.
Writes OUT.ply + OUT.json.
"""
import argparse, json, os, time
from pathlib import Path

import cv2
import numpy as np

R = Path(os.environ.get("GSMAP_RUN", ""))


def plane_find(V, VN, sel, thr=0.02):
    import open3d as o3d
    idx = np.nonzero(sel)[0]
    if len(idx) < 500:
        return None
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(V[idx]))
    model, inl = pc.segment_plane(distance_threshold=thr, ransac_n=3, num_iterations=3000)
    n = np.array(model[:3]); d = model[3]; L = np.linalg.norm(n); n /= L; d /= L
    if n[1] < 0:
        n, d = -n, -d
    return n, d, idx[np.array(inl)]


def footprint(V, n, d, inl, close):
    e1 = np.cross(n, [1, 0, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(n, e1)
    q = np.stack([V[inl] @ e1, V[inl] @ e2], 1); lo = q.min(0) - 0.5; g = np.floor((q - lo) / 0.02).astype(int); gs = g.max(0) + 25
    occ = np.zeros((gs[1], gs[0]), np.uint8); occ[g[:, 1], g[:, 0]] = 1
    k = int(round(close / 0.02)) | 1
    cl = cv2.morphologyEx(occ, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    ncc, lab, st, _ = cv2.connectedComponentsWithStats(cl, connectivity=8)
    if ncc > 1:
        cl = (lab == 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))).astype(np.uint8)
    cnts, _ = cv2.findContours(cl, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE); foot = np.zeros_like(cl); cv2.drawContours(foot, cnts, -1, 1, -1)
    return dict(n=n, d=d, e1=e1, e2=e2, lo=lo, foot=foot.astype(bool))


def coverage(V, F, P, cell):
    """share of footprint cells (cell m) with mesh surface samples within 5 cm of the plane; also the empty cells (centres)."""
    P0, P1, P2 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    pts = np.concatenate([(P0 + P1 + P2) / 3, P0, (P0 + P1) / 2])
    near = np.abs(pts @ P["n"] + P["d"]) < 0.05
    q = np.stack([pts[near] @ P["e1"], pts[near] @ P["e2"]], 1)
    s = int(round(cell / 0.02)); fh, fw = P["foot"].shape
    gh, gw = fh // s, fw // s
    fc = P["foot"][:gh * s, :gw * s].reshape(gh, s, gw, s).mean((1, 3)) > 0.5
    g = np.floor((q - P["lo"]) / cell).astype(int); ok = (g[:, 0] >= 0) & (g[:, 0] < gw) & (g[:, 1] >= 0) & (g[:, 1] < gh)
    cov = np.zeros((gh, gw), bool); cov[g[ok, 1], g[ok, 0]] = True
    return float((cov & fc).sum() / max(1, fc.sum())), fc & ~cov, float(fc.sum() * cell * cell)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("inp"); ap.add_argument("out"); ap.add_argument("--cell", type=float, default=0.04); ap.add_argument("--close", type=float, default=0.80); ap.add_argument("--thr", type=float, default=0.04)
    ap.add_argument("--island-area", type=float, default=0.05); ap.add_argument("--island-tri", type=int, default=200); ap.add_argument("--max-views", type=int, default=2)
    a = ap.parse_args()
    import open3d as o3d
    t0 = time.time(); info = {}
    m = o3d.io.read_triangle_mesh(a.inp); m.compute_vertex_normals()
    V = np.asarray(m.vertices, np.float64); F = np.asarray(m.triangles, np.int64); VN = np.asarray(m.vertex_normals); nf0 = len(F)
    c = np.load(R / "dump_ba" / "cams.npz"); kf, T, K = c["kf"], c["T"], c["Kr"]; W, H = [int(v) for v in c["size"]]
    camy = T[:, 1, 3]; vert = np.abs(VN[:, 1]) > np.cos(np.radians(25))
    planes = {}
    fl = plane_find(V, VN, vert & (V[:, 1] > camy.max() + 0.3), a.thr)
    ce = plane_find(V, VN, vert & (V[:, 1] < camy.min() - 0.2), a.thr)
    for nm, p in (("floor", fl), ("ceiling", ce)):
        if p is None:
            info[nm] = None; continue
        n, d, inl = p; planes[nm] = footprint(V, n, d, inl, a.close)
        info[nm] = dict(normal=n.round(3).tolist(), y_m=round(float(-d / n[1]), 3), inliers=int(len(inl)), footprint_m2=round(float(planes[nm]["foot"].sum() * 4e-4), 2))
    # ---- coverage before + fill
    newV, newF = [], []
    for nm, P in planes.items():
        covb, empty, farea = coverage(V, F, P, a.cell)
        info[nm].update(coverage_before=round(covb, 4), footprint_cells_m2=round(farea, 2))
        ys, xs = np.nonzero(empty); b0 = len(V) + sum(len(x) for x in newV)
        for k_, (y_, x_) in enumerate(zip(ys, xs)):
            q0 = P["lo"] + np.array([x_, y_]) * a.cell
            cs = [q0, q0 + [a.cell, 0], q0 + [a.cell, a.cell], q0 + [0, a.cell]]
            P3 = np.array([qq[0] * P["e1"] + qq[1] * P["e2"] - P["d"] * P["n"] for qq in cs])
            newV.append(P3); bb = b0 + 4 * k_
            tri = [[bb, bb + 1, bb + 2], [bb, bb + 2, bb + 3]]
            nt = np.cross(P3[1] - P3[0], P3[2] - P3[0]); towards = -P["n"] if nm == "floor" else P["n"]   # face the room
            newF += tri if nt @ towards > 0 else [t_[::-1] for t_ in tri]
        info[nm]["filled_cells"] = int(len(ys)); info[nm]["filled_m2"] = round(float(len(ys) * a.cell * a.cell), 2)
    # ---- islands
    cl, ntri, carea = m.cluster_connected_triangles(); cl = np.asarray(cl); ntri = np.asarray(ntri); carea = np.asarray(carea)
    small = (carea < a.island_area) | (ntri < a.island_tri)
    cen = (V[F[:, 0]] + V[F[:, 1]] + V[F[:, 2]]) / 3
    near_pl = np.zeros(len(F), bool)
    for P in planes.values():
        near_pl |= np.abs(cen @ P["n"] + P["d"]) < 0.05
    comp_near = np.bincount(cl, weights=near_pl.astype(float), minlength=len(ntri)) > 0
    cand = small & ~comp_near
    fcand = cand[cl]
    nview = np.zeros(len(F), np.int32)
    if fcand.any():
        sc = o3d.t.geometry.RaycastingScene(); sc.add_triangles(o3d.core.Tensor(V.astype(np.float32)), o3d.core.Tensor(F.astype(np.uint32)))
        w, h = 640, 360; Kl = K.copy(); Kl[:2] *= w / W
        uu, vv = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
        dcam = np.stack([(uu - Kl[0, 2]) / Kl[0, 0], (vv - Kl[1, 2]) / Kl[1, 1], np.ones_like(uu)], -1).reshape(-1, 3)
        ci_ = np.nonzero(fcand)[0]; cc = cen[ci_]
        for j in range(len(kf)):
            Rm, t = T[j][:3, :3], T[j][:3, 3]
            Xc = (cc - t) @ Rm; z = Xc[:, 2]; ok = z > 0.05
            u = np.floor(Kl[0, 0] * Xc[:, 0] / np.where(ok, z, 1) + Kl[0, 2]).astype(int); v = np.floor(Kl[1, 1] * Xc[:, 1] / np.where(ok, z, 1) + Kl[1, 2]).astype(int)
            ok &= (u >= 0) & (u < w) & (v >= 0) & (v < h)
            if not ok.any():
                continue
            zb = sc.cast_rays(o3d.core.Tensor(np.concatenate([np.broadcast_to(t, dcam.shape), dcam @ Rm.T], 1).astype(np.float32)))["t_hit"].numpy().reshape(h, w)
            zr = np.full(len(cc), np.inf); zr[ok] = zb[v[ok], u[ok]]
            nview[ci_] += ok & (np.abs(z - zr) < 0.02 + 0.02 * z)
    comp_views = np.zeros(len(ntri)); np.maximum.at(comp_views, cl, nview)
    remove_c = cand & (comp_views <= a.max_views)
    keep = ~remove_c[cl]
    info["islands"] = dict(n_components=int(len(ntri)), small=int(small.sum()), small_not_near_floor_ceiling=int(cand.sum()),
                           removed_components=int(remove_c.sum()), removed_faces_pct=round(float((~keep).mean() * 100), 2),
                           rule=f"(area < {a.island_area} m2 or < {a.island_tri} tri) and > 5 cm from floor/ceiling and seen by <= {a.max_views} kf")
    F = F[keep]
    if newV:
        V = np.concatenate([V] + newV); F = np.concatenate([F, np.array(newF)])
    used = np.zeros(len(V), bool); used[F.ravel()] = True; rm = -np.ones(len(V), np.int64); rm[used] = np.arange(used.sum()); V = V[used]; F = rm[F]
    for nm, P in planes.items():
        cova, _, _ = coverage(V, F, P, a.cell); info[nm]["coverage_after"] = round(cova, 4)
    mo = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V), o3d.utility.Vector3iVector(F.astype(np.int32)))
    o3d.io.write_triangle_mesh(a.out, mo, write_ascii=False, write_vertex_normals=False)
    info.update(made=time.strftime("%F %T"), inp=a.inp, faces_in=int(nf0), faces_out=int(len(F)), runtime_s=round(time.time() - t0, 1), args=vars(a))
    Path(a.out).with_suffix(".json").write_text(json.dumps(info, indent=1)); print(json.dumps(info), flush=True)


if __name__ == "__main__":
    main()
