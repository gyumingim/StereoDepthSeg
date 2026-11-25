#!/usr/bin/env python3
"""gsmap 2026-11-25: mesh cleanup + room-wide plane / box fitting (CPU only).

  GSMAP_RUN=<run dir> ~/scanner/gsmap_env/venv/bin/python scan3d/gsmap/gsm_planes.py IN.ply OUT.ply [options]

0. weld: merge vertices < --weld (6 mm; the 121801 decimated mesh was 84k separate pieces).
1. cleanup: (a) drop connected components with < --min-comp-tri triangles or < --min-comp-area m2;
   (b) drop faces seen by < --min-views keyframes (centroid inside image, ray-cast depth at 640x360 agrees 2 cm + 2 %).
2. planes: per-vertex reliability = the keyframes whose fused depth agrees with the vertex (3 cm / 3 %) are mostly FFS (F == 1).
   Repeated RANSAC (--thr) on the reliable vertices; each plane's inliers are split into connected 2 cm grid regions;
   regions >= --min-area m2 are kept and refitted (least squares). Region footprint = closed (10 cm) + filled grid.
3. snap: every vertex (reliable or not) within --snap m of a region plane, inside its footprint and with vertex normal
   within --max-angle deg of the plane normal -> projected onto the plane (largest regions first, once per vertex).
4. boxes: 2-3 mutually perpendicular regions (|cos| < 0.15) that touch (< 4 cm) and are compact (all extents 0.1..1.2 m,
   each region < 1.5 m2) -> cuboid from the region points; its missing faces (not a region, not the bottom (+y = down),
   facing at least one camera) are added as flat quads, marked generated.
5. holes: boundary loops whose vertices all belong to one plane region are closed with a flat fan (any size), generated.
Writes OUT.ply (vertex colour: red = generated, else grey) + OUT.json (+ OUT.gen_vertices.npy).
"""
import argparse, json, os, sys, time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

R = Path(os.environ.get("GSMAP_RUN", ""))


def boundary_loops(F, nV, max_len=4000):
    Ed = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
    E = np.sort(Ed, 1); key = E[:, 0] * nV + E[:, 1]
    _, inv, cnt = np.unique(key, return_inverse=True, return_counts=True)
    bd = Ed[cnt[inv] == 1]
    nxt = defaultdict(list)
    for a, b in bd:
        nxt[b].append(a)
    used = set(); loops = []
    for a0, b0 in bd:
        if (a0, b0) in used:
            continue
        loop = [b0, a0]; used.add((a0, b0)); ok = True
        while True:
            cand = nxt.get(loop[-1], [])
            if len(cand) != 1:
                ok = False; break
            n_ = cand[0]; used.add((n_, loop[-1]))
            if n_ == loop[0]:
                break
            loop.append(n_)
            if len(loop) > max_len:
                ok = False; break
        if ok and len(loop) >= 3:
            loops.append(np.array(loop))
    return loops


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("inp"); ap.add_argument("out")
    ap.add_argument("--min-comp-tri", type=int, default=200); ap.add_argument("--min-comp-area", type=float, default=0.005)
    ap.add_argument("--weld", type=float, default=0.006, help="merge vertices closer than this (m): unwelded TSDF blocks, decimation cracks")
    ap.add_argument("--min-views", type=int, default=3)
    ap.add_argument("--thr", type=float, default=0.025); ap.add_argument("--min-area", type=float, default=0.05)
    ap.add_argument("--snap", type=float, default=0.05); ap.add_argument("--max-angle", type=float, default=30.0)
    ap.add_argument("--max-planes", type=int, default=80)
    a = ap.parse_args()
    import open3d as o3d
    from scipy.spatial import cKDTree
    t0 = time.time(); tm = {}; info = {}
    m = o3d.io.read_triangle_mesh(a.inp)
    nv_in = len(m.vertices)
    m.merge_close_vertices(a.weld); m.remove_duplicated_triangles(); m.remove_degenerate_triangles()   # TSDF block seams + decimation cracks
    info["weld"] = dict(vertices_in=int(nv_in), vertices_welded=int(len(m.vertices)))
    V = np.asarray(m.vertices, np.float64); F = np.asarray(m.triangles, np.int64); nf0 = len(F)
    c = np.load(R / "dump_ba" / "cams.npz"); kf, T, K = c["kf"], c["T"], c["Kr"]; W, H = [int(v) for v in c["size"]]
    # ---- 1a components
    cl, ntri, carea = m.cluster_connected_triangles()
    cl = np.asarray(cl); ntri = np.asarray(ntri); carea = np.asarray(carea)
    keep_c = (ntri >= a.min_comp_tri) & (carea >= a.min_comp_area)
    keep = keep_c[cl]
    info["cleanup_components"] = dict(n_components=int(len(ntri)), kept=int(keep_c.sum()), faces_removed_pct=round(float((~keep).mean() * 100), 2),
                                      rule=f"< {a.min_comp_tri} triangles or < {a.min_comp_area} m2")
    F = F[keep]
    # ---- 1b view count
    P0, P1, P2 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]; cen = (P0 + P1 + P2) / 3
    sc = o3d.t.geometry.RaycastingScene(); sc.add_triangles(o3d.core.Tensor(V.astype(np.float32)), o3d.core.Tensor(F.astype(np.uint32)))
    w, h = 640, 360; Kl = K.copy(); Kl[:2] *= w / W
    uu, vv = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    dcam = np.stack([(uu - Kl[0, 2]) / Kl[0, 0], (vv - Kl[1, 2]) / Kl[1, 1], np.ones_like(uu)], -1).reshape(-1, 3)
    nview = np.zeros(len(F), np.int32)
    for j in range(len(kf)):
        Rm, t = T[j][:3, :3], T[j][:3, 3]
        zb = sc.cast_rays(o3d.core.Tensor(np.concatenate([np.broadcast_to(t, dcam.shape), dcam @ Rm.T], 1).astype(np.float32)))["t_hit"].numpy().reshape(h, w)
        Xc = (cen - t) @ Rm; z = Xc[:, 2]; ok = z > 0.05
        u = np.floor(Kl[0, 0] * Xc[:, 0] / np.where(ok, z, 1) + Kl[0, 2]).astype(int); v = np.floor(Kl[1, 1] * Xc[:, 1] / np.where(ok, z, 1) + Kl[1, 2]).astype(int)
        ok &= (u >= 0) & (u < w) & (v >= 0) & (v < h)
        zr = np.full(len(F), np.inf); zr[ok] = zb[v[ok], u[ok]]
        nview += ok & (np.abs(z - zr) < 0.02 + 0.02 * z)
    keepv = nview >= a.min_views
    info["cleanup_views"] = dict(faces_removed_pct_of_input=round(float((~keepv).sum() / nf0 * 100), 2), rule=f"seen by < {a.min_views} keyframes")
    F = F[keepv]
    info["faces_after_cleanup"] = int(len(F)); info["faces_removed_total_pct"] = round(float((1 - len(F) / nf0) * 100), 2)
    used = np.zeros(len(V), bool); used[F.ravel()] = True; remap = -np.ones(len(V), np.int64); remap[used] = np.arange(used.sum())
    V = V[used]; F = remap[F]; nV = len(V)
    tm["cleanup"] = time.time() - t0
    # ---- reliability + vertex normals
    mm = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V), o3d.utility.Vector3iVector(F.astype(np.int32)))
    mm.compute_vertex_normals(); VN = np.asarray(mm.vertex_normals)
    nffs = np.zeros(nV); nall = np.zeros(nV)
    for j, i in enumerate(kf):
        d = np.load(R / "fuse" / "moge_f" / f"{i:05d}.npz"); Z, Fm = d["Z"], d["F"]
        Rm, t = T[j][:3, :3], T[j][:3, 3]; Xc = (V - t) @ Rm; z = Xc[:, 2]; ok = z > 0.05
        u = np.round(K[0, 0] * Xc[:, 0] / np.where(ok, z, 1) + K[0, 2]).astype(int); v = np.round(K[1, 1] * Xc[:, 1] / np.where(ok, z, 1) + K[1, 2]).astype(int)
        ok &= (u >= 0) & (u < W) & (v >= 0) & (v < H)
        zr = np.full(nV, np.nan); fr = np.zeros(nV, np.uint8); zr[ok] = Z[v[ok], u[ok]]; fr[ok] = Fm[v[ok], u[ok]]
        ag = ok & np.isfinite(zr) & (np.abs(z - zr) < np.maximum(0.03, 0.03 * z))
        nall += ag; nffs += ag & (fr == 1)
    rel = (nall > 0) & (nffs >= 0.5 * nall)
    info["reliable_vertex_pct"] = round(float(rel.mean() * 100), 2)
    tm["reliability"] = time.time() - t0
    # ---- planes -> regions
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(V)); pool = np.nonzero(rel)[0]; regions = []
    for it in range(a.max_planes):
        if len(pool) < 500:
            break
        model, inl = pc.select_by_index(pool.tolist()).segment_plane(distance_threshold=a.thr, ransac_n=3, num_iterations=1500)
        inl = pool[np.array(inl)]
        if len(inl) < 300:
            break
        pool = np.setdiff1d(pool, inl)
        n = np.array(model[:3]); n /= np.linalg.norm(n)
        e1 = np.cross(n, [1, 0, 0] if abs(n[0]) < 0.9 else [0, 1, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(n, e1)
        q = np.stack([V[inl] @ e1, V[inl] @ e2], 1); lo = q.min(0); g = np.floor((q - lo) / 0.02).astype(int); gs = g.max(0) + 1
        occ = np.zeros((gs[1], gs[0]), np.uint8); occ[g[:, 1], g[:, 0]] = 1
        occd = cv2.dilate(occ, np.ones((3, 3), np.uint8))
        ncc, lab = cv2.connectedComponents(occd, connectivity=8)
        vlab = lab[g[:, 1], g[:, 0]]
        for k_ in range(1, ncc):
            sel = inl[vlab == k_]
            area = (lab == k_).sum() * 4e-4 * (occ[lab == k_].mean())     # occupied cells only
            if area < a.min_area or len(sel) < 100:
                continue
            P = V[sel]; ctr = P.mean(0); _, _, vt = np.linalg.svd(P - ctr, full_matrices=False); nn = vt[2]
            if nn @ n < 0:
                nn = -nn
            f1 = np.cross(nn, [1, 0, 0] if abs(nn[0]) < 0.9 else [0, 1, 0]); f1 /= np.linalg.norm(f1); f2 = np.cross(nn, f1)
            qq = np.stack([P @ f1, P @ f2], 1); lo2 = qq.min(0); gg = np.floor((qq - lo2) / 0.02).astype(int); gs2 = gg.max(0) + 1
            o2 = np.zeros((gs2[1], gs2[0]), np.uint8); o2[gg[:, 1], gg[:, 0]] = 1
            o2 = cv2.morphologyEx(o2, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
            cnts, _ = cv2.findContours(o2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE); foot = np.zeros_like(o2); cv2.drawContours(foot, cnts, -1, 1, -1)
            regions.append(dict(n=nn, d=-float(nn @ ctr), f1=f1, f2=f2, lo=lo2, foot=foot.astype(bool), pts=sel, area=float(area)))
    regions.sort(key=lambda r: -r["area"])
    tm["planes"] = time.time() - t0
    # ---- snap
    lab_v = -np.ones(nV, np.int64); cosmax = np.cos(np.radians(a.max_angle)); nsnap = []
    for ri, r in enumerate(regions):
        dist = V @ r["n"] + r["d"]
        cand = np.nonzero((np.abs(dist) < a.snap) & (lab_v < 0) & (np.abs(VN @ r["n"]) > cosmax))[0]
        qq = np.stack([V[cand] @ r["f1"], V[cand] @ r["f2"]], 1); gg = np.floor((qq - r["lo"]) / 0.02).astype(int)
        fs = r["foot"]; ins = (gg[:, 0] >= 0) & (gg[:, 0] < fs.shape[1]) & (gg[:, 1] >= 0) & (gg[:, 1] < fs.shape[0])
        ins[ins] = fs[gg[ins, 1], gg[ins, 0]]
        sel = cand[ins]
        V[sel] -= dist[sel, None] * r["n"][None]; lab_v[sel] = ri; nsnap.append(int(len(sel)))
    info["n_planes"] = len(regions); info["snapped_vertex_pct"] = round(float((lab_v >= 0).mean() * 100), 2)
    info["regions"] = [dict(normal=r["n"].round(3).tolist(), area_m2=round(r["area"], 3), snapped=s) for r, s in zip(regions, nsnap)]
    tm["snap"] = time.time() - t0
    # ---- boxes
    newV, newF = [], []
    sub = [V[r["pts"][:: max(1, len(r["pts"]) // 3000)]] for r in regions]
    trees = [cKDTree(s) for s in sub]
    small = [i for i, r in enumerate(regions) if r["area"] < 1.5]
    adj = defaultdict(set)
    for ii, i in enumerate(small):
        for j in small[ii + 1:]:
            if abs(regions[i]["n"] @ regions[j]["n"]) < 0.15:
                if trees[i].query(sub[j], k=1, distance_upper_bound=0.04)[0].min() < 0.04:
                    adj[i].add(j); adj[j].add(i)
    boxes = []; usedr = set(); camc = T[:, :3, 3]
    for i in sorted(adj, key=lambda x: -regions[x]["area"]):
        if i in usedr:
            continue
        for j in sorted(adj[i], key=lambda x: -regions[x]["area"]):
            if j in usedr:
                continue
            grp = [i, j] + [k for k in adj[i] & adj[j] if k not in usedr and abs(regions[k]["n"] @ np.cross(regions[i]["n"], regions[j]["n"])) > 0.85][:1]
            a1 = regions[i]["n"]; a2 = regions[j]["n"] - (regions[j]["n"] @ a1) * a1; a2 /= np.linalg.norm(a2); a3 = np.cross(a1, a2)
            Ax = np.stack([a1, a2, a3])
            P = np.concatenate([V[regions[k]["pts"]] for k in grp]); pr = P @ Ax.T
            lo, hi = np.percentile(pr, 1, 0), np.percentile(pr, 99, 0); ext = hi - lo
            if not ((ext > 0.1).all() and (ext < 1.2).all()):
                continue
            present = set()
            for k in grp:
                nk = regions[k]["n"]; ax_ = int(np.argmax(np.abs(Ax @ nk))); off = -regions[k]["d"] * np.sign(Ax[ax_] @ nk)
                present.add((ax_, 0 if abs(off - lo[ax_]) < abs(off - hi[ax_]) else 1))
            gen = 0
            for ax_ in range(3):
                for side in (0, 1):
                    if (ax_, side) in present:
                        continue
                    nout = Ax[ax_] * (1 if side else -1)
                    if nout[1] > 0.7:        # bottom (world +y = down): resting on something, not generated
                        continue
                    o1, o2 = [x for x in range(3) if x != ax_]
                    cc = np.zeros(3); cc[ax_] = hi[ax_] if side else lo[ax_]
                    corners = []
                    for s1, s2 in ((0, 0), (1, 0), (1, 1), (0, 1)):
                        q = cc.copy(); q[o1] = hi[o1] if s1 else lo[o1]; q[o2] = hi[o2] if s2 else lo[o2]; corners.append(q @ Ax)
                    corners = np.array(corners); fc = corners.mean(0)
                    if not ((camc - fc) @ nout > 0.05).any():
                        continue
                    # subdivide the quad 10 x 10 so per-face texturing / visibility work
                    b0 = len(V) + sum(len(x) for x in newV); gv = []
                    for iy in range(11):
                        for ix in range(11):
                            u_, v_ = ix / 10, iy / 10
                            gv.append((1 - u_) * (1 - v_) * corners[0] + u_ * (1 - v_) * corners[1] + u_ * v_ * corners[2] + (1 - u_) * v_ * corners[3])
                    gf = []
                    for iy in range(10):
                        for ix in range(10):
                            p_ = b0 + iy * 11 + ix
                            tri = [[p_, p_ + 1, p_ + 12], [p_, p_ + 12, p_ + 11]]
                            n_t = np.cross(gv[iy * 11 + ix + 1] - gv[iy * 11 + ix], gv[iy * 11 + ix + 12] - gv[iy * 11 + ix])
                            gf += tri if n_t @ nout > 0 else [t_[::-1] for t_ in tri]
                    newV.append(np.array(gv)); newF.append(np.array(gf)); gen += 1
            boxes.append(dict(regions=grp, extents_m=ext.round(3).tolist(), center=((lo + hi) / 2 @ Ax).round(3).tolist(), generated_faces=gen))
            usedr.update(grp); break
    nV0 = len(V)
    if newV:
        V = np.concatenate([V] + newV); F = np.concatenate([F] + newF)
    gen_v = np.zeros(len(V), bool); gen_v[nV0:] = True
    lab_v = np.concatenate([lab_v, -np.ones(len(V) - nV0, np.int64)])
    info["n_boxes"] = len(boxes); info["boxes"] = boxes
    tm["boxes"] = time.time() - t0
    # ---- holes inside plane regions
    loops = boundary_loops(F, len(V)); hv, hf, nh, harea = [], [], 0, 0.0
    for L in loops:
        lb = lab_v[L]
        if (lb < 0).any() or len(np.unique(lb)) != 1:
            continue
        P = V[L]; ci = len(V) + len(hv); cpt = P.mean(0)
        r = regions[lb[0]]; cpt = cpt - (cpt @ r["n"] + r["d"]) * r["n"]
        hv.append(cpt)
        for k in range(len(L)):
            hf.append([L[k], L[(k + 1) % len(L)], ci])
        nh += 1; harea += 0.5 * np.linalg.norm(np.cross(P - cpt, np.roll(P, -1, 0) - cpt), axis=1).sum()
    if hv:
        V = np.concatenate([V, np.array(hv)]); F = np.concatenate([F, np.array(hf)])
        gen_v = np.concatenate([gen_v, np.ones(len(hv), bool)])
    info["plane_holes_filled"] = nh; info["plane_hole_area_m2"] = round(float(harea), 3)
    tm["holes"] = time.time() - t0
    col = np.where(gen_v[:, None], [[1.0, 0.1, 0.1]], [[0.6, 0.6, 0.6]])
    mo = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V), o3d.utility.Vector3iVector(F.astype(np.int32)))
    mo.vertex_colors = o3d.utility.Vector3dVector(col)
    o3d.io.write_triangle_mesh(a.out, mo, write_ascii=False, write_vertex_normals=False, write_vertex_colors=True)
    np.save(Path(a.out).with_suffix(".gen_vertices.npy"), np.nonzero(gen_v)[0])
    info.update(made=time.strftime("%F %T"), inp=a.inp, faces_in=int(nf0), faces_out=int(len(F)), vertices_out=int(len(V)),
                generated_vertices=int(gen_v.sum()), times_s={k: round(v, 1) for k, v in tm.items()}, args=vars(a))
    Path(a.out).with_suffix(".json").write_text(json.dumps(info, indent=1, default=float))
    print(json.dumps({k: v for k, v in info.items() if k not in ("regions", "boxes")}, default=float), flush=True)


if __name__ == "__main__":
    main()
