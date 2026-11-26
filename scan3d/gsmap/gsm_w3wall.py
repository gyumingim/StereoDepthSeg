#!/usr/bin/env python3
"""gsmap 2026-11-26 overnight W3: conservative vertical wall plane fill (like E3c floor / ceiling), FFS depth only. CPU.

  GSMAP_RUN=<eval run with dump_ba (ba2)> python scan3d/gsmap/gsm_w3wall.py IN.ply OUTDIR --fused FUSEDIR [--detect-only]

1. FFS points: every keyframe of GSMAP_RUN/dump_ba/cams.npz, fused depth FUSEDIR/<kf>.npz (Z, F == 1 only = FFS), every 4th pixel,
   z < 4 m, back-projected with the same poses (ba2), 2 cm voxel, normals (8 cm). Keep near-vertical surface points (|n_y| < 0.17).
2. Vertical-plane RANSAC (normal forced horizontal, 2-point samples, 2 cm band, normal within 20 deg), refit by PCA on a 3 cm band.
   Accept: unconstrained PCA normal within 5 deg of horizontal, RMS (points within 5 cm) < 2 cm, FFS-supported area
   (4 cm plane cells with >= 2 points) >= --min-area m2. Near-parallel accepted planes < 30 cm apart with overlapping extent: keep first.
3. Per accepted plane, 4 cm (u, y) cells between ceiling and floor (--floor-y / --ceil-y = E3c heights of the input, else room_poly.json):
   support = FFS cells; meshcov = existing mesh samples (centroid / vertices / edge midpoints, as E3c) within 5 cm of the plane;
   extent = morphological closing (ellipse --close m) of support | meshcov; hole = extent & ~meshcov & inside the room polygon
   (cell centre xz inside the polygon or within 15 cm of it).
4. Guards per hole cell over all keyframes that see the cell centre (z 0.1..6 m, in image, |cos(view, n)| > 0.2):
   FFS see-through: FFS depth at the pixel > z + max(5 cm, 4 %)   (FFS-observed free space at / behind the cell)
   FFS agree: |FFS - z| <= tol;  mesh ray cast camera -> cell: occluded (hit < dist - 5 cm), behind (finite hit > dist + 10 cm).
   Fill only if (see == 0 or (see == 1 and agree >= 2)) and behind <= 1 and clear (not occluded) views >= 2.
5. 2 flat triangles per filled cell, facing the camera side. No vertex snapping. OUTDIR/w3.ply, w3.json, w3_fill.npz.
"""
import argparse, json, os, sys, time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
R = Path(os.environ.get("GSMAP_RUN", ""))
Y = np.array([0.0, 1.0, 0.0])


def ffs_points(T, K, kf, fused, stride=4, zmax=4.0):
    pts = []
    for j in range(len(T)):
        fp = Path(fused) / f"{int(kf[j]):05d}.npz"
        if not fp.exists():
            continue
        d = np.load(fp); Z = d["Z"][::stride, ::stride]; Fm = (d["F"][::stride, ::stride] == 1) & (Z > 0.1) & (Z < zmax)
        vv, uu = np.nonzero(Fm); z = Z[vv, uu].astype(np.float64); u = uu * stride + 0.5; v = vv * stride + 0.5
        Xc = np.stack([(u - K[0, 2]) / K[0, 0] * z, (v - K[1, 2]) / K[1, 1] * z, z], 1)
        pts.append(Xc @ T[j][:3, :3].T + T[j][:3, 3])
    return np.concatenate(pts)


def ransac_vertical(P, N, it=3000, band=0.02, rng=None):
    best = (0, None); n = len(P); cn = np.cos(np.radians(20))
    for _ in range(it):
        i, k = rng.integers(0, n, 2); d = P[k] - P[i]; nn = np.cross(d, Y); L = np.linalg.norm(nn)
        if L < 0.3:
            continue
        nn /= L; off = nn @ P[i]
        c = int(((np.abs(P @ nn - off) < band) & (np.abs(N @ nn) > cn)).sum())
        if c > best[0]:
            best = (c, (nn, off))
    return best


def refit(P):
    xz = P[:, [0, 2]]; c = xz.mean(0); w_ = np.linalg.svd(xz - c, full_matrices=False)[2][1]
    nn = np.array([w_[0], 0.0, w_[1]]); return nn, float(nn @ np.array([c[0], 0, c[1]]))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("inp"); ap.add_argument("outdir"); ap.add_argument("--fused", required=True)
    ap.add_argument("--cell", type=float, default=0.04); ap.add_argument("--max-planes", type=int, default=16)
    ap.add_argument("--min-area", type=float, default=1.0); ap.add_argument("--close", type=float, default=1.0)
    ap.add_argument("--detect-only", action="store_true"); ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--floor-y", type=float, default=None); ap.add_argument("--ceil-y", type=float, default=None)
    ap.add_argument("--max-rms", type=float, default=0.02)
    ap.add_argument("--min-comp", type=float, default=0.0, help="drop filled patches (4-connected per plane) smaller than this m2 (eval2 P2b counts welded components < 0.25 m2)")
    ap.add_argument("--only", default="", help="comma list of plane ids to fill (default: all accepted)")
    a = ap.parse_args()
    import open3d as o3d
    t0 = time.time(); od = Path(a.outdir); od.mkdir(parents=True, exist_ok=True); rng = np.random.default_rng(a.seed)
    c = np.load(R / "dump_ba" / "cams.npz"); kf, T, K = c["kf"], c["T"], c["Kr"]; w, h = [int(v) for v in c["size"]]
    rp = json.loads((R / "eval2" / "ref" / "room_poly.json").read_text()); poly = np.array(rp["poly_xz"])
    yf, yc = rp["floor"]["y_m"], rp["ceiling"]["y_m"]
    if a.floor_y is not None:   # use the E3c plane heights of the input mesh (room_poly heights differ)
        yf = a.floor_y
    if a.ceil_y is not None:
        yc = a.ceil_y
    ylo, yhi = min(yf, yc), max(yf, yc)
    info = dict(made=time.strftime("%F %T"), worker="W3", n_recordings=1, inp=a.inp, fused=a.fused, cams=str(R / "dump_ba" / "cams.npz"),
                room_poly=str(R / "eval2" / "ref" / "room_poly.json"), floor_y=yf, ceiling_y=yc, args=vars(a))
    # 1. FFS points
    P = ffs_points(T, K, kf, a.fused); nraw = len(P)
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P)).voxel_down_sample(0.02)
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.08, max_nn=30))
    P = np.asarray(pc.points); Nn = np.asarray(pc.normals); vert = np.abs(Nn[:, 1]) < 0.17
    info["ffs_points"] = dict(raw_stride4=int(nraw), voxel2cm=int(len(P)), vertical=int(vert.sum()))
    Pv, Nv = P[vert], Nn[vert]; rem = np.ones(len(Pv), bool); planes = []
    # 2. RANSAC
    for pi in range(a.max_planes * 2):
        idx = np.nonzero(rem)[0]
        if len(idx) < 500:
            break
        cnt, pl = ransac_vertical(Pv[idx], Nv[idx], rng=rng)
        if pl is None or cnt < 300:
            break
        nn, off = pl
        for _ in range(3):
            band = idx[(np.abs(Pv[idx] @ nn - off) < 0.03) & (np.abs(Nv[idx] @ nn) > np.cos(np.radians(20)))]
            nn, off = refit(Pv[band])
        d5 = Pv[idx] @ nn - off; in5 = idx[np.abs(d5) < 0.05]; rms = float(np.sqrt(np.mean((Pv[in5] @ nn - off) ** 2)))
        Q = Pv[band]; q0 = Q.mean(0); nf = np.linalg.svd(Q - q0, full_matrices=False)[2][2]; tilt = float(np.degrees(np.arcsin(min(1, abs(nf[1])))))
        eu = np.cross(Y, nn); u = Q @ eu; g = np.stack([np.floor(u / a.cell), np.floor(Q[:, 1] / a.cell)], 1).astype(np.int64)
        _, gc = np.unique(g, axis=0, return_counts=True); sup_m2 = float((gc >= 2).sum() * a.cell ** 2)
        rem[band] = False; rem[in5] = False
        ok = tilt < 5 and rms < a.max_rms and sup_m2 >= a.min_area
        planes.append(dict(id=len(planes), n=nn.tolist(), off=off, n_band=int(len(band)), rms_cm=round(rms * 100, 2), tilt_deg=round(tilt, 2),
                           sup_m2=round(sup_m2, 2), u_range=[round(float(np.percentile(u, 1)), 2), round(float(np.percentile(u, 99)), 2)],
                           y_range=[round(float(np.percentile(Q[:, 1], 1)), 2), round(float(np.percentile(Q[:, 1], 99)), 2)],
                           xz_mid=[round(float(np.median(Q[:, 0])), 2), round(float(np.median(Q[:, 2])), 2)],
                           dist_poly_m=round(float(cv2.pointPolygonTest(poly.astype(np.float32).reshape(-1, 1, 2),
                                                                         (float(np.median(Q[:, 0])), float(np.median(Q[:, 2]))), True)), 2),
                           accepted=bool(ok), _pts=Q))
        print(pi, {k: v for k, v in planes[-1].items() if k != "_pts"}, flush=True)
        if sum(p["accepted"] for p in planes) >= a.max_planes:
            break
    acc = [p for p in planes if p["accepted"]]
    for i, p in enumerate(acc):   # near-parallel duplicates
        for q in acc[:i]:
            if not q["accepted"]:
                continue
            dn = float(np.dot(p["n"], q["n"]))
            if abs(dn) > np.cos(np.radians(15)) and abs(np.sign(dn) * p["off"] - q["off"]) < 0.30:
                eu = np.cross(Y, np.array(q["n"])); up = p["_pts"] @ eu
                if min(np.percentile(up, 99), q["u_range"][1]) - max(np.percentile(up, 1), q["u_range"][0]) > 0:
                    p["accepted"] = False; p["dup_of"] = q["id"]
                    break
    if a.only:
        keep = {int(x) for x in a.only.split(",")}
        for p in planes:
            if p["accepted"] and p["id"] not in keep:
                p["accepted"] = False; p["skipped_by_only"] = True
    acc = [p for p in planes if p["accepted"]]
    info["planes"] = [{k: v for k, v in p.items() if k != "_pts"} for p in planes]
    if a.detect_only:
        info["runtime_s"] = round(time.time() - t0, 1)
        np.savez_compressed(od / "w3_planepts.npz", **{f"p{p['id']}": p["_pts"].astype(np.float32) for p in planes})
        (od / "w3_detect.json").write_text(json.dumps(info, indent=1)); return
    # 3. holes
    m = o3d.io.read_triangle_mesh(a.inp); V = np.asarray(m.vertices, np.float64); F = np.asarray(m.triangles, np.int64)
    P0, P1, P2 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]; ms = np.concatenate([(P0 + P1 + P2) / 3, P0, (P0 + P1) / 2])
    sc = o3d.t.geometry.RaycastingScene(nthreads=8); sc.add_triangles(o3d.core.Tensor(V.astype(np.float32)), o3d.core.Tensor(F.astype(np.uint32)))
    polyc = poly.astype(np.float32).reshape(-1, 1, 2)
    cells = []
    for p in acc:
        nn = np.array(p["n"]); off = p["off"]; eu = np.cross(Y, nn); Q = p["_pts"]
        u0 = np.floor(min(Q @ eu) / a.cell) - 30; u1 = np.ceil(max(Q @ eu) / a.cell) + 30
        v0 = np.floor(ylo / a.cell); v1 = np.ceil(yhi / a.cell); nu, nv = int(u1 - u0), int(v1 - v0)

        def grid(X, mn=1):
            gu = np.floor(X @ eu / a.cell - u0).astype(int); gv = np.floor(X[:, 1] / a.cell - v0).astype(int)
            ok = (gu >= 0) & (gu < nu) & (gv >= 0) & (gv < nv); G = np.zeros((nv, nu), np.int32); np.add.at(G, (gv[ok], gu[ok]), 1); return G >= mn
        sup = grid(Q, 2); mc = grid(ms[np.abs(ms @ nn - off) < 0.05])
        k = int(round(a.close / a.cell)) | 1; ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        ext = cv2.morphologyEx((sup | mc).astype(np.uint8), cv2.MORPH_CLOSE, ker) > 0
        gv, gu = np.nonzero(ext & ~mc)
        uc = (gu + u0 + 0.5) * a.cell; yv = (gv + v0 + 0.5) * a.cell
        C = eu[None] * uc[:, None] + nn[None] * off; C[:, 1] = yv
        inroom = np.array([cv2.pointPolygonTest(polyc, (float(x), float(z)), True) for x, z in C[:, [0, 2]]]) > -0.15 if len(C) else np.zeros(0, bool)
        keep = inroom & (yv > ylo + 0.02) & (yv < yhi - 0.02)
        p.update(grid=dict(u0=float(u0), v0=float(v0), nu=nu, nv=nv), sup_cells=int(sup.sum()), meshcov_cells=int(mc.sum()),
                 extent_cells=int(ext.sum()), hole_cells=int((ext & ~mc).sum()), hole_inroom_cells=int(keep.sum()))
        cells.append(dict(C=C[keep], gu=gu[keep], gv=gv[keep]))
    # 4. guards
    allC = np.concatenate([x["C"] for x in cells]) if cells else np.zeros((0, 3))
    pid = np.concatenate([np.full(len(x["C"]), i) for i, x in enumerate(cells)]).astype(int) if cells else np.zeros(0, int)
    GU = np.concatenate([x["gu"] for x in cells]) if cells else np.zeros(0, int); GV = np.concatenate([x["gv"] for x in cells]) if cells else np.zeros(0, int)
    NN = np.array([acc[i]["n"] for i in pid]) if len(pid) else np.zeros((0, 3))
    z0 = lambda: np.zeros(len(allC), np.int32)  # noqa: E731
    see, agr, occ, beh, clr, vis = z0(), z0(), z0(), z0(), z0(), z0(); side = np.zeros(len(allC))
    for j in range(len(T)):
        Rm, t = T[j][:3, :3], T[j][:3, 3]; Xc = (allC - t) @ Rm; z = Xc[:, 2]; okz = (z > 0.1) & (z < 6)
        u = np.floor(K[0, 0] * Xc[:, 0] / np.where(okz, z, 1) + K[0, 2]).astype(np.int64); v = np.floor(K[1, 1] * Xc[:, 1] / np.where(okz, z, 1) + K[1, 2]).astype(np.int64)
        okz &= (u >= 0) & (u < w) & (v >= 0) & (v < h)
        dv = allC - t; dist = np.linalg.norm(dv, axis=1); cs = (dv * NN).sum(1) / np.maximum(dist, 1e-9)
        okz &= np.abs(cs) > 0.2
        idx = np.nonzero(okz)[0]
        if len(idx) == 0:
            continue
        side[idx] += np.sign(-cs[idx])
        fp = Path(a.fused) / f"{int(kf[j]):05d}.npz"
        if fp.exists():
            d = np.load(fp); Zf = d["Z"][v[idx], u[idx]]; Ff = d["F"][v[idx], u[idx]] == 1
            tol = np.maximum(0.05, 0.04 * z[idx])
            see[idx] += (Ff & (Zf > z[idx] + tol)).astype(np.int32); agr[idx] += (Ff & (np.abs(Zf - z[idx]) <= tol)).astype(np.int32)
        dd = (dv[idx] / dist[idx, None]).astype(np.float32)
        r = sc.cast_rays(o3d.core.Tensor(np.ascontiguousarray(np.concatenate([np.broadcast_to(t.astype(np.float32), dd.shape), dd], 1))), nthreads=8)["t_hit"].numpy()
        o_ = r < dist[idx] - 0.05; b_ = np.isfinite(r) & (r > dist[idx] + 0.10)
        occ[idx] += o_; beh[idx] += b_; clr[idx] += ~o_; vis[idx] += 1
    ffs_ok = (see == 0) | ((see == 1) & (agr >= 2))
    fill = ffs_ok & (beh <= 1) & (clr >= 2); small = np.zeros(len(fill), bool)
    if a.min_comp > 0:
        for i, p in enumerate(acc):
            s_ = np.nonzero((pid == i) & fill)[0]; g = p["grid"]; G = np.zeros((g["nv"], g["nu"]), np.uint8); G[GV[s_], GU[s_]] = 1
            nl, lab = cv2.connectedComponents(G, connectivity=4); sz = np.bincount(lab.ravel(), minlength=nl) * a.cell ** 2
            small[s_] = sz[lab[GV[s_], GU[s_]]] < a.min_comp
        fill &= ~small
    # 5. faces
    newV, newF = [], []; nb = len(V); A = a.cell ** 2
    for i, p in enumerate(acc):
        s = pid == i; nn = np.array(p["n"]); off = p["off"]; eu = np.cross(Y, nn); g = p["grid"]
        towards = nn * (1 if side[s].sum() >= 0 else -1)
        p.update(cand_m2=round(float(s.sum() * A), 3), filled_m2=round(float((fill & s).sum() * A), 3),
                 blocked_m2=dict(ffs_see_through=round(float((s & ~ffs_ok).sum() * A), 3), mesh_behind=round(float((s & (beh > 1)).sum() * A), 3),
                                 not_visible=round(float((s & (clr < 2)).sum() * A), 3), small_patch=round(float((s & small).sum() * A), 3)),
                 faces_towards=towards.round(3).tolist())
        for c_ in np.nonzero(fill & s)[0]:
            ua = (GU[c_] + g["u0"]) * a.cell; va = (GV[c_] + g["v0"]) * a.cell
            b0 = eu * ua + nn * off; b0[1] = va; b1 = b0 + eu * a.cell; b2 = b1 + Y * a.cell; b3 = b0 + Y * a.cell
            cs = np.array([b0, b1, b2, b3]); bb = nb + 4 * len(newV); newV.append(cs)
            tri = [[bb, bb + 1, bb + 2], [bb, bb + 2, bb + 3]]; nt = np.cross(cs[1] - cs[0], cs[2] - cs[0])
            newF += tri if nt @ towards > 0 else [t_[::-1] for t_ in tri]
    nf0 = len(F)
    if newV:
        V = np.concatenate([V, np.concatenate(newV)]); F = np.concatenate([F, np.array(newF, np.int64)])
    mo = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V), o3d.utility.Vector3iVector(F.astype(np.int32)))
    o3d.io.write_triangle_mesh(str(od / "w3.ply"), mo, write_ascii=False, write_vertex_normals=False)
    np.savez_compressed(od / "w3_fill.npz", C=allC, pid=pid, see=see, agr=agr, occ=occ, beh=beh, clr=clr, vis=vis, fill=fill, nf0=nf0,
                        plane_ids=np.array([p["id"] for p in acc]))
    info["planes"] = [{k: v for k, v in p.items() if k != "_pts"} for p in planes]
    info.update(faces_in=int(nf0), generated_faces=int(len(F) - nf0), filled_m2_total=round(float(fill.sum() * A), 3),
                cand_m2_total=round(float(len(fill) * A), 3), runtime_s=round(time.time() - t0, 1))
    (od / "w3.json").write_text(json.dumps(info, indent=1)); print(json.dumps({k: v for k, v in info.items() if k != "planes"}), flush=True)


if __name__ == "__main__":
    main()
