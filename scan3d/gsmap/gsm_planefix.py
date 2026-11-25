#!/usr/bin/env python3
"""gsmap 2026-11-25: snap unreliable crumpled vertices onto large planes (screen, table, wall) of a mesh.

  ~/scanner/gsmap_env/venv/bin/python scan3d/gsmap/gsm_planefix.py IN.ply OUT.ply [--fused fuse/moge_f]

1. planes: Open3D RANSAC (--thr) on the reliable (FFS-backed) mesh vertices, repeated on the rest; keep planes with >= --min-inl inlier vertices (max 6).
2. footprint per plane: 1 cm grid of inliers in plane coordinates, closed with a --close m kernel (fills holes / dents up to that size).
3. reliability per vertex: project into every keyframe; where the fused depth (Z, F) agrees with the vertex depth (3 cm / 3 %),
   count F == 1 (FFS kept after the multi-view + edge filters) vs F == 2 (MoGe fill). Unreliable = no agreeing observation, or
   FFS share < 0.5.
4. snap: vertex inside a plane footprint, |distance| > 1 cm and (unreliable and < --max-off, or BEHIND the plane as seen from the
   cameras and < --max-behind: an opaque plane hides everything behind it) -> projected onto the plane.
"""
import argparse, json, time
from pathlib import Path

import cv2
import numpy as np

R = Path(__file__).resolve().parent.parent / "runs" / "gsmap_20261125"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("inp"); ap.add_argument("out"); ap.add_argument("--fused", default=str(R / "fuse" / "moge_f"))
    ap.add_argument("--min-inl", type=int, default=2500); ap.add_argument("--close", type=float, default=0.15)
    ap.add_argument("--max-off", type=float, default=0.12); ap.add_argument("--max-behind", type=float, default=0.25)
    ap.add_argument("--iters", type=int, default=12); ap.add_argument("--all-inliers", type=int, default=0); ap.add_argument("--thr", type=float, default=0.005)
    ap.add_argument("--min-area", type=float, default=0.03); ap.add_argument("--min-fill", type=float, default=0.6)
    a = ap.parse_args()
    import open3d as o3d
    t0 = time.time()
    m = o3d.io.read_triangle_mesh(a.inp); V = np.asarray(m.vertices).copy(); nV = len(V)
    c = np.load(R / "dump_ba" / "cams.npz"); kf, T, K = c["kf"], c["T"], c["Kr"]; W, H = [int(v) for v in c["size"]]
    # reliability
    nffs = np.zeros(nV); nall = np.zeros(nV)
    for j, i in enumerate(kf):
        d = np.load(Path(a.fused) / f"{i:05d}.npz"); Z, F = d["Z"], d["F"]
        Rm, t = T[j][:3, :3], T[j][:3, 3]; Xc = (V - t) @ Rm; z = Xc[:, 2]; ok = z > 0.05
        u = np.round(K[0, 0] * Xc[:, 0] / np.where(ok, z, 1) + K[0, 2]).astype(int); v = np.round(K[1, 1] * Xc[:, 1] / np.where(ok, z, 1) + K[1, 2]).astype(int)
        ok &= (u >= 0) & (u < W) & (v >= 0) & (v < H)
        zr = np.full(nV, np.nan); fr = np.zeros(nV, np.uint8); zr[ok] = Z[v[ok], u[ok]]; fr[ok] = F[v[ok], u[ok]]
        ag = ok & np.isfinite(zr) & (np.abs(z - zr) < np.maximum(0.03, 0.03 * z))
        nall += ag; nffs += ag & (fr == 1)
    unrel = (nall == 0) | (nffs < 0.5 * nall)
    # planes
    # planes are fitted on RELIABLE (FFS-backed) vertices only; MoGe-only areas gave spurious parallel slabs
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(V)); rest = np.nonzero(~unrel)[0]; planes = []
    for _ in range(a.iters):
        if len(rest) < a.min_inl:
            break
        sub = pc.select_by_index(rest.tolist())
        model, inl = sub.segment_plane(distance_threshold=a.thr, ransac_n=3, num_iterations=2000)
        if len(inl) < a.min_inl:
            break
        planes.append((np.array(model), rest[np.array(inl)])); rest = np.setdiff1d(rest, rest[np.array(inl)])
    snapped = np.zeros(nV, bool); info = []
    for pi, (mdl, inl) in enumerate(planes):
        n = mdl[:3] / np.linalg.norm(mdl[:3]); dd = mdl[3] / np.linalg.norm(mdl[:3])
        e1 = np.cross(n, [1, 0, 0] if abs(n[0]) < 0.9 else [0, 1, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(n, e1)
        dist0 = V @ n + dd
        inl = np.nonzero(np.abs(dist0) < a.thr)[0] if a.all_inliers else inl     # footprint from all vertices on the plane
        q = np.stack([V @ e1, V @ e2], 1); qi = q[inl]
        lo = qi.min(0); g = np.floor((qi - lo) / 0.01).astype(int); gs = g.max(0) + 1
        occ = np.zeros((gs[1], gs[0]), np.uint8); occ[g[:, 1], g[:, 0]] = 1
        occ = cv2.dilate(occ, np.ones((3, 3), np.uint8))
        ncc, lab_, stt, _ = cv2.connectedComponentsWithStats(occ, connectivity=8)
        big = 1 + int(np.argmax(stt[1:, cv2.CC_STAT_AREA])) if ncc > 1 else 0
        occ = (lab_ == big).astype(np.uint8)
        k = int(round(a.close / 0.01)) | 1
        foot = cv2.morphologyEx(occ, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
        cnts, _ = cv2.findContours(foot, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE); foot = np.zeros_like(foot)
        cv2.drawContours(foot, cnts, -1, 1, -1); foot = foot.astype(bool)
        area_ = occ.sum() * 1e-4; fill_ = occ.sum() / max(1, foot.sum())
        if area_ < a.min_area or fill_ < a.min_fill:
            info.append(dict(plane=pi, normal=n.round(3).tolist(), inliers=int(len(inl)), area_m2=round(float(area_), 3),
                             fill=round(float(fill_), 2), rejected=True)); continue
        ga = np.floor((q - lo) / 0.01).astype(int); inside = (ga[:, 0] >= 0) & (ga[:, 0] < gs[0]) & (ga[:, 1] >= 0) & (ga[:, 1] < gs[1])
        inside[inside] = foot[ga[inside, 1], ga[inside, 0]]
        dist = V @ n + dd
        side = np.sign(np.mean(T[:, :3, 3] @ n + dd))            # cameras are on this side of the plane
        behind = dist * side < -0.01                                # behind an opaque plane seen from the front: cannot be observed
        sel = inside & ~snapped & (np.abs(dist) > 0.01) & ((unrel & (np.abs(dist) < a.max_off)) | (behind & (np.abs(dist) < a.max_behind)))
        V[sel] -= dist[sel, None] * n[None]; snapped |= sel
        info.append(dict(plane=pi, normal=n.round(3).tolist(), d=round(float(dd), 4), inliers=int(len(inl)),
                         area_m2=round(float(area_), 3), fill=round(float(fill_), 2),
                         extent_m=((gs) * 0.01).round(2).tolist(), snapped=int(sel.sum())))
    m.vertices = o3d.utility.Vector3dVector(V)
    o3d.io.write_triangle_mesh(a.out, m, write_ascii=False, write_vertex_normals=False, write_vertex_colors=True)
    res = dict(made=time.strftime("%F %T"), inp=a.inp, n_vertices=int(nV), unreliable_pct=round(float(unrel.mean() * 100), 2),
               snapped_total=int(snapped.sum()), planes=info, runtime_s=round(time.time() - t0, 1))
    Path(a.out).with_suffix(".planes.json").write_text(json.dumps(res, indent=1)); print(json.dumps(res, indent=1), flush=True)


if __name__ == "__main__":
    main()
