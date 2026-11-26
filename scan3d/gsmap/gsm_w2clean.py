#!/usr/bin/env python3
"""gsmap 2026-11-26 overnight W2: mid-level cleanup of floating shards / sheets (aisle, cabinets) on an FFS-only 2DGS mesh. CPU.

  python gsm_w2clean.py votes IN.ply OUT.npz --fused FUSEDIR --cams CAMS.npz [--drop DROP.npy]
  python gsm_w2clean.py apply IN.ply VOTES.npz OUTDIR --base-votes support_votes.npz [--votes-b VOTES_B.npz] [rule args]

votes (per keyframe j, same projection / grazing / z-buffer code as gsm_ffs_support.py; fused FFS depth Z, F == 1, sigma S, full res):
  face centre -> pixel (u, v), z;  tol = max(tol_min, k_sig * S)  (no upper cap)
  centre pixel, face visible in the mesh z-buffer (z <= zbuf + max(2 cm, 1 % z)):
     agree_c  |z - Z| <= tol;   front_c  z < Z - tol  (face sits in FFS-observed free space = contradicted, in front)
  window (no z-buffer: a free-space claim does not depend on what the mesh puts in front; FFS saw past this point):
     fs<r>    z < min over F == 1 pixels within +-r px of (Z - tol)   (in front of EVERY nearby FFS surface), r = 4, 12
     agree_w  any of the 9 samples at offsets {-4, 0, 4}^2 with F == 1 and |z - Z| <= tol
     seen_w   any F == 1 pixel within +-4 px
  backed (one extra ray cast per candidate): front_backed = front_c AND a mesh surface lies within +-tol of the FFS depth on that ray;
     fs12_backed = fs12 AND the first mesh surface behind the face (from z + 2 cm) is no farther than max over the window of (Z + tol).
     Round 1 showed unbacked removal only turns 'front' into 'hole' (the true surface is not in the mesh) -> use the backed fields.
  --drop: faces (bool mask over IN faces) left out of the z-buffer (second pass after the first removal: hidden sheets become visible).
apply:
  type A contradicted-front: front_c >= a_min and front_c >= a_ratio * agree_c (pass 1, or max of pass 1 / pass 2 with --votes-b)
  type B free-space sheet:   fs<r> >= b_min and fs<r> >= b_ratio * agree_w and agree_c(base) == 0, optional component gate:
                             welded component (after A) with FFS-agreeing (base agree >= 1) area share < b_comp_sup and area >= b_comp_min
  type D (optional, --d-dist > 0; a heuristic, NOT free-space evidence): base agree == 0, no FFS pixel within +-4 px in any view
                             (seen_w == 0) and face centre > d_dist m from every kept face with base agree >= d_sup
  type C fragments:         after A + B, welded components < frag_area m2 with no FFS-agreeing face (base votes) that LOST faces in A/B
Writes OUTDIR/w2.ply, OUTDIR/w2_votes.npz (base support votes subset, same face order -> gsm_ffs_support.py --votes ... --comp-area),
OUTDIR/removed.npz (per IN face: 0 kept, 1 A, 2 B, 3 C) and OUTDIR/w2.json (areas by type, whole and aisle ROI, inputs, time).
"""
import argparse, json, sys, time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gsm_ffs_support import components  # noqa: E402

ROI = dict(x=(-0.3, 3.3), z=(-5.0, -3.2))


def geom(V, F):
    P0, P1, P2 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    nrm = np.cross(P1 - P0, P2 - P0); area = 0.5 * np.linalg.norm(nrm, axis=1)
    nrm = (nrm / (2 * area[:, None] + 1e-12)).astype(np.float32); cen = ((P0 + P1 + P2) / 3).astype(np.float32)
    return area, nrm, cen


def load(p):
    import open3d as o3d
    m = o3d.io.read_triangle_mesh(p); return np.asarray(m.vertices, np.float64), np.asarray(m.triangles, np.int64)


def cmd_votes(a):
    import open3d as o3d
    t0 = time.time(); V, F = load(a.inp); area, nrm, cen = geom(V, F); nF = len(F)
    c = np.load(a.cams); kf, T, K = c["kf"], c["T"], c["Kr"]; w, h = [int(v) for v in c["size"]]
    keep = np.ones(nF, bool)
    if a.drop:
        keep &= ~np.load(a.drop)
    sc = o3d.t.geometry.RaycastingScene(nthreads=8)
    sc.add_triangles(o3d.core.Tensor(V.astype(np.float32)), o3d.core.Tensor(F[keep].astype(np.uint32)))
    uu, vv = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    ray = np.stack([(uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones_like(uu)], -1).reshape(-1, 3)
    names = ["agree_c", "front_c", "behind_c", "fs4", "fs12", "fs4_vis", "agree_w", "seen_w", "vis", "inview", "front_backed", "fs12_backed"]
    C = {k: np.zeros(nF, np.int16) for k in names}
    offs = [(dy, dx) for dy in (-4, 0, 4) for dx in (-4, 0, 4)]; nv = 0
    for j in range(len(kf)):
        fp = Path(a.fused) / f"{int(kf[j]):05d}.npz"
        if not fp.exists():
            continue
        d_ = np.load(fp); Fm = (d_["F"] == 1) & np.isfinite(d_["Z"])
        if not Fm.any():
            continue
        nv += 1
        Z = np.where(Fm, d_["Z"], np.inf).astype(np.float32); S = np.where(Fm, d_["S"], 0).astype(np.float32)
        TOL = np.maximum(a.tol_min, a.k_sig * S); Zlo = np.where(Fm, Z - TOL, np.inf).astype(np.float32)
        lo4 = cv2.erode(Zlo, np.ones((9, 9), np.uint8), borderType=cv2.BORDER_CONSTANT, borderValue=np.inf)
        lo12 = cv2.erode(Zlo, np.ones((25, 25), np.uint8), borderType=cv2.BORDER_CONSTANT, borderValue=np.inf)
        hi12 = cv2.dilate(np.where(Fm, Z + TOL, -np.inf).astype(np.float32), np.ones((25, 25), np.uint8), borderType=cv2.BORDER_CONSTANT, borderValue=-np.inf)
        seen4 = cv2.dilate(Fm.astype(np.uint8), np.ones((9, 9), np.uint8)) > 0
        Rm, t = T[j][:3, :3], T[j][:3, 3]
        zb = sc.cast_rays(o3d.core.Tensor(np.concatenate([np.broadcast_to(t, ray.shape), ray @ Rm.T], 1).astype(np.float32)), nthreads=8)["t_hit"].numpy().reshape(h, w)
        R32, t32 = Rm.astype(np.float32), t.astype(np.float32)
        Xc = (cen - t32) @ R32; z = Xc[:, 2]; ok = z > 0.1
        u = np.floor(K[0, 0] * Xc[:, 0] / np.where(ok, z, 1) + K[0, 2]).astype(np.int32); v = np.floor(K[1, 1] * Xc[:, 1] / np.where(ok, z, 1) + K[1, 2]).astype(np.int32)
        ok &= (u >= 0) & (u < w) & (v >= 0) & (v < h); idx = np.nonzero(ok)[0]
        if len(idx) == 0:
            continue
        vd = t32 - cen[idx]; vd /= np.linalg.norm(vd, axis=1, keepdims=True)
        idx = idx[np.abs((nrm[idx] * vd).sum(1)) > a.cos_min]
        uu_, vv_, zz = u[idx], v[idx], z[idx]; C["inview"][idx] += 1
        vis = zz <= zb[vv_, uu_] + np.maximum(a.occ_eps, a.occ_rel * zz); C["vis"][idx[vis]] += 1
        # centre pixel (visible only)
        Zc = Z[vv_, uu_]; tc = TOL[vv_, uu_]; has = Fm[vv_, uu_] & vis; dd = zz - Zc
        C["agree_c"][idx[has & (np.abs(dd) <= tc)]] += 1; C["front_c"][idx[has & (dd < -tc)]] += 1; C["behind_c"][idx[has & (dd > tc)]] += 1
        # window free space (no z-buffer)
        l4 = lo4[vv_, uu_]; l12 = lo12[vv_, uu_]
        f4 = np.isfinite(l4) & (zz < l4); f12 = np.isfinite(l12) & (zz < l12)   # window with no FFS pixel = no vote
        C["fs4"][idx[f4]] += 1; C["fs12"][idx[f12]] += 1; C["fs4_vis"][idx[f4 & vis]] += 1
        C["seen_w"][idx[seen4[vv_, uu_]]] += 1
        # 'backed': removing the face would reveal a mesh surface the FFS agrees with (not a hole). One extra ray per candidate pixel.
        dcam = np.stack([(uu_ + 0.5 - K[0, 2]) / K[0, 0], (vv_ + 0.5 - K[1, 2]) / K[1, 1], np.ones(len(uu_))], 1).astype(np.float32) @ R32.T
        fc = has & (dd < -tc)
        if fc.any():   # front at the centre pixel: any mesh surface within +-tol of the FFS depth
            st = (Zc[fc] - tc[fc])[:, None]
            th = sc.cast_rays(o3d.core.Tensor(np.concatenate([t32 + dcam[fc] * st, dcam[fc]], 1).astype(np.float32)), nthreads=8)["t_hit"].numpy()
            C["front_backed"][idx[fc][th <= 2 * tc[fc]]] += 1
        if f12.any():  # window free space: a mesh surface behind the face no farther than the farthest nearby FFS surface (+ tol)
            hi = hi12[vv_[f12], uu_[f12]]; st = (zz[f12] + a.occ_eps)[:, None]
            th = sc.cast_rays(o3d.core.Tensor(np.concatenate([t32 + dcam[f12] * st, dcam[f12]], 1).astype(np.float32)), nthreads=8)["t_hit"].numpy()
            C["fs12_backed"][idx[f12][np.isfinite(hi) & (zz[f12] + a.occ_eps + th <= hi)]] += 1
        aw = np.zeros(len(idx), bool)
        for dy, dx in offs:
            yy = np.clip(vv_ + dy, 0, h - 1); xx = np.clip(uu_ + dx, 0, w - 1)
            aw |= Fm[yy, xx] & (np.abs(zz - Z[yy, xx]) <= TOL[yy, xx])
        C["agree_w"][idx[aw]] += 1
        if j % 50 == 0:
            print(f"view {j}/{len(kf)} t={time.time() - t0:.0f}s", flush=True)
    np.savez_compressed(a.out, **C, made=time.strftime("%F %T"), inp=a.inp, fused=a.fused, cams=a.cams, drop=a.drop or "",
                        args=json.dumps({k: v for k, v in vars(a).items() if k != "func"}), n_views=nv)
    print(json.dumps(dict(out=a.out, n_views=nv, faces=nF, runtime_s=round(time.time() - t0, 1))), flush=True)


def cmd_apply(a):
    import open3d as o3d
    t0 = time.time(); V, F = load(a.inp); area, _, cen = geom(V, F); A0 = float(area.sum()); nF = len(F)
    vt = np.load(a.votes); base = np.load(a.base_votes); assert len(vt["agree_c"]) == nF and len(base["agree"]) == nF
    ag0 = base["agree"]
    frontc = vt[a.a_field].astype(np.int32); agc = vt["agree_c"].astype(np.int32)
    if a.votes_b:
        vb = np.load(a.votes_b); frontc = np.maximum(frontc, vb[a.a_field]); agc = np.maximum(agc, vb["agree_c"])
    typ = np.zeros(nF, np.uint8)
    rmA = (frontc >= a.a_min) & (frontc >= a.a_ratio * agc)
    if a.a_min > 0:
        typ[rmA] = 1
    if a.b_min > 0:
        fs = vt[a.b_field].astype(np.int32); agw = vt["agree_w"].astype(np.int32)
        cand = (typ == 0) & (fs >= a.b_min) & (fs >= a.b_ratio * agw) & (ag0 == 0) & (agc == 0)
        if a.b_comp_sup < 1.0 or a.b_comp_min > 0:
            kept = typ == 0; cl, nt, ar = components(V, F[kept]); clf = -np.ones(nF, np.int64); clf[kept] = cl
            sup_a = np.bincount(cl, weights=area[kept] * (ag0[kept] > 0), minlength=len(ar))
            gate = ((sup_a / np.maximum(ar, 1e-12)) < a.b_comp_sup) & (ar >= a.b_comp_min)
            cand &= kept & gate[np.maximum(clf, 0)]
        typ[cand] = 2
    if a.d_dist > 0:   # type D (NOT free-space evidence, a heuristic): no FFS pixel within +-4 px in any view and far from FFS-agreeing faces
        from scipy.spatial import cKDTree
        sup = (typ == 0) & (ag0 >= a.d_sup); cand = (typ == 0) & (ag0 == 0) & (agc == 0) & (vt["seen_w"] == 0) & (vt["vis"] >= a.d_vis)
        d, _ = cKDTree(cen[sup]).query(cen[cand], k=1, workers=8); ci = np.nonzero(cand)[0]; typ[ci[d > a.d_dist]] = 4
    if a.frag_area > 0:
        kept = typ == 0; cl, nt, ar = components(V, F[kept]); kidx = np.nonzero(kept)[0]
        # components that touch a removed face: vertex shared with a removed face
        rv = np.zeros(len(V), bool); rv[F[typ > 0].ravel()] = True
        touch = np.bincount(cl, weights=rv[F[kept]].any(1).astype(float), minlength=len(ar)) > 0
        sup = np.bincount(cl, weights=(ag0[kept] > 0).astype(float), minlength=len(ar)) > 0
        frag = (ar < a.frag_area) & ~sup & touch
        typ[kidx[frag[cl]]] = 3
    od = Path(a.outdir); od.mkdir(parents=True, exist_ok=True)
    keep = typ == 0; Fk = F[keep]; used = np.zeros(len(V), bool); used[Fk.ravel()] = True; rmp = -np.ones(len(V), np.int64); rmp[used] = np.arange(used.sum())
    o3d.io.write_triangle_mesh(str(od / "w2.ply"), o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V[used]), o3d.utility.Vector3iVector(rmp[Fk].astype(np.int32))),
                               write_ascii=False, write_vertex_normals=False)
    np.savez_compressed(od / "w2_votes.npz", **{k: base[k][keep] for k in ("agree", "front", "behind", "noffs", "occluded")})
    np.savez_compressed(od / "removed.npz", typ=typ, cen=cen, area=area.astype(np.float32))
    R = (cen[:, 0] > ROI["x"][0]) & (cen[:, 0] < ROI["x"][1]) & (cen[:, 2] > ROI["z"][0]) & (cen[:, 2] < ROI["z"][1])
    ar_ = lambda m: round(float(area[m].sum()), 3)  # noqa: E731
    info = dict(made=time.strftime("%F %T"), inp=a.inp, votes=a.votes, votes_b=a.votes_b, base_votes=a.base_votes,
                args={k: v for k, v in vars(a).items() if k != "func"}, faces_in=nF, area_in_m2=round(A0, 2), roi=ROI, roi_area_in_m2=ar_(R),
                removed_m2={n: dict(whole=ar_(typ == i), roi=ar_(R & (typ == i)), whole_ffs_agree_ge1=ar_((typ == i) & (ag0 > 0)))
                            for i, n in ((1, "A_contradicted_front"), (2, "B_freespace_sheet"), (3, "C_fragment"), (4, "D_unobserved_far"))},
                removed_total_m2=ar_(typ > 0), removed_pct=round(float(area[typ > 0].sum() / A0 * 100), 3),
                roi_removed_pct=round(float(area[R & (typ > 0)].sum() / max(1e-9, area[R].sum()) * 100), 2),
                faces_out=int(keep.sum()), out=str(od / "w2.ply"), runtime_s=round(time.time() - t0, 1))
    (od / "w2.json").write_text(json.dumps(info, indent=1)); print(json.dumps(info), flush=True)


def main():
    ap = argparse.ArgumentParser(); sp = ap.add_subparsers(required=True)
    p = sp.add_parser("votes"); p.set_defaults(func=cmd_votes); p.add_argument("inp"); p.add_argument("out")
    p.add_argument("--fused", required=True); p.add_argument("--cams", required=True); p.add_argument("--drop", default="")
    p.add_argument("--k-sig", type=float, default=3.0); p.add_argument("--tol-min", type=float, default=0.03)
    p.add_argument("--occ-eps", type=float, default=0.02); p.add_argument("--occ-rel", type=float, default=0.01); p.add_argument("--cos-min", type=float, default=0.15)
    p = sp.add_parser("apply"); p.set_defaults(func=cmd_apply); p.add_argument("inp"); p.add_argument("votes"); p.add_argument("outdir")
    p.add_argument("--base-votes", required=True); p.add_argument("--votes-b", default="")
    p.add_argument("--a-field", default="front_c", help="front_c | front_backed"); p.add_argument("--a-min", type=int, default=2); p.add_argument("--a-ratio", type=float, default=2.0)
    p.add_argument("--b-min", type=int, default=0); p.add_argument("--b-ratio", type=float, default=2.0); p.add_argument("--b-field", default="fs4")
    p.add_argument("--b-comp-sup", type=float, default=1.0); p.add_argument("--b-comp-min", type=float, default=0.0)
    p.add_argument("--frag-area", type=float, default=0.0)
    p.add_argument("--d-dist", type=float, default=0.0); p.add_argument("--d-sup", type=int, default=1)
    p.add_argument("--d-vis", type=int, default=0, help="D only for faces visible (z-buffer) in >= d_vis keyframes")
    a = ap.parse_args(); a.func(a)


if __name__ == "__main__":
    main()
