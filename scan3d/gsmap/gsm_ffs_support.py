#!/usr/bin/env python3
"""gsmap 2026-11-26: FFS-support prune of a cleaned mesh (no retrain), CPU.

  python scan3d/gsmap/gsm_ffs_support.py IN.ply OUTDIR --fused FUSEDIR --cams CAMS.npz [--n 1,2] [--island 0.01]

Idea: in the FFS-only 2DGS map, wherever FFS has no depth (white walls, ceiling, parts of the floor) the surface comes from colour alone
(shards, floating sheets). Keep only the faces that FFS depth confirms.

Per keyframe j (all keyframes of CAMS = the training poses; fused depth FUSEDIR/<kf>.npz with Z (z-depth, m), F (1 = FFS), S (sigma, m)):
  face centre -> camera (OpenCV, T = cam->world), z > 0.1, pixel inside 960x540, |cos(normal, view ray)| > 0.15 (same grazing cut as eval2 P1)
  occlusion: z-buffer of the mesh itself (Open3D ray cast at the pixel centres, full res); the face is visible in view j when
             z <= zbuf + max(occ_eps, occ_rel * z)
  FFS check at that pixel (F == 1): d = z - Z_ffs, tol = clip(k_sig * S, tol_min, tol_max)
     agree  |d| <= tol;  front  d < -tol (face floats in front of the FFS surface);  behind  d > tol (FFS sees something closer)
  visible but F == 0 -> 'no FFS' view.
Face classes (for each N): supported = agree >= N. The rest:
     wrong  = at least one FFS view disagrees and agree < N (split front / behind by majority)
     weak   = agree in 1..N-1 and no disagreeing view (only for N >= 2)
     never  = no visible view with FFS depth at all
Prune = drop unsupported faces, then drop welded components < --island m2. Writes OUTDIR/support_N<n>.ply per N, OUTDIR/support.json
(area shares, counts, thresholds, inputs, time) and OUTDIR/support_votes.npz (per-face counters).
"""
import argparse, json, sys, time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))


def components(V, F):
    import open3d as o3d
    m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V), o3d.utility.Vector3iVector(F.astype(np.int32)))
    m.merge_close_vertices(1e-5)
    cl, nt, ar = m.cluster_connected_triangles()
    return np.asarray(cl), np.asarray(nt), np.asarray(ar)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("inp"); ap.add_argument("outdir"); ap.add_argument("--fused", required=True); ap.add_argument("--cams", required=True)
    ap.add_argument("--n", default="1,2"); ap.add_argument("--k-sig", type=float, default=3.0); ap.add_argument("--tol-min", type=float, default=0.03)
    ap.add_argument("--tol-max", type=float, default=0.15); ap.add_argument("--occ-eps", type=float, default=0.02); ap.add_argument("--occ-rel", type=float, default=0.01)
    ap.add_argument("--cos-min", type=float, default=0.15); ap.add_argument("--island", type=float, default=0.01)
    ap.add_argument("--votes", default="", help="reuse OUTDIR/support_votes.npz from an earlier run (skip the per-view pass)")
    ap.add_argument("--comp-area", type=float, default=0.0, help=">0: mild variant 'C': drop only welded components < this m2 with NO face "
                    "agreeing with FFS (agree == 0 on every face); big unsupported surfaces kept -> support_NC.ply, json support_NC.json")
    a = ap.parse_args()
    import open3d as o3d
    t0 = time.time(); od = Path(a.outdir); od.mkdir(parents=True, exist_ok=True)
    m = o3d.io.read_triangle_mesh(a.inp); V = np.asarray(m.vertices, np.float64); F = np.asarray(m.triangles, np.int64)
    P0, P1, P2 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    nrm = np.cross(P1 - P0, P2 - P0); area = 0.5 * np.linalg.norm(nrm, axis=1); A0 = float(area.sum())
    nrm = (nrm / (2 * area[:, None] + 1e-12)).astype(np.float32); cen = ((P0 + P1 + P2) / 3).astype(np.float32)
    c = np.load(a.cams); kf, T, K = c["kf"], c["T"], c["Kr"]; w, h = [int(v) for v in c["size"]]
    sc = o3d.t.geometry.RaycastingScene(); sc.add_triangles(o3d.core.Tensor(V.astype(np.float32)), o3d.core.Tensor(F.astype(np.uint32)))
    uu, vv = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    ray = np.stack([(uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones_like(uu)], -1).reshape(-1, 3)   # z = 1 -> t_hit = z-depth
    if a.votes:
        return mild(a, V, F, area, A0, np.load(a.votes), od, t0)
    nF = len(F); agree = np.zeros(nF, np.int16); front = np.zeros(nF, np.int16); behind = np.zeros(nF, np.int16)
    noffs = np.zeros(nF, np.int16); occl = np.zeros(nF, np.int16)
    dz_agree = []; n_views_used = 0; ffs_valid = []
    for j in range(len(kf)):
        fp = Path(a.fused) / f"{int(kf[j]):05d}.npz"
        if not fp.exists():
            continue
        d_ = np.load(fp); Fm = d_["F"] == 1; ffs_valid.append(float(Fm.mean()))
        if not Fm.any():
            continue
        Z = d_["Z"].astype(np.float32); S = d_["S"].astype(np.float32); n_views_used += 1
        Rm, t = T[j][:3, :3], T[j][:3, 3]
        zb = sc.cast_rays(o3d.core.Tensor(np.concatenate([np.broadcast_to(t, ray.shape), ray @ Rm.T], 1).astype(np.float32)))["t_hit"].numpy().reshape(h, w)
        R32, t32 = Rm.astype(np.float32), t.astype(np.float32)
        Xc = (cen - t32) @ R32; z = Xc[:, 2]; ok = z > 0.1
        u = np.floor(K[0, 0] * Xc[:, 0] / np.where(ok, z, 1) + K[0, 2]).astype(np.int32); v = np.floor(K[1, 1] * Xc[:, 1] / np.where(ok, z, 1) + K[1, 2]).astype(np.int32)
        ok &= (u >= 0) & (u < w) & (v >= 0) & (v < h); idx = np.nonzero(ok)[0]
        if len(idx) == 0:
            continue
        vd = t32 - cen[idx]; vd /= np.linalg.norm(vd, axis=1, keepdims=True)
        idx = idx[np.abs((nrm[idx] * vd).sum(1)) > a.cos_min]
        uu_, vv_, zz = u[idx], v[idx], z[idx]
        vis = zz <= zb[vv_, uu_] + np.maximum(a.occ_eps, a.occ_rel * zz)
        occl[idx[~vis]] += 1; idx, uu_, vv_, zz = idx[vis], uu_[vis], vv_[vis], zz[vis]
        has = Fm[vv_, uu_]; noffs[idx[~has]] += 1
        idx, uu_, vv_, zz = idx[has], uu_[has], vv_[has], zz[has]
        dd = zz - Z[vv_, uu_]; tol = np.clip(a.k_sig * S[vv_, uu_], a.tol_min, a.tol_max)
        ag = np.abs(dd) <= tol; agree[idx[ag]] += 1; front[idx[dd < -tol]] += 1; behind[idx[dd > tol]] += 1
        if j % 10 == 0:
            dz_agree.append(dd[ag][::20])
        if j % 50 == 0:
            print(f"view {j}/{len(kf)} t={time.time() - t0:.0f}s", flush=True)
    np.savez_compressed(od / "support_votes.npz", agree=agree, front=front, behind=behind, noffs=noffs, occluded=occl)
    dz = np.concatenate(dz_agree) if dz_agree else np.zeros(1)
    pct = lambda msk: round(float(area[msk].sum() / A0 * 100), 3)  # noqa: E731
    wrongv = (front + behind) > 0
    info = dict(made=time.strftime("%F %T"), inp=a.inp, fused=a.fused, cams=a.cams, args=vars(a), faces_in=int(nF), area_in_m2=round(A0, 2),
                n_keyframes=int(len(kf)), n_views_with_ffs=n_views_used, ffs_valid_px_mean=round(float(np.mean(ffs_valid)), 4),
                rule=dict(tol=f"clip({a.k_sig}*S, {a.tol_min}, {a.tol_max}) m", occlusion=f"mesh z-buffer, visible if z <= zbuf + max({a.occ_eps}, {a.occ_rel}*z)",
                          grazing=f"|cos| > {a.cos_min}", z_min=0.1, pixel="face centre, floor(u), full res"),
                sanity_dz_agree_m=dict(median=round(float(np.median(dz)), 4), p05=round(float(np.percentile(dz, 5)), 4), p95=round(float(np.percentile(dz, 95)), 4)),
                face_view_area_pct=dict(never_ffs_seen=pct(~((agree + front + behind) > 0)), any_agree=pct(agree > 0), any_wrong_view=pct(wrongv),
                                        never_visible_in_any_kf=pct((agree + front + behind + noffs) == 0)),
                per_N={})
    for N in [int(x) for x in a.n.split(",")]:
        sup = agree >= N; never = (agree + front + behind) == 0; wrong = ~sup & wrongv; weak = ~sup & ~wrongv & ~never
        e = dict(supported_area_pct=pct(sup), unsupported_area_pct=pct(~sup), never_seen_area_pct=pct(~sup & never),
                 wrong_area_pct=pct(wrong), wrong_front_major_area_pct=pct(wrong & (front >= behind)), wrong_behind_major_area_pct=pct(wrong & (behind > front)),
                 weak_area_pct=pct(weak))
        Fk = F[sup]; cl, nt, ar = components(V, Fk); small = (ar < a.island)[cl]
        e.update(islands_removed=int((ar < a.island).sum()), island_area_pct=round(float(area[sup][small].sum() / A0 * 100), 3))
        Fk = Fk[~small]; used = np.zeros(len(V), bool); used[Fk.ravel()] = True; rmp = -np.ones(len(V), np.int64); rmp[used] = np.arange(used.sum())
        Vo = V[used]; Fo = rmp[Fk]; Ao = float(area[sup][~small].sum())
        e.update(faces_out=int(len(Fo)), area_out_m2=round(Ao, 2), removed_area_pct=round((1 - Ao / A0) * 100, 3), out=str(od / f"support_N{N}.ply"))
        mo = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(Vo), o3d.utility.Vector3iVector(Fo.astype(np.int32)))
        o3d.io.write_triangle_mesh(e["out"], mo, write_ascii=False, write_vertex_normals=False)
        info["per_N"][f"N{N}"] = e
    info["runtime_s"] = round(time.time() - t0, 1)
    (od / "support.json").write_text(json.dumps(info, indent=1)); print(json.dumps(info), flush=True)


def mild(a, V, F, area, A0, vt, od, t0):
    import open3d as o3d
    agree = vt["agree"]; assert len(agree) == len(F), "votes do not match the mesh"
    cl, nt, ar = components(V, F); sup = np.bincount(cl, weights=(agree > 0).astype(float), minlength=len(ar)) > 0
    rmc = (ar < a.comp_area) & ~sup; rm = rmc[cl]
    Fk = F[~rm]; used = np.zeros(len(V), bool); used[Fk.ravel()] = True; rmp = -np.ones(len(V), np.int64); rmp[used] = np.arange(used.sum())
    out = od / "support_NC.ply"; Ao = float(area[~rm].sum())
    o3d.io.write_triangle_mesh(str(out), o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V[used]), o3d.utility.Vector3iVector(rmp[Fk].astype(np.int32))),
                               write_ascii=False, write_vertex_normals=False)
    info = dict(made=time.strftime("%F %T"), inp=a.inp, votes=a.votes, args=vars(a), variant="C: drop welded components < comp_area m2 with no FFS-agreeing face",
                faces_in=int(len(F)), area_in_m2=round(A0, 2), components_in=int(len(ar)), components_removed=int(rmc.sum()),
                components_lt_area=int((ar < a.comp_area).sum()), removed_area_m2=round(float(area[rm].sum()), 3),
                removed_area_pct=round(float(area[rm].sum() / A0 * 100), 3), supported_area_removed_m2=round(float(area[rm & (agree > 0)].sum()), 4),
                faces_out=int(len(Fk)), area_out_m2=round(Ao, 2), out=str(out), runtime_s=round(time.time() - t0, 1))
    (od / "support_NC.json").write_text(json.dumps(info, indent=1)); print(json.dumps(info), flush=True)


if __name__ == "__main__":
    main()
