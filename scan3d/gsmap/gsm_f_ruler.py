#!/usr/bin/env python3
"""gsmap W4 2026-11-27: the F ruler, independent of FFS and of 2DGS (CPU only, <= 8 threads).

  python scan3d/gsmap/gsm_f_ruler.py f1 [--reproj 1.0 --min-obs 3 --min-tri 2.0 --min-kf 0]   -> w4/f1_<mesh>.npz + w4/f1.json
  python scan3d/gsmap/gsm_f_ruler.py sanity                                                    -> w4/ruler.json (sanity section)
  python scan3d/gsmap/gsm_f_ruler.py f2f3                                                      -> w4/baselines.json + contact sheets
  python scan3d/gsmap/gsm_f_ruler.py probe                                                     -> w4/probe.json

F1  BA-ray piercing. Tracks + 2D observations of the BA2 solution (ba_partners2/ba/ba_state.npz, BA frame) are moved to the map frame
    of the ba2 meshes by the Sim3 that maps the BA camera centres onto poses_all.npz (exact by construction, checked below).
    Points kept: >= min_obs observations (frames with src == ba), max reprojection error < reproj px over the point's observations,
    triangulation angle (widest ray pair) > min_tri deg. Even point ids score, odd point ids are kept for culling (never for scoring).
    Each observation gives a free-space segment camera centre -> point, shortened at the point end by max(5 cm, 3 % of its length).
    Every mesh face hit by a segment is 'pierced' once by that segment (Open3D list_intersections, all hits, not only the first).
    pierced face = >= 3 segments from >= 2 distinct cameras.
F2  camera-path tube: mesh area whose face centre is within 15 cm of any posed camera centre (all frames of poses_all).
F3  fixed views (audit view of audit7/an2.py + 7 keyframe poses) rendered per mesh with FFS-support classes (per-face FFS votes,
    gsm_ffs_support.py): red = agree < 2 (FFS-unsupported), split into no-evidence and contradicted; the audit7 texture-red count
    (GLB texture R > 140, G < 90, B < 90) is given for the audit view where a GLB exists.
"""
import argparse, json, sys, time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
V2 = Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2"
OUT = V2 / "w4"
POSES = dict(ba2=V2 / "ba_partners2", bap=V2 / "ba_partners")
MESHES = dict(
    ffsonly2_e3c=dict(ply=V2 / "ffsonly_ba2/b/e3c.ply", votes=V2 / "w2/aisle/ffsonly2_e3c/support_votes.npz", frame="ba2", glb=V2 / "ffsonly_ba2/b/tex_121801_ffsonly2.glb"),
    ffsonly2_carve=dict(ply=V2 / "ffsonly_ba2/b/carve.ply", votes=V2 / "ffsonly_ba2/b/support/support_votes.npz", frame="ba2", glb=None),
    W2_final=dict(ply=V2 / "w2/final/e3c.ply", votes=V2 / "w2/aisle/w2_e3c/support_votes.npz", frame="ba2", glb=V2 / "w2/final/tex_121801_w2.glb"),
    W3A_fill=dict(ply=V2 / "w3/fill/w3.ply", votes=OUT / "votes_w3A/support_votes.npz", frame="ba2", glb=V2 / "w3/fill/tex_121801_w3.glb"),
    nC=dict(ply=V2 / "ffsonly_ba/support_test/nC/e3c.ply", votes=OUT / "votes_nC/support_votes.npz", frame="bap",
            glb=V2 / "ffsonly_ba/support_test/nC/tex_121801_ffssupport_NC.glb"),
)
F1_MESHES = ["ffsonly2_e3c", "ffsonly2_carve", "W2_final", "W3A_fill"]      # ba2 frame only (nC is bap: no BA2 tracks in its frame)
ROI = (-0.3, 3.3, -5.0, -3.2)                                                    # aisle: x, z (any y)
NT = 8
AUDIT_EYE, AUDIT_TGT, AUDIT_FL = (-0.9, -0.25, -3.75), (1.5, 0.55, -4.45), 520.0  # audit7/an2.py
W, H = 960, 540


def log(msg):
    with open(OUT / "run.log", "a") as f:
        f.write(time.strftime("%F %T ") + msg + "\n")
    print(msg, flush=True)


def load_mesh(p):
    import open3d as o3d
    m = o3d.io.read_triangle_mesh(str(p)); V = np.asarray(m.vertices, np.float64); F = np.asarray(m.triangles, np.int64)
    P0, P1, P2 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]; n = np.cross(P1 - P0, P2 - P0); ar = 0.5 * np.linalg.norm(n, axis=1)
    return V, F, (P0 + P1 + P2) / 3, ar, n / (2 * ar[:, None] + 1e-12)


def scene(V, F):
    import open3d as o3d
    s = o3d.t.geometry.RaycastingScene(); s.add_triangles(o3d.core.Tensor(V.astype(np.float32)), o3d.core.Tensor(F.astype(np.uint32))); return s


def roi_mask(c):
    return (c[:, 0] > ROI[0]) & (c[:, 0] < ROI[1]) & (c[:, 2] > ROI[2]) & (c[:, 2] < ROI[3])


def umeyama(A, B):
    ma, mb = A.mean(0), B.mean(0); a, b = A - ma, B - mb; U, S, Vt = np.linalg.svd(b.T @ a / len(A)); D = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        D[2, 2] = -1
    R = U @ D @ Vt; s = np.trace(np.diag(S) @ D) / (a ** 2).sum(1).mean(); return s, R, mb - s * R @ ma


def votes_for(name, nf):
    d = np.load(MESHES[name]["votes"]); ag = d["agree"].astype(np.int32)
    assert len(ag) == nf, f"{name}: votes {len(ag)} != faces {nf}"
    wrong = (d["front"] + d["behind"]).astype(np.int32)
    return ag, wrong


# ------------------------------------------------------------------ tracks
def tracks(a):
    st = np.load(POSES["ba2"] / "ba/ba_state.npz"); P = np.load(POSES["ba2"] / "poses_all.npz")
    R, t, X, cam, pt, uv, K = st["R1"], st["t1"], st["X1"], st["cam"], st["pt"], st["uv"].astype(np.float64), st["K"]
    chk = {}
    chk["nobs_match_poses_all"] = bool((np.bincount(cam, minlength=len(P["frames"])) == P["nobs"]).all())
    Xc = np.einsum("nij,nj->ni", R[cam], X[pt]) + t[cam]
    e = np.linalg.norm(np.stack([K[0, 0] * Xc[:, 0] / Xc[:, 2] + K[0, 2], K[1, 1] * Xc[:, 1] / Xc[:, 2] + K[1, 2]], 1) - uv, axis=1)
    isba = P["src"] == "ba"
    Cn = -np.einsum("nji,nj->ni", R, t); Cm = P["T"][:, :3, 3]
    s, Ra, ta = umeyama(Cn[isba], Cm[isba]); res = np.linalg.norm(s * Cn[isba] @ Ra.T + ta - Cm[isba], axis=1)
    Rm = np.einsum("ij,nkj->nik", Ra, R)                      # Ra @ R_wc, compare to map rotations
    rr = np.degrees(np.arccos(np.clip((np.einsum("nij,nij->n", Rm[isba], P["T"][isba, :3, :3]) - 1) / 2, -1, 1)))
    chk.update(sim3_scale=float(s), sim3_centre_resid_mm_max=float(res.max() * 1000), sim3_rot_resid_deg_max=float(rr.max()))
    Xm = s * X @ Ra.T + ta
    # reprojection of the map-frame points with poses_all (must equal the BA error)
    Tm = P["T"]; Rcw = np.transpose(Tm[:, :3, :3], (0, 2, 1)); tcw = -np.einsum("nij,nj->ni", Rcw, Tm[:, :3, 3])
    sel = isba[cam]
    Xc2 = np.einsum("nij,nj->ni", Rcw[cam[sel]], Xm[pt[sel]]) + tcw[cam[sel]]
    e2 = np.linalg.norm(np.stack([K[0, 0] * Xc2[:, 0] / Xc2[:, 2] + K[0, 2], K[1, 1] * Xc2[:, 1] / Xc2[:, 2] + K[1, 2]], 1) - uv[sel], axis=1)
    chk["map_frame_reproj_minus_ba_px_absmax"] = float(np.abs(e2 - e[sel]).max())
    # point filter
    cam, pt, e = cam[sel], pt[sel], e[sel]; npt = len(X)
    nob = np.bincount(pt, minlength=npt); emax = np.zeros(npt); np.maximum.at(emax, pt, e)
    C = Cm[cam]; d = C - Xm[pt]; d /= np.linalg.norm(d, axis=1, keepdims=True)
    mdir = np.zeros((npt, 3)); np.add.at(mdir, pt, d); mdir /= np.linalg.norm(mdir, axis=1, keepdims=True) + 1e-12
    cosm = (d * mdir[pt]).sum(1); far = np.full(npt, 2.0); np.minimum.at(far, pt, cosm)
    isfar = cosm <= far[pt] + 1e-12; dfar = np.zeros((npt, 3)); dfar[pt[isfar]] = d[isfar]
    cmin = np.full(npt, 2.0); np.minimum.at(cmin, pt, (d * dfar[pt]).sum(1)); tri = np.degrees(np.arccos(np.clip(cmin, -1, 1)))
    iskf = P["iskf"]; nkf = np.bincount(pt, weights=iskf[cam].astype(float), minlength=npt)
    keep = (nob >= a.min_obs) & (emax < a.reproj) & (tri > a.min_tri) & (nkf >= a.min_kf)
    filt = dict(points_total=int(npt), obs_total=int(len(st["cam"])), obs_src_ba=int(sel.sum()), pts_nobs_ge=int((nob >= a.min_obs).sum()),
                pts_reproj_ok=int(((nob >= a.min_obs) & (emax < a.reproj)).sum()), pts_kept=int(keep.sum()),
                obs_kept=int(keep[pt].sum()), tri_deg_med_kept=float(np.median(tri[keep])), args=dict(reproj_max_px=a.reproj, min_obs=a.min_obs, min_tri_deg=a.min_tri, min_kf_obs=a.min_kf))
    m = keep[pt]
    return dict(cam=cam[m], pt=pt[m], Xm=Xm, Cm=Cm, chk=chk, filt=filt)


def pierce(sc, Cs, Xs, nf, cam, chunk=250000):
    """all faces hit by each shortened segment -> (face, seg) pairs; returns per-segment blocked and per-face seg / distinct-cam counts"""
    import open3d as o3d
    d = Xs - Cs; L = np.linalg.norm(d, axis=1); d /= L[:, None]; Ls = L - np.maximum(0.05, 0.03 * L)
    blocked = np.zeros(len(L), bool); fs, fc = [], []
    for i in range(0, len(L), chunk):
        r = np.concatenate([Cs[i:i + chunk], d[i:i + chunk]], 1).astype(np.float32)
        li = sc.list_intersections(o3d.core.Tensor(r), nthreads=NT)
        sp = li["ray_splits"].numpy(); th = li["t_hit"].numpy(); pid = li["primitive_ids"].numpy()
        rid = np.repeat(np.arange(len(r)), np.diff(sp)); ok = (th < Ls[i + rid]) & (Ls[i + rid] > 0)
        blocked[i + np.unique(rid[ok])] = True
        fs.append(pid[ok].astype(np.int64)); fc.append(cam[i + rid[ok]].astype(np.int64))
    fs = np.concatenate(fs); fc = np.concatenate(fc)
    nseg = np.bincount(fs, minlength=nf)
    u = np.unique(fs * 4096 + fc); ncam = np.bincount(u // 4096, minlength=nf)
    return blocked, nseg.astype(np.int32), ncam.astype(np.int32), Ls


def cmd_f1(a):
    t0 = time.time(); log("START f1"); T = tracks(a)
    print(json.dumps(T["chk"]), json.dumps(T["filt"]), flush=True)
    cam, pt, Xm, Cm = T["cam"], T["pt"], T["Xm"], T["Cm"]; even = pt % 2 == 0
    res = dict(made=time.strftime("%F %T"), n_recordings=1, inputs=dict(ba_state=str(POSES["ba2"] / "ba/ba_state.npz"), poses=str(POSES["ba2"] / "poses_all.npz"),
                                                                       meshes={k: str(MESHES[k]["ply"]) for k in F1_MESHES}),
               frame_check=T["chk"], filter=T["filt"], n_seg_even=int(even.sum()), n_seg_odd=int((~even).sum()),
               rule="segment cam centre -> BA point, shortened by max(5 cm, 3 %) at the point end; pierced face = >= 3 segments from >= 2 cameras (even tracks)",
               meshes={})
    tag = a.tag
    for name in F1_MESHES:
        V, F, cen, ar, _ = load_mesh(MESHES[name]["ply"]); nf = len(F); sc = scene(V, F); R = roi_mask(cen)
        out = {}; arrs = {}
        for side, msk in (("even", even), ("odd", ~even)):
            bl, nseg, ncam, Ls = pierce(sc, Cm[cam[msk]], Xm[pt[msk]], nf, cam[msk])
            pf = (nseg >= 3) & (ncam >= 2); arrs[side + "_seg"] = nseg; arrs[side + "_cam"] = ncam
            out[side] = dict(n_seg=int(msk.sum()), blocked_frac=round(float(bl.mean()), 5), faces_hit=int((nseg > 0).sum()),
                             pierced_faces=int(pf.sum()), pierced_area_m2=round(float(ar[pf].sum()), 3), pierced_area_roi_m2=round(float(ar[pf & R].sum()), 3),
                             hit_area_m2=round(float(ar[nseg > 0].sum()), 3), hit_area_roi_m2=round(float(ar[(nseg > 0) & R].sum()), 3))
        # points vs mesh: distance of the even kept points to the mesh (frame check)
        import open3d as o3d
        up = np.unique(pt[even]); dist = sc.compute_distance(o3d.core.Tensor(Xm[up].astype(np.float32)), nthreads=NT).numpy()
        out["even_point_to_mesh_cm_p25_50_75"] = [round(float(x) * 100, 2) for x in np.percentile(dist, [25, 50, 75])]
        out["even_points_within_3cm_pct"] = round(float((dist < 0.03).mean() * 100), 1)
        out["area_m2"] = round(float(ar.sum()), 2); out["area_roi_m2"] = round(float(ar[R].sum()), 2); out["faces"] = int(nf)
        res["meshes"][name] = out
        np.savez_compressed(OUT / f"f1{tag}_{name}.npz", **arrs)
        log(f"f1 {name} even blocked {out['even']['blocked_frac']} pierced {out['even']['pierced_area_m2']} m2 roi {out['even']['pierced_area_roi_m2']} "
            f"pt-mesh med {out['even_point_to_mesh_cm_p25_50_75'][1]} cm  t={time.time() - t0:.0f}s")
    res["runtime_s"] = round(time.time() - t0, 1)
    (OUT / f"f1{tag}.json").write_text(json.dumps(res, indent=1)); log("END f1")


def cmd_sanity(a):
    tag = a.tag; f1 = json.loads((OUT / f"f1{tag}.json").read_text())
    res = dict(made=time.strftime("%F %T"), n_recordings=1, f1_json=str(OUT / f"f1{tag}.json"), f1_filter=f1["filter"], frame_check=f1["frame_check"],
               test="area share of FFS-supported faces (agree >= 2, per-face votes of gsm_ffs_support.py) that are F1-pierced (even tracks, >= 3 segs, >= 2 cams); pass < 1 %",
               meshes={})
    allpass = True
    for name in F1_MESHES:
        V, F, cen, ar, _ = load_mesh(MESHES[name]["ply"]); ag, wrong = votes_for(name, len(F)); R = roi_mask(cen)
        z = np.load(OUT / f"f1{tag}_{name}.npz"); pf = (z["even_seg"] >= 3) & (z["even_cam"] >= 2); pfo = (z["odd_seg"] >= 3) & (z["odd_cam"] >= 2)
        d = {}
        for g, gm in (("supported_ge2", ag >= 2), ("agree1", ag == 1), ("agree0", ag == 0)):
            for reg, rm in (("whole", np.ones(len(F), bool)), ("roi", R)):
                s = gm & rm; A = max(ar[s].sum(), 1e-9)
                d[f"{g}/{reg}"] = dict(area_m2=round(float(ar[s].sum()), 3), pierced_even_pct=round(float(ar[s & pf].sum() / A * 100), 3),
                                       pierced_odd_pct=round(float(ar[s & pfo].sum() / A * 100), 3))
        d["pass"] = bool(d["supported_ge2/whole"]["pierced_even_pct"] < 1.0); allpass &= d["pass"]
        d["votes"] = str(MESHES[name]["votes"]); res["meshes"][name] = d
        log(f"sanity {name} supported pierced {d['supported_ge2/whole']['pierced_even_pct']} % (roi {d['supported_ge2/roi']['pierced_even_pct']} %), "
            f"agree0 pierced {d['agree0/whole']['pierced_even_pct']} % (roi {d['agree0/roi']['pierced_even_pct']} %)")
    res["all_pass"] = bool(allpass)
    (OUT / f"sanity{tag}.json").write_text(json.dumps(res, indent=1))


# ------------------------------------------------------------------ F2 / F3
def view_poses():
    """audit view + 7 keyframe poses spread over the trajectory (frame ids fixed; each mesh uses its own pose solution)"""
    P = np.load(POSES["ba2"] / "poses_all.npz"); Q = np.load(POSES["bap"] / "poses_all.npz")
    qf = {int(f): k for k, f in enumerate(Q["frames"]) if Q["src"][k] == "ba"}
    ks = [k for k in np.flatnonzero(P["iskf"] & (P["src"] == "ba")) if int(P["frames"][k]) in qf]
    pick = [ks[int(round(i))] for i in np.linspace(len(ks) * 0.06, len(ks) * 0.94, 7)]
    return [int(P["frames"][k]) for k in pick]


def audit_T():
    eye = np.array(AUDIT_EYE); tgt = np.array(AUDIT_TGT); zc = tgt - eye; zc /= np.linalg.norm(zc); xc = np.cross([0, 1.0, 0], zc); xc /= np.linalg.norm(xc)
    yc = np.cross(zc, xc); T = np.eye(4); T[:3, :3] = np.stack([xc, yc, zc], 1); T[:3, 3] = eye; return T


def render(sc, T, fl=AUDIT_FL):
    import open3d as o3d
    uu, vv = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
    dc = np.stack([(uu - W / 2) / fl, (vv - H / 2) / fl, np.ones_like(uu)], -1).reshape(-1, 3); dw = dc @ T[:3, :3].T
    r = sc.cast_rays(o3d.core.Tensor(np.concatenate([np.broadcast_to(T[:3, 3], dw.shape), dw], 1).astype(np.float32)), nthreads=NT)
    pid = r["primitive_ids"].numpy().astype(np.int64); hit = pid != 4294967295; pn = r["primitive_normals"].numpy()
    cosv = np.zeros(len(pid)); cosv[hit] = np.abs((pn[hit] * dw[hit]).sum(1) / np.linalg.norm(dw[hit], axis=1))
    return pid, hit, cosv, dw


def cmd_f2f3(a):
    import cv2
    from scipy.spatial import cKDTree
    from gsm_eval2 import load_mesh as load_glb
    t0 = time.time(); log("START f2f3")
    frames = view_poses(); PZ = {p: np.load(POSES[p] / "poses_all.npz") for p in POSES}
    res = dict(made=time.strftime("%F %T"), n_recordings=1, f2_rule="mesh area with face centre within 15 cm of any posed camera centre (all frames of poses_all)",
               f3_rule="9 views: audit view (eye %s, target %s, f 520 px, 960x540, from audit7/an2.py) + 7 keyframe poses (own pose solution, f 520 px); "
                       "red px = hit pixels on faces with FFS agree < 2 (per-face votes); split no_evidence (front+behind == 0 or not contradicted) / contradicted "
                       "(front+behind >= 3 and > 2*agree, audit7 rule); tex_red_px_audit = audit7 texture-red count on the GLB (R > 140, G < 90, B < 90)" % (AUDIT_EYE, AUDIT_TGT),
               view_frames=frames, inputs={k: {kk: str(vv) for kk, vv in v.items() if vv is not None} for k, v in MESHES.items()}, meshes={})
    pal = np.array([[40, 40, 40], [200, 200, 200], [40, 40, 230], [230, 140, 30]], np.uint8)   # BGR: hole, supported, unsupported (red), contradicted (blue)
    tiles = {}
    for name, M in MESHES.items():
        V, F, cen, ar, _ = load_mesh(M["ply"]); nf = len(F); R = roi_mask(cen); ag, wrong = votes_for(name, nf)
        contra = (wrong >= 3) & (wrong > 2 * ag); cls_f = np.where(ag >= 2, 1, np.where(contra, 3, 2)).astype(np.uint8)
        P = PZ[M["frame"]]; cc = P["T"][:, :3, 3]
        dk, _ = cKDTree(cc).query(cen, workers=NT); tube = dk < 0.15
        o = dict(frame=M["frame"], n_cam_centres=int(len(cc)), area_m2=round(float(ar.sum()), 2), area_roi_m2=round(float(ar[R].sum()), 2),
                 f2_tube_area_m2=round(float(ar[tube].sum()), 4), f2_tube_area_roi_m2=round(float(ar[tube & R].sum()), 4),
                 unsupported_area_m2=round(float(ar[ag < 2].sum()), 2), unsupported_area_roi_m2=round(float(ar[(ag < 2) & R].sum()), 2), views={})
        sc = scene(V, F); fi = {int(f): k for k, f in enumerate(P["frames"])}
        Ts = [("audit", audit_T())] + [(f"kf{f}", P["T"][fi[f]]) for f in frames]
        tl = []
        for vn, T in Ts:
            pid, hit, cosv, dw = render(sc, T); cl = np.zeros(len(pid), np.uint8); cl[hit] = cls_f[pid[hit]]
            img = (pal[cl] * (0.35 + 0.65 * cosv)[:, None]).astype(np.uint8); img[~hit] = pal[0]
            n = int(hit.sum()); red = int((hit & (cl >= 2)).sum())
            o["views"][vn] = dict(hit_px=n, red_px=red, red_pct_of_hit=round(red / max(n, 1) * 100, 2), red_noevidence_px=int((cl == 2).sum()), red_contradicted_px=int((cl == 3).sum()))
            im = img.reshape(H, W, 3).copy(); cv2.putText(im, f"{name} {vn} red {red / max(n, 1) * 100:.1f}%", (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            tl.append(im)
        o["red_px_total_9views"] = int(sum(v["red_px"] for v in o["views"].values()))
        if M.get("glb") is not None and Path(M["glb"]).exists():
            GV, GF, GUV, tex = load_glb(M["glb"]); sg = scene(GV, GF); pid, hit, cosv, dw = render(sg, audit_T())
            uu, vv = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
            from open3d.core import Tensor  # noqa: F401
            import open3d as o3d
            rr = sg.cast_rays(o3d.core.Tensor(np.concatenate([np.broadcast_to(audit_T()[:3, 3], dw.shape), dw], 1).astype(np.float32)), nthreads=NT)
            bu = rr["primitive_uvs"].numpy(); p = pid[hit]; b1, b2 = bu[hit, 0], bu[hit, 1]; b0 = 1 - b1 - b2; Ah, Aw = tex.shape[:2]
            uv = b0[:, None] * GUV[GF[p, 0]] + b1[:, None] * GUV[GF[p, 1]] + b2[:, None] * GUV[GF[p, 2]]
            ti = np.zeros((H * W, 3), np.uint8); ti[:] = (255, 0, 255)
            ti[hit] = tex[np.clip((uv[:, 1] * Ah).astype(int), 0, Ah - 1), np.clip((uv[:, 0] * Aw).astype(int), 0, Aw - 1)]
            tred = hit & (ti[:, 2].astype(int) > 140) & (ti[:, 1].astype(int) < 90) & (ti[:, 0].astype(int) < 90)
            o["tex_red_px_audit"] = int(tred.sum()); o["glb"] = str(M["glb"])
            im = ti.reshape(H, W, 3).copy(); cv2.putText(im, f"{name} audit texture (GLB)", (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2); tl.insert(1, im)
        else:
            o["tex_red_px_audit"] = None; blank = np.full((H, W, 3), 30, np.uint8); cv2.putText(blank, "no GLB (pre-texture mesh)", (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2); tl.insert(1, blank)
        tiles[name] = tl
        sm = [cv2.resize(x, (W // 2, H // 2), interpolation=cv2.INTER_AREA) for x in tl[:9]]
        while len(sm) < 9:
            sm.append(np.zeros_like(sm[0]))
        sheet = np.vstack([np.hstack(sm[i:i + 3]) for i in (0, 3, 6)])
        cv2.imwrite(str(OUT / f"sheet_{name}.jpg"), sheet, [cv2.IMWRITE_JPEG_QUALITY, 88]); o["sheet"] = str(OUT / f"sheet_{name}.jpg")
        res["meshes"][name] = o
        log(f"f2f3 {name} tube {o['f2_tube_area_m2']} m2 (roi {o['f2_tube_area_roi_m2']}), audit red {o['views']['audit']['red_pct_of_hit']} %, tex red {o['tex_red_px_audit']}  t={time.time() - t0:.0f}s")
    # comparison sheet ffsonly2 / W2 / W3A: audit class, audit texture, 2 keyframe views
    cols = [0, 1, 4, 7]
    rows = [np.hstack([cv2.resize(tiles[n][c], (W // 2, H // 2), interpolation=cv2.INTER_AREA) for c in cols]) for n in ("ffsonly2_e3c", "W2_final", "W3A_fill")]
    leg = np.full((40, rows[0].shape[1], 3), 20, np.uint8)
    cv2.putText(leg, "grey = FFS-supported (agree>=2)   red = no FFS support   blue = FFS-contradicted   dark = hole", (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cmp_ = np.vstack([leg] + rows); cv2.imwrite(str(OUT / "sheet_compare_ffsonly2_W2_W3A.jpg"), cmp_, [cv2.IMWRITE_JPEG_QUALITY, 88])
    res["compare_sheet"] = str(OUT / "sheet_compare_ffsonly2_W2_W3A.jpg"); res["runtime_s"] = round(time.time() - t0, 1)
    (OUT / "f2f3.json").write_text(json.dumps(res, indent=1)); log("END f2f3")


# ------------------------------------------------------------------ probe (future L1 lever), ffsonly2 carve.ply, aisle ROI
def cmd_probe(a):
    import cv2, glob
    import open3d as o3d
    t0 = time.time(); log("START probe"); tag = a.tag; name = "ffsonly2_carve"
    V, F, cen, ar, nrm = load_mesh(MESHES[name]["ply"]); nf = len(F); ag, wrong = votes_for(name, nf); R = roi_mask(cen)
    idx_roi = np.flatnonzero(R); c32 = cen[idx_roi].astype(np.float32); n32 = nrm[idx_roi].astype(np.float32)
    c = np.load(POSES["ba2"] / "run/dump_ba/cams.npz"); kf, T, K = c["kf"], c["T"], c["Kr"]
    imgdir = Path(glob.glob(str(POSES["ba2"] / "cache/rec/*/"))[0])
    sc = scene(V, F); uu, vv = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
    ray = np.stack([(uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones_like(uu)], -1).reshape(-1, 3)
    thr_mult = [1.5, 2.5, 4.0]; n = len(idx_roi)
    nsee = np.zeros(n, np.int32); ntl = np.zeros((len(thr_mult), n), np.int32)
    csum = np.zeros((n, 3)); csq = np.zeros((n, 3)); sig = []
    lapk = np.array([[1, -2, 1], [-2, 4, -2], [1, -2, 1]], np.float32)
    # global noise level: Immerkaer estimate per keyframe, median over keyframes
    for j in range(0, len(kf), 5):
        g = cv2.cvtColor(cv2.imread(str(imgdir / f"{int(kf[j]):05d}.jpg")), cv2.COLOR_BGR2GRAY).astype(np.float32)
        s_ = np.sqrt(np.pi / 2) / (6 * (W - 2) * (H - 2)) * np.abs(cv2.filter2D(g, -1, lapk)[1:-1, 1:-1]).sum(); sig.append(float(s_))
    sig_n = float(np.median(sig)); thrs = [max(1.0, m * sig_n) for m in thr_mult]
    for j in range(len(kf)):
        Rm, t = T[j][:3, :3].astype(np.float32), T[j][:3, 3].astype(np.float32)
        Xc = (c32 - t) @ Rm; z = Xc[:, 2]; ok = z > 0.1
        u = np.floor(K[0, 0] * Xc[:, 0] / np.where(ok, z, 1) + K[0, 2]).astype(np.int32); v = np.floor(K[1, 1] * Xc[:, 1] / np.where(ok, z, 1) + K[1, 2]).astype(np.int32)
        ok &= (u >= 4) & (u < W - 4) & (v >= 4) & (v < H - 4); ii = np.flatnonzero(ok)
        if len(ii) == 0:
            continue
        vd = t - c32[ii]; vd /= np.linalg.norm(vd, axis=1, keepdims=True); ii = ii[np.abs((n32[ii] * vd).sum(1)) > 0.15]
        if len(ii) == 0:
            continue
        zb = sc.cast_rays(o3d.core.Tensor(np.concatenate([np.broadcast_to(T[j][:3, 3], ray.shape), ray @ T[j][:3, :3].T], 1).astype(np.float32)), nthreads=NT)["t_hit"].numpy().reshape(H, W)
        vis = z[ii] <= zb[v[ii], u[ii]] + np.maximum(0.02, 0.01 * z[ii]); ii = ii[vis]
        if len(ii) == 0:
            continue
        img = cv2.imread(str(imgdir / f"{int(kf[j]):05d}.jpg")).astype(np.float32); g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        m1 = cv2.boxFilter(g, -1, (9, 9)); m2 = cv2.boxFilter(g * g, -1, (9, 9)); sd = np.sqrt(np.maximum(m2 - m1 * m1, 0))
        col = cv2.boxFilter(img, -1, (3, 3))
        s9 = sd[v[ii], u[ii]]; nsee[ii] += 1
        for q, th in enumerate(thrs):
            ntl[q, ii] += (s9 < th)
        cv_ = col[v[ii], u[ii]].astype(np.float64); csum[ii] += cv_; csq[ii] += cv_ ** 2
        if j % 50 == 0:
            print(f"probe view {j}/{len(kf)} t={time.time() - t0:.0f}s", flush=True)
    share = ntl / np.maximum(nsee, 1); cstd = np.sqrt(np.maximum(csq / np.maximum(nsee, 1)[:, None] - (csum / np.maximum(nsee, 1)[:, None]) ** 2, 0)).mean(1)
    z1 = np.load(OUT / f"f1{tag}_{name}.npz"); oseg = z1["odd_seg"][idx_roi]; ocam = z1["odd_cam"][idx_roi]; eseg = z1["even_seg"][idx_roi]
    agr = ag[idx_roi]; a_ = ar[idx_roi]
    groups = dict(agree0=agr == 0, agree1=agr == 1, supported_ge2=agr >= 2)
    res = dict(made=time.strftime("%F %T"), n_recordings=1, mesh=str(MESHES[name]["ply"]), votes=str(MESHES[name]["votes"]), roi=dict(x=ROI[:2], z=ROI[2:]),
               images=str(imgdir), cams=str(POSES["ba2"] / "run/dump_ba/cams.npz"), f1=str(OUT / f"f1{tag}_{name}.npz"),
               noise=dict(method="Immerkaer Laplacian estimate on grey 0-255 JPEG keyframes (every 5th), median", sigma_n=round(sig_n, 3),
                          thr_multipliers=thr_mult, thresholds_grey=[round(x, 3) for x in thrs], main_threshold_grey=round(thrs[1], 3)),
               rule=dict(seeing_keyframe="face centre in frame (4 px border), z > 0.1, |cos| > 0.15, not occluded by the mesh itself (z-buffer + max(2 cm, 1 %))",
                         textureless_view="9x9 luminance std at the projected pixel < threshold", textureless_face="textureless share >= 0.5 of seeing keyframes (>= 1 seeing)",
                         colour_std="per-face std over seeing keyframes of the 3x3-mean BGR at the projected pixel, mean over channels (faces with >= 2 views)",
                         odd_pierced="odd-track segments through the face >= 3 (also reported with >= 2 cams)"),
               roi_faces=int(n), groups={})
    for gname, gm in groups.items():
        A = max(a_[gm].sum(), 1e-9); w2v = gm & (nsee >= 2)
        dct = dict(area_m2=round(float(a_[gm].sum()), 3), faces=int(gm.sum()), seen_ge1_area_pct=round(float(a_[gm & (nsee >= 1)].sum() / A * 100), 1),
                   textureless_share_areaweighted_mean=[round(float((share[q][gm] * a_[gm]).sum() / A), 3) for q in range(len(thrs))],
                   textureless_face_area_pct=[round(float(a_[gm & (nsee >= 1) & (share[q] >= 0.5)].sum() / A * 100), 1) for q in range(len(thrs))],
                   odd_seg_ge3_area_pct=round(float(a_[gm & (oseg >= 3)].sum() / A * 100), 2), odd_pierced_ge3_2cams_area_pct=round(float(a_[gm & (oseg >= 3) & (ocam >= 2)].sum() / A * 100), 2),
                   odd_seg_ge1_area_pct=round(float(a_[gm & (oseg >= 1)].sum() / A * 100), 2), even_seg_ge3_area_pct=round(float(a_[gm & (eseg >= 3)].sum() / A * 100), 2),
                   colour_std_p25_50_75=[round(float(x), 2) for x in np.percentile(cstd[w2v], [25, 50, 75])] if w2v.any() else None)
        res["groups"][gname] = dct
    # go / no-go: caught = (textureless AND agree == 0) OR (odd-pierced >= 3)
    unsup = groups["agree0"]; sup = groups["supported_ge2"]; go = {}
    for q in range(len(thrs)):
        tl = (nsee >= 1) & (share[q] >= 0.5)
        for pr, pm in (("odd_seg_ge3", oseg >= 3), ("odd_seg_ge3_2cams", (oseg >= 3) & (ocam >= 2))):
            caught = (tl & (agr == 0)) | pm
            cu = float(a_[caught & unsup].sum() / max(a_[unsup].sum(), 1e-9) * 100); cs = float(a_[caught & sup].sum() / max(a_[sup].sum(), 1e-9) * 100)
            cu1 = float(a_[caught & (agr < 2)].sum() / max(a_[agr < 2].sum(), 1e-9) * 100)
            go[f"thr{thr_mult[q]}x/{pr}"] = dict(unsupported_agree0_caught_pct=round(cu, 1), unsupported_agree_lt2_caught_pct=round(cu1, 1),
                                                 supported_caught_pct=round(cs, 2), textureless_only_agree0_pct=round(float(a_[tl & unsup].sum() / max(a_[unsup].sum(), 1e-9) * 100), 1),
                                                 pierce_only_agree0_pct=round(float(a_[pm & unsup].sum() / max(a_[unsup].sum(), 1e-9) * 100), 1),
                                                 go=bool(cu >= 40 and cs < 1.0))
    res["go_probe"] = go; main = go[f"thr{thr_mult[1]}x/odd_seg_ge3"]
    ph = {}
    for q in range(len(thrs)):
        tl = (nsee >= 1) & (share[q] >= 0.5); c_ = (nsee == 0) | (tl & (agr == 0)) | (oseg >= 3)
        ph[f"thr{thr_mult[q]}x/unseen_or_rule"] = dict(agree0_caught_pct=round(float(a_[c_ & unsup].sum() / a_[unsup].sum() * 100), 1),
                                                     supported_caught_pct=round(float(a_[c_ & sup].sum() / a_[sup].sum() * 100), 2))
    res["post_hoc_not_the_rule"] = dict(note="exploratory, NOT the pre-registered rule: adds faces seen by no keyframe (nsee == 0); supported faces are 0 % unseen by "
                                             "construction (support needs visible views), so the supported side of this term is not a test",
                                        agree0_unseen_area_pct=round(float(a_[unsup & (nsee == 0)].sum() / a_[unsup].sum() * 100), 1), cells=ph)
    res["decision"] = dict(main_cell=f"thr{thr_mult[1]}x/odd_seg_ge3", go=main["go"], rule="GO if >= 40 % of aisle agree==0 area caught and < 1 % of supported (agree>=2) area caught")
    res["runtime_s"] = round(time.time() - t0, 1)
    np.savez_compressed(OUT / f"probe{tag}_faces.npz", idx_roi=idx_roi, nsee=nsee, ntl=ntl, cstd=cstd.astype(np.float32))
    (OUT / f"probe{tag}.json").write_text(json.dumps(res, indent=1)); log(f"END probe go={main['go']} {json.dumps(main)}")


def coverage(a):
    """why F1 is weak: per face, how many BA rays (all kept tracks, extended to infinity) reach it at all; through = before the point
    (minus shortening), near = within the shortening of the point, behind = beyond the point (face hidden behind the observed surface)"""
    import open3d as o3d
    T = tracks(a); cam, pt, Xm, Cm = T["cam"], T["pt"], T["Xm"], T["Cm"]; name = "ffsonly2_carve"
    V, F, cen, ar, _ = load_mesh(MESHES[name]["ply"]); sc = scene(V, F); ag, _ = votes_for(name, len(F)); R = roi_mask(cen); nf = len(F)
    Cs = Cm[cam]; d = Xm[pt] - Cs; L = np.linalg.norm(d, axis=1); d /= L[:, None]; sh = np.maximum(0.05, 0.03 * L)
    thr = np.zeros(nf); near = np.zeros(nf); beh = np.zeros(nf)
    for i in range(0, len(L), 250000):
        li = sc.list_intersections(o3d.core.Tensor(np.concatenate([Cs[i:i + 250000], d[i:i + 250000]], 1).astype(np.float32)), nthreads=NT)
        sp = li["ray_splits"].numpy(); th = li["t_hit"].numpy(); pid = li["primitive_ids"].numpy().astype(np.int64); rid = i + np.repeat(np.arange(len(sp) - 1), np.diff(sp))
        rr = L[rid] - th; np.add.at(thr, pid[rr > sh[rid]], 1); np.add.at(near, pid[np.abs(rr) <= sh[rid]], 1); np.add.at(beh, pid[rr < -sh[rid]], 1)
    out = dict(mesh=str(MESHES[name]["ply"]), tracks="even + odd kept tracks", groups={})
    for g, gm in (("agree0", ag == 0), ("agree1", ag == 1), ("supported_ge2", ag >= 2)):
        for reg, rm in (("roi", R), ("whole", np.ones(nf, bool))):
            s = gm & rm; A = max(ar[s].sum(), 1e-9); f = lambda x: round(float(ar[s & x].sum() / A * 100), 1)  # noqa: E731
            out["groups"][f"{g}/{reg}"] = dict(area_m2=round(float(A), 2), any_ray_pct=f((thr + near + beh) > 0), through_ge1_pct=f(thr >= 1), through_ge3_pct=f(thr >= 3),
                                               near_point_ge1_pct=f(near >= 1), behind_point_ge1_pct=f(beh >= 1), behind_point_ge3_pct=f(beh >= 3))
    return out


def cmd_assemble(a):
    f1 = json.loads((OUT / "f1.json").read_text()); sa = json.loads((OUT / "sanity.json").read_text()); f23 = json.loads((OUT / "f2f3.json").read_text())
    ruler = dict(made=time.strftime("%F %T"), n_recordings=1, script="scan3d/gsmap/gsm_f_ruler.py", inputs=f1["inputs"], frame_check=f1["frame_check"], filter=f1["filter"],
                 f1_rule=f1["rule"], f2_rule=f23["f2_rule"], f3_rule=f23["f3_rule"], view_frames=f23["view_frames"],
                 sanity=dict(test=sa["test"], all_pass=sa["all_pass"], meshes=sa["meshes"]),
                 voxel_variant_rejected="pierce counts pooled per 5 / 10 cm voxel: supported faces 6.4 / 19.4 % pierced in the aisle (fails the < 1 % sanity), agree0 10.1 / 24.0 % -> not used",
                 ray_coverage=coverage(a))
    (OUT / "ruler.json").write_text(json.dumps(ruler, indent=1))
    tab = {}
    for name in MESHES:
        o = f23["meshes"][name]; r = dict(frame=o["frame"], area_m2=o["area_m2"], area_roi_m2=o["area_roi_m2"], unsupported_agree_lt2_m2=o["unsupported_area_m2"],
                                          unsupported_agree_lt2_roi_m2=o["unsupported_area_roi_m2"])
        if name in f1["meshes"]:
            e = f1["meshes"][name]["even"]
            r.update(F1_blocked_frac=e["blocked_frac"], F1_pierced_m2=e["pierced_area_m2"], F1_pierced_roi_m2=e["pierced_area_roi_m2"], F1_hit_m2=e["hit_area_m2"], F1_hit_roi_m2=e["hit_area_roi_m2"],
                     F1_point_to_mesh_cm_med=f1["meshes"][name]["even_point_to_mesh_cm_p25_50_75"][1])
        else:
            r["F1"] = "not measured: mesh is in the bap frame, BA2 tracks are not (F2 + F3 only)"
        r.update(F2_tube_m2=o["f2_tube_area_m2"], F2_tube_roi_m2=o["f2_tube_area_roi_m2"], F3_audit_red_pct=o["views"]["audit"]["red_pct_of_hit"],
                 F3_audit_red_px=o["views"]["audit"]["red_px"], F3_red_px_9views=o["red_px_total_9views"], F3_tex_red_px_audit=o["tex_red_px_audit"],
                 F3_red_pct_per_view={k: v["red_pct_of_hit"] for k, v in o["views"].items()}, sheet=o["sheet"], ply=str(MESHES[name]["ply"]), votes=str(MESHES[name]["votes"]))
        tab[name] = r
    (OUT / "baselines.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), n_recordings=1, sources=[str(OUT / "f1.json"), str(OUT / "f2f3.json")],
                                                        compare_sheet=f23["compare_sheet"], table=tab), indent=1))
    log("assemble ruler.json baselines.json written")


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("cmd", choices=["f1", "sanity", "f2f3", "probe", "assemble"])
    ap.add_argument("--reproj", type=float, default=1.0); ap.add_argument("--min-obs", type=int, default=3); ap.add_argument("--min-tri", type=float, default=2.0)
    ap.add_argument("--min-kf", type=int, default=0, help="require >= this many keyframe observations per point"); ap.add_argument("--tag", default="")
    a = ap.parse_args(); OUT.mkdir(parents=True, exist_ok=True)
    dict(f1=cmd_f1, sanity=cmd_sanity, f2f3=cmd_f2f3, probe=cmd_probe, assemble=cmd_assemble)[a.cmd](a)


if __name__ == "__main__":
    main()
