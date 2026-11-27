#!/usr/bin/env python3
"""Per-stage plane residual diagnostic (2026-11-27), map ffsonly2 (ffsonly_ba2), poses/cams ba_partners2.

usage:  python plane_diag.py render   (GPU: 2DGS median depth at the keyframes that see the ROIs -> OUT/gs_depth.npz)
        python plane_diag.py measure  (CPU: stages a..e, json + png)
Stages per ROI (same footprint pixels in every stage = rays that hit the global ROI plane inside the ROI hull):
  a  single-pair FFS depth (pair stack = gsm_ffs_fix3.pairs_of inputs as in gsm_w7_probe: ba_partners/ffs_pairs2/pairs,
     ba_partners/ffs_pairs3/extra tag low, ffs_relax/src/wide; fuse cut sigma/Z <= 3 %, Z <= 4 m)
  b  per-keyframe fused FFS (ffsonly_ba2/fused, F == 1) = the training depth
  c  all b points pooled in the world frame
  d  2DGS median depth (rasterization_2dgs, alpha > 0.5) of ffsonly_ba2/b/gs/gaussians.pt at the keyframe poses
  e  final mesh ffsonly_ba2/b/e3c.ply ray-cast (first hit; generated e3c faces (id >= faces_in) excluded)
  f, g = d, e restricted to the pixels that have fused FFS (F == 1) in the same keyframe
Point gate: |distance to global plane| <= GATE. Global plane = robust fit to pooled stage-b points.
"""
import json, sys, time
from pathlib import Path

import numpy as np

HOME = Path.home()
GSM = HOME / "scanner/StereoDepthSeg-gsmap/scan3d/gsmap"
sys.path.insert(0, str(GSM)); sys.path.insert(0, str(GSM.parent)); sys.path.insert(0, str(GSM.parent / "gsplat"))
V2 = HOME / "scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2"
DUMP = V2 / "ba_partners2/run/dump_ba"; FUSED = V2 / "ffsonly_ba2/fused"; GS = V2 / "ffsonly_ba2/b/gs/gaussians.pt"
MESH = V2 / "ffsonly_ba2/b/e3c.ply"; E3C = V2 / "ffsonly_ba2/b/e3c.json"
BAP = V2 / "ba_partners"; P2D = BAP / "ffs_pairs2"; P3D = BAP / "ffs_pairs3"; SRC = V2 / "ffs_relax/src"
ROIS = V2 / "eval2/ref/rois.json"; OUT = V2 / "diag_plane_20261127"
USE = ["cabinet_front_kf774", "wall_kf538", "floor_kf3205"]
GATE, MIN_FP, MIN_PTS, CAP, ZMAX = 0.10, 2000, 800, 0.03, 4.0


def cams():
    c = np.load(DUMP / "cams.npz"); return c["kf"], c["T"].astype(np.float64), c["Kr"].astype(np.float64), [int(v) for v in c["size"]]


def rays(Kr, W, H):
    vv, uu = np.mgrid[0:H, 0:W]
    return np.stack([(uu - Kr[0, 2]) / Kr[0, 0], (vv - Kr[1, 2]) / Kr[1, 1], np.ones((H, W))], -1).reshape(-1, 3)


def fit_plane(X, iters=4, c=0.01):
    """Huber IRLS plane. returns n (unit), d (n.x + d = 0)."""
    w = np.ones(len(X))
    for _ in range(iters):
        mu = (X * w[:, None]).sum(0) / w.sum(); Y = (X - mu) * np.sqrt(w)[:, None]
        n = np.linalg.svd(Y, full_matrices=False)[2][2]; d = -n @ mu
        r = np.abs(X @ n + d); w = np.where(r <= c, 1.0, c / np.maximum(r, 1e-9))
    return n, d


def ransac_plane(X, thr=0.015, it=300, seed=0):
    rng = np.random.default_rng(seed); best = None
    for _ in range(it):
        s = X[rng.choice(len(X), 3, replace=False)]; n = np.cross(s[1] - s[0], s[2] - s[0]); nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn; d = -n @ s[0]; k = (np.abs(X @ n + d) < thr).sum()
        if best is None or k > best[0]:
            best = (k, n, d)
    m = np.abs(X @ best[1] + best[2]) < thr
    return fit_plane(X[m])


def basis(n):
    a = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
    e1 = np.cross(n, a); e1 /= np.linalg.norm(e1); e2 = np.cross(n, e1); return e1, e2


def hull2d(P):
    from scipy.spatial import ConvexHull
    h = ConvexHull(P); return P[h.vertices]


def in_hull(Q, H):
    """convex polygon H (ccw from scipy) contains points Q."""
    ok = np.ones(len(Q), bool)
    for i in range(len(H)):
        a, b = H[i], H[(i + 1) % len(H)]
        ok &= ((b[0] - a[0]) * (Q[:, 1] - a[1]) - (b[1] - a[1]) * (Q[:, 0] - a[0])) >= 0
    return ok


def roi_setup():
    kf, T, Kr, (W, H) = cams(); rc = rays(Kr, W, H); rois = {r["name"]: r for r in json.loads(ROIS.read_text())}
    out = {}
    for name in USE:
        r = rois[name]; ci = r["idx"]; x0, x1, y0, y1 = r["rect"]
        d = np.load(FUSED / f"{int(kf[ci]):05d}.npz"); Z = np.where(d["F"] == 1, d["Z"], np.nan)
        m = np.zeros((H, W), bool); m[y0:y1, x0:x1] = True; m &= np.isfinite(Z) & (Z > 0)
        Xc = rc[m.reshape(-1)] * Z[m][:, None]; Xw = Xc @ T[ci][:3, :3].T + T[ci][:3, 3]
        n, dd = ransac_plane(Xw); inl = np.abs(Xw @ n + dd) < 0.03
        e1, e2 = basis(n); P = np.c_[Xw[inl] @ e1, Xw[inl] @ e2]
        out[name] = dict(idx=ci, kf=int(kf[ci]), rect=r["rect"], n=n, d=dd, e1=e1, e2=e2, hull=hull2d(P), fn=n.copy(), fd=dd, fe1=e1.copy(), fe2=e2.copy(), n_ref=int(inl.sum()), n_ref_all=int(len(Xw)))
    return out


def footprint(R, Tc, rc, W, H):
    """pixels whose ray hits the ROI plane inside the hull (t > 0, incidence < 80 deg)."""
    n, d = R["fn"], R["fd"]; o = Tc[:3, 3]; D = rc @ Tc[:3, :3].T
    den = D @ n
    with np.errstate(divide="ignore", invalid="ignore"):
        t = -(n @ o + d) / den
    cosang = np.abs(den) / np.linalg.norm(D, axis=1)
    X = o + D * t[:, None]
    ok = (t > 0.05) & (t < 6) & (cosang > np.cos(np.radians(80)))
    ok &= in_hull(np.c_[X @ R["fe1"], X @ R["fe2"]], R["hull"])
    return ok.reshape(H, W)


# ------------------------------------------------------------------------------------------- render (GPU)
def cmd_render():
    import torch
    from gsplat import rasterization_2dgs
    kf, T, Kr, (W, H) = cams(); rc = rays(Kr, W, H); RS = roi_setup()
    sel = sorted({ci for R in RS.values() for ci in range(len(kf)) if footprint(R, T[ci], rc, W, H).sum() >= MIN_FP})
    ck = torch.load(GS, weights_only=False); p = {k: v.cuda() for k, v in ck["params"].items()}
    assert np.allclose(ck["T"], T, atol=1e-6), "gaussians.pt cams != ba_partners2 cams"
    K = torch.from_numpy(ck["Kr"]).float().cuda(); Zs = []
    t0 = time.time()
    with torch.no_grad():
        for ci in sel:
            vm = torch.from_numpy(np.linalg.inv(T[ci])).float().cuda()[None]
            r = rasterization_2dgs(p["means"], torch.nn.functional.normalize(p["quats"], dim=-1), torch.exp(p["scales"]),
                                   torch.sigmoid(p["opacities"]), torch.cat([p["sh0"], p["shN"]], 1), vm, K[None], W, H,
                                   sh_degree=ck["sh_degree"], render_mode="RGB+ED", near_plane=0.02, far_plane=20.0)
            A = r[1][0, ..., 0].cpu().numpy(); Z = r[5][0, ..., 0].float().cpu().numpy()
            Zs.append(np.where((A > 0.5) & np.isfinite(Z) & (Z > 0.05), Z, 0).astype(np.float32))
    OUT.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(OUT / "gs_depth.npz", Z=np.stack(Zs).astype(np.float16), idx=np.array(sel), made=time.strftime("%F %T"),
                        src=str(GS), note="2DGS median depth r[5], alpha > 0.5 (= gsm_mesh2d rule)")
    print(f"rendered {len(sel)} views in {time.time() - t0:.1f}s, n_gauss {len(p['means'])}")


# ------------------------------------------------------------------------------------------- measure (CPU)
def pair_stack(kf, ci, Kfull, base_c, W, H):
    import gsm_ffs_4m as M
    fi = int(kf[ci]); L = []
    fb = P2D / "pairs" / f"{fi:05d}.npz"
    if fb.exists():
        d = np.load(fb); Z = d["Z"].astype(np.float32); cmap = {int(r[0]): r[1:] for r in base_c.get(str(fi), [])}
        for p in range(len(d["B"])):
            L.append((Z[p], float(d["f"][p]), float(d["B"][p]), M.sin_map(Kfull, cmap[int(d["partner"][p])], W, H), "base", int(d["partner"][p])))
    for tag, fe in (("low", P3D / "extra" / f"{fi:05d}.npz"), ("wide", SRC / "wide" / f"{fi:05d}.npz")):
        if fe.exists():
            d = np.load(fe); Z = d["Z"].astype(np.float32); sn = d["sin"].astype(np.float32)
            for p in range(len(d["B"])):
                if tag == "low" and str(d["tag"][p]) != "low":
                    continue
                L.append((Z[p], float(d["f"][p]), float(d["B"][p]), sn[p], tag, int(d["partner"][p])))
    out = []
    for Z, f, B, sn, tag, j in L:
        sn = np.maximum(sn, 1e-3)
        with np.errstate(invalid="ignore"):
            bad = ~((Z / (f * B * sn)) <= CAP) | (Z > ZMAX) | (Z < 0.05)
        out.append(dict(Z=np.where(bad, np.nan, Z), B=B, tag=tag, partner=j))
    return out


def bands(u, v, r, cell=0.01):
    """1 cm grid of mean residual; RMS of long (> 20 cm box), mid (5-20), short (< 5 cm) parts. plus 1D profile along u."""
    from scipy.ndimage import uniform_filter
    if len(r) < 50:
        return None
    iu = np.floor((u - u.min()) / cell).astype(int); iv = np.floor((v - v.min()) / cell).astype(int)
    S = np.zeros((iu.max() + 1, iv.max() + 1)); C = np.zeros_like(S)
    np.add.at(S, (iu, iv), r); np.add.at(C, (iu, iv), 1)
    m = C > 0; Mn = np.where(m, S / np.maximum(C, 1), 0)

    def sm(w):
        k = max(1, int(round(w / cell)))
        a = uniform_filter(Mn * m, k, mode="constant"); b = uniform_filter(m.astype(float), k, mode="constant")
        return np.where(b > 1e-6, a / np.maximum(b, 1e-6), 0)
    s5, s20 = sm(0.05), sm(0.20)
    rms = lambda x: float(np.sqrt(np.mean(x[m] ** 2)) * 100)
    return dict(cells=int(m.sum()), cell_rms_cm=rms(Mn), long_gt20_cm=rms(s20), mid_5_20_cm=rms(s5 - s20), short_lt5_cm=rms(Mn - s5))


def map_stats(Xw, R, n_min=MIN_PTS):
    """own-plane residual, offset (at the map's footprint centroid, along global normal) and tilt vs global plane."""
    ng, dg = R["n"], R["d"]
    if len(Xw) < n_min:
        return None
    sub = Xw if len(Xw) < 40000 else Xw[np.random.default_rng(0).choice(len(Xw), 40000, replace=False)]
    n, d = fit_plane(sub)
    if n @ ng < 0:
        n, d = -n, -d
    r_own = Xw @ n + d; r_g = Xw @ ng + dg
    e1, e2 = basis(ng); Q = np.c_[sub @ e1, sub @ e2]; ext = np.sqrt(np.linalg.eigvalsh(np.cov(Q.T))) * 100 * 2   # 2 sd widths
    cxy = Xw.mean(0); c0 = cxy - (cxy @ ng + dg) * ng          # footprint centroid projected onto the global plane
    s = -(n @ c0 + d) / (n @ ng)
    mad = lambda x: float(1.4826 * np.median(np.abs(x - np.median(x))) * 100)
    return dict(n=int(len(Xw)), rms_own_cm=float(np.sqrt(np.mean(r_own ** 2)) * 100), mad_own_cm=mad(r_own),
                rms_glob_cm=float(np.sqrt(np.mean(r_g ** 2)) * 100), offset_cm=float(s * 100),
                tilt_deg=float(np.degrees(np.arccos(np.clip(n @ ng, -1, 1)))), r_own=r_own,
                extent_minor_cm=float(ext[0]), extent_major_cm=float(ext[1]), mean_depth_m=None)


def cmd_measure():
    import open3d as o3d
    import gsm_ba_partners as BP
    t0 = time.time()
    kf, T, Kr, (W, H) = cams(); rc = rays(Kr, W, H); RS = roi_setup()
    g = np.load(Path(BP.R1) / "rec" / "lensonly" / "gt.npz"); Kfull = g["K"].copy(); Kfull[:2] *= W / 1920.0
    base_c = json.loads((SRC / "plan_4m.json").read_text())["base_c"]
    G = np.load(OUT / "gs_depth.npz"); gidx = {int(c): i for i, c in enumerate(G["idx"])}; GZ = G["Z"]
    m = o3d.io.read_triangle_mesh(str(MESH)); Vm = np.asarray(m.vertices, np.float32); Fm = np.asarray(m.triangles, np.uint32)
    fin = json.loads(E3C.read_text())["faces_in"]; assert len(Fm) == fin + json.loads(E3C.read_text())["generated_faces"], (len(Fm), fin)
    sc = o3d.t.geometry.RaycastingScene(); sc.add_triangles(o3d.core.Tensor(Vm), o3d.core.Tensor(Fm))
    res = dict(made=time.strftime("%F %T"), script=str(Path(__file__)), gate_m=GATE, min_footprint_px=MIN_FP, min_pts=MIN_PTS,
               inputs=dict(cams=str(DUMP / "cams.npz"), fused=str(FUSED), gs=str(GS), gs_depth=str(OUT / "gs_depth.npz"), gs_depth_made=str(G["made"]),
                           mesh=str(MESH), pairs=[str(P2D / "pairs"), str(P3D / "extra (tag low)"), str(SRC / "wide")], rois=str(ROIS)),
               note=__doc__, rois={})
    rng = np.random.default_rng(0)
    for name, R in RS.items():
        print(name, "setup", R["kf"], R["n_ref"], flush=True)
        fps = {ci: footprint(R, T[ci], rc, W, H) for ci in range(len(kf))}
        fps = {ci: f for ci, f in fps.items() if f.sum() >= MIN_FP}

        def world(ci, Z, fp):
            mm = fp & np.isfinite(Z) & (Z > 0.05)
            X = rc[mm.reshape(-1)] * Z[mm][:, None] @ T[ci][:3, :3].T + T[ci][:3, 3]
            return X, mm
        # pass 1: stage b -> refit global plane
        for it in range(2):
            Pb = []
            for ci, fp in fps.items():
                d = np.load(FUSED / f"{int(kf[ci]):05d}.npz"); Z = np.where(d["F"] == 1, d["Z"], np.nan).astype(np.float64)
                X, _ = world(ci, Z, fp); X = X[np.abs(X @ R["n"] + R["d"]) <= GATE]
                if len(X) >= MIN_PTS:
                    Pb.append(X[rng.choice(len(X), min(len(X), 5000), replace=False)])
            Pb = np.concatenate(Pb); n, d = fit_plane(Pb)
            if n @ R["n"] < 0:
                n, d = -n, -d
            R["n"], R["d"] = n, d
            R["e1"], R["e2"] = basis(n)
        stages = {k: [] for k in "abdefg"}; pooled = {k: [] for k in "abdefg"}; gated = {k: [0, 0] for k in "abdefg"}
        for ci, fp in fps.items():
            fi = int(kf[ci]); Tc = T[ci]
            maps = []
            for q in pair_stack(kf, ci, Kfull, base_c, W, H):
                maps.append(("a", q["Z"].astype(np.float64), dict(partner=q["partner"], tag=q["tag"], B_cm=round(q["B"] * 100, 1))))
            d = np.load(FUSED / f"{fi:05d}.npz"); maps.append(("b", np.where(d["F"] == 1, d["Z"], np.nan).astype(np.float64), {}))
            if ci in gidx:
                Zg = GZ[gidx[ci]].astype(np.float64); maps.append(("d", np.where(Zg > 0, Zg, np.nan), {}))
            # mesh
            D = rc[fp.reshape(-1)] @ Tc[:3, :3].T; o = np.broadcast_to(Tc[:3, 3], D.shape)
            hit = sc.cast_rays(o3d.core.Tensor(np.concatenate([o, D], 1).astype(np.float32)))
            tt = hit["t_hit"].numpy().astype(np.float64); fid = hit["primitive_ids"].numpy().astype(np.int64)
            tt[(~np.isfinite(tt)) | (fid >= fin)] = np.nan
            Ze = np.full(H * W, np.nan); Ze[fp.reshape(-1)] = tt; maps.append(("e", Ze.reshape(H, W), dict(generated_px=int(((fid >= fin) & (fid < len(Fm)) & np.isfinite(hit["t_hit"].numpy())).sum()))))
            Ffs = d["F"] == 1   # f / g = d / e only on pixels with fused FFS in this keyframe (like-for-like with b)
            maps += [("f" if s_ == "d" else "g", np.where(Ffs, Z_, np.nan), {}) for s_, Z_, _ in list(maps) if s_ in "de"]
            for st, Z, info in maps:
                X, _ = world(ci, Z, fp); rg = X @ R["n"] + R["d"]; keep = np.abs(rg) <= GATE
                gated[st][0] += int(len(X)); gated[st][1] += int(keep.sum()); X = X[keep]
                s = map_stats(X, R)
                if s is None:
                    continue
                r_own = s.pop("r_own")
                u, v = X @ R["e1"], X @ R["e2"]
                s["bands_own"] = bands(u, v, r_own)
                s.update(idx=int(ci), kf=fi, footprint_px=int(fp.sum()), **info)
                s["mean_depth_m"] = float(np.median(np.linalg.norm(X - Tc[:3, 3], axis=1)))
                stages[st].append(s)
                k = min(len(X), 20000); sel = rng.choice(len(X), k, replace=False)
                pooled[st].append(np.c_[u[sel], v[sel], (X[sel] @ R["n"] + R["d"]), np.full(k, ci)])
        rr = dict(kf=R["kf"], idx=R["idx"], rect=R["rect"], plane_n=R["n"].tolist(), plane_d=float(R["d"]), n_ref_pts=R["n_ref"],
                  hull_area_m2=float(0.5 * abs(np.sum(R["hull"][:, 0] * np.roll(R["hull"][:, 1], -1) - np.roll(R["hull"][:, 0], -1) * R["hull"][:, 1]))),
                  n_kf_footprint=len(fps), stages={})
        for st in "abdefg":
            L = stages[st]
            if not L:
                continue
            off = np.array([s["offset_cm"] for s in L]); tl = np.array([s["tilt_deg"] for s in L])
            P = np.concatenate(pooled[st]); rg = P[:, 2]
            summ = dict(n_maps=len(L), n_kf=len({s["idx"] for s in L}), kept_frac_gate=gated[st][1] / max(gated[st][0], 1),
                        within_rms_own_cm_median=float(np.median([s["rms_own_cm"] for s in L])),
                        within_mad_own_cm_median=float(np.median([s["mad_own_cm"] for s in L])),
                        offset_sd_cm=float(off.std()), offset_robust_sd_cm=float(1.4826 * np.median(np.abs(off - np.median(off)))),
                        offset_median_cm=float(np.median(off)), offset_p5_p95_cm=[float(np.percentile(off, 5)), float(np.percentile(off, 95))],
                        tilt_median_deg=float(np.median(tl)), tilt_p90_deg=float(np.percentile(tl, 90)),
                        tilt_median_deg_wide=float(np.median([s["tilt_deg"] for s in L if s["extent_minor_cm"] >= 15])) if any(s["extent_minor_cm"] >= 15 for s in L) else None,
                        n_wide=int(sum(s["extent_minor_cm"] >= 15 for s in L)),
                        offset_sd_cm_wide=float(np.std([s["offset_cm"] for s in L if s["extent_minor_cm"] >= 15])) if any(s["extent_minor_cm"] >= 15 for s in L) else None,
                        median_n_pts=float(np.median([s["n"] for s in L])), median_extent_minor_cm=float(np.median([s["extent_minor_cm"] for s in L])),
                        median_depth_m=float(np.median([s["mean_depth_m"] for s in L])),
                        combined_rms_global_cm=float(np.sqrt(np.mean(rg ** 2)) * 100),
                        combined_mad_global_cm=float(1.4826 * np.median(np.abs(rg - np.median(rg))) * 100),
                        bands_combined=bands(P[:, 0], P[:, 1], rg),
                        bands_own_median={k: float(np.median([s["bands_own"][k] for s in L if s["bands_own"]])) for k in ("long_gt20_cm", "mid_5_20_cm", "short_lt5_cm")})
            if st == "a":   # partner-to-partner spread inside one keyframe vs across keyframes
                by = {}
                for s in L:
                    by.setdefault(s["idx"], []).append(s["offset_cm"])
                w = [np.std(v) for v in by.values() if len(v) >= 2]
                kfm = [np.mean(v) for v in by.values()]
                summ.update(within_kf_partner_offset_sd_cm_mean=float(np.mean(w)) if w else None, n_kf_multi_pair=len(w),
                            across_kf_mean_offset_sd_cm=float(np.std(kfm)))
                summ["by_tag"] = {t: dict(n=int(sum(s["tag"] == t for s in L)), rms_own_med=float(np.median([s["rms_own_cm"] for s in L if s["tag"] == t])),
                                          off_sd=float(np.std([s["offset_cm"] for s in L if s["tag"] == t])))
                                  for t in sorted({s["tag"] for s in L})}
            rr["stages"][st] = dict(summary=summ, maps=L)
            np.save(OUT / f"pooled_{name}_{st}.npy", P.astype(np.float32))
        # stage c = pooled b
        if "b" in rr["stages"]:
            rr["stages"]["c"] = dict(summary=dict(note="= pooled stage-b points vs the global plane", **{k: rr["stages"]["b"]["summary"][k] for k in
                                                                                                        ("combined_rms_global_cm", "combined_mad_global_cm", "bands_combined")}))
        res["rois"][name] = rr
        print(name, json.dumps({st: {k: v for k, v in rr["stages"][st]["summary"].items() if k in ("n_maps", "within_rms_own_cm_median", "offset_sd_cm", "offset_robust_sd_cm", "tilt_median_deg", "combined_rms_global_cm")} for st in rr["stages"]}), flush=True)
    res["runtime_s"] = round(time.time() - t0, 1)
    (OUT / "plane_diag.json").write_text(json.dumps(res, indent=1, default=float))
    print("done", res["runtime_s"])


if __name__ == "__main__":
    {"render": cmd_render, "measure": cmd_measure}[sys.argv[1]]()
