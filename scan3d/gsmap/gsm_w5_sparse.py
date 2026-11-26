#!/usr/bin/env python3
"""gsmap W5 2026-11-27: per-keyframe sparse depth targets from the BA2 tracks (geometric, no AI), for gsm_train.py --sparse-depth.

  python scan3d/gsmap/gsm_w5_sparse.py OUTDIR [--reproj 1.0 --min-obs 3 --min-tri 2.0 --half 1 --sigma-px-min 0.5]

Tracks: gsm_f_ruler.py loader (ba_partners2/ba/ba_state.npz moved to the map frame of poses_all.npz by the exact Sim3, observations
of src == ba frames only) and its filter (>= 3 obs, max reprojection < 1 px, widest ray-pair angle > 2 deg). ALL kept points are used
(odd and even ids) -> the F1 BA-ray ruler is NOT independent of a model trained on these targets.
Per keyframe observation (cams.npz of the ba2 dump, T == poses_all T for keyframes): z = camera-frame depth of the map point,
pixel = projection with Kr (checked against the BA uv). Sigma (default --sigma-mode ffs15) = 1.5 x the median FFS sigma (fused S)
at the same depth, from the BA/FFS overlap pixels in 0.25 m bins. Reason: the triangulation sigma (--sigma-mode geo:
z * (max(emax, sigma-px-min) / f) / sin(tri) * sqrt(2 / n_obs), median 0.44 cm) ignores pose error and is 3-5x too small to explain
the observed BA-vs-FFS spread (0.71 % median at 1-2 m) if the FFS sigma is right; the 1.5 x FFS rule matches that spread at 1-2 m. Splat to a (2*half+1)^2 patch, only on pixels where the fused FFS
depth is invalid (F == 0) and inside the undistortion valid mask (same 7x7-eroded mask as gsm_train.py). Several points on one
pixel: nearest kept, pixel dropped when their depths differ by > 5 %.
Output: OUTDIR/<kf:05d>.npz (Z float32 NaN elsewhere, S float32 inf elsewhere) + OUTDIR/sparse.json (stats + sanity check).
Sanity: at observation centre pixels where FFS exists, median |z_BA - z_FFS| / z_FFS must be < 2 % (else exit 3).
"""
import argparse, json, sys, time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gsm_f_ruler as fr  # noqa: E402

V2 = fr.V2
DUMP = V2 / "ba_partners2/run/dump_ba"
FUSED = V2 / "ffsonly_ba2/fused"


def tracks_full(a):
    """Same computation as gsm_f_ruler.tracks, also returning per-point n_obs, max reproj, tri angle."""
    st = np.load(fr.POSES["ba2"] / "ba/ba_state.npz"); P = np.load(fr.POSES["ba2"] / "poses_all.npz")
    R, t, X, cam, pt, uv, K = st["R1"], st["t1"], st["X1"], st["cam"], st["pt"], st["uv"].astype(np.float64), st["K"]
    Xc = np.einsum("nij,nj->ni", R[cam], X[pt]) + t[cam]
    e = np.linalg.norm(np.stack([K[0, 0] * Xc[:, 0] / Xc[:, 2] + K[0, 2], K[1, 1] * Xc[:, 1] / Xc[:, 2] + K[1, 2]], 1) - uv, axis=1)
    isba = P["src"] == "ba"
    Cn = -np.einsum("nji,nj->ni", R, t); Cm = P["T"][:, :3, 3]
    s, Ra, ta = fr.umeyama(Cn[isba], Cm[isba])
    Xm = s * X @ Ra.T + ta
    sel = isba[cam]
    cam, pt, e, uv = cam[sel], pt[sel], e[sel], uv[sel]; npt = len(X)
    nob = np.bincount(pt, minlength=npt); emax = np.zeros(npt); np.maximum.at(emax, pt, e)
    C = Cm[cam]; d = C - Xm[pt]; d /= np.linalg.norm(d, axis=1, keepdims=True)
    mdir = np.zeros((npt, 3)); np.add.at(mdir, pt, d); mdir /= np.linalg.norm(mdir, axis=1, keepdims=True) + 1e-12
    cosm = (d * mdir[pt]).sum(1); far = np.full(npt, 2.0); np.minimum.at(far, pt, cosm)
    isfar = cosm <= far[pt] + 1e-12; dfar = np.zeros((npt, 3)); dfar[pt[isfar]] = d[isfar]
    cmin = np.full(npt, 2.0); np.minimum.at(cmin, pt, (d * dfar[pt]).sum(1)); tri = np.degrees(np.arccos(np.clip(cmin, -1, 1)))
    keep = (nob >= a.min_obs) & (emax < a.reproj) & (tri > a.min_tri)
    m = keep[pt]
    return dict(cam=cam[m], pt=pt[m], uv=uv[m], Xm=Xm, nob=nob, emax=emax, tri=tri, keep=keep, frames=P["frames"], iskf=P["iskf"], K=K,
                sim3_scale=float(s))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out"); ap.add_argument("--reproj", type=float, default=1.0); ap.add_argument("--min-obs", type=int, default=3)
    ap.add_argument("--min-tri", type=float, default=2.0); ap.add_argument("--half", type=int, default=1)
    ap.add_argument("--sigma-px-min", type=float, default=0.5); ap.add_argument("--min-kf", type=int, default=0)
    ap.add_argument("--sigma-mode", default="ffs15", choices=["geo", "ffs15"],
                    help="geo = triangulation sigma; ffs15 = 1.5 x median FFS sigma at the same depth (0.25 m bins of the BA/FFS overlap set)")
    a = ap.parse_args(); out = Path(a.out)
    assert not (out / "sparse.json").exists(), f"never overwrite {out}"
    out.mkdir(parents=True, exist_ok=True); t0 = time.time()
    tr = tracks_full(a)
    ref = fr.tracks(a)   # the W4 loader itself: kept observation set must be identical
    same = bool(len(ref["pt"]) == len(tr["pt"]) and (ref["pt"] == tr["pt"]).all() and (ref["cam"] == tr["cam"]).all())
    chk = dict(ref["chk"], same_as_f_ruler_loader=same); assert same, "loader mismatch vs gsm_f_ruler.tracks"
    cams = np.load(DUMP / "cams.npz"); kf = cams["kf"]; T = cams["T"]; Kr = cams["Kr"].astype(np.float64); W, H = [int(v) for v in cams["size"]]
    assert np.abs(Kr - tr["K"]).max() < 1e-6, "BA K != dump Kr"
    mx, my = cv2.initUndistortRectifyMap(cams["K"], cams["dist"], None, Kr, (W, H), cv2.CV_32FC1)
    vmask = cv2.erode(((mx >= 0) & (mx <= W - 1) & (my >= 0) & (my <= H - 1)).astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)
    fidx = {int(f): i for i, f in enumerate(tr["frames"])}; kidx = {int(k): j for j, k in enumerate(kf)}
    fr_of_cam = tr["frames"][tr["cam"]]
    okkf = np.array([int(f) in kidx for f in fr_of_cam]) & tr["iskf"][tr["cam"]]
    cam, pt, uvo = tr["cam"][okkf], tr["pt"][okkf], tr["uv"][okkf]
    j_of = np.array([kidx[int(f)] for f in tr["frames"][cam]], np.int64)
    Rcw = np.transpose(T[:, :3, :3], (0, 2, 1)); tcw = -np.einsum("nij,nj->ni", Rcw, T[:, :3, 3])
    Xc = np.einsum("nij,nj->ni", Rcw[j_of], tr["Xm"][pt]) + tcw[j_of]
    z = Xc[:, 2]; u = Kr[0, 0] * Xc[:, 0] / z + Kr[0, 2]; v = Kr[1, 1] * Xc[:, 1] / z + Kr[1, 2]
    dpx = np.hypot(u - uvo[:, 0], v - uvo[:, 1])
    chk["kf_obs_proj_vs_ba_uv_px"] = dict(med=float(np.median(dpx)), p99=float(np.percentile(dpx, 99)), max=float(dpx.max()))
    f = Kr[0, 0]; tri = np.radians(tr["tri"][pt]); sp = np.maximum(tr["emax"][pt], a.sigma_px_min)
    sig = z * (sp / f) / np.sin(tri) * np.sqrt(2.0 / tr["nob"][pt])
    ui, vi = np.round(u).astype(int), np.round(v).astype(int)
    inb = (z > 0.05) & (ui >= 0) & (ui < W) & (vi >= 0) & (vi < H)
    # ---- pass 1: sanity vs FFS (observation centre pixels with FFS)
    rel_all, zf_all, zb_all, sb_all, sf_all = [], [], [], [], []
    FZ = {}
    for j in range(len(kf)):
        m = inb & (j_of == j)
        d = np.load(FUSED / f"{int(kf[j]):05d}.npz"); Fz = np.where(vmask, d["F"], 0); Zf = d["Z"]; Sf = d["S"]; FZ[j] = Fz
        uj, vj, zj, sj = ui[m], vi[m], z[m], sig[m]
        isf = Fz[vj, uj] == 1
        if isf.any():
            zf = Zf[vj[isf], uj[isf]].astype(np.float64)
            rel_all.append((zj[isf] - zf) / zf); zf_all.append(zf); zb_all.append(zj[isf]); sb_all.append(sj[isf])
            sf_all.append(Sf[vj[isf], uj[isf]].astype(np.float64))
    zf_c = np.concatenate(zf_all); sf_c = np.concatenate(sf_all); okc = np.isfinite(sf_c) & (sf_c > 0)
    edges = np.arange(0, 8.01, 0.25); bc, bm = [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        mm = okc & (zf_c >= lo) & (zf_c < hi)
        if mm.sum() >= 50:
            bc.append(float(np.median(zf_c[mm]))); bm.append(float(np.median(sf_c[mm])))
    bc, bm = np.array(bc), np.array(bm)

    def ffs_sigma_at(zz):   # median FFS sigma at depth zz (interp); beyond the last bin: quadratic stereo growth; below the first: clamp
        s_ = np.interp(zz, bc, bm); hi_ = zz > bc[-1]; s_[hi_] = bm[-1] * (zz[hi_] / bc[-1]) ** 2; return s_
    sig_geo = sig
    if a.sigma_mode == "ffs15":
        sig = 1.5 * ffs_sigma_at(z)
    sigma_table = dict(depth_m=bc.round(3).tolist(), ffs_sigma_med_cm=(bm * 100).round(3).tolist())
    # ---- pass 2: splat per keyframe
    per = {}; tot_px = 0; tot_drop = 0
    for j in range(len(kf)):
        m = inb & (j_of == j); Fz = FZ[j]
        uj, vj, zj, sj = ui[m], vi[m], z[m], sig[m]
        isf = Fz[vj, uj] == 1
        # splat
        pu, pv, pz, ps = [], [], [], []
        for dv in range(-a.half, a.half + 1):
            for du in range(-a.half, a.half + 1):
                uu, vv = uj + du, vj + dv; ok = (uu >= 0) & (uu < W) & (vv >= 0) & (vv < H)
                pu.append(uu[ok]); pv.append(vv[ok]); pz.append(zj[ok]); ps.append(sj[ok])
        pu, pv, pz, ps = map(np.concatenate, (pu, pv, pz, ps))
        free = (Fz[pv, pu] == 0) & vmask[pv, pu]
        pu, pv, pz, ps = pu[free], pv[free], pz[free], ps[free]
        Z = np.full((H, W), np.nan, np.float32); S = np.full((H, W), np.inf, np.float32)
        n_px = 0; n_drop = 0
        if len(pz):
            lin = pv * W + pu; o = np.lexsort((pz, lin)); lin, pz, ps = lin[o], pz[o], ps[o]
            first = np.r_[True, lin[1:] != lin[:-1]]; gid = np.cumsum(first) - 1
            zmin = pz[first]; zmax = np.zeros(first.sum()); np.maximum.at(zmax, gid, pz)
            good = (zmax - zmin) / zmin <= 0.05
            L = lin[first][good]; Z.ravel()[L] = zmin[good]; S.ravel()[L] = ps[first][good]
            n_px = int(good.sum()); n_drop = int((~good).sum())
        tot_px += n_px; tot_drop += n_drop
        np.savez_compressed(out / f"{int(kf[j]):05d}.npz", Z=Z, S=S)
        per[int(kf[j])] = dict(idx=j, obs=int(m.sum()), obs_on_ffs=int(isf.sum()), sparse_px=n_px, dropped_conflict_px=n_drop,
                               ffs_px=int((Fz == 1).sum()), nonffs_valid_px=int(((Fz == 0) & vmask).sum()))
    rel = np.concatenate(rel_all); zf = np.concatenate(zf_all); sb = np.concatenate(sb_all); sf = np.concatenate(sf_all)
    bins = [0, 1, 2, 3, 5, 20]; byd = {}
    for lo, hi in zip(bins[:-1], bins[1:]):
        mm = (zf >= lo) & (zf < hi)
        if mm.sum():
            byd[f"{lo}-{hi}m"] = dict(n=int(mm.sum()), med_abs_rel_pct=float(np.median(np.abs(rel[mm])) * 100), med_rel_pct=float(np.median(rel[mm]) * 100),
                                      sigma_ba_med_cm=float(np.median(sb[mm]) * 100), sigma_ffs_med_cm=float(np.median(sf[mm][np.isfinite(sf[mm])]) * 100))
    med_abs = float(np.median(np.abs(rel)) * 100)
    sall = sig[inb]; sgeo = sig_geo[inb]
    aisle_idx = list(range(314, 332))
    aisle = {int(kf[j]): per[int(kf[j])] for j in aisle_idx}
    res = dict(made=time.strftime("%F %T"), n_recordings=1,
               inputs=dict(ba_state=str(fr.POSES["ba2"] / "ba/ba_state.npz"), poses_all=str(fr.POSES["ba2"] / "poses_all.npz"),
                           cams=str(DUMP / "cams.npz"), fused=str(FUSED)),
               args=vars(a), sigma_rule=("1.5 x median FFS sigma at depth" if a.sigma_mode == "ffs15" else "z * (max(emax, sigma_px_min) / f) / sin(tri) * sqrt(2 / n_obs)"), checks=chk,
               points_kept=int(tr["keep"].sum()), kf_obs_used=int(inb.sum()),
               sigma_mode=a.sigma_mode, sigma_table_ffs=sigma_table,
               sigma_geo_cm=dict(med=float(np.median(sgeo) * 100), p10=float(np.percentile(sgeo, 10) * 100), p90=float(np.percentile(sgeo, 90) * 100)),
               sparse_depth_m=dict(med=float(np.median(z[inb])), p90=float(np.percentile(z[inb], 90))),
               sigma_used_cm=dict(med=float(np.median(sall) * 100), p10=float(np.percentile(sall, 10) * 100), p90=float(np.percentile(sall, 90) * 100)),
               sanity=dict(n_overlap=int(len(rel)), med_abs_rel_pct=med_abs, med_rel_pct=float(np.median(rel) * 100),
                           p90_abs_rel_pct=float(np.percentile(np.abs(rel), 90) * 100), by_ffs_depth=byd, pass_lt_2pct=bool(med_abs < 2.0)),
               totals=dict(sparse_px=tot_px, dropped_conflict_px=tot_drop, kf_with_sparse=int(sum(p["sparse_px"] > 0 for p in per.values())),
                           sparse_px_per_kf_med=float(np.median([p["sparse_px"] for p in per.values()])),
                           sparse_share_of_nonffs_valid_pct=float(100 * tot_px / max(1, sum(p["nonffs_valid_px"] for p in per.values())))),
               aisle_kf_idx_314_331=dict(sparse_px=int(sum(p["sparse_px"] for p in aisle.values())),
                                         nonffs_valid_px=int(sum(p["nonffs_valid_px"] for p in aisle.values())), per_kf=aisle),
               per_kf=per, runtime_s=round(time.time() - t0, 1))
    (out / "sparse.json").write_text(json.dumps(res, indent=1))
    print(json.dumps({k: res[k] for k in ("checks", "points_kept", "sigma_geo_cm", "sigma_used_cm", "sparse_depth_m", "sanity", "totals")}, indent=1))
    print("aisle", res["aisle_kf_idx_314_331"]["sparse_px"], res["aisle_kf_idx_314_331"]["nonffs_valid_px"])
    sys.exit(0 if res["sanity"]["pass_lt_2pct"] else 3)


if __name__ == "__main__":
    main()
