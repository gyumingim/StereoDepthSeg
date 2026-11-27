#!/usr/bin/env python3
"""Night 11-27 item 1 W-fill: flat-region 2D depth fill per keyframe (no AI depth). Gate: GATE.md (written before running).

  python wfill.py pretest OUT [--seed 0]   CPU: H1/H2/H3 hide tests, BA check, G4 cross-keyframe, G5 ROI b2, floaters -> OUT/pretest.json
  python wfill.py run OUT                  CPU: all keyframes -> OUT/fused/KF.npz (Z, F, S; F = 2 filled, FFS untouched) + OUT/run.json

Fill rule (per keyframe): Felzenszwalb segments of the undistorted photo; per segment a plane fitted to its FFS pixels (F == 1) in the
camera frame; F == 0 pixels of the segment get the plane depth only if all segment gates pass (see P below and GATE.md).
"""
import argparse, json, sys, time
from pathlib import Path

import cv2
import numpy as np

HOME = Path.home()
GSM = HOME / "scanner/StereoDepthSeg-gsmap/scan3d/gsmap"
V2 = HOME / "scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2"
sys.path.insert(0, str(GSM)); sys.path.insert(0, str(V2 / "diag_plane_20261127/scripts"))
import gsm_dc as DC  # noqa: E402  (Ctx, hide_blocks, err only; no model import)

DUMP = DC.DUMP; FUSED = DC.FUSED; SPARSE = DC.SPARSE
MESH = V2 / "ffsonly_ba2/b/e3c.ply"; E3C = V2 / "ffsonly_ba2/b/e3c.json"
P = dict(scale=300, sigma=0.8, min_size=400, min_area=1500, min_sup=300, min_sup_frac=0.05, inl_frac=0.90, rms_abs=0.006, rms_rel=0.004,
         min_spread_px=10, max_inc_deg=75, hull_dil=15, zmin=0.05, zmax=4.0, zlo=0.85, zhi=1.15, ransac_it=120)
H1_IDX = [50, 150, 250, 400, 470, 314, 318, 322, 326, 330]
ROI_NAMES = ["cabinet_front_kf774", "wall_kf538", "floor_kf3205"]
STAGE_E = {"cabinet_front_kf774": 2.2367705280852577, "wall_kf538": 1.8295128640336122, "floor_kf3205": 0.43731740542358005}   # plane_diag.json e
STAGE_B_OWN = {"cabinet_front_kf774": 0.7524384018203124, "wall_kf538": 0.28540742180713596, "floor_kf3205": 0.12858767552193848}


def rays_img(Kr, W, H):
    vv, uu = np.mgrid[0:H, 0:W].astype(np.float64)
    return np.dstack([(uu - Kr[0, 2]) / Kr[0, 0], (vv - Kr[1, 2]) / Kr[1, 1], np.ones((H, W))])


def huber_plane(X, c, iters=4):
    w = np.ones(len(X))
    for _ in range(iters):
        mu = (X * w[:, None]).sum(0) / w.sum(); Y = (X - mu) * np.sqrt(w)[:, None]
        n = np.linalg.svd(Y, full_matrices=False)[2][2]; d = -n @ mu
        r = np.abs(X @ n + d); w = np.where(r <= c, 1.0, c / np.maximum(r, 1e-9))
    return n, d


def ransac(X, thr, it, rng):
    S = X if len(X) <= 3000 else X[rng.choice(len(X), 3000, replace=False)]
    idx = rng.integers(0, len(S), (it, 3)); A, B, C = S[idx[:, 0]], S[idx[:, 1]], S[idx[:, 2]]
    N = np.cross(B - A, C - A); nn = np.linalg.norm(N, axis=1); ok = nn > 1e-12
    N, A = N[ok] / nn[ok, None], A[ok]; D = -(N * A).sum(1)
    cnt = (np.abs(S @ N.T + D) < thr).sum(0); b = int(np.argmax(cnt)); n, d = N[b], D[b]
    m = np.abs(X @ n + d) < thr
    return huber_plane(X[m], thr) if m.sum() >= 3 else (n, d)


def fill(rgb, Z, valid, vmask, R, rng, seg=None):
    """returns Zf (nan where not filled), seg labels, per-segment stats counts."""
    from scipy import ndimage
    from skimage.segmentation import felzenszwalb
    H, W = Z.shape
    if seg is None:
        seg = felzenszwalb(rgb, scale=P["scale"], sigma=P["sigma"], min_size=P["min_size"])
    Zf = np.full((H, W), np.nan); st = dict(seg=0, tried=0, filled_seg=0, fail={})
    cmax = np.cos(np.radians(P["max_inc_deg"])); ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * P["hull_dil"] + 1,) * 2)
    for l, sl in enumerate(ndimage.find_objects(seg + 1)):
        if sl is None:
            continue
        m = seg[sl] == l; A = int(m.sum()); st["seg"] += 1
        if A < P["min_area"]:
            continue
        v = valid[sl]; sup = m & v; ns = int(sup.sum()); hole = m & ~v & vmask[sl]
        if not hole.any():
            continue
        st["tried"] += 1
        why = None
        if ns < P["min_sup"] or ns < P["min_sup_frac"] * A:
            why = "support"
        else:
            yy, xx = np.nonzero(sup); Rr = R[sl]; X = Rr[sup] * Z[sl][sup][:, None]; zmed = float(np.median(X[:, 2]))
            thr = max(P["rms_abs"], P["rms_rel"] * zmed)
            n, d = ransac(X, thr, P["ransac_it"], rng); r = X @ n + d; inl = np.abs(r) < thr
            ev = np.linalg.eigvalsh(np.cov(np.c_[xx, yy].T.astype(np.float64)))
            if inl.mean() < P["inl_frac"]:
                why = "inliers"
            elif np.sqrt(np.mean(r[inl] ** 2)) > thr:
                why = "rms"
            elif np.sqrt(max(ev[0], 0)) < P["min_spread_px"]:
                why = "spread"
            else:
                hm = np.zeros(m.shape, np.uint8); hull = cv2.convexHull(np.c_[xx, yy].astype(np.int32))
                cv2.fillConvexPoly(hm, hull, 1); hm = cv2.dilate(hm, ker).astype(bool)
                cand = hole & hm
                if not cand.any():
                    why = "outside_hull"
                else:
                    rc = Rr[cand]; den = rc @ n
                    with np.errstate(divide="ignore", invalid="ignore"):
                        t = -d / den
                    cosang = np.abs(den) / np.linalg.norm(rc, axis=1)
                    zs = X[inl, 2]
                    ok = np.isfinite(t) & (t > P["zmin"]) & (t < P["zmax"]) & (cosang >= cmax) & (t >= P["zlo"] * zs.min()) & (t <= P["zhi"] * zs.max())
                    if not ok.any():
                        why = "z_or_incidence"
                    else:
                        blk = Zf[sl]; cy, cx = np.nonzero(cand); blk[cy[ok], cx[ok]] = t[ok]; st["filled_seg"] += 1
        if why:
            st["fail"][why] = st["fail"].get(why, 0) + 1
    return Zf, seg, st


class Ctx(DC.Ctx):
    def __init__(s):
        super().__init__(); c = np.load(DUMP / "cams.npz"); s.T = c["T"].astype(np.float64); s.Kr = c["Kr"].astype(np.float64)
        s.R = rays_img(s.Kr, s.W, s.H); s.n = len(s.kf)


def cm_abs(p, g):
    a = np.abs(np.asarray(p) - np.asarray(g)) * 100
    return dict(n=int(len(a)), med_cm=float(np.median(a)) if len(a) else None, p90_cm=float(np.percentile(a, 90)) if len(a) else None)


def hide_test(cx, j, hid_fn, rng, seg_cache):
    i = int(cx.kf[j]); rgb, Z, F, S = cx.load(i); valid = (F == 1) & np.isfinite(Z) & (Z > 0)
    hid = hid_fn(j, valid); keep = valid & ~hid
    Zf, seg, st = fill(rgb, Z, keep, cx.vmask, cx.R, rng, seg_cache.get(j))
    seg_cache[j] = seg
    got = hid & np.isfinite(Zf)
    dist = cv2.distanceTransform((~keep).astype(np.uint8), cv2.DIST_L2, 5)
    bins = {}
    for lo, hi in ((0, 12), (12, 40), (40, 100), (100, 1e9)):
        b = got & (dist >= lo) & (dist < hi); hb = hid & (dist >= lo) & (dist < hi)
        bins[f"{lo}-{int(hi) if hi < 1e8 else 'inf'}px"] = dict(hidden=int(hb.sum()), filled=int(b.sum()), **DC.err(Zf[b], Z[b]))
    return dict(idx=j, kf=i, hidden=int(hid.sum()), filled=int(got.sum()), recall=float(got.sum() / max(hid.sum(), 1)),
                err=DC.err(Zf[got], Z[got]), cm=cm_abs(Zf[got], Z[got]), bins=bins, seg=st), (Zf[got], Z[got])


def cmd_pretest(a):
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True); fj = out / f"pretest_s{a.seed}.json"
    assert not fj.exists(), "never overwrite"
    import open3d as o3d
    import plane_diag as PD
    t0 = time.time(); cx = Ctx(); rng = np.random.default_rng(a.seed); seg_cache = {}
    rois = {r["name"]: r for r in json.loads(PD.ROIS.read_text())}
    res = dict(made=time.strftime("%F %T"), script=str(Path(__file__).resolve()), gate=str(out.parent / "GATE.md" if (out.parent / "GATE.md").exists() else out / "GATE.md"),
               params=P, seed=a.seed, inputs=dict(dump=str(DUMP), fused=str(FUSED), sparse=str(SPARSE), mesh=str(MESH), rois=str(PD.ROIS)), n_recordings=1)
    pools = {"H1": [], "H2": [], "H3": []}; per = {"H1": [], "H2": [], "H3": []}
    # H1: dc-probe protocol
    for j in H1_IDX:
        r, pr = hide_test(cx, j, lambda j_, v: DC.hide_blocks(v, 0.2, 24, rng), rng, seg_cache); per["H1"].append(r); pools["H1"].append(pr)
        print("H1", json.dumps({k: r[k] for k in ("kf", "hidden", "filled", "recall", "err")}), flush=True)
    # H2: ROI rects
    h2cm = {}
    for nm in ROI_NAMES:
        R_ = rois[nm]; j = R_["idx"]; x0, x1, y0, y1 = R_["rect"]
        rm = np.zeros((cx.H, cx.W), bool); rm[y0:y1, x0:x1] = True
        r, pr = hide_test(cx, j, lambda j_, v: DC.hide_blocks(v & rm, 0.25, 24, rng), rng, seg_cache); r["roi"] = nm; per["H2"].append(r); pools["H2"].append(pr)
        h2cm[nm] = dict(med_cm=r["cm"]["med_cm"], limit_cm=max(0.3, 1.5 * STAGE_B_OWN[nm]), recall=r["recall"])
        print("H2", nm, json.dumps({k: r[k] for k in ("kf", "hidden", "filled", "recall", "err", "cm")}), flush=True)
    # H3: transplanted real hole masks
    for j in H1_IDX + [rois[nm]["idx"] for nm in ROI_NAMES]:
        jo = (j + 37) % cx.n; Fo = np.where(cx.vmask, np.load(FUSED / f"{int(cx.kf[jo]):05d}.npz")["F"], 1)
        r, pr = hide_test(cx, j, lambda j_, v: v & (Fo == 0), rng, seg_cache); r["mask_from_idx"] = jo; per["H3"].append(r); pools["H3"].append(pr)
        print("H3", json.dumps({k: r[k] for k in ("kf", "hidden", "filled", "recall", "err")}), flush=True)
    pooled = {}
    for k in pools:
        p_ = np.concatenate([x[0] for x in pools[k]]); g_ = np.concatenate([x[1] for x in pools[k]]); pooled[k] = DC.err(p_, g_)
    pa = np.concatenate([x[0] for k in pools for x in pools[k]]); ga = np.concatenate([x[1] for k in pools for x in pools[k]])
    pooled["H123"] = DC.err(pa, ga)
    h1_recall = float(sum(r["filled"] for r in per["H1"]) / max(sum(r["hidden"] for r in per["H1"]), 1))
    res.update(hide=per, pooled=pooled, h1_recall=h1_recall, h2_cm=h2cm)
    # full-FFS fills on the sample set (every 8th kf) + ROI footprint kfs
    RS = PD.roi_setup(); kfi, T, Kr, (W, H) = PD.cams(); rcp = PD.rays(Kr, W, H)
    for name, R in RS.items():   # global plane refit as in plane_diag measure (2 rounds on stage b)
        fps = {ci: f for ci in range(len(kfi)) if (f := PD.footprint(R, T[ci], rcp, W, H)).sum() >= PD.MIN_FP}; R["fps"] = fps
        for _ in range(2):
            Pb = []
            for ci, fp in fps.items():
                d = np.load(FUSED / f"{int(kfi[ci]):05d}.npz"); Zb = np.where(d["F"] == 1, d["Z"], np.nan).astype(np.float64)
                mm = fp & np.isfinite(Zb) & (Zb > 0.05); X = rcp[mm.reshape(-1)] * Zb[mm][:, None] @ T[ci][:3, :3].T + T[ci][:3, 3]
                X = X[np.abs(X @ R["n"] + R["d"]) <= PD.GATE]
                if len(X) >= PD.MIN_PTS:
                    Pb.append(X[np.random.default_rng(0).choice(len(X), min(len(X), 5000), replace=False)])
            n, d = PD.fit_plane(np.concatenate(Pb)); n, d = (-n, -d) if n @ R["n"] < 0 else (n, d); R["n"], R["d"] = n, d
    sample = sorted(set(range(0, cx.n, 8)) | {ci for R in RS.values() for ci in R["fps"]})
    m = o3d.io.read_triangle_mesh(str(MESH)); fin = json.loads(E3C.read_text())["faces_in"]
    sc = o3d.t.geometry.RaycastingScene(); sc.add_triangles(o3d.core.Tensor(np.asarray(m.vertices, np.float32)), o3d.core.Tensor(np.asarray(m.triangles, np.uint32)))
    full = {}; ba_p, ba_g = [], []; fl = dict(n=0, front3=0, behind3=0, nohit=0); yields = []
    b2 = {nm: dict(res=[], gated=[0, 0], hole_px=0, filled_in_hole=0, b_res=[]) for nm in RS}
    for c, j in enumerate(sample):
        i = int(cx.kf[j]); rgb, Z, F, S = cx.load(i); valid = (F == 1) & np.isfinite(Z) & (Z > 0)
        Zf, _, st = fill(rgb, Z, valid, cx.vmask, cx.R, rng, seg_cache.get(j)); got = np.isfinite(Zf); full[j] = Zf.astype(np.float32)
        yields.append(float(got.sum() / cx.vmask.sum()))
        sp = SPARSE / f"{i:05d}.npz"
        if sp.exists() and j % 8 == 0:
            Zb = np.load(sp)["Z"]; bm = np.isfinite(Zb) & (Zb > 0) & got; ba_p.append(Zf[bm]); ba_g.append(Zb[bm])
        if j % 8 == 0 and got.any():   # floaters vs ffsonly2 mesh (measured faces only)
            yy, xx = np.nonzero(got); D = cx.R[yy, xx] @ T[j][:3, :3].T; o = np.broadcast_to(T[j][:3, 3], D.shape)
            hit = sc.cast_rays(o3d.core.Tensor(np.concatenate([o, D], 1).astype(np.float32)))
            th = hit["t_hit"].numpy().astype(np.float64); pid = hit["primitive_ids"].numpy().astype(np.int64)
            okh = np.isfinite(th) & (pid < fin); dz = (th - Zf[yy, xx]) * np.linalg.norm(cx.R[yy, xx], axis=1)
            fl["n"] += int(okh.sum()); fl["front3"] += int((okh & (dz > 0.03)).sum()); fl["behind3"] += int((okh & (dz < -0.03)).sum()); fl["nohit"] += int((~okh).sum())
        for nm, R in RS.items():
            if j not in R["fps"]:
                continue
            fp = R["fps"][j]; hole = fp & ~valid & cx.vmask; b2[nm]["hole_px"] += int(hole.sum()); b2[nm]["filled_in_hole"] += int((hole & got).sum())
            for key, ZZ in (("res", Zf), ("b_res", np.where(valid, Z, np.nan))):
                mm = fp & np.isfinite(ZZ) & (ZZ > 0.05)
                X = rcp[mm.reshape(-1)] * ZZ[mm][:, None] @ T[j][:3, :3].T + T[j][:3, 3]; rg = X @ R["n"] + R["d"]; keep = np.abs(rg) <= PD.GATE
                if key == "res":
                    b2[nm]["gated"][0] += int(len(rg)); b2[nm]["gated"][1] += int(keep.sum())
                b2[nm][key].append(rg[keep])
        if c % 20 == 0:
            print(f"full {c}/{len(sample)} kf {i} yield {yields[-1]:.3f} {st['fail']}", flush=True)
    bap = np.concatenate(ba_p) if ba_p else np.zeros(0); bag = np.concatenate(ba_g) if ba_g else np.zeros(0)
    res["ba_empty"] = DC.err(bap, bag)
    res["floaters"] = dict(fl, front3_share=fl["front3"] / max(fl["n"], 1), behind3_share=fl["behind3"] / max(fl["n"], 1))
    res["yield_sample_mean"] = float(np.mean(yields)); res["n_sample"] = len(sample)
    g5 = {}
    for nm, v in b2.items():
        r_ = np.concatenate(v["res"]) if v["res"] else np.zeros(0); rb = np.concatenate(v["b_res"]) if v["b_res"] else np.zeros(0)
        g5[nm] = dict(n_b2=int(len(r_)), b2_rms_cm=float(np.sqrt(np.mean(r_ ** 2)) * 100) if len(r_) else None,
                      b2_beyond3cm_share=float((np.abs(r_) > 0.03).mean()) if len(r_) else None,
                      b2_kept_frac_gate=v["gated"][1] / max(v["gated"][0], 1), b_rms_cm_check=float(np.sqrt(np.mean(rb ** 2)) * 100) if len(rb) else None,
                      hole_px=v["hole_px"], filled_in_hole=v["filled_in_hole"], hole_yield=v["filled_in_hole"] / max(v["hole_px"], 1),
                      stage_e_ffsonly2_cm=STAGE_E[nm], n_kf=len(RS[nm]["fps"]))
    res["g5_roi_b2"] = g5
    # G4 cross-keyframe: F == 2 px of the every-8th sample vs F == 1 of other keyframes
    print("G4 load fused", flush=True)
    ZF1 = np.zeros((cx.n, cx.H, cx.W), np.float16)
    for j in range(cx.n):
        d = np.load(FUSED / f"{int(cx.kf[j]):05d}.npz"); ZF1[j] = np.where((d["F"] == 1) & cx.vmask, d["Z"], 0)
    Kr = cx.Kr; acc = []; occl = 0; tot = 0
    for j in [s for s in sample if s % 8 == 0]:
        Zf = full[j]; yy, xx = np.nonzero(np.isfinite(Zf))
        if not len(yy):
            continue
        if len(yy) > 20000:
            s_ = rng.choice(len(yy), 20000, replace=False); yy, xx = yy[s_], xx[s_]
        Xw = cx.R[yy, xx] * Zf[yy, xx][:, None].astype(np.float64) @ T[j][:3, :3].T + T[j][:3, 3]
        for b in range(cx.n):
            if b == j:
                continue
            dB = np.linalg.norm(T[b][:3, 3] - T[j][:3, 3]); ang = np.degrees(np.arccos(np.clip((np.trace(T[b][:3, :3].T @ T[j][:3, :3]) - 1) / 2, -1, 1)))
            if dB < 0.20 and ang < 15:
                continue
            Xc = (Xw - T[b][:3, 3]) @ T[b][:3, :3]; z = Xc[:, 2]; ok = z > 0.05
            u = np.round(Kr[0, 0] * Xc[:, 0] / np.where(ok, z, 1) + Kr[0, 2]).astype(int); v = np.round(Kr[1, 1] * Xc[:, 1] / np.where(ok, z, 1) + Kr[1, 2]).astype(int)
            ok &= (u >= 0) & (u < cx.W) & (v >= 0) & (v < cx.H)
            if not ok.any():
                continue
            zb = ZF1[b][v[ok], u[ok]].astype(np.float64); zz = z[ok]; has = zb > 0
            rel = (zz[has] - zb[has]) / zb[has]; tot += len(rel); oc = rel > 0.05; occl += int(oc.sum()); acc.append(rel[~oc])
    rel = np.concatenate(acc) if acc else np.zeros(0)
    res["g4_cross"] = dict(n_judged=int(len(rel)), n_total=int(tot), occluded_or_behind_share=occl / max(tot, 1),
                           absrel_med_pct=float(np.median(np.abs(rel)) * 100) if len(rel) else None,
                           front_gt5_share_of_total=float((rel < -0.05).sum() / max(tot, 1)),
                           signed_med_pct=float(np.median(rel) * 100) if len(rel) else None,
                           note="rel = (fill depth in B - FFS depth in B) / FFS; rel > +5 % = behind B's surface (occluded, not judged); rel < -5 % = fill in front of a surface B sees = wrong")
    g = {}
    ph = pooled["H123"]
    g["G1_pooled"] = ph["absrel_med_pct"] <= 1.0 and ph["p90_pct"] <= 3.0 and ph["gt5_pct"] <= 3.0
    g["G1_roi_cm"] = all(v["med_cm"] is not None and v["med_cm"] <= v["limit_cm"] for v in h2cm.values())
    be = res["ba_empty"]; g["G2_ba"] = be.get("n", 0) >= 300 and be["absrel_med_pct"] <= 2.0 and be["gt5_pct"] <= 5.0
    g["G3_recall"] = h1_recall >= 0.30; g["G3_yield_sample"] = res["yield_sample_mean"] >= 0.03
    c4 = res["g4_cross"]; g["G4_cross"] = c4["n_judged"] >= 3000 and c4["absrel_med_pct"] <= 1.5 and c4["front_gt5_share_of_total"] <= 0.05
    g["G5_b2"] = all(v["b2_rms_cm"] is None or v["b2_rms_cm"] <= v["stage_e_ffsonly2_cm"] for v in g5.values()) and \
        any((g5[nm]["filled_in_hole"] > 0) for nm in ("cabinet_front_kf774", "wall_kf538"))
    g["floaters"] = res["floaters"]["front3_share"] <= 0.05
    res["gates"] = g; res["GO"] = all(g.values()); res["runtime_s"] = round(time.time() - t0, 1)
    fj.write_text(json.dumps(res, indent=1, default=float))
    print(json.dumps(dict(pooled=pooled, h1_recall=h1_recall, h2_cm=h2cm, ba=be, g4=c4, g5=g5, floaters=res["floaters"], yield_=res["yield_sample_mean"], gates=g, GO=res["GO"]), indent=1, default=float))


def cmd_run(a):
    out = Path(a.out); fo = out / "fused"; assert not (out / "run.json").exists(), "never overwrite"; fo.mkdir(parents=True, exist_ok=True)
    t0 = time.time(); cx = Ctx(); rng = np.random.default_rng(a.seed); ys = []; fails = {}
    for j in range(cx.n):
        i = int(cx.kf[j]); q = fo / f"{i:05d}.npz"
        d = np.load(FUSED / f"{i:05d}.npz"); Z0, F0, S0 = d["Z"], d["F"], d["S"]
        if q.exists():   # resumable
            ys.append(float((np.load(q)["F"] == 2).sum() / cx.vmask.sum())); continue
        rgb, Z, F, S = cx.load(i); valid = (F == 1) & np.isfinite(Z) & (Z > 0)
        Zf, _, st = fill(rgb, Z, valid, cx.vmask, cx.R, rng)
        got = np.isfinite(Zf) & (F0 == 0)
        Fn = F0.copy(); Fn[got] = 2; Zn = Z0.copy(); Zn[got] = Zf[got]
        np.savez_compressed(q, Z=Zn.astype(Z0.dtype), F=Fn, S=S0)
        ys.append(float(got.sum() / cx.vmask.sum()))
        for k, v in st["fail"].items():
            fails[k] = fails.get(k, 0) + v
        if j % 25 == 0:
            print(f"{j}/{cx.n} kf {i} fill {ys[-1]:.3f} {time.time() - t0:.0f}s", flush=True)
    info = dict(made=time.strftime("%F %T"), script=str(Path(__file__).resolve()), src_fused=str(FUSED), params=P, seed=a.seed,
                note="F == 1 FFS (unchanged copy), F == 2 = W-fill planar segment fill (no AI), S unchanged (only F == 1 S is used by gsm_train)",
                n=cx.n, fill_px_share_mean=float(np.mean(ys)), fill_px_share_median=float(np.median(ys)), seg_fail_counts=fails, runtime_s=round(time.time() - t0, 1))
    (fo / "info.json").write_text(json.dumps(info, indent=1)); (out / "run.json").write_text(json.dumps(info, indent=1)); print(json.dumps(info, indent=1))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("cmd"); ap.add_argument("out"); ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args(); {"pretest": cmd_pretest, "run": cmd_run}[a.cmd](a)
