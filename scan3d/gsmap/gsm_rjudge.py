#!/usr/bin/env python3
"""gsmap 2026-11-26 overnight task R: fair comparison of the four 121801 maps (np, nC, ffsonly2, W1) that sit on different BA pose
solutions (bap = ba_partners, ba2 = ba_partners2, ba3 = w1/ba3).

Principle: every map is judged in its OWN pose frame (the keyframe poses it was trained / textured with) against the SAME per-camera
evidence (a keyframe depth image or a photo), which does not depend on the pose solution.

  python gsm_rjudge.py geo     CPU  first-hit depth of each mesh at all 494 keyframes (own poses, 480x270) vs fixed reference depths:
                                    gold (eval2/ref/gold_ffs.npz = OLD run FFS on VIO-anchored pairs, in nobody's training set) and, as a
                                    with/without check, the three fused training sets (F == 1).  -> geo.json + geo_perkf.npz
  python gsm_rjudge.py photo   CPU  textured-mesh render (own poses) vs the real photo: keyframes (texture sources, NOT held out) and
                                    partner frames (never used for texturing or 2DGS training = held out). -> photo.json + photo_perframe.npz
  python gsm_rjudge.py mv      GPU  (light) pose quality: multi-view consistency of the SAME depth set under bap / ba2 / ba3 keyframe poses
                                    (gsm_ffs_4m fail rule + agreement residual). -> pose_mv.json
  python gsm_rjudge.py xrep    CPU  pose quality without depth: feature tracks of BA solution A re-triangulated with the poses of
                                    solution B (DLT + 3 Gauss-Newton steps), reprojection error. -> pose_xrep.json
  python gsm_rjudge.py claims  CPU  recompute the W1 worker claims from the pose files. -> claims.json
Noise: per-keyframe bootstrap (1000 resamples of keyframes) and even / odd keyframe halves. n = 1 recording.
"""
import json, sys, time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent / "gsplat"))
V2 = Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2"
OLD = V2.parent / "gsmap_20261125_121801"
OUT = V2 / "r_judge"
POSES = dict(bap=V2 / "ba_partners", ba2=V2 / "ba_partners2", ba3=V2 / "w1/ba3")
MAPS = dict(np=(V2 / "ffsonly_ba/newpose/tex_121801_ffsonly_np.glb", "bap"),
            nC=(V2 / "ffsonly_ba/support_test/nC/tex_121801_ffssupport_NC.glb", "bap"),
            ffsonly2=(V2 / "ffsonly_ba2/b/tex_121801_ffsonly2.glb", "ba2"),
            W1=(V2 / "w1/b/tex_121801_w1.glb", "ba3"))
FUSED = dict(fused_bap=V2 / "ffsonly_ba/fused", fused_ba2=V2 / "ffsonly_ba2/fused", fused_ba3=V2 / "w1/fused")
RNG = np.random.default_rng(0)
PAIRS = (("W1", "nC"), ("ffsonly2", "nC"), ("W1", "ffsonly2"), ("nC", "np"))


def cams(p):
    c = np.load(POSES[p] / "run/dump_ba/cams.npz"); return c["kf"], c["T"], c["Kr"].copy(), [int(v) for v in c["size"]], c


def vmask(c, W, H):
    mx, my = cv2.initUndistortRectifyMap(c["K"], c["dist"], None, c["Kr"], (W, H), cv2.CV_32FC1)
    return cv2.erode(((mx >= 0) & (mx <= W - 1) & (my >= 0) & (my <= H - 1)).astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)


def scene(V, F):
    import open3d as o3d
    sc = o3d.t.geometry.RaycastingScene(nthreads=8); sc.add_triangles(o3d.core.Tensor(V.astype(np.float32)), o3d.core.Tensor(F.astype(np.uint32))); return sc


def cast(sc, Tc, d):
    import open3d as o3d
    return sc.cast_rays(o3d.core.Tensor(np.concatenate([np.broadcast_to(Tc[:3, 3], d.shape), d @ Tc[:3, :3].T], 1).astype(np.float32)), nthreads=8)


def boot(num, den, nb=1000):
    """ratio-of-sums bootstrap over keyframes: returns (value %, se %)."""
    num, den = np.asarray(num, float), np.asarray(den, float); n = len(num)
    r = RNG.integers(0, n, (nb, n)); v = 100 * num[r].sum(1) / np.maximum(1, den[r].sum(1))
    return round(float(100 * num.sum() / max(1, den.sum())), 3), round(float(v.std()), 3)


def boot_diff(numa, numb, den, nb=1000):
    numa, numb, den = np.asarray(numa, float), np.asarray(numb, float), np.asarray(den, float); n = len(den)
    r = RNG.integers(0, n, (nb, n)); v = 100 * (numa[r].sum(1) - numb[r].sum(1)) / np.maximum(1, den[r].sum(1))
    return round(float(100 * (numa.sum() - numb.sum()) / max(1, den.sum())), 3), round(float(v.std()), 3)


# ------------------------------------------------------------------ geo
COLS = ["n", "c3", "front", "hole", "miss", "c3s", "c15", "behind"]


def cmd_geo():
    from gsm_eval2 import load_mesh
    t0 = time.time(); kf, _, K, (W, H), c0 = cams("bap"); n = len(kf); w, h = W // 2, H // 2
    vm = vmask(c0, W, H)[::2, ::2]
    refs = dict(gold=np.load(V2 / "eval2/ref/gold_ffs.npz")["Z"].astype(np.float32))
    for nm, d in FUSED.items():
        Z = np.zeros((n, h, w), np.float32)
        for ci, i in enumerate(kf):
            f = d / f"{int(i):05d}.npz"
            if f.exists():
                z = np.load(f); Z[ci] = np.where((z["F"] == 1) & np.isfinite(z["Z"]), z["Z"], 0)[::2, ::2] * vm
        refs[nm] = Z
    # consensus of the three training sets: all three F == 1 and within 1 cm + 0.5 % -> depth evidence that did not change with the poses
    st = np.stack([refs["fused_bap"], refs["fused_ba2"], refs["fused_ba3"]])
    ok = (st > 0).all(0) & ((st.max(0) - st.min(0)) < 0.01 + 0.005 * st.mean(0)); refs["fused_consensus"] = np.where(ok, st.mean(0), 0).astype(np.float32); del st
    K2 = K.copy(); K2[:2] /= 2
    uu, vv = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    ray = np.stack([(uu - K2[0, 2]) / K2[0, 0], (vv - K2[1, 2]) / K2[1, 1], np.ones_like(uu)], -1).reshape(-1, 3)
    per = {}; scl = {}
    for m, (glb, p) in MAPS.items():
        V, F, _, _ = load_mesh(glb); sc = scene(V, F); _, T, _, _, _ = cams(p)
        A = {r: np.zeros((n, len(COLS))) for r in refs}; S = {r: np.full(n, np.nan) for r in refs}
        for ci in range(n):
            t = cast(sc, T[ci], ray)["t_hit"].numpy().reshape(h, w)
            for r, Z in refs.items():
                mk = Z[ci] > 0
                if not mk.any():
                    continue
                z = Z[ci][mk]; tt = t[mk]; fin = np.isfinite(tt)
                oks = fin & (np.abs(tt / z - 1) < 0.1); s = 1.0
                if oks.sum() > 200:
                    s = float(np.median(z[oks] / tt[oks])); S[r][ci] = s
                A[r][ci] = [mk.sum(), (np.abs(tt - z) <= 0.03).sum(), (tt < z - 0.03).sum(), (~(tt <= z + 0.03)).sum(), (~fin).sum(),
                            (np.abs(tt * s - z) <= 0.03).sum(), (np.abs(tt - z) <= 0.015).sum(), (fin & (tt > z + 0.03)).sum()]
        per[m] = A; scl[m] = S; print(m, "done", round(time.time() - t0), "s", flush=True)
    np.savez_compressed(OUT / "geo_perkf.npz", cols=COLS, kf=kf, **{f"{m}__{r}": per[m][r] for m in per for r in refs}, **{f"scale__{m}__{r}": scl[m][r] for m in per for r in refs})
    res = dict(made=time.strftime("%F %T"), n_recordings=1,
               inputs=dict(maps={m: dict(glb=str(g), own_poses=str(POSES[p] / "run/dump_ba/cams.npz")) for m, (g, p) in MAPS.items()},
                           refs=dict(gold=str(V2 / "eval2/ref/gold_ffs.npz"), **{k: str(v) for k, v in FUSED.items()})),
               note="first hit of each mesh ray-cast from every keyframe with the map's OWN poses, 480x270 (eval2 P3/S1 rule, all 494 keyframes instead of "
                    "every 6th). correct = |t - z| <= 3 cm; front = t < z - 3 cm; hole = no surface up to z + 3 cm (miss or behind); c3s = correct after one "
                    "per-keyframe depth scale (median z/t), removes stereo-baseline scale differences between map and reference; c15 = 1.5 cm. "
                    "se = bootstrap over keyframes (1000). gold = old-run FFS (VIO-anchored pairs, multi-view confirmed under old poses), in no map's "
                    "training set. fused_* = F==1 pixels of each training depth set (np/nC trained on fused_bap, ffsonly2 on fused_ba2, W1 on fused_ba3). "
                    "fused_consensus = pixels where all three sets agree within 1 cm + 0.5 %.",
               ref_px={r: int((Z > 0).sum()) for r, Z in refs.items()}, table={}, eval2_subset_check={}, halves={}, diffs={}, scale_fit={})
    sub = np.zeros(n, bool); sub[2::6] = True; ev = np.arange(n) % 2 == 0
    for r in refs:
        res["table"][r] = {}; res["halves"][r] = {}; res["scale_fit"][r] = {}
        for m in per:
            A = per[m][r]; e = {}
            for k_, col in (("correct3", 1), ("front", 2), ("hole", 3), ("miss", 4), ("correct3_scalefit", 5), ("correct1p5", 6), ("behind", 7)):
                v, se = boot(A[:, col], A[:, 0]); e[k_ + "_pct"] = v; e[k_ + "_se"] = se
            res["table"][r][m] = e
            res["halves"][r][m] = {k_: [round(float(100 * A[hm, col].sum() / A[hm, 0].sum()), 2) for hm in (ev, ~ev)] for k_, col in (("correct3", 1), ("front", 2), ("hole", 3), ("correct3_scalefit", 5))}
            s = scl[m][r]; s = s[np.isfinite(s)]
            res["scale_fit"][r][m] = dict(n_kf=int(len(s)), median=round(float(np.median(s)), 5), p5=round(float(np.percentile(s, 5)), 5), p95=round(float(np.percentile(s, 95)), 5),
                                          absdev_med_pct=round(float(np.median(np.abs(s - 1)) * 100), 3))
            if r == "gold":
                res["eval2_subset_check"][m] = dict(P3=round(float(100 * A[sub, 1].sum() / A[sub, 0].sum()), 2), front=round(float(100 * A[sub, 2].sum() / A[sub, 0].sum()), 2),
                                                    hole=round(float(100 * A[sub, 3].sum() / A[sub, 0].sum()), 2), n_px=int(A[sub, 0].sum()))
        res["diffs"][r] = {}
        for a_, b_ in PAIRS:
            d = {}
            for k_, col in (("correct3", 1), ("front", 2), ("hole", 3), ("correct3_scalefit", 5), ("correct1p5", 6)):
                v, se = boot_diff(per[a_][r][:, col], per[b_][r][:, col], per[a_][r][:, 0])
                d[k_] = dict(diff_pp=v, se=se, z=round(v / se, 1) if se > 0 else None,
                             halves_pp=[round(float(100 * (per[a_][r][hm, col].sum() - per[b_][r][hm, col].sum()) / per[a_][r][hm, 0].sum()), 2) for hm in (ev, ~ev)])
            res["diffs"][r][f"{a_}-{b_}"] = d
    res["runtime_s"] = round(time.time() - t0, 1)
    (OUT / "geo.json").write_text(json.dumps(res, indent=1)); print(json.dumps({k: res[k] for k in ("table", "eval2_subset_check", "diffs")}, indent=1))


# ------------------------------------------------------------------ photo
def ssim_map(a, b):
    g = lambda x: cv2.GaussianBlur(x, (11, 11), 1.5)  # noqa: E731
    ma, mb = g(a), g(b); saa, sbb, sab = g(a * a) - ma * ma, g(b * b) - mb * mb, g(a * b) - ma * mb
    return ((2 * ma * mb + 1e-4) * (2 * sab + 9e-4)) / ((ma * ma + mb * mb + 1e-4) * (saa + sbb + 9e-4))


def cmd_photo():
    from gsm_eval2 import load_mesh
    t0 = time.time(); kf, _, K, (W, H), c0 = cams("bap"); n = len(kf); w, h = W // 2, H // 2
    vm = cv2.resize(vmask(c0, W, H).astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST).astype(bool)
    imgdir = next((POSES["ba3"] / "cache/rec").iterdir())
    PA = {p: np.load(POSES[p] / "poses_all.npz") for p in POSES}
    TF = {p: {int(f): PA[p]["T"][k] for k, f in enumerate(PA[p]["frames"]) if PA[p]["src"][k] == "ba"} for p in POSES}
    kfs = set(int(x) for x in kf)
    common = sorted(set.intersection(*[set(TF[p]) for p in POSES]) - kfs)
    fr = [(int(kf[ci]), "kf") for ci in range(2, n, 4) if all(int(kf[ci]) in TF[p] for p in POSES)] + [(f, "partner") for f in common[::6]]
    uf, vf = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
    ray = np.stack([(uf - K[0, 2]) / K[0, 0], (vf - K[1, 2]) / K[1, 1], np.ones_like(uf)], -1).reshape(-1, 3)
    SC = {}
    for m, (glb, p) in MAPS.items():
        V, F, UV, tex = load_mesh(glb); SC[m] = (scene(V, F), F, UV, tex, p)
    names = list(MAPS); cols = ["hit_share", "psnr_hit", "psnr_common", "psnr_common_colourfit", "ssim_common", "common_share"]
    R = np.full((len(fr), len(names), len(cols)), np.nan)
    er = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    for fi, (f, kind) in enumerate(fr):
        gt = cv2.resize(cv2.imread(str(imgdir / f"{f:05d}.jpg")), (w, h), interpolation=cv2.INTER_AREA).astype(np.float64) / 255
        ims, hits = [], []
        for m in names:
            sc, F, UV, tex, p = SC[m]; Tc = TF[p][f]; r = cast(sc, Tc, ray)
            pid = r["primitive_ids"].numpy(); hit = pid != 4294967295; bu = r["primitive_uvs"].numpy()
            img = np.zeros((H * W, 3), np.uint8); q = pid[hit]; b1, b2 = bu[hit, 0], bu[hit, 1]; b0 = 1 - b1 - b2
            uvp = b0[:, None] * UV[F[q, 0]] + b1[:, None] * UV[F[q, 1]] + b2[:, None] * UV[F[q, 2]]; Ah, Aw = tex.shape[:2]
            img[hit] = tex[np.clip((uvp[:, 1] * Ah).astype(int), 0, Ah - 1), np.clip((uvp[:, 0] * Aw).astype(int), 0, Aw - 1)]
            ims.append(cv2.resize(img.reshape(H, W, 3), (w, h), interpolation=cv2.INTER_AREA).astype(np.float64) / 255)
            hits.append((cv2.resize(hit.reshape(H, W).astype(np.float32), (w, h), interpolation=cv2.INTER_AREA) > 0.99) & vm)
        com = np.logical_and.reduce(hits); come = cv2.erode(com.astype(np.uint8), er).astype(bool)
        gg = cv2.cvtColor(gt.astype(np.float32), cv2.COLOR_BGR2GRAY)
        for mi, m in enumerate(names):
            a_ = ims[mi]; hm = hits[mi]; R[fi, mi, 0] = hm.sum() / vm.sum(); R[fi, mi, 5] = com.sum() / vm.sum()
            if hm.sum() > 100:
                R[fi, mi, 1] = -10 * np.log10(((a_ - gt) ** 2)[hm].mean())
            if com.sum() > 1000:
                R[fi, mi, 2] = -10 * np.log10(((a_ - gt) ** 2)[com].mean())
                Xa = np.c_[a_[com], np.ones(com.sum())]; M_ = np.linalg.lstsq(Xa, gt[com], rcond=None)[0]
                R[fi, mi, 3] = -10 * np.log10(((Xa @ M_ - gt[com]) ** 2).mean())
                if come.sum() > 500:
                    R[fi, mi, 4] = float(ssim_map(cv2.cvtColor(a_.astype(np.float32), cv2.COLOR_BGR2GRAY), gg)[come].mean())
        if fi % 40 == 0:
            print("photo", fi, len(fr), round(time.time() - t0), "s", flush=True)
    kind = np.array([k for _, k in fr]); frames = np.array([f for f, _ in fr])
    np.savez_compressed(OUT / "photo_perframe.npz", R=R, maps=names, cols=cols, frames=frames, kind=kind)
    res = dict(made=time.strftime("%F %T"), n_recordings=1,
               inputs=dict(maps={m: dict(glb=str(g), own_poses=str(POSES[p] / "poses_all.npz")) for m, (g, p) in MAPS.items()}, images=str(imgdir)),
               note="textured mesh ray-cast at 960x540 (nearest texel, eval2 P4 rule) from the map's OWN pose of the frame, area-downsampled to 480x270, vs the "
                    "undistorted photo. kf = every 4th keyframe (texture sources: NOT held out, tests self-consistency). partner = non-keyframe BA frames "
                    "present in all three pose solutions, every 6th (never used for texture or 2DGS training = held out; they sit a few cm from keyframes). "
                    "common = pixels hit by all four maps (same pixel set for every map). colourfit = per-image 3x3 + bias. SSIM = grey, 11 px gaussian, "
                    "on the eroded common mask. se = standard error over frames of the paired per-frame difference.",
               n_frames={k: int((kind == k).sum()) for k in ("kf", "partner")}, table={}, diffs={}, halves={})
    for k in ("kf", "partner"):
        sel = kind == k; res["table"][k] = {}; res["diffs"][k] = {}; res["halves"][k] = {}
        for mi, m in enumerate(names):
            res["table"][k][m] = {c: round(float(np.nanmean(R[sel, mi, ci])), 4) for ci, c in enumerate(cols)}
            x = R[sel, mi, 2]; res["halves"][k][m] = dict(psnr_common=[round(float(np.nanmean(x[0::2])), 3), round(float(np.nanmean(x[1::2])), 3)])
        for a_, b_ in PAIRS:
            d = {}
            for ci, c in enumerate(cols[:5]):
                x = R[sel, names.index(a_), ci] - R[sel, names.index(b_), ci]; x = x[np.isfinite(x)]
                se = float(x.std(ddof=1) / np.sqrt(len(x)))
                d[c] = dict(diff=round(float(x.mean()), 4), se=round(se, 4), z=round(float(x.mean() / se), 1) if se > 0 else None,
                            halves=[round(float(x[0::2].mean()), 4), round(float(x[1::2].mean()), 4)], n=int(len(x)))
            res["diffs"][k][f"{a_}-{b_}"] = d
    res["runtime_s"] = round(time.time() - t0, 1)
    (OUT / "photo.json").write_text(json.dumps(res, indent=1)); print(json.dumps({k: res[k] for k in ("n_frames", "table", "diffs")}, indent=1))


# ------------------------------------------------------------------ pose quality: multi-view depth consistency
def cmd_mv():
    import torch
    t0 = time.time(); dev = "cuda"
    sets = dict(old_vio_pairs=(OLD / "fuse/moge_f", True, "old run FFS (VIO-anchored pairs), F == 1; source of eval2 gold"),
                s4m_bap=(V2 / "ffsonly_ba/s4m", False, "np/nC FFS set, baselines from bap (+ 294 VIO-anchored wide)"),
                s4m_ba3=(V2 / "w1/s4m", False, "same pair plan and tiers as s4m_bap (p2 + low + wide), baselines from BA3: like for like with s4m_bap"),
                s4m_r_ba2=(V2 / "ffs_relax/s4m_r", False, "ffsonly2 FFS set before its gate (bap + relax on BA2)"),
                s4m_rg_ba3=(V2 / "w1/s4m_rg", False, "W1 FFS set, baselines from BA3"))
    res = dict(made=time.strftime("%F %T"), n_recordings=1, inputs=dict(sets={k: str(v[0]) for k, v in sets.items()}, poses={p: str(POSES[p] / "run/dump_ba/cams.npz") for p in POSES}),
               note="the SAME keyframe depth set is checked under each keyframe pose solution. fail = gsm_ffs_4m measure rule (2x grid; point seen in "
                    "front of another view's depth by > max(3 cm, 3 %) in >= 2 views and >= 30 % of checking views). agreement = over all (pixel, other "
                    "view) checks with |dz| < 10 cm: share within 1 cm / 2 cm and median |dz| (1 mm bins); this is the sensitive number for a smooth "
                    "pose bend. se = bootstrap over keyframes. halves = even / odd keyframe index. kf_scale = per keyframe median of (depth of the "
                    "keyframe's point seen in another view / that view's own depth - 1) over the |dz| < 10 cm checks (0.1 % bins); its spread over keyframes "
                    "(robust sd = 1.4826 MAD, %) measures how well the per-keyframe stereo scale (set by the keyframe-partner baseline of the pose "
                    "solution that made the depth) agrees with the keyframe-keyframe geometry of the judging pose solution.", cells={}, set_info={k: v[2] for k, v in sets.items()})
    for sn, (sd, useF, _) in sets.items():
        kf, _, Kr, (W, H), c0 = cams("bap"); n = len(kf); vm = vmask(c0, W, H)
        Z = np.full((n, H, W), np.nan, np.float32)
        for ci, i in enumerate(kf):
            f = sd / f"{int(i):05d}.npz"
            if f.exists():
                d = np.load(f); Z[ci] = np.where(d["F"] == 1, d["Z"], np.nan) if useF else d["Z"]
        href = np.isfinite(Z).reshape(n, -1).any(1); js = [j for j in range(n) if href[j]]
        rt = torch.from_numpy(Z).to(dev); jt = torch.tensor(js, device=dev)
        fx, fy, cx, cy = [float(v) for v in (Kr[0, 0], Kr[1, 1], Kr[0, 2], Kr[1, 2])]
        for p in POSES:
            _, T, _, _, _ = cams(p)
            Tw = torch.from_numpy(np.linalg.inv(T)).float().to(dev); Tc = torch.from_numpy(T).float().to(dev)
            per = np.zeros((n, 6)); per[:, 5] = np.nan; hist = torch.zeros(100, dtype=torch.float64, device=dev)
            for ci in range(n):
                Zs = Z[ci, ::2, ::2]; val = np.isfinite(Zs) & vm[::2, ::2]
                if not val.any():
                    continue
                v, u = np.nonzero(val); z = torch.from_numpy(Zs[v, u]).float().to(dev)
                ut = torch.from_numpy(u * 2).float().to(dev); vt = torch.from_numpy(v * 2).float().to(dev)
                X = torch.stack([(ut - cx) / fx * z, (vt - cy) / fy * z, z], 1) @ Tc[ci, :3, :3].T + Tc[ci, :3, 3]
                viol = torch.zeros(len(z), dtype=torch.int32, device=dev); chk = torch.zeros_like(viol); h0 = torch.zeros(100, dtype=torch.float64, device=dev); hs = torch.zeros(100, dtype=torch.float64, device=dev)
                for k0 in range(0, len(js), 48):
                    jj = jt[k0:k0 + 48]; jj = jj[jj != ci]
                    if len(jj) == 0:
                        continue
                    Xj = torch.einsum("vij,mj->vmi", Tw[jj, :3, :3], X) + Tw[jj, None, :3, 3]; zj = Xj[..., 2]
                    pu = torch.round(fx * Xj[..., 0] / zj.clamp(min=1e-3) + cx).long(); pv = torch.round(fy * Xj[..., 1] / zj.clamp(min=1e-3) + cy).long()
                    ok = (zj > 0.05) & (pu >= 0) & (pu < W) & (pv >= 0) & (pv < H)
                    zr = rt[jj[:, None].expand_as(pu), pv.clamp(0, H - 1), pu.clamp(0, W - 1)]; zr = torch.where(ok, zr, torch.full_like(zr, float("nan")))
                    fin = torch.isfinite(zr)
                    chk += fin.sum(0).int(); viol += (fin & (zj < zr - torch.clamp(0.03 * zr, min=0.03))).sum(0).int()
                    dz = torch.abs(zj - zr)[fin]; rl = (zj / zr - 1)[fin][dz < 0.10]; dz = dz[dz < 0.10]
                    hs += torch.bincount(((rl + 0.05) * 1000).long().clamp(0, 99), minlength=100).double()
                    h0 += torch.bincount((dz * 1000).long().clamp(0, 99), minlength=100).double()
                fail = (viol >= 2) & (viol >= 0.3 * chk)
                per[ci] = [len(z), int(fail.sum()), float(h0.sum()), float(h0[:10].sum()), float(h0[:20].sum()), np.nan]; hist += h0
                if float(hs.sum()) > 5000:
                    cs = (torch.cumsum(hs, 0) / hs.sum()).cpu().numpy(); k_ = int(np.searchsorted(cs, 0.5)); c0_ = cs[k_ - 1] if k_ > 0 else 0.0
                    per[ci, 5] = (k_ + (0.5 - c0_) / max(1e-9, cs[k_] - c0_)) * 0.1 - 5.0
            hc = hist.cpu().numpy(); cum = np.cumsum(hc) / hc.sum(); ev = np.arange(n) % 2 == 0
            f_, fse = boot(per[:, 1], per[:, 0]); a1, a1se = boot(per[:, 3], per[:, 2]); a2, a2se = boot(per[:, 4], per[:, 2])
            ks = per[:, 5]; ks = ks[np.isfinite(ks)]; rsd = lambda x: float(1.4826 * np.median(np.abs(x - np.median(x))))  # noqa: E731
            kse = float(np.std([rsd(ks[RNG.integers(0, len(ks), len(ks))]) for _ in range(500)]))
            res["cells"][f"{sn}@{p}"] = dict(kf_scale_robust_sd_pct=round(rsd(ks), 4), kf_scale_robust_sd_se=round(kse, 4), kf_scale_median_pct=round(float(np.median(ks)), 4),
                                              kf_scale_p5_p95_pct=[round(float(x), 3) for x in np.percentile(ks, [5, 95])], kf_scale_n=int(len(ks)),
                                              kf_scale_halves_sd=[round(rsd(ks[0::2]), 4), round(rsd(ks[1::2]), 4)], fail_pct=f_, fail_se=fse, px_2x=int(per[:, 0].sum()), agree_1cm_pct=a1, agree_1cm_se=a1se, agree_2cm_pct=a2, agree_2cm_se=a2se,
                                              med_abs_dz_mm=int(np.searchsorted(cum, 0.5)) + 0.5, checks=int(hc.sum()),
                                              halves=dict(fail_pct=[round(float(100 * per[hm, 1].sum() / per[hm, 0].sum()), 2) for hm in (ev, ~ev)],
                                                          agree_1cm_pct=[round(float(100 * per[hm, 3].sum() / per[hm, 2].sum()), 2) for hm in (ev, ~ev)]))
            np.save(OUT / f"mv_perkf_{sn}_{p}.npy", per)
            print(sn, p, res["cells"][f"{sn}@{p}"], round(time.time() - t0), "s", flush=True)
        del rt; torch.cuda.empty_cache()
    res["runtime_s"] = round(time.time() - t0, 1)
    (OUT / "pose_mv.json").write_text(json.dumps(res, indent=1))


# ------------------------------------------------------------------ pose quality: cross reprojection of BA feature tracks
def acc(idx, vals, n):
    return np.stack([np.bincount(idx, weights=vals[:, k], minlength=n) for k in range(vals.shape[1])], 1)


def retri(R, t, K, cam, pt, uv):
    """re-triangulate tracks with fixed cam_from_world (R, t): DLT then 3 damped Gauss-Newton steps on pixel error. returns per-obs error, depth, track id."""
    up, inv = np.unique(pt, return_inverse=True); npt = len(up)
    x = (uv[:, 0] - K[0, 2]) / K[0, 0]; y = (uv[:, 1] - K[1, 2]) / K[1, 1]
    P = np.concatenate([R, t[:, :, None]], 2)[cam]                               # (m, 3, 4)
    r1 = x[:, None] * P[:, 2] - P[:, 0]; r2 = y[:, None] * P[:, 2] - P[:, 1]
    M = acc(inv, (r1[:, :, None] * r1[:, None, :] + r2[:, :, None] * r2[:, None, :]).reshape(-1, 16), npt).reshape(-1, 4, 4)
    wv = np.linalg.eigh(M)[1][:, :, 0]; X = wv[:, :3] / np.where(np.abs(wv[:, 3:]) < 1e-12, 1e-12, wv[:, 3:])
    Rc = R[cam]; tc = t[cam]
    for it in range(4):
        Xc = np.einsum("nij,nj->ni", Rc, X[inv]) + tc; z = np.where(np.abs(Xc[:, 2]) < 1e-6, 1e-6, Xc[:, 2])
        q = np.stack([K[0, 0] * Xc[:, 0] / z + K[0, 2], K[1, 1] * Xc[:, 1] / z + K[1, 2]], 1); r = q - uv
        if it == 3:
            break
        J1 = (K[0, 0] / z)[:, None] * (Rc[:, 0] - (Xc[:, 0] / z)[:, None] * Rc[:, 2]); J2 = (K[1, 1] / z)[:, None] * (Rc[:, 1] - (Xc[:, 1] / z)[:, None] * Rc[:, 2])
        Hn = acc(inv, (J1[:, :, None] * J1[:, None, :] + J2[:, :, None] * J2[:, None, :]).reshape(-1, 9), npt).reshape(-1, 3, 3)
        g = acc(inv, J1 * r[:, :1] + J2 * r[:, 1:], npt)
        Hn += (1e-6 * np.trace(Hn, axis1=1, axis2=2) + 1e-12)[:, None, None] * np.eye(3)
        X = X - np.linalg.solve(Hn, g[:, :, None])[:, :, 0]
    return np.linalg.norm(r, axis=1), z, inv


def cmd_xrep():
    t0 = time.time(); kf, _, _, _, _ = cams("bap"); kfs = set(int(x) for x in kf)
    PA = {p: np.load(POSES[p] / "poses_all.npz") for p in POSES}
    res = dict(made=time.strftime("%F %T"), n_recordings=1, inputs=dict(tracks={p: str(POSES[p] / "ba/ba_state.npz") for p in POSES}, poses={p: str(POSES[p] / "poses_all.npz") for p in POSES}),
               note="tracks (2D feature observations) of BA solution A; cameras = poses of solution B (frames with src == ba in B); observations of frames "
                    "missing in B dropped, tracks with < 3 remaining observations dropped; points re-triangulated with B's poses (DLT + 3 GN), so the "
                    "number depends only on B's camera poses. kf_only = keyframe observations only (the same 494 keyframes in every solution). "
                    "loop = tracks whose observations span > 300 frames (revisits). own cell (A == B) is the reference; each BA was optimised on "
                    "its own tracks, so the cross cells are the informative ones: a pose solution that is right must also fit the other solution's matches.",
               cells={})
    for a in POSES:
        st = np.load(POSES[a] / "ba/ba_state.npz"); cam, pt, uv, K = st["cam"], st["pt"], st["uv"].astype(np.float64), st["K"]; fa = PA[a]["frames"]
        for b in POSES:
            fb = {int(f): k for k, f in enumerate(PA[b]["frames"]) if PA[b]["src"][k] == "ba"}
            Tb = PA[b]["T"]; Rb = np.transpose(Tb[:, :3, :3], (0, 2, 1)); tb = -np.einsum("nij,nj->ni", Rb, Tb[:, :3, 3])
            mapi = np.array([fb.get(int(f), -1) for f in fa]); iskf = np.array([int(f) in kfs for f in fa])
            for tag, base in (("all_common", mapi[cam] >= 0), ("kf_only", (mapi[cam] >= 0) & iskf[cam])):
                cnt = np.bincount(pt[base], minlength=pt.max() + 1); m = base & (cnt[pt] >= 3)
                e, z, inv = retri(Rb, tb, K, mapi[cam[m]], pt[m], uv[m])
                fr = fa[cam[m]].astype(np.int64); npt = inv.max() + 1
                lo = np.full(npt, 10 ** 9); hi = np.full(npt, -1); np.minimum.at(lo, inv, fr); np.maximum.at(hi, inv, fr); loop = (hi - lo > 300)[inv]
                d = {}
                for nm, s in (("all", np.ones(len(e), bool)), ("loop", loop), ("local", ~loop)):
                    ee = e[s]
                    d[nm] = dict(n_obs=int(s.sum()), n_tracks=int(np.unique(inv[s]).size), med_px=round(float(np.median(ee)), 4), mean_px_clip10=round(float(np.minimum(ee, 10).mean()), 4),
                                 p90_px=round(float(np.percentile(ee, 90)), 3), frac_gt_1px=round(float((ee > 1).mean()), 4), frac_gt_2px=round(float((ee > 2).mean()), 4))
                d["behind_cam_frac"] = round(float((z <= 0).mean()), 5)
                res["cells"][f"tracks_{a}@poses_{b}/{tag}"] = d; print(a, b, tag, d["all"], d["loop"], round(time.time() - t0), "s", flush=True)
    res["runtime_s"] = round(time.time() - t0, 1)
    (OUT / "pose_xrep.json").write_text(json.dumps(res, indent=1))


# ------------------------------------------------------------------ claims
def cmd_claims():
    kf, T3, _, _, _ = cams("ba3"); _, T2, _, _, _ = cams("ba2"); _, Tb, _, _, _ = cams("bap")
    inv = json.loads((V2 / "w1/inventory.json").read_text())["pairs"]
    wo = np.load(V2 / "ffsonly_ba/wide_poses.npz"); P3 = np.load(POSES["ba3"] / "poses_all.npz"); PB = np.load(POSES["bap"] / "poses_all.npz")
    f3 = {int(f): P3["T"][k] for k, f in enumerate(P3["frames"])}; fb = {int(f): PB["T"][k] for k, f in enumerate(PB["frames"])}
    out = dict(made=time.strftime("%F %T"), n_recordings=1, inputs=dict(inventory=str(V2 / "w1/inventory.json"), wide_old=str(V2 / "ffsonly_ba/wide_poses.npz"),
                                                                          ba3=str(POSES["ba3"] / "poses_all.npz"), bap=str(POSES["bap"] / "poses_all.npz")),
               note="independent recomputation (own code) of the W1 worker numbers. wide pair old pose = bap keyframe + ffsonly_ba/wide_poses partner "
                    "(matched by frame and anchor keyframe); new = BA3 keyframe + BA3 partner.")
    rot = lambda Ra, Rb: float(np.degrees(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1))))  # noqa: E731
    wt = {}
    for k in range(len(wo["frames"])):
        wt.setdefault(int(wo["frames"][k]), []).append((int(wo["anchor"][k]), wo["T"][k], str(wo["src"][k])))
    out["wide_poses_src_counts"] = {s: int((wo["src"] == s).sum()) for s in np.unique(wo["src"])}
    for nm, pp in (("wide_vio_anchor", "bap:vio_anchor"), ("wide_ba", "bap:ba")):
        rat, rr, amb = [], [], 0
        for r in inv:
            if r["tier"] != "wide" or r["partner_pose"] != pp or r["kf"] not in f3 or r["j"] not in f3 or r["kf"] not in fb:
                continue
            c = [x for x in wt.get(r["j"], []) if x[0] == r["kf"]]
            if not c:
                c = wt.get(r["j"], []); amb += 1
            if not c:
                continue
            To = c[0][1]; Tk = fb[r["kf"]]
            Bo = np.linalg.norm(To[:3, 3] - Tk[:3, 3]); Bn = np.linalg.norm(f3[r["j"]][:3, 3] - f3[r["kf"]][:3, 3]); rat.append(Bn / Bo)
            rr.append(rot(Tk[:3, :3].T @ To[:3, :3], f3[r["kf"]][:3, :3].T @ f3[r["j"]][:3, :3]))
        rat = np.array(rat); rr = np.array(rr)
        out[nm] = dict(n=int(len(rat)), anchor_not_matched=amb, B_ratio_absdev_med_pct=round(float(np.median(np.abs(rat - 1)) * 100), 3),
                       B_ratio_p5_50_95=[round(float(x), 4) for x in np.percentile(rat, [5, 50, 95])],
                       frac_off_gt_1pct=round(float((np.abs(rat - 1) > 0.01).mean()), 4), frac_off_gt_2pct=round(float((np.abs(rat - 1) > 0.02).mean()), 4),
                       rel_rot_deg_p50_p95=[round(float(x), 4) for x in np.percentile(rr, [50, 95])],
                       rel_rot_as_px_p50_p95=[round(float(643.1 * np.tan(np.radians(x))), 2) for x in np.percentile(rr, [50, 95])])
    for nm, (A_, B_) in (("ba3_vs_ba2", (T3, T2)), ("ba3_vs_bap", (T3, Tb)), ("ba2_vs_bap", (T2, Tb))):
        d = np.linalg.norm(A_[:, :3, 3] - B_[:, :3, 3], axis=1) * 100
        out[nm] = dict(shift_cm_med=round(float(np.median(d)), 3), p95=round(float(np.percentile(d, 95)), 3), max=round(float(d.max()), 3), n=int(len(d)),
                       note="both solutions were Sim3-aligned to the same old keyframe centres; no extra alignment here")
    mw = json.loads((V2 / "w1/measure_w1.json").read_text())["sets"]; mr = json.loads((V2 / "ffs_relax/measure_relax.json").read_text())["sets"]
    out["gate"] = dict(w1_s4m_rg_all=mw["s4m_rg"]["all"], w1_s4m_rg_new=mw["s4m_rg"]["new_pixels_only"], w1_s4m_rg_aisle=mw["s4m_rg"]["aisle_314_331"], w1_s4m_all=mw["s4m"]["all"],
                       nC_s4m_judge_bap=mr["s4m_old_judge_bap"]["all"], ffsonly2_s4m_r_judge_ba2=mr["s4m_r_judge_ba2"]["all"],
                       files=[str(V2 / "w1/measure_w1.json"), str(V2 / "ffs_relax/measure_relax.json")])
    (OUT / "claims.json").write_text(json.dumps(out, indent=1)); print(json.dumps(out, indent=1))


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    cmd = sys.argv[1]
    if cmd == "mv":
        import gpu_guard
        with gpu_guard.job("gsmap r_judge mv", torch_cap=True):
            cmd_mv()
    else:
        dict(geo=cmd_geo, photo=cmd_photo, xrep=cmd_xrep, claims=cmd_claims)[cmd]()
