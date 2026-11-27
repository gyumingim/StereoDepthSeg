#!/usr/bin/env python3
"""gsmap 2026-11-26: FFS pair re-selection (step 1) + two-pair agreement fusion (step 2) on top of gsm_ffs_dense.py.

  # CPU: plan only (old rule vs new rule), writes OUT/plan.json
  python scan3d/gsmap/gsm_ffs_pairs2.py plan RUN --vio VIO --rec REC --out OUT
  # GPU: dense FFS with the new plan, per-pair depths -> OUT/pairs/<kf>.npz, old-style combine -> OUT/s1/<kf>.npz
  FFS_ROOT=~/scanner/StereoDepthSeg ~/scanner/StereoDepthSeg/venv_ffs/bin/python scan3d/gsmap/gsm_ffs_pairs2.py ffs RUN --vio .. --rec .. --out OUT
  # CPU: step 2 fusion from OUT/pairs -> OUT/s12/<kf>.npz
  python scan3d/gsmap/gsm_ffs_pairs2.py fuse RUN --vio .. --rec .. --out OUT [--k 2]

Step 1 rule (vs gsm_ffs_dense defaults): min_perp 40 (was 30); reject if the epipole projects inside the image in either frame;
up to 4 partners; a further partner is accepted if its baseline direction differs >= --dir-deg (was 25) from every chosen one OR
its baseline length differs by a factor >= 1.5, and it is >= 0.2 s away from every chosen partner. B 3-25 cm, rot <= 25, win 2 s kept.
Step 2 rule: sigma_px = Z^2/(f B) * 1 px per pair; a pixel is kept if >= 2 pairs agree within k*hypot(sigma_a, sigma_b)
(fused = inverse-variance mean of the pairs agreeing with the best-supported pair). Pixels seen by one pair only are kept only if
they agree (same tolerance, neighbour sigma from its stored S) with a reprojected neighbouring keyframe's step-1 FFS (the 2 nearest
keyframes on each side that have FFS, BA poses).
"""
import argparse, gzip, json, os, sys, time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / "gsplat"))
if os.environ.get("FFS_ROOT"):
    sys.path.insert(0, os.environ["FFS_ROOT"])
import motion_stereo as ms


def epi_inside(K, p, W, H):
    """projection of direction p (camera coords) lies inside the W x H image (either sign of z)."""
    if abs(p[2]) < 1e-9:
        return False
    u = K[0, 0] * p[0] / p[2] + K[0, 2]; v = K[1, 1] * p[1] / p[2] + K[1, 2]
    return bool(0 <= u < W and 0 <= v < H)


def load_common(a):
    run = Path(a.run)
    c = np.load(run / "dump_ba" / "cams.npz"); kf, Tk, Kr = c["kf"], c["T"], c["Kr"]; W0, H0 = [int(v) for v in c["size"]]
    g = np.load(Path(a.rec) / a.gt / "gt.npz"); t_ns = g["t_ns"]; K = g["K"].copy(); K[:2] *= W0 / 1920.0; dist = g["dist"].astype(np.float64)
    vt, vT = [], []
    for line in gzip.open(a.vio, "rt"):
        d = json.loads(line)
        if d.get("type") == "pose" and d.get("T_world_cam0") and d.get("status") == "tracking":
            vt.append(d["t_ns"]); vT.append(np.array(d["T_world_cam0"], float).reshape(4, 4))
    vt = np.array(vt); vT = np.array(vT)
    idx = np.searchsorted(t_ns, vt).clip(1, len(t_ns) - 1); idx = np.where(np.abs(t_ns[idx - 1] - vt) < np.abs(t_ns[idx] - vt), idx - 1, idx)
    good = np.abs(t_ns[idx] - vt) < 8e6
    Tv = {int(i): T for i, T in zip(idx[good], vT[good])}
    return kf, Tk, Kr, W0, H0, t_ns, K, dist, Tv


def make_plan(kf, Tk, K, W0, H0, t_ns, Tv, win_s, b_min, b_max, min_perp, max_rot, n_pairs, new):
    cfg = ms.MSConfig(); cfg.b_target_m = 0.08; cfg.dt_max_s = win_s; cfg.b_min_m = b_min; cfg.b_max_m = b_max
    fb = new.get("fb_perp"); cfg.min_perp_deg = min(min_perp, fb) if fb else min_perp; cfg.max_rot_deg = max_rot
    win = int(round(win_s * 60)); plan, why_all, fbk = {}, Counter(), set()
    for ci, fi in enumerate(kf):
        fi = int(fi)
        if fi not in Tv:
            continue
        A = Tk[ci] @ np.linalg.inv(Tv[fi]); cands = []
        for j in range(max(0, fi - win), min(len(t_ns), fi + win + 1)):
            if j == fi or j not in Tv:
                continue
            Tj = A @ Tv[j]; R, t = ms.rel_pose(Tk[ci], Tj); gg = ms.pair_geometry(R, t)
            s, w = ms.score_pair(gg, abs(t_ns[fi] - t_ns[j]) * 1e-9, cfg)
            strict = s is not None and gg["perp_deg"] >= min_perp and not (new["epi"] and (epi_inside(K, gg["c"], W0, H0) or epi_inside(K, t, W0, H0)))
            if s is not None and not strict:
                w = "forward" if gg["perp_deg"] < min_perp else "epipole"
            why_all[w] += 1
            if s is not None:
                cands.append((s, j, gg, Tj, strict))
        if any(q[4] for q in cands):
            cands = [q for q in cands if q[4]]
        elif not fb:
            cands = []
        else:        # fallback: no strict pair -> old rule (perp >= fb, epipole not tested); these kf are tagged
            fbk.add(ci)
        cands = [q[:4] for q in cands]
        cands.sort(key=lambda x: x[0]); chosen = []
        for s, j, gg, Tj in cands:
            if not chosen:
                ok = True
            elif new["relax"]:
                ok = all((ms.dir_differs(gg, q[2], new["dir_deg"]) or max(gg["B"], q[2]["B"]) / min(gg["B"], q[2]["B"]) >= 1.5)
                         and abs(j - q[1]) >= 12 for q in chosen)
            else:
                ok = all(ms.dir_differs(gg, q[2], cfg.pair2_min_dir_deg) for q in chosen)
            if ok:
                chosen.append((s, j, gg, Tj))
            if len(chosen) >= n_pairs:
                break
        if chosen:
            plan[ci] = chosen
    cfg.min_perp_deg = min_perp; cfg.fallback_kf = sorted(int(kf[c]) for c in fbk if c in plan)
    return plan, cfg, dict(why_all)


def plan_summary(plan, n):
    c = Counter(len(v) for v in plan.values())
    return dict(n_kf=n, ge1=len(plan), ge2=sum(v for k, v in c.items() if k >= 2), hist={int(k): int(v) for k, v in sorted(c.items())},
                n_pairs_total=int(sum(len(v) for v in plan.values())),
                perp_min_med=float(np.median([min(q[2]["perp_deg"] for q in v) for v in plan.values()])) if plan else 0)


def covis_points(run, kf, Kr, step=24):
    """per keyframe: 3-D points in its own camera (old dense FFS where present, else v2 aligned MoGe Zai), every step px."""
    old = run / "ffs_dense"; mg = Path(str(run) + "_v2") / "fuse" / "moge"; pts = []
    fx, fy, cx, cy = Kr[0, 0], Kr[1, 1], Kr[0, 2], Kr[1, 2]
    for i in kf:
        Z = None; src = "none"
        if (old / f"{int(i):05d}.npz").exists():
            Z = np.load(old / f"{int(i):05d}.npz")["Z"][::step, ::step].astype(np.float32); src = "ffs"
            if np.isfinite(Z).sum() < 50:
                Z = None
        if Z is None and (mg / f"{int(i):05d}.npz").exists():
            Z = np.load(mg / f"{int(i):05d}.npz")["Zai"][::step, ::step].astype(np.float32); src = "moge"
        if Z is None:
            pts.append((np.zeros((0, 3)), src)); continue
        v, u = np.nonzero(np.isfinite(Z) & (Z > 0.05) & (Z < 6)); z = Z[v, u]; u = u * step; v = v * step
        pts.append((np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z], 1), src))
    return pts


def make_plan_all(a, run, kf, Tk, Kr, K, W0, H0, t_ns, Tv):
    """whole-video, pose-only, vectorised partner search (user 2026-11-26). Partner pose: within +-win of the keyframe = VIO relative
    motion anchored on the keyframe BA pose (as gsm_ffs_dense); a keyframe = its BA pose ('ba'); else VIO anchored on the nearest
    keyframe in time ('vio_revisit', may carry drift)."""
    win = int(round(a.win_s * 60)); fr = np.array(sorted(Tv)); TV = np.stack([Tv[j] for j in fr])
    kfi = {int(f): c for c, f in enumerate(kf)}; kfr = np.array([int(f) for f in kf if int(f) in Tv]); kfc = np.array([kfi[f] for f in kfr])
    near = kfc[np.abs(fr[:, None] - kfr[None, :]).argmin(1)]
    Tglob = np.einsum("nij,njk->nik", np.einsum("nij,njk->nik", Tk[near], np.linalg.inv(np.stack([Tv[int(kf[c])] for c in near]))), TV)
    src0 = np.array(["ba" if int(j) in kfi else "vio_revisit" for j in fr], dtype=object)
    for n_, j in enumerate(fr):
        if int(j) in kfi:
            Tglob[n_] = Tk[kfi[int(j)]]
    pts = covis_points(run, kf, Kr); plan, info = {}, {}
    for ci, fi in enumerate(kf):
        fi = int(fi)
        if fi not in Tv:
            continue
        Tq = Tglob.copy(); src = src0.copy(); loc = np.abs(fr - fi) <= win
        Tq[loc] = np.einsum("ij,njk->nik", Tk[ci] @ np.linalg.inv(Tv[fi]), TV[loc]); src[loc] = "local"
        Rq, pq = Tk[ci][:3, :3], Tk[ci][:3, 3]
        c = (Tq[:, :3, 3] - pq) @ Rq                                  # partner centre in kf coords
        R = np.einsum("nji,jk->nik", Tq[:, :3, :3], Rq)              # p_j = R p_i + t
        t = -np.einsum("nij,nj->ni", R, c)
        B = np.linalg.norm(c, axis=1); u_ = c / np.maximum(B, 1e-9)[:, None]
        perp = np.degrees(np.arccos(np.clip(np.abs(u_[:, 2]), 0, 1)))
        rot = np.degrees(np.arccos(np.clip((np.trace(R, axis1=1, axis2=2) - 1) / 2, -1, 1)))
        e = np.where((u_[:, 0] < 0)[:, None], -u_, u_); roll = np.degrees(np.arctan2(e[:, 1], e[:, 0]))

        def epi(p):
            with np.errstate(divide="ignore", invalid="ignore"):
                uu = K[0, 0] * p[:, 0] / p[:, 2] + K[0, 2]; vv = K[1, 1] * p[:, 1] / p[:, 2] + K[1, 2]
            return (np.abs(p[:, 2]) > 1e-9) & (uu >= 0) & (uu < W0) & (vv >= 0) & (vv < H0)
        dt = np.abs(t_ns[fr] - t_ns[fi]) * 1e-9
        ok = (dt >= 0.15) & (B >= a.b_min) & (B <= a.b_max) & (perp >= a.min_perp) & (rot <= a.max_rot) & ~epi(c) & ~epi(t)
        P, psrc = pts[ci]; cov = np.zeros(len(fr)); idx = np.nonzero(ok)[0]
        if len(idx) and len(P):
            Xj = np.einsum("nij,mj->nmi", R[idx], P) + t[idx][:, None, :]; zj = Xj[..., 2]
            with np.errstate(divide="ignore", invalid="ignore"):
                pu = K[0, 0] * Xj[..., 0] / zj + K[0, 2]; pv = K[1, 1] * Xj[..., 1] / zj + K[1, 2]
            cov[idx] = ((zj > 0.05) & (pu >= 0) & (pu < W0) & (pv >= 0) & (pv < H0)).mean(1)
        elif len(idx):
            cov[idx] = 1.0
        ok &= cov >= 0.6
        sc = (np.abs(np.log(np.maximum(B, 1e-6) / getattr(a, "b_target", 0.08))) + 1.5 * (1 - np.sin(np.radians(perp))) + 0.5 * rot / a.max_rot + 0.3 * np.abs(roll) / 90
              + 0.2 * np.minimum(dt, 2.0) / 2.0 + 0.3 * (src == "vio_revisit"))
        chosen = []; excl = getattr(a, "exclude", {}).get(ci, set())
        for n_ in np.nonzero(ok)[0][np.argsort(sc[ok])]:
            if int(fr[n_]) in excl:
                continue
            gg = dict(B=float(B[n_]), c=c[n_], perp_deg=float(perp[n_]), rot_deg=float(rot[n_]), roll_deg=float(roll[n_]))
            j = int(fr[n_])
            if all((ms.dir_differs(gg, q[2], a.dir_deg) or max(gg["B"], q[2]["B"]) / min(gg["B"], q[2]["B"]) >= 1.5) and abs(j - q[1]) >= 12 for q in chosen):
                chosen.append((float(sc[n_]), j, gg, Tq[n_], str(src[n_])))
            if len(chosen) >= a.n_pairs:
                break
        info[ci] = dict(n_ok=int(ok.sum()), n_ok_local=int((ok & loc).sum()), n_ok_far=int((ok & ~loc).sum()), pts=psrc)
        if chosen:
            plan[ci] = chosen
    return plan, info


def new_plan(a, kf, Tk, K, W0, H0, t_ns, Tv, Kr=None):
    """whole-video strict plan; keyframes with no strict partner fall back to the old +-2 s rule (perp >= fb, no epipole test)."""
    p_all, info = make_plan_all(a, Path(a.run), kf, Tk, Kr, K, W0, H0, t_ns, Tv)
    fbp, cfg, _ = make_plan(kf, Tk, K, W0, H0, t_ns, Tv, a.win_s, a.b_min, a.b_max, a.fb_perp or a.min_perp, a.max_rot, a.n_pairs,
                            dict(epi=False, relax=True, dir_deg=a.dir_deg))
    plan = dict(p_all); fb = []
    for ci, ch in fbp.items():
        if ci not in plan and a.fb_perp > 0:
            plan[ci] = [q + ("fallback",) for q in ch]; fb.append(int(kf[ci]))
    cfg.min_perp_deg = a.min_perp; cfg.fallback_kf = sorted(fb); cfg.search_info = info
    return plan, cfg, {}


def new_plan_window(a, kf, Tk, K, W0, H0, t_ns, Tv):
    return make_plan(kf, Tk, K, W0, H0, t_ns, Tv, a.win_s, a.b_min, a.b_max, a.min_perp, a.max_rot, a.n_pairs,
                     dict(epi=True, relax=True, dir_deg=a.dir_deg, fb_perp=a.fb_perp))


def cmd_plan(a):
    kf, Tk, Kr, W0, H0, t_ns, K, dist, Tv = load_common(a); out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    old, _, why_o = make_plan(kf, Tk, K, W0, H0, t_ns, Tv, 2.0, 0.03, 0.25, 30.0, 25.0, 3, dict(epi=False, relax=False))
    res = dict(made=time.strftime("%F %T"), args=vars(a), old_rule=dict(plan_summary(old, len(kf)), why=why_o))
    variants = {}
    for dd in (25.0, 10.0):
        for perp, fbp in ((40.0, None), (40.0, 30.0), (40.0, 35.0), (30.0, None)):
            p, _, why = make_plan(kf, Tk, K, W0, H0, t_ns, Tv, a.win_s, a.b_min, a.b_max, perp, a.max_rot, a.n_pairs, dict(epi=True, relax=True, dir_deg=dd, fb_perp=fbp))
            variants[f"perp{perp:.0f}_fb{fbp}_epi_dir{dd:.0f}"] = dict(plan_summary(p, len(kf)), lost_vs_old=sorted(int(kf[c]) for c in old if c not in p),
                                                              gained_vs_old=len([c for c in p if c not in old]), why=why)
    p40, _, _ = make_plan(kf, Tk, K, W0, H0, t_ns, Tv, a.win_s, a.b_min, a.b_max, 40.0, a.max_rot, 3, dict(epi=False, relax=False))
    variants["perp40_only_old_select"] = plan_summary(p40, len(kf))
    res["variants"] = variants
    p, cf, _ = new_plan(a, kf, Tk, K, W0, H0, t_ns, Tv, Kr)
    noffs = [c for c in range(len(kf)) if c not in old]; inf = cf.search_info
    src_n = Counter(q[4] for v in p.values() for q in v)
    res["whole_video"] = dict(n_old_noffs=len(noffs), noffs_gain_window_strict=sum(inf.get(c, {}).get("n_ok_local", 0) > 0 for c in noffs),
                              noffs_gain_whole_video=sum(inf.get(c, {}).get("n_ok", 0) > 0 for c in noffs),
                              noffs_gain_far_only=sum(inf.get(c, {}).get("n_ok_local", 0) == 0 and inf.get(c, {}).get("n_ok", 0) > 0 for c in noffs),
                              noffs_no_vio=sum(c not in inf for c in noffs),
                              kf_strict_window=sum(v["n_ok_local"] > 0 for v in inf.values()), kf_strict_whole=sum(v["n_ok"] > 0 for v in inf.values()),
                              pair_src=dict(src_n), kf_with_far_partner=sum(any(q[4] in ("ba", "vio_revisit") for q in v) for v in p.values()),
                              covis_pts_src=dict(Counter(v["pts"] for v in inf.values())))
    res["chosen"] = dict(dir_deg=a.dir_deg, fb_perp=a.fb_perp, fallback_kf=cf.fallback_kf, **plan_summary(p, len(kf)), lost_vs_old=sorted(int(kf[c]) for c in old if c not in p),
                         plan={int(kf[c]): [[int(q[1]), q[4], round(q[2]["B"] * 100, 1), round(q[2]["perp_deg"], 1)] for q in v] for c, v in p.items()})
    (out / "plan.json").write_text(json.dumps(res, indent=1, default=float))
    print(json.dumps({k: v for k, v in res.items() if k not in ("chosen", "variants")} | {"chosen": {k: v for k, v in res["chosen"].items() if k != "plan"}}, default=float, indent=1))


def cmd_ffs(a):
    import lib_ffs
    t0 = time.time()
    kf, Tk, Kr, W0, H0, t_ns, K, dist, Tv = load_common(a); out = Path(a.out)
    plan, cfg, _ = new_plan(a, kf, Tk, K, W0, H0, t_ns, Tv, Kr); cfg.z_max = a.zmax
    (out / "pairs").mkdir(parents=True, exist_ok=True); (out / "s1").mkdir(exist_ok=True)
    rp = dict(size=(W0, H0), f=float(K[0, 0]), cx=float(K[0, 2]), cy=float(K[1, 2]), R1=np.eye(3).tolist())
    win = int(round(a.win_s * 60)); need = {}
    for ci, ch in plan.items():
        for q in ch:
            need.setdefault(q[1], []).append(ci)
        need.setdefault(int(kf[ci]), []).append(ci)
    print(f"plan: {len(plan)}/{len(kf)} kf, {sum(len(v) for v in plan.values())} pairs, {len(need)} frames, {time.time() - t0:.1f}s", flush=True)
    model = lib_ffs.load(valid_iters=a.iters)
    ready0 = {ci: max([int(kf[ci])] + [q[1] for q in plan[ci]]) for ci in plan}
    last_use = {f: max(ready0[ci] for ci in cis) for f, cis in need.items()}
    todo = [ci for ci in plan if not (out / "s1" / f"{int(kf[ci]):05d}.npz").exists()]
    todo.sort(key=lambda ci: max([int(kf[ci])] + [q[1] for q in plan[ci]]))
    ready_at = {ci: max([int(kf[ci])] + [q[1] for q in plan[ci]]) for ci in plan}
    print(f"todo {len(todo)} (resume skips {len(plan) - len(todo)})", flush=True)
    cap = cv2.VideoCapture(str(Path(a.rec) / "cam5.mp4")); buf = {}; fidx = -1; ti = 0; stats = {}
    while ti < len(todo):
        r_, f = cap.read(); fidx += 1
        if not r_:
            break
        if fidx in need:
            buf[fidx] = cv2.resize(f, (W0, H0), interpolation=cv2.INTER_AREA) if f.shape[1] != W0 else f
        while ti < len(todo) and ready_at[todo[ti]] <= fidx:
            ci = todo[ti]; ti += 1; fi = int(kf[ci]); res = []
            for s, j, gg, Tj, _src in plan[ci]:
                R, t = ms.rel_pose(Tk[ci], Tj)
                res.append(ms.pair_depth(model, lib_ffs.infer, buf[fi], buf[j], R, t, K, dist, rp, cfg, iters=a.iters))
            Z, V, S, agree = ms.combine_motion(res, cfg)
            np.savez_compressed(out / "pairs" / f"{fi:05d}.npz", Z=np.stack([np.where(r["valid"], r["Z"], np.nan) for r in res]).astype(np.float16),
                                B=np.array([r["B"] for r in res]), f=np.array([r["f"] for r in res]), partner=np.array([q[1] for q in plan[ci]]), src=np.array([q[4] for q in plan[ci]]),
                                perp=np.array([r["perp_deg"] for r in res]))
            np.savez_compressed(out / "s1" / f"{fi:05d}.npz", Z=np.where(V, Z, np.nan).astype(np.float32), V=V, S=np.where(V, S, np.inf).astype(np.float32))
            stats[fi] = dict(pairs=len(res), valid_frac=float(V.mean()), B_cm=[round(r["B"] * 100, 1) for r in res])
            if ti % 25 == 0:
                print(f"{ti}/{len(todo)} kf {fi} pairs {len(res)} valid {V.mean():.3f} t {time.time() - t0:.0f}s", flush=True)
        for fdel in [k for k in buf if last_use[k] < fidx]:
            del buf[fdel]
    cap.release()
    (out / "s1" / "ffs_info.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), args=vars(a), fallback_kf=cfg.fallback_kf, n_done=len(stats), total_s=round(time.time() - t0, 1),
                                                              per_kf=stats), indent=1, default=float))
    print(f"done {len(stats)} kf in {time.time() - t0:.0f}s", flush=True)


def cmd_fuse(a):
    t0 = time.time()
    run, out = Path(a.run), Path(a.out); (out / "s12").mkdir(parents=True, exist_ok=True)
    c = np.load(run / "dump_ba" / "cams.npz"); kf, T, Kr = c["kf"], c["T"], c["Kr"]; W, H = [int(v) for v in c["size"]]; n = len(kf)
    fx, fy, cx, cy = [float(v) for v in (Kr[0, 0], Kr[1, 1], Kr[0, 2], Kr[1, 2])]
    has = [(out / "pairs" / f"{int(i):05d}.npz").exists() for i in kf]
    S1 = {}
    def s1(ci):
        if ci not in S1:
            d = np.load(out / "s1" / f"{int(kf[ci]):05d}.npz"); S1[ci] = (d["Z"], d["S"])
        return S1[ci]
    vv, uu = np.mgrid[0:H, 0:W]; info = {}
    for ci in range(n):
        if not has[ci]:
            continue
        d = np.load(out / "pairs" / f"{int(kf[ci]):05d}.npz"); Zp = d["Z"].astype(np.float32); B = d["B"]; f = d["f"]; m = len(B)
        sig = np.stack([Zp[p] ** 2 / (f[p] * B[p]) for p in range(m)])      # 1 px of disparity
        val = np.isfinite(Zp); nv = val.sum(0)
        Zo = np.full((H, W), np.nan, np.float32); So = np.full((H, W), np.inf, np.float32)
        # ---- >= 2 pairs agree
        if m >= 2:
            sup = np.zeros((m, H, W), np.int32)
            for p in range(m):
                for q in range(m):
                    if p != q:
                        with np.errstate(invalid="ignore"):
                            sup[p] += (val[p] & val[q] & (np.abs(Zp[p] - Zp[q]) <= a.k * np.hypot(sig[p], sig[q])))
            best = np.argmax(sup - 1e-3 * np.nan_to_num(sig, nan=1e9).clip(0, 999), 0)   # most support, then smallest sigma
            ok2 = np.take_along_axis(sup, best[None], 0)[0] >= 1
            Zb = np.take_along_axis(Zp, best[None], 0)[0]; sb = np.take_along_axis(sig, best[None], 0)[0]
            num = np.zeros((H, W)); den = np.zeros((H, W))
            for p in range(m):
                with np.errstate(invalid="ignore"):
                    inn = ok2 & val[p] & (np.abs(Zp[p] - Zb) <= a.k * np.hypot(sig[p], sb))
                w = np.where(inn, 1.0 / np.maximum(np.nan_to_num(sig[p], nan=1.0), 1e-6) ** 2, 0)
                num += np.where(inn, np.nan_to_num(Zp[p]), 0) * w; den += w
            with np.errstate(invalid="ignore", divide="ignore"):
                Zo[ok2] = (num / den)[ok2]; So[ok2] = (1.0 / np.sqrt(den))[ok2]
        else:
            ok2 = np.zeros((H, W), bool)
        # ---- single-pair keyframes: neighbour check
        single = (nv == 1) & (m == 1)
        n_single = int(single.sum()); n_nb = 0
        if single.any():
            p0 = 0; z = Zp[p0][single]; s0 = sig[p0][single]; u = uu[single]; v = vv[single]
            X = np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z], 1) @ T[ci, :3, :3].T + T[ci, :3, 3]
            nbs = [j for j in range(ci - 1, -1, -1) if has[j]][:2] + [j for j in range(ci + 1, n) if has[j]][:2]
            okn = np.zeros(len(z), bool)
            for j in nbs:
                Tw = np.linalg.inv(T[j]); Xj = X @ Tw[:3, :3].T + Tw[:3, 3]; zj = Xj[:, 2]
                with np.errstate(invalid="ignore", divide="ignore"):
                    pu = np.round(fx * Xj[:, 0] / np.maximum(zj, 1e-3) + cx).astype(int); pv = np.round(fy * Xj[:, 1] / np.maximum(zj, 1e-3) + cy).astype(int)
                ok = (zj > 0.05) & (pu >= 0) & (pu < W) & (pv >= 0) & (pv < H)
                Zj, Sj = s1(j); zr = np.full(len(z), np.nan, np.float32); sr = np.full(len(z), np.inf, np.float32)
                zr[ok] = Zj[pv[ok], pu[ok]]
                # neighbour S1 sigma is the old unanchored model (incl. rotation term) -> use 1 px at its own depth with B 8 cm as a stand-in
                sr = zr ** 2 / (fx * 0.08)
                with np.errstate(invalid="ignore"):
                    okn |= np.isfinite(zr) & (np.abs(zj - zr) <= a.k * np.hypot(s0, sr))
            Zs = Zo[single]; Zs[okn] = z[okn]; Zo[single] = Zs
            Ss = So[single]; Ss[okn] = s0[okn]; So[single] = Ss; n_nb = int(okn.sum())
        V = np.isfinite(Zo)
        np.savez_compressed(out / "s12" / f"{int(kf[ci]):05d}.npz", Z=Zo, V=V, S=np.where(V, So, np.inf).astype(np.float32))
        info[int(kf[ci])] = dict(pairs=m, valid_frac=float(V.mean()), px_any=int((nv >= 1).sum()), px_agree2=int(ok2.sum()),
                                 px_single=n_single, px_single_nb_ok=n_nb)
        if ci % 50 == 0:
            print(ci, info[int(kf[ci])], f"{time.time() - t0:.0f}s", flush=True)
    (out / "s12" / "fuse_info.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), args=vars(a), per_kf=info), indent=1))
    print(f"fuse done {len(info)} kf {time.time() - t0:.0f}s", flush=True)



# ================================================================ 4 m extension (user 2026-11-26)
def wide_plan(a, kf, Tk, Kr, K, W0, H0, t_ns, Tv):
    """up to --n-wide extra wide partners per keyframe: B --wb-min..--wb-max, same pose-only score (target --wb-target), perp >= min_perp,
    epipole outside, rot <= max_rot, covisibility >= 60 %; frames already used as short partners are excluded."""
    short, _, _ = new_plan(a, kf, Tk, K, W0, H0, t_ns, Tv, Kr)
    import copy
    b = copy.copy(a); b.b_min, b.b_max, b.n_pairs, b.b_target = a.wb_min, a.wb_max, a.n_wide, a.wb_target
    b.exclude = {ci: {q[1] for q in v} for ci, v in short.items()}
    wide, info = make_plan_all(b, Path(a.run), kf, Tk, Kr, K, W0, H0, t_ns, Tv)
    return short, wide, info


def cmd_plan4(a):
    kf, Tk, Kr, W0, H0, t_ns, K, dist, Tv = load_common(a); out = Path(a.out)
    short, wide, info = wide_plan(a, kf, Tk, Kr, K, W0, H0, t_ns, Tv)
    c = Counter(len(v) for v in wide.values())
    res = dict(made=time.strftime("%F %T"), args=vars(a), kf_with_wide=len(wide), hist={int(k): int(v) for k, v in c.items()},
               n_wide_pairs=int(sum(c_ * v for c_, v in c.items())), wide_on_kf_without_short=len([ci for ci in wide if ci not in short]),
               src=dict(Counter(q[4] for v in wide.values() for q in v)), B_cm_median=float(np.median([q[2]["B"] * 100 for v in wide.values() for q in v])) if wide else 0,
               plan={int(kf[ci]): [[int(q[1]), q[4], round(q[2]["B"] * 100, 1), round(q[2]["perp_deg"], 1)] for q in v] for ci, v in wide.items()})
    (out / "plan_wide.json").write_text(json.dumps(res, indent=1, default=float))
    print(json.dumps({k: v for k, v in res.items() if k != "plan"}, default=float, indent=1))


def cmd_ffs4(a):
    import lib_ffs
    t0 = time.time()
    kf, Tk, Kr, W0, H0, t_ns, K, dist, Tv = load_common(a); out = Path(a.out)
    _, plan, _ = wide_plan(a, kf, Tk, Kr, K, W0, H0, t_ns, Tv)
    cfg = ms.MSConfig(); cfg.b_target_m = a.wb_target; cfg.z_max = a.zmax
    (out / "wide").mkdir(parents=True, exist_ok=True)
    rp = dict(size=(W0, H0), f=float(K[0, 0]), cx=float(K[0, 2]), cy=float(K[1, 2]), R1=np.eye(3).tolist())
    need = {}
    for ci, ch in plan.items():
        for q in ch:
            need.setdefault(q[1], []).append(ci)
        need.setdefault(int(kf[ci]), []).append(ci)
    print(f"wide plan: {len(plan)} kf, {sum(len(v) for v in plan.values())} pairs, {len(need)} frames", flush=True)
    model = lib_ffs.load(valid_iters=a.iters)
    ready = {ci: max([int(kf[ci])] + [q[1] for q in plan[ci]]) for ci in plan}
    last_use = {f: max(ready[ci] for ci in cis) for f, cis in need.items()}
    todo = sorted([ci for ci in plan if not (out / "wide" / f"{int(kf[ci]):05d}.npz").exists()], key=lambda ci: ready[ci])
    cap = cv2.VideoCapture(str(Path(a.rec) / "cam5.mp4")); buf = {}; fidx = -1; ti = 0; stats = {}
    while ti < len(todo):
        r_, f = cap.read(); fidx += 1
        if not r_:
            break
        if fidx in need:
            buf[fidx] = cv2.resize(f, (W0, H0), interpolation=cv2.INTER_AREA) if f.shape[1] != W0 else f
        while ti < len(todo) and ready[todo[ti]] <= fidx:
            ci = todo[ti]; ti += 1; fi = int(kf[ci]); res = []
            for s_, j, gg, Tj, _src in plan[ci]:
                R, t = ms.rel_pose(Tk[ci], Tj)
                # far pixels matter: z_near 1 m keeps the FFS input near full resolution (near pixels above d_cap drop out)
                res.append(ms.pair_depth(model, lib_ffs.infer, buf[fi], buf[j], R, t, K, dist, rp, cfg, z_near=a.wide_znear, iters=a.iters))
            np.savez_compressed(out / "wide" / f"{fi:05d}.npz", Z=np.stack([np.where(r["valid"], r["Z"], np.nan) for r in res]).astype(np.float16),
                                B=np.array([r["B"] for r in res]), f=np.array([r["f"] for r in res]), partner=np.array([q[1] for q in plan[ci]]),
                                src=np.array([q[4] for q in plan[ci]]), perp=np.array([r["perp_deg"] for r in res]), scale=np.array([r["scale"] for r in res]))
            stats[fi] = dict(pairs=len(res), valid=[round(float(r["valid"].mean()), 3) for r in res], B_cm=[round(r["B"] * 100, 1) for r in res],
                             scale=[r["scale"] for r in res])
            if ti % 25 == 0:
                print(f"{ti}/{len(todo)} kf {fi} t {time.time() - t0:.0f}s", flush=True)
        for fdel in [k for k in buf if last_use[k] < fidx]:
            del buf[fdel]
    cap.release()
    (out / "wide" / "ffs_info.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), args=vars(a), n_done=len(stats), total_s=round(time.time() - t0, 1),
                                                                per_kf=stats), indent=1, default=float))
    print(f"done {len(stats)} kf in {time.time() - t0:.0f}s", flush=True)


def cmd_fuse4(a):
    """s4m = s12 for pixels <= --z-split; beyond it the depth comes from the wide pairs when a wide pair is confirmed by (the other wide
    pair) or (a short pair, tolerance k*sqrt2*sigma_wide) or (a neighbouring keyframe's wide depth, k*hypot(sigma, sigma_nb)); fused =
    inverse-variance mean of the confirmed wide pairs. s12 pixels > z-split without a wide estimate are kept (short pairs, <= 2 m saved).
    Cap --zmax."""
    t0 = time.time()
    run, out = Path(a.run), Path(a.out); od = out / "s4m"; od.mkdir(parents=True, exist_ok=True)
    c = np.load(run / "dump_ba" / "cams.npz"); kf, T, Kr = c["kf"], c["T"], c["Kr"]; W, H = [int(v) for v in c["size"]]; n = len(kf)
    fx, fy, cx, cy = [float(v) for v in (Kr[0, 0], Kr[1, 1], Kr[0, 2], Kr[1, 2])]
    hw = [(out / "wide" / f"{int(i):05d}.npz").exists() for i in kf]
    WC = {}

    def wide_comp(ci):        # neighbour composite: inverse-variance mean of its valid wide pairs, + sigma
        if ci not in WC:
            d = np.load(out / "wide" / f"{int(kf[ci]):05d}.npz"); Z = d["Z"].astype(np.float32); sg = np.stack([Z[p] ** 2 / (d["f"][p] * d["B"][p]) for p in range(len(d["B"]))])
            w = np.where(np.isfinite(Z), 1 / np.nan_to_num(sg, nan=1) ** 2, 0); den = w.sum(0)
            with np.errstate(invalid="ignore", divide="ignore"):
                WC[ci] = (np.where(den > 0, (np.nan_to_num(Z) * w).sum(0) / den, np.nan).astype(np.float32), np.where(den > 0, 1 / np.sqrt(den), np.inf).astype(np.float32))
        return WC[ci]
    vv, uu = np.mgrid[0:H, 0:W]; info = {}; k = a.k
    for ci in range(n):
        f12 = out / "s12" / f"{int(kf[ci]):05d}.npz"
        Z12 = np.load(f12)["Z"] if f12.exists() else np.full((H, W), np.nan, np.float32)
        if not hw[ci]:
            if f12.exists():
                Zo = np.where(Z12 <= a.zmax, Z12, np.nan).astype(np.float32); V = np.isfinite(Zo)
                np.savez_compressed(od / f"{int(kf[ci]):05d}.npz", Z=Zo, V=V); info[int(kf[ci])] = dict(wide=0, valid_frac=float(V.mean()))
            continue
        d = np.load(out / "wide" / f"{int(kf[ci]):05d}.npz"); Zw = d["Z"].astype(np.float32); m = len(d["B"])
        sw = np.stack([Zw[p] ** 2 / (d["f"][p] * d["B"][p]) for p in range(m)]); vw = np.isfinite(Zw)
        fp = out / "pairs" / f"{int(kf[ci]):05d}.npz"
        if fp.exists():
            Zs = np.load(fp)["Z"].astype(np.float32)
        else:
            Zs = np.zeros((0, H, W), np.float32)
        conf = np.zeros((m, H, W), bool); why = Counter()
        for p in range(m):
            with np.errstate(invalid="ignore"):
                for q in range(m):
                    if q != p:
                        x = vw[p] & vw[q] & (np.abs(Zw[p] - Zw[q]) <= k * np.hypot(sw[p], sw[q])); conf[p] |= x; why["wide"] += int(x.sum())
                for q in range(len(Zs)):
                    x = vw[p] & np.isfinite(Zs[q]) & (np.abs(Zw[p] - Zs[q]) <= k * np.sqrt(2) * sw[p]); conf[p] |= x; why["short"] += int(x.sum())
            rest = vw[p] & ~conf[p]
            if rest.any():
                z = Zw[p][rest]; s0 = sw[p][rest]; u = uu[rest]; v = vv[rest]
                X = np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z], 1) @ T[ci, :3, :3].T + T[ci, :3, 3]
                nbs = [j for j in range(ci - 1, -1, -1) if hw[j]][:2] + [j for j in range(ci + 1, n) if hw[j]][:2]; okn = np.zeros(len(z), bool)
                for j in nbs:
                    Tw_ = np.linalg.inv(T[j]); Xj = X @ Tw_[:3, :3].T + Tw_[:3, 3]; zj = Xj[:, 2]
                    with np.errstate(invalid="ignore", divide="ignore"):
                        pu = np.round(fx * Xj[:, 0] / np.maximum(zj, 1e-3) + cx).astype(int); pv = np.round(fy * Xj[:, 1] / np.maximum(zj, 1e-3) + cy).astype(int)
                    ok = (zj > 0.05) & (pu >= 0) & (pu < W) & (pv >= 0) & (pv < H)
                    Zn, Sn = wide_comp(j); zr = np.full(len(z), np.nan, np.float32); sr = np.full(len(z), np.inf, np.float32)
                    zr[ok] = Zn[pv[ok], pu[ok]]; sr[ok] = Sn[pv[ok], pu[ok]]
                    with np.errstate(invalid="ignore"):
                        okn |= np.isfinite(zr) & (np.abs(zj - zr) <= k * np.hypot(s0, sr))
                cp = conf[p].copy(); cp[rest] = okn; conf[p] = cp; why["nb"] += int(okn.sum())
        w = np.where(conf, 1 / np.maximum(np.nan_to_num(sw, nan=1), 1e-6) ** 2, 0); den = w.sum(0)
        with np.errstate(invalid="ignore", divide="ignore"):
            Zf = np.where(den > 0, (np.nan_to_num(Zw) * w).sum(0) / den, np.nan)
        if m >= 2:      # two confirmed wide pairs that disagree with each other -> drop
            both = conf[0] & conf[1]
            with np.errstate(invalid="ignore"):
                Zf[both & (np.abs(Zw[0] - Zw[1]) > k * np.hypot(sw[0], sw[1]))] = np.nan
        far = np.isfinite(Zf) & (Zf > a.z_split) & (Zf <= a.zmax)
        Zo = np.where(np.isfinite(Z12) & (Z12 <= a.z_split), Z12, np.nan)
        Zo = np.where(far & ~(np.isfinite(Z12) & (Z12 <= a.z_split)), Zf, Zo)
        keep12 = np.isfinite(Z12) & (Z12 > a.z_split) & ~far
        Zo = np.where(keep12, Z12, Zo).astype(np.float32); Zo[Zo > a.zmax] = np.nan; V = np.isfinite(Zo)
        np.savez_compressed(od / f"{int(kf[ci]):05d}.npz", Z=Zo, V=V)
        info[int(kf[ci])] = dict(wide=m, valid_frac=float(V.mean()), px_wide_valid=int(vw.any(0).sum()), px_wide_conf=int(conf.any(0).sum()),
                                 px_far_from_wide=int((far & ~(np.isfinite(Z12) & (Z12 <= a.z_split))).sum()), px_s12_far_kept=int(keep12.sum()), conf_by=dict(why))
        if ci % 50 == 0:
            print(ci, info[int(kf[ci])], f"{time.time() - t0:.0f}s", flush=True)
    (od / "fuse_info.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), args=vars(a), per_kf=info), indent=1))
    print(f"fuse4 done {len(info)} kf {time.time() - t0:.0f}s", flush=True)

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["plan", "ffs", "fuse", "plan4", "ffs4", "fuse4"]); ap.add_argument("run")
    ap.add_argument("--vio", required=True); ap.add_argument("--rec", required=True); ap.add_argument("--gt", default="lensonly")
    ap.add_argument("--out", required=True); ap.add_argument("--win-s", type=float, default=2.0); ap.add_argument("--n-pairs", type=int, default=4)
    ap.add_argument("--b-min", type=float, default=0.03); ap.add_argument("--b-max", type=float, default=0.25)
    ap.add_argument("--min-perp", type=float, default=40.0); ap.add_argument("--max-rot", type=float, default=25.0)
    ap.add_argument("--dir-deg", type=float, default=10.0); ap.add_argument("--fb-perp", type=float, default=0.0); ap.add_argument("--zmax", type=float, default=2.0)
    ap.add_argument("--iters", type=int, default=8); ap.add_argument("--k", type=float, default=2.0)
    ap.add_argument("--n-wide", type=int, default=2); ap.add_argument("--wb-min", type=float, default=0.15); ap.add_argument("--wb-max", type=float, default=0.35)
    ap.add_argument("--wb-target", type=float, default=0.25); ap.add_argument("--wide-znear", type=float, default=1.0); ap.add_argument("--z-split", type=float, default=1.5)
    a = ap.parse_args()
    if a.cmd in ("plan4", "fuse4"):
        {"plan4": cmd_plan4, "fuse4": cmd_fuse4}[a.cmd](a)
    elif a.cmd == "ffs4":
        import gpu_guard
        with gpu_guard.job("gsmap ffs_pairs2 wide " + a.out, torch_cap=True):
            cmd_ffs4(a)
    elif a.cmd == "plan":
        cmd_plan(a)
    elif a.cmd == "fuse":
        cmd_fuse(a)
    else:
        import gpu_guard
        with gpu_guard.job("gsmap ffs_pairs2 " + a.out, torch_cap=True):
            cmd_ffs(a)
