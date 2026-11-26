#!/usr/bin/env python3
"""gsmap 2026-11-26: bundle adjustment WITH the FFS partner frames on 121801, then the adopted FFS plans re-run with the new BA poses for
keyframe AND partner (audit4: VIO-vs-BA baseline scale mismatch = main cause of the FFS multi-view mismatch).

  frames  (CPU)  OUT/manifest.json  keyframes = 494 old keyframes + every partner frame of ffs_pairs2/plan.json + ffs_pairs3/plan3.json
                 OUT/extra_pairs.txt = old keyframe-BA pairs (the original keyframe graph) + every keyframe-partner FFS pair
  ba      (GPU)  StereoDepthSeg-fixes scanA/kf_ba.py with the original settings (ba.json of the 11-12 run) + --kf-list + --extra-pairs;
                 SIFT / matcher CPU threads capped at 8; final BA state dumped to OUT/ba/ba_state.npz
  post    (CPU)  reprojection keyframes vs partners, Sim3 of the new keyframe centres onto the old map frame (dump_ba/cams.npz)
                 (kf_ba has no pose prior), partner movement vs the pose the old FFS used -> OUT/post.json,
                 OUT/poses_all.npz (every frame, map frame), OUT/run/dump_ba/cams.npz (new keyframe T, the judge of measure)
  ffs2    (GPU)  pairs2 plan (strict + fallback), same partner frames, new poses -> OUT/ffs_pairs2/{pairs,s1}
  ffs3    (GPU)  pairs3 low tier, same partner frames, new poses -> OUT/ffs_pairs3/extra
  fuse    (CPU)  gsm_ffs_pairs2 fuse + gsm_ffs_pairs3 fuse (variant L), new keyframe poses
  measure (GPU)  gsm_ffs_pairs3 measure (measure3.json rule) with the new poses as judge; control = old depths judged with new poses
Plans are rebuilt with the old inputs and asserted equal to the stored plan.json / plan3.json, then only the poses are swapped.
"""
import argparse, gzip, json, os, shutil, sys, time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

HOME = Path.home()
REPO = HOME / "scanner" / "StereoDepthSeg-gsmap"
R1 = REPO / "scan3d/runs/gsmap_20261125_121801"
V2 = Path(str(R1) + "_v2")
OUT = V2 / "ba_partners"
VIO = HOME / "scanner/StereoDepthSeg-fixes/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan/vio/B_o0.poses.jsonl.gz"
OLD_BA = HOME / "scanner/StereoDepthSeg-fixes/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan/ba"
FIXES = HOME / "scanner/StereoDepthSeg-fixes/scan3d/scanA"
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / "gsplat"))
if os.environ.get("FFS_ROOT"):
    sys.path.insert(0, os.environ["FFS_ROOT"])


def ffs_args(run=R1, out=None, p2=None, **kw):
    """= run_ffs_pairs2.sh / run_ffs_pairs3.sh arguments (+ argparse defaults of both scripts)."""
    a = dict(run=str(run), vio=str(VIO), rec=str(R1 / "rec"), gt="lensonly", out=str(out), p2=str(p2) if p2 else "",
             win_s=2.0, n_pairs=4, b_min=0.03, b_max=0.25, min_perp=40.0, max_rot=25.0, dir_deg=10.0, fb_perp=30.0, zmax=2.0,
             iters=8, k=2.0, low_perp=10.0, theta_min=25.0, cap=0.03, f_scale=1.0, max_px=1.0e6, z_near=0.4, gsmap_run="", fig="", fig_kf="762")
    a.update(kw)
    return SimpleNamespace(**a)


def umeyama(A, B, with_scale=True):
    ma, mb = A.mean(0), B.mean(0)
    U, S, Vt = np.linalg.svd((B - mb).T @ (A - ma))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    s = float(np.trace(np.diag(S) @ D) / ((A - ma) ** 2).sum()) if with_scale else 1.0
    return s, R, mb - s * R @ ma


def rot_deg(R):
    return np.degrees(np.arccos(np.clip((np.trace(R, axis1=-2, axis2=-1) - 1) / 2, -1, 1)))


def load_vio_frames(t_ns):
    vt, vT = [], []
    for line in gzip.open(VIO, "rt"):
        d = json.loads(line)
        if d.get("type") == "pose" and d.get("T_world_cam0") and d.get("status") == "tracking":
            vt.append(int(d["t_ns"])); vT.append(np.array(d["T_world_cam0"], float).reshape(4, 4))
    vt = np.array(vt); vT = np.array(vT)
    idx = np.searchsorted(t_ns, vt).clip(1, len(t_ns) - 1); idx = np.where(np.abs(t_ns[idx - 1] - vt) < np.abs(t_ns[idx] - vt), idx - 1, idx)
    good = np.abs(t_ns[idx] - vt) < 8e6
    return {int(i): T for i, T in zip(idx[good], vT[good])}


# ------------------------------------------------------------------------------------------------------------------- frames
def cmd_frames(a):
    OUT.mkdir(parents=True, exist_ok=True)
    c = np.load(R1 / "dump_ba" / "cams.npz"); kf = [int(x) for x in c["kf"]]
    p2 = json.loads((V2 / "ffs_pairs2" / "plan.json").read_text())["chosen"]["plan"]
    p3 = json.loads((V2 / "ffs_pairs3" / "plan3.json").read_text())["plan"]
    pairs, parts = set(), {"p2": set(), "p3": set()}
    for nm, src in (("p2", p2), ("p3", p3)):
        for k, v in src.items():
            for q in v:
                j = int(q[0]); parts[nm].add(j); pairs.add((min(int(k), j), max(int(k), j)))
    allp = parts["p2"] | parts["p3"]; allf = sorted(set(kf) | allp)
    old = [l.split() for l in (OLD_BA / "pairs.txt").read_text().splitlines() if l.strip()]
    oldp = {(int(x[:-4]), int(y[:-4])) for x, y in old}
    ex = sorted(oldp | pairs)
    (OUT / "extra_pairs.txt").write_text("".join(f"{x:05d}.jpg {y:05d}.jpg\n" for x, y in ex))
    info = dict(made=time.strftime("%F %T"), inputs=dict(cams=str(R1 / "dump_ba" / "cams.npz"), plan2=str(V2 / "ffs_pairs2" / "plan.json"),
                                                         plan3=str(V2 / "ffs_pairs3" / "plan3.json"), old_pairs=str(OLD_BA / "pairs.txt")),
                n_kf=len(kf), n_partner_frames=len(allp), n_partner_nonkf=len(allp - set(kf)), n_partner_nonkf_p2=len(parts["p2"] - set(kf)),
                n_partner_nonkf_p3=len(parts["p3"] - set(kf)), n_frames=len(allf), n_old_kf_pairs=len(oldp), n_ffs_pairs=len(pairs),
                n_extra_pairs=len(ex), keyframes=allf, old_kf=kf)
    (OUT / "manifest.json").write_text(json.dumps(info, indent=1))
    print(json.dumps({k: v for k, v in info.items() if k not in ("keyframes", "old_kf")}, indent=1))


# ------------------------------------------------------------------------------------------------------------------- ba
def cmd_ba(a):
    sys.path.insert(0, str(FIXES))
    import pycolmap
    for nm in ("FeatureExtractionOptions", "FeatureMatchingOptions"):     # power rule: <= 8 CPU threads
        orig = getattr(pycolmap, nm)

        def mk(orig=orig):
            o = orig(); o.num_threads = 8; return o
        setattr(pycolmap, nm, mk)
    import ba_torch, kf_ba
    out = OUT / "ba"; out.mkdir(parents=True, exist_ok=True)
    orig_ba = ba_torch.ba_lm_torch

    def ba_dump(R, t, X, cam, pt, uv, K, **kw):
        R1_, t1_, X1_, st = orig_ba(R, t, X, cam, pt, uv, K, **kw)
        np.savez_compressed(out / "ba_state.npz", R0=np.asarray(R), t0=np.asarray(t), R1=np.asarray(R1_), t1=np.asarray(t1_), X1=np.asarray(X1_),
                            cam=cam, pt=pt, uv=uv, K=K, fixed=np.array(sorted(kw.get("fixed", [])), int))
        return R1_, t1_, X1_, st
    ba_torch.ba_lm_torch = ba_dump
    # original settings = OLD_BA/ba.json args (feat sift 6000, seq 6, cov 10/30, iters 40, kf-from-vio, match-ms 1, seed 0, drop, cuda BA, cpu SIFT)
    sys.argv = ["kf_ba.py", str(R1 / "rec"), "--vio", str(VIO), "--out", str(out), "--gt", "lensonly", "--kf-step", "10", "--scale", "0.5",
                "--seq", "6", "--cov-k", "10", "--cov-angle", "30", "--max-features", "6000", "--iters", "40", "--min-tri-deg", "1.0",
                "--kf-from-vio", "--match-ms", "1.0", "--feat", "sift", "--loop", "none", "--seed", "0", "--tracks", "drop",
                "--ba-device", "cuda", "--match-device", "auto", "--sift-device", "cpu", "--keep-db",
                "--cache-root", str(OUT / "cache"), "--kf-list", str(OUT / "manifest.json"), "--extra-pairs", str(OUT / "extra_pairs.txt")]
    try:
        kf_ba.main()
    except BaseException as e:
        if kf_ba._GJ[0] is not None:
            kf_ba._GJ[0].__exit__(type(e), e, e.__traceback__)
        raise


# ------------------------------------------------------------------------------------------------------------------- post
def old_pair_poses():
    """the exact partner poses the adopted FFS used: pairs2 plan (strict + fallback) and pairs3 low tier, rebuilt with the old inputs."""
    import gsm_ffs_pairs2 as P2
    import gsm_ffs_pairs3 as P3
    A3 = ffs_args(out=V2 / "ffs_pairs3", p2=V2 / "ffs_pairs2")
    b = P3.build(A3)
    assert b["same"], "rebuilt pairs2 plan != ffs_pairs2/plan.json"
    p3 = json.loads((V2 / "ffs_pairs3" / "plan3.json").read_text())["plan"]
    mine3 = {str(int(b["kf"][c])): [int(q[1]) for q in v] for c, v in b["low"].items()}
    assert mine3 == {k: [q[0] for q in v] for k, v in p3.items()}, "rebuilt low tier != plan3.json"
    return b


def cmd_post(a):
    import motion_stereo as ms
    man = json.loads((OUT / "manifest.json").read_text()); kfs = sorted(man["keyframes"])
    st = np.load(OUT / "ba" / "ba_state.npz"); bj = json.loads((OUT / "ba" / "ba.json").read_text())
    c = dict(np.load(R1 / "dump_ba" / "cams.npz")); kf0 = [int(x) for x in c["kf"]]; T0 = c["T"]; t_ns = c["t_ns"] if "t_ns" in c else None
    g = np.load(R1 / "rec" / "lensonly" / "gt.npz"); t_ns = g["t_ns"]
    Tv = load_vio_frames(t_ns)
    iskf = np.array([i in set(kf0) for i in kfs])
    # ---- reprojection (final BA state)
    R, t, X, cam, pt, uv, K = st["R1"], st["t1"], st["X1"], st["cam"], st["pt"], st["uv"], st["K"]
    Xc = np.einsum("nij,nj->ni", R[cam], X[pt]) + t[cam]
    q = np.stack([K[0, 0] * Xc[:, 0] / Xc[:, 2] + K[0, 2], K[1, 1] * Xc[:, 1] / Xc[:, 2] + K[1, 2]], 1)
    e = np.linalg.norm(q - uv, axis=1); ok = iskf[cam]
    nobs = np.bincount(cam, minlength=len(kfs))
    fixed = set(int(x) for x in st["fixed"]); gauge = int(st["fixed"][0]) if len(st["fixed"]) else -1
    free = np.array([(k not in fixed) or k == gauge for k in range(len(kfs))])
    # fixed[0] is the gauge (argmax obs of the first 5): kf_ba stores fixed unsorted -> take it from ba.json
    gauge_f = bj["fixed_kf"][0]; free = np.array([(k not in fixed) or kfs[k] == gauge_f for k in range(len(kfs))])

    def rs(m):
        return dict(n_cams=int(np.unique(cam[m]).size), n_obs=int(m.sum()), med_px=round(float(np.median(e[m])), 3),
                    rmse_px=round(float(np.sqrt((e[m] ** 2).mean())), 3), frac_le_1px=round(float((e[m] <= 1).mean()), 4),
                    obs_per_cam_median=float(np.median(nobs[np.unique(cam[m])])))
    reproj = dict(all=rs(np.ones(len(e), bool)), keyframes=rs(ok), partners=rs(~ok),
                  n_kf_no_ba=int((iskf & ~free).sum()), n_partner_no_ba=int((~iskf & ~free).sum()))
    # ---- poses: BA (cam_from_world) -> map frame by Sim3 of the new keyframe centres onto the old keyframe centres (cams.npz T)
    Cn = np.array([-R[k].T @ t[k] for k in range(len(kfs))]); Rwc = np.transpose(R, (0, 2, 1))
    pos = {f: k for k, f in enumerate(kfs)}
    kk = np.array([pos[f] for f in kf0]); m = free[kk]
    s, Ra, ta = umeyama(Cn[kk][m], T0[:, :3, 3][m])
    Tm = np.zeros((len(kfs), 4, 4)); Tm[:, 3, 3] = 1
    Tm[:, :3, :3] = np.einsum("ij,njk->nik", Ra, Rwc); Tm[:, :3, 3] = s * Cn @ Ra.T + ta
    res = np.linalg.norm(Tm[kk, :3, 3] - T0[:, :3, 3], axis=1); rres = rot_deg(np.einsum("nji,njk->nik", Tm[kk, :3, :3], T0[:, :3, :3]))
    src = np.array(["ba"] * len(kfs), dtype=object)
    near_kf = lambda j: kf0[int(np.argmin(np.abs(np.array(kf0) - j)))]
    for k in np.flatnonzero(~free):           # < 15 obs: keyframe -> old BA pose, partner -> VIO anchored on the nearest keyframe (old rule)
        f = kfs[k]
        if f in set(kf0):
            Tm[k] = T0[kf0.index(f)]; src[k] = "old_ba"
        else:
            n_ = near_kf(f); Tm[k] = Tm[pos[n_]] @ np.linalg.inv(Tv[n_]) @ Tv[f]; src[k] = "vio_anchor"
    align = dict(sim3_scale=round(s, 5), n_kf_used=int(m.sum()), centre_rms_cm=round(float(np.sqrt((res[m] ** 2).mean()) * 100), 3),
                 centre_med_cm=round(float(np.median(res[m]) * 100), 3), centre_p95_cm=round(float(np.percentile(res[m], 95) * 100), 3),
                 rot_med_deg=round(float(np.median(rres[m])), 3), rot_p95_deg=round(float(np.percentile(rres[m], 95)), 3))
    np.savez_compressed(OUT / "poses_all.npz", frames=np.array(kfs), T=Tm, src=src.astype(str), iskf=iskf, nobs=nobs,
                        note="map frame = gsmap_20261125_121801/dump_ba/cams.npz (Sim3 of new BA keyframe centres onto old)")
    TmD = {f: Tm[k] for k, f in enumerate(kfs)}
    # ---- the judge / fusion run dir (new keyframe T, everything else = old cams.npz)
    rd = OUT / "run" / "dump_ba"; rd.mkdir(parents=True, exist_ok=True)
    c2 = dict(c); c2["T"] = np.stack([TmD[f] for f in kf0]); c2["made"] = np.array(time.strftime("%F %T"))
    c2["poses"] = np.array(json.dumps(dict(source=str(OUT / "ba" / "poses_ba.jsonl.gz"), map_frame="Sim3 onto gsmap_20261125_121801 dump_ba/cams.npz T", **align)))
    np.savez(rd / "cams.npz", **c2)
    for lk, tg in ((rd / "img", V2 / "dump_ba" / "img"), (OUT / "run" / "eval2" / "ref" / "gold_ffs.npz", V2 / "eval2" / "ref" / "gold_ffs.npz")):
        lk.parent.mkdir(parents=True, exist_ok=True)
        if not lk.exists():
            os.symlink(tg, lk)
    # ---- partner movement: pose the old FFS used vs new BA pose, per FFS pair (relative to the keyframe, kf frame)
    b = old_pair_poses(); kfa = b["kf"]; Tk0 = b["Tk"]
    rows = []
    for tier, plan in (("p2", b["base"]), ("low", b["low"])):
        for ci, ch in plan.items():
            fi = int(kfa[ci])
            for qq in ch:
                j, Tj_old = int(qq[1]), qq[3]
                Ro, to = ms.rel_pose(Tk0[ci], Tj_old); Rn, tn = ms.rel_pose(TmD[fi], TmD[j])
                co, cn = -Ro.T @ to, -Rn.T @ tn
                rows.append(dict(tier=tier, kf=fi, j=j, src=(qq[4] if len(qq) > 4 else "low"), j_src=str(src[pos[j]]),
                                 B_old=float(np.linalg.norm(co)), B_new=float(np.linalg.norm(cn)), rot_deg=float(rot_deg(Rn @ Ro.T)),
                                 dc_cm=float(np.linalg.norm(cn - co) * 100),
                                 dir_deg=float(np.degrees(np.arccos(np.clip(co @ cn / np.linalg.norm(co) / np.linalg.norm(cn), -1, 1)))),
                                 abs_rot_deg=float(rot_deg(TmD[j][:3, :3].T @ Tj_old[:3, :3])), abs_dc_cm=float(np.linalg.norm(TmD[j][:3, 3] - Tj_old[:3, 3]) * 100),
                                 kf_dc_cm=float(np.linalg.norm(TmD[fi][:3, 3] - Tk0[ci][:3, 3]) * 100)))

    def summ(rr):
        if not rr:
            return {}
        ratio = np.array([r["B_new"] / r["B_old"] for r in rr]); f = lambda k: np.array([r[k] for r in rr])
        pc = lambda x: [round(float(v), 4) for v in np.percentile(x, [5, 50, 95])]
        return dict(n=len(rr), B_ratio_p5_50_95=pc(ratio), B_ratio_absdev_med=round(float(np.median(np.abs(ratio - 1))), 4),
                    frac_B_ratio_off_gt_5pct=round(float(np.mean(np.abs(ratio - 1) > 0.05)), 4), rel_rot_deg_p5_50_95=pc(f("rot_deg")),
                    rel_centre_dc_cm_p5_50_95=pc(f("dc_cm")), baseline_dir_deg_p5_50_95=pc(f("dir_deg")),
                    abs_partner_rot_deg_p5_50_95=pc(f("abs_rot_deg")), abs_partner_move_cm_p5_50_95=pc(f("abs_dc_cm")))
    mv = dict(all=summ(rows), p2_local=summ([r for r in rows if r["src"] == "local"]), p2_fallback=summ([r for r in rows if r["src"] == "fallback"]),
              low=summ([r for r in rows if r["tier"] == "low"]), partner_is_kf=summ([r for r in rows if r["j"] in set(kf0)]),
              partner_nonkf=summ([r for r in rows if r["j"] not in set(kf0)]))
    with gzip.open(OUT / "pair_motion.jsonl.gz", "wt") as f:
        for r in rows:
            f.write(json.dumps(r) + "\n")
    info = dict(made=time.strftime("%F %T"), inputs=dict(ba_state=str(OUT / "ba" / "ba_state.npz"), ba_json=str(OUT / "ba" / "ba.json"),
                                                         old_cams=str(R1 / "dump_ba" / "cams.npz"), vio=str(VIO)),
                ba=dict(runtime_s=bj["runtime_s"], timing=bj.get("timing"), n_frames=bj["n_kf"], pairs=bj["pairs"], verified_pairs=bj["verified_pairs"],
                        points=bj["points"], obs=bj["obs"], n_vio_pose=bj.get("n_kf_vio_pose"), sim3_vio_scale=bj["sim3_scale"], sim3_vio_rms_cm=bj["sim3_rms_cm"],
                        final_med_px=bj["final_med_px"], final_rmse_px=bj["final_rmse_px"], pair_stats=bj.get("pair_stats")),
                registered=dict(keyframes=int((iskf & free).sum()), keyframes_total=int(iskf.sum()), partners=int((~iskf & free).sum()), partners_total=int((~iskf).sum())),
                reproj=reproj, align_to_old_map=align, pair_motion=mv,
                note="pair motion = pose the adopted FFS used (VIO relative motion anchored on the old keyframe BA pose; 'ba' partners = old BA) "
                     "vs new BA pose, both in the old map frame; B_ratio = new / old keyframe-partner baseline")
    (OUT / "post.json").write_text(json.dumps(info, indent=1, default=float))
    print(json.dumps({k: v for k, v in info.items() if k != "inputs"}, indent=1, default=float))


# ------------------------------------------------------------------------------------------------------------------- ffs
def new_poses():
    P = np.load(OUT / "poses_all.npz"); TmD = {int(f): T for f, T in zip(P["frames"], P["T"])}
    return TmD


def cmd_ffs2(a):
    import cv2, lib_ffs
    import gsm_ffs_pairs2 as P2, motion_stereo as ms
    t0 = time.time(); out = OUT / "ffs_pairs2"; (out / "pairs").mkdir(parents=True, exist_ok=True); (out / "s1").mkdir(exist_ok=True)
    A = ffs_args(out=out)
    kf, Tk0, Kr, W0, H0, t_ns, K, dist, Tv = P2.load_common(A)
    plan, cfg, _ = P2.new_plan(A, kf, Tk0, K, W0, H0, t_ns, Tv, Kr); cfg.z_max = A.zmax
    ref = json.loads((V2 / "ffs_pairs2" / "plan.json").read_text())["chosen"]["plan"]
    mine = {str(int(kf[c])): [[int(q[1]), q[4], round(q[2]["B"] * 100, 1), round(q[2]["perp_deg"], 1)] for q in v] for c, v in plan.items()}
    assert mine == ref, "rebuilt pairs2 plan != plan.json"
    shutil.copy(V2 / "ffs_pairs2" / "plan.json", out / "plan.json")
    TmD = new_poses(); Tk = np.stack([TmD[int(f)] for f in kf])
    rp = dict(size=(W0, H0), f=float(K[0, 0]), cx=float(K[0, 2]), cy=float(K[1, 2]), R1=np.eye(3).tolist())
    need = {}
    for ci, ch in plan.items():
        for q in ch:
            need.setdefault(q[1], []).append(ci)
        need.setdefault(int(kf[ci]), []).append(ci)
    print(f"plan: {len(plan)}/{len(kf)} kf, {sum(len(v) for v in plan.values())} pairs, {len(need)} frames, new BA poses", flush=True)
    model = lib_ffs.load(valid_iters=A.iters)
    ready = {ci: max([int(kf[ci])] + [q[1] for q in plan[ci]]) for ci in plan}
    last_use = {f: max(ready[ci] for ci in cis) for f, cis in need.items()}
    todo = sorted([ci for ci in plan if not (out / "s1" / f"{int(kf[ci]):05d}.npz").exists()], key=lambda ci: ready[ci])
    print(f"todo {len(todo)}", flush=True)
    cap = cv2.VideoCapture(str(Path(A.rec) / "cam5.mp4")); buf = {}; fidx = -1; ti = 0; stats = {}
    while ti < len(todo):
        r_, f = cap.read(); fidx += 1
        if not r_:
            break
        if fidx in need:
            buf[fidx] = cv2.resize(f, (W0, H0), interpolation=cv2.INTER_AREA) if f.shape[1] != W0 else f
        while ti < len(todo) and ready[todo[ti]] <= fidx:
            ci = todo[ti]; ti += 1; fi = int(kf[ci]); res = []
            for s, j, gg, Tj_old, _src in plan[ci]:
                R, t = ms.rel_pose(Tk[ci], TmD[j])
                res.append(ms.pair_depth(model, lib_ffs.infer, buf[fi], buf[j], R, t, K, dist, rp, cfg, iters=A.iters))
            Z, V, S, agree = ms.combine_motion(res, cfg)
            np.savez_compressed(out / "pairs" / f"{fi:05d}.npz", Z=np.stack([np.where(r["valid"], r["Z"], np.nan) for r in res]).astype(np.float16),
                                B=np.array([r["B"] for r in res]), f=np.array([r["f"] for r in res]), partner=np.array([q[1] for q in plan[ci]]),
                                src=np.array([q[4] for q in plan[ci]]), perp=np.array([r["perp_deg"] for r in res]),
                                B_old=np.array([q[2]["B"] for q in plan[ci]]))
            np.savez_compressed(out / "s1" / f"{fi:05d}.npz", Z=np.where(V, Z, np.nan).astype(np.float32), V=V, S=np.where(V, S, np.inf).astype(np.float32))
            stats[fi] = dict(pairs=len(res), valid_frac=float(V.mean()), B_cm=[round(r["B"] * 100, 1) for r in res], B_old_cm=[round(q[2]["B"] * 100, 1) for q in plan[ci]])
            if ti % 25 == 0:
                print(f"{ti}/{len(todo)} kf {fi} pairs {len(res)} valid {V.mean():.3f} t {time.time() - t0:.0f}s", flush=True)
        for fdel in [k for k in buf if last_use[k] < fidx]:
            del buf[fdel]
    cap.release()
    (out / "s1" / "ffs_info.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), args=vars(A), poses=str(OUT / "poses_all.npz"), fallback_kf=cfg.fallback_kf,
                                                              n_done=len(stats), total_s=round(time.time() - t0, 1), per_kf=stats), indent=1, default=float))
    print(f"done {len(stats)} kf in {time.time() - t0:.0f}s", flush=True)


def cmd_ffs3(a):
    import cv2, lib_ffs
    import gsm_ffs_pairs3 as P3, motion_stereo as ms
    t0 = time.time(); out = OUT / "ffs_pairs3"; (out / "extra").mkdir(parents=True, exist_ok=True)
    A = ffs_args(out=out, p2=V2 / "ffs_pairs2")
    b = old_pair_poses(); kf = b["kf"]
    shutil.copy(V2 / "ffs_pairs3" / "plan3.json", out / "plan3.json")
    TmD = new_poses(); Tk = np.stack([TmD[int(f)] for f in kf])
    plan = {ci: [(q, "low") for q in b["low"][ci]] for ci in b["low"]}
    need = {}
    for ci, ch in plan.items():
        for q, _ in ch:
            need.setdefault(q[1], []).append(ci)
        need.setdefault(int(kf[ci]), []).append(ci)
    ready = {ci: max([int(kf[ci])] + [q[1] for q, _ in plan[ci]]) for ci in plan}
    last_use = {f_: max(ready[ci] for ci in cis) for f_, cis in need.items()}
    todo = sorted([ci for ci in plan if not (out / "extra" / f"{int(kf[ci]):05d}.npz").exists()], key=lambda ci: ready[ci])
    print(f"plan {len(plan)} kf {sum(len(v) for v in plan.values())} pairs todo {len(todo)}, new BA poses", flush=True)
    model = lib_ffs.load(valid_iters=A.iters)
    cap = cv2.VideoCapture(str(Path(A.rec) / "cam5.mp4")); buf = {}; fidx = -1; ti = 0; stats = {}
    W0, H0 = b["W0"], b["H0"]
    while ti < len(todo):
        r_, fr = cap.read(); fidx += 1
        if not r_:
            break
        if fidx in need:
            buf[fidx] = cv2.resize(fr, (W0, H0), interpolation=cv2.INTER_AREA) if fr.shape[1] != W0 else fr
        while ti < len(todo) and ready[todo[ti]] <= fidx:
            ci = todo[ti]; ti += 1; fi = int(kf[ci]); Zs, Ss, Bs, inf = [], [], [], []
            for q, tag in plan[ci]:
                R, t = ms.rel_pose(Tk[ci], TmD[q[1]])
                Z, st, B, f0, info = P3.partial_depth(model, lib_ffs.infer, buf[fi], buf[q[1]], R, t, b["K"], b["dist"], W0, H0, A, A.iters)
                Zs.append(Z); Ss.append(st); Bs.append(B); inf.append(dict(tag=tag, j=q[1], valid=round(float(np.isfinite(Z).mean()), 4), B_cm=round(B * 100, 2),
                                                                         B_old_cm=round(q[2]["B"] * 100, 2), sec=info))
            np.savez_compressed(out / "extra" / f"{fi:05d}.npz", Z=np.stack(Zs).astype(np.float16), sin=np.stack(Ss).astype(np.float16), B=np.array(Bs),
                                f=np.full(len(Bs), float(b["K"][0, 0])), partner=np.array([q[1] for q, _ in plan[ci]]), tag=np.array([t_ for _, t_ in plan[ci]]),
                                perp=np.array([q[2]["perp_deg"] for q, _ in plan[ci]]))
            stats[fi] = inf
            if ti % 20 == 0:
                print(f"{ti}/{len(todo)} kf {fi} {[x['valid'] for x in inf]} t {time.time() - t0:.0f}s", flush=True)
        for fdel in [k for k in buf if last_use[k] < fidx]:
            del buf[fdel]
    cap.release()
    (out / "extra" / "ffs_info.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), args=vars(A), poses=str(OUT / "poses_all.npz"),
                                                                 total_s=round(time.time() - t0, 1), per_kf=stats), indent=1, default=float))
    print(f"done {len(stats)} kf {time.time() - t0:.0f}s", flush=True)


def cmd_fuse(a):
    import gsm_ffs_pairs2 as P2, gsm_ffs_pairs3 as P3
    P2.cmd_fuse(ffs_args(run=OUT / "run", out=OUT / "ffs_pairs2"))
    P3.cmd_fuse(ffs_args(run=OUT / "run", out=OUT / "ffs_pairs3", p2=OUT / "ffs_pairs2"))


def cmd_measure(a):
    import gsm_ffs_pairs3 as P3
    # main: new depths, new judge
    P3.cmd_measure(ffs_args(run=OUT / "run", out=OUT / "ffs_pairs3", p2=OUT / "ffs_pairs2", gsmap_run=str(OUT / "run"),
                            fig=str(OUT / "ffs_lowangle_cmp.jpg"), fig_kf="762"))
    # control: OLD depths (adopted s12 / s12_L) judged with the NEW poses -> separates the judge change from the depth change
    c2, c3 = OUT / "ctrl_olddepth" / "p2", OUT / "ctrl_olddepth" / "p3"
    for d_, files, links in ((c2, [V2 / "ffs_pairs2" / "plan.json"], [("s12", V2 / "ffs_pairs2" / "s12")]),
                             (c3, [V2 / "ffs_pairs3" / "plan3.json"], [("s12_L", V2 / "ffs_pairs3" / "s12_L")])):
        d_.mkdir(parents=True, exist_ok=True)
        for f in files:
            shutil.copy(f, d_ / f.name)
        for nm, tg in links:
            if not (d_ / nm).exists():
                os.symlink(tg, d_ / nm)
    P3.cmd_measure(ffs_args(run=OUT / "run", out=c3, p2=c2, gsmap_run=str(OUT / "run"), fig="", fig_kf="762"))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["frames", "ba", "post", "ffs2", "ffs3", "fuse", "measure"])
    a = ap.parse_args()
    if a.cmd in ("ffs2", "ffs3", "measure"):
        import gpu_guard
        with gpu_guard.job("gsmap ba_partners " + a.cmd, torch_cap=True):
            {"ffs2": cmd_ffs2, "ffs3": cmd_ffs3, "measure": cmd_measure}[a.cmd](a)
    else:
        {"frames": cmd_frames, "ba": cmd_ba, "post": cmd_post, "fuse": cmd_fuse}[a.cmd](a)
