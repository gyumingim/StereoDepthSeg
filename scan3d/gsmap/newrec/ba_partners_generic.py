#!/usr/bin/env python3
"""Generic BA WITH the FFS partner frames for a new recording (night 11-27). Copy of the gsm_ba_partners.py idea (121801-hardwired)
with every path from arguments. Steps (one heavy job at a time; SIFT/matcher threads capped at 8):

  frames (CPU): OUT/manifest.json = keyframes of RUN/dump_ba/cams.npz + every partner frame of PLAN.json; OUT/extra_pairs.txt = kf-partner pairs
  ba     (GPU): StereoDepthSeg-fixes scanA/kf_ba.py, the 121801 BA settings (sift 6000, seq 6, cov 10/30, iters 40, kf-from-vio, match 1 ms,
                seed 0, tracks drop, cuda BA) + --kf-list manifest + --extra-pairs; final BA state -> OUT/ba/ba_state.npz
  post   (CPU): BA poses -> VIO world frame by Sim3 of all well-observed frames (>= 15 obs) onto their VIO centres (no old map exists);
                frames with < 15 obs: VIO relative motion anchored on the nearest BA frame. -> OUT/poses_all.npz, OUT/run/dump_ba/cams.npz, OUT/post.json
  ffs    (GPU): the pairs2 plan rebuilt from RUN (same partners), FFS with BA poses for keyframe AND partner -> OUT/ffs/{pairs,s1}

  python ba_partners_generic.py STEP --run RUN --plan PLAN.json --rec REC --vio VIO --out OUT [--profile new]
"""
import argparse, gzip, json, sys, time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

HOME = Path.home(); GSM = HOME / "scanner/StereoDepthSeg-gsmap/scan3d/gsmap"; FIXES = HOME / "scanner/StereoDepthSeg-fixes/scan3d/scanA"
sys.path.insert(0, str(GSM)); sys.path.insert(0, str(GSM.parent)); sys.path.insert(0, str(GSM.parent / "gsplat"))
import os  # noqa: E402
if os.environ.get("FFS_ROOT"):
    sys.path.insert(0, os.environ["FFS_ROOT"])


def pargs(a):
    return SimpleNamespace(run=a.run, vio=a.vio, rec=a.rec, gt="lensonly", out=str(Path(a.out) / "ffs"), win_s=2.0, n_pairs=4, b_min=0.03, b_max=0.25,
                           min_perp=40.0, max_rot=25.0, dir_deg=10.0, fb_perp=0.0, zmax=2.0, iters=8, k=2.0, fps=0.0, min_gap_s=0.2, profile=a.profile)


def vio_frames(vio, t_ns):
    vt, vT = [], []
    for line in gzip.open(vio, "rt"):
        d = json.loads(line)
        if d.get("type") == "pose" and d.get("T_world_cam0") and d.get("status") == "tracking":
            vt.append(int(d["t_ns"])); vT.append(np.array(d["T_world_cam0"], float).reshape(4, 4))
    vt = np.array(vt); vT = np.array(vT)
    idx = np.searchsorted(t_ns, vt).clip(1, len(t_ns) - 1); idx = np.where(np.abs(t_ns[idx - 1] - vt) < np.abs(t_ns[idx] - vt), idx - 1, idx)
    good = np.abs(t_ns[idx] - vt) < 8e6
    return {int(i): T for i, T in zip(idx[good], vT[good])}


def cmd_frames(a):
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    kf = [int(x) for x in np.load(Path(a.run) / "dump_ba/cams.npz")["kf"]]
    plan = json.loads(Path(a.plan).read_text())["chosen"]["plan"]
    parts, pairs = set(), set()
    for k, v in plan.items():
        for q in v:
            j = int(q[0]); parts.add(j); pairs.add((min(int(k), j), max(int(k), j)))
    allf = sorted(set(kf) | parts)
    (out / "extra_pairs.txt").write_text("".join(f"{x:05d}.jpg {y:05d}.jpg\n" for x, y in sorted(pairs)))
    info = dict(made=time.strftime("%F %T"), run=a.run, plan=a.plan, n_kf=len(kf), n_partner_frames=len(parts), n_partner_nonkf=len(parts - set(kf)),
                n_frames=len(allf), n_pairs=len(pairs), keyframes=allf, kf=kf)
    (out / "manifest.json").write_text(json.dumps(info, indent=1)); print(json.dumps({k: v for k, v in info.items() if k not in ("keyframes", "kf")}))


def cmd_ba(a):
    sys.path.insert(0, str(FIXES))
    import pycolmap
    for nm in ("FeatureExtractionOptions", "FeatureMatchingOptions"):     # power rule: <= 8 CPU threads
        orig = getattr(pycolmap, nm)

        def mk(orig=orig):
            o = orig(); o.num_threads = 8; return o
        setattr(pycolmap, nm, mk)
    import ba_torch, kf_ba
    out = Path(a.out) / "ba"; out.mkdir(parents=True, exist_ok=True)
    orig_ba = ba_torch.ba_lm_torch

    def ba_dump(R, t, X, cam, pt, uv, K, **kw):
        R1_, t1_, X1_, st = orig_ba(R, t, X, cam, pt, uv, K, **kw)
        np.savez_compressed(out / "ba_state.npz", R1=np.asarray(R1_), t1=np.asarray(t1_), cam=cam, pt=pt, uv=uv, K=K, X1=np.asarray(X1_),
                            fixed=np.array(sorted(kw.get("fixed", [])), int))
        return R1_, t1_, X1_, st
    ba_torch.ba_lm_torch = ba_dump
    sys.argv = ["kf_ba.py", str(a.rec), "--vio", str(a.vio), "--out", str(out), "--gt", "lensonly", "--kf-step", "10", "--scale", "0.5",
                "--seq", "6", "--cov-k", "10", "--cov-angle", "30", "--max-features", "6000", "--iters", "40", "--min-tri-deg", "1.0",
                "--kf-from-vio", "--match-ms", "1.0", "--feat", "sift", "--loop", "none", "--seed", "0", "--tracks", "drop",
                "--ba-device", "cuda", "--match-device", "auto", "--sift-device", "cpu", "--keep-db",
                "--cache-root", str(Path(a.out) / "cache"), "--kf-list", str(Path(a.out) / "manifest.json"), "--extra-pairs", str(Path(a.out) / "extra_pairs.txt")]
    try:
        kf_ba.main()
    except BaseException as e:
        if kf_ba._GJ[0] is not None:
            kf_ba._GJ[0].__exit__(type(e), e, e.__traceback__)
        raise


def umeyama(A, B):
    ma, mb = A.mean(0), B.mean(0); U, S, Vt = np.linalg.svd((B - mb).T @ (A - ma)); D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt; s = float(np.trace(np.diag(S) @ D) / ((A - ma) ** 2).sum()); return s, R, mb - s * R @ ma


def cmd_post(a):
    out = Path(a.out); man = json.loads((out / "manifest.json").read_text()); kfs = sorted(man["keyframes"]); kf0 = man["kf"]
    st = np.load(out / "ba/ba_state.npz"); bj = json.loads((out / "ba/ba.json").read_text())
    g = np.load(Path(a.rec) / "lensonly/gt.npz"); Tv = vio_frames(a.vio, g["t_ns"])
    R, t, cam = st["R1"], st["t1"], st["cam"]; nobs = np.bincount(cam, minlength=len(kfs))
    free = nobs >= 15
    Cn = np.array([-R[k].T @ t[k] for k in range(len(kfs))]); Rwc = np.transpose(R, (0, 2, 1))
    m = np.array([free[k] and kfs[k] in Tv for k in range(len(kfs))])
    s, Ra, ta = umeyama(Cn[m], np.array([Tv[kfs[k]][:3, 3] for k in np.flatnonzero(m)]))
    Tm = np.zeros((len(kfs), 4, 4)); Tm[:, 3, 3] = 1; Tm[:, :3, :3] = np.einsum("ij,njk->nik", Ra, Rwc); Tm[:, :3, 3] = s * Cn @ Ra.T + ta
    res = np.linalg.norm(Tm[m, :3, 3] - np.array([Tv[kfs[k]][:3, 3] for k in np.flatnonzero(m)]), axis=1)
    src = np.array(["ba"] * len(kfs), dtype=object); goodk = np.flatnonzero(free)
    for k in np.flatnonzero(~free):
        n_ = goodk[np.argmin(np.abs(np.array(kfs)[goodk] - kfs[k]))]; Tm[k] = Tm[n_] @ np.linalg.inv(Tv[kfs[n_]]) @ Tv[kfs[k]]; src[k] = "vio_anchor"
    np.savez_compressed(out / "poses_all.npz", frames=np.array(kfs), T=Tm, src=src.astype(str), nobs=nobs, note="VIO world frame (Sim3 of BA centres onto VIO)")
    c = dict(np.load(Path(a.run) / "dump_ba/cams.npz")); TmD = {f: Tm[k] for k, f in enumerate(kfs)}
    c["T"] = np.stack([TmD[int(f)] for f in c["kf"]]); c["poses"] = np.array("ba_partners_generic " + str(out)); c["made"] = np.array(time.strftime("%F %T"))
    (out / "run/dump_ba").mkdir(parents=True, exist_ok=True); np.savez(out / "run/dump_ba/cams.npz", **c)
    info = dict(made=time.strftime("%F %T"), n_frames=len(kfs), registered=int(free.sum()), vio_anchor=int((~free).sum()), sim3_scale=s,
                centre_vs_vio_med_cm=float(np.median(res) * 100), centre_vs_vio_p95_cm=float(np.percentile(res, 95) * 100),
                ba=dict(final_med_px=bj.get("final_med_px"), final_rmse_px=bj.get("final_rmse_px"), points=bj.get("points"), obs=bj.get("obs"), runtime_s=bj.get("runtime_s")))
    (out / "post.json").write_text(json.dumps(info, indent=1, default=float)); print(json.dumps(info, default=float))


def cmd_ffs(a):
    import cv2, lib_ffs
    import gsm_ffs_pairs2 as P2, motion_stereo as ms
    t0 = time.time(); A = pargs(a); out = Path(A.out); (out / "pairs").mkdir(parents=True, exist_ok=True); (out / "s1").mkdir(exist_ok=True)
    kf, Tk0, Kr, W0, H0, t_ns, K, dist, Tv = P2.load_common(A)
    plan, cfg, _ = P2.new_plan(A, kf, Tk0, K, W0, H0, t_ns, Tv, Kr); cfg.z_max = A.zmax
    ref = json.loads(Path(a.plan).read_text())["chosen"]["plan"]
    mine = {str(int(kf[c])): [int(q[1]) for q in v] for c, v in plan.items()}
    same = mine == {k: [q[0] for q in v] for k, v in ref.items() if k in mine}
    P = np.load(Path(a.out) / "poses_all.npz"); TmD = {int(f): T for f, T in zip(P["frames"], P["T"])}
    Tk = np.stack([TmD[int(f)] for f in kf])
    rp = dict(size=(W0, H0), f=float(K[0, 0]), cx=float(K[0, 2]), cy=float(K[1, 2]), R1=np.eye(3).tolist())
    need = {}
    for ci, ch in plan.items():
        for q in ch:
            need.setdefault(q[1], []).append(ci)
        need.setdefault(int(kf[ci]), []).append(ci)
    model = lib_ffs.load(valid_iters=A.iters)
    ready = {ci: max([int(kf[ci])] + [q[1] for q in plan[ci]]) for ci in plan}
    last_use = {f: max(ready[ci] for ci in cis) for f, cis in need.items()}
    todo = sorted([ci for ci in plan if not (out / "s1" / f"{int(kf[ci]):05d}.npz").exists()], key=lambda ci: ready[ci])
    print(f"plan {len(plan)} kf (same partners as PLAN: {same}), todo {len(todo)}, BA poses for kf and partner", flush=True)
    cap = cv2.VideoCapture(str(Path(A.rec) / "cam5.mp4")); buf = {}; fidx = -1; ti = 0; stats = {}
    while ti < len(todo):
        r_, f = cap.read(); fidx += 1
        if not r_:
            break
        if fidx in need:
            buf[fidx] = cv2.resize(f, (W0, H0), interpolation=cv2.INTER_AREA) if f.shape[1] != W0 else f
        while ti < len(todo) and ready[todo[ti]] <= fidx:
            ci = todo[ti]; ti += 1; fi = int(kf[ci]); res = []
            for s, j, gg, Tj_vio, _src in plan[ci]:
                R, t = ms.rel_pose(Tk[ci], TmD[j])
                res.append(ms.pair_depth(model, lib_ffs.infer, buf[fi], buf[j], R, t, K, dist, rp, cfg, iters=A.iters))
            Z, V, S, agree = ms.combine_motion(res, cfg)
            np.savez_compressed(out / "pairs" / f"{fi:05d}.npz", Z=np.stack([np.where(r["valid"], r["Z"], np.nan) for r in res]).astype(np.float16),
                                B=np.array([r["B"] for r in res]), f=np.array([r["f"] for r in res]), partner=np.array([q[1] for q in plan[ci]]),
                                src=np.array([q[4] for q in plan[ci]]), perp=np.array([r["perp_deg"] for r in res]), B_vio=np.array([q[2]["B"] for q in plan[ci]]))
            np.savez_compressed(out / "s1" / f"{fi:05d}.npz", Z=np.where(V, Z, np.nan).astype(np.float32), V=V, S=np.where(V, S, np.inf).astype(np.float32))
            stats[fi] = dict(pairs=len(res), valid_frac=float(V.mean()), B_cm=[round(r["B"] * 100, 1) for r in res], B_vio_cm=[round(q[2]["B"] * 100, 1) for q in plan[ci]])
        for fdel in [k for k in buf if last_use[k] < fidx]:
            del buf[fdel]
    cap.release()
    (out / "s1" / "ffs_info.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), poses=str(Path(a.out) / "poses_all.npz"), same_partners=same,
                                                              n_done=len(stats), total_s=round(time.time() - t0, 1), per_kf=stats), indent=1, default=float))
    print(f"done {len(stats)} kf in {time.time() - t0:.0f}s", flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("step", choices=["frames", "ba", "post", "ffs"])
    for k in ("run", "plan", "rec", "vio", "out"):
        ap.add_argument("--" + k, required=True)
    ap.add_argument("--profile", default="new"); a = ap.parse_args()
    if a.step in ("ba", "ffs"):
        import gpu_guard
        with gpu_guard.job(f"newrec ba_partners_generic {a.step} {a.out}", torch_cap=True):
            {"ba": cmd_ba, "ffs": cmd_ffs}[a.step](a)
    else:
        {"frames": cmd_frames, "post": cmd_post}[a.step](a)
