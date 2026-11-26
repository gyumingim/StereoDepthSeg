#!/usr/bin/env python3
"""gsmap 2026-11-26: FFS-only room map of 121801 on the new BA poses (ba_partners: 494 keyframes + 964 partners).
Stage A = the 4 m FFS chain (gsm_ffs_4m wide tier + error cut + inverse-variance fusion, S key) re-run on the new poses.

  prep    (CPU)  OUT/plan_4m.json: SAME wide partner frames as ffs_4m/plan_4m.json (old plan), geometry recomputed with the new poses;
                 base_c (baseline direction of every pairs2 pair, for sin theta) recomputed with the new poses.
                 Partner pose = BA pose from poses_all.npz if the frame was in the BA; else VIO relative motion anchored on the nearest
                 BA-posed frame (all such partners are <= 4 frames from a BA-posed frame) -> OUT/wide_poses.npz
  ffs     (GPU)  wide pairs with the new poses -> OUT/wide/<kf>.npz (gsm_ffs_4m.cmd_ffs body)
  fuse    (CPU)  gsm_ffs_fix3 fuse (= gsm_ffs_4m fuse rule, near = L, ivw, no BA/VIO rescale) on ba_partners/ffs_pairs2 + ffs_pairs3 + OUT/wide,
                 judge poses = ba_partners/run/dump_ba/cams.npz; optional --kscale
  kfit / judge (CPU)  gsm_ffs_fix3 kfit / judge with RUN = ba_partners/run
  measure (GPU)  gsm_ffs_4m measure (pairs3 measure rule, gold, depth bins) with RUN = ba_partners/run
  fused   (CPU)  training dir: Z, F (1 = FFS, no fill), S  from a chosen set
"""
import argparse, json, os, sys, time, bisect
from pathlib import Path
from types import SimpleNamespace

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / "gsplat"))
if os.environ.get("FFS_ROOT"):
    sys.path.insert(0, os.environ["FFS_ROOT"])
import gsm_ba_partners as BP

R1, V2 = BP.R1, BP.V2
BAP = V2 / "ba_partners"; RUNN = BAP / "run"; P2N = BAP / "ffs_pairs2"; P3N = BAP / "ffs_pairs3"; OLD4 = V2 / "ffs_4m"
OUT = V2 / "ffsonly_ba"


def A_(**kw):
    a = BP.ffs_args(run=R1, out=OUT, p2=P2N)
    d = vars(a); d.update(p3=str(P3N), n_wide=2, wb_min=0.15, wb_max=0.35, wide_znear=1.0, z_split=1.5, zmax=4.0, cap=0.03, iters=8, k=2.0,
                         weight="ivw", rescale=0, near="L", name="s4m", zcap=0.0, json="", gsmap_run=str(RUNN), fig="", fig_set="", fig_kf="642,649,656",
                         src=str(OUT), rs_exp=0.0, kscale="", dark_thr=0.0, dark_mode="drop", dark_w=0.25, set="", kout="", parity="all", kmin=0.9, kmax=1.1,
                         ref_set="", gold="", sets="", views="all,even")
    d.update(kw)
    return SimpleNamespace(**d)


def poses_for_plan():
    import gsm_ffs_pairs2 as P2
    a = A_(); kf, _Tk0, Kr, W0, H0, t_ns, K, dist, Tv = P2.load_common(a)
    TmD = BP.new_poses(); posed = sorted(f for f in TmD if f in Tv)
    old = json.loads((OLD4 / "plan_4m.json").read_text())

    def pose(j):
        if j in TmD:
            return TmD[j], "ba", j
        i = bisect.bisect_left(posed, j); c = [posed[q] for q in (i - 1, i) if 0 <= q < len(posed)]; nb = min(c, key=lambda f: abs(f - j))
        assert j in Tv and abs(nb - j) <= 6, (j, nb)
        return TmD[nb] @ np.linalg.inv(Tv[nb]) @ Tv[j], "vio_anchor", nb
    return kf, K, dist, W0, H0, Tv, TmD, old, pose


def cmd_prep(_):
    import motion_stereo as ms
    kf, K, dist, W0, H0, Tv, TmD, old, pose = poses_for_plan(); OUT.mkdir(parents=True, exist_ok=True)
    kfi = {int(f): ci for ci, f in enumerate(kf)}; plan = {}; base_c = {}; fr, Ts, srcs, anc = [], [], [], []
    for fk, lst in old["plan"].items():
        Tk = TmD[int(fk)]; row = []
        for q in lst:
            j = int(q[0]); Tj, s, nb = pose(j); R, t = ms.rel_pose(Tk, Tj); g = ms.pair_geometry(R, t)
            row.append([j, round(g["B"] * 100, 1), round(g["perp_deg"], 1), round(g["rot_deg"], 1), round(g["B"] * np.sin(np.radians(g["perp_deg"])) * 100, 1),
                        [float(x) for x in g["c"]], s, q[1]])
            fr.append(j); Ts.append(Tj); srcs.append(s); anc.append(nb)
        plan[fk] = row
    nb_missing = 0
    for f in sorted((P2N / "pairs").glob("*.npz")):
        fi = int(f.stem); d = np.load(f); Tk = TmD[fi]; rr = []
        for j in d["partner"]:
            j = int(j)
            if j not in TmD:
                nb_missing += 1; continue
            R, t = ms.rel_pose(Tk, TmD[j]); g = ms.pair_geometry(R, t); rr.append([j] + [float(x) for x in g["c"]])
        base_c[str(fi)] = rr
    Bn = np.array([q[1] for v in plan.values() for q in v]); Bo = np.array([q[7] for v in plan.values() for q in v])
    res = dict(made=time.strftime("%F %T"), note="wide partner frames = ffs_4m/plan_4m.json (old plan); geometry with new BA poses. row = "
               "[partner, B_cm, perp, rot, B*sin(perp) cm, c (partner centre in kf frame), pose source, B_cm_old]",
               poses=str(BAP / "poses_all.npz"), n_kf=len(kf), kf_with_wide=len(plan), n_wide_pairs=int(len(Bn)),
               pose_src={s: srcs.count(s) for s in set(srcs)}, B_cm_med=float(np.median(Bn)), B_new_over_old_p5_50_95=[float(x) for x in np.percentile(Bn / Bo, [5, 50, 95])],
               base_pairs_missing_pose=nb_missing, plan=plan, base_c=base_c)
    (OUT / "plan_4m.json").write_text(json.dumps(res, indent=1, default=float))
    np.savez(OUT / "wide_poses.npz", frames=np.array(fr), T=np.array(Ts), src=np.array(srcs), anchor=np.array(anc))
    print(json.dumps({k: v for k, v in res.items() if k not in ("plan", "base_c")}, indent=1, default=float))


def cmd_ffs(a):
    import cv2, lib_ffs
    import motion_stereo as ms
    import gsm_ffs_4m as M
    t0 = time.time(); kf, K, dist, W0, H0, Tv, TmD, old, pose = poses_for_plan()
    pl = json.loads((OUT / "plan_4m.json").read_text())["plan"]; kfi = {int(f): ci for ci, f in enumerate(kf)}
    plan = {kfi[int(fk)]: [(q[0], pose(q[0])[0], q[5]) for q in v] for fk, v in pl.items()}
    (OUT / "wide").mkdir(parents=True, exist_ok=True); cfg = ms.MSConfig(); cfg.z_max = 4.0
    rp = dict(size=(W0, H0), f=float(K[0, 0]), cx=float(K[0, 2]), cy=float(K[1, 2]), R1=np.eye(3).tolist())
    need = {}
    for ci, ch in plan.items():
        for q in ch:
            need.setdefault(q[0], []).append(ci)
        need.setdefault(int(kf[ci]), []).append(ci)
    ready = {ci: max([int(kf[ci])] + [q[0] for q in plan[ci]]) for ci in plan}
    last_use = {f_: max(ready[ci] for ci in cis) for f_, cis in need.items()}
    todo = sorted([ci for ci in plan if not (OUT / "wide" / f"{int(kf[ci]):05d}.npz").exists()], key=lambda ci: ready[ci])
    print(f"wide plan {len(plan)} kf, todo {len(todo)}, new BA poses", flush=True)
    model = lib_ffs.load(valid_iters=8)
    cap = cv2.VideoCapture(str(R1 / "rec" / "cam5.mp4")); buf = {}; fidx = -1; ti = 0; stats = {}
    while ti < len(todo):
        r_, fr = cap.read(); fidx += 1
        if not r_:
            break
        if fidx in need:
            buf[fidx] = cv2.resize(fr, (W0, H0), interpolation=cv2.INTER_AREA) if fr.shape[1] != W0 else fr
        while ti < len(todo) and ready[todo[ti]] <= fidx:
            ci = todo[ti]; ti += 1; fi = int(kf[ci]); res = []; Tk = TmD[fi]
            for j, Tj, c in plan[ci]:
                R, t = ms.rel_pose(Tk, Tj)
                res.append(ms.pair_depth(model, lib_ffs.infer, buf[fi], buf[j], R, t, K, dist, rp, cfg, z_near=1.0, iters=8))
            np.savez_compressed(OUT / "wide" / f"{fi:05d}.npz", Z=np.stack([np.where(r["valid"], r["Z"], np.nan) for r in res]).astype(np.float16),
                                sin=np.stack([M.sin_map(K, c, W0, H0) for _, _, c in plan[ci]]).astype(np.float16),
                                B=np.array([r["B"] for r in res]), f=np.array([r["f"] for r in res]), partner=np.array([q[0] for q in plan[ci]]),
                                perp=np.array([r["perp_deg"] for r in res]), scale=np.array([r["scale"] for r in res]))
            stats[fi] = dict(valid=[round(float(r["valid"].mean()), 4) for r in res], B_cm=[round(r["B"] * 100, 1) for r in res])
            if ti % 25 == 0:
                print(f"{ti}/{len(todo)} kf {fi} {stats[fi]['valid']} t {time.time() - t0:.0f}s", flush=True)
        for fdel in [k for k in buf if last_use[k] < fidx]:
            del buf[fdel]
    cap.release()
    (OUT / "wide" / "ffs_info.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), poses=str(BAP / "poses_all.npz"), total_s=round(time.time() - t0, 1), per_kf=stats), indent=1))
    print(f"done {len(stats)} kf {time.time() - t0:.0f}s", flush=True)


def cmd_fuse(a):
    import gsm_ffs_fix3 as F3
    F3.cmd_fuse(A_(run=str(RUNN), name=a.name, kscale=a.kscale))


def cmd_kfit(a):
    import gsm_ffs_fix3 as F3
    F3.cmd_kfit(A_(run=str(RUNN), set=str(OUT / a.name), parity=a.parity, kout=a.kout))


def cmd_judge(a):
    import gsm_ffs_fix3 as F3
    F3.cmd_judge(A_(run=str(RUNN), ref_set=str(OUT / "s4m"), gold=str(RUNN / "eval2" / "ref" / "gold_ffs.npz"), json=str(OUT / "judge.json"),
                    sets=",".join(f"{n}={OUT / n}" if n != "L" else f"L={P3N / 's12_L'}" for n in a.name.split(","))))


def cmd_measure(a):
    import gsm_ffs_4m as M
    M.cmd_measure(A_(out=str(OUT), name=a.name, json="measure_4m.json"))


def cmd_fused(a):
    src = OUT / a.name; od = OUT / "fused"; od.mkdir(parents=True, exist_ok=True)
    kf = np.load(RUNN / "dump_ba" / "cams.npz")["kf"]; vf = []
    for i in kf:
        f = src / f"{int(i):05d}.npz"
        if f.exists():
            d = np.load(f); Z = d["Z"].astype(np.float32); V = np.isfinite(Z) & (Z > 0); S = np.where(V, d["S"], np.inf).astype(np.float32)
        else:
            Z = np.full((540, 960), np.nan, np.float32); V = np.zeros_like(Z, bool); S = np.full_like(Z, np.inf)
        np.savez_compressed(od / f"{int(i):05d}.npz", Z=np.where(V, Z, 0).astype(np.float32), F=V.astype(np.uint8), S=S); vf.append(float(V.mean()))
    (od / "info.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), src=str(src), note="FFS only (F=1), no MoGe/DA3 fill", n=len(kf),
                                                  valid_mean=float(np.mean(vf)), kf_with_depth=int(sum(v > 0 for v in vf))), indent=1))
    print("fused", od, np.mean(vf))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["prep", "ffs", "fuse", "kfit", "judge", "measure", "fused"])
    ap.add_argument("--name", default="s4m"); ap.add_argument("--kscale", default=""); ap.add_argument("--parity", default="all"); ap.add_argument("--kout", default="")
    a = ap.parse_args()
    if a.cmd in ("ffs", "measure"):
        import gpu_guard
        with gpu_guard.job("gsmap ffsonly_ba " + a.cmd, torch_cap=True):
            {"ffs": cmd_ffs, "measure": cmd_measure}[a.cmd](a)
    else:
        {"prep": cmd_prep, "fuse": cmd_fuse, "kfit": cmd_kfit, "judge": cmd_judge, "fused": cmd_fused}[a.cmd](a)
