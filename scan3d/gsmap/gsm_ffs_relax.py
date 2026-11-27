#!/usr/bin/env python3
"""gsmap 2026-11-26: FFS partners for the keyframes WITHOUT a wide partner (121801, forward walking), relaxed rule, BA-posed.

Why (audit7 an5/an6): 166 of 494 keyframes have no wide (gsm_ffs_4m, perp >= 40) partner; their usable FFS pixels average 7.9 % vs
40.5 %. The user walked along the viewing direction there (motion vs view ~21 deg), so --min-perp 40 rejects every partner.

  plan    (CPU)  OUT/plan_relax.json: for the target keyframes only (no ffsonly_ba/wide/<kf>.npz) search partners in +-win_s (4 s),
                 B 0.10-0.50 m, perp >= 10, rot <= 25, dt >= 0.15 s, not an existing base/low partner. Screening pose = ba_partners
                 BA pose if the frame is in it, else VIO relative motion anchored on the keyframe BA pose. Score = predicted usable
                 share of the image: pixel rays at Z = 1.5 / 2.5 / 3.5 m with theta (ray vs baseline) >= theta_min (25, the
                 pairs3 epipole cone), sigma_Z/Z = Z/(f B sin theta) <= cap (3 %), point in front of and inside the partner image.
                 Up to 2 partners per keyframe, greedy by score; diversity: >= 0.2 s apart (12 frames @ 60 fps) and (direction >= 10 deg or B ratio
                 >= 1.5 or opposite time side). Min score 0.03. Reasons for keyframes without a partner are counted.
  frames  (CPU)  BA2/manifest.json = ba_partners frames + new partner frames; BA2/extra_pairs.txt = ba_partners pairs + new kf-partner pairs
  ba      (GPU)  gsm_ba_partners.cmd_ba with OUT = BA2 (identical kf_ba settings)
  post    (CPU)  gsm_ba_partners.cmd_post with OUT = BA2 (Sim3 onto the same old map frame) + BA2/shift.json (keyframe shift vs ba_partners)
  ffs     (GPU)  pairs3 partial rectification (P3.partial_depth: 4 azimuth sectors, 25 deg epipole cone excluded, per-pixel 3 % cut,
                 zmax 4 m, z_near 1 m, FFS max_disp 192 at 960x540, same model) with BA2 poses for keyframe AND partner; partners
                 that BA2 did not register (src != ba) are dropped -> OUT/relax/<kf>.npz (wide format: Z, sin, B, f, partner)
  fuse    (CPU)  gsm_ffs_fix3 fuse (= ffsonly_ba s4m rule: error cut, >= 2 pairs agree k=2, inverse variance, single-pair neighbour
                 check), judge = BA2 poses, src = OUT/src (plan_4m.json of ffsonly_ba + wide/ = ffsonly_ba/wide + relax files):
                 run 1 z_split 1.5 (= s4m rule) -> OUT/r15, run 2 z_split 0 (no L near domain) -> OUT/r0
  combine (CPU)  OUT/s4m_r: non-target keyframes = ffsonly_ba/s4m unchanged; target keyframes = r15 where finite, else r0 (pair fusion
                 also below 1.5 m where L has nothing)
  measure (GPU)  multi-view fail % (gsm_ffs_4m measure rule = pairs3 rule, 2x grid, each set its own reference) for s4m (judge
                 ba_partners, reproduces measure_4m 3.55) and s4m_r (judge BA2): all / non-target / target / aisle 314-331 / new pixels
  fused   (CPU)  training dir (Z, F, S) -> TRAIN/fused
"""
import argparse, json, os, shutil, sys, time
from collections import Counter
from pathlib import Path
from types import SimpleNamespace

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / "gsplat"))
if os.environ.get("FFS_ROOT"):
    sys.path.insert(0, os.environ["FFS_ROOT"])
import gsm_ba_partners as BP

R1, V2 = BP.R1, BP.V2
BAP = V2 / "ba_partners"; BA2 = V2 / "ba_partners2"; OUT = V2 / "ffs_relax"; OLD = V2 / "ffsonly_ba"; TRAIN = V2 / "ffsonly_ba2"
AISLE = list(range(314, 332))


def common():
    import gsm_ffs_pairs2 as P2
    a = BP.ffs_args(run=BAP / "run", out=OUT)
    kf, Tk, Kr, W0, H0, t_ns, K, dist, Tv = P2.load_common(a)
    return kf, Tk, Kr, W0, H0, t_ns, K, dist, Tv


def targets(kf):
    return [ci for ci, f in enumerate(kf) if not (OLD / "wide" / f"{int(f):05d}.npz").exists()]


def used_partners(fi):
    s = set()
    for f in (BAP / "ffs_pairs2" / "pairs" / f"{fi:05d}.npz", BAP / "ffs_pairs3" / "extra" / f"{fi:05d}.npz"):
        if f.exists():
            s |= {int(x) for x in np.load(f)["partner"]}
    return s


def cmd_plan(a):
    import motion_stereo as ms
    kf, Tk, Kr, W0, H0, t_ns, K, dist, Tv = common(); n = len(kf)
    P = np.load(BAP / "poses_all.npz"); TmD = {int(f): T for f, T in zip(P["frames"], P["T"])}
    import rec_profile as RP
    if RP.profile(BP.R1 / "rec", a.profile) == "new":
        raise SystemExit("relax partners are a low-angle backdoor: off in the new profile (use --profile legacy)")
    fps = RP.rec_fps(BP.R1 / "rec", t_ns, a.fps); gap = RP.gap_frames(a.min_gap_s, fps)        # was 60 fps / 12 frames (2026-11-27)
    tg = targets(kf); win = RP.frames(a.win_s, fps); f0 = float(Kr[0, 0])
    gu, gv = np.meshgrid(np.linspace(8, W0 - 8, 64), np.linspace(8, H0 - 8, 36))
    ray = np.stack([(gu - Kr[0, 2]) / Kr[0, 0], (gv - Kr[1, 2]) / Kr[1, 1], np.ones_like(gu)], -1).reshape(-1, 3)
    rn = ray / np.linalg.norm(ray, axis=1, keepdims=True); smin = np.sin(np.radians(a.theta_min))

    def score(R, t, g):
        e = g["c"] / g["B"]; sn = np.sqrt(np.clip(1 - (rn @ e) ** 2, 0, 1)); s = []
        for Z in (1.5, 2.5, 3.5):
            X = ray * Z; Xj = X @ R.T + t; zj = Xj[:, 2]
            with np.errstate(divide="ignore", invalid="ignore"):
                u = Kr[0, 0] * Xj[:, 0] / zj + Kr[0, 2]; v = Kr[1, 1] * Xj[:, 1] / zj + Kr[1, 2]
            ok = (zj > 0.3) & (u >= 0) & (u < W0) & (v >= 0) & (v < H0) & (sn >= smin) & (Z / (f0 * g["B"] * np.maximum(sn, 1e-6)) <= a.cap)
            s.append(ok.mean())
        return float(np.mean(s)), [round(float(x), 3) for x in s]
    plan, why, stats = {}, Counter(), {}
    for ci in tg:
        fi = int(kf[ci])
        if fi not in Tv:
            why["kf_no_vio"] += 1; stats[fi] = "kf_no_vio"; continue
        used = used_partners(fi); A = Tk[ci] @ np.linalg.inv(Tv[fi]); cnt = Counter(); cands = []
        for j in range(max(0, fi - win), min(len(t_ns), fi + win + 1)):
            if j == fi or j in used or abs(t_ns[fi] - t_ns[j]) * 1e-9 < 0.15:
                continue
            if j not in TmD and j not in Tv:
                continue
            cnt["window"] += 1
            Tj = TmD[j] if j in TmD else A @ Tv[j]; R, t = ms.rel_pose(Tk[ci], Tj); g = ms.pair_geometry(R, t)
            if not (a.b_min <= g["B"] <= a.b_max):
                continue
            cnt["B_ok"] += 1
            if g["perp_deg"] < a.min_perp:
                continue
            cnt["perp_ok"] += 1
            if g["rot_deg"] > a.max_rot:
                continue
            cnt["rot_ok"] += 1
            sc, sz = score(R, t, g)
            if sc < a.min_score:
                continue
            cnt["cov_ok"] += 1; g["j"] = j
            cands.append(dict(score=sc, s_z=sz, j=j, g=g, dt=round(float((t_ns[j] - t_ns[fi]) * 1e-9), 3), src="ba" if j in TmD else "vio"))
        cands.sort(key=lambda x: -x["score"]); ch = []
        for c_ in cands:
            if all(abs(c_["j"] - q["j"]) >= gap and (ms.dir_differs(c_["g"], q["g"], 10.0) or max(c_["g"]["B"], q["g"]["B"]) / min(c_["g"]["B"], q["g"]["B"]) >= 1.5
                                                     or (c_["j"] - fi) * (q["j"] - fi) < 0) for q in ch):
                ch.append(c_)
            if len(ch) >= a.n_new:
                break
        if ch:
            plan[fi] = [[q["j"], round(q["g"]["B"] * 100, 1), round(q["g"]["perp_deg"], 1), round(q["g"]["rot_deg"], 1), q["dt"], round(q["score"], 3), q["s_z"], q["src"]] for q in ch]
            why[f"got_{len(ch)}"] += 1
        else:
            r = "no_frame_in_window" if not cnt["window"] else "B_out_of_range" if not cnt["B_ok"] else "perp_lt_min" if not cnt["perp_ok"] else \
                "rot_gt_max" if not cnt["rot_ok"] else "predicted_cover_lt_min"
            why[r] += 1; stats[fi] = dict(reason=r, **cnt)
    allq = [q for v in plan.values() for q in v]
    res = dict(made=time.strftime("%F %T"), args=vars(a), inputs=dict(kf_poses=str(BAP / "run/dump_ba/cams.npz"), partner_poses=str(BAP / "poses_all.npz"),
                                                                       vio=str(BP.VIO), wide=str(OLD / "wide")),
               n_kf=n, n_target=len(tg), target_kf_idx=tg, kf_with_new=len(plan), why=dict(why), fail_detail=stats,
               n_pairs=len(allq), new_frames=len({q[0] for q in allq} - set(TmD)), partner_src=dict(Counter(q[7] for q in allq)),
               B_cm_p10_50_90=[float(x) for x in np.percentile([q[1] for q in allq], [10, 50, 90])] if allq else None,
               perp_p10_50_90=[float(x) for x in np.percentile([q[2] for q in allq], [10, 50, 90])] if allq else None,
               absdt_p10_50_90=[float(x) for x in np.percentile([abs(q[4]) for q in allq], [10, 50, 90])] if allq else None,
               score_p10_50_90=[float(x) for x in np.percentile([q[5] for q in allq], [10, 50, 90])] if allq else None,
               aisle_314_331=dict(n=len(AISLE), with_new=sum(int(kf[ci]) in plan for ci in AISLE), target=sum(ci in tg for ci in AISLE)),
               note="row = [partner frame, B cm, perp deg, rot deg, dt s, score, usable share at Z 1.5/2.5/3.5, screening pose src]",
               plan={str(k): v for k, v in plan.items()})
    OUT.mkdir(parents=True, exist_ok=True); (OUT / "plan_relax.json").write_text(json.dumps(res, indent=1, default=float))
    print(json.dumps({k: v for k, v in res.items() if k not in ("plan", "fail_detail", "target_kf_idx", "args")}, indent=1, default=float))


def cmd_frames(a):
    BA2.mkdir(parents=True, exist_ok=True)
    m = json.loads((BAP / "manifest.json").read_text()); pl = json.loads((OUT / "plan_relax.json").read_text())["plan"]
    newp = {(int(k), int(q[0])) for k, v in pl.items() for q in v}; newf = {j for _, j in newp}
    allf = sorted(set(m["keyframes"]) | newf)
    old = [l.split() for l in (BAP / "extra_pairs.txt").read_text().splitlines() if l.strip()]
    oldp = {(int(x[:-4]), int(y[:-4])) for x, y in old}
    ex = sorted(oldp | {(min(k, j), max(k, j)) for k, j in newp})
    (BA2 / "extra_pairs.txt").write_text("".join(f"{x:05d}.jpg {y:05d}.jpg\n" for x, y in ex))
    info = dict(made=time.strftime("%F %T"), inputs=dict(manifest=str(BAP / "manifest.json"), extra_pairs=str(BAP / "extra_pairs.txt"), plan=str(OUT / "plan_relax.json")),
                n_kf=m["n_kf"], n_frames_old=len(m["keyframes"]), n_new_frames=len(newf - set(m["keyframes"])), n_frames=len(allf),
                n_extra_pairs_old=len(oldp), n_new_pairs=len(newp), n_extra_pairs=len(ex), keyframes=allf, old_kf=m["old_kf"])
    (BA2 / "manifest.json").write_text(json.dumps(info, indent=1))
    print(json.dumps({k: v for k, v in info.items() if k not in ("keyframes", "old_kf")}, indent=1))


def cmd_ba(a):
    BP.OUT = BA2; BP.cmd_ba(a)


def cmd_post(a):
    BP.OUT = BA2; BP.cmd_post(a)
    o = np.load(BAP / "run/dump_ba/cams.npz"); nw = np.load(BA2 / "run/dump_ba/cams.npz"); kf = o["kf"]
    d = np.linalg.norm(nw["T"][:, :3, 3] - o["T"][:, :3, 3], axis=1) * 100
    r = BP.rot_deg(np.einsum("nji,njk->nik", nw["T"][:, :3, :3], o["T"][:, :3, :3]))
    pa = np.load(BA2 / "poses_all.npz"); pl = json.loads((OUT / "plan_relax.json").read_text())["plan"]
    src = {int(f): str(s) for f, s in zip(pa["frames"], pa["src"])}; newf = sorted({int(q[0]) for v in pl.values() for q in v})
    tg = targets(kf); ais = np.array(AISLE)
    res = dict(made=time.strftime("%F %T"), inputs=dict(old=str(BAP / "run/dump_ba/cams.npz"), new=str(BA2 / "run/dump_ba/cams.npz")),
               kf_shift_cm=dict(max=round(float(d.max()), 3), med=round(float(np.median(d)), 3), p95=round(float(np.percentile(d, 95)), 3),
                                argmax_kf_idx=int(np.argmax(d)), target_med=round(float(np.median(d[tg])), 3), aisle_med=round(float(np.median(d[ais])), 3)),
               kf_rot_deg=dict(max=round(float(r.max()), 4), med=round(float(np.median(r)), 4), p95=round(float(np.percentile(r, 95)), 4)),
               new_partner_src=dict(Counter(src.get(j, "missing") for j in newf)), n_new_partners=len(newf))
    (BA2 / "shift.json").write_text(json.dumps(res, indent=1)); print(json.dumps(res, indent=1))


def cmd_ffs(a):
    import cv2, lib_ffs
    import gsm_ffs_pairs3 as P3, motion_stereo as ms
    t0 = time.time(); kf, _Tk, Kr, W0, H0, t_ns, K, dist, Tv = common()
    pa = np.load(BA2 / "poses_all.npz"); TmD = {int(f): T for f, T in zip(pa["frames"], pa["T"])}; src = {int(f): str(s) for f, s in zip(pa["frames"], pa["src"])}
    pl = json.loads((OUT / "plan_relax.json").read_text())["plan"]; od = OUT / "relax"; od.mkdir(parents=True, exist_ok=True)
    A = SimpleNamespace(theta_min=a.theta_min, cap=a.cap, zmax=4.0, z_near=1.0, f_scale=1.0, max_px=1.0e6)
    plan, dropped = {}, []
    for k, v in pl.items():
        fi = int(k); keep = [q for q in v if src.get(int(q[0])) == "ba" and src.get(fi) in ("ba", "old_ba")]
        dropped += [[fi, int(q[0]), src.get(int(q[0]))] for q in v if q not in keep]
        if keep:
            plan[fi] = keep
    need = {}
    for fi, ch in plan.items():
        for q in ch:
            need.setdefault(int(q[0]), []).append(fi)
        need.setdefault(fi, []).append(fi)
    ready = {fi: max([fi] + [int(q[0]) for q in ch]) for fi, ch in plan.items()}
    last_use = {f_: max(ready[x] for x in fs) for f_, fs in need.items()}
    todo = sorted([fi for fi in plan if not (od / f"{fi:05d}.npz").exists()], key=lambda x: ready[x])
    print(f"relax plan {len(plan)} kf {sum(len(v) for v in plan.values())} pairs, dropped (not BA-registered) {len(dropped)}, todo {len(todo)}", flush=True)
    model = lib_ffs.load(valid_iters=8)
    cap = cv2.VideoCapture(str(R1 / "rec" / "cam5.mp4")); buf = {}; fidx = -1; ti = 0; stats = {}
    while ti < len(todo):
        r_, fr = cap.read(); fidx += 1
        if not r_:
            break
        if fidx in need:
            buf[fidx] = cv2.resize(fr, (W0, H0), interpolation=cv2.INTER_AREA) if fr.shape[1] != W0 else fr
        while ti < len(todo) and ready[todo[ti]] <= fidx:
            fi = todo[ti]; ti += 1; Zs, Ss, Bs, inf = [], [], [], []
            for q in plan[fi]:
                j = int(q[0]); R, t = ms.rel_pose(TmD[fi], TmD[j])
                Z, st, B, f0, info = P3.partial_depth(model, lib_ffs.infer, buf[fi], buf[j], R, t, K, dist, W0, H0, A, 8)
                Zs.append(Z); Ss.append(st); Bs.append(B)
                inf.append(dict(j=j, valid=round(float(np.isfinite(Z).mean()), 4), B_cm=round(B * 100, 2), B_plan_cm=q[1], sec=info))
            np.savez_compressed(od / f"{fi:05d}.npz", Z=np.stack(Zs).astype(np.float16), sin=np.stack(Ss).astype(np.float16), B=np.array(Bs),
                                f=np.full(len(Bs), float(K[0, 0])), partner=np.array([int(q[0]) for q in plan[fi]]), perp=np.array([q[2] for q in plan[fi]]),
                                tag=np.array(["relax"] * len(Bs)))
            stats[fi] = inf
            if ti % 20 == 0:
                print(f"{ti}/{len(todo)} kf {fi} {[x['valid'] for x in inf]} t {time.time() - t0:.0f}s", flush=True)
        for fdel in [k for k in buf if last_use[k] < fidx]:
            del buf[fdel]
    cap.release()
    (od / "ffs_info.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), poses=str(BA2 / "poses_all.npz"), args=vars(A), dropped=dropped,
                                                      total_s=round(time.time() - t0, 1), per_kf={str(k): v for k, v in stats.items()}), indent=1, default=float))
    print(f"done {len(stats)} kf {time.time() - t0:.0f}s", flush=True)


def cmd_fuse(a):
    import gsm_ffsonly_ba as FO, gsm_ffs_fix3 as F3
    s = OUT / "src"; (s / "wide").mkdir(parents=True, exist_ok=True); shutil.copy(OLD / "plan_4m.json", s / "plan_4m.json")
    for f in (OLD / "wide").glob("*.npz"):
        if not (s / "wide" / f.name).exists():
            os.symlink(f, s / "wide" / f.name)
    for f in (OUT / "relax").glob("*.npz"):
        assert not (OLD / "wide" / f.name).exists(), f
        if not (s / "wide" / f.name).exists():
            os.symlink(f, s / "wide" / f.name)
    for nm, zs in (("r15", 1.5), ("r0", 0.0)):
        F3.cmd_fuse(FO.A_(run=str(BA2 / "run"), out=str(OUT), src=str(s), name=nm, z_split=zs))


def cmd_combine(a):
    kf = np.load(BA2 / "run/dump_ba/cams.npz")["kf"]; tg = set(targets(kf)); od = OUT / "s4m_r"; od.mkdir(parents=True, exist_ok=True)
    st = Counter(); diff_nt = []; per = {}
    for ci, i in enumerate(kf):
        nm = f"{int(i):05d}.npz"; fo = OLD / "s4m" / nm
        if ci not in tg:
            if fo.exists():
                shutil.copy(fo, od / nm)
                fr = OUT / "r15" / nm
                if fr.exists():
                    zo = np.load(fo)["Z"]; zr = np.load(fr)["Z"]; diff_nt.append(float((np.isfinite(zo) != np.isfinite(zr)).mean()))
            continue
        d1 = np.load(OUT / "r15" / nm) if (OUT / "r15" / nm).exists() else None; d0 = np.load(OUT / "r0" / nm) if (OUT / "r0" / nm).exists() else None
        if d1 is None and d0 is None:
            if fo.exists():
                shutil.copy(fo, od / nm)
            st["target_no_output"] += 1; continue
        H, W = 540, 960
        Z1 = d1["Z"] if d1 is not None else np.full((H, W), np.nan, np.float32); S1 = d1["S"] if d1 is not None else np.full((H, W), np.inf, np.float32)
        T1 = d1["T"] if d1 is not None else np.zeros((H, W), np.uint8)
        Z0 = d0["Z"] if d0 is not None else np.full((H, W), np.nan, np.float32); S0 = d0["S"] if d0 is not None else np.full((H, W), np.inf, np.float32)
        T0 = d0["T"] if d0 is not None else np.zeros((H, W), np.uint8)
        add = ~np.isfinite(Z1) & np.isfinite(Z0) & (Z0 >= 0.3)
        Z = np.where(np.isfinite(Z1), Z1, np.where(add, Z0, np.nan)).astype(np.float32); V = np.isfinite(Z)
        S = np.where(np.isfinite(Z1), S1, np.where(add, S0, np.inf)).astype(np.float32); T = np.where(add, T0 | 8, T1).astype(np.uint8)
        np.savez_compressed(od / nm, Z=Z, V=V, T=T, S=S)
        old_v = float(np.isfinite(np.load(fo)["Z"]).mean()) if fo.exists() else 0.0
        per[int(i)] = dict(old=round(old_v, 4), new=round(float(V.mean()), 4), from_r0_below_split=round(float(add.mean()), 4)); st["target_written"] += 1
    info = dict(made=time.strftime("%F %T"), inputs=dict(old=str(OLD / "s4m"), r15=str(OUT / "r15"), r0=str(OUT / "r0")),
                note="non-target kf = ffsonly_ba/s4m copied; target kf = r15 (s4m rule) where finite else r0 (Z >= 0.3, T bit 8)",
                counts=dict(st), nontarget_r15_vs_s4m_validmask_diff_mean=float(np.mean(diff_nt)) if diff_nt else None, per_target=per)
    (od / "combine_info.json").write_text(json.dumps(info, indent=1)); print(json.dumps({k: v for k, v in info.items() if k != "per_target"}, indent=1))


def mvfail(run, setdir):
    """gsm_ffs_4m cmd_measure mvcheck (pairs3 measure rule), per keyframe (v, u, z, fail) on the 2x grid; reference = the set itself."""
    import cv2, torch
    c = np.load(run / "dump_ba" / "cams.npz"); kf, T, Kr = c["kf"], c["T"], c["Kr"]; W, H = [int(v) for v in c["size"]]; n = len(kf)
    fx, fy, cx, cy = [float(v) for v in (Kr[0, 0], Kr[1, 1], Kr[0, 2], Kr[1, 2])]
    mx, my = cv2.initUndistortRectifyMap(c["K"], c["dist"], None, Kr, (W, H), cv2.CV_32FC1)
    vm = cv2.erode(((mx >= 0) & (mx <= W - 1) & (my >= 0) & (my <= H - 1)).astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)
    Z = np.full((n, H, W), np.nan, np.float32)
    for ci, i in enumerate(kf):
        f = setdir / f"{int(i):05d}.npz"
        if f.exists():
            Z[ci] = np.load(f)["Z"]
    href = np.isfinite(Z).reshape(n, -1).any(1)
    dev = "cuda"; Tw = torch.from_numpy(np.linalg.inv(T)).float().to(dev); Tc = torch.from_numpy(T).float().to(dev)
    rt = torch.from_numpy(Z).to(dev); outl = [None] * n; js = [j for j in range(n) if href[j]]; jt = torch.tensor(js, device=dev)
    for ci in range(n):
        Zs = Z[ci, ::2, ::2]; val = np.isfinite(Zs) & vm[::2, ::2]
        if not val.any():
            continue
        v, u = np.nonzero(val); z = torch.from_numpy(Zs[v, u]).float().to(dev)
        ut = torch.from_numpy(u * 2).float().to(dev); vt = torch.from_numpy(v * 2).float().to(dev)
        X = torch.stack([(ut - cx) / fx * z, (vt - cy) / fy * z, z], 1) @ Tc[ci, :3, :3].T + Tc[ci, :3, 3]
        viol = torch.zeros(len(z), dtype=torch.int32, device=dev); chk = torch.zeros_like(viol)
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
        outl[ci] = (v, u, Zs[v, u], ((viol >= 2) & (viol >= 0.3 * chk)).cpu().numpy())
    del rt; torch.cuda.empty_cache()
    return kf, Z, vm, outl


def cmd_measure(a):
    t0 = time.time(); res = dict(made=time.strftime("%F %T"), note="gsm_ffs_4m measure rule (2x grid, undistortion mask, each set its own reference, "
                                                                  "fail = violated in >= 2 views and >= 30 % of checking views); n = 1 recording", sets={})
    for nm, run, sd in (("s4m_old_judge_bap", BAP / "run", OLD / "s4m"), ("s4m_old_judge_ba2", BA2 / "run", OLD / "s4m"), ("s4m_r_judge_ba2", BA2 / "run", OUT / "s4m_r")):
        kf, Z, vm, outl = mvfail(run, sd); n = len(kf); tg = set(targets(kf))
        old = None
        if nm == "s4m_r_judge_ba2":
            old = np.full(Z.shape, False)
            for ci, i in enumerate(kf):
                f = OLD / "s4m" / f"{int(i):05d}.npz"
                if f.exists():
                    old[ci] = np.isfinite(np.load(f)["Z"])

        def pct(sel, newonly=False):
            fsum = tsum = 0
            for ci in range(n):
                if outl[ci] is None or not sel(ci):
                    continue
                v, u, z, fl = outl[ci]
                if newonly:
                    m = ~old[ci][v * 2, u * 2]; fl = fl[m]
                fsum += int(fl.sum()); tsum += len(fl)
            return dict(fail_pct=round(100 * fsum / max(1, tsum), 2), px_2x=tsum)
        V = np.isfinite(Z) & vm[None]; vf = np.isfinite(Z).reshape(n, -1).mean(1)
        e = dict(run=str(run), set=str(sd), all=pct(lambda ci: True), nontarget=pct(lambda ci: ci not in tg), target=pct(lambda ci: ci in tg),
                 aisle_314_331=pct(lambda ci: ci in AISLE),
                 usable_pct=dict(all=round(100 * float(vf.mean()), 2), target=round(100 * float(vf[sorted(tg)].mean()), 2),
                                 nontarget=round(100 * float(vf[[c for c in range(n) if c not in tg]].mean()), 2), aisle_314_331=round(100 * float(vf[AISLE].mean()), 2),
                                 target_lt2pct=int((vf[sorted(tg)] < 0.02).sum()), all_lt2pct=int((vf < 0.02).sum())))
        if old is not None:
            e["new_pixels_only"] = pct(lambda ci: True, newonly=True); e["new_pixels_target"] = pct(lambda ci: ci in tg, newonly=True)
            e["new_pixels_aisle"] = pct(lambda ci: ci in AISLE, newonly=True)
            zz = np.concatenate([Z[ci][np.isfinite(Z[ci]) & ~old[ci]] for ci in range(n)])
            e["new_pixels_depth_p10_50_90"] = [round(float(x), 3) for x in np.percentile(zz, [10, 50, 90])] if len(zz) else None
        res["sets"][nm] = e; print(nm, json.dumps(e), f"{time.time() - t0:.0f}s", flush=True)
    (OUT / "measure_relax.json").write_text(json.dumps(res, indent=1)); print("wrote", OUT / "measure_relax.json")


def cmd_fused(a):
    src = OUT / a.name; od = TRAIN / "fused"; od.mkdir(parents=True, exist_ok=True)
    kf = np.load(BA2 / "run" / "dump_ba" / "cams.npz")["kf"]; vf = []; drop = set(int(x) for x in a.drop.split(",") if x)
    revert = set(int(kf[int(x)]) for x in a.revert_idx.split(",") if x)
    for i in kf:
        f = src / f"{int(i):05d}.npz"
        if int(i) in revert:      # gate: keep only the old s4m depth for this keyframe (its new pairs dropped)
            f = OLD / "s4m" / f"{int(i):05d}.npz"
        if f.exists() and int(i) not in drop:
            d = np.load(f); Z = d["Z"].astype(np.float32); V = np.isfinite(Z) & (Z > 0); S = np.where(V, d["S"], np.inf).astype(np.float32)
        else:
            Z = np.full((540, 960), np.nan, np.float32); V = np.zeros_like(Z, bool); S = np.full_like(Z, np.inf)
        np.savez_compressed(od / f"{int(i):05d}.npz", Z=np.where(V, Z, 0).astype(np.float32), F=V.astype(np.uint8), S=S); vf.append(float(V.mean()))
    (od / "info.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), src=str(src), note="FFS only (F=1), no MoGe/DA3 fill; relax partners added",
                                                  n=len(kf), valid_mean=float(np.mean(vf)), kf_with_depth=int(sum(v > 0 for v in vf)), dropped_kf=sorted(drop),
                                                  reverted_kf_frames=sorted(revert), revert_rule="target kf idx whose NEW pixels fail > 30 % (measure rule): old s4m depth only"), indent=1))
    print("fused", od, np.mean(vf))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["plan", "frames", "ba", "post", "ffs", "fuse", "combine", "measure", "fused"])
    ap.add_argument("--win-s", type=float, default=4.0); ap.add_argument("--b-min", type=float, default=0.10); ap.add_argument("--b-max", type=float, default=0.50)
    ap.add_argument("--min-perp", type=float, default=10.0); ap.add_argument("--max-rot", type=float, default=25.0); ap.add_argument("--theta-min", type=float, default=25.0)
    ap.add_argument("--cap", type=float, default=0.03); ap.add_argument("--n-new", type=int, default=2); ap.add_argument("--min-score", type=float, default=0.03)
    ap.add_argument("--name", default="s4m_r"); ap.add_argument("--drop", default=""); ap.add_argument("--revert-idx", default="")
    sys.path.insert(0, str(HERE)); import rec_profile as RP; RP.add_args(ap)
    a = ap.parse_args()
    if a.cmd in ("ffs", "measure"):
        import gpu_guard
        with gpu_guard.job("gsmap ffs_relax " + a.cmd, torch_cap=True):
            {"ffs": cmd_ffs, "measure": cmd_measure}[a.cmd](a)
    else:
        {"plan": cmd_plan, "frames": cmd_frames, "ba": cmd_ba, "post": cmd_post, "fuse": cmd_fuse, "combine": cmd_combine, "fused": cmd_fused}[a.cmd](a)
