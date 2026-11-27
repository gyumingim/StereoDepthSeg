#!/usr/bin/env python3
"""gsmap 2026-11-25: FFS motion-stereo depth for every keyframe with partners from the FULL 60 fps video (not only keyframes).

  FFS_ROOT=~/scanner/StereoDepthSeg ~/scanner/StereoDepthSeg/venv_ffs/bin/python scan3d/gsmap/gsm_ffs_dense.py RUN --vio VIO.jsonl.gz
      --rec REC --gt lensonly --out RUN/ffs_dense [--win-s 2] [--n-pairs 3] [--b-min 0.03 --b-max 0.25 --min-perp 30]

Partner pose: T_partner = T_kf(dump, BA) @ inv(Tvio_kf) @ Tvio_partner (VIO relative motion over <= 2 s, anchored on the BA pose).
Same motion_stereo calls as scan_offline_dump.py (score_pair, dir_differs, pair_depth incl. its dy gate, combine_motion),
config relaxed: dt <= --win-s, baseline --b-min..--b-max, min_perp --min-perp, up to --n-pairs partners.
Frames are decoded once, streamed (only frames within the window of a keyframe are kept).
Writes OUT/<kf>.npz (Z, V, S like the dump) + OUT/stats.json (per kf: pairs, valid_frac, no-candidate reasons) and a
multi-view free-space check of the new FFS (same rule as gsm_filtmesh: >= 2 views and >= 30 % of checkable, 3 cm / 3 %).
"""
import argparse, gzip, json, os, sys, time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / "gsplat"))
if os.environ.get("FFS_ROOT"):
    sys.path.insert(0, os.environ["FFS_ROOT"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run"); ap.add_argument("--vio", required=True); ap.add_argument("--rec", required=True); ap.add_argument("--gt", default="lensonly")
    ap.add_argument("--out", required=True); ap.add_argument("--win-s", type=float, default=2.0); ap.add_argument("--n-pairs", type=int, default=3)
    ap.add_argument("--b-min", type=float, default=0.03); ap.add_argument("--b-max", type=float, default=0.25)
    ap.add_argument("--min-perp", type=float, default=30.0); ap.add_argument("--max-rot", type=float, default=25.0)
    ap.add_argument("--zmax", type=float, default=2.0); ap.add_argument("--iters", type=int, default=8)
    ap.add_argument("--fps", type=float, default=0.0, help="0 = from the recording (meta.json checked by frame timestamps)")
    a = ap.parse_args()
    import motion_stereo as ms, lib_ffs, torch
    t0 = time.time()
    run, out = Path(a.run), Path(a.out); out.mkdir(parents=True, exist_ok=True)
    c = np.load(run / "dump_ba" / "cams.npz"); kf, Tk, Kr = c["kf"], c["T"], c["Kr"]; W0, H0 = [int(v) for v in c["size"]]
    g = np.load(Path(a.rec) / a.gt / "gt.npz"); t_ns = g["t_ns"]; K = g["K"].copy(); K[:2] *= W0 / 1920.0; dist = g["dist"].astype(np.float64)
    # VIO pose per frame (same nearest-time rule as the dump)
    vt, vT = [], []
    for line in gzip.open(a.vio, "rt"):
        d = json.loads(line)
        if d.get("type") == "pose" and d.get("T_world_cam0") and d.get("status") == "tracking":
            vt.append(d["t_ns"]); vT.append(np.array(d["T_world_cam0"], float).reshape(4, 4))
    vt = np.array(vt); vT = np.array(vT)
    idx = np.searchsorted(t_ns, vt).clip(1, len(t_ns) - 1); idx = np.where(np.abs(t_ns[idx - 1] - vt) < np.abs(t_ns[idx] - vt), idx - 1, idx)
    good = np.abs(t_ns[idx] - vt) < 8e6
    Tv = {int(i): T for i, T in zip(idx[good], vT[good])}
    cfg = ms.MSConfig(); cfg.b_target_m = 0.08; cfg.z_max = a.zmax
    cfg.dt_max_s = a.win_s; cfg.b_min_m = a.b_min; cfg.b_max_m = a.b_max; cfg.min_perp_deg = a.min_perp; cfg.max_rot_deg = a.max_rot
    rp = dict(size=(W0, H0), f=float(K[0, 0]), cx=float(K[0, 2]), cy=float(K[1, 2]), R1=np.eye(3).tolist())
    import rec_profile as RP
    fps = RP.rec_fps(a.rec, t_ns, a.fps); win = RP.frames(a.win_s, fps)       # was win_s * 60 (2026-11-27)
    plan, stats = {}, {}
    for ci, fi in enumerate(kf):
        fi = int(fi); st = dict(kf=fi, idx=ci)
        if fi not in Tv:
            st.update(pairs=0, why="no VIO pose at the keyframe"); stats[ci] = st; continue
        A = Tk[ci] @ np.linalg.inv(Tv[fi]); cands, why = [], Counter()
        for j in range(max(0, fi - win), min(len(t_ns), fi + win + 1)):
            if j == fi or j not in Tv:
                continue
            Tj = A @ Tv[j]; R, t = ms.rel_pose(Tk[ci], Tj); gg = ms.pair_geometry(R, t)
            s, w = ms.score_pair(gg, abs(t_ns[fi] - t_ns[j]) * 1e-9, cfg); why[w] += 1
            if s is not None:
                cands.append((s, j, gg, Tj))
        cands.sort(key=lambda x: x[0]); chosen = []
        for s, j, gg, Tj in cands:
            if not chosen or all(ms.dir_differs(gg, q[2], cfg.pair2_min_dir_deg) for q in chosen):
                chosen.append((s, j, gg, Tj))
            if len(chosen) >= a.n_pairs:
                break
        st.update(n_cand=len(cands), why=dict(why), max_B_cm=round(max([0] + [q[2]["B"] * 100 for q in cands]), 1))
        if chosen:
            plan[ci] = chosen
        else:
            st["pairs"] = 0
        stats[ci] = st
    need = {}
    for ci, ch in plan.items():
        for q in ch:
            need.setdefault(q[1], []).append(ci)
        need.setdefault(int(kf[ci]), []).append(ci)
    print(f"plan: {len(plan)}/{len(kf)} keyframes with partners, {len(need)} frames to decode, {time.time() - t0:.1f}s", flush=True)
    model = lib_ffs.load(valid_iters=a.iters) if len(plan) else None
    last_use = {f: max(int(kf[ci]) + win for ci in cis) for f, cis in need.items()}
    done = []
    for ci in list(plan):   # resume: keyframes whose npz exists and loads are not recomputed
        f_ = out / f"{int(kf[ci]):05d}.npz"
        if f_.exists():
            try:
                Vd = np.load(f_)["V"]
                stats[ci].update(pairs=len(plan[ci]), valid_frac=float(Vd.mean()), partner_frames=[q[1] for q in plan[ci]], resumed=True)
                done.append(ci)
            except Exception:
                f_.unlink()
    print(f"resume: {len(done)} keyframes already done", flush=True)
    todo = sorted([ci for ci in plan if ci not in done], key=lambda ci: max([int(kf[ci])] + [q[1] for q in plan[ci]]))
    ready_at = {ci: max([int(kf[ci])] + [q[1] for q in plan[ci]]) for ci in plan}
    cap = cv2.VideoCapture(str(Path(a.rec) / "cam5.mp4")); buf = {}; fidx = -1; ti = 0; t_ffs = 0.0
    while ti < len(todo):
        r_, f = cap.read(); fidx += 1
        if not r_:
            break
        if fidx in need:
            buf[fidx] = cv2.resize(f, (W0, H0), interpolation=cv2.INTER_AREA) if f.shape[1] != W0 else f
        while ti < len(todo) and ready_at[todo[ti]] <= fidx:
            ci = todo[ti]; ti += 1; fi = int(kf[ci]); res = []; t1 = time.time()
            for s, j, gg, Tj in plan[ci]:
                R, t = ms.rel_pose(Tk[ci], Tj)
                res.append(ms.pair_depth(model, lib_ffs.infer, buf[fi], buf[j], R, t, K, dist, rp, cfg, iters=a.iters))
            Z, V, S, agree = ms.combine_motion(res, cfg)
            t_ffs += time.time() - t1
            np.savez_compressed(out / f"{fi:05d}.npz", Z=np.where(V, Z, np.nan).astype(np.float32), V=V, S=np.where(V, S, np.inf).astype(np.float32))
            stats[ci].update(pairs=len(res), valid_frac=float(V.mean()), B_cm=[round(r["B"] * 100, 1) for r in res],
                             partner_frames=[q[1] for q in plan[ci]])
            if ti % 25 == 0:
                print(f"{ti}/{len(todo)} kf {fi} pairs {len(res)} valid {V.mean():.3f} t {time.time() - t0:.0f}s", flush=True)
        for fdel in [k for k in buf if last_use[k] < fidx]:
            del buf[fdel]
    cap.release()
    t_total_ffs = time.time() - t0
    # ---- multi-view free-space check of the new FFS (as gsm_filtmesh)
    dev = "cuda"; n = len(kf)
    FF = []
    for i in kf:
        p = out / f"{int(i):05d}.npz"
        FF.append(np.load(p)["Z"] if p.exists() else np.full((H0, W0), np.nan, np.float32))
    mx, my = cv2.initUndistortRectifyMap(K, dist, None, Kr, (W0, H0), cv2.CV_32FC1)
    vm = cv2.erode(((mx >= 0) & (mx <= W0 - 1) & (my >= 0) & (my <= H0 - 1)).astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)
    ffs = torch.from_numpy(np.stack(FF)).to(dev); has = [bool(np.isfinite(z).any()) for z in FF]
    Tw = torch.from_numpy(np.linalg.inv(Tk)).float().to(dev); Tc = torch.from_numpy(Tk).float().to(dev)
    fx, fy, cx, cy = [float(v) for v in (Kr[0, 0], Kr[1, 1], Kr[0, 2], Kr[1, 2])]
    tot, bad = 0, 0
    for ci in range(n):
        Z = FF[ci]; val = np.isfinite(Z) & vm
        if not val.any():
            continue
        v, u = np.nonzero(val); z = torch.from_numpy(Z[v, u]).float().to(dev)
        X = torch.stack([(torch.from_numpy(u).float().to(dev) - cx) / fx * z, (torch.from_numpy(v).float().to(dev) - cy) / fy * z, z], 1) @ Tc[ci, :3, :3].T + Tc[ci, :3, 3]
        viol = torch.zeros(len(z), dtype=torch.int32, device=dev); chk = torch.zeros_like(viol)
        for j in range(n):
            if j == ci or not has[j]:
                continue
            Xj = X @ Tw[j, :3, :3].T + Tw[j, :3, 3]; zj = Xj[:, 2]
            pu = torch.round(fx * Xj[:, 0] / zj.clamp(min=1e-3) + cx).long(); pv = torch.round(fy * Xj[:, 1] / zj.clamp(min=1e-3) + cy).long()
            ok = (zj > 0.05) & (pu >= 0) & (pu < W0) & (pv >= 0) & (pv < H0)
            zr = torch.full_like(zj, float("nan")); zr[ok] = ffs[j][pv[ok], pu[ok]]
            chk += torch.isfinite(zr).int(); viol += (torch.isfinite(zr) & (zj < zr - torch.clamp(0.03 * zr, min=0.03))).int()
        bad += int(((viol >= 2) & (viol >= 0.3 * chk)).sum()); tot += len(z)
        stats[ci]["mv_fail_frac"] = float(((viol >= 2) & (viol >= 0.3 * chk)).float().mean())
    vf = [s.get("valid_frac", 0.0) for s in stats.values()]
    summ = dict(made=time.strftime("%F %T"), args=vars(a), n_kf=n, n_with_ffs=int(sum(s.get("pairs", 0) > 0 for s in stats.values())),
                valid_frac_median_all=float(np.median(vf)), valid_frac_median_with_ffs=float(np.median([x for x in vf if x > 0])) if any(vf) else 0,
                mv_fail_pct=round(100 * bad / max(1, tot), 2), ffs_s=round(t_ffs, 1), total_s_ffs_stage=round(t_total_ffs, 1),
                total_s=round(time.time() - t0, 1), per_kf=[stats[ci] for ci in range(n)])
    (out / "stats.json").write_text(json.dumps(summ, indent=1, default=float))
    print(json.dumps({k: v for k, v in summ.items() if k != "per_kf"}, default=float), flush=True)


if __name__ == "__main__":
    import gpu_guard
    with gpu_guard.job("gsmap ffs_dense " + sys.argv[sys.argv.index("--out") + 1], torch_cap=True):
        main()
