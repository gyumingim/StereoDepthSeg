#!/usr/bin/env python3
"""gsmap 2026-11-26: TEST ONLY - does FFS motion stereo at 1920x1080 give better depth than the current 960x540?

  FFS_ROOT=~/scanner/StereoDepthSeg nice -n 19 ~/scanner/StereoDepthSeg/venv_ffs/bin/python scan3d/gsmap/gsm_ffs1920_test.py
      --run scan3d/runs/gsmap_20261125_121801 --ref scan3d/runs/gsmap_20261125_121801_v2/eval2/ref --vio VIO.jsonl.gz
      --out scan3d/runs/gsmap_20261125_121801_v2/ffs1920test

Changes no defaults. Same partners as RUN/ffs_dense/stats.json (partner_frames), same partner-pose rule and the same
ms.pair_depth / ms.combine_motion calls as gsm_ffs_dense.py. 40 keyframes = 10 clusters of 4 (so every keyframe has 3
overlapping test keyframes); one cluster = the keyframes with dense FFS that see the centre of ROI cabinet_front_kf774
(kf774 itself has no FFS pair).
Settings: a960 = current (960x540, max_disp 192); b1920h = 1920x1080 hiera (run_hierachical small_ratio 0.5, model max_disp
192, pair scale budget 384); c1920p = 1920x1080 plain, model max_disp 384. Pair input scale s follows pair_depth's own rule
(0.92 * ffs_max_disp / d_need), so with budget 384 at 1920 the input is >= 2x the 960 input for every pair.
1920 outputs are on a 1920 grid with rect K = (2f, 2cx+0.5, 2cy+0.5) so a 2x2 block mean lands exactly on the 960 rect0 grid.
Metrics at 960: valid frac, depth range, multi-view agreement among the 40 (|dz| <= max(2 cm, 1 %)), free-space violation
(gsm_ffs_dense rule 3 cm / 3 %, >= 2 views and >= 30 %), agreement with eval2 gold FFS (480x270). Also 1920-native valid px.
"""
import argparse, functools, gzip, json, os, sys, time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / "gsplat"))
if os.environ.get("FFS_ROOT"):
    sys.path.insert(0, os.environ["FFS_ROOT"])
cv2.setNumThreads(8)

SETTINGS = [dict(name="a960", W=960, model_max_disp=192, budget=192, hiera=False),
            dict(name="b1920h", W=1920, model_max_disp=192, budget=384, hiera=True),
            dict(name="c1920p", W=1920, model_max_disp=384, budget=384, hiera=False)]


def pick_kf(stats, kf, T, Kr, ref):
    elig = [d["idx"] for d in stats["per_kf"] if d.get("pairs", 0) > 0 and d.get("partner_frames")]
    # cabinet/tub: 3D centre of ROI cabinet_front_kf774 from the 2DGS depth reference, keyframes with FFS that see it
    roi = [r for r in json.load(open(ref / "rois.json")) if r["name"] == "cabinet_front_kf774"][0]
    ZM = np.load(ref / "gs_depth_all.npz")["ZM"][roi["idx"]].astype(np.float32)
    x0, x1, y0, y1 = roi["rect"]; u, v = (x0 + x1) / 2, (y0 + y1) / 2
    z = float(np.nanmedian(ZM[y0 // 2:y1 // 2, x0 // 2:x1 // 2]))
    Pc = np.array([(u - Kr[0, 2]) / Kr[0, 0] * z, (v - Kr[1, 2]) / Kr[1, 1] * z, z, 1.0]); Pw = T[roi["idx"]] @ Pc
    sc = []
    for ci in elig:
        pj = np.linalg.inv(T[ci]) @ Pw
        if 0.3 < pj[2] < 2.5:
            pu, pv = Kr[0, 0] * pj[0] / pj[2] + Kr[0, 2], Kr[1, 1] * pj[1] / pj[2] + Kr[1, 2]
            if 60 < pu < 900 and 60 < pv < 480:
                sc.append((np.hypot(pu - 480, pv - 270), ci))
    sc.sort(); tub = sorted(ci for _, ci in sc[:4])
    rest = [ci for ci in elig if ci not in tub]
    clusters = [tub]
    for c in np.linspace(0, len(rest) - 4, 9).round().astype(int):
        cl = rest[c:c + 4]
        if any(ci in sum(clusters, []) for ci in cl):
            cl = [ci for ci in rest[c:] if ci not in sum(clusters, [])][:4]
        clusters.append(cl)
    return clusters, dict(roi_point_w=Pw[:3].tolist(), roi_z=z, tub_candidates=[(round(d, 1), int(kf[ci])) for d, ci in sc[:8]])


def down2(Z):
    H, W = Z.shape; b = Z.reshape(H // 2, 2, W // 2, 2); ok = np.isfinite(b); n = ok.sum((1, 3))
    with np.errstate(invalid="ignore"):
        m = np.where(ok, b, 0).sum((1, 3)) / np.maximum(n, 1)
    return np.where(n >= 2, m, np.nan).astype(np.float32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True); ap.add_argument("--ref", required=True); ap.add_argument("--vio", required=True)
    ap.add_argument("--rec", default=None); ap.add_argument("--gt", default="lensonly"); ap.add_argument("--out", required=True)
    ap.add_argument("--zmax", type=float, default=2.0); ap.add_argument("--iters", type=int, default=8)
    ap.add_argument("--only", default="")
    a = ap.parse_args()
    import motion_stereo as ms, lib_ffs, torch
    t0 = time.time(); run, ref, out = Path(a.run), Path(a.ref), Path(a.out); out.mkdir(parents=True, exist_ok=True)
    rec = Path(a.rec) if a.rec else run / "rec"
    stats = json.load(open(run / "ffs_dense" / "stats.json")); sa = stats["args"]
    c = np.load(run / "dump_ba" / "cams.npz"); kf, Tk, Kr = c["kf"], c["T"], c["Kr"]; W0, H0 = [int(v) for v in c["size"]]
    g = np.load(rec / a.gt / "gt.npz"); t_ns = g["t_ns"]; Kraw = g["K"].astype(np.float64); dist = g["dist"].astype(np.float64)
    clusters, pinfo = pick_kf(stats, kf, Tk, Kr, ref); sel = sum(clusters, [])
    print("clusters (kf):", [[int(kf[ci]) for ci in cl] for cl in clusters], flush=True)
    vt, vT = [], []
    for line in gzip.open(a.vio, "rt"):
        d = json.loads(line)
        if d.get("type") == "pose" and d.get("T_world_cam0") and d.get("status") == "tracking":
            vt.append(d["t_ns"]); vT.append(np.array(d["T_world_cam0"], float).reshape(4, 4))
    vt = np.array(vt); vT = np.array(vT)
    idx = np.searchsorted(t_ns, vt).clip(1, len(t_ns) - 1); idx = np.where(np.abs(t_ns[idx - 1] - vt) < np.abs(t_ns[idx] - vt), idx - 1, idx)
    good = np.abs(t_ns[idx] - vt) < 8e6; Tv = {int(i): T for i, T in zip(idx[good], vT[good])}
    pairs = {}
    for ci in sel:
        fi = int(kf[ci]); A = Tk[ci] @ np.linalg.inv(Tv[fi])
        pairs[ci] = [(j, A @ Tv[j]) for j in stats["per_kf"][ci]["partner_frames"]]
    need = set(int(kf[ci]) for ci in sel) | set(j for ci in sel for j, _ in pairs[ci])
    cap = cv2.VideoCapture(str(rec / "cam5.mp4")); fr = {}; fidx = -1
    while fidx < max(need):
        r_, f = cap.read(); fidx += 1
        if not r_:
            break
        if fidx in need:
            fr[fidx] = f
    cap.release(); print(f"decoded {len(fr)} frames {next(iter(fr.values())).shape}, {time.time() - t0:.0f}s", flush=True)
    cfg = ms.MSConfig(); cfg.b_target_m = 0.08; cfg.z_max = a.zmax
    cfg.dt_max_s = sa["win_s"]; cfg.b_min_m = sa["b_min"]; cfg.b_max_m = sa["b_max"]; cfg.min_perp_deg = sa["min_perp"]; cfg.max_rot_deg = sa["max_rot"]
    K960 = Kraw.copy(); K960[:2] *= W0 / 1920.0
    rp960 = dict(size=(W0, H0), f=float(K960[0, 0]), cx=float(K960[0, 2]), cy=float(K960[1, 2]), R1=np.eye(3).tolist())
    rp1920 = dict(size=(2 * W0, 2 * H0), f=2 * rp960["f"], cx=2 * rp960["cx"] + 0.5, cy=2 * rp960["cy"] + 0.5, R1=np.eye(3).tolist())
    res_all = {}
    for S in SETTINGS:
        if a.only and S["name"] not in a.only.split(","):
            continue
        sd = out / S["name"]; sd.mkdir(exist_ok=True)
        torch.cuda.empty_cache(); torch.cuda.reset_peak_memory_stats()
        model = lib_ffs.load(valid_iters=a.iters, max_disp=S["model_max_disp"])
        cfg.ffs_max_disp = S["budget"]
        infer = functools.partial(lib_ffs.infer, hiera=True) if S["hiera"] else lib_ffs.infer
        big = S["W"] == 1920; K1 = Kraw if big else K960; rp = rp1920 if big else rp960
        imgs = {k: (v if big else cv2.resize(v, (W0, H0), interpolation=cv2.INTER_AREA)) for k, v in fr.items()}
        Z960, nat, tpair, tkf, scales, err = {}, {}, [], [], [], None
        # warm-up (torch.compile / cudnn autotune) on the first pair, not timed
        ci0 = sel[0]; j0, Tj0 = pairs[ci0][0]; R, t = ms.rel_pose(Tk[ci0], Tj0)
        try:
            ms.pair_depth(model, infer, imgs[int(kf[ci0])], imgs[j0], R, t, K1, dist, rp, cfg, iters=a.iters)
        except RuntimeError as e:
            err = str(e)[:300]
        for ci in (sel if err is None else []):
            fi = int(kf[ci]); res = []; t1 = time.time()
            try:
                for j, Tj in pairs[ci]:
                    R, t = ms.rel_pose(Tk[ci], Tj); torch.cuda.empty_cache(); torch.cuda.synchronize(); tp = time.time()   # empty_cache: try1 OOM at 1642x923 from fragmentation (11.7 GB reserved)
                    r = ms.pair_depth(model, infer, imgs[fi], imgs[j], R, t, K1, dist, rp, cfg, iters=a.iters)
                    torch.cuda.synchronize(); tpair.append(time.time() - tp); scales.append(r["scale"]); res.append(r)
            except RuntimeError as e:
                err = f"kf {fi}: {str(e)[:300]}"; print("FAIL", S["name"], err, flush=True); break
            Z, V, Sg, agree = ms.combine_motion(res, cfg); tkf.append(time.time() - t1)
            Z = np.where(V, Z, np.nan).astype(np.float32)
            nat[ci] = int(V.sum())
            Z960[ci] = down2(Z) if big else Z
            np.savez_compressed(sd / f"{fi:05d}.npz", Z=Z.astype(np.float16), Z960=Z960[ci].astype(np.float16))
        peak = dict(alloc_GB=round(torch.cuda.max_memory_allocated() / 2 ** 30, 2), reserved_GB=round(torch.cuda.max_memory_reserved() / 2 ** 30, 2))
        res_all[S["name"]] = dict(S=S, Z=Z960, nat=nat, tpair=tpair, tkf=tkf, scales=scales, err=err, peak=peak)
        print(S["name"], "pairs", len(tpair), "median s/pair", np.median(tpair) if tpair else None, peak, err, flush=True)
        del model; lib_ffs._MODEL.clear(); torch.cuda.empty_cache()
    # ---- metrics at 960
    dev = "cuda"; mx, my = cv2.initUndistortRectifyMap(K960, dist, None, Kr, (W0, H0), cv2.CV_32FC1)
    vm = cv2.erode(((mx >= 0) & (mx <= W0 - 1) & (my >= 0) & (my <= H0 - 1)).astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)
    gold = np.load(ref / "gold_ffs.npz")["Z"]
    fx, fy, cx, cy = [float(v) for v in (Kr[0, 0], Kr[1, 1], Kr[0, 2], Kr[1, 2])]
    Tw = {ci: torch.from_numpy(np.linalg.inv(Tk[ci])).float().to(dev) for ci in sel}; Tc = {ci: torch.from_numpy(Tk[ci]).float().to(dev) for ci in sel}
    old = {ci: np.load(run / "ffs_dense" / f"{int(kf[ci]):05d}.npz")["Z"] for ci in sel}
    summ = dict(made=time.strftime("%F %T"), inputs=dict(run=str(run), ref=str(ref), vio=a.vio, rec=str(rec), stats=str(run / "ffs_dense" / "stats.json")),
                clusters_kf=[[int(kf[ci]) for ci in cl] for cl in clusters], pick=pinfo, n_pairs=sum(len(pairs[ci]) for ci in sel),
                plan_pairs_full_run=int(sum(d.get("pairs", 0) for d in stats["per_kf"])), plan_kf_full_run=int(sum(d.get("pairs", 0) > 0 for d in stats["per_kf"])),
                gpu_cap_note="gpu_guard torch cap 0.75 x 15.9 GB = 11.9 GB", settings={})
    for name, R_ in res_all.items():
        m = dict(setting=R_["S"], error=R_["err"], peak_vram=R_["peak"], n_pairs_done=len(R_["tpair"]))
        if R_["tpair"]:
            m.update(s_per_pair_median=round(float(np.median(R_["tpair"])), 3), s_per_pair_mean=round(float(np.mean(R_["tpair"])), 3),
                     s_per_kf_mean=round(float(np.mean(R_["tkf"])), 3), input_scale_median=float(np.median(R_["scales"])),
                     est_full_run_min=round(float(np.mean(R_["tpair"])) * summ["plan_pairs_full_run"] / 60, 1),
                     est_full_run_min_if_3_pairs_every_kf=round(float(np.mean(R_["tpair"])) * 3 * len(kf) / 60, 1))
        Zs = R_["Z"]
        if len(Zs) == len(sel):
            vf = [float(np.isfinite(Zs[ci]).mean()) for ci in sel]
            lo = [float(np.nanpercentile(Zs[ci], 2)) for ci in sel if np.isfinite(Zs[ci]).any()]
            hi = [float(np.nanpercentile(Zs[ci], 98)) for ci in sel if np.isfinite(Zs[ci]).any()]
            near = sum(int((Zs[ci] < 0.7).sum()) for ci in sel)
            m.update(valid_frac_median=round(float(np.median(vf)), 4), valid_px_960=int(sum(np.isfinite(Zs[ci]).sum() for ci in sel)),
                     valid_px_native=int(sum(R_["nat"].values())), depth_p2_median=round(float(np.median(lo)), 3), depth_p98_median=round(float(np.median(hi)), 3),
                     near_px_below_0p7m=near)
            ffs = {ci: torch.from_numpy(Zs[ci]).to(dev) for ci in sel}
            chk_t = agr_t = pts = conf2 = bad = 0
            for cl in clusters:
                for ci in cl:
                    Z = Zs[ci]; val = np.isfinite(Z) & vm
                    if not val.any():
                        continue
                    v, u = np.nonzero(val); z = torch.from_numpy(Z[v, u]).float().to(dev)
                    X = torch.stack([(torch.from_numpy(u).float().to(dev) - cx) / fx * z, (torch.from_numpy(v).float().to(dev) - cy) / fy * z, z], 1) @ Tc[ci][:3, :3].T + Tc[ci][:3, 3]
                    chk = torch.zeros(len(z), dtype=torch.int32, device=dev); agr = torch.zeros_like(chk); viol = torch.zeros_like(chk)
                    for j in sel:
                        if j == ci:
                            continue
                        Xj = X @ Tw[j][:3, :3].T + Tw[j][:3, 3]; zj = Xj[:, 2]
                        pu = torch.round(fx * Xj[:, 0] / zj.clamp(min=1e-3) + cx).long(); pv = torch.round(fy * Xj[:, 1] / zj.clamp(min=1e-3) + cy).long()
                        ok = (zj > 0.05) & (pu >= 0) & (pu < W0) & (pv >= 0) & (pv < H0)
                        zr = torch.full_like(zj, float("nan")); zr[ok] = ffs[j][pv[ok], pu[ok]]
                        f_ = torch.isfinite(zr); chk += f_.int()
                        agr += (f_ & (torch.abs(zj - zr) <= torch.clamp(0.01 * zr, min=0.02))).int()
                        viol += (f_ & (zj < zr - torch.clamp(0.03 * zr, min=0.03))).int()
                    chk_t += int(chk.sum()); agr_t += int(agr.sum()); pts += len(z)
                    conf2 += int((agr >= 2).sum()); bad += int(((viol >= 2) & (viol >= 0.3 * chk)).sum())
            m.update(mv_checks=chk_t, mv_agree_rate=round(agr_t / max(1, chk_t), 4), mv_px_agree_ge2=conf2,
                     mv_px_agree_ge2_frac=round(conf2 / max(1, pts), 4), mv_freespace_fail_pct=round(100 * bad / max(1, pts), 2))
            go = gn = 0; gd = []
            for ci in sel:
                z2 = Zs[ci][::2, ::2]; gz = gold[ci].astype(np.float32); both = np.isfinite(z2) & (gz > 0)
                gn += int(both.sum()); dz = np.abs(z2[both] - gz[both]); go += int((dz <= np.maximum(0.02, 0.01 * gz[both])).sum()); gd.append(dz)
            gd = np.concatenate(gd) if gd else np.zeros(0)
            m.update(gold_overlap_px_480=gn, gold_px_total_480=int(sum((gold[ci] > 0).sum() for ci in sel)), gold_agree_rate=round(go / max(1, gn), 4),
                     gold_absdiff_median_cm=round(100 * float(np.median(gd)), 2) if len(gd) else None)
            if name == "a960":   # reproduction check vs the existing ffs_dense output
                same = [np.nanmax(np.abs(Zs[ci] - old[ci])) if (np.isfinite(Zs[ci]) & np.isfinite(old[ci])).any() else 0 for ci in sel]
                iou = [float((np.isfinite(Zs[ci]) & np.isfinite(old[ci])).sum() / max(1, (np.isfinite(Zs[ci]) | np.isfinite(old[ci])).sum())) for ci in sel]
                m.update(repro_vs_ffs_dense=dict(max_absdiff_m=float(np.max(same)), valid_iou_min=round(min(iou), 4)))
        summ["settings"][name] = m
    (out / "metrics.json").write_text(json.dumps(summ, indent=1, default=float))
    print(json.dumps({k: {kk: vv for kk, vv in v.items() if kk != "setting"} for k, v in summ["settings"].items()}, indent=1, default=float), flush=True)
    # ---- side-by-side: tub keyframe + 3 from other clusters
    names = [n for n in ("a960", "b1920h", "c1920p") if n in res_all and len(res_all[n]["Z"]) == len(sel)]
    show = [clusters[0][0], clusters[3][1], clusters[6][1], clusters[9][1]]
    rows = []
    for ci in show:
        Za = res_all["a960"]["Z"][ci] if "a960" in names else None
        lo, hi = (np.nanpercentile(Za, 2), np.nanpercentile(Za, 98)) if Za is not None and np.isfinite(Za).any() else (0.3, 2.0)
        rgb = cv2.remap(cv2.resize(fr[int(kf[ci])], (W0, H0), interpolation=cv2.INTER_AREA), mx, my, cv2.INTER_LINEAR)
        tiles = [cv2.resize(rgb, (480, 270))]; cv2.putText(tiles[0], f"kf {int(kf[ci])}", (8, 26), 0, 0.8, (255, 255, 255), 2)
        for n in names:
            Z = res_all[n]["Z"][ci]; q = np.clip((Z - lo) / max(hi - lo, 1e-3) * 255, 0, 255); q = np.nan_to_num(q, nan=0).astype(np.uint8)
            im = cv2.applyColorMap(q, cv2.COLORMAP_TURBO); im[~np.isfinite(Z)] = 0; im = cv2.resize(im, (480, 270), interpolation=cv2.INTER_NEAREST)
            cv2.putText(im, f"{n} valid {np.isfinite(Z).mean():.2f}", (8, 26), 0, 0.7, (255, 255, 255), 2); tiles.append(im)
        rows.append(np.hstack(tiles))
    cv2.imwrite(str(out / "ffs1920_cmp.jpg"), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 88])
    print(f"done {time.time() - t0:.0f}s", flush=True)


if __name__ == "__main__":
    import gpu_guard
    with gpu_guard.job("gsmap ffs1920test " + sys.argv[sys.argv.index("--out") + 1], torch_cap=True):
        main()
