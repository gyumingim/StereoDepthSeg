#!/usr/bin/env python3
"""scanA 2026-11-05 step 3 (from scanplan/e2e_mono_ms.py, same depth/fusion code; changes: pooled + per-keyframe depth error
stats incl. mean/p10/p90, --voxel default 5 mm, coverage cell fix, OBJ export, --kf-only uses poses only at keyframes).
Original docstring: one quick end-to-end attempt of the scanner on a mono 60 fps recording (hs std60, lens 5 1920x1080).

record (cam5.mp4) -> poses (default: board GT imu_gt_spl = best case; VIO poses are the realistic case, not done here)
-> two-frame motion stereo of the same lens (motion_stereo.pair_depth, FFS, no two-lens anchor) -> TSDF (tsdf_map) -> mesh.ply.
Checks (the ChArUco board on the laptop screen is the known object; world = board frame, board plane z = 0):
  * per-keyframe depth error inside the board rectangle vs the GT plane depth (cm, median / p90 of |dZ|)
  * mesh vertices over the board rectangle: z offset and RMS from the plane (cm)
Usage: venv_ffs/bin/python scan3d/scanplan/e2e_mono_ms.py <rec dir> --out <dir> [--scale 0.5] [--kf-step 10]
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parent.parent))
import os  # noqa: E402
if os.environ.get("FFS_ROOT"):            # repo root that holds tools/Fast-FoundationStereo (worktrees do not)
    sys.path.insert(0, os.environ["FFS_ROOT"])
import motion_stereo as ms  # noqa: E402
import tsdf_map  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("--out", required=True)
    ap.add_argument("--gt", default="imu_gt_spl")
    ap.add_argument("--scale", type=float, default=0.5, help="image scale vs 1920x1080")
    ap.add_argument("--kf-step", type=int, default=10)
    ap.add_argument("--n-pairs", type=int, default=2)
    ap.add_argument("--iters", type=int, default=8)
    ap.add_argument("--voxel", type=float, default=0.005)
    ap.add_argument("--zmax", type=float, default=2.0)
    ap.add_argument("--tsdf-max-blocks", type=int, default=60000,
                    help="TSDF block limit (8^3 voxels each, ~6.0 KiB GPU per block; default 60000 = tsdf_map default, ~0.37 GB). "
                         "fixes 11-12 C: a room scan at 5 mm hits 60000 (runs/fixes_20261112 README)")
    # fixes 11-12 unit 12: TSDF block eviction to a disk spill folder (opt-in; 0 = off = old path unchanged). tsdf_evict.py
    ap.add_argument("--tsdf-evict-gpu-blocks", type=int, default=0,
                    help="> 0: keep at most N blocks on the GPU, least recently touched blocks go to a disk spill (numpy memmap) and "
                         "come back when seen again; --tsdf-max-blocks then limits ALL blocks (GPU + disk). 0 = off (default)")
    ap.add_argument("--tsdf-evict-dtype", choices=("f32", "f16"), default="f32",
                    help="spill sdf/w as float32 (exact, mesh = no-evict mesh) or float16 (4.0 instead of 6.0 KiB per block, lossy)")
    ap.add_argument("--tsdf-evict-dir", default="", help="spill folder (default OUT/tsdf_spill; deleted after a successful run)")
    ap.add_argument("--tsdf-evict-keep", action="store_true", help="keep the spill folder")
    ap.add_argument("--b-target", type=float, default=0.08)
    ap.add_argument("--vio", default="", help="VIO poses.jsonl.gz (T_world_cam0 per frame); rigid SE(3) align to GT (scale NOT fixed)")
    ap.add_argument("--free-scale", action="store_true",
                    help="ORACLE global scale: Sim(3) of the VIO positions to GT, translations x s (depth scales with the baseline)")
    ap.add_argument("--pose-noise-cm", type=float, default=0.0, help="add N(0, s) cm position noise per frame (VIO-like test)")
    ap.add_argument("--allow-fullres", action="store_true",
                    help="allow --scale > 0.5 (1920x1080 FFS needs > 16 GB; runs only under the gpu_guard cap, fails cleanly)")
    ap.add_argument("--rows-from", default="", help="poses.jsonl.gz: keep only recording rows that have a pose in this file "
                    "(t_ns within 1 ms), e.g. the BA keyframes, so a GT-pose run uses the same keyframes")
    ap.add_argument("--compress", action="store_true",
                    help="HS 240 fps recordings: keep only rows with a pose (RAM), --kf-step counts kept rows; pair search "
                         "window +-150 kept rows (dt still capped by MSConfig.dt_max_s)")
    ap.add_argument("--tsdf-alloc", choices=("sparse", "dense"), default="sparse",
                    help="TSDF block allocation: sparse = 3 points z-mu, z, z+mu (tsdf_map default, old); dense = the whole [-mu, +mu] "
                         "band every half block (ghost 11-11: no ghost double layer when mu > block; run_scanner.sh passes dense)")
    ap.add_argument("--ffs-views", type=int, default=None,
                    help="images per keyframe depth map = reference + (N-1) partner keyframes (= --n-pairs + 1; default "
                         "n-pairs 2 -> 3 = old behaviour). Partners 1..2 = old rule; partners 3.. = extra views (pick_extra)")
    ap.add_argument("--ffs-fuse", choices=("auto", "mean", "median"), default="auto",
                    help="combine the per-pair depths: mean = motion_stereo.combine_motion (old: inverse-variance mean, drop pixels "
                         "where a pair is > 3 sigma off); median = fuse_median (median, keep pairs within 3 sigma of it, need 2 "
                         "agreeing pairs where >= 2 are valid); auto = mean for <= 2 pairs (old), median for >= 3")
    ap.add_argument("--ffs-partners", choices=("near", "all"), default="near",
                    help="partner search: near = old (+-150 kept rows and dt 0.15-2.5 s: time neighbours only); all = every keyframe "
                         "(no dt cap, no window; same baseline / angle / rotation limits). pair_diag.py 133820: near 81/342 keyframes "
                         "have a partner (dt cap leaves baselines < 4 cm), all 313/342")
    ap.add_argument("--ffs-dt-max", type=float, default=None,
                    help="with --ffs-partners all: max time between reference and partner (s, default 30; 1e9 = no cap). "
                         "near mode keeps MSConfig.dt_max_s 2.5")
    ap.add_argument("--pair-min-valid", type=float, default=None,
                    help="pair gate: drop a partner whose FFS depth is valid on < F of the image and try the next candidate "
                         "(up to MSConfig.max_tries = 4 drops per keyframe). Default 0 = off. Tried 0.2 with --ffs-partners all on "
                         "133820 (fv_g*): worse (380 drops, fewer keyframes, double layer 0.25) -> replaced by --pair-check dy")
    ap.add_argument("--pair-check", choices=("auto", "none", "dy"), default="auto",
                    help="check a partner before FFS: dy = SIFT rectification check (dy_ok); auto = dy with --ffs-partners all, "
                         "none (old) with near")
    ap.add_argument("--keep-gt-not-ok", action="store_true",
                    help="fixes 11-12 M1: with --vio, also use keyframes whose GT row is not 'ok' (board not seen / GT fit failed). "
                         "Default (off) = old behaviour: such keyframes are dropped from the mesh input (133820: 7 of 203). "
                         "Their board-depth check is skipped (no GT plane).")
    # fixes 11-12 H1: depth noise model of a motion pair without a two-lens anchor (motion_stereo.pair_depth) + TSDF thresholds.
    # Defaults = old values (rotation 0.5 deg, baseline 10 %, sigma_max 0.08 m, sigma_ok 0.02 m): with them nothing beyond ~0.63 m
    # (baseline 7.2 cm) reaches the mesh. README of runs/fixes_20261112 section 2 for the 'ba' values (0.1 deg, 2 %).
    ap.add_argument("--sigma-rot-deg", type=float, default=None, help="relative rotation error of a pair (deg) in the depth sigma "
                    "(default MSConfig.sigma_rot_deg_free 0.5)")
    ap.add_argument("--sigma-b-rel", type=float, default=None, help="relative baseline error of a pair in the depth sigma "
                    "(default MSConfig.sigma_b_rel_free 0.1)")
    ap.add_argument("--sigma-max", type=float, default=0.08, help="TSDF: drop pixels with depth sigma above this (m)")
    ap.add_argument("--sigma-ok", type=float, default=0.02, help="TSDF: weight of one observation at this sigma = mesh threshold (m)")
    a = ap.parse_args()
    if a.ffs_views is not None:
        if not 2 <= a.ffs_views <= 8:
            sys.exit("--ffs-views must be 2..8")
        a.n_pairs = a.ffs_views - 1
    a.ffs_views = a.n_pairs + 1
    fuse = a.ffs_fuse if a.ffs_fuse != "auto" else ("mean" if a.n_pairs <= 2 else "median")
    if a.scale > 0.5 and not a.allow_fullres:
        sys.exit("scale > 0.5 refused: 1920x1080 FFS overflowed 16 GB VRAM and crashed WSL (11-05); pass --allow-fullres to try under the cap")
    t_start = time.time()
    rec, out = Path(a.rec), Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    g = np.load(rec / a.gt / "gt.npz")
    board = json.loads((rec / a.gt / "board.json").read_text())
    sq = float(g["square_mm"]) / 1000.0
    bj = json.loads(Path(str(g["board_json"])).read_text())
    BW, BH = bj["squares_x"] * sq, bj["squares_y"] * sq
    s = a.scale
    K = g["K"].copy()
    K[:2] *= s
    dist = g["dist"].astype(np.float64)
    ok = g["ok"]
    n = len(ok)
    rng = np.random.default_rng(0)
    T = np.zeros((n, 4, 4))
    T[:, :3, :3] = g["R_bc"]
    T[:, :3, 3] = g["p_bc"] + (rng.normal(0, a.pose_noise_cm / 100.0, (n, 3)) if a.pose_noise_cm > 0 else 0)
    T[:, 3, 3] = 1
    t_ns = g["t_ns"]
    pose_info = dict(source=a.gt)
    gt_ok = ok.copy()                    # GT plane usable (board check); M1: also the mesh-input filter unless --keep-gt-not-ok
    n_gt_drop = None
    Tgt = T.copy()
    if a.vio:
        import gzip
        vt, vT = [], []
        for line in gzip.open(a.vio, "rt"):
            d = json.loads(line)
            if d.get("type") == "pose" and d.get("T_world_cam0") and d.get("status") == "tracking":
                vt.append(d["t_ns"]); vT.append(np.array(d["T_world_cam0"], float).reshape(4, 4))
        vt = np.array(vt); vT = np.array(vT)
        # frame index by nearest time (VIO t_ns vs GT t_ns are both frame times; allow 4 ms)
        idx = np.searchsorted(t_ns, vt).clip(1, len(t_ns) - 1)
        idx = np.where(np.abs(t_ns[idx - 1] - vt) < np.abs(t_ns[idx] - vt), idx - 1, idx)
        good = np.abs(t_ns[idx] - vt) < 8e6   # VIO t_ns is shifted by the GT align dt (~4.7 ms) vs t_mid; frame period 16.7 ms
        idx, vT = idx[good], vT[good]
        both = ok[idx]
        A = vT[both][:, :3, 3]; Bp = T[idx[both]][:, :3, 3]
        ma, mb = A.mean(0), Bp.mean(0)
        U, S_, Vt = np.linalg.svd((Bp - mb).T @ (A - ma))
        D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
        Ra = U @ D @ Vt
        sc = float(np.trace(np.diag(S_) @ D) / ((A - ma) ** 2).sum())
        ta = mb - Ra @ ma
        if a.free_scale:                       # scale the pose translations by the oracle Sim(3) scale, then rigid again
            vT = vT.copy(); vT[:, :3, 3] *= sc
            A = A * sc; ma = A.mean(0)
            U, S_, Vt = np.linalg.svd((Bp - mb).T @ (A - ma))
            D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
            Ra = U @ D @ Vt
            ta = mb - Ra @ ma
        Tal = np.eye(4); Tal[:3, :3] = Ra; Tal[:3, 3] = ta
        newT = np.zeros_like(T); okv = np.zeros(len(ok), bool)
        for k, Tv in zip(idx, vT):
            newT[k] = Tal @ Tv; okv[k] = True
        res = np.linalg.norm((newT[idx[both]][:, :3, 3]) - Bp, axis=1)
        # rotation check: relative GT vs VIO camera rotation (axes convention)
        dR = [np.degrees(np.arccos(np.clip((np.trace(newT[k][:3, :3].T @ T[k][:3, :3]) - 1) / 2, -1, 1))) for k in idx[both][::20]]
        pose_info = dict(source=a.vio, n=int(len(idx)), se3_rmse_cm=float(np.sqrt((res ** 2).mean()) * 100),
                         sim3_scale_would_be=sc, free_scale_applied=bool(a.free_scale), rot_diff_deg_median=float(np.median(dR)))
        print("VIO poses", pose_info, flush=True)
        # GT 'ok' is still needed for the board-depth check plane (pose of the plane uses VIO world). M1 (fixes 11-12): by default it
        # ALSO drops keyframes from the mesh input (old behaviour); --keep-gt-not-ok keeps them (board check skipped for them)
        n_gt_drop = int((okv & ~ok).sum())
        T, ok = newT, (okv if a.keep_gt_not_ok else okv & ok)

    if a.rows_from:
        import gzip
        rt = np.array([json.loads(l)["t_ns"] for l in gzip.open(a.rows_from, "rt") if "t_ns" in l], np.int64)
        j = np.searchsorted(t_ns, rt).clip(1, len(t_ns) - 1)
        j = np.where(np.abs(t_ns[j - 1] - rt) < np.abs(t_ns[j] - rt), j - 1, j)
        keep = np.zeros(len(ok), bool); keep[j[np.abs(t_ns[j] - rt) < 1e6]] = True
        ok = ok & keep
        pose_info["rows_from"] = dict(file=a.rows_from, n=int(keep.sum()))
    sel = np.flatnonzero(ok) if a.compress else None
    if sel is not None:                                # compressed row index k <-> recording frame sel[k]
        t_ns, T, Tgt, ok, gt_ok = t_ns[sel], T[sel], Tgt[sel], ok[sel], gt_ok[sel]
        n = len(sel)
        pose_info["compressed_rows"] = int(n)

    # decode (scaled) frames
    W0, H0 = int(round(1920 * s)), int(round(1080 * s))
    imgs = []
    if (rec / "frames_kf").is_dir() and sel is None:
        sys.exit("REC/frames_kf (reduced keyframe-only recording) needs --compress (reads only rows with a pose)")
    cap = None if sel is not None else cv2.VideoCapture(str(rec / "cam5.mp4"))
    if sel is not None:
        import kf_frames                               # kfonly 11-11: REC/frames_kf/ (reduced recording) or cam5.mp4, same frames
        pos = {int(r_): k for k, r_ in enumerate(sel)}
        imgs = [None] * n
        nimg = 0
        for fi, f in kf_frames.iter_frames(rec, sel):
            imgs[pos[fi]] = f if (W0, H0) == (f.shape[1], f.shape[0]) else cv2.resize(f, (W0, H0), interpolation=cv2.INTER_AREA)
            nimg = pos[fi] + 1
    while sel is None:
        r, f = cap.read()
        if not r:
            break
        k_ = len(imgs)                                   # keep only frames that have a pose (RAM at full res)
        imgs.append((f if (W0, H0) == (f.shape[1], f.shape[0]) else cv2.resize(f, (W0, H0), interpolation=cv2.INTER_AREA))
                    if (k_ < len(ok) and ok[k_]) else None)
    if cap is not None:
        cap.release()
    nimg = len(imgs) if sel is None else nimg
    print(f"decoded {nimg} frames ({W0}x{H0}), gt rows {n}, ok {int(ok.sum())}", flush=True)
    n = min(n, nimg)

    # keyframe grid = undistorted pinhole with the same K (R1 = I)
    rp = dict(size=(W0, H0), f=float(K[0, 0]), cx=float(K[0, 2]), cy=float(K[1, 2]), R1=np.eye(3).tolist())
    Kr = ms.K_of(rp["f"], rp["cx"], rp["cy"])
    und = cv2.initUndistortRectifyMap(K, dist, np.eye(3), Kr, (W0, H0), cv2.CV_32FC1)
    cfg = ms.MSConfig()
    cfg.b_target_m = a.b_target
    cfg.z_max = a.zmax
    if a.sigma_rot_deg is not None:
        cfg.sigma_rot_deg_free = a.sigma_rot_deg
    if a.sigma_b_rel is not None:
        cfg.sigma_b_rel_free = a.sigma_b_rel
    if a.ffs_partners == "all":
        # time cap: 1e9 = none (fv_a*/d*: pairs > 30 s apart on 133820 = keyframe 3/5 'hub' partners at the start, FFS valid 0.14
        # or confidently wrong (|dZ| p90 44 cm), not caught by the dy check; cause not found) -> default 30 s (fv_c*)
        cfg.dt_max_s = a.ffs_dt_max if a.ffs_dt_max is not None else 30.0
    min_valid = a.pair_min_valid if a.pair_min_valid is not None else 0.0
    pair_check = a.pair_check if a.pair_check != "auto" else ("dy" if a.ffs_partners == "all" else "none")
    ffs_ms_all = [0.0]
    _feat, _dy, dy_stat = {}, {}, dict(checked=0, rejected=0, s=0.0)

    def dy_ok(i, j):
        """pair check before FFS (motion_stereo gate: SIFT matches moved into this pair's motion rectification with the BA
        relative pose; |dy| median <= cfg.gate_dy_px 1.2 px and dx > 0 for >= 90 %). Catches keyframes whose BA rotation is off
        (133820 row 2: 0.33 deg -> ~4 px dy -> FFS valid 0.10, 85 keyframes chose it with --ffs-partners all and no check)."""
        if (i, j) in _dy:
            return _dy[(i, j)]
        t0 = time.perf_counter()
        for k_ in (i, j):
            if k_ not in _feat:
                _feat[k_] = ms.sift_feats(cv2.cvtColor(imgs[k_], cv2.COLOR_BGR2GRAY), cfg.sift_n)
        mt = ms.match_norm(_feat[i], _feat[j], K, dist)
        okk = False
        if mt is not None:
            R_, t_ = ms.rel_pose(T[i], T[j])
            okk = ms.gate_ok(ms.rect_match_stats(mt[0], mt[1], R_, t_, rp["f"]), cfg)
        _dy[(i, j)] = okk
        dy_stat["checked"] += 1; dy_stat["rejected"] += int(not okk); dy_stat["s"] += time.perf_counter() - t0
        return okk
    import lib_ffs
    model = lib_ffs.load(valid_iters=a.iters)
    tsdf_msgs = []
    def tsdf_log(m):                     # M3 (fixes 11-12): block-limit message printed + kept for the summary (no behaviour change)
        print(m, flush=True); tsdf_msgs.append(str(m))
    tsdf_kw = dict(voxel_m=a.voxel, z_max=a.zmax, sigma_max=a.sigma_max, sigma_ok=a.sigma_ok, min_obs=2,
                   alloc_dense=(a.tsdf_alloc == "dense"), log=tsdf_log, max_blocks=a.tsdf_max_blocks)
    if a.tsdf_evict_gpu_blocks > 0:
        import tsdf_evict
        tsdf = tsdf_evict.EvictTSDFMap(gpu_blocks=a.tsdf_evict_gpu_blocks, cpu_dtype=a.tsdf_evict_dtype,
                                       disk_dir=a.tsdf_evict_dir or str(out / "tsdf_spill"), **tsdf_kw)
        print(f"[tsdf] evict on: GPU {a.tsdf_evict_gpu_blocks} blocks, spill {tsdf.cpu.disk} ({a.tsdf_evict_dtype})", flush=True)
    else:
        tsdf = tsdf_map.TSDFMap(**tsdf_kw)
    print(f"[tsdf] alloc {a.tsdf_alloc}", flush=True)

    # board quad in the undistorted keyframe grid -> GT plane depth per pixel
    uu, vv = np.meshgrid(np.arange(W0, dtype=np.float64), np.arange(H0, dtype=np.float64))
    rays = np.stack([(uu - Kr[0, 2]) / Kr[0, 0], (vv - Kr[1, 2]) / Kr[1, 1], np.ones_like(uu)], -1)
    rows, dz_all = [], []
    integ_i = []                         # fixes 11-12: keyframes that went into the TSDF (for kf_poses.npz + vertex camera depth)
    for i in range(0, n, a.kf_step):
        if not ok[i] or imgs[i] is None:
            continue
        cands = []
        for j in (range(max(0, i - 150), min(n, i + 150)) if a.ffs_partners == "near" else range(n)):
            if j == i or not ok[j] or imgs[j] is None:
                continue
            R, t = ms.rel_pose(T[i], T[j])
            gg = ms.pair_geometry(R, t)
            sc, why = ms.score_pair(gg, abs(t_ns[i] - t_ns[j]) * 1e-9, cfg)
            if sc is not None:
                cands.append((sc, j, gg))
        cands.sort(key=lambda x: x[0])
        rejected, rcache = [], {}
        for _try in range(1 + (cfg.max_tries if min_valid > 0 else 0)):
            cl = [c for c in cands if c[1] not in rejected]
            chosen = []
            for sc, j, gg in cl:
                if not chosen or all(ms.dir_differs(gg, c[2], cfg.pair2_min_dir_deg) for c in chosen):
                    if pair_check == "dy" and not dy_ok(i, j):
                        continue
                    chosen.append((sc, j, gg))
                if len(chosen) >= min(a.n_pairs, 2):           # partners 1..2: the old rule unchanged (same pairs for every N >= 3)
                    break
            if a.n_pairs > 2 and chosen:
                chosen = pick_extra(chosen, cl, a.n_pairs, cfg.pair2_min_dir_deg,
                                    ok_fn=(lambda j: dy_ok(i, j)) if pair_check == "dy" else None)
            bad = False
            for sc, j, gg in chosen:
                if j not in rcache:
                    R, t = ms.rel_pose(T[i], T[j])
                    rcache[j] = ms.pair_depth(model, lib_ffs.infer, imgs[i], imgs[j], R, t, K, dist, rp, cfg, iters=a.iters)
                    ffs_ms_all[0] += rcache[j]["ms"]["ffs"]
                if min_valid > 0 and float(rcache[j]["valid"].mean()) < min_valid:   # pair gate (ffs-partners all): pose/overlap bad
                    rejected.append(j); bad = True
            if not bad:
                break
        chosen = [c for c in chosen if c[1] not in rejected]
        if not chosen:
            rows.append(dict(i=i, pairs=0, **({"rejected": [int(x) for x in rejected]} if rejected else {})))
            continue
        res = [rcache[c[1]] for c in chosen]
        Z, V, S, agree = ms.combine_motion(res, cfg) if fuse == "mean" else fuse_median(res, cfg.k_sigma)
        bgr = cv2.remap(imgs[i], *und, cv2.INTER_LINEAR)
        # GT plane depth: board plane z=0 in world; ray in cam: X = Z*ray, world = R X + p, z_w = 0
        Rw, pw = Tgt[i][:3, :3], Tgt[i][:3, 3]   # GT plane depth always from the GT pose
        rz = rays @ Rw[2]                      # z_world component per unit-depth ray
        with np.errstate(divide="ignore", invalid="ignore"):
            Zp = -pw[2] / rz
        Xw = pw[None, None] + Zp[..., None] * (rays @ Rw.T)
        inb = (Xw[..., 0] > 0.01) & (Xw[..., 0] < BW - 0.01) & (Xw[..., 1] > 0.01) & (Xw[..., 1] < BH - 0.01) & (Zp > 0)
        m = inb & V & np.isfinite(Z)
        row = dict(i=i, pairs=len(res), B_cm=[round(r["B"] * 100, 1) for r in res], valid_frac=float(V.mean()),
                   board_px=int(inb.sum()), board_valid=int(m.sum()), ffs_ms=[round(r["ms"]["ffs"]) for r in res])
        if m.sum() > 200 and gt_ok[i]:      # gt_ok: always true here unless --keep-gt-not-ok
            dz = (Z[m] - Zp[m])
            dz_all.append(dz.astype(np.float32))
            row.update(board_dZ_mean_cm=float(dz.mean() * 100), board_dZ_p10_cm=float(np.percentile(dz, 10) * 100),
                       board_dZ_p90_cm=float(np.percentile(dz, 90) * 100), board_absdZ_mean_cm=float(np.abs(dz).mean() * 100))
            row.update(board_dZ_med_cm=float(np.median(dz) * 100), board_absdZ_p50_cm=float(np.median(np.abs(dz)) * 100),
                       board_absdZ_p90_cm=float(np.percentile(np.abs(dz), 90) * 100), board_Z_m=float(np.median(Zp[m])))
        if sel is not None:
            row["frame"] = int(sel[i])
        if a.n_pairs != 2 or a.ffs_partners != "near":     # keyframes.json of the default run stays byte-identical to the old one
            row.update(n_cand=len(cands), partners=[int(c[1]) for c in chosen], rejected=[int(x) for x in rejected])
        rows.append(row)
        tsdf.integrate(np.where(V, Z, np.nan).astype(np.float32), V, bgr, rp["f"], rp["cx"], rp["cy"], 0.08, T[i],
                       sigma=np.where(V, S, np.inf).astype(np.float32))
        integ_i.append(i)
        if len(rows) % 10 == 0:
            print(f"kf {i}/{n}: {row}", flush=True)
    tsdf.update_mesh(force=True)
    Vm, Cm, Fm = tsdf.mesh_arrays()
    tsdf_map.write_mesh_ply(out / "mesh.ply", Vm, Cm, Fm)
    (out / "mesh.obj").write_bytes(tsdf_map.mesh_obj_bytes(Vm, Cm, Fm))
    summ = dict(poses=pose_info, made=time.strftime("%Y-%m-%d %H:%M:%S"), rec=str(rec), gt=a.gt, args=vars(a), board_mm=[BW * 1000, BH * 1000],
                square_mm=sq * 1000, gt_corner_px_median=board.get("corner_px_median"), n_kf=len(rows),
                n_kf_depth=sum(1 for r in rows if r.get("pairs")), mesh_vertices=int(len(Vm)), mesh_faces=int(len(Fm)),
                runtime_s=round(time.time() - t_start, 1), image_size=[W0, H0], K_used=K.tolist(),
                gpu_max_alloc_gb=round(__import__('torch').cuda.max_memory_allocated() / 2 ** 30, 2),
                gpu_max_reserved_gb=round(__import__('torch').cuda.max_memory_reserved() / 2 ** 30, 2))
    summ.update(ffs_views=a.ffs_views, ffs_fuse=fuse, ffs_partners=a.ffs_partners, ffs_dt_max=cfg.dt_max_s, pair_min_valid=min_valid, pair_check=pair_check,
                pair_check_n=dy_stat["checked"], pair_check_rejected=dy_stat["rejected"], pair_check_s=round(dy_stat["s"], 1),
                ffs_s_total=round(ffs_ms_all[0] / 1000, 1), ffs_s_used=round(sum(sum(r.get("ffs_ms", [])) for r in rows) / 1000, 1),
                n_pairs_rejected=sum(len(r.get("rejected", [])) for r in rows),
                n_kf_by_pairs={str(k): sum(1 for r in rows if r.get("pairs", 0) == k) for k in range(a.n_pairs + 1)})
    ev = [r for r in rows if "board_absdZ_p50_cm" in r]
    if ev:
        summ["kf_board_absdZ_p50_cm_median"] = float(np.median([r["board_absdZ_p50_cm"] for r in ev]))
        summ["kf_board_absdZ_p90_cm_median"] = float(np.median([r["board_absdZ_p90_cm"] for r in ev]))
        summ["kf_board_dZ_bias_cm_median"] = float(np.median([r["board_dZ_med_cm"] for r in ev]))
        summ["kf_board_Z_m_median"] = float(np.median([r["board_Z_m"] for r in ev]))
        summ["n_kf_board_eval"] = len(ev)
        summ["n_kf_no_pair"] = sum(1 for r in rows if not r.get("pairs"))
        for k in ("board_dZ_mean_cm", "board_dZ_med_cm", "board_absdZ_p50_cm", "board_absdZ_mean_cm"):
            v = np.array([r[k] for r in ev])
            summ[f"kf_{k}_stats"] = dict(mean=float(v.mean()), median=float(np.median(v)), p10=float(np.percentile(v, 10)),
                                         p90=float(np.percentile(v, 90)))
        D = np.concatenate(dz_all) * 100
        summ["pooled_dZ_cm"] = dict(n=int(len(D)), mean=float(D.mean()), median=float(np.median(D)), p10=float(np.percentile(D, 10)),
                                    p90=float(np.percentile(D, 90)))
        summ["pooled_absdZ_cm"] = dict(mean=float(np.abs(D).mean()), median=float(np.median(np.abs(D))),
                                       p90=float(np.percentile(np.abs(D), 90)))
        summ["kf_frac_abs_bias_gt_1cm"] = float(np.mean([abs(r["board_dZ_med_cm"]) > 1 for r in ev]))
    if len(Vm):
        Vm = np.asarray(Vm)
        mb = (Vm[:, 0] > 0.02) & (Vm[:, 0] < BW - 0.02) & (Vm[:, 1] > 0.02) & (Vm[:, 1] < BH - 0.02) & (np.abs(Vm[:, 2]) < 0.10)
        if mb.sum() > 20:
            z = Vm[mb, 2]
            summ.update(mesh_board_vertices=int(mb.sum()), mesh_board_z_med_cm=float(np.median(z) * 100),
                        mesh_board_z_rms_cm=float(np.sqrt(np.mean((z - np.median(z)) ** 2)) * 100),
                        mesh_board_z_p5_p95_cm=[float(np.percentile(z, 5) * 100), float(np.percentile(z, 95) * 100)])
            # board coverage: fraction of 1 cm cells of the board rectangle that hold a mesh vertex within 3 cm of the plane
            near = mb & (np.abs(Vm[:, 2]) < 0.03)
            cx_ = np.floor((Vm[near, 0] - 0.02) * 100).astype(int); cy_ = np.floor((Vm[near, 1] - 0.02) * 100).astype(int)
            nx_, ny_ = int(np.ceil((BW - 0.04) * 100)), int(np.ceil((BH - 0.04) * 100))
            cells = set(zip(cx_.clip(0, nx_ - 1), cy_.clip(0, ny_ - 1)))
            summ["mesh_board_cover_frac"] = len(cells) / float(nx_ * ny_)
            summ["mesh_board_z_unique_mm"] = int(len(np.unique(np.round(z * 1000, 1))))   # sub-voxel check (many = interpolated)
    summ["tsdf_alloc"] = a.tsdf_alloc
    summ["depth_sigma"] = dict(rot_deg=cfg.sigma_rot_deg_free, b_rel=cfg.sigma_b_rel_free, sigma_d_px=cfg.sigma_d_px,
                               sigma_max=a.sigma_max, sigma_ok=a.sigma_ok, z_max=a.zmax)
    # fixes 11-12 (diagnostic only): camera depth of each mesh vertex = nearest integrated keyframe camera that has it in view
    # (no occlusion test); poses of the integrated keyframes saved for later analysis
    np.savez_compressed(out / "kf_poses.npz", i=np.array(integ_i, int), T=T[integ_i] if integ_i else np.zeros((0, 4, 4)),
                        t_ns=t_ns[integ_i] if integ_i else np.zeros(0, np.int64), f=rp["f"], cx=rp["cx"], cy=rp["cy"], size=np.array([W0, H0]))
    if len(Vm) and integ_i:
        Vv = np.asarray(Vm, np.float64)
        best = np.full(len(Vv), np.inf)
        for i_ in integ_i:
            Rm_, tm_ = T[i_][:3, :3], T[i_][:3, 3]
            pc = (Vv - tm_) @ Rm_
            zc = pc[:, 2]
            with np.errstate(divide="ignore", invalid="ignore"):
                uu_ = pc[:, 0] / zc * rp["f"] + rp["cx"]; vv_ = pc[:, 1] / zc * rp["f"] + rp["cy"]
            inn = (zc > 0.05) & (uu_ >= 0) & (uu_ < W0) & (vv_ >= 0) & (vv_ < H0)
            best = np.where(inn, np.minimum(best, zc), best)
        fin = best[np.isfinite(best)]
        if len(fin):
            summ["mesh_cam_depth_m"] = dict(n=int(len(fin)), p50=float(np.median(fin)), p90=float(np.percentile(fin, 90)),
                                            p99=float(np.percentile(fin, 99)), max=float(fin.max()),
                                            frac_gt_0p65=float(np.mean(fin > 0.65)), frac_gt_1p0=float(np.mean(fin > 1.0)))
    # fixes 11-12: quality / provenance records (results unchanged)
    summ.update(tsdf_full=bool(tsdf.full), tsdf_blocks=int(tsdf.nb), tsdf_max_blocks=tsdf.cfg.get("max_blocks"), tsdf_log=tsdf_msgs,
                gt_ok_filter=(not a.keep_gt_not_ok), n_kf_gt_not_ok_dropped=(0 if a.keep_gt_not_ok else n_gt_drop),
                n_kf_gt_not_ok=n_gt_drop)
    if a.tsdf_evict_gpu_blocks > 0:      # unit 12: blocks = GPU + spill; spill folder removed unless --tsdf-evict-keep
        summ.update(tsdf_blocks=tsdf.n_total, tsdf_evict=tsdf.evict_stats())
        tsdf.cpu.close(delete=not a.tsdf_evict_keep)
    try:
        import hashlib
        summ["lib_ffs"] = dict(file=str(Path(lib_ffs.__file__).resolve()),
                               sha=hashlib.sha256(Path(lib_ffs.__file__).read_bytes()).hexdigest()[:16])
    except Exception as e:
        summ["lib_ffs"] = dict(error=repr(e))
    (out / "summary.json").write_text(json.dumps(summ, indent=1))
    (out / "keyframes.json").write_text(json.dumps(rows, indent=0))
    print(json.dumps(summ, indent=1))


def _signed_dir_deg(g0, g1):
    """angle between the two baseline vectors WITH sign (0..180): the opposite side of the reference counts as a new direction."""
    u0, u1 = g0["c"] / g0["B"], g1["c"] / g1["B"]
    return float(np.degrees(np.arccos(np.clip(u0 @ u1, -1, 1))))


def pick_extra(chosen, cands, n_pairs, min_dir_deg, b_ratio=1.5, ok_fn=None):
    """ffs-views 11-11: partners beyond the old two. cands = all partners that pass score_pair (B 4-30 cm, dt 0.15-2.5 s,
    baseline >= 40 deg off the optical axis, rotation <= 25 deg), best score first. Pass 1: signed baseline direction >= min_dir_deg
    from every chosen partner (old rule uses |cos| -> the other side of the reference on the same motion line is new here: its
    occlusions are on the other side). Pass 2: same direction but baseline length x1.5 / /1.5 vs every chosen partner of that
    direction (different disparity range). Near-duplicates (neighbouring keyframes, same B and direction) are never added."""
    chosen = list(chosen)
    used = {c[1] for c in chosen}
    for rule in ("dir", "dir_or_b"):
        for sc, j, gg in cands:
            if len(chosen) >= n_pairs:
                return chosen
            if j in used:
                continue
            okk = True
            for c in chosen:
                d = _signed_dir_deg(gg, c[2]) >= min_dir_deg
                if rule == "dir_or_b":
                    d = d or max(gg["B"] / c[2]["B"], c[2]["B"] / gg["B"]) >= b_ratio
                okk &= d
            if okk and (ok_fn is None or ok_fn(j)):
                chosen.append((sc, j, gg)); used.add(j)
    return chosen


def fuse_median(results, k_sigma):
    """ffs-views 11-11: per pixel median of the valid pair depths; pairs within k_sigma * own sigma of the median are inliers;
    pixel kept if 1 pair is valid (as combine_motion) or >= 2 pairs agree; Z / sigma = inverse-variance mean of the inliers.
    Returns (Z, valid, sigma, agree) like motion_stereo.combine_motion."""
    Zs = np.stack([np.where(r["valid"], r["Z"], np.nan) for r in results]).astype(np.float64)
    Ss = np.stack([np.maximum(r["sigma"], 1e-6) for r in results]).astype(np.float64)
    Vs = np.stack([r["valid"] for r in results])
    nv = Vs.sum(0)
    import warnings
    with warnings.catch_warnings(), np.errstate(invalid="ignore", divide="ignore"):
        warnings.simplefilter("ignore", RuntimeWarning)
        med = np.nanmedian(Zs, 0)
        inl = Vs & (np.abs(Zs - med[None]) <= k_sigma * Ss)
        ni = inl.sum(0)
        w = np.where(inl, 1.0 / Ss ** 2, 0.0)
        den = w.sum(0)
        Z = np.where(den > 0, (w * np.nan_to_num(Zs)).sum(0) / den, np.nan)
        S = np.where(den > 0, 1.0 / np.sqrt(den), np.inf)
    agree = (nv >= 2) & (ni >= 2)
    V = (nv == 1) | agree
    return Z.astype(np.float32), V, S.astype(np.float32), agree


def _out_arg():
    import sys as _s
    return _s.argv[_s.argv.index("--out") + 1] if "--out" in _s.argv else None


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import gpu_guard  # one GPU job at a time, torch memory cap, clean OOM exit (README 10)
    with gpu_guard.job("scan_offline " + str(_out_arg()), out_dir=_out_arg(), torch_cap=True):
        main()
