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
    ap.add_argument("--b-target", type=float, default=0.08)
    ap.add_argument("--vio", default="", help="VIO poses.jsonl.gz (T_world_cam0 per frame); rigid SE(3) align to GT (scale NOT fixed)")
    ap.add_argument("--free-scale", action="store_true",
                    help="ORACLE global scale: Sim(3) of the VIO positions to GT, translations x s (depth scales with the baseline)")
    ap.add_argument("--pose-noise-cm", type=float, default=0.0, help="add N(0, s) cm position noise per frame (VIO-like test)")
    ap.add_argument("--allow-fullres", action="store_true",
                    help="allow --scale > 0.5 (1920x1080 FFS needs > 16 GB; runs only under the gpu_guard cap, fails cleanly)")
    a = ap.parse_args()
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
        T, ok = newT, okv & ok      # GT 'ok' is still needed for the board-depth check plane (pose of the plane uses VIO world)

    # decode (scaled) frames
    cap = cv2.VideoCapture(str(rec / "cam5.mp4"))
    W0, H0 = int(round(1920 * s)), int(round(1080 * s))
    imgs = []
    while True:
        r, f = cap.read()
        if not r:
            break
        k_ = len(imgs)                                   # keep only frames that have a pose (RAM at full res)
        imgs.append((f if (W0, H0) == (f.shape[1], f.shape[0]) else cv2.resize(f, (W0, H0), interpolation=cv2.INTER_AREA))
                    if (k_ < len(ok) and ok[k_]) else None)
    cap.release()
    nimg = len(imgs)
    print(f"decoded {nimg} frames ({W0}x{H0}), gt rows {n}, ok {int(ok.sum())}", flush=True)
    n = min(n, nimg)

    # keyframe grid = undistorted pinhole with the same K (R1 = I)
    rp = dict(size=(W0, H0), f=float(K[0, 0]), cx=float(K[0, 2]), cy=float(K[1, 2]), R1=np.eye(3).tolist())
    Kr = ms.K_of(rp["f"], rp["cx"], rp["cy"])
    und = cv2.initUndistortRectifyMap(K, dist, np.eye(3), Kr, (W0, H0), cv2.CV_32FC1)
    cfg = ms.MSConfig()
    cfg.b_target_m = a.b_target
    cfg.z_max = a.zmax
    import lib_ffs
    model = lib_ffs.load(valid_iters=a.iters)
    tsdf = tsdf_map.TSDFMap(voxel_m=a.voxel, z_max=a.zmax, sigma_max=0.08, sigma_ok=0.02, min_obs=2)

    # board quad in the undistorted keyframe grid -> GT plane depth per pixel
    uu, vv = np.meshgrid(np.arange(W0, dtype=np.float64), np.arange(H0, dtype=np.float64))
    rays = np.stack([(uu - Kr[0, 2]) / Kr[0, 0], (vv - Kr[1, 2]) / Kr[1, 1], np.ones_like(uu)], -1)
    rows, dz_all = [], []
    for i in range(0, n, a.kf_step):
        if not ok[i]:
            continue
        cands = []
        for j in range(max(0, i - 150), min(n, i + 150)):
            if j == i or not ok[j]:
                continue
            R, t = ms.rel_pose(T[i], T[j])
            gg = ms.pair_geometry(R, t)
            sc, why = ms.score_pair(gg, abs(t_ns[i] - t_ns[j]) * 1e-9, cfg)
            if sc is not None:
                cands.append((sc, j, gg))
        cands.sort(key=lambda x: x[0])
        chosen = []
        for sc, j, gg in cands:
            if not chosen or all(ms.dir_differs(gg, c[2], cfg.pair2_min_dir_deg) for c in chosen):
                chosen.append((sc, j, gg))
            if len(chosen) >= a.n_pairs:
                break
        if not chosen:
            rows.append(dict(i=i, pairs=0))
            continue
        res = []
        for sc, j, gg in chosen:
            R, t = ms.rel_pose(T[i], T[j])
            r = ms.pair_depth(model, lib_ffs.infer, imgs[i], imgs[j], R, t, K, dist, rp, cfg, iters=a.iters)
            res.append(r)
        Z, V, S, agree = ms.combine_motion(res, cfg)
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
        if m.sum() > 200:
            dz = (Z[m] - Zp[m])
            dz_all.append(dz.astype(np.float32))
            row.update(board_dZ_mean_cm=float(dz.mean() * 100), board_dZ_p10_cm=float(np.percentile(dz, 10) * 100),
                       board_dZ_p90_cm=float(np.percentile(dz, 90) * 100), board_absdZ_mean_cm=float(np.abs(dz).mean() * 100))
            row.update(board_dZ_med_cm=float(np.median(dz) * 100), board_absdZ_p50_cm=float(np.median(np.abs(dz)) * 100),
                       board_absdZ_p90_cm=float(np.percentile(np.abs(dz), 90) * 100), board_Z_m=float(np.median(Zp[m])))
        rows.append(row)
        tsdf.integrate(np.where(V, Z, np.nan).astype(np.float32), V, bgr, rp["f"], rp["cx"], rp["cy"], 0.08, T[i],
                       sigma=np.where(V, S, np.inf).astype(np.float32))
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
    (out / "summary.json").write_text(json.dumps(summ, indent=1))
    (out / "keyframes.json").write_text(json.dumps(rows, indent=0))
    print(json.dumps(summ, indent=1))


def _out_arg():
    import sys as _s
    return _s.argv[_s.argv.index("--out") + 1] if "--out" in _s.argv else None


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import gpu_guard  # one GPU job at a time, torch memory cap, clean OOM exit (README 10)
    with gpu_guard.job("scan_offline " + str(_out_arg()), out_dir=_out_arg(), torch_cap=True):
        main()
