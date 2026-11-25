#!/usr/bin/env python3
"""gsmap 2026-11-25 E6 re-fuse (plan section 3 E6 / section 6): FFS (already filtered) + AI fill with a multi-view gate, zmax 5 m.

  GSMAP_RUN=<run> python scan3d/gsmap/gsm_e6fuse.py OUTDIR --src da3|moge_pooled [--da3 DIR] [--zmax 5] [--gate 2]
FFS pixels = F == 1 of the run's fuse/moge_f (multi-view + edge + 2 m filtered FFS, unchanged).
AI depth: da3 = DA3 pose-conditioned raw (no fit); moge_pooled = MoGe-2 x median(Z_ffs / Z_moge) over keyframes +-12 (scale only).
AI pixels pass the median5 3 % edge cut, 0.05 < Z < zmax, and the multi-view gate: projected into the other keyframes with
>= 10 cm baseline, >= --gate of them must have AI depth within 3 % at that pixel; else no depth there (no loss, no init point).
Writes OUTDIR/<kf>.npz (Z, F: 1 FFS, 2 AI) + OUTDIR/fuse_info.json.
"""
import json, os, sys, time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "gsplat"))
R = Path(os.environ["GSMAP_RUN"])


def main():
    import torch
    out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
    arg = lambda k, d=None: sys.argv[sys.argv.index(k) + 1] if k in sys.argv else d  # noqa: E731
    src = arg("--src"); da3 = arg("--da3"); zmax = float(arg("--zmax", 5.0)); gate = int(arg("--gate", 2))
    t0 = time.time()
    c = np.load(R / "dump_ba" / "cams.npz"); kf, T, K = c["kf"], c["T"], c["Kr"]; W, H = [int(v) for v in c["size"]]; n = len(kf)
    mx, my = cv2.initUndistortRectifyMap(c["K"], c["dist"], None, K, (W, H), cv2.CV_32FC1)
    vm = cv2.erode(((mx >= 0) & (mx <= W - 1) & (my >= 0) & (my <= H - 1)).astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)
    FF, AI = [], []
    for i in kf:
        d = np.load(R / "fuse" / "moge_f" / f"{i:05d}.npz"); FF.append(np.where(d["F"] == 1, d["Z"], np.nan).astype(np.float32))
    if src == "da3":
        AI = [np.load(Path(da3) / f"{i:05d}.npz")["Z"].astype(np.float32) for i in kf]; scale_info = "none (DA3 pose units)"
    else:
        Mo = [np.load(R / "depth_moge" / f"{i:05d}.npz")["Z"].astype(np.float32) for i in kf]; rat = {}
        for ci in range(n):
            m = np.isfinite(FF[ci]) & np.isfinite(Mo[ci]) & (Mo[ci] > 0)
            if m.sum() > 500:
                rat[ci] = float(np.median(FF[ci][m] / Mo[ci][m]))
        gs = float(np.median(list(rat.values())))
        for ci in range(n):
            nb = [rat[j] for j in range(ci - 12, ci + 13) if j in rat]; AI.append((float(np.median(nb)) if nb else gs) * Mo[ci])
        scale_info = f"pooled +-12 median ratio (global {gs:.3f})"
    for ci in range(n):   # edge cut + range + border
        A = AI[ci]; med = cv2.medianBlur(np.nan_to_num(A, nan=0).astype(np.float32), 5)
        ok = vm & np.isfinite(A) & (A > 0.05) & (A < zmax) & (np.abs(A - med) / np.maximum(A, 1e-3) < 0.03)
        AI[ci] = np.where(ok, A, np.nan).astype(np.float32)
    dev = "cuda"; At = torch.from_numpy(np.stack(AI)).to(dev)
    Tw = torch.from_numpy(np.linalg.inv(T)).float().to(dev); Tc = torch.from_numpy(T).float().to(dev); cen = T[:, :3, 3]
    fx, fy, cx, cy = [float(v) for v in (K[0, 0], K[1, 1], K[0, 2], K[1, 2])]
    st = dict(ai_px=0, ai_kept=0, ffs_px=0)
    for ci in range(n):
        A = AI[ci]; need = np.isfinite(A) & ~np.isfinite(FF[ci])
        v, u = np.nonzero(need); keep = np.zeros(len(v), bool)
        if len(v):
            z = torch.from_numpy(A[v, u]).to(dev); ut = torch.from_numpy(u).float().to(dev); vt = torch.from_numpy(v).float().to(dev)
            X = torch.stack([(ut - cx) / fx * z, (vt - cy) / fy * z, z], 1) @ Tc[ci, :3, :3].T + Tc[ci, :3, 3]
            agree = torch.zeros(len(z), dtype=torch.int32, device=dev)
            for j in range(n):
                if j == ci or np.linalg.norm(cen[j] - cen[ci]) < 0.10:
                    continue
                Xj = X @ Tw[j, :3, :3].T + Tw[j, :3, 3]; zj = Xj[:, 2]
                pu = torch.round(fx * Xj[:, 0] / zj.clamp(min=1e-3) + cx).long(); pv = torch.round(fy * Xj[:, 1] / zj.clamp(min=1e-3) + cy).long()
                ok = (zj > 0.05) & (pu >= 0) & (pu < W) & (pv >= 0) & (pv < H)
                zr = torch.full_like(zj, float("nan")); zr[ok] = At[j][pv[ok], pu[ok]]
                agree += (torch.abs(zr - zj) < 0.03 * zj).int()
            keep = (agree >= gate).cpu().numpy()
        Z = np.where(np.isfinite(FF[ci]), FF[ci], np.nan).astype(np.float32); Fz = np.isfinite(Z).astype(np.uint8)
        Z[v[keep], u[keep]] = A[v[keep], u[keep]]; Fz[v[keep], u[keep]] = 2
        np.savez_compressed(out / f"{kf[ci]:05d}.npz", Z=Z, F=Fz)
        st["ai_px"] += int(len(v)); st["ai_kept"] += int(keep.sum()); st["ffs_px"] += int(np.isfinite(FF[ci]).sum())
    info = dict(made=time.strftime("%F %T"), src=src, da3=da3, scale=scale_info, zmax=zmax, gate=gate, **st,
                ai_kept_share=round(st["ai_kept"] / max(1, st["ai_px"]), 4), runtime_s=round(time.time() - t0, 1))
    (out / "fuse_info.json").write_text(json.dumps(info, indent=1)); print(json.dumps(info), flush=True)


if __name__ == "__main__":
    import gpu_guard
    with gpu_guard.job("gsmap e6fuse", torch_cap=True):
        main()
