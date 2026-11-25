#!/usr/bin/env python3
"""gsmap 2026-11-25: filtered FFS + MoGe-2 depth (same as pc_ffsmoge_f.ply) -> fuse/moge_f/<kf>.npz -> scanA TSDFMap mesh.

  ~/scanner/gsmap_env/venv/bin/python scan3d/gsmap/gsm_filtmesh.py OUT.ply
FFS pixels dropped if: multi-view free-space check (>= 2 views and >= 30 % of checkable views, 3 cm / 3 %), within 3 px of a
> 3 % depth jump, or > 2 m. MoGe-2 (aligned, fuse/moge Zai) fills dropped + missing pixels where it passes the 3 % edge cut
(median5) and is < 4 m. Black undistortion border masked (eroded 3 px). Depth <= 3 m.
TSDF: tsdf_map.TSDFMap voxel 5 mm, pixel sigma 1 cm -> trunc 2 cm (trunc_max 0.02), min_obs 2, all 134 keyframes, BA poses,
colour from the 960 keyframes.
"""
import json, sys, time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / "gsplat"))
import os
R = Path(os.environ.get("GSMAP_RUN", str(HERE.parent / "runs" / "gsmap_20261125")))


def main():
    import torch, tsdf_map
    outp = Path(sys.argv[1]); t0 = time.time()
    c = np.load(R / "dump_ba" / "cams.npz"); kf, T, Kr = c["kf"], c["T"], c["Kr"]; W, H = [int(v) for v in c["size"]]; n = len(kf)
    fx, fy, cx, cy = [float(v) for v in (Kr[0, 0], Kr[1, 1], Kr[0, 2], Kr[1, 2])]
    mx, my = cv2.initUndistortRectifyMap(c["K"], c["dist"], None, Kr, (W, H), cv2.CV_32FC1)
    vm = cv2.erode(((mx >= 0) & (mx <= W - 1) & (my >= 0) & (my <= H - 1)).astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)
    FF = [np.load(R / "fuse" / "ffs" / f"{i:05d}.npz")["Z"] for i in kf]
    MG = [np.load(R / "fuse" / "moge" / f"{i:05d}.npz")["Zai"].astype(np.float32) for i in kf]
    dev = "cuda"
    ffs = torch.from_numpy(np.stack(FF)).to(dev); has = [bool(np.isfinite(z).any()) for z in FF]
    Tw = torch.from_numpy(np.linalg.inv(T)).float().to(dev); Tc = torch.from_numpy(T).float().to(dev)
    k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    od = R / "fuse" / "moge_f"; od.mkdir(exist_ok=True)
    tsdf = tsdf_map.TSDFMap(voxel_m=0.005, z_max=3.0, sigma_max=0.08, sigma_ok=0.02, min_obs=2, trunc_max=0.02, max_blocks=400000)
    st = dict(ffs=0, mv=0, edge=0, far=0, drop=0, px_ffs=0, px_moge=0)
    for c_ in range(n):
        Z = FF[c_]; val = np.isfinite(Z) & vm
        mv = np.zeros((H, W), bool); edge = np.zeros((H, W), bool)
        if val.any():
            v, u = np.nonzero(val); z = torch.from_numpy(Z[v, u]).float().to(dev)
            X = torch.stack([(torch.from_numpy(u).float().to(dev) - cx) / fx * z, (torch.from_numpy(v).float().to(dev) - cy) / fy * z, z], 1) @ Tc[c_, :3, :3].T + Tc[c_, :3, 3]
            viol = torch.zeros(len(z), dtype=torch.int32, device=dev); chk = torch.zeros_like(viol)
            for j in range(n):
                if j == c_ or not has[j]:
                    continue
                Xj = X @ Tw[j, :3, :3].T + Tw[j, :3, 3]; zj = Xj[:, 2]
                pu = torch.round(fx * Xj[:, 0] / zj.clamp(min=1e-3) + cx).long(); pv = torch.round(fy * Xj[:, 1] / zj.clamp(min=1e-3) + cy).long()
                ok = (zj > 0.05) & (pu >= 0) & (pu < W) & (pv >= 0) & (pv < H)
                zr = torch.full_like(zj, float("nan")); zr[ok] = ffs[j][pv[ok], pu[ok]]
                chk += torch.isfinite(zr).int(); viol += (torch.isfinite(zr) & (zj < zr - torch.clamp(0.03 * zr, min=0.03))).int()
            d = ((viol >= 2) & (viol >= 0.3 * chk)).cpu().numpy(); mv[v[d], u[d]] = True
            Z0 = np.nan_to_num(Z); Zp = np.pad(Z0, 1, mode="edge"); jump = np.zeros((H, W), bool)
            for dy, dx in [(0, 1), (2, 1), (1, 0), (1, 2)]:
                N = Zp[dy:dy + H, dx:dx + W]; jump |= (N > 0) & (Z0 > 0) & (np.abs(N - Z0) / np.maximum(Z0, 1e-3) > 0.03)
            edge = cv2.dilate(jump.astype(np.uint8), k3).astype(bool) & val
        far = val & (Z > 2.0); drop = val & (mv | edge | far); keep = val & ~drop
        Za = MG[c_]; med = cv2.medianBlur(np.nan_to_num(Za, nan=0).astype(np.float32), 5)
        mok = vm & np.isfinite(Za) & (Za > 0.05) & (Za < 4.0) & (np.abs(Za - med) / np.maximum(Za, 1e-3) < 0.03)
        Zo = np.where(keep, Z, np.where(mok, Za, np.nan)).astype(np.float32)
        Fo = np.where(keep, 1, np.where(mok, 2, 0)).astype(np.uint8)
        Vv = np.isfinite(Zo) & (Zo <= 3.0) & vm
        Fo[~Vv] = 0; Zo[~Vv] = np.nan
        np.savez_compressed(od / f"{kf[c_]:05d}.npz", Z=Zo, F=Fo)
        for k_, x in dict(ffs=val, mv=mv, edge=edge, far=far, drop=drop).items():
            st[k_] += int(x.sum())
        st["px_ffs"] += int((Fo == 1).sum()); st["px_moge"] += int((Fo == 2).sum())
        if os.environ.get("GSMAP_NO_MESH"):
            continue
        img = cv2.imread(str(R / "dump_ba" / "img" / f"{kf[c_]:05d}.png"))
        tsdf.integrate(Zo, Vv, img, fx, cx, cy, 0.08, T[c_], sigma=np.where(Vv, 0.01, np.inf).astype(np.float32))
    if os.environ.get("GSMAP_NO_MESH"):
        Vm, Fm = np.zeros((0, 3)), np.zeros((0, 3))
    else:
        tsdf.update_mesh(force=True)
        Vm, Cm, Fm = tsdf.mesh_arrays()
        tsdf_map.write_mesh_ply(outp, Vm, Cm, Fm)
    info = dict(made=time.strftime("%F %T"), faces=int(len(Fm)), verts=int(len(Vm)), runtime_s=round(time.time() - t0, 1),
                drop_share_of_ffs={k: round(st[k] / max(1, st["ffs"]), 4) for k in ("mv", "edge", "far", "drop")},
                px_moge_share=round(st["px_moge"] / max(1, st["px_ffs"] + st["px_moge"]), 4))
    (R / "fuse" / "moge_f" / "mesh_info.json").write_text(json.dumps(info, indent=1)); print(json.dumps(info), flush=True)


if __name__ == "__main__":
    import gpu_guard
    with gpu_guard.job("gsmap filtmesh", torch_cap=True):
        main()
