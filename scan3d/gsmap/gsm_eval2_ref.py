#!/usr/bin/env python3
"""gsmap 2026-11-25 E0: build the FIXED references of eval2 once (plan gsmap_perfect_plan_20261125.md, section 2).

  GSMAP_RUN=<v2 run> python scan3d/gsmap/gsm_eval2_ref.py OLD_RUN OUTDIR [--gs GAUSSIANS.pt] [--mesh ROOM_MESH.ply]
Writes OUTDIR/:
* gold_ffs.npz   old-run filtered FFS (fuse/moge_f F == 1) at 480x270 (every 2nd px), kept only where >= 3 other keyframes
                 with >= 10 cm baseline confirm it within 2 cm + 1 % (their old filtered FFS). Z float16, 0 = none.
* gs_depth_all.npz  2DGS median depth + alpha (uint8) of ALL keyframes at 480x270 (one GPU render pass of --gs).
* room_poly.json + room_poly.png  floor plane (from --mesh, v3) + XZ footprint of mesh vertices between floor+0.3 m and
                 floor+1.8 m, 2 cm grid, closed 0.8 m, outer contour. Also the ceiling plane (v3 fit) for S4.
* rois.json      6 hand-picked flat ROIs (keyframe index, rect x0 x1 y0 y1 at 960x540), picked on eval2/ref/roi_pick.jpg.
* novel_poses.json  24 inside poses = 12 keyframes x (yaw -30 / +30 deg, both pitched +25 deg up).
* ref_info.json  made time + inputs.
"""
import json, os, sys, time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "gsplat"))
R = Path(os.environ["GSMAP_RUN"])
ROIS = [dict(name="panel_kf813", idx=80, rect=[715, 930, 110, 515]), dict(name="door_kf625", idx=48, rect=[440, 640, 100, 480]),
        dict(name="wall_kf538", idx=36, rect=[230, 500, 40, 220]), dict(name="wall_kf838", idx=84, rect=[180, 300, 40, 300]),
        dict(name="cabinet_front_kf774", idx=72, rect=[560, 720, 250, 500]), dict(name="floor_kf3205", idx=444, rect=[120, 480, 260, 500])]


def rot(Tv, yaw, pitch, piv_d=1.2):
    piv = Tv[:3, 3] + Tv[:3, 2] * piv_d; a, b = np.radians(yaw), np.radians(pitch)
    Ry = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]]); Rx = np.array([[1, 0, 0], [0, np.cos(b), -np.sin(b)], [0, np.sin(b), np.cos(b)]])
    Rw = Tv[:3, :3] @ (Ry @ Rx) @ Tv[:3, :3].T; Tn = np.eye(4); Tn[:3, :3] = Rw @ Tv[:3, :3]; Tn[:3, 3] = piv + Rw @ (Tv[:3, 3] - piv); return Tn


def main():
    import torch
    old, out = Path(sys.argv[1]), Path(sys.argv[2]); out.mkdir(parents=True, exist_ok=True)
    gsp = sys.argv[sys.argv.index("--gs") + 1] if "--gs" in sys.argv else str(R / "gs2DM" / "gaussians.pt")
    meshp = sys.argv[sys.argv.index("--mesh") + 1] if "--mesh" in sys.argv else str(R / "v3" / "mesh_v3.ply")
    t0 = time.time()
    c = np.load(R / "dump_ba" / "cams.npz"); kf, T, K = c["kf"], c["T"], c["Kr"]; W, H = [int(v) for v in c["size"]]; n = len(kf)
    w, h = W // 2, H // 2; K2 = K.copy(); K2[:2] /= 2
    # ---- gold FFS
    Z = np.stack([np.where(np.load(old / "fuse" / "moge_f" / f"{i:05d}.npz")["F"] == 1, np.load(old / "fuse" / "moge_f" / f"{i:05d}.npz")["Z"], np.nan)[::2, ::2] for i in kf]).astype(np.float32)
    dev = "cuda"; Zt = torch.from_numpy(Z).to(dev); Tw = torch.from_numpy(np.linalg.inv(T)).float().to(dev); Tc = torch.from_numpy(T).float().to(dev)
    fx, fy, cx, cy = [float(v) for v in (K2[0, 0], K2[1, 1], K2[0, 2], K2[1, 2])]
    cen = T[:, :3, 3]; gold = np.zeros_like(Z); has = [bool(torch.isfinite(Zt[j]).any()) for j in range(n)]
    for ci in range(n):
        v, u = np.nonzero(np.isfinite(Z[ci]))
        if len(v) == 0:
            continue
        z = Zt[ci][v, u]; ut = torch.from_numpy(u).float().to(dev); vt = torch.from_numpy(v).float().to(dev)
        X = torch.stack([(ut - cx) / fx * z, (vt - cy) / fy * z, z], 1) @ Tc[ci, :3, :3].T + Tc[ci, :3, 3]
        conf = torch.zeros(len(z), dtype=torch.int32, device=dev)
        for j in range(n):
            if j == ci or not has[j] or np.linalg.norm(cen[j] - cen[ci]) < 0.10:
                continue
            Xj = X @ Tw[j, :3, :3].T + Tw[j, :3, 3]; zj = Xj[:, 2]
            pu = torch.round(fx * Xj[:, 0] / zj.clamp(min=1e-3) + cx).long(); pv = torch.round(fy * Xj[:, 1] / zj.clamp(min=1e-3) + cy).long()
            ok = (zj > 0.05) & (pu >= 0) & (pu < w) & (pv >= 0) & (pv < h)
            zr = torch.full_like(zj, float("nan")); zr[ok] = Zt[j][pv[ok], pu[ok]]
            conf += (torch.abs(zr - zj) < 0.02 + 0.01 * zj).int()
        g = (conf >= 3).cpu().numpy(); gold[ci][v[g], u[g]] = Z[ci][v[g], u[g]]
    gold = np.nan_to_num(gold, nan=0.0)
    np.savez_compressed(out / "gold_ffs.npz", Z=gold.astype(np.float16), stride=2)
    info = dict(gold_px=int((gold > 0).sum()), ffs_px=int(np.isfinite(Z).sum()), gold_share_of_ffs=round(float((gold > 0).sum() / np.isfinite(Z).sum()), 4),
                gold_kf=int(((gold > 0).sum((1, 2)) > 0).sum()))
    print("gold", info, flush=True)
    # ---- 2DGS depth, all keyframes
    from gsplat import rasterization_2dgs
    ck = torch.load(gsp, weights_only=False); p = {k: v.cuda() for k, v in ck["params"].items()}
    Kt = torch.from_numpy(K2).float().cuda(); ZM, AL = [], []
    with torch.no_grad():
        for ci in range(n):
            r = rasterization_2dgs(p["means"], torch.nn.functional.normalize(p["quats"], dim=-1), torch.exp(p["scales"]), torch.sigmoid(p["opacities"]),
                                   torch.cat([p["sh0"], p["shN"]], 1), torch.from_numpy(np.linalg.inv(T[ci])).float().cuda()[None], Kt[None], w, h,
                                   sh_degree=ck["sh_degree"], render_mode="RGB+ED", near_plane=0.02, far_plane=20.0)
            ZM.append(r[5][0, ..., 0].float().cpu().numpy().astype(np.float16)); AL.append((r[1][0, ..., 0].cpu().numpy() * 255).astype(np.uint8))
    np.savez_compressed(out / "gs_depth_all.npz", ZM=np.stack(ZM), AL=np.stack(AL), stride=2)
    del p; torch.cuda.empty_cache()
    # ---- room polygon + planes (from the v3 mesh and gsm_v3fix planes)
    import open3d as o3d
    m = o3d.io.read_triangle_mesh(meshp); V = np.asarray(m.vertices)
    v3 = json.loads((R / "v3" / "mesh_v3.json").read_text())
    nfl = np.array(v3["floor"]["normal"]); yfl = v3["floor"]["y_m"]; nce = np.array(v3["ceiling"]["normal"]); yce = v3["ceiling"]["y_m"]
    band = (V[:, 1] < yfl - 0.3) & (V[:, 1] > yfl - 1.8)
    q = V[band][:, [0, 2]]; lo = q.min(0) - 0.5; g = np.floor((q - lo) / 0.02).astype(int); gs = g.max(0) + 50
    occ = np.zeros((gs[1], gs[0]), np.uint8); occ[g[:, 1], g[:, 0]] = 1
    cl = cv2.morphologyEx(occ, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (41, 41)))
    cnts, _ = cv2.findContours(cl, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE); big = max(cnts, key=cv2.contourArea)
    poly = (big[:, 0, :] * 0.02 + lo).tolist()
    img = np.zeros((gs[1], gs[0], 3), np.uint8); img[occ > 0] = (90, 90, 90); cv2.drawContours(img, [big], -1, (0, 255, 0), 2)
    for pp in T[:, [0, 2], 3]:
        q_ = ((pp - lo) / 0.02).astype(int); cv2.circle(img, tuple(int(x) for x in q_), 2, (0, 200, 255), -1)
    cv2.imwrite(str(out / "room_poly.png"), img)
    area = cv2.contourArea(big) * 4e-4
    (out / "room_poly.json").write_text(json.dumps(dict(poly_xz=poly, area_m2=round(area, 2), floor=dict(normal=nfl.tolist(), y_m=yfl),
                                                         ceiling=dict(normal=nce.tolist(), y_m=yce), note="x,z in world (y down); band floor+0.3..1.8 m")))
    (out / "rois.json").write_text(json.dumps(ROIS, indent=1))
    pick = np.linspace(0, n - 1, 12).astype(int)
    nov = [dict(idx=int(j), yaw=yw, pitch=25, T=rot(T[j], yw, 25).tolist()) for j in pick for yw in (-30, 30)]
    (out / "novel_poses.json").write_text(json.dumps(nov))
    info.update(made=time.strftime("%F %T"), old_run=str(old), gs=gsp, room_mesh=meshp, room_area_m2=round(area, 2), runtime_s=round(time.time() - t0, 1),
                held_out="keyframe index % 8 == 0")
    (out / "ref_info.json").write_text(json.dumps(info, indent=1)); print(json.dumps(info), flush=True)


if __name__ == "__main__":
    import gpu_guard
    with gpu_guard.job("gsmap eval2 ref", torch_cap=True):
        main()
