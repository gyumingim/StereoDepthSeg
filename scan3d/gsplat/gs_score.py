#!/usr/bin/env python3
"""gsplat 2026-11-05: same scoring as scanA scan_offline.py, for any mesh (TSDF or 2DGS) and any per-keyframe depth maps.

  python scan3d/gsplat/gs_score.py --cams DUMP/cams.npz --mesh MESH.ply --kf-json TSDF_RUN/keyframes.json
         [--depth-dir DIR (npz with Z per keyframe, e.g. GS render or DUMP/ffs)] --out score.json
(b) mesh depth dZ: ray-cast the mesh (open3d RaycastingScene) from every keyframe of the evaluation set (keyframes of the
    TSDF run that had a board depth score) -> Z - Z_plane(GT pose), board rectangle minus 1 cm (scan_offline formulas).
(a) depth-map dZ: same, from the given depth maps (alpha/valid already applied = NaN outside).
(c) mesh plane: vertices over the board (2 cm margin, |z| < 10 cm): z median / RMS / p5-p95, coverage of 1 cm cells
    (scan_offline formulas copied verbatim).
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np


def stats(rows, dz_all, key=""):
    ev = [r for r in rows if "board_dZ_med_cm" in r]
    s = dict(n_kf_eval=len(ev))
    if not ev:
        return s
    for k in ("board_dZ_mean_cm", "board_dZ_med_cm", "board_absdZ_p50_cm", "board_absdZ_mean_cm"):
        v = np.array([r[k] for r in ev])
        s[f"kf_{k}_stats"] = dict(mean=float(v.mean()), median=float(np.median(v)), p10=float(np.percentile(v, 10)),
                                  p90=float(np.percentile(v, 90)))
    D = np.concatenate(dz_all) * 100
    s["pooled_dZ_cm"] = dict(n=int(len(D)), mean=float(D.mean()), median=float(np.median(D)), p10=float(np.percentile(D, 10)),
                             p90=float(np.percentile(D, 90)))
    s["pooled_absdZ_cm"] = dict(mean=float(np.abs(D).mean()), median=float(np.median(np.abs(D))), p90=float(np.percentile(np.abs(D), 90)))
    s["kf_frac_abs_bias_gt_1cm"] = float(np.mean([abs(r["board_dZ_med_cm"]) > 1 for r in ev]))
    s["board_valid_px_total"] = int(sum(r["board_valid"] for r in ev))
    s["board_px_total"] = int(sum(r["board_px"] for r in rows))
    return s


def score_depth(Zs, cams, kfs):
    """Zs: dict kf -> (H,W) depth (NaN = none). Same board mask + dZ as scan_offline."""
    Kr, Tgt, kf_all = cams["Kr"], cams["Tgt"], list(cams["kf"])
    W, H = [int(v) for v in cams["size"]]
    BW, BH = float(cams["BW"]), float(cams["BH"])
    uu, vv = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
    rays = np.stack([(uu - Kr[0, 2]) / Kr[0, 0], (vv - Kr[1, 2]) / Kr[1, 1], np.ones_like(uu)], -1)
    rows, dz_all = [], []
    for i in kfs:
        c = kf_all.index(i)
        Rw, pw = Tgt[c][:3, :3], Tgt[c][:3, 3]
        rz = rays @ Rw[2]
        with np.errstate(divide="ignore", invalid="ignore"):
            Zp = -pw[2] / rz
        Xw = pw[None, None] + Zp[..., None] * (rays @ Rw.T)
        inb = (Xw[..., 0] > 0.01) & (Xw[..., 0] < BW - 0.01) & (Xw[..., 1] > 0.01) & (Xw[..., 1] < BH - 0.01) & (Zp > 0)
        Z = Zs.get(i)
        row = dict(i=int(i), board_px=int(inb.sum()), board_valid=0)
        if Z is not None:
            m = inb & np.isfinite(Z)
            row["board_valid"] = int(m.sum())
            if m.sum() > 200:
                dz = (Z[m] - Zp[m])
                dz_all.append(dz.astype(np.float32))
                row.update(board_dZ_mean_cm=float(dz.mean() * 100), board_dZ_med_cm=float(np.median(dz) * 100),
                           board_absdZ_p50_cm=float(np.median(np.abs(dz)) * 100), board_absdZ_mean_cm=float(np.abs(dz).mean() * 100),
                           board_absdZ_p90_cm=float(np.percentile(np.abs(dz), 90) * 100))
        rows.append(row)
    return rows, dz_all


def mesh_plane(Vm, BW, BH):
    summ = {}
    mb = (Vm[:, 0] > 0.02) & (Vm[:, 0] < BW - 0.02) & (Vm[:, 1] > 0.02) & (Vm[:, 1] < BH - 0.02) & (np.abs(Vm[:, 2]) < 0.10)
    if mb.sum() > 20:
        z = Vm[mb, 2]
        summ.update(mesh_board_vertices=int(mb.sum()), mesh_board_z_med_cm=float(np.median(z) * 100),
                    mesh_board_z_rms_cm=float(np.sqrt(np.mean((z - np.median(z)) ** 2)) * 100),
                    mesh_board_z_p5_p95_cm=[float(np.percentile(z, 5) * 100), float(np.percentile(z, 95) * 100)])
        near = mb & (np.abs(Vm[:, 2]) < 0.03)
        cx_ = np.floor((Vm[near, 0] - 0.02) * 100).astype(int); cy_ = np.floor((Vm[near, 1] - 0.02) * 100).astype(int)
        nx_, ny_ = int(np.ceil((BW - 0.04) * 100)), int(np.ceil((BH - 0.04) * 100))
        cells = set(zip(cx_.clip(0, nx_ - 1), cy_.clip(0, ny_ - 1)))
        summ["mesh_board_cover_frac"] = len(cells) / float(nx_ * ny_)
    return summ


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cams", required=True)
    ap.add_argument("--mesh", required=True)
    ap.add_argument("--kf-json", required=True, help="TSDF run keyframes.json: evaluation keyframe set")
    ap.add_argument("--depth-dir", default="")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    import open3d as o3d
    cams = np.load(a.cams)
    kfr = json.loads(Path(a.kf_json).read_text())
    kfs = [r["i"] for r in kfr if "board_absdZ_p50_cm" in r]
    BW, BH = float(cams["BW"]), float(cams["BH"])
    mesh = o3d.io.read_triangle_mesh(a.mesh)
    Vm = np.asarray(mesh.vertices)
    out = dict(made=time.strftime("%F %T"), cams=a.cams, mesh=a.mesh, kf_json=a.kf_json, depth_dir=a.depth_dir,
               n_kf_eval_set=len(kfs), mesh_vertices=int(len(Vm)), mesh_faces=int(len(mesh.triangles)))
    out.update(mesh_plane(Vm, BW, BH))
    # (b) ray-cast mesh depth
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    Kr, T, kf_all = cams["Kr"], cams["T"], list(cams["kf"])
    W, H = [int(v) for v in cams["size"]]
    uu, vv = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
    rays = np.stack([(uu - Kr[0, 2]) / Kr[0, 0], (vv - Kr[1, 2]) / Kr[1, 1], np.ones_like(uu)], -1)
    Zm = {}
    for i in kfs:
        c = kf_all.index(i)
        d = rays @ T[c][:3, :3].T                       # unnormalised: t_hit = camera z depth
        o = np.broadcast_to(T[c][:3, 3], d.shape)
        r = o3d.core.Tensor(np.concatenate([o, d], -1).reshape(-1, 6).astype(np.float32))
        th = scene.cast_rays(r)["t_hit"].numpy().reshape(H, W)
        Zm[i] = np.where(np.isfinite(th), th, np.nan)
    rows, dz = score_depth(Zm, cams, kfs)
    out["mesh_raycast"] = stats(rows, dz)
    out["mesh_raycast_kf"] = rows
    if a.depth_dir:
        Zd = {}
        for i in kfs:
            p = Path(a.depth_dir) / f"{i:05d}.npz"
            if p.exists():
                Zd[i] = np.load(p)["Z"].astype(np.float64)
        rows, dz = score_depth(Zd, cams, kfs)
        out["depth_map"] = stats(rows, dz)
    Path(a.out).write_text(json.dumps(out, indent=1))
    print(json.dumps({k: v for k, v in out.items() if k != "mesh_raycast_kf"}, indent=1))


if __name__ == "__main__":
    main()
