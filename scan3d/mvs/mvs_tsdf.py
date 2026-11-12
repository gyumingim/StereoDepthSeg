#!/usr/bin/env python3
"""mvs 11-12: put per-keyframe depth maps from ANY source into OUR TSDF (tsdf_map.TSDFMap) with the baseline mesh-step settings
(scan_offline --depth-sigma ba: voxel 5 mm, z_max 2.0, sigma_max 0.08, sigma_ok 0.02, min_obs 2, alloc dense, max_blocks 250000),
so that only the depth source changes. Same keyframes / poses / 960x540 undistorted grid / colours (mvs_prep.py).

Sources:
  colmap WS      : WS/stereo/depth_maps/NAME.geometric.bin (960 or 1920; 1920 -> 960 by 2x2 median of valid, >= 2 valid)
  mvsf DIR       : DIR/depth/NAME.npy (float32 m, 0 = invalid, already filtered), any size -> 960 (same 2x2 rule or nearest)
  base DIR       : scan_offline --dump-depth DIR/kf_XXXX.npz (Z01mm uint16, 0 = invalid) — baseline replay (sanity check)
Depth sigma for MVS sources (no per-pixel sigma from the methods) = the FFS motion-pair model of scan_offline 'ba':
  sigma = hypot(Z^2 * hypot(0.5 px, f * 0.1 deg) / (f * B), 0.02 * Z), B = median baseline of the keyframe's source views (views.json).
  This is an ASSUMPTION (same noise model as the baseline, not measured for MVS).
Usage: mvs_tsdf.py PREP OUT --source colmap|mvsf|base --path P [--zmax 2.0] [--max-blocks 250000] [--evict-gpu-blocks 0]"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import tsdf_map  # noqa: E402


def read_colmap_array(p):
    with open(p, "rb") as f:
        hdr = b""
        n = 0
        while n < 3:
            c = f.read(1)
            hdr += c
            n += c == b"&"
        w, h, ch = map(int, hdr.decode().strip("&").split("&"))
        a = np.fromfile(f, np.float32)
    return a.reshape((w, h, ch), order="F").transpose(1, 0, 2).squeeze()


def to960(D, W0=960, H0=540):
    if D.shape == (H0, W0):
        return D
    if D.shape == (2 * H0, 2 * W0):
        b = D.reshape(H0, 2, W0, 2).transpose(0, 2, 1, 3).reshape(H0, W0, 4)
        b = np.where(b > 0, b, np.nan)
        nv = np.isfinite(b).sum(-1)
        with np.errstate(all="ignore"):
            import warnings
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                m = np.nanmedian(b, -1)
        return np.where(nv >= 2, m, 0).astype(np.float32)
    return cv2.resize(D, (W0, H0), interpolation=cv2.INTER_NEAREST)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("prep"); ap.add_argument("out")
    ap.add_argument("--source", required=True, choices=("colmap", "mvsf", "base"))
    ap.add_argument("--path", required=True)
    ap.add_argument("--zmax", type=float, default=2.0)
    ap.add_argument("--max-blocks", type=int, default=250000)
    ap.add_argument("--evict-gpu-blocks", type=int, default=0)
    ap.add_argument("--sigma-rot-deg", type=float, default=0.1)
    ap.add_argument("--sigma-b-rel", type=float, default=0.02)
    ap.add_argument("--sigma-d-px", type=float, default=0.5)
    a = ap.parse_args()
    import torch
    t0 = time.time()
    P, out, src = Path(a.prep), Path(a.out), Path(a.path)
    out.mkdir(parents=True, exist_ok=True)
    C = np.load(P / "cams.npz")
    names = [str(x) for x in C["names"]]
    T = C["T"]; K = C["K960"]; f, cx, cy = float(K[0, 0]), float(K[0, 2]), float(K[1, 2])
    V = json.loads((P / "views.json").read_text())
    kw = dict(voxel_m=0.005, z_max=a.zmax, sigma_max=0.08, sigma_ok=0.02, min_obs=2, alloc_dense=True, max_blocks=a.max_blocks)
    msgs = []
    kw["log"] = lambda m: (print(m, flush=True), msgs.append(str(m)))
    if a.evict_gpu_blocks > 0:
        import tsdf_evict
        tsdf = tsdf_evict.EvictTSDFMap(gpu_blocks=a.evict_gpu_blocks, cpu_dtype="f32", disk_dir=str(out / "tsdf_spill"), **kw)
    else:
        tsdf = tsdf_map.TSDFMap(**kw)
    sdp = float(np.hypot(a.sigma_d_px, f * np.radians(a.sigma_rot_deg)))
    integ, rows = [], []
    t_read = t_int = 0.0
    for i, n in enumerate(names):
        tr = time.time()
        if a.source == "colmap":
            p = src / "stereo" / "depth_maps" / f"{n}.geometric.bin"
            D = read_colmap_array(p) if p.exists() else None
        elif a.source == "mvsf":
            p = src / "depth" / (Path(n).stem + ".npy")
            D = np.load(p) if p.exists() else None
        else:
            p = src / f"kf_{i:04d}.npz"
            D = np.load(p)["Z01mm"].astype(np.float32) / 1e4 if p.exists() else None
        t_read += time.time() - tr
        if D is None:
            rows.append(dict(i=i, name=n, depth=False)); continue
        D = to960(np.asarray(D, np.float32))
        Vd = np.isfinite(D) & (D > 0)
        B = float(np.median(V[n]["B_m"])) if V[n]["B_m"] else 0.1
        S = np.hypot(D * D * sdp / (f * B), a.sigma_b_rel * D).astype(np.float32)
        if a.source == "base":            # baseline replay: its own sigma is not dumped -> same formula with its pair baseline unknown;
            pass                          # (only a sanity check of the pipeline; the real baseline is the scan_offline run)
        bgr = cv2.imread(str(P / "img960" / n))
        ti = time.time()
        tsdf.integrate(np.where(Vd, D, np.nan).astype(np.float32), Vd, bgr, f, cx, cy, 0.08, T[i], sigma=np.where(Vd, S, np.inf).astype(np.float32))
        t_int += time.time() - ti
        integ.append(i)
        rows.append(dict(i=i, name=n, depth=True, valid_frac=round(float(Vd.mean()), 4), B_m=B,
                         z_p50=round(float(np.median(D[Vd])), 3) if Vd.any() else None,
                         z_p99=round(float(np.percentile(D[Vd], 99)), 3) if Vd.any() else None,
                         frac_sigma_le_0p08=round(float((S[Vd] <= 0.08).mean()), 4) if Vd.any() else None))
        if i % 50 == 0:
            print(f"kf {i}/{len(names)} {rows[-1]}", flush=True)
    tm = time.time()
    tsdf.update_mesh(force=True)
    Vm, Cm, Fm = tsdf.mesh_arrays()
    tsdf_map.write_mesh_ply(out / "mesh.ply", Vm, Cm, Fm)
    t_mesh = time.time() - tm
    np.savez_compressed(out / "kf_poses.npz", i=np.array(integ, int), T=T[integ], f=f, cx=cx, cy=cy, size=np.array([960, 540]))
    summ = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), prep=str(P), source=a.source, path=str(src), args=vars(a), tsdf_cfg={k: v for k, v in kw.items() if k != "log"},
                sigma_model=dict(sigma_d_px=a.sigma_d_px, rot_deg=a.sigma_rot_deg, b_rel=a.sigma_b_rel, B="median baseline of views.json src"),
                n_kf=len(names), n_kf_depth=len(integ), mesh_vertices=int(len(Vm)), mesh_faces=int(len(Fm)),
                read_s=round(t_read, 1), integrate_s=round(t_int, 1), mesh_s=round(t_mesh, 1), runtime_s=round(time.time() - t0, 1),
                tsdf_full=bool(tsdf.full), tsdf_blocks=int(getattr(tsdf, "n_total", tsdf.nb)), tsdf_log=msgs,
                gpu_max_alloc_gb=round(torch.cuda.max_memory_allocated() / 2 ** 30, 2),
                valid_frac_median=float(np.median([r["valid_frac"] for r in rows if r.get("depth")])))
    if a.evict_gpu_blocks > 0:
        summ["tsdf_evict"] = tsdf.evict_stats(); tsdf.cpu.close(delete=True)
    (out / "summary.json").write_text(json.dumps(summ, indent=1))
    (out / "keyframes.json").write_text(json.dumps(rows, indent=0))
    print(json.dumps(summ, indent=1))


if __name__ == "__main__":
    main()
