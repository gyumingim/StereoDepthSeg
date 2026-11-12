#!/usr/bin/env python3
"""fixes 11-12 unit 12: replay recorded TSDF integrate inputs (SCAN3D_TSDF_RECORD=DIR on a scan_offline run) into
  (a) tsdf_map.TSDFMap (eviction OFF, max_blocks from the recorded cfg or --max-blocks) and
  (b) tsdf_evict.EvictTSDFMap (eviction ON, --gpu-blocks, disk spill under --spill),
then compare the final meshes: vertex / face counts, bit-identical arrays, else max nearest-vertex distance (scipy cKDTree).
  python tools/tsdf_evict_check.py REC_DIR --gpu-blocks 40000 --spill /tmp/spill [--cpu-dtype f16] [--skip-off] [--out res.json]
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import tsdf_map  # noqa: E402
import tsdf_evict  # noqa: E402


def replay(m, files):
    t0 = time.perf_counter()
    for fn in files:
        z = np.load(fn)
        m.integrate(z["Z"], z["valid"], z["bgr"], float(z["f"]), float(z["cx"]), float(z["cy"]), float(z["B"]), z["T"],
                    ffs_scale=float(z["s"]), t=float(z["t"]), sigma=z["sigma"] if "sigma" in z else None,
                    wmul=z["wmul"] if "wmul" in z else None)
    ti = time.perf_counter() - t0
    m.update_mesh(force=True)
    V, C, F = m.mesh_arrays()
    return V, C, F, dict(integrate_s=round(ti, 1), total_s=round(time.perf_counter() - t0, 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("--gpu-blocks", type=int, required=True)
    ap.add_argument("--max-blocks", type=int, default=None, help="total block limit for both maps (default: recorded cfg)")
    ap.add_argument("--cpu-dtype", default="f32", choices=("f32", "f16"))
    ap.add_argument("--spill", required=True, help="spill folder for the evicting map (deleted at the end)")
    ap.add_argument("--skip-off", action="store_true")
    ap.add_argument("--save-ply", default="", help="write the evict-ON mesh here")
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    import torch
    rec = Path(a.rec)
    cfg = json.loads((rec / "cfg.json").read_text())
    if a.max_blocks:
        cfg["max_blocks"] = a.max_blocks
    files = sorted(rec.glob("int_*.npz"))
    res = dict(made=time.strftime("%F %T"), rec=str(rec), n_frames=len(files), cfg=cfg, gpu_blocks=a.gpu_blocks, cpu_dtype=a.cpu_dtype)
    print(f"{len(files)} recorded integrate calls, max_blocks {cfg['max_blocks']}", flush=True)
    off = None
    if not a.skip_off:
        torch.cuda.reset_peak_memory_stats()
        m0 = tsdf_map.TSDFMap(**cfg)
        V0, C0, F0, t = replay(m0, files)
        res["off"] = dict(t, blocks=int(m0.nb), full=bool(m0.full), vertices=len(V0), faces=len(F0),
                          gpu_max_alloc_gb=round(torch.cuda.max_memory_allocated() / 2 ** 30, 3))
        off = (V0, C0, F0)
        print("OFF", res["off"], flush=True)
        del m0
        torch.cuda.empty_cache()
    torch.cuda.reset_peak_memory_stats()
    m1 = tsdf_evict.EvictTSDFMap(gpu_blocks=a.gpu_blocks, cpu_dtype=a.cpu_dtype, disk_dir=a.spill, **cfg)
    V1, C1, F1, t = replay(m1, files)
    res["on"] = dict(t, blocks=m1.n_total, full=bool(m1.full), vertices=len(V1), faces=len(F1),
                     gpu_max_alloc_gb=round(torch.cuda.max_memory_allocated() / 2 ** 30, 3), evict=m1.evict_stats())
    m1.cpu.close(delete=True)
    print("ON", res["on"], flush=True)
    if a.save_ply:
        tsdf_map.write_mesh_ply(a.save_ply, V1, C1, F1)
    if off is not None:
        V0, C0, F0 = off
        same = len(V0) == len(V1) and len(F0) == len(F1)
        cmp = dict(same_counts=same, bit_identical=bool(same and np.array_equal(V0, V1) and np.array_equal(C0, C1) and np.array_equal(F0, F1)))
        if not cmp["bit_identical"] and len(V0) and len(V1):
            from scipy.spatial import cKDTree
            d01 = cKDTree(V1).query(V0)[0]
            d10 = cKDTree(V0).query(V1)[0]
            cmp.update(max_dist_off_to_on_mm=float(d01.max() * 1000), max_dist_on_to_off_mm=float(d10.max() * 1000),
                       p99_off_to_on_mm=float(np.percentile(d01, 99) * 1000), frac_off_gt_1mm=float(np.mean(d01 > 1e-3)))
            if same:
                cmp.update(max_vertex_diff_mm=float(np.abs(V0 - V1).max() * 1000), color_max_diff=int(np.abs(C0.astype(int) - C1).max()),
                           faces_equal=bool(np.array_equal(F0, F1)))
        res["compare"] = cmp
        print("COMPARE", cmp, flush=True)
    if a.out:
        Path(a.out).write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
