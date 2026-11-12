"""fixes 11-12 unit 11: compare two scan_offline runs (PyTorch FFS vs TensorRT FFS) on the same keyframes / poses.

python trt_compare.py --a DUMP_A --b DUMP_B --sa A/summary.json --sb B/summary.json [--ka A/keyframes.json --kb ...] --out cmp.json
Dumps = scan_offline --dump-depth (fused keyframe depth, uint16 0.1 mm, 0 = invalid).
Per keyframe: pixels valid in both -> |dZ| median / p95 (mm) and |dZ|/Z; valid counts A, B, both, only-A, only-B.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--a", required=True); ap.add_argument("--b", required=True)
    ap.add_argument("--sa", required=True); ap.add_argument("--sb", required=True)
    ap.add_argument("--ka", default=""); ap.add_argument("--kb", default="")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    fa = {p.name: p for p in Path(a.a).glob("kf_*.npz")}
    fb = {p.name: p for p in Path(a.b).glob("kf_*.npz")}
    names = sorted(set(fa) & set(fb))
    per, pooled_abs, pooled_rel = [], [], []
    same_partners = 0
    for nm in names:
        A, B = np.load(fa[nm]), np.load(fb[nm])
        za, zb = A["Z01mm"].astype(np.float64) * 1e-4, B["Z01mm"].astype(np.float64) * 1e-4
        va, vb = za > 0, zb > 0
        both = va & vb
        same_partners += int(np.array_equal(A["partners"], B["partners"]))
        r = dict(kf=nm, n_a=int(va.sum()), n_b=int(vb.sum()), n_both=int(both.sum()), only_a=int((va & ~vb).sum()),
                 only_b=int((vb & ~va).sum()), scale=[float(x) for x in A["scale"]])
        if both.sum() > 100:
            d = np.abs(za[both] - zb[both])
            rel = d / za[both]
            r.update(absdz_med_mm=float(np.median(d) * 1e3), absdz_p95_mm=float(np.percentile(d, 95) * 1e3),
                     rel_med=float(np.median(rel)), rel_p95=float(np.percentile(rel, 95)))
            sub = np.random.default_rng(0).choice(len(d), min(len(d), 20000), replace=False)
            pooled_abs.append(d[sub]); pooled_rel.append(rel[sub])
        per.append(r)
    ev = [r for r in per if "absdz_med_mm" in r]
    P = np.concatenate(pooled_abs) * 1e3 if pooled_abs else np.zeros(1)
    PR = np.concatenate(pooled_rel) if pooled_rel else np.zeros(1)
    sa, sb = json.loads(Path(a.sa).read_text()), json.loads(Path(a.sb).read_text())

    def ffs_ms(kpath):
        if not kpath or not Path(kpath).exists():
            return None
        ms = [m for r in json.loads(Path(kpath).read_text()) for m in r.get("ffs_ms", [])]
        return dict(n=len(ms), median=float(np.median(ms)), p90=float(np.percentile(ms, 90)), mean=float(np.mean(ms))) if ms else None

    out = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), inputs=vars(a), n_kf_both=len(names), n_kf_only_a=len(set(fa) - set(fb)),
               n_kf_only_b=len(set(fb) - set(fa)), n_kf_same_partners=same_partners, n_kf_eval=len(ev),
               kf_absdz_med_mm_median=float(np.median([r["absdz_med_mm"] for r in ev])) if ev else None,
               kf_absdz_p95_mm_median=float(np.median([r["absdz_p95_mm"] for r in ev])) if ev else None,
               kf_absdz_p95_mm_p90=float(np.percentile([r["absdz_p95_mm"] for r in ev], 90)) if ev else None,
               kf_rel_med_median=float(np.median([r["rel_med"] for r in ev])) if ev else None,
               kf_rel_p95_median=float(np.median([r["rel_p95"] for r in ev])) if ev else None,
               pooled_absdz_mm=dict(median=float(np.median(P)), p95=float(np.percentile(P, 95)), p99=float(np.percentile(P, 99))),
               pooled_frac_absdz_gt_10mm=float(np.mean(P > 10)), pooled_frac_absdz_gt_50mm=float(np.mean(P > 50)),
               kf_frac_p95_gt_10mm=float(np.mean([r["absdz_p95_mm"] > 10 for r in ev])) if ev else None,
               pooled_rel=dict(median=float(np.median(PR)), p95=float(np.percentile(PR, 95))),
               valid_px_a=int(sum(r["n_a"] for r in per)), valid_px_b=int(sum(r["n_b"] for r in per)),
               valid_px_both=int(sum(r["n_both"] for r in per)),
               mesh_vertices=[sa.get("mesh_vertices"), sb.get("mesh_vertices")], mesh_faces=[sa.get("mesh_faces"), sb.get("mesh_faces")],
               n_kf_depth=[sa.get("n_kf_depth"), sb.get("n_kf_depth")], runtime_s=[sa.get("runtime_s"), sb.get("runtime_s")],
               ffs_s_total=[sa.get("ffs_s_total"), sb.get("ffs_s_total")], ffs_ms=[ffs_ms(a.ka), ffs_ms(a.kb)],
               gpu_max_alloc_gb=[sa.get("gpu_max_alloc_gb"), sb.get("gpu_max_alloc_gb")],
               gpu_max_reserved_gb=[sa.get("gpu_max_reserved_gb"), sb.get("gpu_max_reserved_gb")],
               board=[{k: s.get(k) for k in ("kf_board_absdZ_p50_cm_median", "kf_board_dZ_bias_cm_median", "mesh_board_z_rms_cm",
                                             "mesh_board_cover_frac")} for s in (sa, sb)],
               ffs_trt=sb.get("ffs_trt"), per_kf=per)
    Path(a.out).write_text(json.dumps(out, indent=1))
    print(json.dumps({k: v for k, v in out.items() if k != "per_kf"}, indent=1))


if __name__ == "__main__":
    main()
