#!/usr/bin/env python3
"""Table of scanA step 3 runs (summary.json in each run dir) -> step3/table.md + table.json."""
import json, sys
from pathlib import Path
D = Path(sys.argv[1])
rows = []
order = ["gt", "vio_all", "vio_kf", "ba", "ba_rs", "ba_mask_rs", "vio_all_fs", "vio_kf_fs", "ba_fs", "ba_rs_fs", "ba_mask_rs_fs"]
for n in order:
    p = D / n / "summary.json"
    if not p.exists():
        continue
    s = json.loads(p.read_text()); po = s.get("poses", {})
    k = s.get("kf_board_dZ_med_cm_stats", {}); ka = s.get("kf_board_absdZ_p50_cm_stats", {})
    pd, pa = s.get("pooled_dZ_cm", {}), s.get("pooled_absdZ_cm", {})
    rows.append(dict(name=n, scale=po.get("sim3_scale_would_be"), free_scale=po.get("free_scale_applied", False),
                     ate_cm=po.get("se3_rmse_cm"), n_kf=s.get("n_kf"), n_eval=s.get("n_kf_board_eval"), n_nopair=s.get("n_kf_no_pair"),
                     kf_bias=k, kf_abs=ka, pooled=pd, pooled_abs=pa, frac_bias_gt1=s.get("kf_frac_abs_bias_gt_1cm"),
                     mesh_z=s.get("mesh_board_z_med_cm"), mesh_rms=s.get("mesh_board_z_rms_cm"), mesh_p5p95=s.get("mesh_board_z_p5_p95_cm"),
                     cover=s.get("mesh_board_cover_frac"), z_unique=s.get("mesh_board_z_unique_mm"), verts=s.get("mesh_vertices"),
                     runtime=s.get("runtime_s")))
f = lambda x, d=2: "—" if x is None else f"{x:.{d}f}"
L = ["| 자세 | 축척 (Sim3) | ATE cm | kf 평가/짝없음 | kf 치우침 중앙값: 평균 / 중앙 / p10 / p90 cm | kf |dZ| 중앙값: 평균 / 중앙 / p90 | 전체 화소 dZ 평균 / 중앙 / p10 / p90 cm | 전체 |dZ| 평균 / p90 | |치우침|>1cm kf | 메시 z 중앙 / RMS cm | 덮임 |",
     "|---|---|---|---|---|---|---|---|---|---|---|"]
for r in rows:
    k, ka, p, pa = r["kf_bias"], r["kf_abs"], r["pooled"], r["pooled_abs"]
    L.append(f"| {r['name']} | {f(r['scale'], 4)}{' (적용)' if r['free_scale'] else ''} | {f(r['ate_cm'])} | {r['n_eval']}/{r['n_nopair']} | "
             f"{f(k.get('mean'))} / {f(k.get('median'))} / {f(k.get('p10'))} / {f(k.get('p90'))} | {f(ka.get('mean'))} / {f(ka.get('median'))} / {f(ka.get('p90'))} | "
             f"{f(p.get('mean'))} / {f(p.get('median'))} / {f(p.get('p10'))} / {f(p.get('p90'))} | {f(pa.get('mean'))} / {f(pa.get('p90'))} | "
             f"{f(r['frac_bias_gt1'])} | {f(r['mesh_z'])} / {f(r['mesh_rms'])} | {f(r['cover'])} |")
(D / "table.md").write_text("\n".join(L) + "\n")
(D / "table.json").write_text(json.dumps(rows, indent=1))
print("\n".join(L))
