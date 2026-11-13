#!/usr/bin/env python3
"""u16 (11-13): one table row per u16 full test (reads the pulled merge_summary.json / ba.json / ate json; no compute)."""
import json, sys
from pathlib import Path

L = Path(__file__).resolve().parents[1] / "laptop" / "20261112_121801_std60"
rows = []
for tag in sys.argv[1:] or ["fix_u16_u11", "fix_u16_shift", "fix_u16_u11b", "fix_u16_x105", "fix_u16_u10", "fix_u16_neg", "fix_u16_x095",
                            "fix_u16_shift1m"]:
    f = L / tag / "merge" / "merge_summary.json"
    if not f.exists():
        rows.append(dict(tag=tag, missing=True)); continue
    s = json.loads(f.read_text())
    p = ((s.get("merge") or {}).get("parts") or [{}])[0]
    ml = s.get("merge_link") or {}
    sc = s.get("scan") or {}
    r = dict(tag=tag, status=s["status"][:110], exit=s.get("exit_code"),
             pnp=p.get("n_pnp_ok"), agree=p.get("n_consensus"), frac=p.get("consensus_frac"), centre_cm=p.get("centre_err_cm_median"),
             tilt=p.get("tilt_deg_median"), pts_scale=p.get("pts_scale_ratio"), centre_scale=p.get("sim3_scale_ratio"),
             scale_applied=p.get("scale_applied"), cells=p.get("grid_cells_union"), extent=p.get("inlier_extent_m"),
             span=p.get("consensus_span_s"), second=p.get("second_n"),
             link_init=(ml.get("init") or {}).get("linked_points"), link_retri1=(ml.get("retri1") or {}).get("linked_points"),
             link_retri2=(ml.get("retri2") or {}).get("linked_points"), gate=ml.get("gate_value"),
             move=[(m.get("part"), round(m.get("rigid_t_cm", 0), 1), round(m.get("sim3_scale", 0), 4)) for m in ml.get("part_move", [])],
             ba=(sc.get("ba_quality") or {}), mesh_v=((sc.get("mesh") or {}).get("mesh_vertices")), steps={k: v["s"] for k, v in s["steps"].items()})
    a = L / tag / "ate_vs_fix_u8.json"
    if a.exists():
        aj = json.loads(a.read_text())
        r["ate_se3_rms_cm"] = {k: round(v["se3"]["rms_cm"], 2) for k, v in aj.items()}
    rows.append(r)
print(json.dumps(rows, indent=1))
