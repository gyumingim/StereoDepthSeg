#!/usr/bin/env python3
"""fixes 11-13 unit 15: one table from far_eval.py json files + scan_offline summaries. Usage: u15_table.py DIR [PAIR ...]
PAIR = OTHER_vs_BASE (file DIR/u15_<PAIR>_eval.json). Prints markdown rows; writes DIR/u15_table.json."""
import json
import sys
import time
from pathlib import Path

d = Path(sys.argv[1])
out = {}
for pair in sys.argv[2:]:
    r = json.loads((d / f"u15_{pair}_eval.json").read_text())["results"]
    b, f, n = r["base"], r["far"], r["near_lt_1p2m"]
    fp = f.get("far_pairs") or {}

    def fl16(x):
        return None if x.get("gt_1p6_floater_frac_lt5000") is None else round(x["gt_1p6_floater_frac_lt5000"] * x["v_gt_1p6"])
    row = dict(pair=pair, base_v=b["vertices"], v=f["vertices"], v_gt_1p6=f["v_gt_1p6"], v_gt_2p0=f["v_gt_2p0"], v_gt_2p5=f["v_gt_2p5"],
               base_v_gt_1p6=b["v_gt_1p6"], area=f["top_area_m2"], base_area=b["top_area_m2"], area_gt_1p6=f["top_area_gt_1p6_m2"],
               area_new=f.get("top_area_new_vs_base_m2"), cam_max=round(f["cam_depth_m"]["max"], 2),
               fl_v5000=f["floater_v_lt5000"], fl_c5000=f["components_lt5000"], fl_v500=f["floater_v_lt500"], fl_c500=f["components_lt500"],
               base_fl_v5000=b["floater_v_lt5000"], base_fl_c5000=b["components_lt5000"], base_fl_v500=b["floater_v_lt500"], base_fl_c500=b["components_lt500"],
               fl_gt_1p6_v5000=fl16(f), base_fl_gt_1p6_v5000=fl16(b),
               near_b2o_med_mm=round(n["base_to_far_median_mm"], 3), near_b2o_p90_mm=round(n["base_to_far_p90_mm"], 2),
               near_o2b_p90_mm=round(n["far_to_base_p90_mm"], 2), near_o2b_gt2cm=round(n["far_to_base_frac_gt_2cm"] * 100, 2),
               near_covered_2cm=round(n["base_covered_frac_2cm"] * 100, 2),
               band_b2o_p90_mm=round((r.get("band_1p2_1p6m") or {}).get("base_to_far_p90_mm", float("nan")), 2),
               runtime_s=f["runtime_s"], backend=f["ffs_backend"],
               far={k: fp.get(k) for k in ("kf_with_partner", "px_added", "px_replaced", "px_wide_only_cand", "px_wide_only_rejected",
                                            "px_rej_lr", "px_rej_sat", "px_rej_p2", "px_repl_below_min_z", "kf_partner2", "kf_no_partner2",
                                            "ffs_s", "ffs2_s")} if fp else None)
    out[pair] = row
    print(json.dumps(row))
(d / "u15_table.json").write_text(json.dumps(dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), dir=str(d), rows=out), indent=1))
