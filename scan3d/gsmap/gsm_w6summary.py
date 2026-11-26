#!/usr/bin/env python3
"""gsmap W6 2026-11-27: collect W6 numbers (w6.json, table.json, rjudge geo/photo, fruler/w6_compare.json) into w6/summary.json"""
import json, time
from pathlib import Path

V2 = Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2"; O = V2 / "w6"
out = O / "summary.json"; assert not out.exists(), "never overwrite"
tab = json.loads((O / "table.json").read_text()); fr = json.loads((O / "fruler/w6_compare.json").read_text())
photo = json.loads((O / "rjudge/photo.json").read_text()) if (O / "rjudge/photo.json").exists() else {}
rows = {}
for nm in ("ffsonly2", "C1", "C2", "C3"):
    t = tab["rows"][nm]; f = fr["rows"][f"{nm}_e3c" if nm != "ffsonly2" else "ffsonly2_e3c"]
    r = dict(cab_hit=t["cab_hit"], panel_hit=t["panel_hit"], P3=t["P3"], P2c=t["P2c"], P2b=t["P2b"], S1_front=t["S1_front"], area_m2_eval2=t["area_m2"],
             gold_c3=t["gold_c3"], gold_front=t["gold_front"], gold_hole=t["gold_hole"], gold_c3s=t["gold_c3s"],
             F3_red_px_9views=f["F3_red_px_9views"], F3_audit_red_px=f["F3_audit_red_px"], F3_audit_red_pct=f["F3_audit_red_pct"], F3_audit_hit_px=f["F3_audit_hit_px"],
             F3_tex_red_px_audit=f["F3_tex_red_px_audit"], unsupported_lt2_m2=f["unsupported_lt2_m2"], unsupported_lt2_roi_m2=f["unsupported_lt2_roi_m2"],
             e3c_area_m2=f["area_m2"], e3c_area_roi_m2=f["area_roi_m2"], F3_sheet=f["sheet"])
    if nm != "ffsonly2":
        w = json.loads((O / nm.lower() / "w6.json").read_text())
        r.update(removed_vs_ffsonly2_m2=w["removed_vs_ffsonly2_m2"], removed_vs_ffsonly2_pct=w["removed_vs_ffsonly2_pct"], removed_by_type_m2=w["removed_m2"],
                 removed_with_ffs_agree_ge1_m2=w["removed_with_ffs_agree_ge1_m2"], gold_paired_diff_vs_base=t.get("gold_paired_diff_vs_base"))
    rows[nm] = r
res = dict(made=time.strftime("%F %T"), worker="W6", n_recordings=1, base="ffsonly2 (ffsonly_ba2/b)", poses="ba2", depth="ffsonly_ba2/fused",
           inputs=dict(table=str(O / "table.json"), rjudge_geo=str(O / "rjudge/geo.json"), rjudge_photo=str(O / "rjudge/photo.json"), fruler=str(O / "fruler/w6_compare.json"),
                       vis=str(O / "vis.json"), picture=str(O / "w6_clean_121801.jpg")),
           photo_summary={k: v for k, v in photo.items() if k in ("table", "summary")}, rows=rows,
           caveats=["P2c not gated (rewards fake sheets)", "C1 uses vis without grazing cut (conservative)", "n=1 recording, no repeat"])
out.write_text(json.dumps(res, indent=1)); print(json.dumps(rows, indent=1))
