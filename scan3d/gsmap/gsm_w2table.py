#!/usr/bin/env python3
"""gsmap 2026-11-26 W2: guard table from eval2 jsons + an R-ruler geo json.
  python gsm_w2table.py OUT.json GEO.json name=EVAL2.json[:W2JSON] [...]   (first entry = baseline)
Guards (vs baseline): cabinet_front_kf774 and panel_kf813 ROI hit drop <= 1 pt; gold front % not higher; gold correct % drop <= 0.3 pt;
P2c rise <= 1 pt."""
import json, sys, time
from pathlib import Path


def main():
    out, geo = sys.argv[1], json.loads(Path(sys.argv[2]).read_text())
    rows = {}
    for s in sys.argv[3:]:
        nm, rest = s.split("=", 1); ps = rest.split(":"); e = json.loads(Path(ps[0]).read_text())
        g = geo["table"]["gold"].get(nm, {}); cs = geo["table"].get("fused_consensus", {}).get(nm, {})
        r = dict(eval2=ps[0], area_m2=e["S7"]["area_m2"], P3=e["P3_correct_pct"], S1_front=e["S1"]["front_pct"], S1_hole=e["S1"]["hole_pct"],
                 P2b=e["P2_inside"]["P2b_small_component_front_pct"], P2c=e["P2_inside"]["P2c_empty_pct"], S9_N3=e["S9_floater_area_pct"]["N3"],
                 S8=e["S8_islands"]["area_pct"], cab_hit=e["S2_S3_rois"]["cabinet_front_kf774"]["hit"] * 100, panel_hit=e["S2_S3_rois"]["panel_kf813"]["hit"] * 100,
                 roi_rms_mean6=round(sum(v["rms_cm"] for v in e["S2_S3_rois"].values()) / 6, 2), floor=e["S4"]["floor_coverage_pct"],
                 gold_c3=g.get("correct3_pct"), gold_front=g.get("front_pct"), gold_hole=g.get("hole_pct"), gold_c3s=g.get("correct3_scalefit_pct"),
                 cons_c3=cs.get("correct3_pct"), cons_front=cs.get("front_pct"))
        if len(ps) > 1:
            w = json.loads(Path(ps[1]).read_text()); r["removed_m2"] = {k: v for k, v in w["removed_m2"].items()}; r["removed_pct"] = w["removed_pct"]
        rows[nm] = r
    b = rows[next(iter(rows))]
    for nm, r in rows.items():
        g = dict(cab_hit=r["cab_hit"] >= b["cab_hit"] - 1, panel_hit=r["panel_hit"] >= b["panel_hit"] - 1, P2c=r["P2c"] <= b["P2c"] + 1)
        if r["gold_c3"] is not None and b["gold_c3"] is not None:
            g.update(gold_front=r["gold_front"] <= b["gold_front"] + 1e-9, gold_c3=r["gold_c3"] >= b["gold_c3"] - 0.3)
        r["guards"] = g; r["all_guards_ok"] = all(g.values())
        dg = geo.get("diffs", {}).get("gold", {}).get(f"{nm}-{next(iter(rows))}")
        if dg:
            r["gold_paired_diff_vs_base"] = {k: [v["diff_pp"], v["se"]] for k, v in dg.items()}
    res = dict(made=time.strftime("%F %T"), geo=sys.argv[2], baseline=next(iter(rows)), rows=rows)
    Path(out).write_text(json.dumps(res, indent=1))
    keys = ["area_m2", "P3", "S1_front", "P2b", "P2c", "S9_N3", "cab_hit", "panel_hit", "roi_rms_mean6", "gold_c3", "gold_front", "gold_hole", "cons_c3", "all_guards_ok"]
    print("name".ljust(10) + "".join(k[:9].rjust(10) for k in keys))
    for nm, r in rows.items():
        print(nm.ljust(10) + "".join(str(r[k]).rjust(10) for k in keys), r.get("removed_m2", ""))


if __name__ == "__main__":
    main()
