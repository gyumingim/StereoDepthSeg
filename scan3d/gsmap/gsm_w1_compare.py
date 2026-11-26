#!/usr/bin/env python3
"""gsmap 2026-11-26 W1: comparison table np / nC / ffsonly2 / W1 from eval2 JSONs + aisle ROI support (gsm_relax_aisle) + FFS measure.
  python gsm_w1_compare.py   -> runs/gsmap_20261125_121801_v2/w1/compare.json  (CPU, light)
Ruler = the cams.npz the eval2 run used (GSMAP_RUN): 'bap' = ffsonly_ba/evalrun_newcams (ba_partners poses), 'ba2' = ffsonly_ba2/evalrun_newcams2,
'ba3' = w1/evalrun_ba3. The eval2 references (eval2/ref) are the same files in all three; only the judge camera poses differ."""
import json, time
from pathlib import Path

V2 = Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2"
O = V2 / "w1"


def row(p):
    d = json.loads(Path(p).read_text()); P2 = d["P2_inside"]; R = d["S2_S3_rois"]
    rois = {k: [v["rms_cm"], v["hit"]] for k, v in R.items()}
    return dict(P2a=P2["P2a_backface_pct"], P2b=P2["P2b_small_component_front_pct"], P2c=P2["P2c_empty_pct"], P3=d["P3_correct_pct"],
                S1_front=d["S1"]["front_pct"], S1_hole=d["S1"]["hole_pct"], roi_rms_cm=round(sum(v[0] for v in rois.values()) / len(rois), 2),
                roi_hit=round(sum(v[1] for v in rois.values()) / len(rois), 3), S4_floor=d["S4"]["floor_coverage_pct"], S4_ceil=d["S4"]["ceiling_coverage_pct"],
                S8_count=d["S8_islands"]["count"], S8_area=d["S8_islands"]["area_pct"], S9_N3=d["S9_floater_area_pct"]["N3"], S9_N5=d["S9_floater_area_pct"]["N5"],
                area_m2=d["S7"]["area_m2"], rois=rois, file=str(p), made=d.get("made"))


def main():
    ev = {("ffsonly_np", "bap"): V2 / "ffsonly_ba/newpose/eval_newcams_ffsonly_np.json",
          ("nC", "bap"): V2 / "ffsonly_ba/support_test/nC/eval_newcams_ffssupport_NC.json",
          ("ffsonly2", "bap"): V2 / "ffsonly_ba2/b/eval_newcams_ffsonly2.json",
          ("ffsonly2", "ba2"): V2 / "ffsonly_ba2/b/eval_newcams2_ffsonly2.json",
          ("w1", "bap"): O / "b/eval_newcams_w1.json",
          ("w1", "ba3"): O / "b/eval_ba3_w1.json"}
    rows = {f"{m}@{r}": row(p) for (m, r), p in ev.items() if p.exists()}
    ais = json.loads((O / "aisle_roi.json").read_text())["rows"] if (O / "aisle_roi.json").exists() else {}
    aisle = {k: dict(roi_supported_ge2_pct=v["roi_supported_ge2_pct"], roi_unsupported_area_m2=v["roi_unsupported_area_m2"], roi_wrong_pct=v["roi_wrong_pct"],
                     roi_never_pct=v["roi_never_pct"], roi_area_m2=v["roi_area_m2"], whole_supported_ge2_pct=v["whole_supported_ge2_pct"]) for k, v in ais.items()}
    meas = {}
    for nm, p, k in (("s4m (nC/np depth) judge bap", V2 / "ffs_relax/measure_relax.json", "s4m_old_judge_bap"),
                     ("s4m_r (ffsonly2 depth, before gate) judge ba2", V2 / "ffs_relax/measure_relax.json", "s4m_r_judge_ba2")):
        e = json.loads(p.read_text())["sets"][k]; meas[nm] = dict(all=e["all"]["fail_pct"], aisle=e["aisle_314_331"]["fail_pct"],
                                                                   new=e.get("new_pixels_only", {}).get("fail_pct"), usable_all=e["usable_pct"]["all"],
                                                                   usable_aisle=e["usable_pct"]["aisle_314_331"], file=str(p))
    for p in sorted(O.glob("measure_w1*.json")):
        for k, e in json.loads(p.read_text())["sets"].items():
            meas[f"W1 {k} ({p.name}) judge ba3"] = dict(all=e["all"]["fail_pct"], aisle=e["aisle_314_331"]["fail_pct"], new=e["new_pixels_only"]["fail_pct"],
                                                         new_aisle=e["new_pixels_aisle"]["fail_pct"], usable_all=e["usable_pct"]["all"],
                                                         usable_aisle=e["usable_pct"]["aisle_314_331"], file=str(p))
    res = dict(made=time.strftime("%F %T"), inputs={f"{m}@{r}": str(p) for (m, r), p in ev.items()} | dict(aisle=str(O / "aisle_roi.json")),
               rulers=dict(bap=str(V2 / "ffsonly_ba/evalrun_newcams/dump_ba/cams.npz"), ba2=str(V2 / "ffsonly_ba2/evalrun_newcams2/dump_ba/cams.npz"),
                           ba3=str(O / "evalrun_ba3/dump_ba/cams.npz")),
               note="ROI RMS/hit = mean over the 6 S2_S3 rois; name@ruler; eval2 references identical, judge poses differ (bap/ba2/ba3) -> "
                    "cross-ruler numbers are NOT like for like (ruler unification = task R). Aisle ROI support: each mesh voted with its OWN fused FFS + "
                    "its own cams (np/nC: ffsonly_ba fused + bap; ffsonly2: ffsonly_ba2 fused + ba2; w1: w1 fused + ba3), pre-prune carve mesh. n = 1 recording.",
               rows=rows, aisle_roi=aisle, ffs_measure=meas)
    (O / "compare.json").write_text(json.dumps(res, indent=1)); print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
