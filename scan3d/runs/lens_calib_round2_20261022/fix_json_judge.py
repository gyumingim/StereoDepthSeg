"""Apply runs/lens_calib_round2_review/judge.md "고칠 것" 1-2 to config/lens/hs_1920x1080_focus_cal_20261022.json
(labels and usage only; no number changes). Run once: python fix_json_judge.py"""
import json, time
from pathlib import Path

S3 = Path.home() / "scanner/StereoDepthSeg/scan3d"
p = S3 / "config/lens/hs_1920x1080_focus_cal_20261022.json"
j = json.load(open(p))
j["status"] = "not_repeatable"
j["status_reason"] = (
    "targets fx <0.3 %, principal point <2 px between same-focus recordings. Met only at 2.22 dpt and only in the JOINT fit "
    "(k2 p1 p2 k3 shared over 9 recordings); per-recording fits (own distortion) fail on the principal point at every focus. "
    "3.33 weak (one pair, 0.14-0.43 % depending on detection settings), 0.37 and 1.00 not decidable with this data. "
    "No single recording reached the pre-registered 85 % 32x18 coverage (max 0.84 usable / 0.79 selected). "
    "Judge: runs/lens_calib_round2_review/judge.md.")
j["usage"] = {
    "focus_2.22": "use the 2.22 node K + distortion (absolute). As a lens file: config/lens/hs_1920x1080_f2.22_cal20261022.json "
                  "(VIO default only after the A/B in docs/lens_ab_plan.md / docs/lens_ab_result.md).",
    "other_focus": "2.22 node with fx, fy scaled by (1 + 0.008 (D - 2.22)) (linear k 0.8 %/dpt, relative to 2.22); cx, cy and "
                   "distortion = the 2.22 node. Do NOT use the 0.37 / 1.00 / 3.33 node values (mixing node rules jumps corner "
                   "projection by several px when focus changes, critic 10).",
    "uncertainty": "fx +-0.2 %, principal point +-2 px (between-recording scatter, estimate). The ci95 fields are within-recording "
                   "1 s block bootstrap and are 2-3x too narrow.",
    "scale_law": "joint-fit slope is model dependent (0.98 %/dpt with k1 per focus, 1.27 with k1 shared; neither CI contains 0.80); "
                 "focus and board distance change together and 0.37 / 1.00 are not decidable, so this is no reason to change the "
                 "same-pose sweep law: keep k = 0.8 %/dpt.",
}
reasons = {
    0.37: ("not_decidable", "both recordings have the board at 1.04-1.11 m (PnP) while focus 0.37 dpt = 2.7 m (blur, estimate); the "
           "pair difference depends on the model: 0.05 % (per-recording k5), 0.22 % (rational), 0.83 % (joint), 1.55 % (k1 shared); "
           "per-recording cy differs 8-9 px. Cannot tell lens repeatability from data quality."),
    1.0: ("not_decidable", "second recording 172351 stopped early by overheating: 60 views, 28 % coverage, 3 tilt directions; "
          "temperature is mixed in (estimate). Joint fit 0.29 % / 2.1 / 3.9 px, per-recording fit 0.32 %."),
    3.33: ("weak", "one pair only (173113 unusable). Joint fit 0.21 %, but an independent re-fit gives 0.14 % or 0.43 % / 2.2 px "
           "depending on detection settings; 173834's principal point comes from >60 deg views (dropping them: cy +11.3 px), "
           "which also have the largest residuals; per-recording fit cx differs 3.1 px."),
}
for f in j["focus"]:
    d = round(f["focus_dpt"], 2)
    if d == 2.22:
        f["repeatable"] = True
        f["repeatable_basis"] = ("joint fit (shared k2 p1 p2 k3); per-recording fit fails on principal point "
                                 "(171455 vs others 2.4-5 px). Holds without the recovered recording 171455 (round-1 pair alone).")
        f["use"] = "yes (absolute K + distortion for focus 2.22; base for other focus via linear k)"
    else:
        f["repeatable"], f["repeatable_reason"] = reasons[d]
        f["use"] = "no (use the 2.22 node + linear k 0.8 %/dpt)"
j["judge_fix"] = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), script="runs/lens_calib_round2_20261022/fix_json_judge.py",
                      judge="runs/lens_calib_round2_review/judge.md", numbers_changed=False)
json.dump(j, open(p, "w"), indent=1)
print("ok", [(f["focus_dpt"], f["repeatable"]) for f in j["focus"]])
