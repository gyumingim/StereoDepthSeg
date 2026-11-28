"""noise_floor_20261128: seed 0 vs seed 1 of the ffsonly2 chain, every ruler -> noise_floor.json (read-only on inputs)."""
import json, time, sys
from pathlib import Path
import numpy as np
R = Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/runs/noise_floor_20261128"
V2 = Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2"
S0, S1 = V2 / "ffsonly_ba2/b", R / "seed1"
out = dict(made=time.strftime("%F %T"), n_recordings=1, n_seeds=2, seeds=[0, 1],
           what="ffsonly2 2DGS retrained with only --seed 1 (seed 0 = the original ffsonly_ba2/b/gs); same mesh chain, texture recipe (w9 f2) and rulers. "
                "With 2 seeds the spread is |seed1 - seed0| (one sample, not an SD).",
           inputs=dict(seed0_gs=str(S0 / "gs"), seed1_gs=str(S1 / "gs"), chain=str(R / "scripts/run_nf.sh"),
                       photo=str(R / "photo/photo.json"), plane=[str(R / "plane_s0/plane_diag.json"), str(R / "plane_s1/plane_diag.json")],
                       eval2=[str(R / f"eval_newcams2_s{s}.json") for s in (0, 1)]),
           reference_ffsonly2_vs_nC="report_20261127 / audit10: ffsonly2_pad - nC_tex3 partner psnr_common +0.109 +- 0.031 dB")
def pair(a, b):
    return dict(seed0=a, seed1=b, diff=None if a is None or b is None else round(b - a, 4))
# 2DGS summary
g = [json.load(open(p / "gs/summary.json")) for p in (S0, S1)]
out["gs"] = {k: pair(g[0].get(k), g[1].get(k)) for k in ("n_gauss", "train_psnr", "n_dark_opaque", "n_aniso5_opaque", "valid_px")}
out["gs"]["floater_frac"] = pair(g[0]["floaters"]["frac_of_seen"], g[1]["floaters"]["frac_of_seen"])
# mesh chain sizes
def faces(p):
    try: return json.load(open(p)).get("faces_out") or json.load(open(p)).get("faces")
    except Exception: return None
# photo (perframe)
z = np.load(R / "photo/photo_perframe.npz", allow_pickle=True); Rr, names, cols, kind = z["R"], list(z["maps"]), list(z["cols"]), z["kind"]
ph = {}
for k in ("partner", "kf"):
    sel = kind == k; ph[k] = {"n_frames": int(sel.sum()), "table": {}, "diffs": {}}
    for mi, m in enumerate(names):
        ph[k]["table"][m] = {c: round(float(np.nanmean(Rr[sel, mi, ci])), 4) for ci, c in enumerate(cols)}
    for a_, b_ in (("s1_pad", "ffsonly2_pad"), ("s1_tex", "ffsonly2_tex"), ("ffsonly2_pad", "nC_tex3"), ("s1_pad", "nC_tex3")):
        d = {}
        for ci, c in enumerate(cols[:5]):
            x = Rr[sel, names.index(a_), ci] - Rr[sel, names.index(b_), ci]; x = x[np.isfinite(x)]
            se = float(x.std(ddof=1) / np.sqrt(len(x)))
            d[c] = dict(diff=round(float(x.mean()), 4), se=round(se, 4), abs_frame_diff_median=round(float(np.median(np.abs(x))), 4), n=int(len(x)))
        ph[k]["diffs"][f"{a_}-{b_}"] = d
out["photo"] = ph
# plane_diag
P = [json.load(open(R / f"plane_s{s}/plane_diag.json")) for s in (0, 1)]
pl = {}
for roi in P[0]["rois"]:
    pl[roi] = {}
    for st in ("d", "e", "f", "g"):
        a, b = P[0]["rois"][roi]["stages"][st]["summary"], P[1]["rois"][roi]["stages"][st]["summary"]
        pl[roi][st] = {k: pair(round(a[k], 4), round(b[k], 4)) for k in ("offset_sd_cm", "offset_robust_sd_cm", "within_rms_own_cm_median", "combined_rms_global_cm", "tilt_median_deg")}
out["plane_diag"] = pl
# eval2 (two camera sets)
ev = {}
for nm in ("eval_newcams2", "eval_newcams"):
    E = [json.load(open(R / f"{nm}_s{s}.json")) for s in (0, 1)]
    f = lambda e: {"P3_correct_pct": e["P3_correct_pct"], "S1_front_pct": e["S1"]["front_pct"], "S1_hole_pct": e["S1"]["hole_pct"],
                   "S9_floater_N3": e["S9_floater_area_pct"]["N3"], "S7_area_m2": e["S7"]["area_m2"], "S7_faces": e["S7"]["faces"],
                   "S4_floor_cov": e["S4"]["floor_coverage_pct"], "S4_ceiling_cov": e["S4"]["ceiling_coverage_pct"],
                   "P2a": e["P2_inside"]["P2a_backface_pct"], "P2b": e["P2_inside"]["P2b_small_component_front_pct"], "P2c": e["P2_inside"]["P2c_empty_pct"],
                   **{f"roi_rms_{k}": v["rms_cm"] for k, v in e["S2_S3_rois"].items()}}
    a, b = f(E[0]), f(E[1]); ev[nm] = {k: pair(a[k], b.get(k)) for k in a}
out["eval2"] = ev
(R / "noise_floor.json").write_text(json.dumps(out, indent=1))
print(json.dumps({"gs": out["gs"], "photo_partner": {k: {c: v[c]["diff"] for c in v} for k, v in ph["partner"]["diffs"].items()},
                  "plane_e_offset_sd": {r: pl[r]["e"]["offset_sd_cm"] for r in pl}, "eval2": ev["eval_newcams2"]}, indent=1))
