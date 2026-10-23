"""results/calib_all.json → config/lens/std_1920x1080_f0.50_cal20261023.json (status not_repeatable, provisional).
Not added to lens_calib.DEFAULT_LENS."""
import json, time
from pathlib import Path
HERE = Path(__file__).resolve().parent
S3D = HERE.parents[1]
R = json.load(open(HERE / "results" / "calib_all.json"))
X = json.load(open(HERE / "results" / "extra.json"))
M = R["models"]
main = "k1_prior"
cand = {}
for m, v in M.items():
    cand[m] = dict(K=v["K"], dist=v["dist"], rms_px=round(v["rms_px"], 4), n_views=v["n_views"],
                   holdout_by_pose_px=[round(v["holdout_px"]["even_fit_odd_test"][0], 4), round(v["holdout_px"]["odd_fit_even_test"][0], 4)],
                   lopo_test_px=round(v["lopo_test_median"], 4), lopo_fx_range=v["lopo_fx_range"], lopo_cx_range=v["lopo_cx_range"],
                   lopo_cy_range=v["lopo_cy_range"],
                   halves_stratified_abs_p50={k: round(x, 3) for k, x in v["strat_abs_p50"].items()},
                   halves_stratified_abs_max={k: round(x, 3) for k, x in v["strat_abs_max"].items()})
v = M[main]
out = dict(K=v["K"], dist=v["dist"], focus_dpt=0.5, model="k1k2p_k3", fit=main,
           fit_note="fx fy cx cy k1 fitted; k2 p1 p2 k3 fixed to config/lens/hs_1920x1080_f2.22_cal20261022.json (round-2 joint-fit structure: distortion shared over focus, k1 per focus)",
           rms_px=round(v["rms_px"], 4), n_views=v["n_views"], n_poses=24, n_pose_clusters=len(R["clusters"]),
           status="not_repeatable",
           status_note="PROVISIONAL. Criteria fx +-0.3 % and principal point +-3 px between disjoint pose halves NOT met by any model "
                       "(stratified halves median |dfx| 0.31-0.95 %, principal point 2.7-12 px; full fit fx changes +0.35-0.45 % when the single 65 deg pose 210928 is dropped). "
                       "All tilted poses are in the image centre, all edge poses are frontal. See docs/lens_calib_f050_20261023.md.",
           uncertainty="fx about +-0.5 % (pose-half scatter), principal point about +-3-5 px (k1_prior) / +-10 px (free distortion models); estimate",
           expected_fx=dict(focus_law_hs=1285.95, focus_law_std60=1280.94, note="2.22 node 1303.89 x (1 + 0.008 (0.5 - 2.22)); std60 x (1 - 0.0039) from docs/fov_60_vs_240_20261022.md"),
           candidates=cand, model_choice="holdout-by-pose best = rational (0.105 px) but its coefficients are wild (k3 17.7) and its principal point/fx are not repeatable; "
                       "k1_prior chosen: holdout 0.123 px, LOPO 0.113 px, only model whose principal point stays near the 2.22 node (966.9/553.7 vs 968.4/552.5)",
           fixed_lens_scores_px=X["fixed_lens_scores"],
           recs=sorted(sum(R["members"].values(), [])), size=[1920, 1080], session="std (regular, 60 fps)", exposure="2 ms, ISO 500",
           board="board_laptop_6x4.json (201830 203443 204636 204819, legacy pattern OFF) + board_laptop_5x3.json (rest); square_mm from json (60.3 / 47.1, spec estimate; irrelevant for K)",
           source="scan3d/runs/lens_calib_f050_20261023 (collect.py, steady.py, calib.py, extra.py, write_lens.py)",
           note="Not in lens_calib.DEFAULT_LENS (do not use as default until re-shoot passes the repeatability check).",
           made=time.strftime("%Y-%m-%d %H:%M:%S"))
p = S3D / "config" / "lens" / "std_1920x1080_f0.50_cal20261023.json"
assert not p.exists() or json.load(open(p)).get("source", "").startswith("scan3d/runs/lens_calib_f050_20261023")
json.dump(out, open(p, "w"), indent=1, ensure_ascii=False)
print("->", p)
