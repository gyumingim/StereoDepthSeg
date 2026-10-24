"""(2026-10-23 night) results/calib_all.json → config/lens/std_1920x1080_f0.50_cal20261023.json (status not_repeatable, provisional).
Not added to lens_calib.DEFAULT_LENS."""
import json, time
from pathlib import Path
HERE = Path(__file__).resolve().parent
S3D = HERE.parents[1]
def write_1023():
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


def write_1024(tags=("all1024", "notan1024", "k12notan1024"), out_name="std_1920x1080_f0.50_cal20261024.json",
               n_poses=53, status=None, status_note=None, extra=None):
    """2026-10-24: night + morning retakes (53 poses). Model k12_notan (k1 k2 free, p1 p2 k3 = 0)."""
    import numpy as np
    res = {}
    for tag in tags:
        res.update(json.load(open(HERE / "results" / f"calib_{tag}.json"))["models"])
    R0 = json.load(open(HERE / "results" / f"calib_{tags[0]}.json"))

    def passfrac(M):
        H = M["halves"]
        f = lambda h: abs(h["dfx_pct"]) <= 0.3
        q = lambda h: abs(h["dcx"]) <= 3 and abs(h["dcy"]) <= 3
        return dict(fx=round(float(np.mean([f(h) for h in H])), 2), pp=round(float(np.mean([q(h) for h in H])), 2),
                    both=round(float(np.mean([f(h) and q(h) for h in H])), 2), n_splits=len(H))
    cand = {}
    for m, v in res.items():
        cand[m] = dict(K=v["K"], dist=v["dist"], rms_px=round(v["rms_px"], 4), n_views=v["n_views"],
                       holdout_by_pose_px=[round(v["holdout_px"]["even_fit_odd_test"][0], 4), round(v["holdout_px"]["odd_fit_even_test"][0], 4)],
                       lopo_test_px=round(v["lopo_test_median"], 4), lopo_fx_range=v["lopo_fx_range"], lopo_cx_range=v["lopo_cx_range"],
                       lopo_cy_range=v["lopo_cy_range"], halves_abs_p50={k: round(x, 3) for k, x in v["halves_abs_p50"].items()},
                       halves_abs_max={k: round(x, 3) for k, x in v["halves_abs_max"].items()}, halves_pass_frac=passfrac(v),
                       even_odd={k: round(v["halves"][0][k], 3) for k in ("dfx_pct", "dcx", "dcy")})
    main = "k12_notan"
    v = res[main]
    d = list(v["dist"]) + [0.0] * (5 - len(v["dist"]))
    out = dict(K=v["K"], dist=d[:5], focus_dpt=0.5, model="k1k2p", fit=main,
               fit_note="k1 k2 free, p1 p2 k3 fixed 0 (OpenVINS radtan compatible). Free tangential terms trade with cx (pp scatter 8-17 px between halves); without them pp repeats to ~1-2 px.",
               rms_px=round(v["rms_px"], 4), n_views=v["n_views"], n_poses=n_poses, n_pose_clusters=len(R0["clusters"]),
               status="repeatable_lopo_halves_borderline",
               status_note="Leave-one-pose-out: fx within -0.13/+0.07 %, pp within 1.1 px -> pass. Disjoint pose halves (62 splits): median |dfx| 0.20 %, |dcx| 1.2, |dcy| 2.3 px; "
                           "fx <= 0.3 % in 68 % of splits, pp <= 3 px in 56 %, both in 35 %; even/odd split passes (-0.25 %, 0.5, 0.1 px). See docs/lens_calib_f050_20261023.md section 9.",
               uncertainty="fx about +-0.2 % (half-split scatter / 2), principal point about +-2 px; estimate",
               expected_fx=dict(focus_law_hs=1285.95, focus_law_std60=1280.94),
               candidates=cand,
               model_choice="rational has the lowest pose holdout but its pp is not repeatable (halves |dcx| median 9 px); among pp-repeatable models (k12_notan, k123_notan, k1_prior) k12_notan has the lowest holdout/LOPO",
               recs=sorted(sum(R0["members"].values(), [])), excluded=["hs_20261024_094619_std60", "hs_20261024_094655_std60 (no board)"],
               size=[1920, 1080], session="std (regular, 60 fps)", exposure="2 ms, ISO 500",
               board="board_laptop_6x4.json (4 night recordings) + board_laptop_5x3.json; legacy pattern off",
               source="scan3d/runs/lens_calib_f050_20261023 (collect.py, steady.py, calib.py --tag all1024/notan1024/k12notan1024, write_lens.py --1024)",
               supersedes="config/lens/std_1920x1080_f0.50_cal20261023.json (night only, not repeatable; kept)",
               note="Not in lens_calib.DEFAULT_LENS yet.", made=time.strftime("%Y-%m-%d %H:%M:%S"))
    if status:
        out["status"], out["status_note"] = status, status_note
    if extra:
        out.update(extra)
    p = S3D / "config" / "lens" / out_name
    json.dump(out, open(p, "w"), indent=1, ensure_ascii=False)
    print("->", p)



if __name__ == "__main__":
    import sys
    if "--1024b" in sys.argv:
        write_1024(("all1024b",), "std_1920x1080_f0.50_cal20261024b.json", 59, "lopo_pass_halves_fail",
                   "Leave-one-pose-out passes (fx -0.10/+0.07 %, pp within 0.8 px). Disjoint pose halves (62 splits): median |dfx| 0.29 %, "
                   "|dcx| 1.6, |dcy| 2.0 px; fx <= 0.3 % in 52 %, pp <= 3 px in 56 %, both in 24 % -> FAIL. Cause: fx depends on board tilt "
                   "(frontal + 25-45 deg poses: fx 1287.5; frontal + >=45 deg: 1279.1, -0.65 %), a systematic error, so more poses of the same kind do not fix it. "
                   "docs/lens_calib_f050_20261023.md section 10.",
                   dict(uncertainty="fx about +-0.35 % (tilt-dependent systematic, estimate), principal point about +-2 px",
                        supersedes="std_1920x1080_f0.50_cal20261024.json and _cal20261023.json (both kept)",
                        source="scan3d/runs/lens_calib_f050_20261023 (calib.py --tag all1024b, split_test.py, aspect_scan.py, run_lt55.sh, write_lens.py --1024b)",
                        model_choice="kept k12_notan (coordinator: unless evidence says otherwise). pp-repeatable models are tied: holdout k12_notan 0.154/0.115, k1_prior 0.150/0.116, k123_notan 0.157/0.115; LOPO 0.130/0.131/0.129. k1_prior passes most half-splits (40 %) but borrows k2 p1 p2 k3 from the 2.22 HS node. rational/k1k2p_k3: pp not repeatable (halves |dcx| p90 17 px)",
                        tilt_split_fx=json.load(open(HERE / "results" / "split_test.json"))["groups"]))
    elif "--1024" in sys.argv:
        write_1024()
    else:
        write_1023()
