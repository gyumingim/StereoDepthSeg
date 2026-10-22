"""Build config/lens/hs_1920x1080_focus_table.json from cal_<rec>.json (calib3.py). usage: mktable.py DIR OUT"""
import json, sys, time
import numpy as np
D, OUT = sys.argv[1], sys.argv[2]
M = "rational"
recs = [("hs_20261022_120556", 0.37133896), ("hs_20261022_120450", 1.0007355), ("hs_20261022_115405", 3.3286483)]
ent = []
for rec, dpt in recs:
    j = json.load(open(f"{D}/cal_{rec[-6:]}.json"))
    f = j["fits"][M]
    def ci(key):
        out = {}
        for kind in ("frame", "block"):
            b = np.array([[x["K"][0][0], x["K"][1][1], x["K"][0][2], x["K"][1][2]] for x in j["boot"][f"{M}_{kind}"]])
            lo, hi = np.percentile(b, [2.5, 97.5], 0)
            out[kind] = {n: [round(float(l), 2), round(float(h), 2)] for n, l, h in zip(("fx", "fy", "cx", "cy"), lo, hi)}
        return out
    ent.append(dict(focus_dpt=dpt, K=f["K"], dist=f["dist"], model=M, rms_px=round(f["rms_px"], 4),
                    holdout_px={k: round(v, 4) for k, v in j["holdout_px"].items()}, n_views=f["n_views"],
                    coverage_cells_32x18=round(j["coverage_cells_32x18"], 3), coverage_hull=round(j["coverage_hull"], 3),
                    board_tilt_deg_p10_50_90=[round(x, 1) for x in f["tilt_deg_pct"]],
                    board_dist_m_p10_50_90=[round(x * (1000 if x < 0.05 else 1), 3) for x in f["dist_m_pct"]],   # cal_* made with a /1000 bug
                    ci95_bootstrap=ci(M), n_boot=len(j["boot"][f"{M}_block"]), rec=rec))
tab = dict(
    what="per-focus lens intrinsics, 1920x1080 hs mode (cam5), for vio_bench_hs.py --focus-comp --focus-table",
    entries=ent,
    interpolation="vio_bench_hs.FocusComp: per node the raw-frame sample map for the same output pinhole is built; the map "
                  "at focus D is the linear blend (in dpt) of the two bracketing node maps = linear interpolation of the "
                  "projection, NOT of the coefficients (rational coefficients are ill-conditioned: k1..k6 of tens to "
                  "hundreds trade off). D outside [first, last] node is clamped. Nodes are piecewise-linear; 0.37-1.0 and "
                  "1.0-3.33 dpt segments.",
    warning="NOT RECOMMENDED (docs/focus_calib_20261022.md): board views are nearly fronto-parallel (median tilt 8-14 deg) "
            "so fx is poorly determined; the same 0.37 dpt focus calibrated from hs_20261022_111605 gives a -4.2 % image "
            "scale (outside these bootstrap CIs), and using all views instead of 120 moves fx at 1.0 dpt by -2.4 %. On the "
            "focus-sweep clip (hs_20261022_105441) this table leaves -0.54..+0.79 % residual scale and 1-4 px shift across "
            "0.37-3.33 dpt (no compensation: -1.37..+0.90 %; --focus-k 0.008: 0.00..+0.10 %). Use "
            "hs_1920x1080_focus_table_sweep.json (or --focus-k) until a calibration with tilted views (30-45 deg) exists.",
    board="board_monitor_6x6.json (square 63.294 mm; does not affect K)",
    tool="lens_calib.py selection (|w|<0.3 rad/s, >=20 corners, 3x3 x tilt bins, 120 views, 2.5x median outlier refit) "
         "+ drop phone-on-stand views (|w|<0.04 rad/s for +-0.5 s) and views sharper than 0.5x median only; model forced "
         "rational for all (best holdout in all three); bootstrap 300x over views (frame) and over 1 s blocks (block)",
    made=time.strftime("%Y-%m-%d %H:%M:%S"))
json.dump(tab, open(OUT, "w"), indent=1, ensure_ascii=False)
print("wrote", OUT)
