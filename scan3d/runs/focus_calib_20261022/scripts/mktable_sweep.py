"""Sweep-based table: K, dist of config/lens/hs_1920x1080_f2.22.json (best-covered calibration) with fx, fy scaled by
the same-pose ArUco image scale of each focus step vs 2.22 dpt (ftest_cal.json 'off' column, clip hs_20261022_105441).
usage: mktable_sweep.py FTEST_JSON RESULTS_CSV LENS OUT"""
import collections, csv, json, sys, time
import numpy as np
ft = json.load(open(sys.argv[1]))
fd = collections.Counter(float(r["focus_dpt"]) for r in csv.DictReader(open(sys.argv[2])))
lens = json.load(open(sys.argv[3]))
K0 = np.array(lens["K"])
ent = []
for key, v in sorted(ft.items()):
    tag, ref, D = key.split("|")
    if tag != "off" or ref != "2.22":
        continue
    D = float(D)
    exact = max((x for x in fd if abs(x - D) < 0.006), key=lambda x: fd[x])
    s = 1 + v["scale_pct"] / 100
    K = K0.copy(); K[0, 0] *= s; K[1, 1] *= s
    ent.append(dict(focus_dpt=exact, K=K.tolist(), dist=lens["dist"], model=lens["model"], image_scale_vs_2p22=round(s, 6),
                    scale_p10_p90_pct=[round(v["p10"], 3), round(v["p90"], 3)], n_frames=v["n"]))
tab = dict(
    what="per-focus lens intrinsics, 1920x1080 hs mode (cam5), for vio_bench_hs.py --focus-comp --focus-table (RECOMMENDED "
         "of the two tables, docs/focus_calib_20261022.md)",
    entries=sorted(ent, key=lambda e: e["focus_dpt"]),
    source="K/dist: " + sys.argv[3].split("/")[-1] + f" (rec {lens['recs']}, made {lens['made']}); fx, fy x image scale "
           "of each focus step vs 2.22 dpt measured at one pose on the focus-sweep clip hs_20261022_105441 (ArUco "
           "similarity fit, median over 145-185 frames). Principal point and distortion: unchanged (focus calibrations "
           "of 2026-10-22 could not resolve a change: cx, cy differences 3-6 px within 95 % CIs of +-3-10 px).",
    interpolation="pixel-map blend linear in dpt between bracketing nodes (vio_bench_hs.FocusComp.table_map); clamped "
                  "outside. Same model at all nodes and only fx, fy differ, so this equals a piecewise-linear --focus-k "
                  "(non-linear near 0.37: 0.37->1.0 slope +0.64 %/dpt vs +0.80 overall).",
    caveat="One pose, one clip, board at about 45 cm; fitted on the same clip used for its validation (not independent).",
    made=time.strftime("%Y-%m-%d %H:%M:%S"))
json.dump(tab, open(sys.argv[4], "w"), indent=1, ensure_ascii=False)
for e in tab["entries"]:
    print(e["focus_dpt"], e["image_scale_vs_2p22"], round(e["K"][0][0], 2))
