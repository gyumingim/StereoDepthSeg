"""FAR two-marker set (2026-10-24): (1) fx for tilt 30-37 vs 38-45 groups (fitted tilt), alone and + old frontal ChArUco;
(2) live tilt readout check: tools/board2_markers.detect (single-frame IPPE pick, v4 layout, K of the live page = provisional
0.5 dpt K without distortion) on the same video frames vs the joint-fit tilt. → results/far_groups.json"""
import json, sys, time
from pathlib import Path
import cv2
import numpy as np
HERE = Path(__file__).resolve().parent
S3D = HERE.parents[1]
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(S3D / "tools"))
import calib
import board2_markers as b2
exA = open(HERE / "logs" / "ex_A_nonfar.txt").read().split()
exB = open(HERE / "logs" / "ex_B_oldtilted.txt").read().split()
M = "k12_notan"
views, clusters, members = calib.build(exA)
rms, K, d, rv, tv = calib.fit(views, M)
tilt = {}
for v, r_, t_ in zip(views, rv, tv):
    tilt.setdefault(v["rec"], []).append(calib.tilt_dist(r_, t_)[0])
tilt = {k: float(np.mean(x)) for k, x in tilt.items()}
far = sorted(tilt)
g1 = [r for r in far if tilt[r] < 37.5]
g2 = [r for r in far if tilt[r] >= 37.5]
out = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), model=M, far_fit=dict(fx=K[0, 0], fy=K[1, 1], cx=K[0, 2], cy=K[1, 2], rms=rms),
           tilt=tilt, groups={})
viewsB, _, _ = calib.build(exB)
frontal_old = [v for v in viewsB if v["rec"] not in far]
for name, g in (("30-37", g1), ("38-45+", g2)):
    a = calib.fit([v for v in views if v["rec"] in g], M)
    b = calib.fit([v for v in views if v["rec"] in g] + frontal_old, M)
    out["groups"][name] = dict(n=len(g), tilt_mean=float(np.mean([tilt[r] for r in g])), alone=dict(fx=a[1][0, 0], cx=a[1][0, 2], cy=a[1][1, 2], rms=a[0]),
                               with_frontal=dict(fx=b[1][0, 0], cx=b[1][0, 2], cy=b[1][1, 2], rms=b[0]))
    print(f"group {name}: n {len(g)} tilt {out['groups'][name]['tilt_mean']:.1f} | alone fx {a[1][0,0]:.1f} cx {a[1][0,2]:.1f} cy {a[1][1,2]:.1f} "
          f"rms {a[0]:.3f} | + old frontal fx {b[1][0,0]:.1f} cx {b[1][0,2]:.1f} cy {b[1][1,2]:.1f}")
# live readout check
Klive = np.array([[1288.8, 0, 966.9], [0, 1290.0, 553.7], [0, 0, 1.0]])
rows = []
for v in views:
    if v["rec"] in [x["rec"] for x in rows]:
        continue
    cap = cv2.VideoCapture(str(S3D / "recordings" / v["rec"] / "cam5.mp4"))
    cap.set(cv2.CAP_PROP_POS_FRAMES, v["frame"])
    ok, f = cap.read()
    g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
    r_live = b2.detect(g, Klive)
    r_fitK = b2.detect(g, K)
    row = dict(rec=v["rec"], fit_tilt=tilt[v["rec"]], live_tilt=r_live["tilt"] if r_live else None,
               live_ambiguous=r_live["ambiguous"] if r_live else None, live_turn=r_live["turn"] if r_live else None,
               live_lean=r_live["lean"] if r_live else None, live_dist=r_live["dist_m"] if r_live else None,
               fitK_tilt=r_fitK["tilt"] if r_fitK else None)
    rows.append(row)
    print(f"{v['rec'][12:18]} fit tilt {row['fit_tilt']:5.1f} | live detect tilt {row['live_tilt']:5.1f} (turn {row['live_turn']:+.0f} lean {row['live_lean']:+.0f}, "
          f"amb {row['live_ambiguous']}, dist {row['live_dist']:.2f}) | detect with fitted K {row['fitK_tilt']:5.1f}")
dt = np.array([r["live_tilt"] - r["fit_tilt"] for r in rows if r["live_tilt"] is not None])
out["live"] = dict(rows=rows, diff_mean=float(dt.mean()), diff_sd=float(dt.std()), diff_max=float(np.abs(dt).max()))
print(f"live - fit tilt: mean {dt.mean():+.2f} sd {dt.std():.2f} max |{np.abs(dt).max():.2f}| deg")
json.dump(out, open(HERE / "results" / "far_groups.json", "w"), indent=1, default=float)
