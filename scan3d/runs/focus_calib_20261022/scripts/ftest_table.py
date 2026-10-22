"""Residual ArUco scale per focus step on the focus-sweep clip (ftest2 = hs_20261022_105441): off / linear k / table.
Same method as fc/ftest.py (every 6th frame, lens-moving 40 ms dropped, similarity fit to reference-step mean corners).
usage: ftest_table.py REC_DIR LENS_JSON TABLE_JSON [k]"""
import collections, csv, json, sys
from pathlib import Path
import cv2, numpy as np
S3 = Path.home() / "scanner/StereoDepthSeg/scan3d"
sys.path[:0] = [str(S3), str(S3 / "imu_learn")]
import vio_bench_hs as vb  # noqa: E402
import lens_calib  # noqa: E402

rec, lens, table = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
k = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0080
K, dist, lj = lens_calib.load(lens)
d_ref = float(lj["focus_dpt"])
meta = json.load(open(rec / "meta.json"))
Wf, Hf = int(meta["width"]), int(meta["height"])
rows = list(csv.DictReader(open(rec / "frames_cam5.csv")))
fl = vb.FocusComp(rec, rows, K, dist, K, (Wf, Hf), (Wf, Hf), d_ref, k, 0.0, 40.0, True, True)
ft = vb.FocusComp(rec, rows, K, dist, K, (Wf, Hf), (Wf, Hf), d_ref, k, 0.0, 40.0, True, True, table=table)
m_off = cv2.initUndistortRectifyMap(K, dist, None, K, (Wf, Hf), cv2.CV_16SC2)
det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250))
TAGS = ("off", "linear", "table")
per = {t: collections.defaultdict(list) for t in TAGS}
cap = cv2.VideoCapture(str(rec / "cam5.mp4"))
for fi in range(len(rows)):
    if not cap.grab():
        break
    if fi % 6 or fl.moving(fi):
        continue
    _, im = cap.retrieve()
    g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    D = round(float(fl.fd[fl.nearest(fi)]), 2)
    for tag, mp in (("off", m_off), ("linear", fl.map_for(fi) or m_off), ("table", ft.map_for(fi) or m_off)):
        u = cv2.remap(g, *mp, cv2.INTER_LINEAR)
        mc, mi, _ = det.detectMarkers(u)
        if mi is not None and len(mi) >= 5:
            ids = np.concatenate([[int(i) * 4 + q for q in range(4)] for i in mi.ravel()])
            per[tag][D].append((ids, np.concatenate([c.reshape(4, 2) for c in mc])))
print(f"lens {Path(lens).name} D_ref {d_ref:.3f}, k {k}, table {Path(table).name} nodes {ft.tab_d.tolist()}, "
      f"clamped {ft.n_clamped}")
out = {}
for tag in TAGS:
    steps = sorted(per[tag])
    for ref in steps:
        acc = collections.defaultdict(list)
        for ids, c in per[tag][ref]:
            for i, p in zip(ids, c):
                acc[int(i)].append(p)
        R = {i: np.mean(v, 0) for i, v in acc.items() if len(v) >= 3}
        for Dd in steps:
            sc, tr = [], []
            for ids, c in per[tag][Dd]:
                com = [(R[int(i)], p) for i, p in zip(ids, c) if int(i) in R]
                if len(com) < 16:
                    continue
                A_ = np.float32([a for a, _ in com]); B_ = np.float32([b for _, b in com])
                M, _ = cv2.estimateAffinePartial2D(A_ - [Wf / 2, Hf / 2], B_ - [Wf / 2, Hf / 2], method=cv2.LMEDS)
                sc.append(np.hypot(M[0, 0], M[1, 0])); tr.append(M[:, 2])
            s = np.array(sc); t = np.array(tr)
            out[f"{tag}|{ref}|{Dd}"] = dict(scale_pct=float((np.median(s) - 1) * 100), p10=float((np.percentile(s, 10) - 1) * 100),
                                          p90=float((np.percentile(s, 90) - 1) * 100), n=len(s),
                                          shift_px=[float(x) for x in np.median(t, 0)])
for tag in TAGS:
    steps = sorted(per[tag])
    print(f"\n{tag}: residual scale % (median) of step (row) vs reference step (col); p10..p90 for ref 2.22")
    print("        " + "".join(f"ref {r:5.2f}      " for r in steps))
    for Dd in steps:
        print(f"  {Dd:5.2f} " + "".join(f"{out[f'{tag}|{r}|{Dd}']['scale_pct']:+8.3f}        " for r in steps)
              + f" | vs 2.22: p10..p90 {out[f'{tag}|2.22|{Dd}']['p10']:+.3f}..{out[f'{tag}|2.22|{Dd}']['p90']:+.3f}, "
              f"shift {np.round(out[f'{tag}|2.22|{Dd}']['shift_px'], 2).tolist()} px, n={out[f'{tag}|2.22|{Dd}']['n']}")
json.dump(out, open(sys.argv[5] if len(sys.argv) > 5 else "ftest_table_out.json", "w"), indent=1)
