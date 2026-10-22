import cv2, csv, numpy as np, sys, collections
D = sys.argv[1]
fr = list(csv.DictReader(open(D + "/frames_cam5.csv")))
tf = np.array([int(r["t_mid_ns"]) for r in fr], dtype=np.int64)
res = list(csv.DictReader(open(D + "/results_cam5.csv")))
rt = np.array([int(r["sensor_ts_ns"]) for r in res], dtype=np.int64)
rf = np.array([float(r["focus_dpt"]) for r in res]); rs = np.array([int(r["lens_state"]) for r in res])
def info(t):
    i = int(np.clip(np.searchsorted(rt, t), 0, len(rt) - 1)); return rf[i], rs[i]
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
board = cv2.aruco.CharucoBoard((9, 5), 63.294, 63.294 * 0.5882352941176471, dic)
md = cv2.aruco.ArucoDetector(dic)
cap = cv2.VideoCapture(D + "/cam5.mp4")
per = collections.defaultdict(list)   # focus -> list of (ids, corners, sharp)
k = 0
while True:
    ok, im = cap.read()
    if not ok: break
    if k % 6 == 0 and k < len(tf):
        f, st = info(tf[k])
        # skip frames within 60 ms after any lens movement
        j = int(np.clip(np.searchsorted(rt, tf[k]), 0, len(rt) - 1))
        recent = rs[max(0, j - 4):j + 1].max()
        if not recent:
            g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
            mc, mi, _ = md.detectMarkers(g)
            if mi is not None and len(mi) >= 5:
                ids = np.concatenate([[int(i) * 4 + q for q in range(4)] for i in mi.ravel()])
                pts = np.concatenate([c.reshape(4, 2) for c in mc])
                per[round(f, 2)].append((ids, pts, cv2.Laplacian(g, cv2.CV_64F).var()))
    k += 1
ref_f = 0.37 if 0.37 in per else sorted(per)[0]
# reference corner positions: mean over reference-focus frames
def mean_pos(lst):
    acc = collections.defaultdict(list)
    for ids, c, _ in lst:
        for i, p in zip(ids, c): acc[int(i)].append(p)
    return {i: np.mean(v, 0) for i, v in acc.items() if len(v) >= 3}
R = mean_pos(per[ref_f])
print("frames per focus:", {f: len(v) for f, v in sorted(per.items())})
for f in sorted(per):
    scales = []
    for ids, c, _ in per[f]:
        common = [(R[int(i)], p) for i, p in zip(ids, c) if int(i) in R]
        if len(common) < 16: continue
        a = np.float32([x for x, _ in common]); b = np.float32([y for _, y in common])
        M, _ = cv2.estimateAffinePartial2D(a, b, method=cv2.LMEDS)
        scales.append(np.hypot(M[0, 0], M[1, 0]))
    sh = np.median([x[2] for x in per[f]])
    s = np.array(scales)
    print(f"focus {f:.2f} dpt ({100 / f:.0f} cm): scale vs {ref_f} dpt = {np.median(s):.5f} ({(np.median(s) - 1) * 100:+.3f} %), spread(p10-p90) {np.percentile(s,10):.5f}-{np.percentile(s,90):.5f}, n={len(s)}, sharpness={sh:.0f}")
# thin-lens prediction relative to ref: m ~ 1/(1 - f*D) with f = 5.4 mm
fl = 0.0054
for f in sorted(per):
    print(f"  thin-lens predicted {f:.2f}: {((1/(1-fl*f))/(1/(1-fl*ref_f)) - 1) * 100:+.3f} %")
