import cv2, csv, numpy as np, sys, collections
D = sys.argv[1]
fr = list(csv.DictReader(open(D + "/frames_cam5.csv"))); tf = np.array([int(r["t_mid_ns"]) for r in fr], dtype=np.int64)
res = sorted(csv.DictReader(open(D + "/results_cam5.csv")), key=lambda r: int(r["sensor_ts_ns"]))
rt = np.array([int(r["sensor_ts_ns"]) for r in res], dtype=np.int64); rf = np.array([float(r["focus_dpt"]) for r in res]); rs = np.array([int(r["lens_state"]) for r in res])
md = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250))
cap = cv2.VideoCapture(D + "/cam5.mp4"); per = collections.defaultdict(list); k = 0
while True:
    ok, im = cap.read()
    if not ok: break
    if k % 4 == 0 and k < len(tf):
        j = int(np.clip(np.searchsorted(rt, tf[k]), 0, len(rt) - 1))
        if rs[max(0, j - 4):j + 1].max() == 0:
            c, ids, _ = md.detectMarkers(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY))
            if ids is not None and len(ids) >= 4:
                per[round(rf[j], 2)].append({int(i) * 4 + q: p for i, cc in zip(ids.ravel(), c) for q, p in enumerate(cc.reshape(4, 2))})
    k += 1
def mean(lst):
    acc = collections.defaultdict(list)
    for d in lst:
        for i, p in d.items(): acc[i].append(p)
    return {i: np.mean(v, 0) for i, v in acc.items() if len(v) >= 5}
ref = 2.22; R = mean(per[ref]); print("frames per focus", {f: len(v) for f, v in sorted(per.items())}, "ref pts", len(R))
for f in sorted(per):
    M = mean(per[f]); ids = [i for i in M if i in R]
    a = np.array([R[i] for i in ids]); b = np.array([M[i] for i in ids])
    # fit b = c + s (a - c)  ->  b = s a + t,  t = c (1 - s)
    A = np.zeros((2 * len(a), 3)); A[0::2, 0] = a[:, 0]; A[1::2, 0] = a[:, 1]; A[0::2, 1] = 1; A[1::2, 2] = 1
    sol, *_ = np.linalg.lstsq(A, b.reshape(-1), rcond=None); s, tx, ty = sol
    resid = np.sqrt(np.mean((A @ sol - b.reshape(-1)) ** 2))
    c = (tx / (1 - s), ty / (1 - s)) if abs(1 - s) > 1e-5 else (np.nan, np.nan)
    print(f"focus {f:.2f}: scale {100*(s-1):+.3f} %  shift ({tx:+.2f},{ty:+.2f}) px  zoom-center ({c[0]:.0f},{c[1]:.0f})  rms {resid:.3f} px  n={len(ids)}  board x {a[:,0].min():.0f}-{a[:,0].max():.0f} y {a[:,1].min():.0f}-{a[:,1].max():.0f}")
