"""Per-recording lens calibration with lens_calib's selection/fit/holdout, all 3 models, + frame bootstrap.
usage: calib3.py VIEWS.pkl OUT.json [--max-w 0.3] [--boot B] [--model rational] [--workers 5]"""
import argparse, json, pickle, sys, time
from multiprocessing import Pool
from pathlib import Path
import cv2, numpy as np
S3 = Path.home() / "scanner/StereoDepthSeg/scan3d"
sys.path[:0] = [str(S3), str(S3 / "imu_learn")]
import lens_calib as lc

ap = argparse.ArgumentParser()
ap.add_argument("views"); ap.add_argument("out")
ap.add_argument("--max-w", type=float, default=0.3)
ap.add_argument("--boot", type=int, default=0)
ap.add_argument("--models", default="k1k2p,k1k2p_k3,rational")
ap.add_argument("--boot-models", default="rational,k1k2p_k3")
ap.add_argument("--workers", type=int, default=5)
ap.add_argument("--stock", action="store_true", help="no stand/sharpness filter (= lens_calib default selection)")
ap.add_argument("--sharp-frac", type=float, default=0.5)
ap.add_argument("--n", type=int, default=120)
a = ap.parse_args()
P = pickle.load(open(a.views, "rb"))
size = tuple(P["size"])
A = sorted(P["views"], key=lambda v: v["fi"])
fi = np.array([v["fi"] for v in A]); ww = np.array([v["w"] for v in A])
stand = np.array([bool((ww[(fi >= f - 120) & (fi <= f + 120)] < 0.04).all()) for f in fi])   # phone on the stand
sh = np.array([v["sharp"] for v in A])
sharp_min = a.sharp_frac * float(np.median(sh[~stand])) if (~stand).any() else 0.0
if a.stock:
    V = [v for v in A if v["w"] < a.max_w]
else:
    V = [v for v, st, s_ in zip(A, stand, sh) if v["w"] < a.max_w and not st and s_ >= sharp_min]
tup = [(v["obj"], v["img"], v["cx"], v["cy"], v["tilt"]) for v in V]
pick, nb = lc.spread(tup, size, a.n)
sel = [tup[k] for k in pick]
selv = [V[k] for k in pick]


def coverage(views, size, gx=32, gy=18):
    W, H = size
    occ = np.zeros((gy, gx), bool)
    pts = np.concatenate([v[1].reshape(-1, 2) for v in views])
    occ[np.clip((pts[:, 1] / H * gy).astype(int), 0, gy - 1), np.clip((pts[:, 0] / W * gx).astype(int), 0, gx - 1)] = True
    hull = cv2.convexHull(pts.astype(np.float32))
    return float(occ.mean()), float(cv2.contourArea(hull) / (W * H)), occ


def final_fit(sel, flags):
    rms, K, d, rv, tv = lc.fit(sel, size, flags)
    per = np.array([np.sqrt(np.mean(np.sum((cv2.projectPoints(v[0], r_, t_, K, d)[0] - v[1]) ** 2, -1)))
                    for v, r_, t_ in zip(sel, rv, tv)])
    keep = per < 2.5 * np.median(per)
    if (~keep).any():
        sel = [v for v, k in zip(sel, keep) if k]
        rms, K, d, rv, tv = lc.fit(sel, size, flags)
    tilt = [float(np.degrees(np.arccos(abs(cv2.Rodrigues(r_)[0][2, 2])))) for r_ in rv]
    dist_m = [float(np.linalg.norm(t_)) for t_ in tv]
    return rms, K, d, sel, int((~keep).sum()), keep, tilt, dist_m


def boot_one(args):
    seed, model, sel, blocks = args
    rng = np.random.default_rng(seed)
    if blocks is None:
        idx = rng.integers(0, len(sel), len(sel))
    else:                                                   # resample whole 1 s blocks (neighbouring frames correlate)
        ub = sorted(set(blocks))
        idx = np.concatenate([np.nonzero(np.asarray(blocks) == ub[j])[0] for j in rng.integers(0, len(ub), len(ub))])
    rms, K, d, _, _ = lc.fit([sel[i] for i in idx], size, lc.MODELS[model])
    return K.tolist(), d.tolist(), float(rms)


t0 = time.time()
out = dict(views_pkl=a.views, max_w=a.max_w, n_frames=P["n_frames"], n_slow_0p3=P["n_slow"], n_views_detected=len(P["views"]),
           n_views_below_w=len(V), n_stand=int(stand.sum()), sharp_min=sharp_min, stock=a.stock, n_bins=nb, n_selected=len(sel), n_req=a.n, size=list(size),
           sel_w_median=float(np.median([v["w"] for v in selv])), sel_w_max=float(max(v["w"] for v in selv)),
           sel_sharp_median=float(np.median([v["sharp"] for v in selv])),
           sel_corners_median=float(np.median([len(v["ids"]) for v in selv])),
           sel_tilt_deg_pct=[float(x) for x in np.percentile([v["tilt"] for v in selv], [10, 50, 90])],
           made=time.strftime("%Y-%m-%d %H:%M:%S"))
c1, c2, occ = coverage(sel, size)
out.update(coverage_cells_32x18=c1, coverage_hull=c2, occ_rows=["".join("#" if x else "." for x in r) for r in occ])
out["holdout_px"] = {m: lc.holdout_err(sel, size, lc.MODELS[m]) for m in a.models.split(",")}
print(json.dumps({k: out[k] for k in ("n_views_detected", "n_views_below_w", "n_selected", "coverage_cells_32x18",
                                      "coverage_hull", "holdout_px")}), f"{time.time() - t0:.0f}s", flush=True)
out["fits"] = {}
finsel = {}
for m in a.models.split(","):
    rms, K, d, s2, nrej, keep, tilt, dist_m = final_fit(sel, lc.MODELS[m])
    blk = [int(v["fi"] // 240) for v, k in zip(selv, keep) if k]
    finsel[m] = (s2, blk)
    out["fits"][m] = dict(K=K.tolist(), dist=d.tolist(), rms_px=float(rms), n_views=len(s2), n_rejected=nrej,
                          tilt_deg_pct=[float(x) for x in np.percentile(tilt, [10, 50, 90])],
                          dist_m_pct=[float(x) for x in np.percentile(dist_m, [10, 50, 90])], n_blocks_1s=len(set(blk)))
    print(m, f"rms {rms:.3f} f {K[0,0]:.2f} {K[1,1]:.2f} c {K[0,2]:.2f} {K[1,2]:.2f}", f"{time.time() - t0:.0f}s", flush=True)
json.dump(out, open(a.out, "w"), indent=1)
if a.boot:
    out["boot"] = {}
    for m in a.boot_models.split(","):
      for kind in ("frame", "block"):
        with Pool(a.workers) as pool:
            r = pool.map(boot_one, [(s, m, finsel[m][0], None if kind == "frame" else finsel[m][1]) for s in range(a.boot)])
        out["boot"][f"{m}_{kind}"] = [dict(K=k, dist=d, rms=e) for k, d, e in r]
        f = np.array([[x[0][0][0], x[0][1][1], x[0][0][2], x[0][1][2]] for x in r])
        print(m, kind, "boot sd fx fy cx cy", np.round(f.std(0), 3), f"{time.time() - t0:.0f}s", flush=True)
        json.dump(out, open(a.out, "w"), indent=1)
