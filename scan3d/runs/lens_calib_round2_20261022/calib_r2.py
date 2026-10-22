"""Per-recording and pooled-per-focus calibration (lens_calib.fit = cv2.calibrateCamera, same TERM and 2.5x-median
view rejection), models rational (lens_calib's choice in round 1) and k1k2p_k3, 1 s block bootstrap.

usage: calib_r2.py UNIT [--boot 100] [--n 160] [--wmax 0.05] [--workers 16] [--tag ""]
  UNIT = recording short name (e.g. 162218) or focus key (2.22 / 1.00 / 3.33 / 0.37 = pooled recordings)
"""
import argparse, json, sys, time
from multiprocessing import Pool
import numpy as np, cv2

from common import D, S3, SIZE, FOCUS, REC2F, ROUND, load_views, spread, coverage, tilt_table, cvtup
sys.path[:0] = [str(S3), str(S3 / "imu_learn")]
import lens_calib as lc

ap = argparse.ArgumentParser()
ap.add_argument("unit")
ap.add_argument("--boot", type=int, default=100)
ap.add_argument("--n", type=int, default=0, help="0 = 160 per recording, 320 pooled")
ap.add_argument("--wmax", type=float, default=0.05)
ap.add_argument("--workers", type=int, default=16)
ap.add_argument("--models", default="rational,k1k2p_k3")
ap.add_argument("--tag", default="")
ap.add_argument("--corner-reject", type=float, default=0, help="sensitivity: drop single corners with error > this x median corner error (after a first fit), then refit")
ap.add_argument("--max-tilt", type=float, default=90, help="sensitivity: drop views with rough tilt >= this (deg)")
a = ap.parse_args()
cv2.setNumThreads(1)

recs = FOCUS[a.unit] if a.unit in FOCUS else [a.unit]
focus = a.unit if a.unit in FOCUS else REC2F[a.unit]
n_req = a.n or (320 if a.unit in FOCUS else 160)
views, counts = [], {}
for r in recs:
    v, c = load_views(r, wmax=a.wmax)
    if a.max_tilt < 90:
        v = [x for x in v if x["tilt"] < a.max_tilt]; c["n_after_max_tilt"] = len(v)
    views += v; counts[r] = c
pick, nbins = spread(views, n_req)
sel = [views[k] for k in pick]
cov_all, _ = coverage(views)
cov_sel, occ = coverage(sel)
out = dict(unit=a.unit, focus_dpt=focus, recs=recs, rounds=[ROUND[r] for r in recs], wmax=a.wmax, counts=counts,
           n_usable=len(views), n_bins_usable=nbins, n_req=n_req, n_selected=len(sel),
           n_selected_per_rec={r: sum(v["rec"] == r for v in sel) for r in recs},
           cov32x18_usable=cov_all, cov32x18_selected=cov_sel, occ_rows=["".join("#" if x else "." for x in r) for r in occ],
           tilt_usable=tilt_table(views), tilt_selected=tilt_table(sel),
           tilt_pct_selected=[float(x) for x in np.percentile([v["tilt"] for v in sel], [10, 50, 90])],
           n_blocks_selected=len({v["block"] for v in sel}), made=time.strftime("%Y-%m-%d %H:%M:%S"),
           script="runs/lens_calib_round2_20261022/calib_r2.py", board="config/charuco/board_monitor_6x6.json (square 63.294 mm)")
print(a.unit, "usable", len(views), "selected", len(sel), "cov", round(cov_all, 3), round(cov_sel, 3), flush=True)
tup = [cvtup(v) for v in sel]
out["holdout_px"] = {m: lc.holdout_err(tup, SIZE, lc.MODELS[m]) for m in a.models.split(",")}


def corner_clean(tup, flags, k):
    """drop single corners with reprojection error > max(1 px, k x median) after one fit (views keep >= 8 corners)"""
    rms, K, d, rv, tv = lc.fit(tup, SIZE, flags)
    errs = [np.linalg.norm((cv2.projectPoints(v[0], r_, t_, K, d)[0] - v[1]).reshape(-1, 2), axis=1) for v, r_, t_ in zip(tup, rv, tv)]
    thr = max(1.0, k * float(np.median(np.concatenate(errs))))
    out, nd = [], 0
    for v, e in zip(tup, errs):
        m = e < thr
        nd += int((~m).sum())
        out.append((v[0][m], v[1][m], v[2], v[3], v[4]) if m.sum() >= 8 else v)
    return out, nd, thr


def final_fit(tup, blocks, flags):
    rms, K, d, rv, tv = lc.fit(tup, SIZE, flags)
    per = np.array([np.sqrt(np.mean(np.sum((cv2.projectPoints(v[0], r_, t_, K, d)[0] - v[1]) ** 2, -1)))
                    for v, r_, t_ in zip(tup, rv, tv)])
    keep = per < 2.5 * np.median(per)
    if (~keep).any():
        tup = [v for v, k in zip(tup, keep) if k]; blocks = [b for b, k in zip(blocks, keep) if k]
        rms, K, d, rv, tv = lc.fit(tup, SIZE, flags)
    tilt = [float(np.degrees(np.arccos(abs(cv2.Rodrigues(r_)[0][2, 2])))) for r_ in rv]
    dist = [float(np.linalg.norm(t_)) for t_ in tv]
    return rms, K, d, tup, blocks, int((~keep).sum()), tilt, dist


def boot_one(args):
    seed, flags, tup, blocks = args
    rng = np.random.default_rng(seed)
    ub = sorted(set(blocks)); barr = np.asarray(blocks)
    idx = np.concatenate([np.nonzero(barr == ub[j])[0] for j in rng.integers(0, len(ub), len(ub))])
    rms, K, d, _, _ = lc.fit([tup[i] for i in idx], SIZE, flags)
    return [K[0, 0], K[1, 1], K[0, 2], K[1, 2]], d.tolist(), float(rms)


t0 = time.time()
out["fits"] = {}
blocks0 = [v["block"] for v in sel]
fin = {}
for m in a.models.split(","):
    tup_m = tup
    if a.corner_reject:
        tup_m, ncd, cthr = corner_clean(tup, lc.MODELS[m], a.corner_reject)
        out.setdefault("corner_reject", {})[m] = dict(n_dropped=ncd, thr_px=cthr)
        print(m, f"corner reject: dropped {ncd} corners (> {cthr:.2f} px)", flush=True)
    rms, K, d, t2, b2, nrej, tilt, dist = final_fit(tup_m, blocks0, lc.MODELS[m])
    fin[m] = (t2, b2)
    out["fits"][m] = dict(K=K.tolist(), dist=d.tolist(), fx=K[0, 0], fy=K[1, 1], cx=K[0, 2], cy=K[1, 2], rms_px=float(rms),
                          n_views=len(t2), n_rejected=nrej, n_blocks=len(set(b2)),
                          tilt_deg_pct=[float(x) for x in np.percentile(tilt, [10, 50, 90, 100])],
                          dist_m_pct=[float(x) for x in np.percentile(dist, [10, 50, 90])])
    print(m, f"rms {rms:.3f} f {K[0,0]:.2f} {K[1,1]:.2f} c {K[0,2]:.2f} {K[1,2]:.2f} tilt {np.percentile(tilt,[50,90]).round(1)}",
          f"{time.time() - t0:.0f}s", flush=True)
name = f"{a.unit}{a.tag}"
(D / "results").mkdir(exist_ok=True)
json.dump(out, open(D / "results" / f"cal_{name}.json", "w"), indent=1)
if a.boot:
    out["boot"] = {}
    for m in a.models.split(","):
        with Pool(a.workers) as pool:
            r = pool.map(boot_one, [(s, lc.MODELS[m], fin[m][0], fin[m][1]) for s in range(a.boot)])
        f = np.array([x[0] for x in r])
        out["boot"][m] = dict(fxfycxcy=f.tolist(), dist=[x[1] for x in r], rms=[x[2] for x in r],
                              sd=f.std(0, ddof=1).tolist(), ci95=np.percentile(f, [2.5, 97.5], axis=0).tolist())
        print(m, "block boot sd fx fy cx cy", np.round(f.std(0, ddof=1), 3), f"{time.time() - t0:.0f}s", flush=True)
        json.dump(out, open(D / "results" / f"cal_{name}.json", "w"), indent=1)
