"""lenscal an2: (1) split halves with rotation-fit diff vs uncertainty, (2) capture bootstrap of OPENCV4 fx/cx/cy, (3) board warp (x bow) profile."""
import json, os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lenscal_lib import *
import mrcal
t0 = time.time(); M = meta(); rng = np.random.default_rng(1)
R = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), halves_rotfit={}, boot={}, warp={})


def unc_pts(m):
    P = test_points(); return {k: np.atleast_1d(unc_at(m, p)) for k, p in P.items()}


def rotdiff(m1, m2, use_unc):
    difflen, q0, _, Rt = mrcal.projection_diff([m1, m2], gridn_width=61, use_uncertainties=use_unc)
    gy, gx = difflen.shape; P = test_points(); out = {}
    for k, pts in P.items():
        vals = [difflen[int(round(p[1] / (H - 1) * (gy - 1))), int(round(p[0] / (W - 1) * (gx - 1)))] for p in pts]
        out[k] = float(np.max(vals))
    ang = float(np.degrees(np.linalg.norm(mrcal.r_from_R(Rt[:3, :]))))
    return out, ang


for s in ["A", "B"]:
    for mk in ["OPENCV4", "OPENCV8"]:
        m0, oi, names = base(s, mk)
        caps = caps_of(names, M); uc = np.array(sorted(set(caps)))
        rows = []
        for k in range(20):
            perm = rng.permutation(len(uc)); h1 = uc[perm[: len(uc) // 2]]
            i1 = np.where(np.isin(caps, h1))[0]; i2 = np.where(~np.isin(caps, h1))[0]
            o1 = subset(oi, i1); solve(o1); o2 = subset(oi, i2); solve(o2)
            m1, m2 = model_of(o1), model_of(o2)
            u1, u2 = unc_pts(m1), unc_pts(m2)
            d, ang = rotdiff(m1, m2, False)
            row = dict(ang=ang, dfx=float(o1["intrinsics"][0][0] - o2["intrinsics"][0][0]))
            for key in d:
                row[key] = (d[key], float(np.max(np.sqrt(u1[key] ** 2 + u2[key] ** 2))))
            rows.append(row)
        summ = dict(rot_deg_med=round(float(np.median([r["ang"] for r in rows])), 3),
                    abs_dfx_med_pct=round(float(np.median(np.abs([r["dfx"] for r in rows])) / 1286 * 100), 3))
        for key in ["centre", "r0.5", "r0.8", "r1.0"]:
            dd = np.array([r[key][0] for r in rows]); uu = np.array([r[key][1] for r in rows])
            summ[key] = dict(diff_med=round(float(np.median(dd)), 3), diff_p90=round(float(np.percentile(dd, 90)), 3),
                             unc_med=round(float(np.median(uu)), 3), ratio_med=round(float(np.median(dd / uu)), 2),
                             frac_le_2unc=round(float(np.mean(dd <= 2 * uu)), 2))
        R["halves_rotfit"][f"{s}_{mk}"] = summ; print("halves_rotfit", s, mk, summ, flush=True)

for mk in ["OPENCV4", "OPENCV8"]:
    mA, oiA, nA = base("A", mk); mB, oiB, nB = base("B", mk)
    oA = subset(oiA, np.arange(len(nA))); solve(oA); oB = subset(oiB, np.arange(len(nB))); solve(oB)
    d, ang = rotdiff(model_of(oA), model_of(oB), True)
    R.setdefault("AvsB_rotfit_useunc", {})[mk] = dict(diff=d, rot_deg=ang); print("AvsB useunc", mk, d, ang, flush=True)

# bootstrap by capture, OPENCV4 (k1 k2 p1 p2), 100 reps
for s in ["A", "B", "AB"]:
    m0, oi, names = base(s, "OPENCV4")
    caps = caps_of(names, M); uc = np.array(sorted(set(caps)))
    vals = []
    for b in range(100):
        pick = rng.choice(uc, len(uc), replace=True)
        idx = np.concatenate([np.where(caps == c)[0] for c in pick])
        o = subset(oi, idx); solve(o); vals.append(o["intrinsics"][0][:6].copy())
    vals = np.array(vals); sd = vals.std(0)
    R["boot"][s] = dict(n_caps=len(uc), sd_fx=round(float(sd[0]), 2), sd_fy=round(float(sd[1]), 2), sd_cx=round(float(sd[2]), 2),
                        sd_cy=round(float(sd[3]), 2), sd_fx_pct=round(float(sd[0] / 1286 * 100), 3), mean=[round(float(x), 3) for x in vals.mean(0)])
    print("boot", s, R["boot"][s], flush=True)

# warp profile (x bow only; y unobservable with 2 corner rows)
ws = np.round(np.arange(-0.6, 1.21, 0.15), 3)
for s in ["A", "B", "AB"]:
    for mk in ["OPENCV4", "OPENCV8"]:
        m0, oi, names = base(s, mk)
        prof = []
        for w in ws:
            o = subset(oi, np.arange(len(names)), warp=w / 1000.0, outlier=False); st, rms, cost, n = solve(o)
            prof.append((float(w), rms, cost, float(o["intrinsics"][0][0]), float(o["intrinsics"][0][2]), float(o["intrinsics"][0][3])))
        P = np.array(prof); i = int(np.argmin(P[:, 2]))
        sl = slice(max(0, i - 2), min(len(P), i + 3)); c = np.polyfit(P[sl, 0], P[sl, 2], 2)
        wmin = -c[1] / (2 * c[0]); sig2 = P[i, 1] ** 2
        sd_noise = float(np.sqrt(sig2 / c[0]))  # delta chi2 = 1
        dfx_dw = float(np.polyfit(P[:, 0], P[:, 3], 1)[0])
        R["warp"][f"{s}_{mk}"] = dict(w_mm=[float(x) for x in ws], rms=[round(x, 4) for x in P[:, 1]], fx=[round(x, 2) for x in P[:, 3]],
                                     cx=[round(x, 2) for x in P[:, 4]], cy=[round(x, 2) for x in P[:, 5]],
                                     w_best_mm=round(float(wmin), 3), w_sd_noise_mm=round(sd_noise, 3), dfx_per_mm=round(dfx_dw, 2))
        print("warp", s, mk, R["warp"][f"{s}_{mk}"]["w_best_mm"], R["warp"][f"{s}_{mk}"]["w_sd_noise_mm"], dfx_dw, flush=True)

# warp bootstrap by capture (OPENCV4): 40 reps, parabola on 5 warp values
wq = np.array([-0.3, 0.0, 0.3, 0.6, 0.9])
for s in ["A", "B"]:
    m0, oi, names = base(s, "OPENCV4"); caps = caps_of(names, M); uc = np.array(sorted(set(caps)))
    est = []
    for b in range(40):
        pick = rng.choice(uc, len(uc), replace=True)
        idx = np.concatenate([np.where(caps == c)[0] for c in pick])
        cs = []
        for w in wq:
            o = subset(oi, idx, warp=w / 1000.0, outlier=False); st, rms, cost, n = solve(o); cs.append(cost)
        c = np.polyfit(wq, cs, 2); est.append(-c[1] / (2 * c[0]) if c[0] > 0 else np.nan)
    est = np.array(est)
    R["warp"][f"boot_{s}_OPENCV4"] = dict(n=int(np.isfinite(est).sum()), med=round(float(np.nanmedian(est)), 3), sd=round(float(np.nanstd(est)), 3),
                                          p05=round(float(np.nanpercentile(est, 5)), 3), p95=round(float(np.nanpercentile(est, 95)), 3))
    print("warp boot", s, R["warp"][f"boot_{s}_OPENCV4"], flush=True)
R["sec"] = round(time.time() - t0, 1)
json.dump(R, open(os.path.join(T, "an2.json"), "w"), indent=1, default=float)
print("done", R["sec"])
