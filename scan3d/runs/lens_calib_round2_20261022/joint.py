"""Joint fit across recordings: one shared distortion, per-group fx, fy, cx, cy, per-view pose (scipy least_squares,
sparse finite-difference Jacobian). Views = the same per-recording selection as calib_r2.py (spread, n 160).

usage: joint.py --groups focus|rec [--model k5|rational] [--boot 40] [--exclude 173113] [--workers 16]
  groups=focus: per-focus intrinsics (main joint fit); groups=rec: per-recording intrinsics (repeatability with shared distortion)
"""
import argparse, json, sys, time
from multiprocessing import Pool
import numpy as np, cv2

from common import D, S3, SIZE, FOCUS, REC2F, ROUND, load_views, spread
sys.path[:0] = [str(S3), str(S3 / "imu_learn")]
import lens_calib as lc

ap = argparse.ArgumentParser()
ap.add_argument("--groups", default="focus")
ap.add_argument("--model", default="k5")
ap.add_argument("--boot", type=int, default=40)
ap.add_argument("--exclude", default="173113")
ap.add_argument("--workers", type=int, default=16)
ap.add_argument("--n", type=int, default=160)
ap.add_argument("--tag", default="")
ap.add_argument("--k1-per-group", action="store_true", help="k1 per group too (judge protocol variant)")
a = ap.parse_args()
cv2.setNumThreads(1)
ND = 5 if a.model == "k5" else 8
NK = 5 if a.k1_per_group else 4
excl = set(a.exclude.split(",")) if a.exclude else set()
recs = [r for f in ("0.37", "1.00", "2.22", "3.33") for r in FOCUS[f] if r not in excl]
views = []
for r in recs:
    v, _ = load_views(r)
    pick, _ = spread(v, a.n)
    views += [v[k] for k in pick]
gname = (lambda v: REC2F[v["rec"]]) if a.groups == "focus" else (lambda v: v["rec"])
gnames = list(dict.fromkeys(gname(v) for v in views))
gidx = np.array([gnames.index(gname(v)) for v in views])
G, V = len(gnames), len(views)


def project(dist, K4, rv, tv, X):
    """dist (ND,), K4 (N,4), rv/tv (N,3), X (N,3) -> (N,2) OpenCV model."""
    th = np.linalg.norm(rv, axis=1, keepdims=True)
    k = rv / np.maximum(th, 1e-12)
    c, s = np.cos(th), np.sin(th)
    Xc = X * c + np.cross(k, X) * s + k * np.sum(k * X, 1, keepdims=True) * (1 - c) + tv
    x, y = Xc[:, 0] / Xc[:, 2], Xc[:, 1] / Xc[:, 2]
    r2 = x * x + y * y
    if ND == 5:
        k1, k2, p1, p2, k3 = dist
        if K4.shape[1] == 5:
            k1 = K4[:, 4]
        rad = 1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3
    else:
        k1, k2, p1, p2, k3, k4, k5, k6 = dist
        if K4.shape[1] == 5:
            k1 = K4[:, 4]
        rad = (1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3) / (1 + k4 * r2 + k5 * r2 ** 2 + k6 * r2 ** 3)
    xd = x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return np.stack([K4[:, 0] * xd + K4[:, 2], K4[:, 1] * yd + K4[:, 3]], 1)


def solve(vidx, init=None):
    """Fit on views vidx (list of indices into `views`, repeats allowed). Returns dict."""
    vs = [views[i] for i in vidx]
    gi = gidx[vidx]
    nv = len(vs)
    X = np.concatenate([v["obj"].astype(np.float64) for v in vs])
    U = np.concatenate([v["img"].reshape(-1, 2).astype(np.float64) for v in vs])
    pv = np.concatenate([np.full(len(v["obj"]), j) for j, v in enumerate(vs)])
    # init: per-group cv2 fit (k1k2p_k3) for K, shared dist = mean, poses by PnP
    if init is None:
        K4 = np.zeros((G, NK)); dl = []
        for g in range(G):
            tg = [(v["obj"], v["img"], 0, 0, 0) for v, q in zip(vs, gi) if q == g]
            _, K, d, _, _ = lc.fit(tg, SIZE, lc.MODELS["k1k2p_k3"] if ND == 5 else lc.MODELS["rational"])
            K4[g, :4] = [K[0, 0], K[1, 1], K[0, 2], K[1, 2]]; dl.append(d[:ND])
            if NK == 5:
                K4[g, 4] = d[0]
        dist0 = np.median(dl, 0)
    else:
        K4, dist0 = init
    P0 = np.zeros((nv, 6))
    for j, v in enumerate(vs):
        g = gi[j]
        Km = np.array([[K4[g, 0], 0, K4[g, 2]], [0, K4[g, 1], K4[g, 3]], [0, 0, 1]])
        dfull = np.zeros(8 if ND == 8 else 5); dfull[:ND] = dist0
        ok, rv, tv = cv2.solvePnP(v["obj"], v["img"], Km, dfull, flags=cv2.SOLVEPNP_IPPE)
        ok, rv, tv = cv2.solvePnP(v["obj"], v["img"], Km, dfull, rv, tv, True, cv2.SOLVEPNP_ITERATIVE)
        P0[j] = np.r_[rv.ravel(), tv.ravel()]
    # Levenberg-Marquardt with Schur complement on the per-view poses (bundle-adjustment style),
    # central-difference Jacobian (all views / groups perturbed at once since each residual sees one view and one group)
    m = ND + NK * G
    gpv = gi[pv]

    def resid(dist, K4_, P):
        return project(dist, K4_[gpv], P[pv, :3], P[pv, 3:], X) - U

    def jac(dist, K4_, P):
        Jp = np.zeros((len(X), 2, 6)); Jg = np.zeros((len(X), 2, m))
        for c in range(6):
            e = 1e-6
            Pa, Pb = P.copy(), P.copy(); Pa[:, c] += e; Pb[:, c] -= e
            Jp[:, :, c] = (resid(dist, K4_, Pa) - resid(dist, K4_, Pb)) / (2 * e)
        for c in range(ND):
            e = 1e-6
            da, db = dist.copy(), dist.copy(); da[c] += e; db[c] -= e
            Jg[:, :, c] = (resid(da, K4_, P) - resid(db, K4_, P)) / (2 * e)
        for c in range(NK):
            e = 1e-3 if c < 4 else 1e-6
            Ka, Kb = K4_.copy(), K4_.copy(); Ka[:, c] += e; Kb[:, c] -= e
            dcol = (resid(dist, Ka, P) - resid(dist, Kb, P)) / (2 * e)
            Jg[np.arange(len(X)), :, ND + NK * gpv + c] = dcol
        return Jp, Jg

    dist, K4c, P = dist0.astype(np.float64).copy(), K4.astype(np.float64).copy(), P0.copy()
    r = resid(dist, K4c, P); cost = float(np.sum(r ** 2))
    lam, it = 1e-3, 0
    for it in range(100):
        Jp, Jg = jac(dist, K4c, P)
        A = np.zeros((nv, 6, 6)); np.add.at(A, pv, np.einsum("nki,nkj->nij", Jp, Jp))
        B = np.zeros((nv, 6, m)); np.add.at(B, pv, np.einsum("nki,nkj->nij", Jp, Jg))
        gp = np.zeros((nv, 6)); np.add.at(gp, pv, np.einsum("nki,nk->ni", Jp, r))
        C = np.einsum("nki,nkj->ij", Jg, Jg); gg = np.einsum("nki,nk->i", Jg, r)
        improved = False
        for _ in range(12):
            Ad = A + lam * np.einsum("nii->ni", A)[:, :, None] * np.eye(6)
            Cd = C + lam * np.diag(np.diag(C)) + 1e-12 * np.eye(m) * np.diag(C).max()
            Ai = np.linalg.inv(Ad)
            AiB = Ai @ B
            S = Cd - np.einsum("nim,nik->mk", B, AiB)
            rhs = -gg + np.einsum("nim,ni->m", B, np.einsum("nij,nj->ni", Ai, gp))
            dg = np.linalg.solve(S, rhs)
            dp = -np.einsum("nij,nj->ni", Ai, gp + B @ dg)
            dist_n = dist + dg[:ND]; K4_n = K4c + dg[ND:].reshape(G, NK); P_n = P + dp
            r_n = resid(dist_n, K4_n, P_n); cost_n = float(np.sum(r_n ** 2))
            if cost_n < cost:
                improved = True
                rel = (cost - cost_n) / cost
                dist, K4c, P, r, cost = dist_n, K4_n, P_n, r_n, cost_n
                lam = max(lam / 3, 1e-9)
                break
            lam *= 5
        if not improved or rel < 1e-11:
            break
    per = np.sqrt(np.bincount(pv, np.sum(r ** 2, 1)) / np.bincount(pv))
    grms = np.sqrt(np.bincount(gpv, np.sum(r ** 2, 1), minlength=G) / np.maximum(np.bincount(gpv, minlength=G), 1))
    return dict(dist=dist, K4=K4c, rms=float(np.sqrt(cost / len(X))), per=per, nfev=it + 1, status=int(improved),
                grms=grms.tolist())


t0 = time.time()
base = list(range(V))
s1 = solve(base)
keep = s1["per"] < 2.5 * np.median(s1["per"])          # same rejection rule as lens_calib
base = [i for i, k in zip(base, keep) if k]
s = solve(base, init=(s1["K4"], s1["dist"]))
print(f"joint {a.groups} {a.model}: views {len(base)} (rejected {(~keep).sum()}), rms {s['rms']:.3f}, nfev {s['nfev']}, "
      f"{time.time() - t0:.0f}s", flush=True)
for g, n in enumerate(gnames):
    print(f"  {n}: fx {s['K4'][g,0]:.2f} fy {s['K4'][g,1]:.2f} cx {s['K4'][g,2]:.2f} cy {s['K4'][g,3]:.2f} rms {s['grms'][g]:.3f}",
          flush=True)
print("  dist", np.round(s["dist"], 5), flush=True)
out = dict(groups=a.groups, model=a.model, group_names=gnames, recs=recs, excluded=sorted(excl), n_views=len(base),
           n_rejected=int((~keep).sum()), n_views_per_group={n: int(sum(gidx[i] == g for i in base)) for g, n in enumerate(gnames)},
           K4={n: s["K4"][g].tolist() for g, n in enumerate(gnames)}, dist=s["dist"].tolist(), rms_px=s["rms"],
           rms_px_group={n: s["grms"][g] for g, n in enumerate(gnames)}, lm_iters=s["nfev"],
           made=time.strftime("%Y-%m-%d %H:%M:%S"), script="runs/lens_calib_round2_20261022/joint.py",
           views="per recording: common.load_views (w<0.05 raw gyro, no stand, sharp>=0.5 med, 0.1 s thinning) + spread n=%d" % a.n)
name = f"joint_{a.groups}_{a.model}{'_k1g' if a.k1_per_group else ''}{a.tag}"
json.dump(out, open(D / "results" / f"{name}.json", "w"), indent=1)

blocks = np.array([views[i]["block"] for i in base])
recb = np.array([views[i]["rec"] for i in base])


def boot_one(seed):
    rng = np.random.default_rng(seed)
    idx = []
    for r in recs:                                         # resample 1 s blocks within each recording
        ub = sorted(set(blocks[recb == r]))
        for j in rng.integers(0, len(ub), len(ub)):
            idx += [base[k] for k in np.nonzero(blocks == ub[j])[0]]
    sb = solve(idx, init=(s["K4"], s["dist"]))
    return sb["K4"].tolist(), sb["dist"].tolist(), sb["rms"]


if a.boot:
    with Pool(a.workers) as pool:
        bs = pool.map(boot_one, range(a.boot))
    K4b = np.array([b[0] for b in bs])
    out["boot"] = dict(K4=[b[0] for b in bs], dist=[b[1] for b in bs], rms=[b[2] for b in bs],
                       sd={n: K4b[:, g].std(0, ddof=1).tolist() for g, n in enumerate(gnames)},
                       ci95={n: np.percentile(K4b[:, g], [2.5, 97.5], axis=0).tolist() for g, n in enumerate(gnames)})
    for g, n in enumerate(gnames):
        print(f"  boot sd {n}: {np.round(K4b[:, g].std(0, ddof=1), 3)}", flush=True)
    json.dump(out, open(D / "results" / f"{name}.json", "w"), indent=1)
print(f"done {time.time() - t0:.0f}s")
