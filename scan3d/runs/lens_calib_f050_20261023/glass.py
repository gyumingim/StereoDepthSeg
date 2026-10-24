"""Is the laptop cover glass the cause of the tilt-dependent fx? (2026-10-24)

Model: every board point lies on the back face of a flat glass slab (index N = 1.5, thickness t, same for all poses,
parallel to the board). Board frame: board plane z = 0, camera on the z < 0 side, slab front face z = -t.
For camera centre C and board point P the ray C -> Q (Q on the front face) refracts (Snell) and reaches P;
Q is found per point by bisection on the horizontal offset. Q is then projected with K + k1 k2 (no tangential, no k3 —
the saved k12_notan model). Free: fx fy cx cy k1 k2, per-view rvec tvec, t (0..3 mm). scipy least_squares, sparse
finite-difference Jacobian, loss soft_l1 f_scale 1 px (≈ linear at 0.15 px residuals; calib.py used plain LSQ).
Board units: metres from the board json square size (5x3 60.3 mm, 6x4 47.1 mm, both spec ESTIMATES) → t scales with them.
  ../venv_ffs/bin/python runs/lens_calib_f050_20261023/glass.py [--halves]
→ results/glass.json
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import calib  # noqa: E402

N_GLASS = 1.5
T_MAX = 0.003
NI = 7   # fx fy cx cy k1 k2 t


def rodrigues(r):
    th = np.linalg.norm(r, axis=1, keepdims=True)
    k = r / np.where(th > 0, th, 1)
    K = np.zeros((len(r), 3, 3))
    K[:, 0, 1], K[:, 0, 2], K[:, 1, 2] = -k[:, 2], k[:, 1], -k[:, 0]
    K[:, 1, 0], K[:, 2, 0], K[:, 2, 1] = k[:, 2], -k[:, 1], k[:, 0]
    s, c = np.sin(th)[:, :, None], np.cos(th)[:, :, None]
    return np.eye(3)[None] + s * K + (1 - c) * K @ K


def refract_points(P, C, t):
    """P (M,3) board points (z = 0), C (M,3) camera centres in board frame (z < -t). → Q (M,3) on z = -t."""
    if t <= 0:
        return P.copy()
    d = P[:, :2] - C[:, :2]
    D = np.linalg.norm(d, axis=1)
    u = d / np.where(D > 0, D, 1)[:, None]
    h = -t - C[:, 2]
    lo, hi = np.zeros_like(D), D.copy()
    for _ in range(60):
        x = 0.5 * (lo + hi)
        f = x / np.hypot(x, h) - N_GLASS * (D - x) / np.hypot(D - x, t)
        hi = np.where(f > 0, x, hi)
        lo = np.where(f > 0, lo, x)
    x = 0.5 * (lo + hi)
    Q = np.empty_like(P)
    Q[:, :2] = C[:, :2] + u * x[:, None]
    Q[:, 2] = -t
    return Q


class Problem:
    def __init__(self, views, fix_t=None):
        self.views = views
        self.nv = len(views)
        self.obj = np.concatenate([v["obj"].reshape(-1, 3).astype(float) for v in views])
        self.img = np.concatenate([v["img"].reshape(-1, 2).astype(float) for v in views])
        self.vid = np.concatenate([np.full(len(v["obj"]), k) for k, v in enumerate(views)])
        self.fix_t = fix_t

    def unpack(self, p):
        intr = p[:NI]
        if self.fix_t is not None:
            intr = intr.copy()
            intr[6] = self.fix_t
        pose = p[NI:].reshape(self.nv, 6)
        return intr, pose

    def project(self, p):
        (fx, fy, cx, cy, k1, k2, t), pose = self.unpack(p)
        R = rodrigues(pose[:, :3])[self.vid]
        T = pose[:, 3:][self.vid]
        C = -np.einsum("mji,mj->mi", R, T)                # camera centre in board frame = -R^T T
        Q = refract_points(self.obj, C, t)
        X = np.einsum("mij,mj->mi", R, Q) + T
        x, y = X[:, 0] / X[:, 2], X[:, 1] / X[:, 2]
        r2 = x * x + y * y
        g = 1 + k1 * r2 + k2 * r2 * r2
        return np.c_[fx * x * g + cx, fy * y * g + cy]

    def resid(self, p):
        return (self.project(p) - self.img).ravel()

    def sparsity(self):
        m = len(self.obj) * 2
        A = lil_matrix((m, NI + 6 * self.nv), dtype=int)
        A[:, :NI] = 1
        for i, k in enumerate(self.vid):
            A[2 * i:2 * i + 2, NI + 6 * k:NI + 6 * k + 6] = 1
        return A


def solve(views, init, t0=0.0005, fix_t=None):
    """init = (K, d, rv, tv) from a cv2 k12_notan fit of the same views."""
    K, d, rv, tv = init
    pr = Problem(views, fix_t)
    p0 = np.r_[K[0, 0], K[1, 1], K[0, 2], K[1, 2], d[0], d[1], t0 if fix_t is None else 0.0,
               np.hstack([np.r_[np.ravel(a), np.ravel(b)] for a, b in zip(rv, tv)])]
    lb = np.full(len(p0), -np.inf)
    ub = np.full(len(p0), np.inf)
    lb[6], ub[6] = (0.0, T_MAX) if fix_t is None else (-1e-9, 1e-9)
    r = least_squares(pr.resid, p0, jac_sparsity=pr.sparsity(), bounds=(lb, ub), loss="soft_l1", f_scale=1.0,
                      x_scale="jac", method="trf", max_nfev=200)
    res = pr.resid(r.x).reshape(-1, 2)
    intr, pose = pr.unpack(r.x)
    per = np.array([np.sqrt(np.mean(np.sum(res[pr.vid == k] ** 2, 1))) for k in range(pr.nv)])
    return dict(fx=float(intr[0]), fy=float(intr[1]), cx=float(intr[2]), cy=float(intr[3]), k1=float(intr[4]),
                k2=float(intr[5]), t_m=float(intr[6]), rms=float(np.sqrt(np.mean(np.sum(res ** 2, 1)))), per_view=per,
                pose=pose, nfev=int(r.nfev), status=int(r.status))


def cv_init(views):
    rms, K, d, rv, tv = calib.fit(views, "k12_notan")
    return K, d, rv, tv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--halves", action="store_true")
    ap.add_argument("--tmax_mm", type=float, default=3.0, help="upper bound of t (sensitivity: 20 mm)")
    a = ap.parse_args()
    global T_MAX
    T_MAX = a.tmax_mm / 1000
    t0 = time.time()
    views, clusters, members = calib.build()
    init = cv_init(views)
    no = solve(views, init, fix_t=0.0)
    gl = solve(views, init)
    print(f"no glass: fx {no['fx']:.2f} fy {no['fy']:.2f} cx {no['cx']:.2f} cy {no['cy']:.2f} k1 {no['k1']:.4f} k2 {no['k2']:.4f} rms {no['rms']:.4f}")
    print(f"glass   : fx {gl['fx']:.2f} fy {gl['fy']:.2f} cx {gl['cx']:.2f} cy {gl['cy']:.2f} k1 {gl['k1']:.4f} k2 {gl['k2']:.4f} rms {gl['rms']:.4f} "
          f"t {gl['t_m'] * 1000:.3f} mm ({time.time() - t0:.0f} s)")
    # tilt per cluster from the no-glass poses
    tilt = {}
    for v, pz in zip(views, no["pose"]):
        tilt.setdefault(v["cluster"], []).append(calib.tilt_dist(pz[:3], pz[3:])[0])
    tilt = {c: float(np.mean(x)) for c, x in tilt.items()}
    front = {c for c in clusters if tilt[c] < 25}
    groups = {"tilt 25-45": [c for c in clusters if 25 <= tilt[c] < 45], "tilt >=45": [c for c in clusters if tilt[c] >= 45]}
    out = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), n_glass=N_GLASS, t_bound_m=[0, T_MAX], loss="soft_l1 f_scale 1 px",
               square_mm={"5x3": 60.3, "6x4": 47.1, "note": "spec estimates; t scales with them"},
               n_views=len(views), n_clusters=len(clusters),
               full=dict(no_glass={k: v for k, v in no.items() if k not in ("per_view", "pose")},
                         glass={k: v for k, v in gl.items() if k not in ("per_view", "pose")}),
               t_over_square_5x3=gl["t_m"] / 0.0603, groups={})
    for g, cs in groups.items():
        vs = [v for v in views if v["cluster"] in front or v["cluster"] in cs]
        ini = cv_init(vs)
        r0 = solve(vs, ini, fix_t=0.0)
        r1 = solve(vs, ini, fix_t=gl["t_m"])
        r2 = solve(vs, ini)
        out["groups"][g] = dict(n=len(cs), no_glass=dict(fx=r0["fx"], cx=r0["cx"], cy=r0["cy"], rms=r0["rms"]),
                                glass_t_global=dict(fx=r1["fx"], cx=r1["cx"], cy=r1["cy"], rms=r1["rms"]),
                                glass_t_free=dict(fx=r2["fx"], cx=r2["cx"], cy=r2["cy"], rms=r2["rms"], t_mm=r2["t_m"] * 1000))
        print(f"{g}: no glass fx {r0['fx']:.1f} | glass t global fx {r1['fx']:.1f} | glass t free fx {r2['fx']:.1f} t {r2['t_m'] * 1000:.3f} mm")
    if a.halves:
        # same splits as calib.py (seed 0: even/odd, first/second, 30 random, 30 stratified by tilt > 25)
        tilted = [c for c in clusters if tilt[c] > 25]
        ab = [("even_odd", set(clusters[0::2])), ("first_second", set(clusters[:len(clusters) // 2]))]
        rng = np.random.default_rng(0)
        for k in range(30):
            ab.append((f"rand{k}", set(rng.permutation(clusters)[:len(clusters) // 2].tolist())))
        for k in range(30):
            S = set()
            for grp in (tilted, [c for c in clusters if c not in tilted]):
                S |= set(rng.permutation(grp)[:len(grp) // 2 + (k % 2) * (len(grp) % 2)].tolist())
            ab.append((f"strat{k}", S))
        hs = []
        for nm, S in ab:
            A, B = calib.split_by(views, S)
            ra, rb = solve(A, cv_init(A)), solve(B, cv_init(B))
            h = dict(name=nm, dfx_pct=(rb["fx"] - ra["fx"]) / ra["fx"] * 100, dcx=rb["cx"] - ra["cx"], dcy=rb["cy"] - ra["cy"],
                     tA_mm=ra["t_m"] * 1000, tB_mm=rb["t_m"] * 1000)
            hs.append(h)
            print(nm, {k: round(x, 3) if isinstance(x, float) else x for k, x in h.items()}, flush=True)
        f = [abs(h["dfx_pct"]) <= 0.3 for h in hs]
        q = [abs(h["dcx"]) <= 3 and abs(h["dcy"]) <= 3 for h in hs]
        out["halves"] = hs
        out["halves_pass"] = dict(fx=float(np.mean(f)), pp=float(np.mean(q)), both=float(np.mean(np.array(f) & np.array(q))),
                                  p50_dfx=float(np.median([abs(h["dfx_pct"]) for h in hs])),
                                  p90_dfx=float(np.quantile([abs(h["dfx_pct"]) for h in hs], 0.9)))
        print("halves pass", out["halves_pass"])
    json.dump(out, open(HERE / "results" / (("glass_halves" if a.halves else "glass") + ("" if a.tmax_mm == 3.0 else f"_tmax{a.tmax_mm:g}") + ".json"), "w"), indent=1)


if __name__ == "__main__":
    main()
