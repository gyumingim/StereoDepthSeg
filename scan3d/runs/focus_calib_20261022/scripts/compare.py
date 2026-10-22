"""Compare per-focus calibrations (cal_*.json from calib3.py): K CIs, fx vs dpt, projection-field differences.
usage: compare.py DIR MODEL KIND(frame|block) [extra recs tag:dpt ...]"""
import json, sys
import cv2, numpy as np
D, M, KIND = sys.argv[1], sys.argv[2], sys.argv[3]
recs = [("115405", 3.3286), ("120450", 1.0007), ("120556", 0.3713)] + [(x.split(":")[0], float(x.split(":")[1])) for x in sys.argv[4:]]
W, H = 1920, 1080
C = {}
for r, dpt in recs:
    j = json.load(open(f"{D}/cal_{r}.json"))
    f = j["fits"][M]
    occ = np.array([[c == "#" for c in row] for row in j["occ_rows"]])
    bs = j.get("boot", {}).get(f"{M}_{KIND}", [])
    C[r] = dict(dpt=dpt, K=np.array(f["K"]), d=np.array(f["dist"]), occ=occ, rms=f["rms_px"], n=f["n_views"],
                boot=[(np.array(b["K"]), np.array(b["dist"])) for b in bs], j=j, f=f)

def ci(x):
    return np.percentile(x, [2.5, 97.5])

print(f"model {M}, bootstrap {KIND}")
print("rec     dpt   fx (95% CI)               fy        cx (CI)            cy (CI)           rms  n  cov  tilt50 dist50")
for r, c in C.items():
    K = c["K"]; b = np.array([[k[0, 0], k[1, 1], k[0, 2], k[1, 2]] for k, _ in c["boot"]]) if c["boot"] else None
    s = lambda i: f"[{ci(b[:, i])[0]:.1f},{ci(b[:, i])[1]:.1f}]" if b is not None else ""
    print(f"{r} {c['dpt']:.2f} {K[0,0]:8.2f} {s(0):18s} {K[1,1]:8.2f} {K[0,2]:7.2f} {s(2):16s} {K[1,2]:7.2f} {s(3):16s} "
          f"{c['rms']:.3f} {c['n']} {c['j']['coverage_cells_32x18']:.2f} {c['f']['tilt_deg_pct'][1]:.1f} {c['f']['dist_m_pct'][1]:.2f}")

gx, gy = np.meshgrid(np.linspace(30, W - 30, 64), np.linspace(30, H - 30, 36))
G = np.stack([gx.ravel(), gy.ravel()], 1).astype(np.float64)
cell = (np.clip((G[:, 1] / H * 18).astype(int), 0, 17), np.clip((G[:, 0] / W * 32).astype(int), 0, 31))

def field(KA, dA, KB, dB, mask):
    n = cv2.undistortPoints(G.reshape(-1, 1, 2), KA, dA).reshape(-1, 2)
    x = cv2.projectPoints(np.c_[n, np.ones(len(n))], np.zeros(3), np.zeros(3), KB, dB)[0].reshape(-1, 2)
    P, Q = G[mask], x[mask]
    c0 = np.array([W / 2, H / 2])
    A = np.zeros((2 * len(P), 3)); A[0::2, 0] = P[:, 0] - c0[0]; A[1::2, 0] = P[:, 1] - c0[1]; A[0::2, 1] = 1; A[1::2, 2] = 1
    sol = np.linalg.lstsq(A, (Q - c0).ravel(), rcond=None)[0]
    res = (Q - c0).ravel() - A @ sol
    rr = np.hypot(res[0::2], res[1::2])
    return sol[0] - 1, sol[1:], float(np.sqrt(np.mean(rr ** 2))), float(rr.max())

def distmag(K, d, mask):
    n = cv2.undistortPoints(G.reshape(-1, 1, 2), K, d).reshape(-1, 2)
    p = n * [K[0, 0], K[1, 1]] + [K[0, 2], K[1, 2]]
    return np.hypot(*(G - p).T)[mask]

names = list(C)
print("\ndistortion |distorted - pinhole| px (own K), own covered region: median / p95 / max; corners of full image")
for r in names:
    c = C[r]; m = c["occ"][cell]
    dm = distmag(c["K"], c["d"], m); dfull = distmag(c["K"], c["d"], np.ones(len(G), bool))
    print(f"  {r}: {np.median(dm):.1f} / {np.percentile(dm, 95):.1f} / {dm.max():.1f}   full-image max {dfull.max():.1f}")

rng = np.random.default_rng(1)
print("\npair: same ray -> pixel in B vs A. scale % (CI), shift px (CI), non-similarity residual rms/max px (CI of rms)"
      " [region = covered in both]")
for i in range(len(names)):
    for k in range(i + 1, len(names)):
        a, b = C[names[i]], C[names[k]]
        mask = a["occ"][cell] & b["occ"][cell]
        s, t, rr, rm = field(a["K"], a["d"], b["K"], b["d"], mask)
        sb, tb, rb = [], [], []
        nb = min(len(a["boot"]), len(b["boot"]))
        for q in range(nb):
            ka, da = a["boot"][rng.integers(nb)]; kb, db = b["boot"][rng.integers(nb)]
            s_, t_, r_, _ = field(ka, da, kb, db, mask)
            sb.append(s_); tb.append(t_); rb.append(r_)
        sb, tb, rb = np.array(sb) * 100, np.array(tb), np.array(rb)
        dd = b["dpt"] - a["dpt"]
        print(f"  {names[i]}({a['dpt']:.2f}) -> {names[k]}({b['dpt']:.2f}) [{mask.mean():.2f} of grid]: scale {s*100:+.2f} % "
              f"[{ci(sb)[0]:+.2f},{ci(sb)[1]:+.2f}] = {s*100/dd:+.2f} %/dpt; shift ({t[0]:+.1f},{t[1]:+.1f}) "
              f"x[{ci(tb[:,0])[0]:+.1f},{ci(tb[:,0])[1]:+.1f}] y[{ci(tb[:,1])[0]:+.1f},{ci(tb[:,1])[1]:+.1f}]; "
              f"resid {rr:.2f}/{rm:.2f} [{ci(rb)[0]:.2f},{ci(rb)[1]:.2f}]" if nb else
              f"  {names[i]} -> {names[k]}: scale {s*100:+.2f} % shift {t.round(1)} resid {rr:.2f}/{rm:.2f}")
