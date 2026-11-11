#!/usr/bin/env python3
"""Metric scale of the scanner from a laptop screen showing a full-screen solid red page (redscreen 11-11; method = runs/redscreen_20261111
measure_red.py estimator B, made faster: one image read per keyframe, vectorised rectangle bundle, small bootstrap).

  redscreen_scale.py KF_DIR --poses BA/poses_ba.jsonl.gz --lens LENS(.json|gt.npz) --out DIR [--red-size 302.0x188.75] [--boot 100]

KF_DIR = reduced recording (frames_kf/NNNNN.jpg, 1920x1080, the scanner's own keyframes).
Per keyframe: HSV red mask -> largest blob with all 4 corners >= 12 px inside the image -> rough quad -> on each edge only the MIDDLE part
(15 % of the edge length excluded at each end: the panel corners are rounded) -> sub-pixel crossings of the redness profile
r = R - max(G,B) at THR (0.3 / 0.5 / 0.7) between the inside and outside levels -> undistort (lens) -> straight-line TLS fit.
Rectangle bundle: plane pose (6) + W, H; residual = px distance of every edge point to the projected rectangle edge line (Huber).
s = mean(W / W_ref, H / H_ref) at THR 0.5.  Scaled poses = positions / s (rotations unchanged) -> OUT/poses_redscreen.jsonl.gz.
Uncertainty: sigma = sqrt(boot_sd^2 + (thr spread / 2)^2 + ref^2) with ref = 0.2 % (reference size 가정).
Gates (any fail -> exit 3, no scaled poses written, redscreen_scale.json status 'failed: ...'):
  >= --min-views views with >= 3 edges, per-view line rms median <= 0.6 px, rectangle corner angle within 1 deg of 90 (free-lines check),
  measured W/H within 3 % of the reference W/H, bundle median |residual| <= 2 px, thr spread (0.3 vs 0.7) <= 1.5 %, 0.8 < s < 1.25.
"""
import argparse, glob, gzip, json, sys, time
from pathlib import Path
import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

EXCL = 0.15
THRS = (0.3, 0.5, 0.7)
EDGES = {"top": (0, 1), "right": (1, 2), "bottom": (3, 2), "left": (0, 3)}
ENAMES = list(EDGES)
ECORN = {"top": ((-1, 1), (1, 1)), "bottom": ((-1, -1), (1, -1)), "left": ((-1, 1), (-1, -1)), "right": ((1, 1), (1, -1))}


def load_lens(p):
    p = Path(p)
    if p.suffix == ".npz":
        g = np.load(p); return np.array(g["K"], float), np.array(g["dist"], float).ravel()
    L = json.loads(p.read_text()); return np.array(L["K"], float), np.array(L["dist"], float).ravel()


def red_quad(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    m = (((h <= 20) | (h >= 172)) & (s >= 170) & (v >= 200)).astype(np.uint8)   # screen red: R clipped, G 60-100, B ~0
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    if n < 2:
        return None
    k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    if st[k, cv2.CC_STAT_AREA] < 30000:
        return None
    cs, _ = cv2.findContours((lab == k).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    hull = cv2.convexHull(max(cs, key=cv2.contourArea))
    per = cv2.arcLength(hull, True)
    for eps in (0.01, 0.015, 0.02, 0.03, 0.04):
        q = cv2.approxPolyDP(hull, eps * per, True).reshape(-1, 2).astype(float)
        if len(q) == 4:
            break
    else:
        return None
    H, W = img.shape[:2]
    if q[:, 0].min() < 12 or q[:, 1].min() < 12 or q[:, 0].max() > W - 13 or q[:, 1].max() > H - 13:
        return None
    c = q.mean(0)
    q = q[np.argsort(np.arctan2(q[:, 1] - c[1], q[:, 0] - c[0]))]
    q = np.roll(q, -int(np.argmin(q[:, 0] + q[:, 1])), 0)              # TL, TR, BR, BL
    if not (q[1, 0] > q[0, 0] and q[3, 1] > q[0, 1]):
        return None
    return q


def edge_points(score, q, thrs, step_px=3.0, half=9.0, ds=0.25):
    c = q.mean(0); s = np.arange(-half, half + 1e-9, ds)
    out = {t: {} for t in thrs}
    for name, (i, j) in EDGES.items():
        a, b = q[i], q[j]; L = np.linalg.norm(b - a); t_ = (b - a) / L
        n = np.array([-t_[1], t_[0]])
        if np.dot(n, (a + b) / 2 - c) < 0:
            n = -n
        fs = np.arange(EXCL * L, (1 - EXCL) * L + 1e-9, step_px)
        P = a[None] + fs[:, None] * t_[None]
        X = (P[:, None, 0] + s[None] * n[0]).astype(np.float32); Y = (P[:, None, 1] + s[None] * n[1]).astype(np.float32)
        prof = cv2.remap(score, X, Y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        hi = np.median(prof[:, s <= -half + 3.5], 1); lo = np.median(prof[:, s >= half - 3.5], 1)
        for thr in thrs:
            pts = []
            for k in np.flatnonzero(hi - lo >= 50):
                pr = prof[k]; lev = lo[k] + thr * (hi[k] - lo[k]); above = pr >= lev
                cr = np.flatnonzero(above[:-1] & ~above[1:])
                if len(cr) != 1:
                    continue
                m = cr[0]; f = (pr[m] - lev) / (pr[m] - pr[m + 1])
                pts.append(P[k] + (s[m] + f * ds) * n)
            if len(pts) >= 15:
                out[thr][name] = np.array(pts)
    return out


def fit_line(xn, fx):
    keep = np.ones(len(xn), bool)
    for _ in range(4):
        m = xn[keep].mean(0); _, _, vt = np.linalg.svd(xn[keep] - m); nrm = vt[1]
        l = np.array([nrm[0], nrm[1], -nrm @ m]); r = (xn @ l[:2] + l[2]) * fx
        sd = np.sqrt(np.mean(r[keep] ** 2)); nk = np.abs(r) < max(3 * sd, 0.3)
        if (nk == keep).all():
            break
        keep = nk
    return l, keep, float(np.sqrt(np.mean(r[keep] ** 2)))


class Pack:
    """all edge points of a view set, flat arrays for the vectorised residual"""
    def __init__(self, views, fx):
        self.fx = fx
        self.Tcw = np.array([np.linalg.inv(v["T"]) for v in views])
        vi, ei, xs = [], [], []
        for k, v in enumerate(views):
            for name, e in v["edges"].items():
                vi.append(np.full(len(e["xn"]), k)); ei.append(np.full(len(e["xn"]), ENAMES.index(name))); xs.append(e["xn"])
        self.vi, self.ei, self.xn = np.concatenate(vi), np.concatenate(ei), np.concatenate(xs)


def resid(x, pk):
    R = Rot.from_rotvec(x[:3]).as_matrix(); c, W, H = x[3:6], x[6], x[7]
    P1 = np.array([c + R @ np.array([ECORN[n][0][0] * W / 2, ECORN[n][0][1] * H / 2, 0]) for n in ENAMES])
    P2 = np.array([c + R @ np.array([ECORN[n][1][0] * W / 2, ECORN[n][1][1] * H / 2, 0]) for n in ENAMES])
    Rc, tc = pk.Tcw[:, :3, :3], pk.Tcw[:, :3, 3]
    p1 = np.einsum("vij,ej->vei", Rc, P1) + tc[:, None]; p2 = np.einsum("vij,ej->vei", Rc, P2) + tc[:, None]
    h1 = np.concatenate([p1[..., :2] / p1[..., 2:], np.ones(p1.shape[:2] + (1,))], -1)
    h2 = np.concatenate([p2[..., :2] / p2[..., 2:], np.ones(p2.shape[:2] + (1,))], -1)
    l = np.cross(h1, h2); l /= np.linalg.norm(l[..., :2], axis=-1, keepdims=True)
    L = l[pk.vi, pk.ei]
    return (np.einsum("ni,ni->n", pk.xn, L[:, :2]) + L[:, 2]) * pk.fx


def line3d(views, name):
    N, b = [], []
    for v in views:
        if name in v["edges"]:
            nw = v["T"][:3, :3] @ v["edges"][name]["l"]; nw /= np.linalg.norm(nw); N.append(nw); b.append(nw @ v["T"][:3, 3])
    N, b = np.array(N), np.array(b); d = np.linalg.svd(N)[2][2]
    return np.linalg.lstsq(np.vstack([N, d[None] * 10]), np.concatenate([b, [0]]), rcond=None)[0], d


def closest(p1, d1, p2, d2):
    w = p1 - p2; a, bb, c, dd, e = d1 @ d1, d1 @ d2, d2 @ d2, d1 @ w, d2 @ w; den = a * c - bb * bb
    return (p1 + (bb * e - c * dd) / den * d1 + p2 + (a * e - bb * dd) / den * d2) / 2


def free_lines(views):
    L = {k: line3d(views, k) for k in EDGES}
    C = np.array([closest(*L["top"], *L["left"]), closest(*L["top"], *L["right"]), closest(*L["bottom"], *L["right"]), closest(*L["bottom"], *L["left"])])
    e1, e2 = C[1] - C[0], C[3] - C[0]
    ang = float(np.degrees(np.arccos(abs(e1 @ e2) / np.linalg.norm(e1) / np.linalg.norm(e2))))
    W = (np.linalg.norm(C[1] - C[0]) + np.linalg.norm(C[2] - C[3])) / 2; H = (np.linalg.norm(C[3] - C[0]) + np.linalg.norm(C[2] - C[1])) / 2
    ex = (C[1] - C[0]) + (C[2] - C[3]); ex /= np.linalg.norm(ex)
    ey = (C[0] - C[3]) + (C[1] - C[2]); ey -= (ey @ ex) * ex; ey /= np.linalg.norm(ey)
    x0 = np.concatenate([Rot.from_matrix(np.stack([ex, ey, np.cross(ex, ey)], 1)).as_rotvec(), C.mean(0), [W, H]])
    return dict(W=float(W), H=float(H), angle_deg=ang), x0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("kf_dir"); ap.add_argument("--poses", required=True); ap.add_argument("--lens", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--red-size", default="302.0x188.75", help="red area W x H mm (default: 14in Lenovo 83K9 panel, 1920x1200 x 0.15729 mm EDID pitch, 가정)")
    ap.add_argument("--boot", type=int, default=100); ap.add_argument("--min-views", type=int, default=20)
    a = ap.parse_args()
    t0 = time.time(); out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    W_ref, H_ref = (float(v) / 1000 for v in a.red_size.lower().split("x"))
    K, dist = load_lens(a.lens); fx = K[0, 0]
    rows = [json.loads(l) for l in gzip.open(a.poses, "rt")]
    poses = {int(d["frame"]): np.array(d["T_world_cam0"], float).reshape(4, 4) for d in rows if d.get("T_world_cam0") is not None}
    imgs = Path(a.kf_dir) / "frames_kf"
    frames = sorted(f for f in (int(Path(p).stem) for p in glob.glob(str(imgs / "*.jpg"))) if f in poses)
    V = {t: [] for t in THRS}
    n_mask = n_quad = 0
    for fr in frames:
        img = cv2.imread(str(imgs / f"{fr:05d}.jpg"))
        q = red_quad(img)
        if q is None:
            continue
        n_quad += 1
        b = img.astype(np.float32); score = b[..., 2] - np.maximum(b[..., 1], b[..., 0])
        ep = edge_points(score, q, THRS)
        for thr in THRS:
            if len(ep[thr]) < 3:
                continue
            v = dict(frame=fr, T=poses[fr], edges={}); ok = True
            for name, pts in ep[thr].items():
                xn = cv2.undistortPoints(pts.reshape(-1, 1, 2), K, dist).reshape(-1, 2)
                l, keep, rms = fit_line(xn, fx)
                if keep.sum() < 15 or rms > 1.5:
                    ok = False; break
                v["edges"][name] = dict(l=l, xn=xn[keep], rms_px=rms)
            if ok:
                V[thr].append(v)
    t_img = time.time() - t0
    res = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), kf_dir=str(a.kf_dir), poses=a.poses, lens=a.lens, red_size_mm=[W_ref * 1e3, H_ref * 1e3],
               red_size_note="assumption (가정) unless measured: default = 14in Lenovo 83K9 panel 1920x1200 x 0.15729 mm (EDID pitch)",
               excl=EXCL, n_kf=len(frames), n_kf_red_quad=n_quad, n_views={str(t): len(V[t]) for t in THRS})
    fail = []

    def finish(status):
        res["status"] = status; res["runtime_s"] = round(time.time() - t0, 1)
        (out / "redscreen_scale.json").write_text(json.dumps(res, indent=1, default=float))
        print(json.dumps({k: v for k, v in res.items() if k not in ("per_thr",)}, default=float, indent=1))
        if status != "ok":
            print(f"REDSCREEN SCALE FAILED: {status}", file=sys.stderr); sys.exit(3)

    if len(V[0.5]) < a.min_views:
        finish(f"failed: {len(V[0.5])} keyframes with a fitted red screen (< {a.min_views}); is the red page in view?")
    per = {}
    for thr in THRS:
        if len(V[thr]) < a.min_views:
            fail.append(f"thr {thr}: {len(V[thr])} views"); continue
        fl, x0 = free_lines(V[thr]); pk = Pack(V[thr], fx)
        r = least_squares(resid, x0, args=(pk,), loss="huber", f_scale=1.0, x_scale="jac")
        rr = r.fun
        per[thr] = dict(n_views=len(V[thr]), W_mm=r.x[6] * 1e3, H_mm=r.x[7] * 1e3, s=(r.x[6] / W_ref + r.x[7] / H_ref) / 2,
                        sW=r.x[6] / W_ref, sH=r.x[7] / H_ref, ratio_WH=r.x[6] / r.x[7], med_abs_px=float(np.median(np.abs(rr))),
                        rms_px=float(np.sqrt(np.mean(rr ** 2))), free_lines=dict(W_mm=fl["W"] * 1e3, H_mm=fl["H"] * 1e3, angle_deg=fl["angle_deg"]),
                        line_rms_px_median=float(np.median([e["rms_px"] for v in V[thr] for e in v["edges"].values()])), x=r.x.tolist())
        if thr == 0.5:
            pk5, x5 = pk, r.x
    res["per_thr"] = {str(k): v for k, v in per.items()}
    if 0.5 not in per:
        finish("failed: " + "; ".join(fail))
    b = per[0.5]
    rng = np.random.default_rng(0); bs = []
    nv = len(V[0.5])
    for _ in range(a.boot):
        idx = rng.integers(0, nv, nv)
        sel = np.isin(pk5.vi, idx)                   # cluster bootstrap approximated by unique-view resampling with multiplicity
        w = np.bincount(idx, minlength=nv)[pk5.vi]
        pkb = Pack.__new__(Pack); pkb.fx = fx; pkb.Tcw = pk5.Tcw; pkb.vi, pkb.ei, pkb.xn = pk5.vi[sel], pk5.ei[sel], pk5.xn[sel]
        sw = np.sqrt(w[sel])
        rb = least_squares(lambda x: resid(x, pkb) * sw, x5, loss="huber", f_scale=1.0, x_scale="jac", max_nfev=30)
        bs.append((rb.x[6] / W_ref + rb.x[7] / H_ref) / 2)
    boot_sd = float(np.std(bs)) if bs else float("nan")
    spread = (per[0.3]["s"] - per[0.7]["s"]) if (0.3 in per and 0.7 in per) else float("nan")
    sigma = float(np.sqrt(boot_sd ** 2 + (spread / 2) ** 2 + 0.002 ** 2))
    s = float(b["s"])
    res.update(s=s, scale_error_pct=(s - 1) * 100, sigma=sigma, sigma_pct=sigma * 100, boot_sd=boot_sd, thr_spread=spread,
               W_mm=b["W_mm"], H_mm=b["H_mm"], ratio_WH=b["ratio_WH"], ratio_ref=W_ref / H_ref, n_views_used=b["n_views"],
               line_rms_px_median=b["line_rms_px_median"], bundle_med_abs_px=b["med_abs_px"], bundle_rms_px=b["rms_px"],
               corner_angle_deg=b["free_lines"]["angle_deg"], t_images_s=round(t_img, 1))
    gates = [(b["line_rms_px_median"] <= 0.6, f"line rms median {b['line_rms_px_median']:.2f} px > 0.6"),
             (abs(b["free_lines"]["angle_deg"] - 90) <= 1.0, f"corner angle {b['free_lines']['angle_deg']:.2f} deg not 90 +- 1"),
             (abs(b["ratio_WH"] / (W_ref / H_ref) - 1) <= 0.03, f"W/H {b['ratio_WH']:.3f} vs reference {W_ref / H_ref:.3f} (> 3 %): wrong --red-size or not the red page"),
             (b["med_abs_px"] <= 2.0, f"bundle median |residual| {b['med_abs_px']:.2f} px > 2"),
             (not np.isfinite(spread) or abs(spread) <= 0.015, f"threshold spread {spread:.4f} > 1.5 %"),
             (0.8 < s < 1.25, f"s {s:.4f} outside 0.8..1.25")]
    bad = [m for ok, m in gates if not ok] + fail
    if bad:
        finish("failed: " + "; ".join(bad))
    with gzip.open(out / "poses_redscreen.jsonl.gz", "wt") as f:
        for d in rows:
            if d.get("T_world_cam0") is not None:
                T = np.array(d["T_world_cam0"], float).reshape(4, 4); T[:3, 3] /= s
                d = dict(d, T_world_cam0=T.ravel().tolist(), scale_source="redscreen", scale_applied=1 / s)
            f.write(json.dumps(d) + "\n")
    finish("ok")


if __name__ == "__main__":
    main()
