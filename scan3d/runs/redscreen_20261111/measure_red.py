#!/usr/bin/env python3
"""redscreen 11-11 (a): size of the red full-screen rectangle from many BA keyframes -> scanner scale error.

Per keyframe (1920x1080 JPEG, the scanner's own keyframes):
  red mask (HSV) -> largest blob, all 4 edges inside the image -> rough quad (convex hull + approxPolyDP, 4 vertices)
  -> for each edge, ONLY the middle part (exclude EXCL of the edge length at both ends; screen corners are rounded)
     sub-pixel edge points: profile of redness r = R - max(G,B) across the edge, crossing at THR between the inside and outside levels
  -> points undistorted with the lens (K, dist) -> straight line fit (TLS, 3-sigma clip), residual in px.
3D, two estimators on the BA poses (T_world_cam0):
  A  free lines: each edge = 3D line from the back-projected planes of all views (LSQ); W = mean distance between the left and right
     lines (evaluated along the segment between the top/bottom lines), H likewise.
  B  rectangle bundle: plane pose (6) + W, H; residual = px distance of every edge point to the projected rectangle edge line (infinite line).
Uncertainty: bootstrap over keyframes (B), plus the spread over EXCL (0.10/0.15/0.20) and THR (0.3/0.5/0.7).
usage: measure_red.py SCAN_OUT LENS_JSON OUT_DIR [--boot 300]
"""
import argparse, glob, gzip, json, os, sys, time
from pathlib import Path
import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

REF_W, REF_H = 1920 * 0.15729e-3, 1200 * 0.15729e-3      # EDID pitch (165506 board.json), 가정
REF_W2, REF_H2 = 301.6e-3, 188.5e-3                       # task text (0.1571 mm/px), 가정


def load_poses(p):
    out = {}
    for line in gzip.open(p, "rt"):
        d = json.loads(line)
        if d.get("T_world_cam0") is not None:
            out[int(d["frame"])] = np.array(d["T_world_cam0"], float).reshape(4, 4)
    return out


def red_mask(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    # screen red (S24 exposure): R clipped ~255, G 60-100, B ~0 -> hue 5..12, S > 200
    m = (((h <= 20) | (h >= 172)) & (s >= 170) & (v >= 200)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    if n < 2:
        return None
    k = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    if st[k, cv2.CC_STAT_AREA] < 30000:
        return None
    blob = (lab == k).astype(np.uint8)
    cs, _ = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    filled = np.zeros_like(blob)
    cv2.drawContours(filled, cs, -1, 1, -1)                 # fill holes (text, glare)
    return filled, max(cs, key=cv2.contourArea)


def rough_quad(cnt, shape):
    hull = cv2.convexHull(cnt)
    per = cv2.arcLength(hull, True)
    for eps in (0.01, 0.015, 0.02, 0.03, 0.04):
        q = cv2.approxPolyDP(hull, eps * per, True).reshape(-1, 2).astype(float)
        if len(q) == 4:
            break
    else:
        return None
    H, W = shape[:2]
    if (q[:, 0].min() < 12) or (q[:, 1].min() < 12) or (q[:, 0].max() > W - 13) or (q[:, 1].max() > H - 13):
        return None                                          # an edge may be cut by the image border
    c = q.mean(0)
    ang = np.arctan2(q[:, 1] - c[1], q[:, 0] - c[0])
    q = q[np.argsort(ang)]                                   # image coords (y down): order = TL? -> rotate so q[0] = top-left
    k = int(np.argmin(q[:, 0] + q[:, 1]))
    q = np.roll(q, -k, 0)                                    # TL, TR, BR, BL (clockwise on screen with y down)
    if not (q[1, 0] > q[0, 0] and q[3, 1] > q[0, 1]):
        return None
    return q


EDGES = {"top": (0, 1), "right": (1, 2), "bottom": (3, 2), "left": (0, 3)}


def edge_points(score, q, excl, thr, step_px=3.0, half=9.0, ds=0.25):
    """sub-pixel crossings across each edge (middle part only). returns {edge: Nx2}"""
    c = q.mean(0)
    s = np.arange(-half, half + 1e-9, ds)
    out = {}
    for name, (i, j) in EDGES.items():
        a, b = q[i], q[j]
        L = np.linalg.norm(b - a)
        t = (b - a) / L
        n = np.array([-t[1], t[0]])
        if np.dot(n, (a + b) / 2 - c) < 0:
            n = -n                                           # outward
        fs = np.arange(excl * L, (1 - excl) * L + 1e-9, step_px)
        P = a[None] + fs[:, None] * t[None]
        X = (P[:, None, 0] + s[None] * n[0]).astype(np.float32)
        Y = (P[:, None, 1] + s[None] * n[1]).astype(np.float32)
        prof = cv2.remap(score, X, Y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        pts = []
        for k in range(len(P)):
            pr = prof[k]
            hi = np.median(pr[s <= -half + 3.5])                # inside
            lo = np.median(pr[s >= half - 3.5])                 # outside
            if hi - lo < 50:
                continue
            lev = lo + thr * (hi - lo)
            above = pr >= lev
            cr = np.flatnonzero(above[:-1] & ~above[1:])        # inside -> outside transitions
            if len(cr) != 1:
                continue
            m = cr[0]
            f = (pr[m] - lev) / (pr[m] - pr[m + 1])
            pts.append(P[k] + (s[m] + f * ds) * n)
        if len(pts) >= 15:
            out[name] = np.array(pts)
    return out


def fit_line(xn, fx):
    """TLS line in normalized coords with 3-sigma clipping. returns (l (a,b,c) with a^2+b^2=1, keep mask, rms px)"""
    keep = np.ones(len(xn), bool)
    for _ in range(4):
        m = xn[keep].mean(0)
        _, _, vt = np.linalg.svd(xn[keep] - m)
        nrm = vt[1]
        l = np.array([nrm[0], nrm[1], -nrm @ m])
        r = (xn @ l[:2] + l[2]) * fx
        sd = np.sqrt(np.mean(r[keep] ** 2))
        nk = np.abs(r) < max(3 * sd, 0.3)
        if (nk == keep).all():
            break
        keep = nk
    return l, keep, float(np.sqrt(np.mean(r[keep] ** 2)))


def views_for(frames, imgs_dir, poses, K, dist, excl, thr, cache):
    fx = K[0, 0]
    V = []
    for fr in frames:
        key = (fr, excl, thr)
        if key in cache:
            if cache[key] is not None:
                V.append(cache[key])
            continue
        img = cv2.imread(str(imgs_dir / f"{fr:05d}.jpg"))
        cache[key] = None
        r = red_mask(img)
        if r is None:
            continue
        q = rough_quad(r[1], img.shape)
        if q is None:
            continue
        bgr = img.astype(np.float32)
        score = (bgr[..., 2] - np.maximum(bgr[..., 1], bgr[..., 0]))
        ep = edge_points(score, q, excl, thr)
        if len(ep) < 3:                                       # a glare spot may kill one edge; keep the other 3
            continue
        v = dict(frame=fr, T=poses[fr], quad=q, edges={})
        okv = True
        for name, pts in ep.items():
            xn = cv2.undistortPoints(pts.reshape(-1, 1, 2), K, dist).reshape(-1, 2)
            l, keep, rms = fit_line(xn, fx)
            if keep.sum() < 15 or rms > 1.5:
                okv = False; break
            v["edges"][name] = dict(l=l, xn=xn[keep], rms_px=rms, n=int(keep.sum()), n_raw=len(pts))
        if okv:
            cache[key] = v
            V.append(v)
    return V


def line3d(views, name):
    """3D line from planes n_w . X = n_w . C (one per view)"""
    N, b = [], []
    for v in views:
        if name not in v["edges"]:
            continue
        R, C = v["T"][:3, :3], v["T"][:3, 3]
        nw = R @ v["edges"][name]["l"]
        nw /= np.linalg.norm(nw)
        N.append(nw); b.append(nw @ C)
    N, b = np.array(N), np.array(b)
    _, _, vt = np.linalg.svd(N)
    d = vt[2]
    p = np.linalg.lstsq(np.vstack([N, d[None] * 10]), np.concatenate([b, [0]]), rcond=None)[0]
    return p, d


def closest(p1, d1, p2, d2):
    w = p1 - p2
    a, bb, c, dd, e = d1 @ d1, d1 @ d2, d2 @ d2, d1 @ w, d2 @ w
    den = a * c - bb * bb
    s = (bb * e - c * dd) / den
    t = (a * e - bb * dd) / den
    return (p1 + s * d1 + p2 + t * d2) / 2


def est_A(views):
    L = {k: line3d(views, k) for k in EDGES}
    cTL = closest(*L["top"], *L["left"]); cTR = closest(*L["top"], *L["right"])
    cBL = closest(*L["bottom"], *L["left"]); cBR = closest(*L["bottom"], *L["right"])

    def dist_pt_line(x, pd):
        p, d = pd
        return np.linalg.norm(np.cross(x - p, d / np.linalg.norm(d)))

    def seg_to_line(a, b, other):
        return np.mean([dist_pt_line(a + f * (b - a), other) for f in np.linspace(0.15, 0.85, 8)])

    W = (seg_to_line(cTL, cBL, L["right"]) + seg_to_line(cTR, cBR, L["left"])) / 2
    H = (seg_to_line(cTL, cTR, L["bottom"]) + seg_to_line(cBL, cBR, L["top"])) / 2
    corners = np.array([cTL, cTR, cBR, cBL])
    # flatness / angles
    e1, e2 = cTR - cTL, cBL - cTL
    ang = np.degrees(np.arccos(abs(e1 @ e2) / np.linalg.norm(e1) / np.linalg.norm(e2)))
    return dict(W=W, H=H, corners=corners, angle_TL_deg=float(ang),
                W_top_corner=float(np.linalg.norm(cTR - cTL)), W_bot_corner=float(np.linalg.norm(cBR - cBL)),
                H_left_corner=float(np.linalg.norm(cBL - cTL)), H_right_corner=float(np.linalg.norm(cBR - cTR)))


ECORN = {"top": ((-1, 1), (1, 1)), "bottom": ((-1, -1), (1, -1)), "left": ((-1, 1), (-1, -1)), "right": ((1, 1), (1, -1))}


def rect_corners_world(x):
    R = Rot.from_rotvec(x[:3]).as_matrix()
    c, W, H = x[3:6], x[6], x[7]
    return R, c, W, H


def resid_B(x, views, fx):
    R, c, W, H = rect_corners_world(x)
    res = []
    for v in views:
        Tcw = np.linalg.inv(v["T"])
        for name, e in v["edges"].items():
            (sx1, sy1), (sx2, sy2) = ECORN[name]
            P1 = c + R @ np.array([sx1 * W / 2, sy1 * H / 2, 0]); P2 = c + R @ np.array([sx2 * W / 2, sy2 * H / 2, 0])
            p1 = Tcw[:3, :3] @ P1 + Tcw[:3, 3]; p2 = Tcw[:3, :3] @ P2 + Tcw[:3, 3]
            l = np.cross(np.append(p1[:2] / p1[2], 1), np.append(p2[:2] / p2[2], 1))
            l /= np.linalg.norm(l[:2])
            xn = e["xn"]
            res.append((xn @ l[:2] + l[2]) * fx)
    return np.concatenate(res)


def init_B(A):
    C = A["corners"]                                          # TL TR BR BL (world)
    c = C.mean(0)
    ex = ((C[1] - C[0]) + (C[2] - C[3])); ex /= np.linalg.norm(ex)
    ey = ((C[0] - C[3]) + (C[1] - C[2])); ey -= (ey @ ex) * ex; ey /= np.linalg.norm(ey)
    R = np.stack([ex, ey, np.cross(ex, ey)], 1)
    return np.concatenate([Rot.from_matrix(R).as_rotvec(), c, [A["W"], A["H"]]])


def est_B(views, fx, x0):
    r = least_squares(resid_B, x0, args=(views, fx), loss="huber", f_scale=1.0, x_scale="jac")
    res = r.fun
    return dict(W=float(r.x[6]), H=float(r.x[7]), x=r.x, rms_px=float(np.sqrt(np.mean(res ** 2))), n_res=int(len(res)),
                med_abs_px=float(np.median(np.abs(res))))


def summ(W, H, tag=""):
    d = float(np.hypot(W, H))
    return dict(W_mm=W * 1e3, H_mm=H * 1e3, diag_mm=d * 1e3, sW=W / REF_W, sH=H / REF_H, s_diag=d / np.hypot(REF_W, REF_H),
                s_mean=(W / REF_W + H / REF_H) / 2, ratio_WH=W / H, ratio_ref=REF_W / REF_H)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scan_out"); ap.add_argument("lens"); ap.add_argument("out")
    ap.add_argument("--boot", type=int, default=300)
    a = ap.parse_args()
    t0 = time.time()
    so, out = Path(a.scan_out), Path(a.out); out.mkdir(parents=True, exist_ok=True)
    kfd = Path(glob.glob(str(so / "red" / "*_kf"))[0]); imgs = kfd / "frames_kf"
    poses = load_poses(so / "ba" / "poses_ba.jsonl.gz")
    L = json.loads(Path(a.lens).read_text()); K = np.array(L["K"], float); dist = np.array(L["dist"], float)
    fx = K[0, 0]
    frames = sorted(int(Path(f).stem) for f in glob.glob(str(imgs / "*.jpg")))
    frames = [f for f in frames if f in poses]
    cache = {}
    res = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), scan_out=str(so), lens=a.lens, K=K.tolist(), dist=dist.tolist(),
               poses=str(so / "ba" / "poses_ba.jsonl.gz"), n_kf=len(frames), ref=dict(W_mm=REF_W * 1e3, H_mm=REF_H * 1e3,
               note="EDID pitch 0.15729 mm/px x 1920x1200 (가정); task text 301.6 x 188.5"), variants={})
    base = None
    for excl in (0.10, 0.15, 0.20):
        for thr in (0.3, 0.5, 0.7):
            if excl != 0.15 and thr != 0.5:
                continue
            V = views_for(frames, imgs, poses, K, dist, excl, thr, cache)
            A = est_A(V)
            B = est_B(V, fx, init_B(A))
            rms = [e["rms_px"] for v in V for e in v["edges"].values()]
            r = dict(excl=excl, thr=thr, n_views=len(V), n_edges={k: sum(k in v["edges"] for v in V) for k in EDGES}, line_rms_px_median=float(np.median(rms)), line_rms_px_p90=float(np.percentile(rms, 90)),
                     A=dict(summ(A["W"], A["H"]), angle_TL_deg=A["angle_TL_deg"], W_top_corner_mm=A["W_top_corner"] * 1e3,
                            W_bot_corner_mm=A["W_bot_corner"] * 1e3, H_left_corner_mm=A["H_left_corner"] * 1e3, H_right_corner_mm=A["H_right_corner"] * 1e3),
                     B=dict(summ(B["W"], B["H"]), rms_px=B["rms_px"], med_abs_px=B["med_abs_px"], n_res=B["n_res"]))
            print(f"excl {excl} thr {thr}: views {len(V)} A W {A['W']*1e3:.2f} H {A['H']*1e3:.2f} | B W {B['W']*1e3:.2f} H {B['H']*1e3:.2f} "
                  f"rms {B['rms_px']:.3f}px  s_mean A {r['A']['s_mean']:.4f} B {r['B']['s_mean']:.4f}", flush=True)
            res["variants"][f"e{excl:.2f}_t{thr:.1f}"] = r
            if excl == 0.15 and thr == 0.5:
                base = (V, B, A)
    V, B, A = base
    # per-view info + distance/angle stats of the base set
    cams = np.array([v["T"][:3, 3] for v in V])
    ctr = B["x"][3:6]
    res["base_B_x"] = B["x"].tolist()                       # rotvec(3), centre(3) [world = BA world], W, H
    res["base_A_corners_TL_TR_BR_BL"] = A["corners"].tolist()
    res["base_views"] = dict(frames=[v["frame"] for v in V], dist_m=dict(min=float(np.linalg.norm(cams - ctr, axis=1).min()),
                             median=float(np.median(np.linalg.norm(cams - ctr, axis=1))), max=float(np.linalg.norm(cams - ctr, axis=1).max())))
    # per-view B residual rms
    pv = []
    for v in V:
        rr = resid_B(B["x"], [v], fx); pv.append(float(np.sqrt(np.mean(rr ** 2))))
    res["base_views"]["resid_rms_px_per_view"] = dict(median=float(np.median(pv)), p90=float(np.percentile(pv, 90)), max=float(np.max(pv)))
    # bootstrap over keyframes (B, base variant)
    rng = np.random.default_rng(0)
    bs = []
    for k in range(a.boot):
        idx = rng.integers(0, len(V), len(V))
        Vb = [V[i] for i in idx]
        bb = least_squares(resid_B, B["x"], args=(Vb, fx), loss="huber", f_scale=1.0, x_scale="jac", max_nfev=50)
        bs.append(bb.x[6:8])
    bs = np.array(bs)
    sm = (bs[:, 0] / REF_W + bs[:, 1] / REF_H) / 2
    res["bootstrap_B"] = dict(n=a.boot, W_mm_sd=float(bs[:, 0].std() * 1e3), H_mm_sd=float(bs[:, 1].std() * 1e3),
                              s_mean_sd=float(sm.std()), s_mean_p2_5=float(np.percentile(sm, 2.5)), s_mean_p97_5=float(np.percentile(sm, 97.5)))
    # split halves by time (first / second half of the views): drift check
    h = len(V) // 2
    for nm, Vs in (("first_half", V[:h]), ("second_half", V[h:])):
        bb = est_B(Vs, fx, B["x"]); res[f"B_{nm}"] = summ(bb["W"], bb["H"])
    allB = [r["B"]["s_mean"] for r in res["variants"].values()]
    res["variant_spread_s_mean_B"] = dict(min=min(allB), max=max(allB))
    res["runtime_s"] = round(time.time() - t0, 1)
    (out / "red_measure.json").write_text(json.dumps(res, indent=1, default=float))
    # overlay images for a few views (check by eye)
    for v in V[:: max(1, len(V) // 6)][:6]:
        img = cv2.imread(str(imgs / f"{v['frame']:05d}.jpg"))
        cv2.polylines(img, [v["quad"].astype(np.int32).reshape(-1, 1, 2)], True, (255, 255, 0), 1)
        for name, e in v["edges"].items():
            pts = cv2.undistortPoints(e["xn"].reshape(-1, 1, 2).astype(np.float64), np.eye(3), None)  # identity (already normalized)
            # redistort normalized points for drawing
            P3 = np.concatenate([e["xn"], np.ones((len(e["xn"]), 1))], 1)
            pp, _ = cv2.projectPoints(P3, np.zeros(3), np.zeros(3), K, dist)
            for p in pp.reshape(-1, 2):
                cv2.circle(img, (int(round(p[0])), int(round(p[1]))), 2, (0, 255, 0), -1)
        cv2.imwrite(str(out / f"overlay_{v['frame']:05d}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 85])
    print(json.dumps({k: res[k] for k in ("bootstrap_B", "variant_spread_s_mean_B", "base_views")}, default=float)[:1500])
    print("B first/second half s_mean", res["B_first_half"]["s_mean"], res["B_second_half"]["s_mean"])


if __name__ == "__main__":
    main()
