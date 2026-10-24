"""Joint lens fit over all tripod poses (std60 1920x1080, 0.5 dpt, 2026-10-23) + holdout, coverage, repeatability.

Input: views/poses.pkl (steady.py). Each recording = one static pose; near-identical poses (< 30 px mean corner
difference, steady.py "near") form one cluster and share N_PER_CLUSTER frames so every cluster weighs about the same.
Models: lens_calib.MODELS (k1k2p, k1k2p_k3, rational) + "k1_prior" (k2 p1 p2 k3 fixed to the 2.22 dpt node, fit fx fy cx cy
k1 — the round-2 joint-fit structure). Model choice like lens_calib: lowest held-out error, but held out BY POSE
(frames of one pose are near copies, so lens_calib.holdout_err's even/odd frame split would leak).
  ../venv_ffs/bin/python runs/lens_calib_f050_20261023/calib.py [--exclude REC ...] [--tag name]
→ results/calib_<tag>.json
"""
import argparse
import itertools
import json
import pickle
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
S3D = HERE.parents[1]
sys.path.insert(0, str(S3D))
sys.path.insert(0, str(S3D / "imu_learn"))
import aruco_scale  # noqa: E402
import lens_calib  # noqa: E402

N_PER_CLUSTER = 5
REF = S3D / "config" / "lens" / "hs_1920x1080_f2.22_cal20261022.json"
FLAGS = dict(lens_calib.MODELS)
FLAGS["k1_prior"] = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_TANGENT_DIST)
# f only: principal point, aspect and ALL distortion from the 2.22 node; only the scale fx (fy = fx * node ratio) is fitted
FLAGS["f_prior"] = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_FIX_ASPECT_RATIO
                    | cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_TANGENT_DIST)
PRIOR = ("k1_prior", "f_prior")
# free radial k1 k2 k3, tangential fixed at 0 (2026-10-24: free p1 p2 trade with cx; check if that is the pp scatter)
FLAGS["k123_notan"] = cv2.CALIB_FIX_TANGENT_DIST
FLAGS["k12_notan"] = cv2.CALIB_FIX_TANGENT_DIST | cv2.CALIB_FIX_K3     # OpenVINS radtan-compatible (k1 k2, p = 0)
W, H = 1920, 1080
EXPECT_FX = None


def ref_lens():
    K, d, j = lens_calib.load(REF)
    return K, d


def guess():
    K, d = ref_lens()
    s = (1 + 0.008 * (0.5 - 2.22)) * (1 - 0.0039)
    K = K.copy()
    K[0, 0] *= s
    K[1, 1] *= s
    return K, d.copy()


def fit(views, model):
    obj = [v["obj"] for v in views]
    img = [v["img"] for v in views]
    if model in PRIOR:
        K0, d0 = guess()
        rms, K, d, rv, tv = cv2.calibrateCamera(obj, img, (W, H), K0.copy(), d0.copy(), flags=FLAGS[model],
                                                criteria=lens_calib.TERM)
    else:
        rms, K, d, rv, tv = cv2.calibrateCamera(obj, img, (W, H), None, None, flags=FLAGS[model],
                                                criteria=lens_calib.TERM)
    return rms, K, d.ravel(), rv, tv


def view_err(v, K, d, rv=None, tv=None):
    if rv is None:
        ok, rv, tv = cv2.solvePnP(v["obj"], v["img"], K, d, flags=cv2.SOLVEPNP_ITERATIVE)
    p, _ = cv2.projectPoints(v["obj"], rv, tv, K, d)
    r = (p - v["img"]).reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum(r ** 2, -1)))), r, rv, tv


def fit_clean(views, model):
    """lens_calib.calibrate step 4: drop views whose error > 2.5 x median, refit."""
    rms, K, d, rv, tv = fit(views, model)
    per = np.array([view_err(v, K, d, a, b)[0] for v, a, b in zip(views, rv, tv)])
    keep = per < 2.5 * np.median(per)
    dropped = [f"{v['rec'][12:18]}#{v['frame']}" for v, k in zip(views, keep) if not k]
    if (~keep).any():
        views = [v for v, k in zip(views, keep) if k]
        rms, K, d, rv, tv = fit(views, model)
    return dict(rms=float(rms), K=K, d=d, rv=rv, tv=tv, views=views, dropped=dropped)


def holdout(views_fit, views_test, model):
    r = fit(views_fit, model)
    e = [view_err(v, r[1], r[2])[0] for v in views_test]
    return float(np.median(e)), float(np.mean(e))


def build(exclude=()):
    P = pickle.load(open(HERE / "views" / "poses.pkl", "rb"))
    poses = [p for p in P["poses"] if p["rec"] not in exclude]
    # clusters of near poses
    cl = {p["rec"]: p["rec"] for p in poses}
    for a, b, _ in P["near"]:
        if a in cl and b in cl:
            ra, rb = cl[a], cl[b]
            for k in cl:
                if cl[k] == rb:
                    cl[k] = ra
    members = {}
    for p in poses:
        members.setdefault(cl[p["rec"]], []).append(p)
    boards = {}
    views = []
    for c, ps in members.items():
        n_each = max(2, int(round(N_PER_CLUSTER / len(ps))))
        for p in ps:
            if p["board"] not in boards:
                if "2markers" in p["board"]:
                    import collect
                    boards[p["board"]] = collect.TwoMarkers(v2=p["board"].endswith("_v2"))
                else:
                    boards[p["board"]] = aruco_scale.Board(aruco_scale.load_board_spec(str(S3D / "config" / "charuco" / p["board"])))
            b = boards[p["board"]]
            vs = p["views"]
            sel = [vs[int(round(x))] for x in np.linspace(0, len(vs) - 1, min(n_each, len(vs)))]
            for fi, ids, c2 in sel:
                views.append(dict(rec=p["rec"], cluster=c, frame=int(fi), ids=ids,
                                  obj=b.obj[ids].astype(np.float32), img=c2.astype(np.float32).reshape(-1, 1, 2)))
    clusters = sorted(members)            # time order (rec names sort by time)
    return views, clusters, members


def split_by(views, keyset, key="cluster"):
    a = [v for v in views if v[key] in keyset]
    b = [v for v in views if v[key] not in keyset]
    return a, b


def lens_summary(K, d):
    return dict(fx=round(float(K[0, 0]), 2), fy=round(float(K[1, 1]), 2), cx=round(float(K[0, 2]), 2),
                cy=round(float(K[1, 2]), 2), dist=[round(float(x), 6) for x in d])


def coverage(views, nx=4, ny=3, fx=32, fy=18):
    pts = np.concatenate([v["img"].reshape(-1, 2) for v in views])
    g = np.zeros((ny, nx), int)
    gp = [[set() for _ in range(nx)] for _ in range(ny)]
    for v in views:
        for x, y in v["img"].reshape(-1, 2):
            i, j = min(int(y / H * ny), ny - 1), min(int(x / W * nx), nx - 1)
            g[i, j] += 1
            gp[i][j].add(v["cluster"])
    fine = np.zeros((fy, fx), bool)
    for x, y in pts:
        fine[min(int(y / H * fy), fy - 1), min(int(x / W * fx), fx - 1)] = True
    # radius coverage (normalized by half diagonal)
    r = np.hypot(pts[:, 0] - W / 2, pts[:, 1] - H / 2) / np.hypot(W / 2, H / 2)
    return dict(grid_corners=g.tolist(), grid_poses=[[len(s) for s in row] for row in gp], fine_frac=float(fine.mean()),
                r_max=float(r.max()), r_p90=float(np.quantile(r, 0.9)),
                x_range=[float(pts[:, 0].min()), float(pts[:, 0].max())], y_range=[float(pts[:, 1].min()), float(pts[:, 1].max())])


def _absstat(hs, f):
    return {k: float(f([abs(h[k]) for h in hs])) for k in ("dfx_pct", "dcx", "dcy")} if hs else None


TILTED = []


def tilt_dist(rv, tv):
    R, _ = cv2.Rodrigues(rv)
    n = R[:, 2]
    tilt = float(np.degrees(np.arccos(abs(n[2]))))
    return tilt, float(np.linalg.norm(tv))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exclude", nargs="*", default=[])
    ap.add_argument("--exclude-file", default=None, help="file with recording names to exclude (whitespace separated)")
    ap.add_argument("--tag", default="all")
    ap.add_argument("--models", nargs="*", default=["k1k2p", "k1k2p_k3", "rational", "k1_prior", "f_prior"])
    a = ap.parse_args()
    if a.exclude_file:
        a.exclude = list(a.exclude) + open(a.exclude_file).read().split()
    views, clusters, members = build(a.exclude)
    # tilt of each cluster from a quick k1_prior fit (used for stratified halves)
    r0 = fit(views, "k1_prior")
    tl = {}
    for v, rv, tv in zip(views, r0[3], r0[4]):
        tl.setdefault(v["cluster"], []).append(tilt_dist(rv, tv)[0])
    TILTED[:] = [c for c in clusters if np.mean(tl[c]) > 25]
    print("tilted clusters:", [c[12:18] for c in TILTED])
    print(f"views {len(views)}, poses {sum(len(m) for m in members.values())}, clusters {len(clusters)}")
    res = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), exclude=a.exclude, n_views=len(views), clusters=clusters,
               members={c: [p["rec"] for p in m] for c, m in members.items()}, models={})
    even = set(clusters[0::2])
    for model in a.models:
        t0 = time.time()
        full = fit_clean(views, model)
        A, B = split_by(views, even)
        h1 = holdout(A, B, model)
        h2 = holdout(B, A, model)
        # leave-one-pose(cluster)-out
        lopo = {}
        for c in clusters:
            tr, te = split_by(views, set(clusters) - {c})
            r = fit(tr, model)
            e = [view_err(v, r[1], r[2])[0] for v in te]
            lopo[c] = dict(lens_summary(r[1], r[2]), test_err=float(np.median(e)))
        # halves: alternate split + all random balanced splits (up to 40)
        halves = []
        ab = []
        for nm, S in [("even_odd", even), ("first_second", set(clusters[:len(clusters) // 2]))]:
            ab.append((nm, S))
        rng = np.random.default_rng(0)
        for k in range(30):
            S = set(rng.permutation(clusters)[:len(clusters) // 2].tolist())
            ab.append((f"rand{k}", S))
        # stratified: tilted (> 25 deg, all near the image centre) and frontal (edge) poses split evenly
        for k in range(30):
            S = set()
            for grp in (TILTED, [c for c in clusters if c not in TILTED]):
                g = [c for c in grp if c in clusters]
                S |= set(rng.permutation(g)[:len(g) // 2 + (k % 2) * (len(g) % 2)].tolist())
            ab.append((f"strat{k}", S))
        for nm, S in ab:
            A, B = split_by(views, S)
            ra, rb = fit(A, model), fit(B, model)
            Ka, Kb = ra[1], rb[1]
            halves.append(dict(name=nm, A=lens_summary(Ka, ra[2]), B=lens_summary(Kb, rb[2]),
                               dfx_pct=float((Kb[0, 0] - Ka[0, 0]) / Ka[0, 0] * 100),
                               dfy_pct=float((Kb[1, 1] - Ka[1, 1]) / Ka[1, 1] * 100),
                               dcx=float(Kb[0, 2] - Ka[0, 2]), dcy=float(Kb[1, 2] - Ka[1, 2])))
        # per-pose residuals with full fit
        perpose = {}
        for v, rv, tv in zip(full["views"], full["rv"], full["tv"]):
            e, r, _, _ = view_err(v, full["K"], full["d"], rv, tv)
            tl, dist = tilt_dist(rv, tv)
            p = perpose.setdefault(v["rec"], dict(err=[], tilt=[], dist=[], centre=None, n_corners=len(v["ids"])))
            p["err"].append(e)
            p["tilt"].append(tl)
            p["dist"].append(dist)
            p["centre"] = [float(x) for x in v["img"].reshape(-1, 2).mean(0)]
            p["bbox"] = [float(x) for x in np.r_[v["img"].reshape(-1, 2).min(0), v["img"].reshape(-1, 2).max(0)]]
        for p in perpose.values():
            p["err"] = float(np.mean(p["err"]))
            p["tilt"] = float(np.mean(p["tilt"]))
            p["dist"] = float(np.mean(p["dist"]))
        # std devs from calibrateCameraExtended
        try:
            K0, d0 = (guess() if model in PRIOR else (None, None))
            out = cv2.calibrateCameraExtended([v["obj"] for v in full["views"]], [v["img"] for v in full["views"]],
                                              (W, H), None if K0 is None else K0.copy(),
                                              None if d0 is None else d0.copy(), flags=FLAGS[model],
                                              criteria=lens_calib.TERM)
            sd = out[5].ravel()[:4].tolist()
        except cv2.error:
            sd = None
        hv = [h for h in halves]
        res["models"][model] = dict(
            lens=lens_summary(full["K"], full["d"]), K=full["K"].tolist(), dist=full["d"].tolist(), rms_px=full["rms"],
            n_views=len(full["views"]), dropped=full["dropped"], holdout_px=dict(even_fit_odd_test=h1, odd_fit_even_test=h2,
                                                                               mean_median=(h1[0] + h2[0]) / 2),
            lopo=lopo, lopo_test_median=float(np.median([x["test_err"] for x in lopo.values()])),
            lopo_fx_range=[min(x["fx"] for x in lopo.values()), max(x["fx"] for x in lopo.values())],
            lopo_cx_range=[min(x["cx"] for x in lopo.values()), max(x["cx"] for x in lopo.values())],
            lopo_cy_range=[min(x["cy"] for x in lopo.values()), max(x["cy"] for x in lopo.values())],
            halves=halves,
            strat_abs_max=_absstat([h for h in hv if h["name"].startswith("strat")], max),
            strat_abs_p50=_absstat([h for h in hv if h["name"].startswith("strat")], np.median),
            rand_abs_p50=_absstat([h for h in hv if h["name"].startswith("rand")], np.median),
            halves_abs_max=dict(dfx_pct=max(abs(h["dfx_pct"]) for h in hv), dcx=max(abs(h["dcx"]) for h in hv),
                                dcy=max(abs(h["dcy"]) for h in hv)),
            halves_abs_p50=dict(dfx_pct=float(np.median([abs(h["dfx_pct"]) for h in hv])),
                                dcx=float(np.median([abs(h["dcx"]) for h in hv])),
                                dcy=float(np.median([abs(h["dcy"]) for h in hv]))),
            sd_fx_fy_cx_cy=sd, perpose=perpose, coverage=coverage(full["views"]), sec=round(time.time() - t0, 1))
        m = res["models"][model]
        L = m["lens"]
        print(f"   strat max {m['strat_abs_max']} p50 {m['strat_abs_p50']} rand p50 {m['rand_abs_p50']}")
        print(f"[{model}] fx {L['fx']} fy {L['fy']} cx {L['cx']} cy {L['cy']} rms {m['rms_px']:.3f} holdout "
              f"{h1[0]:.3f}/{h2[0]:.3f} lopo {m['lopo_test_median']:.3f} fx-lopo {m['lopo_fx_range']} "
              f"cx {m['lopo_cx_range']} cy {m['lopo_cy_range']} halves max {m['halves_abs_max']} p50 {m['halves_abs_p50']}"
              f" even/odd {halves[0]['dfx_pct']:.2f}% {halves[0]['dcx']:.1f} {halves[0]['dcy']:.1f} sd {sd} "
              f"dropped {len(full['dropped'])} ({m['sec']} s)")
    json.dump(res, open(HERE / "results" / f"calib_{a.tag}.json", "w"), indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))


if __name__ == "__main__":
    main()
