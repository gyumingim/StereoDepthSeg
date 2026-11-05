#!/usr/bin/env python3
"""scanA 2026-11-05 step 2, added AFTER the first results (README 5): board-corner triangulation ruler.

Why: the pre-registered ruler (pair relative pose vs per-frame board GT) has a noise floor of the same size as the VIO
error (imu_gt raw vs imu_gt_spl differ 11 mm / 0.9 deg median on the same pairs; only 8 ChArUco corners per frame).
This ruler does NOT use the per-frame GT poses: it triangulates the detected ChArUco inner corners (imu_gt_rs/gt.npz
'corners', raw 1920x1080 pixels, undistorted with the GT lens K/dist) with the poses under test and compares the shape
with the known board layout (square 60.4 mm, 4x2 inner corners):
  * pair2v: for each keyframe pair of the fixed GT pair set (baseline 4-30 cm) with >= 6 common corners: two-view
    triangulation -> rigid fit (no scale) to the board model -> RMS (mm); similarity scale - 1 (%) = depth scale error
  * multi-view: each corner id triangulated from every keyframe that sees it -> reprojection RMS (px @1920) and the
    rigid-fit RMS of the 8 points (mm)
Circularity: the GT poses come from the same corners (GT rows = reference only). BA rows are independent of the corners
only for BA run with --mask-board (board region grey in the BA images).
  python scan3d/scanA/corner_eval.py REC --pose NAME=POSES.jsonl.gz [...] --out OUT.json
"""
import argparse, gzip, json, time
from pathlib import Path
import cv2
import numpy as np


def load(p):
    d = {}
    for line in gzip.open(p, "rt"):
        m = json.loads(line)
        if m.get("type") == "pose" and m.get("T_world_cam0") and m.get("status") == "tracking":
            d[int(m["t_ns"])] = np.array(m["T_world_cam0"], float).reshape(4, 4)
    return d


def fit(A, B, scale=False):
    ma, mb = A.mean(0), B.mean(0)
    U, S, Vt = np.linalg.svd((B - mb).T @ (A - ma))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    s = float(np.trace(np.diag(S) @ D) / ((A - ma) ** 2).sum()) if scale else 1.0
    return s, R, mb - s * R @ ma


def st(x):
    x = np.asarray(x, float)
    return dict(n=int(len(x)), mean=float(x.mean()), median=float(np.median(x)), p10=float(np.percentile(x, 10)),
                p90=float(np.percentile(x, 90)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec"); ap.add_argument("--pose", action="append", default=[])
    ap.add_argument("--kf-step", type=int, default=10); ap.add_argument("--out", required=True)
    ap.add_argument("--rs-skew-ms", type=float, default=0.0, help="rolling shutter: per-corner row-time camera (same twist for all sources)")
    ap.add_argument("--twist", default="", help="poses for the camera twist (frames i-1, i+1); needed with --rs-skew-ms")
    a = ap.parse_args()
    rec = Path(a.rec)
    gs = np.load(rec / "imu_gt_spl" / "gt.npz"); t_ns, ok = gs["t_ns"], gs["ok"]
    gc = np.load(rec / "imu_gt_rs" / "gt.npz", allow_pickle=True)
    assert np.all(np.abs(gc["t_ns"] - t_ns) < 2e6)
    K, dist = gc["K"], gc["dist"]
    sq = float(gc["square_mm"]) / 1000
    model = {k: np.array([((k % 4) + 1) * sq, ((k // 4) + 1) * sq, 0.0]) for k in range(8)}
    obs = {}
    for i in range(0, len(t_ns), a.kf_step):
        c, ids = gc["corners"][i], gc["corner_ids"][i]
        if c is None or ids is None or len(np.atleast_1d(ids)) < 4:
            continue
        c = np.asarray(c, np.float64).reshape(-1, 1, 2)
        n = cv2.undistortPoints(c, K, dist).reshape(-1, 2)
        obs[i] = {int(k): (n[x], c[x, 0]) for x, k in enumerate(np.asarray(ids).ravel())}
    tw = {}
    if a.rs_skew_ms > 0:
        d = load(a.twist); tt = np.array(sorted(d))
        def near(tq):
            k = np.searchsorted(tt, tq).clip(1, len(tt) - 1)
            k = k - 1 if abs(tt[k - 1] - tq) < abs(tt[k] - tq) else k
            return tt[k] if abs(tt[k] - tq) < 4e6 else None
        for i in obs:
            ka, kb = near(t_ns[max(i - 1, 0)]), near(t_ns[min(i + 1, len(t_ns) - 1)])
            if ka is None or kb is None or ka == kb:
                continue
            D = np.linalg.inv(d[ka]) @ d[kb]; dtt = (kb - ka) * 1e-9
            tw[i] = (cv2.Rodrigues(D[:3, :3])[0].ravel() / dtt, D[:3, 3] / dtt)

    def cam_of(cw, i, k):
        """cam_from_world (3x4) for corner k seen in frame i (row-time camera with rolling shutter)."""
        if i not in tw:
            return cw[i][:3]
        dt = (obs[i][k][1][1] / 1080.0 - 0.5) * a.rs_skew_ms * 1e-3
        Dl = np.eye(4); Dl[:3, :3] = cv2.Rodrigues(tw[i][0] * dt)[0]; Dl[:3, 3] = tw[i][1] * dt
        return (np.linalg.inv(Dl) @ cw[i])[:3]

    def tri(views):
        A = np.concatenate([np.stack([n[0] * P[2] - P[0], n[1] * P[2] - P[1]]) for P, n in views])
        Xh = np.linalg.svd(A)[2][-1]
        return Xh[:3] / Xh[3]
    src = {}
    for kv in a.pose:
        nm, p = kv.split("=", 1)
        d = load(p); tt = np.array(sorted(d)); src[nm] = {}
        for i in obs:
            k = np.searchsorted(tt, t_ns[i]).clip(1, len(tt) - 1)
            k = k - 1 if abs(tt[k - 1] - t_ns[i]) < abs(tt[k] - t_ns[i]) else k
            if abs(tt[k] - t_ns[i]) < 4e6:
                src[nm][i] = d[tt[k]]
    kf = sorted(i for i in obs if ok[i] and all(i in s for s in src.values()))
    Pg = {i: np.eye(4) for i in kf}
    for i in kf:
        Pg[i][:3, :3] = gs["R_bc"][i]; Pg[i][:3, 3] = gs["p_bc"][i]
    pairs = []
    for x, i in enumerate(kf):
        for j in kf[x + 1:]:
            if j - i > 300:
                break
            B = np.linalg.norm(Pg[i][:3, 3] - Pg[j][:3, 3])
            com = sorted(set(obs[i]) & set(obs[j]))
            if 0.04 <= B <= 0.30 and len(com) >= 6:
                pairs.append((i, j, com))
    res = dict(made=time.strftime("%F %T"), rec=str(rec.resolve()), args=vars(a), n_kf=len(kf), n_pairs=len(pairs),
               corners_src="imu_gt_rs/gt.npz corners (raw px) undistorted with its K/dist", sources={})
    for nm, s in src.items():
        cw = {i: np.linalg.inv(s[i]) for i in kf}
        rms2, scl = [], []
        for i, j, com in pairs:
            X = np.array([tri([(cam_of(cw, i, k), obs[i][k][0]), (cam_of(cw, j, k), obs[j][k][0])]) for k in com])
            M = np.array([model[k] for k in com])
            _, R, t = fit(M, X)
            rms2.append(np.sqrt(((X - (R @ M.T).T - t) ** 2).sum(1).mean()) * 1000)
            sc, _, _ = fit(M, X, scale=True)
            scl.append((sc - 1) * 100)
        # multi-view
        Xmv, rep = {}, []
        for k in range(8):
            vs = [i for i in kf if k in obs[i]]
            if len(vs) < 3:
                continue
            X = tri([(cam_of(cw, i, k), obs[i][k][0]) for i in vs]); Xmv[k] = X
            for i in vs:
                Pc = cam_of(cw, i, k)
                Xc = Pc[:, :3] @ X + Pc[:, 3]
                q = cv2.projectPoints(Xc.reshape(1, 3), np.zeros(3), np.zeros(3), K, dist)[0].ravel()
                rep.append(np.linalg.norm(q - obs[i][k][1]))
        ks = sorted(Xmv)
        M = np.array([model[k] for k in ks]); X = np.array([Xmv[k] for k in ks])
        _, R, t = fit(M, X); sc, _, _ = fit(M, X, scale=True)
        res["sources"][nm] = dict(pair2v_rigid_rms_mm=st(rms2), pair2v_scale_err_pct=st(scl),
                                  pair2v_abs_scale_err_pct=st(np.abs(scl)),
                                  mv_reproj_px=st(rep), mv_rigid_rms_mm=float(np.sqrt(((X - (R @ M.T).T - t) ** 2).sum(1).mean()) * 1000),
                                  mv_scale_err_pct=(sc - 1) * 100)
    Path(a.out).write_text(json.dumps(res, indent=1))
    print(f"kf {len(kf)} pairs {len(pairs)}")
    for nm, r in res["sources"].items():
        p = r["pair2v_rigid_rms_mm"]; q = r["pair2v_abs_scale_err_pct"]; m = r["mv_reproj_px"]
        print(f"{nm:14s} pair2v rms mm mean {p['mean']:.2f} med {p['median']:.2f} p90 {p['p90']:.2f} | |scale| % med {q['median']:.2f} "
              f"p90 {q['p90']:.2f} | mv reproj px med {m['median']:.2f} p90 {m['p90']:.2f} | mv rigid {r['mv_rigid_rms_mm']:.2f} mm "
              f"scale {r['mv_scale_err_pct']:+.2f} %")


if __name__ == "__main__":
    main()
