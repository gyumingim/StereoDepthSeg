#!/usr/bin/env python3
"""scanA 2026-11-05 step 2: keyframe bundle adjustment seeded by VIO poses (mono lens-5 hs recording).

  venv_ffs/bin/python scan3d/scanA/kf_ba.py REC --vio POSES.jsonl.gz --out DIR [--kf-step 10] [--scale 0.5] [--mask-board]

1. keyframes = every --kf-step-th frame that has a VIO 'tracking' pose (T_world_cam0, t_ns = frame t_mid, nearest < 4 ms)
2. images: cam5.mp4 -> resize (scale) -> undistort to pinhole Kr with the GT lens K/dist (= e2e_mono_ms grid, R = I),
   cached as JPEG under REC/scanA_cache/<tag>/ (regenerable). --mask-board fills the GT-projected board quad (+2 cm) gray
   (only frames with a GT pose; test of circularity: GT comes from the board corners of the same images).
3. pycolmap (CUDA) SIFT + matching of pairs (time neighbours +-seq kf, plus VIO-covisible pairs) + two-view verification
4. tracks (union-find of verified inlier matches), DLT triangulation with the VIO poses, outlier obs > 4 px removed
5. Levenberg-Marquardt BA (own Schur-complement solver, Huber 1 px): all keyframe poses + points free, intrinsics fixed,
   first keyframe fixed (gauge), scale free -> Sim(3) (Umeyama) of BA centres onto the VIO centres, so world frame,
   gravity and metric scale stay the VIO ones (as in a real run without GT).
Writes OUT/poses_ba.jsonl.gz (same format as the bridge: type pose, status tracking, t_ns, T_world_cam0; keyframes only),
OUT/poses_vio_kf.jsonl.gz (the VIO poses of the same keyframes), OUT/ba.json (stats, inputs, made).
"""
import argparse, gzip, json, sys, time
from pathlib import Path

import cv2
import numpy as np


def load_vio(p):
    t, T = [], []
    for line in gzip.open(p, "rt"):
        d = json.loads(line)
        if d.get("type") == "pose" and d.get("T_world_cam0") and d.get("status") == "tracking":
            t.append(int(d["t_ns"])); T.append(np.array(d["T_world_cam0"], float).reshape(4, 4))
    return np.array(t, np.int64), np.array(T)


def skew(v):
    z = np.zeros(v.shape[:-1])
    return np.stack([np.stack([z, -v[..., 2], v[..., 1]], -1), np.stack([v[..., 2], z, -v[..., 0]], -1),
                     np.stack([-v[..., 1], v[..., 0], z], -1)], -2)


def expso3(w):
    return cv2.Rodrigues(np.asarray(w, float).reshape(3, 1))[0]


def umeyama(A, B, with_scale=True):
    """B ~= s R A + t"""
    ma, mb = A.mean(0), B.mean(0)
    U, S, Vt = np.linalg.svd((B - mb).T @ (A - ma))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    s = float(np.trace(np.diag(S) @ D) / ((A - ma) ** 2).sum()) if with_scale else 1.0
    return s, R, mb - s * R @ ma


class UF:
    def __init__(self):
        self.p = {}

    def f(self, x):
        p = self.p
        p.setdefault(x, x)
        while p[x] != x:
            p[x] = p[p[x]]
            x = p[x]
        return x

    def u(self, a, b):
        a, b = self.f(a), self.f(b)
        if a != b:
            self.p[max(a, b)] = min(a, b)


def project(Rc, tc, X, K):
    Xc = np.einsum("nij,nj->ni", Rc, X) + tc
    return Xc, np.stack([K[0, 0] * Xc[:, 0] / Xc[:, 2] + K[0, 2], K[1, 1] * Xc[:, 1] / Xc[:, 2] + K[1, 2]], 1)


def ba_lm(R, t, X, cam, pt, uv, K, fixed=(0,), iters=30, huber=1.0, log=print, rs=None, tmr=None):
    """R,t: cam_from_world (N,3,3),(N,3); X (P,3); obs (cam idx, point idx, uv). Returns refined R,t,X, stats.
    tmr: optional dict, gets wall seconds per BA phase (gpuba 11-09 timers only, numbers unchanged)."""
    tmr = {} if tmr is None else tmr
    _tk = [time.time()]

    def tick(k):
        n_ = time.time(); tmr[k] = round(tmr.get(k, 0.0) + n_ - _tk[0], 3); _tk[0] = n_
    N, P = len(R), len(X)
    R, t, X = R.copy(), t.copy(), X.copy()
    fixed = set(fixed)
    free = np.array([i not in fixed for i in range(N)])
    cidx = -np.ones(N, int); cidx[free] = np.arange(free.sum())
    Nf = int(free.sum())
    order = np.argsort(pt, kind="stable")
    cam, pt, uv = cam[order], pt[order], uv[order]
    # rolling shutter: per observation fixed row-time motion, X_row = Rr X_mid + tr (identity when rs is None)
    if rs is None:
        Rr, tr = np.broadcast_to(np.eye(3), (len(cam), 3, 3)), np.zeros((len(cam), 3))
    else:
        Rr, tr = rs[0][order], rs[1][order]
    # obs pairs within the same point (for the Schur complement)
    starts = np.r_[0, np.flatnonzero(np.diff(pt)) + 1, len(pt)]
    pa, pb = [], []
    for s0, s1 in zip(starts[:-1], starts[1:]):
        idx = np.arange(s0, s1)
        a_, b_ = np.meshgrid(idx, idx, indexing="ij")
        pa.append(a_.ravel()); pb.append(b_.ravel())
    pa, pb = np.concatenate(pa), np.concatenate(pb)
    tmr["n_obs_pairs"] = int(len(pa))
    lam = 1e-3

    def cost(R, t, X):
        Xm = np.einsum("nij,nj->ni", R[cam], X[pt]) + t[cam]
        Xc, q = project(Rr, tr, Xm, K)
        r = (q - uv)
        e = np.linalg.norm(r, axis=1)
        w = np.where(e <= huber, 1.0, huber / np.maximum(e, 1e-12))
        rho = np.where(e <= huber, e ** 2, 2 * huber * e - huber ** 2)
        return r, e, w, rho.sum(), (Xm, Xc)

    r, e, w, c0, (Xm, Xc) = cost(R, t, X)
    tick("setup_s")
    hist = [dict(it=0, cost=float(c0), rmse_px=float(np.sqrt((e ** 2).mean())), med_px=float(np.median(e)))]
    for it in range(1, iters + 1):
        Z = Xc[:, 2]
        fx, fy = K[0, 0], K[1, 1]
        Jq = np.zeros((len(Z), 2, 3))
        Jq[:, 0, 0] = fx / Z; Jq[:, 0, 2] = -fx * Xc[:, 0] / Z ** 2
        Jq[:, 1, 1] = fy / Z; Jq[:, 1, 2] = -fy * Xc[:, 1] / Z ** 2
        JqR = Jq @ Rr
        Jc = np.concatenate([-JqR @ skew(Xm), JqR], 2)        # d q / d(dtheta, dt), left perturbation of cam_from_world
        Jp = JqR @ R[cam]                                      # d q / d X
        sw = w[:, None, None]
        Hcc_o = np.einsum("nki,nkj->nij", Jc, sw * Jc)
        Hcp_o = np.einsum("nki,nkj->nij", Jc, sw * Jp)
        Hpp_o = np.einsum("nki,nkj->nij", Jp, sw * Jp)
        gc_o = np.einsum("nki,nk->ni", Jc, w[:, None] * r)
        gp_o = np.einsum("nki,nk->ni", Jp, w[:, None] * r)
        Hpp = np.zeros((P, 3, 3)); np.add.at(Hpp, pt, Hpp_o)
        gp = np.zeros((P, 3)); np.add.at(gp, pt, gp_o)
        tick("jac_s")
        while True:
            tmr["n_solves"] = tmr.get("n_solves", 0) + 1
            Hpp_d = Hpp + lam * (np.einsum("pii->pi", Hpp)[:, :, None] * np.eye(3) + 1e-9 * np.eye(3))
            Hpp_inv = np.linalg.inv(Hpp_d)
            S = np.zeros((Nf * 6, Nf * 6)); b = np.zeros(Nf * 6)
            fo = free[cam]
            # camera-camera blocks
            Hcc = np.zeros((N, 6, 6)); np.add.at(Hcc, cam, Hcc_o)
            gcam = np.zeros((N, 6)); np.add.at(gcam, cam, gc_o)
            for i in np.flatnonzero(free):
                k = cidx[i] * 6
                D = Hcc[i] + lam * np.diag(np.diag(Hcc[i]) + 1e-9)
                S[k:k + 6, k:k + 6] += D
                b[k:k + 6] -= gcam[i]
            # - W Hpp^-1 W^T
            V = np.einsum("nij,njk->nik", Hcp_o, Hpp_inv[pt])      # (n,6,3)
            m = fo[pa] & fo[pb]
            Am, Bm = pa[m], pb[m]
            CH = 2_000_000                                          # obs pairs per chunk (RAM: long recordings > 30 GB unchunked)
            for c0_ in range(0, len(Am), CH):                       # one chunk (= unchanged sums) when < CH pairs
                A_, B_ = Am[c0_:c0_ + CH], Bm[c0_:c0_ + CH]
                blk = np.einsum("nij,nkj->nik", V[A_], Hcp_o[B_])      # (m,6,6)
                ri = (cidx[cam[A_]] * 6)[:, None, None] + np.arange(6)[None, :, None]
                ci = (cidx[cam[B_]] * 6)[:, None, None] + np.arange(6)[None, None, :]
                flat = (np.broadcast_to(ri, blk.shape) * (Nf * 6) + np.broadcast_to(ci, blk.shape)).ravel()
                S -= np.bincount(flat, weights=blk.ravel(), minlength=(Nf * 6) ** 2).reshape(Nf * 6, Nf * 6)
                del blk, ri, ci, flat
            # b += W Hpp^-1 gp
            vg = np.einsum("nij,nj->ni", V, gp[pt])
            bb = np.zeros((N, 6)); np.add.at(bb, cam, vg)
            for i in np.flatnonzero(free):
                k = cidx[i] * 6
                b[k:k + 6] += bb[i]
            tick("schur_s")
            try:
                dc = np.linalg.solve(S, b)
            except np.linalg.LinAlgError:
                lam *= 10; tick("solve_s"); continue
            dcam = np.zeros((N, 6)); dcam[free] = dc.reshape(-1, 6)
            tick("solve_s")
            # dp = Hpp^-1 (-gp - W^T dc)
            wt = np.einsum("nij,ni->nj", Hcp_o, dcam[cam])
            rhs = np.zeros((P, 3)); np.add.at(rhs, pt, wt)
            dp = np.einsum("pij,pj->pi", Hpp_inv, -gp - rhs)
            Rn = np.array([expso3(dcam[i, :3]) @ R[i] for i in range(N)])
            tn = np.array([expso3(dcam[i, :3]) @ t[i] + dcam[i, 3:] for i in range(N)])
            Xn = X + dp
            rn, en, wn, cn, Xcn = cost(Rn, tn, Xn)
            tick("update_cost_s")
            if cn < c0 and np.all(Xcn[1][:, 2] > 1e-3):
                R, t, X, r, e, w, (Xm, Xc) = Rn, tn, Xn, rn, en, wn, Xcn
                rel = (c0 - cn) / c0
                c0 = cn
                lam = max(lam / 3, 1e-7)
                break
            lam *= 4
            if lam > 1e8:
                rel = 0
                break
        hist.append(dict(it=it, cost=float(c0), lam=lam, rmse_px=float(np.sqrt((e ** 2).mean())), med_px=float(np.median(e)),
                         step_cam_max=float(np.abs(dcam).max()) if lam <= 1e8 else 0.0))
        log(f"[ba] it {it} cost {c0:.1f} med {np.median(e):.3f} px rmse {np.sqrt((e ** 2).mean()):.3f} lam {lam:.1e}")
        if lam > 1e8 or rel < 1e-7:
            break
    return R, t, X, dict(hist=hist, final_med_px=float(np.median(e)), final_rmse_px=float(np.sqrt((e ** 2).mean())),
                         frac_inlier_1px=float((e <= 1).mean()))


def tri_one(nodes, kfi, kps, Rcw, tcw, Pm, Kr, C, kfs, min_tri_deg):
    """scanA triangulation of one track (DLT with the VIO poses, drop obs > 4 px, up to 3 rounds) -> (X, ks, q) or None.
    Same code as the loop in main (kept there unchanged for the default path)."""
    ks = [kfi[n[0]] for n in nodes]; q = np.array([kps[n[0]][n[1]] for n in nodes])
    for _ in range(3):
        A = np.concatenate([q[:, :1] * Pm[ks, 2] - Pm[ks, 0], q[:, 1:2] * Pm[ks, 2] - Pm[ks, 1]], 0)
        Xh = np.linalg.svd(A)[2][-1]
        if abs(Xh[3]) < 1e-12:
            break
        X = Xh[:3] / Xh[3]
        Xc = np.einsum("nij,j->ni", Rcw[ks], X) + tcw[ks]
        e = np.linalg.norm(Xc[:, :2] / Xc[:, 2:] * Kr[0, 0] + Kr[:2, 2] - q, axis=1)
        good = (e < 4.0) & (Xc[:, 2] > 0.05)
        if good.all() or good.sum() < 2:
            break
        ks = [k for k, g_ in zip(ks, good) if g_]; q = q[good]
    if len(ks) < 2 or abs(Xh[3]) < 1e-12:
        return None
    Xc = np.einsum("nij,j->ni", Rcw[ks], X) + tcw[ks]
    e = np.linalg.norm(Xc[:, :2] / Xc[:, 2:] * Kr[0, 0] + Kr[:2, 2] - q, axis=1)
    if not ((e < 4.0) & (Xc[:, 2] > 0.05) & (Xc[:, 2] < 5.0)).all():
        return None
    cen = np.array([C[kfs[k]] for k in ks])
    rays = X - cen; rays /= np.linalg.norm(rays, axis=1, keepdims=True)
    ang = np.degrees(np.arccos(np.clip(np.min(rays @ rays.T), -1, 1)))
    if ang < min_tri_deg:
        return None
    return X, ks, q


def tracks_drop_fast(edges, kfs):
    """gpuba 11-09: the scanA 'drop' tracks without the Python union-find: connected components (scipy) of the same match
    graph; tracks and their nodes in the same order as kf_ba's UF dict (order of first appearance in the edge stream);
    tracks with two keypoints of one image dropped. Returns node lists [(kf, kp), ...] like the default path."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    if not edges:
        return []
    kfi = {i: n for n, i in enumerate(kfs)}
    KM = int(max(int(mm.max()) for *_, mm in edges)) + 1
    st = []
    for _, _, _, k1, k2, mm in edges:
        mm = mm.astype(np.int64)
        st.append(np.stack([kfi[k1] * KM + mm[:, 0], kfi[k2] * KM + mm[:, 1]], 1).ravel())
    S = np.concatenate(st)
    uniq, first, inv = np.unique(S, return_index=True, return_inverse=True)
    nU = len(uniq)
    g = coo_matrix((np.ones(len(S) // 2, np.int8), (inv[0::2], inv[1::2])), shape=(nU, nU))
    _, lab = connected_components(g, directed=False)
    ordn = np.argsort(first, kind="stable")            # nodes in UF insertion order
    lab_o = lab[ordn]
    _, gfirst = np.unique(lab_o, return_index=True)    # per label: position of its first node
    grank = np.empty(len(gfirst), np.int64); grank[np.argsort(gfirst, kind="stable")] = np.arange(len(gfirst))
    o2 = np.argsort(grank[lab_o], kind="stable")
    nodes_s = uniq[ordn][o2]; g_s = grank[lab_o][o2]
    bnd = np.r_[0, np.flatnonzero(np.diff(g_s)) + 1, len(g_s)]
    camk = nodes_s // KM; kp = nodes_s % KM
    tracks = []
    for s0, s1 in zip(bnd[:-1].tolist(), bnd[1:].tolist()):
        if s1 - s0 < 2:
            continue
        cc = camk[s0:s1]
        if len(np.unique(cc)) != len(cc):
            continue
        tracks.append([(kfs[c], x) for c, x in zip(cc.tolist(), kp[s0:s1].tolist())])
    return tracks


def tri_fast(tracks, kfi, kps, Rcw, tcw, Pm, Kr, C, kfs, min_tri_deg):
    """gpuba 11-09: tri_one for all tracks, batched by track length (same arithmetic per track, numpy stacked SVD);
    tracks that need a second round (some obs > 4 px but >= 2 good) go through tri_one itself. Returns per-track results."""
    res = [None] * len(tracks)
    if not tracks:
        return res
    Ls = np.array([len(t) for t in tracks])
    Cv = np.array([C[i] for i in kfs])
    for L in np.unique(Ls):
        ti = np.flatnonzero(Ls == L)
        ks = np.array([[kfi[n[0]] for n in tracks[t]] for t in ti])                    # (n,L)
        q = np.array([[kps[n[0]][n[1]] for n in tracks[t]] for t in ti])               # (n,L,2)
        Pk = Pm[ks]                                                                     # (n,L,3,4)
        A = np.concatenate([q[:, :, :1] * Pk[:, :, 2] - Pk[:, :, 0], q[:, :, 1:2] * Pk[:, :, 2] - Pk[:, :, 1]], 1)
        Xh = np.linalg.svd(A)[2][:, -1]                                                 # (n,4)
        degen = np.abs(Xh[:, 3]) < 1e-12
        X = Xh[:, :3] / np.where(degen, 1.0, Xh[:, 3])[:, None]
        Xc = np.einsum("nlij,nj->nli", Rcw[ks], X) + tcw[ks]
        e = np.linalg.norm(Xc[:, :, :2] / Xc[:, :, 2:] * Kr[0, 0] + Kr[:2, 2] - q, axis=2)
        good = (e < 4.0) & (Xc[:, :, 2] > 0.05)
        allg = good.all(1)
        again = ~degen & ~allg & (good.sum(1) >= 2)
        ok = ~degen & allg & (Xc[:, :, 2] < 5.0).all(1)
        rays = X[:, None, :] - Cv[ks]; rays /= np.linalg.norm(rays, axis=2, keepdims=True)
        with np.errstate(invalid="ignore"):
            ang = np.degrees(np.arccos(np.clip((rays @ np.transpose(rays, (0, 2, 1))).min((1, 2)), -1, 1)))
        ok &= ang >= min_tri_deg
        for j in np.flatnonzero(ok):
            res[ti[j]] = (X[j], ks[j].tolist(), q[j])
        for j in np.flatnonzero(again):
            res[ti[j]] = tri_one(tracks[ti[j]], kfi, kps, Rcw, tcw, Pm, Kr, C, kfs, min_tri_deg)
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec"); ap.add_argument("--vio", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--gt", default="imu_gt_spl")
    ap.add_argument("--kf-step", type=int, default=10)
    ap.add_argument("--scale", type=float, default=0.5)
    ap.add_argument("--seq", type=int, default=6, help="match each keyframe with the next N keyframes")
    ap.add_argument("--cov-k", type=int, default=10, help="extra VIO-covisible partners per keyframe")
    ap.add_argument("--cov-angle", type=float, default=30.0)
    ap.add_argument("--max-features", type=int, default=6000)
    ap.add_argument("--mask-board", action="store_true")
    ap.add_argument("--iters", type=int, default=40)
    ap.add_argument("--min-tri-deg", type=float, default=1.0)
    ap.add_argument("--rs-skew-ms", type=float, default=0.0, help="rolling-shutter readout (6.54 ms on lens 5); 0 = global shutter model")
    ap.add_argument("--twist", default="", help="poses for the per-keyframe camera twist used by --rs-skew-ms (default --vio)")
    ap.add_argument("--kf-from-vio", action="store_true",
                    help="HS 240 fps recordings (VIO fed at 60 fps): keyframes = every --kf-step-th VIO-fed frame (not every "
                         "kf-step-th recording row); sequential/covisible gaps counted in keyframes")
    ap.add_argument("--match-ms", type=float, default=4.0, help="VIO t_ns vs GT row t_ns tolerance (use 1 for 240 fps rows)")
    # featloop 2026-11-05 (defaults = scanA behaviour)
    ap.add_argument("--feat", default="sift", choices=["sift", "xfeat_lg"],
                    help="sift = pycolmap CUDA SIFT + SIFT matcher (scanA); xfeat_lg = XFeat top-k + LighterGlue (PyTorch GPU), "
                         "matches written into the same COLMAP database and verified by the same pycolmap.verify_matches")
    ap.add_argument("--xf-topk", type=int, default=4096)
    ap.add_argument("--xf-subpix", type=int, default=0)
    ap.add_argument("--lg-conf", type=float, default=0.1)
    ap.add_argument("--loop", default="none", choices=["none", "salad"],
                    help="salad: extra appearance pairs (SALAD global descriptor retrieval), each keyframe vs its --loop-k most "
                         "similar keyframes that are not time neighbours (> --seq keyframes apart) and not already paired")
    ap.add_argument("--loop-k", type=int, default=5)
    ap.add_argument("--loop-min-sim", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=-1, help="RANSAC random_seed of the two-view verification (-1 = COLMAP default = "
                    "not repeatable run to run; featloop found inlier sets differ on 1838/1844 pairs between identical runs)")
    ap.add_argument("--db-from", default="", help="reuse features + raw matches (db.db, pairs.txt, loop_pairs.txt) of an earlier run dir: "
                    "only the verification (with --seed) and BA are redone, on the CPU, no GPU lock")
    ap.add_argument("--keep-db", action="store_true", help="keep OUT/db.db (for --db-from repeats)")
    ap.add_argument("--tracks", default="drop", choices=["drop", "greedy"],
                    help="drop = scanA (union-find over all verified inliers, drop every track that holds two keypoints of one "
                         "image); greedy = featloop 11-06: conflict-free union-find, pairs in order of inlier count (high first), "
                         "an edge is skipped when it would put two keypoints of one image into one track")
    ap.add_argument("--ba-device", default="cpu", choices=["cpu", "cuda"],
                    help="gpuba 11-09: cpu = numpy ba_lm (scanA); cuda = same LM / Schur steps in torch float64 on the GPU (ba_torch.py), "
                         "GPU lock held through the BA")
    ap.add_argument("--fast-tracks", action="store_true", help="gpuba 11-09: drop-tracks by scipy connected components + "
                    "length-batched triangulation (same tracks, same order, same per-track arithmetic as the default path)")
    ap.add_argument("--match-device", default="auto", choices=["auto", "cpu"], help="gpuba 11-11: SIFT matcher device")
    ap.add_argument("--sort-matches", action="store_true", help="gpuba 11-11: sort each pair's inlier matches before building tracks "
                    "(track / point order independent of the order the matcher returns)")
    ap.add_argument("--sift-device", default="auto", choices=["auto", "cpu"],
                    help="auto = pycolmap CUDA SIFT when available (scanA); cpu = pycolmap CPU SIFT (gpuba 11-11, deterministic run to run)")
    ap.add_argument("--save-tri", action="store_true", help="write OUT/tri_dump.npz (points, obs) after triangulation and go on")
    ap.add_argument("--dump-tri", action="store_true", help="write OUT/tri_dump.npz (points, obs) after triangulation and stop")
    ap.add_argument("--cache-root", default="", help="keyframe image cache root (default REC/scanA_cache)")
    a = ap.parse_args()
    import pycolmap
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import gpu_guard
    t0 = time.time()
    tm = {}
    rec, out = Path(a.rec).resolve(), Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    logf = open(out / "ba.log", "w")

    def log(s):
        print(s, flush=True); logf.write(s + "\n"); logf.flush()

    g = np.load(rec / a.gt / "gt.npz")
    t_ns, ok_gt = g["t_ns"], g["ok"]
    s = a.scale
    K = g["K"].copy(); K[:2] *= s
    dist = g["dist"].astype(np.float64)
    W0, H0 = int(round(1920 * s)), int(round(1080 * s))
    Kr = np.array([[K[0, 0], 0, K[0, 2]], [0, K[0, 0], K[1, 2]], [0, 0, 1.0]])   # = motion_stereo.K_of(f, cx, cy)
    und = cv2.initUndistortRectifyMap(K, dist, np.eye(3), Kr, (W0, H0), cv2.CV_32FC1)
    vt, vT = load_vio(a.vio)
    idx = np.searchsorted(vt, t_ns).clip(1, len(vt) - 1)
    idx = np.where(np.abs(vt[idx - 1] - t_ns) < np.abs(vt[idx] - t_ns), idx - 1, idx)
    has = np.abs(vt[idx] - t_ns) < a.match_ms * 1e6
    if a.kf_from_vio:
        kfs = [int(i) for i in np.flatnonzero(has)[::a.kf_step]]
    else:
        kfs = [i for i in range(0, len(t_ns), a.kf_step) if has[i]]
    Tv = {i: vT[idx[i]] for i in kfs}
    log(f"[kf] {len(kfs)} keyframes (step {a.kf_step}) with VIO tracking poses, frames {kfs[0]}..{kfs[-1]}")
    # board quad (for --mask-board)
    bj = json.loads(Path(str(g["board_json"])).read_text())
    sq = float(g["square_mm"]) / 1000.0
    BW, BH = bj["squares_x"] * sq, bj["squares_y"] * sq
    tag = f"kf{a.kf_step}_s{s:g}" + ("_mask" if a.mask_board else "") + (f"_vio_{a.gt}" if a.kf_from_vio else "")
    img_dir = (Path(a.cache_root) / rec.name if a.cache_root else rec / "scanA_cache") / tag
    img_dir.mkdir(parents=True, exist_ok=True)
    names = {i: f"{i:05d}.jpg" for i in kfs}
    need = [i for i in kfs if not (img_dir / names[i]).exists()]
    if need:
        need_s = set(need)
        cap = cv2.VideoCapture(str(rec / "cam5.mp4"))
        fi = 0
        while need_s:
            r_, f = cap.read()
            if not r_:
                break
            if fi in need_s:
                im = cv2.remap(cv2.resize(f, (W0, H0), interpolation=cv2.INTER_AREA), *und, cv2.INTER_LINEAR)
                if a.mask_board and ok_gt[fi]:
                    m = 0.02
                    corners = np.array([[-m, -m, 0], [BW + m, -m, 0], [BW + m, BH + m, 0], [-m, BH + m, 0]])
                    Rbc, pbc = g["R_bc"][fi], g["p_bc"][fi]
                    Xc = (corners - pbc) @ Rbc            # board -> cam: R_bc^T (X - p)
                    if np.all(Xc[:, 2] > 0.05):
                        q = (Xc[:, :2] / Xc[:, 2:]) * Kr[0, 0] + Kr[:2, 2]
                        cv2.fillConvexPoly(im, np.round(q).astype(np.int32), (128, 128, 128))
                cv2.imwrite(str(img_dir / names[fi]), im, [cv2.IMWRITE_JPEG_QUALITY, 95])
                need_s.discard(fi)
            fi += 1
        cap.release()
    log(f"[img] {len(kfs)} images in {img_dir} ({len(need)} new) {time.time() - t0:.0f}s")
    tm["images_s"] = round(time.time() - t0, 1)
    import contextlib, shutil
    gpu_job = gpu_guard.job(f"kf_ba(featloop) {out}", out_dir=str(out), torch_cap=True) if not a.db_from else contextlib.nullcontext()
    gpu_job.__enter__()                            # GPU lock only for features/matching
    _GJ[0] = gpu_job
    tg = time.time()

    db_path = out / "db.db"
    if a.db_from:
        shutil.copy(Path(a.db_from) / "db.db", db_path)
    else:
        # ---- pycolmap features + matches
        db_path = out / "db.db"
        if db_path.exists():
            db_path.unlink()
        eo = pycolmap.FeatureExtractionOptions()
        eo.sift.max_num_features = a.max_features if a.feat == "sift" else 16    # xfeat_lg: SIFT pass only creates camera/image rows
        eo.max_image_size = max(W0, H0)
        if a.sift_device == "cpu":                   # gpuba 11-11: CPU SIFT (repeatable; CUDA SIFT on the RTX 5080 is not)
            eo.use_gpu = False
        ro = pycolmap.ImageReaderOptions()
        ro.camera_model = "PINHOLE"
        ro.camera_params = f"{Kr[0, 0]:.10g},{Kr[1, 1]:.10g},{Kr[0, 2] + 0.5:.10g},{Kr[1, 2] + 0.5:.10g}"
        pycolmap.extract_features(db_path, img_dir, image_names=[names[i] for i in kfs], camera_mode=pycolmap.CameraMode.SINGLE,
                                  reader_options=ro, extraction_options=eo,
                                  device=pycolmap.Device.cpu if a.sift_device == "cpu" else pycolmap.Device.auto)
        xfeats = None
        if a.feat == "xfeat_lg":
            sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "featloop"))
            import feats as FL
            xm = FL.XFLG(topk=a.xf_topk, lg_conf=a.lg_conf, subpix=a.xf_subpix)
            db = pycolmap.Database.open(str(db_path))
            ims0 = {im.name: im.image_id for im in db.read_all_images()}
            db.clear_keypoints(); db.clear_descriptors()
            xfeats = {}
            for i in kfs:
                gr = cv2.imread(str(img_dir / names[i]), cv2.IMREAD_GRAYSCALE)
                xfeats[i] = xm.extract(gr)
                db.write_keypoints(ims0[names[i]], (xfeats[i]["kp"] + 0.5).astype(np.float32))   # OpenCV 0 -> COLMAP centre 0.5
            db.close()
            log(f"[xfeat] {len(kfs)} images, keypoints median {np.median([len(v['kp']) for v in xfeats.values()]):.0f}, "
                f"ms/img median {np.median(xm.ms_f):.1f}")
    tm["features_s"] = round(time.time() - tg, 1)
    C = {i: Tv[i][:3, 3] for i in kfs}
    Zax = {i: Tv[i][:3, 2] for i in kfs}
    pairs = set()
    kpos = {i: n for n, i in enumerate(kfs)}
    for k, i in enumerate(kfs):
        for j in kfs[k + 1:k + 1 + a.seq]:
            pairs.add((i, j))
        cands = []
        for j in kfs:
            if (abs(kpos[j] - kpos[i]) <= a.seq) if a.kf_from_vio else (abs(j - i) <= a.seq * a.kf_step):
                continue
            ang = np.degrees(np.arccos(np.clip(Zax[i] @ Zax[j], -1, 1)))
            d = np.linalg.norm(C[i] - C[j])
            if ang < a.cov_angle and d < 0.6:
                cands.append((ang / a.cov_angle + d / 0.6, j))
        for _, j in sorted(cands)[:a.cov_k]:
            pairs.add((min(i, j), max(i, j)))
    n_base_pairs = len(pairs)
    loop_pairs = set()
    loop_info = {}
    if a.loop == "salad" and a.db_from:
        for l_ in (Path(a.db_from) / "loop_pairs.txt").read_text().splitlines():
            if l_.strip():
                x_, y_ = l_.split(); loop_pairs.add((int(x_[:-4]), int(y_[:-4])))
        pairs |= loop_pairs
        loop_info = json.loads((Path(a.db_from) / "ba.json").read_text()).get("loop", {}) if (Path(a.db_from) / "ba.json").exists() else {}
    elif a.loop == "salad":
        t2 = time.time()
        sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "featloop"))
        import feats as FL
        Dg = FL.salad_descriptors([cv2.imread(str(img_dir / names[i]), cv2.IMREAD_GRAYSCALE) for i in kfs])
        Sg = Dg @ Dg.T
        np.save(out / "salad_sim.npy", Sg.astype(np.float16))
        sims = []
        for k, i in enumerate(kfs):
            order_ = np.argsort(-Sg[k])
            got = 0
            for q in order_:
                if got >= a.loop_k or Sg[k, q] < a.loop_min_sim:
                    break
                j = kfs[q]
                if abs(int(q) - k) <= a.seq:
                    continue
                pr = (min(i, j), max(i, j))
                if pr in pairs:
                    continue
                if pr not in loop_pairs:
                    loop_pairs.add(pr); sims.append(float(Sg[k, q]))
                got += 1
        pairs |= loop_pairs
        # how far apart (VIO) are the loop pairs: centre distance and view angle
        dd = [float(np.linalg.norm(C[x] - C[y])) for x, y in loop_pairs]
        aa = [float(np.degrees(np.arccos(np.clip(Zax[x] @ Zax[y], -1, 1)))) for x, y in loop_pairs]
        loop_info = dict(n_loop_pairs=len(loop_pairs), sim_median=float(np.median(sims)) if sims else None,
                         vio_dist_m_median=float(np.median(dd)) if dd else None, vio_angle_deg_median=float(np.median(aa)) if aa else None,
                         frac_outside_cov_gate=float(np.mean([(d_ >= 0.6) or (a_ >= a.cov_angle) for d_, a_ in zip(dd, aa)])) if dd else None,
                         salad_s=round(time.time() - t2, 1))
        log(f"[loop] SALAD {len(loop_pairs)} extra pairs (k {a.loop_k}), {loop_info}")
    pl = out / "pairs.txt"
    pl.write_text("\n".join(f"{names[x]} {names[y]}" for x, y in sorted(pairs)) + "\n")
    (out / "loop_pairs.txt").write_text("".join(f"{names[x]} {names[y]}\n" for x, y in sorted(loop_pairs)))
    if a.db_from:
        assert pl.read_text() == (Path(a.db_from) / "pairs.txt").read_text(), "pairs differ from --db-from run"
    vopt = pycolmap.TwoViewGeometryOptions(); vopt.ransac.random_seed = a.seed
    t3 = time.time()
    n_raw = {}
    if a.db_from:
        db = pycolmap.Database.open(str(db_path)); db.clear_two_view_geometries(); db.close()
        pycolmap.verify_matches(str(db_path), str(pl), vopt)
        n_raw = {tuple(map(int, k.split())): v for k, v in json.loads((Path(a.db_from) / "ba.json").read_text()).get("n_raw", {}).items()} if (Path(a.db_from) / "ba.json").exists() else {}
    elif a.feat == "sift":
        mo = pycolmap.FeatureMatchingOptions()
        po = pycolmap.ImportedPairingOptions(); po.match_list_path = str(pl)
        if a.match_device == "cpu":
            mo.use_gpu = False
        pycolmap.match_image_pairs(str(db_path), matching_options=mo, pairing_options=po, verification_options=vopt,
                                   device=pycolmap.Device.cpu if a.match_device == "cpu" else pycolmap.Device.auto)
    else:
        db = pycolmap.Database.open(str(db_path))
        for x, y in sorted(pairs):
            m_ = xm.match(xfeats[x], xfeats[y])
            n_raw[(x, y)] = len(m_)
            db.write_matches(ims0[names[x]], ims0[names[y]], m_)
        db.close()
        log(f"[lg] {len(pairs)} pairs, raw matches median {np.median(list(n_raw.values())):.0f}, ms/pair median {np.median(xm.ms_m):.1f}")
        pycolmap.verify_matches(str(db_path), str(pl), vopt)
        del xm, xfeats
    tm["matching_s"] = round(time.time() - t3, 1)
    log(f"[match] {len(pairs)} pairs {time.time() - t0:.0f}s")
    if a.ba_device == "cpu":
        _GJ[0] = None
        gpu_job.__exit__(None, None, None)            # GPU free from here (BA = CPU)
    tm["gpu_stage_s"] = round(time.time() - tg, 1)
    t4 = time.time()
    db = pycolmap.Database.open(str(db_path))
    ims = {im.name: im.image_id for im in db.read_all_images()}
    id2kf = {ims[names[i]]: i for i in kfs}
    kps = {}
    for i in kfs:
        kp = np.asarray(db.read_keypoints(ims[names[i]]))[:, :2] - 0.5      # COLMAP pixel centre 0.5 -> OpenCV 0
        kps[i] = kp
    ids, geoms = db.read_two_view_geometries()
    db.close()
    uf = UF()
    n_inl = []
    inl_of = {}
    edges = []
    for pid, gm in zip(ids, geoms):
        i1, i2 = pid if isinstance(pid, tuple) else pycolmap.pair_id_to_image_pair(pid)
        mm = np.asarray(gm.inlier_matches)
        if a.sort_matches and len(mm):              # gpuba 11-11: the GPU matcher can return inliers in run-dependent order
            mm = mm[np.lexsort((mm[:, 1], mm[:, 0]))]
        k1, k2 = id2kf[i1], id2kf[i2]
        inl_of[(min(k1, k2), max(k1, k2))] = len(mm)
        if len(mm) < 15:
            continue
        n_inl.append(len(mm))
        edges.append((len(mm), min(k1, k2), max(k1, k2), k1, k2, mm))
    n_skip = 0
    if a.tracks == "drop" and a.fast_tracks:
        pass                                        # tracks_drop_fast below
    elif a.tracks == "drop":
        for _, _, _, k1, k2, mm in edges:
            for x, y in mm:
                uf.u((k1, int(x)), (k2, int(y)))
    else:
        cimg = {}                                   # root -> set of keyframes in the track
        for _, _, _, k1, k2, mm in sorted(edges, key=lambda e: (-e[0], e[1], e[2])):
            for x, y in mm:
                ra, rb = uf.f((k1, int(x))), uf.f((k2, int(y)))
                if ra == rb:
                    continue
                sa, sb = cimg.get(ra, {k1}), cimg.get(rb, {k2})
                if sa & sb:
                    n_skip += 1
                    continue
                uf.u(ra, rb)
                cimg[uf.f(ra)] = sa | sb
        log(f"[tracks] greedy: {n_skip} conflicting match edges skipped")
    tm["read_db_uf_s"] = round(time.time() - t4, 1)
    groups = {}
    for node in list(uf.p):
        groups.setdefault(uf.f(node), []).append(node)
    kfi = {i: n for n, i in enumerate(kfs)}
    tracks = tracks_drop_fast(edges, kfs) if (a.tracks == "drop" and a.fast_tracks) else []
    for nodes in groups.values():
        if len(nodes) < 2:
            continue
        cams_ = [n[0] for n in nodes]
        if len(set(cams_)) != len(cams_):          # two keypoints of one image in one track: drop the track
            continue
        tracks.append(nodes)
    tm["tracks_s"] = round(time.time() - t4, 1)
    log(f"[tracks] verified pairs {len(n_inl)} (inliers median {np.median(n_inl) if n_inl else 0:.0f}), tracks {len(tracks)}")
    # ---- triangulate with VIO poses (cam_from_world)
    Rcw = np.array([Tv[i][:3, :3].T for i in kfs]); tcw = np.array([-Tv[i][:3, :3].T @ Tv[i][:3, 3] for i in kfs])
    Pm = np.array([Kr @ np.hstack([Rcw[k], tcw[k][:, None]]) for k in range(len(kfs))])
    Xs, cam_l, pt_l, uv_l = [], [], [], []
    if a.fast_tracks:
        for r_ in tri_fast(tracks, kfi, kps, Rcw, tcw, Pm, Kr, C, kfs, a.min_tri_deg):
            if r_ is None:
                continue
            X, ks, q = r_
            p = len(Xs); Xs.append(X)
            cam_l += ks; pt_l += [p] * len(ks); uv_l.append(q)
    for nodes in (tracks if not a.fast_tracks else []):
        ks = [kfi[n[0]] for n in nodes]; q = np.array([kps[n[0]][n[1]] for n in nodes])
        for _ in range(3):
            A = np.concatenate([q[:, :1] * Pm[ks, 2] - Pm[ks, 0], q[:, 1:2] * Pm[ks, 2] - Pm[ks, 1]], 0)
            Xh = np.linalg.svd(A)[2][-1]
            if abs(Xh[3]) < 1e-12:
                break
            X = Xh[:3] / Xh[3]
            Xc = np.einsum("nij,j->ni", Rcw[ks], X) + tcw[ks]
            e = np.linalg.norm(Xc[:, :2] / Xc[:, 2:] * Kr[0, 0] + Kr[:2, 2] - q, axis=1)
            good = (e < 4.0) & (Xc[:, 2] > 0.05)
            if good.all() or good.sum() < 2:
                break
            ks = [k for k, g_ in zip(ks, good) if g_]; q = q[good]
        else:
            pass
        if len(ks) < 2 or abs(Xh[3]) < 1e-12:
            continue
        Xc = np.einsum("nij,j->ni", Rcw[ks], X) + tcw[ks]
        e = np.linalg.norm(Xc[:, :2] / Xc[:, 2:] * Kr[0, 0] + Kr[:2, 2] - q, axis=1)
        if not ((e < 4.0) & (Xc[:, 2] > 0.05) & (Xc[:, 2] < 5.0)).all():
            continue
        cen = np.array([C[kfs[k]] for k in ks])
        rays = X - cen; rays /= np.linalg.norm(rays, axis=1, keepdims=True)
        ang = np.degrees(np.arccos(np.clip(np.min(rays @ rays.T), -1, 1)))
        if ang < a.min_tri_deg:
            continue
        p = len(Xs); Xs.append(X)
        cam_l += ks; pt_l += [p] * len(ks); uv_l.append(q)
    X0 = np.array(Xs); cam = np.array(cam_l); pt = np.array(pt_l); uv = np.concatenate(uv_l)
    tm["tri_s"] = round(time.time() - t4 - tm["tracks_s"], 1)
    if a.dump_tri or a.save_tri:
        np.savez(out / "tri_dump.npz", X0=X0, cam=cam, pt=pt, uv=uv, n_tracks=len(tracks))
        log(f"[dump] {out / 'tri_dump.npz'} points {len(X0)} obs {len(cam)} tracks {len(tracks)} timing {tm}")
        if a.dump_tri:
            return
    obs_per_kf = np.bincount(cam, minlength=len(kfs))
    # featloop: verified pairs / inliers split base vs loop, and points that link a loop pair (both keyframes observe it)
    vb = [inl_of.get(p_, 0) for p_ in pairs if p_ not in loop_pairs]
    vl = [inl_of.get(p_, 0) for p_ in loop_pairs]
    pair_stats = dict(base_pairs=n_base_pairs, base_verified=int(sum(v >= 15 for v in vb)),
                      base_inliers_median=float(np.median([v for v in vb if v >= 15])) if any(v >= 15 for v in vb) else 0.0,
                      loop_pairs=len(loop_pairs), loop_verified=int(sum(v >= 15 for v in vl)),
                      loop_inliers_median=float(np.median([v for v in vl if v >= 15])) if any(v >= 15 for v in vl) else 0.0,
                      inliers_total=int(sum(v for v in inl_of.values() if v >= 15)),
                      raw_matches_median=float(np.median(list(n_raw.values()))) if n_raw else None)
    if loop_pairs:
        lp_k = {(kfi[x], kfi[y]) for x, y in loop_pairs}
        o_ = np.argsort(pt, kind="stable"); cs_, ps_ = cam[o_], pt[o_]
        st_ = np.r_[0, np.flatnonzero(np.diff(ps_)) + 1, len(ps_)]
        n_lp_pts = n_lp_obs = 0
        for s0, s1 in zip(st_[:-1], st_[1:]):
            cc = sorted(cs_[s0:s1])
            if any((cc[u], cc[v]) in lp_k for u in range(len(cc)) for v in range(u + 1, len(cc))):
                n_lp_pts += 1; n_lp_obs += s1 - s0
        pair_stats.update(loop_linked_points=n_lp_pts, loop_linked_obs=int(n_lp_obs))
    log(f"[pairs] {pair_stats}")
    log(f"[tri] points {len(X0)}, obs {len(cam)}, track len mean {len(cam) / max(1, len(X0)):.2f}, "
        f"obs/kf min {obs_per_kf.min()} median {np.median(obs_per_kf):.0f} {time.time() - t0:.0f}s")
    # keyframes without observations cannot be refined: keep VIO pose (counted)
    # ---- rolling shutter: camera twist per keyframe from neighbour-frame poses (i-1, i+1), fixed during BA
    rs = None
    if a.rs_skew_ms > 0:
        wt_, wT_ = load_vio(a.twist or a.vio)
        def near(tq):
            k = np.searchsorted(wt_, tq).clip(1, len(wt_) - 1)
            k = k - 1 if abs(wt_[k - 1] - tq) < abs(wt_[k] - tq) else k
            return k if abs(wt_[k] - tq) < 4e6 else None
        tw = np.zeros((len(kfs), 6)); n_tw = 0
        for k, i in enumerate(kfs):
            ka, kb = near(t_ns[max(i - 1, 0)]), near(t_ns[min(i + 1, len(t_ns) - 1)])
            if ka is None or kb is None or ka == kb:
                continue
            D = np.linalg.inv(wT_[ka]) @ wT_[kb]; dtt = (wt_[kb] - wt_[ka]) * 1e-9
            tw[k, :3] = cv2.Rodrigues(D[:3, :3])[0].ravel() / dtt; tw[k, 3:] = D[:3, 3] / dtt; n_tw += 1
        dtr = (uv[:, 1] / H0 - 0.5) * a.rs_skew_ms * 1e-3
        Rd = np.array([expso3(tw[c, :3] * d) for c, d in zip(cam, dtr)])
        td = tw[cam, 3:] * dtr[:, None]
        rs = (np.transpose(Rd, (0, 2, 1)), -np.einsum("nji,nj->ni", Rd, td))
        log(f"[rs] skew {a.rs_skew_ms} ms, twist from {a.twist or a.vio} for {n_tw}/{len(kfs)} kf, "
            f"|w| median {np.degrees(np.median(np.linalg.norm(tw[:, :3], axis=1))):.1f} deg/s, |v| median {np.median(np.linalg.norm(tw[:, 3:], axis=1)):.3f} m/s")
    # ---- BA
    fixed = [int(np.argmax(obs_per_kf[:5]))] + [k for k in range(len(kfs)) if obs_per_kf[k] < 15]
    tm["tracks_tri_s"] = round(time.time() - t4, 1)
    t5 = time.time()
    tba = {}
    if a.ba_device == "cuda":
        if a.db_from:                              # --db-from runs had no GPU lock so far: take it for the BA
            gpu_job = gpu_guard.job(f"kf_ba(gpuba) {out}", out_dir=str(out), torch_cap=True); gpu_job.__enter__(); _GJ[0] = gpu_job
        import ba_torch
        R1, t1, X1, st = ba_torch.ba_lm_torch(Rcw, tcw, X0, cam, pt, uv, Kr, fixed=fixed, iters=a.iters, log=log, rs=rs, tmr=tba)
        _GJ[0] = None
        gpu_job.__exit__(None, None, None)
    else:
        R1, t1, X1, st = ba_lm(Rcw, tcw, X0, cam, pt, uv, Kr, fixed=fixed, iters=a.iters, log=log, rs=rs, tmr=tba)
    tm["ba_phases"] = tba
    # ---- Sim(3) BA centres -> VIO centres (free keyframes only)
    Cb = np.array([-R1[k].T @ t1[k] for k in range(len(kfs))])
    Cv = np.array([C[i] for i in kfs])
    fr = np.array([k not in set(fixed) or k == fixed[0] for k in range(len(kfs))])
    tm["ba_s"] = round(time.time() - t5, 1)
    sc, Ra, ta = umeyama(Cb[fr], Cv[fr])
    res = np.linalg.norm((sc * (Ra @ Cb.T).T + ta) - Cv, axis=1)
    log(f"[sim3] scale {sc:.5f}, BA-vs-VIO centre rms {np.sqrt((res[fr] ** 2).mean()) * 100:.2f} cm")
    Tba = {}
    for k, i in enumerate(kfs):
        Twc = np.eye(4)
        Twc[:3, :3] = Ra @ R1[k].T
        Twc[:3, 3] = sc * Ra @ Cb[k] + ta
        Tba[i] = Twc if fr[k] else Tv[i]
    for nm, TT in (("poses_ba.jsonl.gz", Tba), ("poses_vio_kf.jsonl.gz", Tv)):
        with gzip.open(out / nm, "wt") as f:
            for i in kfs:
                f.write(json.dumps(dict(type="pose", status="tracking", t_ns=int(t_ns[i]), frame=i,
                                        T_world_cam0=TT[i].ravel().tolist())) + "\n")
    info = dict(made=time.strftime("%F %T"), rec=str(rec), vio=str(Path(a.vio).resolve()), args=vars(a), n_kf=len(kfs),
                img_dir=str(img_dir), Kr=Kr.tolist(), pairs=len(pairs), verified_pairs=len(n_inl), tracks=len(tracks),
                points=int(len(X0)), obs=int(len(cam)), fixed_kf=[kfs[k] for k in fixed],
                kf_obs_min=int(obs_per_kf.min()), sim3_scale=sc, sim3_rms_cm=float(np.sqrt((res[fr] ** 2).mean()) * 100),
                runtime_s=round(time.time() - t0, 1), timing=tm, pair_stats=pair_stats, loop=loop_info,
                points_per_kf_median=float(np.median(obs_per_kf)), n_raw={f"{x} {y}": v for (x, y), v in n_raw.items()}, track_edges_skipped=n_skip, **{k: v for k, v in st.items() if k != "hist"}, hist=st["hist"])
    (out / "ba.json").write_text(json.dumps(info, indent=1))
    if not a.keep_db and db_path.exists():
        db_path.unlink()
    log(json.dumps({k: v for k, v in info.items() if k != "hist"}))


def _out_arg():
    import sys as _s
    return _s.argv[_s.argv.index("--out") + 1] if "--out" in _s.argv else None


_GJ = [None]   # featloop: the gpu_guard job is held only around features + matching (BA on CPU runs without the lock)

if __name__ == "__main__":
    try:
        main()
    except BaseException as e:
        if _GJ[0] is not None:     # OOM -> gpu_guard clean exit (FAILED_OOM.json, rc 3); other errors re-raised, lock released
            _GJ[0].__exit__(type(e), e, e.__traceback__)
        raise
