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


def ba_lm(R, t, X, cam, pt, uv, K, fixed=(0,), iters=30, huber=1.0, log=print, rs=None):
    """R,t: cam_from_world (N,3,3),(N,3); X (P,3); obs (cam idx, point idx, uv). Returns refined R,t,X, stats."""
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
        while True:
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
            A_, B_ = pa[m], pb[m]
            blk = np.einsum("nij,nkj->nik", V[A_], Hcp_o[B_])      # (m,6,6)
            ri = (cidx[cam[A_]] * 6)[:, None, None] + np.arange(6)[None, :, None]
            ci = (cidx[cam[B_]] * 6)[:, None, None] + np.arange(6)[None, None, :]
            flat = (np.broadcast_to(ri, blk.shape) * (Nf * 6) + np.broadcast_to(ci, blk.shape)).ravel()
            S -= np.bincount(flat, weights=blk.ravel(), minlength=(Nf * 6) ** 2).reshape(Nf * 6, Nf * 6)
            # b += W Hpp^-1 gp
            vg = np.einsum("nij,nj->ni", V, gp[pt])
            bb = np.zeros((N, 6)); np.add.at(bb, cam, vg)
            for i in np.flatnonzero(free):
                k = cidx[i] * 6
                b[k:k + 6] += bb[i]
            try:
                dc = np.linalg.solve(S, b)
            except np.linalg.LinAlgError:
                lam *= 10; continue
            dcam = np.zeros((N, 6)); dcam[free] = dc.reshape(-1, 6)
            # dp = Hpp^-1 (-gp - W^T dc)
            wt = np.einsum("nij,ni->nj", Hcp_o, dcam[cam])
            rhs = np.zeros((P, 3)); np.add.at(rhs, pt, wt)
            dp = np.einsum("pij,pj->pi", Hpp_inv, -gp - rhs)
            Rn = np.array([expso3(dcam[i, :3]) @ R[i] for i in range(N)])
            tn = np.array([expso3(dcam[i, :3]) @ t[i] + dcam[i, 3:] for i in range(N)])
            Xn = X + dp
            rn, en, wn, cn, Xcn = cost(Rn, tn, Xn)
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
    a = ap.parse_args()
    import pycolmap
    t0 = time.time()
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
    has = np.abs(vt[idx] - t_ns) < 4e6
    kfs = [i for i in range(0, len(t_ns), a.kf_step) if has[i]]
    Tv = {i: vT[idx[i]] for i in kfs}
    log(f"[kf] {len(kfs)} keyframes (step {a.kf_step}) with VIO tracking poses, frames {kfs[0]}..{kfs[-1]}")
    # board quad (for --mask-board)
    bj = json.loads(Path(str(g["board_json"])).read_text())
    sq = float(g["square_mm"]) / 1000.0
    BW, BH = bj["squares_x"] * sq, bj["squares_y"] * sq
    tag = f"kf{a.kf_step}_s{s:g}" + ("_mask" if a.mask_board else "")
    img_dir = rec / "scanA_cache" / tag
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

    # ---- pycolmap features + matches
    db_path = out / "db.db"
    if db_path.exists():
        db_path.unlink()
    eo = pycolmap.FeatureExtractionOptions()
    eo.sift.max_num_features = a.max_features
    eo.max_image_size = max(W0, H0)
    ro = pycolmap.ImageReaderOptions()
    ro.camera_model = "PINHOLE"
    ro.camera_params = f"{Kr[0, 0]:.10g},{Kr[1, 1]:.10g},{Kr[0, 2] + 0.5:.10g},{Kr[1, 2] + 0.5:.10g}"
    pycolmap.extract_features(db_path, img_dir, image_names=[names[i] for i in kfs], camera_mode=pycolmap.CameraMode.SINGLE,
                              reader_options=ro, extraction_options=eo, device=pycolmap.Device.auto)
    C = {i: Tv[i][:3, 3] for i in kfs}
    Zax = {i: Tv[i][:3, 2] for i in kfs}
    pairs = set()
    for k, i in enumerate(kfs):
        for j in kfs[k + 1:k + 1 + a.seq]:
            pairs.add((i, j))
        cands = []
        for j in kfs:
            if abs(j - i) <= a.seq * a.kf_step:
                continue
            ang = np.degrees(np.arccos(np.clip(Zax[i] @ Zax[j], -1, 1)))
            d = np.linalg.norm(C[i] - C[j])
            if ang < a.cov_angle and d < 0.6:
                cands.append((ang / a.cov_angle + d / 0.6, j))
        for _, j in sorted(cands)[:a.cov_k]:
            pairs.add((min(i, j), max(i, j)))
    pl = out / "pairs.txt"
    pl.write_text("\n".join(f"{names[x]} {names[y]}" for x, y in sorted(pairs)) + "\n")
    mo = pycolmap.FeatureMatchingOptions()
    po = pycolmap.ImportedPairingOptions(); po.match_list_path = str(pl)
    pycolmap.match_image_pairs(str(db_path), matching_options=mo, pairing_options=po, device=pycolmap.Device.auto)
    log(f"[match] {len(pairs)} pairs {time.time() - t0:.0f}s")
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
    for pid, gm in zip(ids, geoms):
        i1, i2 = pid if isinstance(pid, tuple) else pycolmap.pair_id_to_image_pair(pid)
        mm = np.asarray(gm.inlier_matches)
        if len(mm) < 15:
            continue
        k1, k2 = id2kf[i1], id2kf[i2]
        n_inl.append(len(mm))
        for x, y in mm:
            uf.u((k1, int(x)), (k2, int(y)))
    groups = {}
    for node in list(uf.p):
        groups.setdefault(uf.f(node), []).append(node)
    kfi = {i: n for n, i in enumerate(kfs)}
    tracks = []
    for nodes in groups.values():
        if len(nodes) < 2:
            continue
        cams_ = [n[0] for n in nodes]
        if len(set(cams_)) != len(cams_):          # two keypoints of one image in one track: drop the track
            continue
        tracks.append(nodes)
    log(f"[tracks] verified pairs {len(n_inl)} (inliers median {np.median(n_inl) if n_inl else 0:.0f}), tracks {len(tracks)}")
    # ---- triangulate with VIO poses (cam_from_world)
    Rcw = np.array([Tv[i][:3, :3].T for i in kfs]); tcw = np.array([-Tv[i][:3, :3].T @ Tv[i][:3, 3] for i in kfs])
    Pm = np.array([Kr @ np.hstack([Rcw[k], tcw[k][:, None]]) for k in range(len(kfs))])
    Xs, cam_l, pt_l, uv_l = [], [], [], []
    for nodes in tracks:
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
    obs_per_kf = np.bincount(cam, minlength=len(kfs))
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
    R1, t1, X1, st = ba_lm(Rcw, tcw, X0, cam, pt, uv, Kr, fixed=fixed, iters=a.iters, log=log, rs=rs)
    # ---- Sim(3) BA centres -> VIO centres (free keyframes only)
    Cb = np.array([-R1[k].T @ t1[k] for k in range(len(kfs))])
    Cv = np.array([C[i] for i in kfs])
    fr = np.array([k not in set(fixed) or k == fixed[0] for k in range(len(kfs))])
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
                runtime_s=round(time.time() - t0, 1), **{k: v for k, v in st.items() if k != "hist"}, hist=st["hist"])
    (out / "ba.json").write_text(json.dumps(info, indent=1))
    log(json.dumps({k: v for k, v in info.items() if k != "hist"}))


if __name__ == "__main__":
    main()
