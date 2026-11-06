#!/usr/bin/env python3
"""boardscale 2026-11-06: metric scale of BA keyframe poses from a known-size ChArUco board in the scene (no GT poses).

  venv_ffs/bin/python scan3d/scanA/board_scale.py REC --poses BA/poses_ba.jsonl.gz --board BOARD.json --lens GTDIR/gt.npz --out DIR

1. detect the ChArUco inner corners in the raw 1920x1080 frame of every BA keyframe (aruco_scale.Board.detect = the same
   detector the board GT uses); keep keyframes with >= --min-corners corners
2. per corner id: DLT triangulation over all keyframes that see it, with the BA poses (undistorted normalised coords);
   drop observations with reprojection > max(3 px, median + 4 MAD), repeat (<= 3x); need >= 3 views and >= 2 deg ray spread
3. similarity fit board model -> points (Umeyama), drop corners with residual > max(2 mm * s, 3 * median), refit once;
   correction c = 1 / s (multiply the pose translations)
4. block bootstrap over keyframes (blocks of 10 consecutive keyframes, 200 draws, seed 0) -> sd and 2.5/97.5 % of c
5. writes DIR/poses_board.jsonl.gz (BA poses, translation x c), DIR/board_scale.json, DIR/detections.npz
--lens: only the arrays 'K' and 'dist' are read (lens calibration; the same values the BA used). No GT pose is read.
--check-gt-corners GTRS/gt.npz (optional, check only): pixel difference of my detections vs the stored GT detections.
"""
import argparse, gzip, json, sys, time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import aruco_scale  # noqa: E402


def load_poses(p):
    rows = []
    for line in gzip.open(p, "rt"):
        d = json.loads(line)
        if d.get("type") == "pose" and d.get("T_world_cam0") and d.get("status") == "tracking":
            rows.append(d)
    return rows


def umeyama(A, B):
    """B ~= s R A + t"""
    ma, mb = A.mean(0), B.mean(0)
    U, S, Vt = np.linalg.svd((B - mb).T @ (A - ma))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    s = float(np.trace(np.diag(S) @ D) / ((A - ma) ** 2).sum())
    return s, R, mb - s * R @ ma


def dlt(P, n):
    """P (m,3,4) cam_from_world, n (m,2) normalised coords -> X (3,)"""
    A = np.concatenate([n[:, :1] * P[:, 2] - P[:, 0], n[:, 1:] * P[:, 2] - P[:, 1]], 0)
    Xh = np.linalg.svd(A)[2][-1]
    return Xh[:3] / Xh[3]


def detect_keyframes(video, frames, board, log=print):
    want = set(frames)
    last = max(frames)
    cap = cv2.VideoCapture(str(video))
    det = {}
    fi = 0
    t0 = time.time()
    while fi <= last:
        if fi in want:
            r, f = cap.read()
            if not r:
                break
            c, ids, _ = board.detect(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
            det[fi] = (c, ids)
        else:
            if not cap.grab():
                break
        fi += 1
        if fi % 5000 == 0:
            log(f"  decode {fi}/{last} ({time.time() - t0:.0f} s)")
    cap.release()
    return det


def estimate(kfs, obs, Pcw, model, K, dist, args, want_detail=False):
    """kfs: list of keyframe keys (may repeat, bootstrap); obs[k] = {id: (norm(2), px(2))}; Pcw[k] = 3x4.
    returns dict(c, s, n_corners, ...) or None"""
    views = {}
    for k in kfs:
        for cid, (nrm, px) in obs[k].items():
            views.setdefault(cid, []).append(k)
    pts, rep_all, n_obs = {}, [], 0
    for cid, ks in views.items():
        ks = list(ks)
        for _ in range(4):
            if len(set(ks)) < 3:
                ks = []
                break
            P = np.array([Pcw[k] for k in ks]); n = np.array([obs[k][cid][0] for k in ks])
            X = dlt(P, n)
            Xc = np.einsum("nij,j->ni", P[:, :, :3], X) + P[:, :, 3]
            if np.any(Xc[:, 2] <= 0):
                good = Xc[:, 2] > 0
            else:
                good = np.ones(len(ks), bool)
            q = cv2.projectPoints(Xc.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, dist)[0].reshape(-1, 2)
            e = np.linalg.norm(q - np.array([obs[k][cid][1] for k in ks]), axis=1)
            med = np.median(e); mad = np.median(np.abs(e - med)) * 1.4826
            good &= e <= max(args.max_rep_px, med + 4 * mad)
            if good.all():
                break
            ks = [k for k, g in zip(ks, good) if g]
        if len(set(ks)) < 3:
            continue
        P = np.array([Pcw[k] for k in ks]); n = np.array([obs[k][cid][0] for k in ks])   # final set
        X = dlt(P, n)
        Xc = np.einsum("nij,j->ni", P[:, :, :3], X) + P[:, :, 3]
        q = cv2.projectPoints(Xc.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, dist)[0].reshape(-1, 2)
        e = np.linalg.norm(q - np.array([obs[k][cid][1] for k in ks]), axis=1); good = np.ones(len(ks), bool)
        # ray spread: largest angle between camera centres as seen from X
        C = np.array([-Pcw[k][:, :3].T @ Pcw[k][:, 3] for k in set(ks)])
        r = C - X; r /= np.linalg.norm(r, axis=1, keepdims=True)
        spread = np.degrees(np.arccos(np.clip((r @ r.T).min(), -1, 1)))
        if spread < args.min_spread_deg:
            continue
        pts[cid] = X; rep_all += list(e[good]); n_obs += int(good.sum())
    ids = sorted(i for i in pts if i < len(model))
    if len(ids) < 4:
        return None
    M = model[ids]; X = np.array([pts[i] for i in ids])
    s, R, t = umeyama(M, X)
    res = np.linalg.norm(X - (s * (R @ M.T).T + t), axis=1)
    keep = res <= max(args.max_sim_mm / 1000 * s, 3 * np.median(res))
    if keep.sum() >= 4 and not keep.all():
        s, R, t = umeyama(M[keep], X[keep])
        res = np.linalg.norm(X - (s * (R @ M.T).T + t), axis=1)
    out = dict(c=1.0 / s, s=s, n_corners=int(keep.sum()), n_corners_tri=len(ids), n_obs=n_obs,
               sim_rms_mm=float(np.sqrt((res[keep] ** 2).mean()) * 1000 / s),
               rep_px_median=float(np.median(rep_all)) if rep_all else None,
               rep_px_p90=float(np.percentile(rep_all, 90)) if rep_all else None)
    if want_detail:
        out["dropped_corner_ids"] = [int(i) for i, k in zip(ids, keep) if not k]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec"); ap.add_argument("--poses", required=True); ap.add_argument("--board", required=True)
    ap.add_argument("--lens", required=True, help="npz with K, dist (1920x1080 px; only these two arrays are read)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--min-corners", type=int, default=6)
    ap.add_argument("--max-rep-px", type=float, default=3.0)
    ap.add_argument("--min-spread-deg", type=float, default=2.0)
    ap.add_argument("--max-sim-mm", type=float, default=2.0)
    ap.add_argument("--boot", type=int, default=200); ap.add_argument("--block", type=int, default=10)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--check-gt-corners", default="", help="(check only) GT detection npz with corners/corner_ids per frame")
    a = ap.parse_args()
    t_start = time.time()
    rec, out = Path(a.rec), Path(a.out); out.mkdir(parents=True, exist_ok=True)
    rows = load_poses(a.poses)
    frames = [int(r["frame"]) for r in rows]
    lz = np.load(a.lens, allow_pickle=False)
    K, dist = np.array(lz["K"], float), np.array(lz["dist"], float).ravel()
    spec = aruco_scale.load_board_spec(a.board)
    board = aruco_scale.Board(spec)
    model = board.obj.copy()
    print(f"board {Path(a.board).name}: {spec['squares_x']}x{spec['squares_y']} square {spec['square_m'] * 1000:.3f} mm "
          f"({spec['square_src']}), {board.n_corners} inner corners; keyframes {len(rows)}", flush=True)
    det_cache = out / "detections.npz"
    if det_cache.exists():
        z = np.load(det_cache, allow_pickle=True)
        det = {int(f): (c, i) for f, c, i in zip(z["frame"], z["corners"], z["ids"])}
        assert set(det) >= set(frames), "detection cache does not cover the poses"
        print("detections from cache", flush=True)
    else:
        det = detect_keyframes(rec / "cam5.mp4", frames, board, log=lambda s: print(s, flush=True))
        fr = sorted(det)
        np.savez_compressed(det_cache, frame=np.array(fr), corners=np.array([det[f][0] for f in fr], dtype=object),
                            ids=np.array([det[f][1] for f in fr], dtype=object), made=time.strftime("%F %T"),
                            video=str(rec / "cam5.mp4"), board=a.board)
    Pcw, obs, kf = {}, {}, []
    for r in rows:
        f = int(r["frame"])
        if f not in det:
            continue
        c, ids = det[f]
        if len(ids) < a.min_corners:
            continue
        T = np.array(r["T_world_cam0"], float).reshape(4, 4)
        Pcw[f] = np.linalg.inv(T)[:3]
        n = cv2.undistortPoints(np.asarray(c, np.float64).reshape(-1, 1, 2), K, dist).reshape(-1, 2)
        obs[f] = {int(k): (n[x], np.asarray(c[x], float)) for x, k in enumerate(np.asarray(ids).ravel())}
        kf.append(f)
    print(f"keyframes with >= {a.min_corners} corners: {len(kf)} / {len(rows)}", flush=True)
    if len(kf) < 3:
        sys.exit("too few board keyframes")
    best = estimate(kf, obs, Pcw, model, K, dist, a, want_detail=True)
    if best is None:
        sys.exit("board scale failed (too few corners triangulated)")
    rng = np.random.default_rng(a.seed)
    blocks = [kf[i:i + a.block] for i in range(0, len(kf), a.block)]
    bs = []
    for _ in range(a.boot):
        pick = rng.integers(0, len(blocks), len(blocks))
        r_ = estimate([k for b in pick for k in blocks[b]], obs, Pcw, model, K, dist, a)
        if r_ is not None:
            bs.append(r_["c"])
    bs = np.array(bs)
    # auxiliary (not used): two-view scale over BA baseline 4-30 cm pairs
    C = {f: -Pcw[f][:, :3].T @ Pcw[f][:, 3] for f in kf}
    s2 = []
    for x, i in enumerate(kf):
        for j in kf[x + 1:x + 60]:
            B = np.linalg.norm(C[i] - C[j]); com = sorted(set(obs[i]) & set(obs[j]))
            if 0.04 <= B <= 0.30 and len(com) >= 6:
                X = np.array([dlt(np.array([Pcw[i], Pcw[j]]), np.array([obs[i][k][0], obs[j][k][0]])) for k in com])
                s2.append(1.0 / umeyama(model[com], X)[0])
    res = dict(made=time.strftime("%F %T"), rec=str(rec.resolve()), poses=str(Path(a.poses).resolve()), board=str(Path(a.board).resolve()),
               square_mm=spec["square_m"] * 1000, square_src=spec["square_src"], square_estimated=bool(spec.get("square_estimated", False)),
               lens=str(Path(a.lens).resolve()), lens_fx=float(K[0, 0]), args=vars(a), n_poses=len(rows), n_kf_board=len(kf),
               c=best["c"], best=best, boot=dict(n=int(len(bs)), sd=float(bs.std(ddof=1)), p2_5=float(np.percentile(bs, 2.5)),
                                                 p97_5=float(np.percentile(bs, 97.5)), median=float(np.median(bs))),
               pair2v=dict(n=len(s2), median=float(np.median(s2)) if s2 else None,
                           p10=float(np.percentile(s2, 10)) if s2 else None, p90=float(np.percentile(s2, 90)) if s2 else None),
               uses_gt_poses=False)
    if a.check_gt_corners:
        g = np.load(a.check_gt_corners, allow_pickle=True)
        d = []
        nfr = 0
        for f in kf:
            if f >= len(g["corners"]) or g["corners"][f] is None:
                continue
            gc, gi = np.asarray(g["corners"][f], float).reshape(-1, 2), np.asarray(g["corner_ids"][f]).ravel()
            mine = {int(k): v[1] for k, v in obs[f].items()}
            nfr += 1
            d += [np.linalg.norm(mine[int(k)] - gc[x]) for x, k in enumerate(gi) if int(k) in mine]
        res["check_vs_gt_detections"] = dict(file=a.check_gt_corners, frames=nfr, n=len(d),
                                             median_px=float(np.median(d)) if d else None, p99_px=float(np.percentile(d, 99)) if d else None)
    c = best["c"]
    with gzip.open(out / "poses_board.jsonl.gz", "wt") as fo:
        for r in rows:
            T = np.array(r["T_world_cam0"], float).reshape(4, 4); T[:3, 3] *= c
            fo.write(json.dumps(dict(r, T_world_cam0=T.ravel().tolist(), board_scale=c)) + "\n")
    res["runtime_s"] = round(time.time() - t_start, 1)
    (out / "board_scale.json").write_text(json.dumps(res, indent=1))
    print(f"c = {c:.5f} (boot sd {res['boot']['sd']:.5f}, 95% {res['boot']['p2_5']:.5f}-{res['boot']['p97_5']:.5f}); "
          f"corners {best['n_corners']}/{best['n_corners_tri']}, obs {best['n_obs']}, rep med {best['rep_px_median']:.2f} px, "
          f"sim rms {best['sim_rms_mm']:.2f} mm; pair2v median {res['pair2v']['median']} (n {len(s2)})", flush=True)
    if "check_vs_gt_detections" in res:
        print("check vs GT detections:", res["check_vs_gt_detections"], flush=True)


if __name__ == "__main__":
    main()
