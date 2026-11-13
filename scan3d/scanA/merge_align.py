#!/usr/bin/env python3
"""merge 2026-11-12: put two (or more) keyframe recordings of one scene into ONE keyframe folder (run_merge.sh step 'align').

  venv_ffs/bin/python scan3d/scanA/merge_align.py RED_A RED_B [RED_C ...] --out MERGED_DIR [--max-sub 80] [--min-kf 5]

RED_* = reduced keyframe folders (run_scanner.sh --stop-after reduce -> OUT/red/<rec>_kf: frames_kf/, phone_live_poses.jsonl.gz,
lensonly/gt.npz or another --gt folder, manifest.json, frames_cam5.csv, meta.json). Part 0 (A) is the reference: its world frame
(VIO, gravity z-up, metric) is the world of the merged result. For part k = 1, 2, ...:
 1. keyframe images undistorted to the kf_ba pinhole (960x540), subsampled: <= --max-sub keyframes of every already aligned part
    (reference) and of part k (uniform in keyframe order)
 2. pycolmap CPU SIFT (6000, same as kf_ba) + matching of every reference x part-k pair, plus reference-reference pairs
    (neighbours 1 and 2 in the subsample of the same part) + two-view verification (RANSAC seed 0)
 3. tracks (union-find of verified inliers, tracks with two keypoints of one image dropped), triangulated with the reference
    poses only (>= 2 reference views, reprojection < 3 px, ray angle >= 2 deg) -> 3D points in the reference world
 4. PnP RANSAC (OpenCV, seed 0, 3 px) for every subsampled part-k keyframe on those points -> its pose in the reference world
 5. 4-DoF fit (yaw + translation; both VIO worlds are gravity aligned and metric): every PnP keyframe proposes yaw (from its
    rotation) + translation, consensus = keyframes within 10 cm / 3 deg, refit on the consensus. Checks written to merge.json:
    tilt (roll/pitch left after the yaw, should be ~0), Sim(3) scale ratio of the consensus centres (should be ~1).
    Fewer than --min-kf consensus keyframes -> "MERGE FAILED" + exit 6 (the recordings do not overlap / do not match).
 6. part k poses (all of them) := M_k * VIO pose; frame numbers shifted by the frames of the earlier parts, times shifted so
    part k starts 100 s after the end of part k-1 (scan_offline FFS partners need dt <= 30 s -> depth pairs stay inside
    one recording, as before).
Gates (u16, 11-13; review code_review2.md 1-2 .. 1-4, 1-7). Every failure writes merge.json status "failed: ..." and exits
with its own code (run_merge.sh passes it on):
   6  no overlap: fewer than --min-kf part-k keyframes placed by PnP
  10  lens / camera metadata differ between the parts, or a part's focus differs from --lens (meta.json of each recording)
  11  a part is unusable (missing files, fewer than --min-kf keyframes with a VIO pose)
  12  no consensus: fewer than --min-kf keyframes agree, or agreeing / PnP keyframes < --min-consensus-frac
  13  consensus too concentrated (moved object / repeated structure guard): PnP inliers of the agreeing keyframes cover
      < --min-cells of the 4x4 image grid (union), or their 3D points span < --min-extent-m, or the agreeing keyframes span
      < --min-span-s of part-k time, or a second, different transform has nearly as many keyframes (--max-second-frac)
  14  scale: the metric scale ratio of the two VIO worlds (same points triangulated in both; fallback Sim(3) of the agreeing
      centres when < --min-scale-pts points, spread >= --scale-min-spread) is off by more than --max-scale-sim3 (10 %,
      --scale-mode sim3 = default: the ratio is applied = 7-DoF yaw + translation + scale) or --max-scale-dev (3 %,
      --scale-mode refuse: 4-DoF as before)
  15  tilt: median roll/pitch left after the yaw > --max-tilt-deg (both VIO worlds should be gravity aligned)
Writes MERGED_DIR = a reduced keyframe folder that run_scanner.sh takes as REC (frames_kf/ hard links, phone_live_poses.jsonl.gz,
frames_cam5.csv, manifest.json, meta.json), plus extra_pairs.txt (verified cross pairs of the consensus keyframes, for
kf_ba --extra-pairs) and merge.json (made, inputs, per-part transform and checks).
"""
import argparse, csv, gzip, json, os, shutil, sys, time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))


REQ = ("frames_kf", "manifest.json", "phone_live_poses.jsonl.gz", "frames_cam5.csv", "meta.json")
LENS_KEYS = ("width", "height", "logical", "physical", "focus_dpt_request", "af_mode_request", "focus_sweep_dpt", "focus_locked_dpt",
             "active_array")


def load_part(d, gt):
    d = Path(d)
    miss = [f for f in REQ + (f"{gt}/gt.npz",) if not (d / f).exists()]
    if miss:
        raise FileNotFoundError(f"{d}: missing {miss}")
    g = np.load(d / gt / "gt.npz")
    man = json.loads((d / "manifest.json").read_text())
    kfs = sorted(int(i) for i in man["keyframes"])
    vt, vT = [], []
    for line in gzip.open(d / "phone_live_poses.jsonl.gz", "rt"):
        m = json.loads(line)
        if m.get("type") == "pose" and m.get("T_world_cam0") and m.get("status") == "tracking":
            vt.append(int(m["t_ns"])); vT.append(np.array(m["T_world_cam0"], float).reshape(4, 4))
    vt = np.array(vt, np.int64); vT = np.array(vT)
    t_ns = g["t_ns"]
    T = {}
    for i in kfs:
        j = int(np.argmin(np.abs(vt - t_ns[i])))
        if abs(int(vt[j]) - int(t_ns[i])) < 1e6:
            T[i] = vT[j]
    kfs = [i for i in kfs if i in T]
    rows = list(csv.DictReader(open(d / "frames_cam5.csv")))
    return dict(dir=d, man=man, kfs=kfs, T=T, t_ns=t_ns, K=g["K"], dist=g["dist"], rows=rows,
                meta=json.loads((d / "meta.json").read_text()))


def yaw_of(R):
    return float(np.arctan2(R[1, 0], R[0, 0]))


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def rot_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def apply_m(Mk, T):
    """Mk = (4x4 rigid [R t], scale s): camera pose T (world_k) -> world_A. Rotation R @ R_T, centre s R c + t (s = 1: plain Mk @ T)."""
    M_, s_ = Mk
    o = np.eye(4); o[:3, :3] = M_[:3, :3] @ T[:3, :3]; o[:3, 3] = s_ * M_[:3, :3] @ T[:3, 3] + M_[:3, 3]
    return o


def best_consensus(ys, yaw, Cr, Cn, Rr, Rn, ce_tol=0.10, re_tol=3.0, scale=1.0):
    best = None
    for y0 in ys:
        R0 = Rz(yaw[y0]); t0_ = Cr[y0] - scale * R0 @ Cn[y0]
        ce = np.array([np.linalg.norm(Cr[y] - (scale * R0 @ Cn[y] + t0_)) for y in ys])
        re = np.array([rot_deg(Rr[y].T @ R0 @ Rn[y]) for y in ys])
        inl = (ce < ce_tol) & (re < re_tol)
        key = (int(inl.sum()), -float(np.median(ce[inl])) if inl.any() else 0)
        if best is None or key > best[0]:
            best = (key, inl, y0, t0_)
    return best


def tri_dlt(q, Pl, Tl, Kr, px=3.0):
    """DLT of one point from >= 2 views (q (n,2), projection matrices Pl, camera-to-world poses Tl), drop views > px, ray angle >= 2 deg.
    Same rule as the reference triangulation above. -> X or None"""
    q = np.asarray(q, float); Pl = list(Pl); Tl = list(Tl)
    for _ in range(2):
        Pa = np.array(Pl)
        A_ = np.concatenate([q[:, :1] * Pa[:, 2] - Pa[:, 0], q[:, 1:2] * Pa[:, 2] - Pa[:, 1]], 0)
        Xh = np.linalg.svd(A_)[2][-1]
        if abs(Xh[3]) < 1e-12:
            return None
        X = Xh[:3] / Xh[3]
        Xc = np.array([T[:3, :3].T @ (X - T[:3, 3]) for T in Tl])
        e = np.linalg.norm(Xc[:, :2] / Xc[:, 2:] * Kr[0, 0] + Kr[:2, 2] - q, axis=1)
        good = (e < px) & (Xc[:, 2] > 0.05) & (Xc[:, 2] < 8.0)
        if good.all():
            break
        if good.sum() < 2:
            return None
        q = q[good]; Pl = [x for x, g_ in zip(Pl, good) if g_]; Tl = [x for x, g_ in zip(Tl, good) if g_]
    else:
        return None
    cen = np.array([T[:3, 3] for T in Tl]); rays = X - cen; rays /= np.linalg.norm(rays, axis=1, keepdims=True)
    if np.degrees(np.arccos(np.clip(np.min(rays @ rays.T), -1, 1))) < 2.0:
        return None
    return X


def pts_scale(XA, XB, n=4000, min_d=0.3, seed=0):
    """scale world_A / world_k from the same 3D points triangulated in both worlds: median of distance ratios of random point pairs
    (|XB_i - XB_j| >= min_d); bootstrap sd over points. Independent of PnP and of the lens focal error (both sides use the same K)."""
    m = len(XA)
    if m < 20:
        return None, None, 0
    rng = np.random.default_rng(seed)

    def est(idx):
        i, j = rng.choice(idx, n), rng.choice(idx, n)
        dB = np.linalg.norm(XB[i] - XB[j], axis=1); ok = dB >= min_d
        return float(np.median(np.linalg.norm(XA[i] - XA[j], axis=1)[ok] / dB[ok])) if ok.sum() >= 50 else None
    r = est(np.arange(m))
    bs = [est(rng.integers(0, m, m)) for _ in range(100)]
    bs = [b for b in bs if b is not None]
    return r, (float(np.std(bs)) if bs else None), m


def sub(lst, n):
    if len(lst) <= n:
        return list(lst)
    return [lst[int(round(k))] for k in np.linspace(0, len(lst) - 1, n)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("parts", nargs="+")
    ap.add_argument("--out", required=True)
    ap.add_argument("--gt", default="lensonly")
    ap.add_argument("--max-sub", type=int, default=80, help="keyframes per part used for the cross matching")
    ap.add_argument("--min-kf", type=int, default=5, help="fewest part-k keyframes that agree on one transform")
    ap.add_argument("--min-pnp", type=int, default=30, help="fewest PnP inliers of a part-k keyframe")
    ap.add_argument("--min-cross-inl", type=int, default=30, help="verified cross pair -> extra_pairs.txt at >= this many inliers")
    ap.add_argument("--gap-s", type=float, default=100.0, help="time gap put between the parts (> FFS partner dt cap 30 s)")
    ap.add_argument("--max-features", type=int, default=6000)
    ap.add_argument("--scale", type=float, default=0.5)
    ap.add_argument("--lens", default="", help="lens json used for every part (run_merge --lens): its focus_dpt must equal every part's "
                    "meta.json focus_dpt_request")
    ap.add_argument("--min-consensus-frac", type=float, default=0.5, help="agreeing / PnP-placed part-k keyframes (u10 0.86, u11 0.77)")
    ap.add_argument("--min-cells", type=int, default=8, help="4x4 image grid cells (union over agreeing keyframes) holding >= 5 PnP inliers")
    ap.add_argument("--min-extent-m", type=float, default=1.0, help="3D span (p5-p95 diagonal) of the agreeing keyframes' PnP inlier points")
    ap.add_argument("--min-span-s", type=float, default=2.0, help="time span of the agreeing part-k keyframes")
    ap.add_argument("--max-second-frac", type=float, default=0.5, help="a second transform (>= 25 cm or 10 deg away) agreed by >= this "
                    "fraction of the best one's keyframes (and >= --min-kf) -> ambiguous")
    ap.add_argument("--scale-mode", default="sim3", choices=["refuse", "sim3"], help="sim3 (u16 default): apply the points scale "
                    "ratio (7-DoF), refuse beyond --max-scale-sim3; refuse: 4-DoF, refuse beyond --max-scale-dev")
    ap.add_argument("--min-scale-pts", type=int, default=100, help="fewest points for the points scale ratio (else centres, not applied)")
    ap.add_argument("--max-scale-dev", type=float, default=0.03)
    ap.add_argument("--max-scale-sim3", type=float, default=0.10)
    ap.add_argument("--scale-min-spread", type=float, default=0.5, help="m; below it the ratio is not measurable (only recorded)")
    ap.add_argument("--max-tilt-deg", type=float, default=1.5)
    ap.add_argument("--test-offset-m", type=float, default=0.0, help="TEST ONLY (u16 link-gate test): add this x offset (m) to every "
                    "part-k transform after the gates, i.e. a deliberately wrong alignment; recorded in merge.json")
    a = ap.parse_args()
    import pycolmap
    import gpu_guard
    t0 = time.time()
    out = Path(a.out)
    if out.exists():
        shutil.rmtree(out)
    work = out.parent / (out.name + "_align")
    if work.exists():
        shutil.rmtree(work)
    (work / "img").mkdir(parents=True)
    logf = open(work / "align.log", "w")

    def log(s):
        print(s, flush=True); logf.write(s + "\n"); logf.flush()

    def fail(msg, info, code=6):
        log(f"MERGE FAILED (exit {code}): " + msg)
        info.update(status="failed: " + msg, exit_code=code, made=time.strftime("%F %T"))
        (work / "merge.json").write_text(json.dumps(info, indent=1, default=str))
        print(f"!!!! MERGE FAILED (exit {code}): {msg} (details {work / 'merge.json'})", file=sys.stderr, flush=True)
        sys.exit(code)

    info = dict(inputs=[str(Path(d).resolve()) for d in a.parts], args=vars(a), parts=[])
    P = []
    for d in a.parts:
        try:
            P.append(load_part(d, a.gt))
        except (FileNotFoundError, KeyError, ValueError, json.JSONDecodeError) as e:
            fail(f"part {len(P)} unusable: {e}", info, 11)
    lf = None
    if a.lens:
        lf = json.loads(Path(a.lens).read_text()).get("focus_dpt")
    info["lens_meta"] = [{k_: p["meta"].get(k_) for k_ in LENS_KEYS} for p in P]
    info["lens_focus_dpt"] = lf
    for k, p in enumerate(P):
        log(f"[part {k}] {p['dir']}: {len(p['kfs'])} keyframes with a VIO pose, {len(p['rows'])} frames, "
            f"focus {p['meta'].get('focus_dpt_request')} dpt, camera {p['meta'].get('physical')}")
        diff = [k_ for k_ in LENS_KEYS if p["meta"].get(k_) != P[0]["meta"].get(k_)]
        if diff:
            fail(f"part {k} camera/lens metadata differ from part 0 ({ {k_: (P[0]['meta'].get(k_), p['meta'].get(k_)) for k_ in diff} }); "
                 f"one lens per merged scan", info, 10)
        fd = p["meta"].get("focus_dpt_request")
        if lf is not None and (fd is None or abs(float(fd) - float(lf)) > 0.01):
            fail(f"part {k} was recorded at focus {fd} dpt but the lens {a.lens} is for {lf} dpt", info, 10)
        if not np.allclose(p["K"], P[0]["K"]) or not np.allclose(p["dist"], P[0]["dist"]):
            fail(f"part {k} lens K/dist differs from part 0 (one lens per merged scan)", info, 10)
        if len(p["kfs"]) < a.min_kf:
            fail(f"part {k} has only {len(p['kfs'])} keyframes with a VIO pose", info, 11)
    s = a.scale
    K = P[0]["K"].astype(np.float64).copy(); K[:2] *= s
    W0, H0 = int(round(1920 * s)), int(round(1080 * s))
    Kr = np.array([[K[0, 0], 0, K[0, 2]], [0, K[0, 0], K[1, 2]], [0, 0, 1.0]])
    und = cv2.initUndistortRectifyMap(K, P[0]["dist"].astype(np.float64), np.eye(3), Kr, (W0, H0), cv2.CV_32FC1)
    M = [(np.eye(4), 1.0)]                          # world_A <- world_k: (rigid 4x4, scale)
    cross_pairs_all = []
    for k in range(1, len(P)):
        tk = time.time()
        ref = [(r, i) for r in range(k) for i in sub(P[r]["kfs"], a.max_sub)]
        new = [(k, i) for i in sub(P[k]["kfs"], a.max_sub)]
        nm = lambda ri: f"p{ri[0]}_{ri[1]:05d}.jpg"
        for r, i in ref + new:
            f = cv2.imread(str(P[r]["dir"] / "frames_kf" / f"{i:05d}.jpg"), cv2.IMREAD_COLOR)
            im = cv2.remap(cv2.resize(f, (W0, H0), interpolation=cv2.INTER_AREA), *und, cv2.INTER_LINEAR)
            cv2.imwrite(str(work / "img" / nm((r, i))), im, [cv2.IMWRITE_JPEG_QUALITY, 95])
        pairs = [(x, y) for x in ref for y in new]
        n_cross = len(pairs)
        for r in range(k):
            rr = [x for x in ref if x[0] == r]
            pairs += [(rr[j], rr[j + d]) for d in (1, 2) for j in range(len(rr) - d)]
        db_path = work / f"db_{k}.db"
        pl = work / f"pairs_{k}.txt"
        pl.write_text("".join(f"{nm(x)} {nm(y)}\n" for x, y in pairs))
        with gpu_guard.job(f"merge_align {out}", out_dir=str(work), torch_cap=False):
            eo = pycolmap.FeatureExtractionOptions(); eo.sift.max_num_features = a.max_features; eo.max_image_size = max(W0, H0)
            eo.use_gpu = False
            ro = pycolmap.ImageReaderOptions(); ro.camera_model = "PINHOLE"
            ro.camera_params = f"{Kr[0, 0]:.10g},{Kr[1, 1]:.10g},{Kr[0, 2] + 0.5:.10g},{Kr[1, 2] + 0.5:.10g}"
            pycolmap.extract_features(db_path, work / "img", image_names=[nm(x) for x in ref + new], camera_mode=pycolmap.CameraMode.SINGLE,
                                      reader_options=ro, extraction_options=eo, device=pycolmap.Device.cpu)
            vopt = pycolmap.TwoViewGeometryOptions(); vopt.ransac.random_seed = 0
            mo = pycolmap.FeatureMatchingOptions()
            po = pycolmap.ImportedPairingOptions(); po.match_list_path = str(pl)
            pycolmap.match_image_pairs(str(db_path), matching_options=mo, pairing_options=po, verification_options=vopt,
                                       device=pycolmap.Device.auto)
        log(f"[part {k}] {len(ref)} reference + {len(new)} new keyframes, {n_cross} cross + {len(pairs) - n_cross} reference pairs, "
            f"features + matching {time.time() - tk:.0f} s")
        db = pycolmap.Database.open(str(db_path))
        id2 = {im.image_id: im.name for im in db.read_all_images()}
        name2 = {nm(x): x for x in ref + new}
        kps = {}
        for iid, n_ in id2.items():
            kps[name2[n_]] = np.asarray(db.read_keypoints(iid))[:, :2] - 0.5
        ids, geoms = db.read_two_view_geometries()
        db.close()
        # ---- tracks
        parent = {}

        def f_(x):
            parent.setdefault(x, x)
            while parent[x] != x:
                parent[x] = parent[parent[x]]; x = parent[x]
            return x
        cross_inl = {}
        for pid, gm in zip(ids, geoms):
            i1, i2 = pid if isinstance(pid, tuple) else pycolmap.pair_id_to_image_pair(pid)
            x, y = name2[id2[i1]], name2[id2[i2]]
            mm = np.asarray(gm.inlier_matches)
            if (x[0] == k) != (y[0] == k):
                cross_inl[(x, y) if y[0] == k else (y, x)] = len(mm)
            if len(mm) < 15:
                continue
            for u, v in mm:
                ra, rb = f_((x, int(u))), f_((y, int(v)))
                if ra != rb:
                    parent[max(ra, rb)] = min(ra, rb)
        groups = {}
        for node in list(parent):
            groups.setdefault(f_(node), []).append(node)
        Tref = {x: apply_m(M[x[0]], P[x[0]]["T"][x[1]]) for x in ref}
        Rcw = {x: Tref[x][:3, :3].T for x in ref}
        tcw = {x: -Tref[x][:3, :3].T @ Tref[x][:3, 3] for x in ref}
        Pm = {x: Kr @ np.hstack([Rcw[x], tcw[x][:, None]]) for x in ref}
        obj = {y: [] for y in new}
        n_tri = 0
        Tnew = {y: P[k]["T"][y[1]] for y in new}                                  # part-k VIO world (before the transform)
        Pnew = {y: Kr @ np.hstack([Tnew[y][:3, :3].T, (-Tnew[y][:3, :3].T @ Tnew[y][:3, 3])[:, None]]) for y in new}
        XA_l, XB_l = [], []
        for nodes in groups.values():
            imgs = [n_[0] for n_ in nodes]
            if len(set(imgs)) != len(imgs):
                continue
            ro_ = [n_ for n_ in nodes if n_[0][0] != k]
            no_ = [n_ for n_ in nodes if n_[0][0] == k]
            if len(ro_) < 2 or not no_:
                continue
            q = np.array([kps[n_[0]][n_[1]] for n_ in ro_]); cams = [n_[0] for n_ in ro_]
            for _ in range(2):
                A_ = np.concatenate([q[:, :1] * np.array([Pm[c][2] for c in cams]) - np.array([Pm[c][0] for c in cams]),
                                     q[:, 1:2] * np.array([Pm[c][2] for c in cams]) - np.array([Pm[c][1] for c in cams])], 0)
                Xh = np.linalg.svd(A_)[2][-1]
                if abs(Xh[3]) < 1e-12:
                    break
                X = Xh[:3] / Xh[3]
                Xc = np.array([Rcw[c] @ X + tcw[c] for c in cams])
                e = np.linalg.norm(Xc[:, :2] / Xc[:, 2:] * Kr[0, 0] + Kr[:2, 2] - q, axis=1)
                good = (e < 3.0) & (Xc[:, 2] > 0.05) & (Xc[:, 2] < 8.0)
                if good.all() or good.sum() < 2:
                    break
                q = q[good]; cams = [c for c, g_ in zip(cams, good) if g_]
            if abs(Xh[3]) < 1e-12 or len(cams) < 2 or not good.all():
                continue
            cen = np.array([Tref[c][:3, 3] for c in cams]); rays = X - cen; rays /= np.linalg.norm(rays, axis=1, keepdims=True)
            if np.degrees(np.arccos(np.clip(np.min(rays @ rays.T), -1, 1))) < 2.0:
                continue
            n_tri += 1
            for n_ in no_:
                obj[n_[0]].append((X, kps[n_[0]][n_[1]]))
            if len(no_) >= 2:                   # u16: the same point in part k's own VIO world -> scale check from points
                XB = tri_dlt([kps[n_[0]][n_[1]] for n_ in no_], [Pnew[n_[0]] for n_ in no_], [Tnew[n_[0]] for n_ in no_], Kr)
                if XB is not None:
                    XA_l.append(X); XB_l.append(XB)
        # ---- PnP per new keyframe
        cv2.setRNGSeed(0)
        pnp = {}
        for y in new:
            if len(obj[y]) < a.min_pnp:
                continue
            Xo = np.array([o[0] for o in obj[y]]); uo = np.array([o[1] for o in obj[y]])
            okp, rv, tv, inl = cv2.solvePnPRansac(Xo, uo, Kr, None, iterationsCount=2000, reprojectionError=3.0, confidence=0.999)
            if not okp or inl is None or len(inl) < a.min_pnp:
                continue
            inl = inl.ravel()
            rv, tv = cv2.solvePnPRefineLM(Xo[inl], uo[inl], Kr, None, rv, tv)
            Rc = cv2.Rodrigues(rv)[0]
            Tw = np.eye(4); Tw[:3, :3] = Rc.T; Tw[:3, 3] = (-Rc.T @ tv).ravel()
            pnp[y] = dict(T=Tw, n_inl=int(len(inl)), n_obj=int(len(Xo)), uv=uo[inl], X=Xo[inl])
        log(f"[part {k}] triangulated reference points seen by part {k}: {n_tri}; PnP ok on {len(pnp)}/{len(new)} keyframes "
            f"(>= {a.min_pnp} inliers); verified cross pairs (>= 15 inl) {sum(v >= 15 for v in cross_inl.values())}/{n_cross}")
        pk = dict(part=k, dir=str(P[k]["dir"]), n_ref=len(ref), n_new=len(new), n_cross_pairs=n_cross,
                  n_cross_verified=int(sum(v >= 15 for v in cross_inl.values())), n_ref_points=n_tri, n_pnp_ok=len(pnp),
                  pnp_inliers_median=float(np.median([v["n_inl"] for v in pnp.values()])) if pnp else 0.0)
        info["parts"].append(pk)
        if len(pnp) < a.min_kf:
            fail(f"part {k} ({P[k]['dir'].name}): only {len(pnp)} keyframes could be placed in the earlier recordings by PnP "
                 f"(need {a.min_kf}); verified cross pairs {pk['n_cross_verified']}/{n_cross} -- the recordings do not overlap "
                 f"(start the next recording looking at what the end of the previous one saw)", info, 6)
        # ---- 4-DoF consensus
        ys = sorted(pnp)
        Cn = {y: P[k]["T"][y[1]][:3, 3] for y in ys}; Rn = {y: P[k]["T"][y[1]][:3, :3] for y in ys}
        Cr = {y: pnp[y]["T"][:3, 3] for y in ys}; Rr = {y: pnp[y]["T"][:3, :3] for y in ys}
        yaw = {y: yaw_of(Rr[y] @ Rn[y].T) for y in ys}
        # u16: metric scale ratio world_A / world_k from the same points triangulated in both VIO worlds (u11 1.025, u10 0.992,
        # u11 with B x 1.05 0.976 = 1.025 / 1.05). --scale-mode sim3 (default): the consensus and the transform use it (7-DoF)
        sc_p, sc_p_sd, n_sp = pts_scale(np.array(XA_l), np.array(XB_l)) if XA_l else (None, None, 0)
        if sc_p is not None and n_sp < a.min_scale_pts:
            sc_p = None
        s_k = sc_p if (a.scale_mode == "sim3" and sc_p is not None) else 1.0
        best = best_consensus(ys, yaw, Cr, Cn, Rr, Rn, scale=s_k)
        inl = best[1]
        ci = [y for y, g_ in zip(ys, inl) if g_]
        frac = len(ci) / len(ys)
        pk.update(n_consensus=len(ci), consensus_frac=frac)
        if len(ci) < a.min_kf or frac < a.min_consensus_frac:
            fail(f"part {k} ({P[k]['dir'].name}): PnP poses of {len(ys)} keyframes do not agree on one transform "
                 f"(best {len(ci)} = {frac:.2f} within 10 cm / 3 deg, need {a.min_kf} and {a.min_consensus_frac:.2f}) -- matches are "
                 f"probably wrong (no real overlap)", info, 12)
        ps = float(np.arctan2(np.mean([np.sin(yaw[y]) for y in ci]), np.mean([np.cos(yaw[y]) for y in ci])))
        R4 = Rz(ps); t4 = np.mean([Cr[y] - s_k * R4 @ Cn[y] for y in ci], 0)
        tilt = np.array([rot_deg(Rz(yaw[y]).T @ (Rr[y] @ Rn[y].T)) for y in ci])
        A_ = np.array([Cn[y] for y in ci]); B_ = np.array([Cr[y] for y in ci])
        spread = float(np.sqrt(((A_ - A_.mean(0)) ** 2).sum(1).mean()))

        def sim3_ratio(idx):
            a_, b_ = A_[idx], B_[idx]
            ma, mb = a_.mean(0), b_.mean(0)
            U, S_, Vt = np.linalg.svd((b_ - mb).T @ (a_ - ma)); D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
            return float(np.trace(np.diag(S_) @ D) / max(((a_ - ma) ** 2).sum(), 1e-12))
        sc_c = sim3_ratio(np.arange(len(ci))) if spread > 0.15 else None   # centre ratio of the raw VIO centres
        sc = sc_p if sc_p is not None else sc_c         # u16: points ratio preferred (centre ratio: u10 1.024 vs points 0.992)
        sc_sd = None
        if sc_c is not None:
            rng = np.random.default_rng(0)
            bs = [sim3_ratio(rng.integers(0, len(ci), len(ci))) for _ in range(200)]
            sc_sd = float(np.std(bs))
        # moved-object / repeated-structure guard: spread of the agreeing keyframes' PnP inliers
        cells = np.zeros(16, int)
        for y in ci:
            uv_ = pnp[y]["uv"]
            cx = np.clip((uv_[:, 0] / W0 * 4).astype(int), 0, 3); cy = np.clip((uv_[:, 1] / H0 * 4).astype(int), 0, 3)
            cells += np.bincount(cy * 4 + cx, minlength=16)
        n_cells = int((cells >= 5).sum())
        cells_kf = [int((np.bincount(np.clip((pnp[y]["uv"][:, 1] / H0 * 4).astype(int), 0, 3) * 4
                                     + np.clip((pnp[y]["uv"][:, 0] / W0 * 4).astype(int), 0, 3), minlength=16) >= 3).sum()) for y in ci]
        Xall = np.concatenate([pnp[y]["X"] for y in ci])
        extent = float(np.linalg.norm(np.percentile(Xall, 95, 0) - np.percentile(Xall, 5, 0)))
        tk_ = np.array([int(P[k]["t_ns"][y[1]]) for y in ci]); span_s = float((tk_.max() - tk_.min()) * 1e-9)
        rest = [y for y in ys if y not in set(ci)]
        n2, d2t, d2y = 0, None, None
        if len(rest) >= 2:
            b2 = best_consensus(rest, yaw, Cr, Cn, Rr, Rn, scale=s_k)
            c2 = [y for y, g_ in zip(rest, b2[1]) if g_]
            if c2:
                ps2 = float(np.arctan2(np.mean([np.sin(yaw[y]) for y in c2]), np.mean([np.cos(yaw[y]) for y in c2])))
                t2 = np.mean([Cr[y] - s_k * Rz(ps2) @ Cn[y] for y in c2], 0)
                n2, d2t = len(c2), float(np.linalg.norm(t2 - t4)); d2y = float(abs(np.degrees(np.angle(np.exp(1j * (ps2 - ps))))))
        ambiguous = n2 >= max(a.min_kf, a.max_second_frac * len(ci)) and (d2t > 0.25 or d2y > 10)
        scale_checked = (sc_p is not None) or (sc_c is not None and spread >= a.scale_min_spread)
        Mk = np.eye(4); Mk[:3, :3] = R4; Mk[:3, 3] = t4
        ce = np.array([np.linalg.norm(Cr[y] - (s_k * R4 @ Cn[y] + t4)) for y in ci])
        re = np.array([rot_deg(Rr[y].T @ R4 @ Rn[y]) for y in ci])
        pk.update(yaw_deg=float(np.degrees(ps)), t_m=t4.tolist(), M=Mk.tolist(), scale_applied=s_k,
                  centre_err_cm_median=float(np.median(ce) * 100), centre_err_cm_max=float(ce.max() * 100),
                  rot_err_deg_median=float(np.median(re)), tilt_deg_median=float(np.median(tilt)), tilt_deg_max=float(tilt.max()),
                  centre_spread_m=spread, sim3_scale_ratio=sc_c, sim3_scale_boot_sd=sc_sd, scale_checked=scale_checked,
                  pts_scale_ratio=sc_p, pts_scale_boot_sd=sc_p_sd, pts_scale_n=n_sp, scale_used=sc,
                  scale_source="points" if sc_p is not None else ("centres" if sc_c is not None else None),
                  grid_cells_union=n_cells, grid_cells_kf_median=float(np.median(cells_kf)), inlier_extent_m=extent,
                  consensus_span_s=span_s, second_n=n2, second_dt_m=d2t, second_dyaw_deg=d2y, ambiguous=bool(ambiguous),
                  consensus_frames=[y[1] for y in ci])
        log(f"[part {k}] 4-DoF: {len(ci)}/{len(ys)} keyframes agree ({frac:.2f}), yaw {np.degrees(ps):.2f} deg, t {np.round(t4, 3).tolist()} m, "
            f"centre err median {np.median(ce) * 100:.1f} cm, rot err median {np.median(re):.2f} deg, tilt median {np.median(tilt):.2f} deg, "
            f"Sim3 scale ratio {sc_c if sc_c is None else round(sc_c, 4)} (boot sd {sc_sd if sc_sd is None else round(sc_sd, 4)}, centre spread {spread:.2f} m), "
            f"points scale ratio {sc_p if sc_p is None else round(sc_p, 4)} (boot sd {sc_p_sd if sc_p_sd is None else round(sc_p_sd, 4)}, {n_sp} points), "
            f"grid cells {n_cells}/16 (per kf median {np.median(cells_kf):.0f}), inlier extent {extent:.2f} m, span {span_s:.1f} s, "
            f"second transform {n2} kf (dt {d2t}, dyaw {d2y})")
        if n_cells < a.min_cells or extent < a.min_extent_m or span_s < a.min_span_s or ambiguous:
            fail(f"part {k} ({P[k]['dir'].name}): the agreeing keyframes see too little of the scene or another transform fits nearly as well "
                 f"(grid cells {n_cells}/16 need {a.min_cells}, 3D extent {extent:.2f} m need {a.min_extent_m}, time span {span_s:.1f} s need "
                 f"{a.min_span_s}, second transform {n2} kf vs {len(ci)}) -- a moved object or repeated structure? Film more of the "
                 f"unchanged room at the start of the next recording", info, 13)
        if float(np.median(tilt)) > a.max_tilt_deg:
            fail(f"part {k} ({P[k]['dir'].name}): residual tilt {np.median(tilt):.2f} deg > {a.max_tilt_deg} (gravity of the two VIO worlds "
                 f"disagrees: bad VIO init?)", info, 15)
        if scale_checked and abs(sc - 1) > (a.max_scale_sim3 if a.scale_mode == "sim3" else a.max_scale_dev):
            fail(f"part {k} ({P[k]['dir'].name}): metric scale of the two VIO runs differs by {(sc - 1) * 100:+.1f} % (from "
                 f"{'points' if sc_p is not None else 'centres'}, limit {(a.max_scale_sim3 if a.scale_mode == 'sim3' else a.max_scale_dev) * 100:.0f} %, "
                 f"--scale-mode {a.scale_mode}); B would drift {abs(sc - 1) * 100 * 5:.0f} cm at 5 m. Re-record with a still VIO start, "
                 f"or pass --align-args '--scale-mode sim3' to apply the ratio", info, 14)
        if a.test_offset_m:
            Mk[0, 3] += a.test_offset_m; pk["test_offset_m"] = a.test_offset_m
            log(f"[part {k}] TEST: transform shifted by {a.test_offset_m} m in x (--test-offset-m)")
        Mk = (Mk, s_k)
        M.append(Mk)
        cset = set(ci)
        cross_pairs_all += [(x, y, v) for (x, y), v in cross_inl.items() if y in cset and v >= a.min_cross_inl]
    # ---- merged keyframe folder
    out.mkdir(parents=True)
    (out / "frames_kf").mkdir()
    off_f, shift_t, rows_out, kf_out, poses_out = [], [], [], [], []
    fr0, t_end = 0, None
    jpeg_bytes = 0
    for k, p in enumerate(P):
        tf = np.array([int(r["t_mid_ns"]) for r in p["rows"]], np.int64)
        sh = 0 if k == 0 else int(t_end + a.gap_s * 1e9 - tf[0])
        off_f.append(fr0); shift_t.append(sh)
        for r in p["rows"]:
            r2 = dict(r); r2["frame_idx"] = str(int(r["frame_idx"]) + fr0)
            for c in ("t_mid_ns", "physical_ts_ns", "logical_ts_ns"):
                if c in r2 and r2[c] not in ("", None):
                    r2[c] = str(int(r2[c]) + sh)
            rows_out.append(r2)
        for i in p["kfs"]:
            src = p["dir"] / "frames_kf" / f"{i:05d}.jpg"; dst = out / "frames_kf" / f"{i + fr0:05d}.jpg"
            try:
                os.link(src, dst)
            except OSError:
                shutil.copy2(src, dst)
            jpeg_bytes += src.stat().st_size
            kf_out.append(i + fr0)
        for line in gzip.open(p["dir"] / "phone_live_poses.jsonl.gz", "rt"):
            m = json.loads(line)
            if "t_ns" in m:
                m["t_ns"] = int(m["t_ns"]) + sh
            if m.get("T_world_cam0"):
                m["T_world_cam0"] = apply_m(M[k], np.array(m["T_world_cam0"], float).reshape(4, 4)).ravel().tolist()
            m["merge_part"] = k
            poses_out.append(m)
        t_end = int(tf[-1]) + sh
        fr0 += len(p["rows"])
    poses_out.sort(key=lambda m: m.get("t_ns", 0))
    with open(out / "frames_cam5.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(P[0]["rows"][0].keys())); w.writeheader(); w.writerows(rows_out)
    with gzip.open(out / "phone_live_poses.jsonl.gz", "wt") as f:
        for m in poses_out:
            f.write(json.dumps(m) + "\n")
    man = dict(P[0]["man"])
    man.update(made=time.strftime("%F %T"), keyframes=sorted(kf_out), n_keyframes=len(kf_out), jpeg_bytes=jpeg_bytes,
               duration_s=float(sum(p["man"].get("duration_s", 0) for p in P)), merged_from=[str(p["dir"]) for p in P])
    (out / "manifest.json").write_text(json.dumps(man, indent=1))
    meta = dict(P[0]["meta"]); meta.update(folder=out.name, frames=len(rows_out), merged_from=[str(p["dir"]) for p in P])
    (out / "meta.json").write_text(json.dumps(meta, indent=1))
    (out / "extra_pairs.txt").write_text("".join(f"{x[1] + off_f[x[0]]:05d}.jpg {y[1] + off_f[y[0]]:05d}.jpg\n"
                                                 for x, y, _ in sorted(cross_pairs_all)))
    info.update(status="ok", made=time.strftime("%F %T"), out=str(out), frame_offset=off_f, time_shift_ns=shift_t,
                n_keyframes=len(kf_out), n_keyframes_per_part=[len(p["kfs"]) for p in P], n_extra_pairs=len(cross_pairs_all),
                extra_pairs_same_source_frame=int(sum(1 for x, y, _ in cross_pairs_all
                                                       if P[x[0]]["meta"].get("folder") == P[y[0]]["meta"].get("folder") and x[1] == y[1])),
                runtime_s=round(time.time() - t0, 1),
                scale_per_part=[float(m_[1]) for m_ in M], exit_code=0,
                note="M[k] maps part-k VIO world into part-0 world (4-DoF yaw + translation, + scale with --scale-mode sim3); "
                     "poses of part k in the folder: R = R_M R_vio, c = s R_M c_vio + t_M")
    (out / "merge.json").write_text(json.dumps(info, indent=1, default=str))
    shutil.copy(out / "merge.json", work / "merge.json")
    log(f"[merge] {len(kf_out)} keyframes ({info['n_keyframes_per_part']}), extra pairs {len(cross_pairs_all)}, -> {out} "
        f"({time.time() - t0:.0f} s)")


if __name__ == "__main__":
    main()
