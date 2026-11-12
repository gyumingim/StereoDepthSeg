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
Writes MERGED_DIR = a reduced keyframe folder that run_scanner.sh takes as REC (frames_kf/ hard links, phone_live_poses.jsonl.gz,
frames_cam5.csv, manifest.json, meta.json), plus extra_pairs.txt (verified cross pairs of the consensus keyframes, for
kf_ba --extra-pairs) and merge.json (made, inputs, per-part transform and checks).
"""
import argparse, csv, gzip, json, os, shutil, sys, time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))


def load_part(d, gt):
    d = Path(d)
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

    def fail(msg, info):
        log("MERGE FAILED: " + msg)
        info.update(status="failed: " + msg, made=time.strftime("%F %T"))
        (work / "merge.json").write_text(json.dumps(info, indent=1, default=str))
        print(f"!!!! MERGE FAILED: {msg} (details {work / 'merge.json'})", file=sys.stderr, flush=True)
        sys.exit(6)

    P = [load_part(d, a.gt) for d in a.parts]
    info = dict(inputs=[str(Path(d).resolve()) for d in a.parts], args=vars(a), parts=[])
    for k, p in enumerate(P):
        log(f"[part {k}] {p['dir']}: {len(p['kfs'])} keyframes with a VIO pose, {len(p['rows'])} frames")
        if not np.allclose(p["K"], P[0]["K"]) or not np.allclose(p["dist"], P[0]["dist"]):
            fail(f"part {k} lens K/dist differs from part 0 (one lens per merged scan)", info)
        if len(p["kfs"]) < a.min_kf:
            fail(f"part {k} has only {len(p['kfs'])} keyframes", info)
    s = a.scale
    K = P[0]["K"].astype(np.float64).copy(); K[:2] *= s
    W0, H0 = int(round(1920 * s)), int(round(1080 * s))
    Kr = np.array([[K[0, 0], 0, K[0, 2]], [0, K[0, 0], K[1, 2]], [0, 0, 1.0]])
    und = cv2.initUndistortRectifyMap(K, P[0]["dist"].astype(np.float64), np.eye(3), Kr, (W0, H0), cv2.CV_32FC1)
    M = [np.eye(4)]                                 # world_A <- world_k
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
        Tref = {x: M[x[0]] @ P[x[0]]["T"][x[1]] for x in ref}
        Rcw = {x: Tref[x][:3, :3].T for x in ref}
        tcw = {x: -Tref[x][:3, :3].T @ Tref[x][:3, 3] for x in ref}
        Pm = {x: Kr @ np.hstack([Rcw[x], tcw[x][:, None]]) for x in ref}
        obj = {y: [] for y in new}
        n_tri = 0
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
            pnp[y] = dict(T=Tw, n_inl=int(len(inl)), n_obj=int(len(Xo)))
        log(f"[part {k}] triangulated reference points seen by part {k}: {n_tri}; PnP ok on {len(pnp)}/{len(new)} keyframes "
            f"(>= {a.min_pnp} inliers); verified cross pairs (>= 15 inl) {sum(v >= 15 for v in cross_inl.values())}/{n_cross}")
        pk = dict(part=k, dir=str(P[k]["dir"]), n_ref=len(ref), n_new=len(new), n_cross_pairs=n_cross,
                  n_cross_verified=int(sum(v >= 15 for v in cross_inl.values())), n_ref_points=n_tri, n_pnp_ok=len(pnp),
                  pnp_inliers_median=float(np.median([v["n_inl"] for v in pnp.values()])) if pnp else 0.0)
        info["parts"].append(pk)
        if len(pnp) < a.min_kf:
            fail(f"part {k} ({P[k]['dir'].name}): only {len(pnp)} keyframes could be placed in the earlier recordings by PnP "
                 f"(need {a.min_kf}); verified cross pairs {pk['n_cross_verified']}/{n_cross} -- the recordings do not overlap "
                 f"(start the next recording looking at what the end of the previous one saw)", info)
        # ---- 4-DoF consensus
        ys = sorted(pnp)
        Cn = {y: P[k]["T"][y[1]][:3, 3] for y in ys}; Rn = {y: P[k]["T"][y[1]][:3, :3] for y in ys}
        Cr = {y: pnp[y]["T"][:3, 3] for y in ys}; Rr = {y: pnp[y]["T"][:3, :3] for y in ys}
        yaw = {y: yaw_of(Rr[y] @ Rn[y].T) for y in ys}
        best = None
        for y0 in ys:
            R0 = Rz(yaw[y0]); t0_ = Cr[y0] - R0 @ Cn[y0]
            ce = np.array([np.linalg.norm(Cr[y] - (R0 @ Cn[y] + t0_)) for y in ys])
            re = np.array([rot_deg(Rr[y].T @ R0 @ Rn[y]) for y in ys])
            inl = (ce < 0.10) & (re < 3.0)
            key = (int(inl.sum()), -float(np.median(ce[inl])) if inl.any() else 0)
            if best is None or key > best[0]:
                best = (key, inl)
        inl = best[1]
        ci = [y for y, g_ in zip(ys, inl) if g_]
        if len(ci) < a.min_kf:
            fail(f"part {k} ({P[k]['dir'].name}): PnP poses of {len(ys)} keyframes do not agree on one transform "
                 f"(best {len(ci)} within 10 cm / 3 deg, need {a.min_kf}) -- matches are probably wrong (no real overlap)", info)
        ps = float(np.arctan2(np.mean([np.sin(yaw[y]) for y in ci]), np.mean([np.cos(yaw[y]) for y in ci])))
        R4 = Rz(ps); t4 = np.mean([Cr[y] - R4 @ Cn[y] for y in ci], 0)
        Mk = np.eye(4); Mk[:3, :3] = R4; Mk[:3, 3] = t4
        ce = np.array([np.linalg.norm(Cr[y] - (R4 @ Cn[y] + t4)) for y in ci])
        re = np.array([rot_deg(Rr[y].T @ R4 @ Rn[y]) for y in ci])
        tilt = np.array([rot_deg(Rz(yaw[y]).T @ (Rr[y] @ Rn[y].T)) for y in ci])
        A_ = np.array([Cn[y] for y in ci]); B_ = np.array([Cr[y] for y in ci])
        spread = float(np.sqrt(((A_ - A_.mean(0)) ** 2).sum(1).mean()))
        sc = None
        if spread > 0.15:
            ma, mb = A_.mean(0), B_.mean(0)
            U, S_, Vt = np.linalg.svd((B_ - mb).T @ (A_ - ma)); D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
            sc = float(np.trace(np.diag(S_) @ D) / ((A_ - ma) ** 2).sum())
        pk.update(n_consensus=len(ci), consensus_frac=len(ci) / len(ys), yaw_deg=float(np.degrees(ps)), t_m=t4.tolist(), M=Mk.tolist(),
                  centre_err_cm_median=float(np.median(ce) * 100), centre_err_cm_max=float(ce.max() * 100),
                  rot_err_deg_median=float(np.median(re)), tilt_deg_median=float(np.median(tilt)), tilt_deg_max=float(tilt.max()),
                  centre_spread_m=spread, sim3_scale_ratio=sc,
                  consensus_frames=[y[1] for y in ci])
        log(f"[part {k}] 4-DoF: {len(ci)}/{len(ys)} keyframes agree, yaw {np.degrees(ps):.2f} deg, t {np.round(t4, 3).tolist()} m, "
            f"centre err median {np.median(ce) * 100:.1f} cm, rot err median {np.median(re):.2f} deg, tilt median {np.median(tilt):.2f} deg, "
            f"Sim3 scale ratio {sc if sc is None else round(sc, 4)} (centre spread {spread:.2f} m)")
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
                m["T_world_cam0"] = (M[k] @ np.array(m["T_world_cam0"], float).reshape(4, 4)).ravel().tolist()
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
                note="M[k] maps part-k VIO world into part-0 world (4-DoF yaw + translation); poses of part k in the folder = M[k] * VIO")
    (out / "merge.json").write_text(json.dumps(info, indent=1, default=str))
    shutil.copy(out / "merge.json", work / "merge.json")
    log(f"[merge] {len(kf_out)} keyframes ({info['n_keyframes_per_part']}), extra pairs {len(cross_pairs_all)}, -> {out} "
        f"({time.time() - t0:.0f} s)")


if __name__ == "__main__":
    main()
