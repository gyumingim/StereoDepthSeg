#!/usr/bin/env python3
"""mvs 11-12: shared input for the MVS comparison (COLMAP PatchMatch, MVSFormer++) = the SAME keyframes and BA poses that the
baseline mesh step (scanA/scan_offline.py --vio BA/poses_ba.jsonl.gz --compress --kf-step 1) uses, in the SAME world frame.

CPU only (SIFT + matching on the CPU), no GPU.
  * poses: copied logic of scan_offline.py (VIO/BA poses -> rigid SE(3) align to the --gt rows, compress to rows with a pose)
  * images: undistorted pinhole (fx = fy = K[0,0], same as motion_stereo / kf_ba Kr, R = I) at 960x540 (INTER_AREA from 1920, as
    scan_offline) and 1920x1080, JPEG q95
  * sparse model for MVS view selection / depth ranges: CPU SIFT on the 960 images, matches on the BA pair list (BA/pairs.txt),
    pycolmap.triangulate_points with the poses FIXED (no BA, no pose change) -> COLMAP text + bin model
  * cams.npz: names, frames, T_world_cam (4x4, OpenCV camera), K960, K1920
Usage: venv_ffs/bin/python scan3d/mvs/mvs_prep.py REC_KF BA_DIR OUT [--gt lensonly]
"""
import argparse
import gzip
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "scanA"))


def load_poses(rec, vio, gt):
    """= scan_offline.py lines 'g = np.load(...)' .. compress (no free scale, no noise, keep_gt_not_ok False)."""
    g = np.load(rec / gt / "gt.npz")
    ok = g["ok"]
    n = len(ok)
    T = np.zeros((n, 4, 4)); T[:, :3, :3] = g["R_bc"]; T[:, :3, 3] = g["p_bc"]; T[:, 3, 3] = 1
    t_ns = g["t_ns"]
    vt, vT = [], []
    for line in gzip.open(vio, "rt"):
        d = json.loads(line)
        if d.get("type") == "pose" and d.get("T_world_cam0") and d.get("status") == "tracking":
            vt.append(d["t_ns"]); vT.append(np.array(d["T_world_cam0"], float).reshape(4, 4))
    vt = np.array(vt); vT = np.array(vT)
    idx = np.searchsorted(t_ns, vt).clip(1, len(t_ns) - 1)
    idx = np.where(np.abs(t_ns[idx - 1] - vt) < np.abs(t_ns[idx] - vt), idx - 1, idx)
    good = np.abs(t_ns[idx] - vt) < 8e6
    idx, vT = idx[good], vT[good]
    both = ok[idx]
    A = vT[both][:, :3, 3]; Bp = T[idx[both]][:, :3, 3]
    ma, mb = A.mean(0), Bp.mean(0)
    U, S_, Vt = np.linalg.svd((Bp - mb).T @ (A - ma))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    Ra = U @ D @ Vt
    ta = mb - Ra @ ma
    Tal = np.eye(4); Tal[:3, :3] = Ra; Tal[:3, 3] = ta
    newT = np.zeros_like(T); okv = np.zeros(n, bool)
    for k, Tv in zip(idx, vT):
        newT[k] = Tal @ Tv; okv[k] = True
    ok2 = okv & ok
    sel = np.flatnonzero(ok2)
    return sel, newT[sel], t_ns[sel], g["K"].astype(np.float64), g["dist"].astype(np.float64), Tal


def qvec_of(R):
    from scipy.spatial.transform import Rotation
    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return [w, x, y, z]


def write_text_model(d, cam_line, names, ids, T):
    d.mkdir(parents=True, exist_ok=True)
    (d / "cameras.txt").write_text(cam_line + "\n")
    L = []
    for nm, iid, Twc in zip(names, ids, T):
        Rcw = Twc[:3, :3].T; tcw = -Rcw @ Twc[:3, 3]
        q = qvec_of(Rcw)
        L.append(f"{iid} {q[0]:.17g} {q[1]:.17g} {q[2]:.17g} {q[3]:.17g} {tcw[0]:.17g} {tcw[1]:.17g} {tcw[2]:.17g} 1 {nm}\n")
    (d / "images.txt").write_text("".join(l + "\n" for l in L))   # empty 2D-point line per image
    (d / "points3D.txt").write_text("")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec"); ap.add_argument("ba"); ap.add_argument("out")
    ap.add_argument("--gt", default="lensonly")
    ap.add_argument("--max-features", type=int, default=6000)
    ap.add_argument("--skip-images", action="store_true")
    a = ap.parse_args()
    import pycolmap
    t0 = time.time()
    rec, ba, out = Path(a.rec), Path(a.ba), Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    sel, T, t_ns, K, dist, Tal = load_poses(rec, ba / "poses_ba.jsonl.gz", a.gt)
    names = [f"{int(f):05d}.jpg" for f in sel]
    print(f"[poses] {len(sel)} keyframes, frames {sel[0]}..{sel[-1]}", flush=True)
    info = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), rec=str(rec), ba=str(ba), gt=a.gt, n=len(sel), K_full=K.tolist(),
                dist=dist.tolist(), T_align=Tal.tolist())
    Ks = {}
    for W0, H0 in ((960, 540), (1920, 1080)):
        s = W0 / 1920.0
        Kd = K.copy(); Kd[:2] *= s
        Kr = np.array([[Kd[0, 0], 0, Kd[0, 2]], [0, Kd[0, 0], Kd[1, 2]], [0, 0, 1.0]])
        Ks[W0] = (Kd, Kr)
    if not a.skip_images:
        maps = {W0: cv2.initUndistortRectifyMap(Ks[W0][0], dist, np.eye(3), Ks[W0][1], (W0, W0 * 9 // 16), cv2.CV_32FC1)
                for W0 in Ks}
        for W0 in Ks:
            (out / f"img{W0}").mkdir(exist_ok=True)
        import kf_frames
        nm_of = {int(f): nm for f, nm in zip(sel, names)}
        k = 0
        for fi, f in kf_frames.iter_frames(rec, sel):
            if fi not in nm_of:
                continue
            for W0 in Ks:
                H0 = W0 * 9 // 16
                src = f if (W0, H0) == (f.shape[1], f.shape[0]) else cv2.resize(f, (W0, H0), interpolation=cv2.INTER_AREA)
                cv2.imwrite(str(out / f"img{W0}" / nm_of[fi]), cv2.remap(src, *maps[W0], cv2.INTER_LINEAR),
                            [cv2.IMWRITE_JPEG_QUALITY, 95])
            k += 1
        print(f"[img] {k} images x 2 sizes {time.time() - t0:.0f}s", flush=True)
        assert k == len(sel), (k, len(sel))
    info["images_s"] = round(time.time() - t0, 1)
    np.savez(out / "cams.npz", names=np.array(names), frames=sel, T=T, t_ns=t_ns, K960=Ks[960][1], K1920=Ks[1920][1])

    # ---- sparse model with fixed poses (view selection + depth range only)
    t1 = time.time()
    sp = out / "sparse960"; sp.mkdir(exist_ok=True)
    db = sp / "db.db"
    if db.exists():
        db.unlink()
    Kr = Ks[960][1]
    eo = pycolmap.FeatureExtractionOptions(); eo.sift.max_num_features = a.max_features; eo.max_image_size = 960; eo.use_gpu = False
    ro = pycolmap.ImageReaderOptions(); ro.camera_model = "PINHOLE"
    ro.camera_params = f"{Kr[0, 0]:.10g},{Kr[1, 1]:.10g},{Kr[0, 2] + 0.5:.10g},{Kr[1, 2] + 0.5:.10g}"   # COLMAP pixel centre +0.5
    pycolmap.extract_features(db, out / "img960", image_names=names, camera_mode=pycolmap.CameraMode.SINGLE,
                              reader_options=ro, extraction_options=eo, device=pycolmap.Device.cpu)
    info["extract_s"] = round(time.time() - t1, 1)
    t2 = time.time()
    pl = sp / "pairs.txt"
    have = set(names)
    pairs = [l for l in (ba / "pairs.txt").read_text().splitlines() if l.strip() and all(x in have for x in l.split())]
    pl.write_text("\n".join(pairs) + "\n")
    mo = pycolmap.FeatureMatchingOptions(); mo.use_gpu = False
    po = pycolmap.ImportedPairingOptions(); po.match_list_path = str(pl)
    vopt = pycolmap.TwoViewGeometryOptions(); vopt.ransac.random_seed = 0
    pycolmap.match_image_pairs(str(db), matching_options=mo, pairing_options=po, verification_options=vopt, device=pycolmap.Device.cpu)
    info["match_s"] = round(time.time() - t2, 1); info["pairs"] = len(pairs)
    d_ = pycolmap.Database.open(str(db))
    ids = {im.name: im.image_id for im in d_.read_all_images()}
    cams = d_.read_all_cameras()
    d_.close()
    assert len(cams) == 1, len(cams)
    cam_id = cams[0].camera_id
    cam_line = f"{cam_id} PINHOLE 960 540 {Kr[0, 0]:.17g} {Kr[1, 1]:.17g} {Kr[0, 2] + 0.5:.17g} {Kr[1, 2] + 0.5:.17g}"
    txt = sp / "posed_txt"
    write_text_model(txt, cam_line, names, [ids[n] for n in names], T)
    # image rows must also point to the db camera id
    lines = (txt / "images.txt").read_text().splitlines()
    (txt / "images.txt").write_text("\n".join(" ".join(l.split()[:8] + [str(cam_id)] + l.split()[9:]) if l.strip() else l
                                              for l in lines) + "\n")
    t3 = time.time()
    rec0 = pycolmap.Reconstruction(str(txt))
    tri = sp / "tri"; tri.mkdir(exist_ok=True)
    r = pycolmap.triangulate_points(rec0, str(db), str(out / "img960"), str(tri), clear_points=True, refine_intrinsics=False)
    r.write_text(str(tri))
    info["tri_s"] = round(time.time() - t3, 1)
    info["sparse"] = dict(points=r.num_points3D(), mean_track=r.compute_mean_track_length(),
                          reproj_px=r.compute_mean_reprojection_error(), images=r.num_reg_images())
    # check: poses unchanged
    dmax = 0.0
    for iid, im in r.images.items():
        Tcw = im.cam_from_world() if callable(im.cam_from_world) else im.cam_from_world
        c = -Tcw.rotation.matrix().T @ Tcw.translation
        dmax = max(dmax, float(np.linalg.norm(c - T[names.index(im.name)][:3, 3])))
    info["sparse"]["pose_centre_diff_max_m"] = dmax
    info["total_s"] = round(time.time() - t0, 1)
    (out / "prep.json").write_text(json.dumps(info, indent=1))
    print(json.dumps({k: v for k, v in info.items() if k not in ("K_full", "T_align")}, indent=1))


if __name__ == "__main__":
    main()
