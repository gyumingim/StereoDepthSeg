#!/usr/bin/env python3
"""Compare the field of view of two arlat recordings of the same STATIC scene (e.g. hs 240 fps vs std 60 fps).

For each recording (folder with cam5.mp4 [+ results_cam5.csv, meta.json]) it takes the per-pixel median of N evenly
spaced frames (removes noise / the odd blurred frame), matches SIFT features between the two medians (Lowe ratio test),
fits a similarity (estimateAffinePartial2D) and a full affine (estimateAffine2D) with RANSAC, mapping A -> B, and reports:
  scale (%)    : 100 * (s - 1); s = how much bigger an object looks in B than in A (pixels)
  fov ratio    : A's horizontal / vertical field of view in units of B's (1.0 = same; < 1 = A sees less, i.e. A is cropped)
  shift (px)   : where A's image centre lands in B minus B's centre (in B pixels)
  rotation     : degrees
  affine sx/sy : per-axis scale from the full affine (unequal -> anisotropic scaling, e.g. different binning/aspect)
plus the SCALER_CROP_REGION values from each results_cam5.csv (column crop_region, arlat >= 0.6.5) and meta.json's
session / stream_config / active_array.

  python scan3d/tools/fov_compare.py <rec_hs240> <rec_std60> [--frames 30] [--out result.json] [--png overlay.png]
A recording may also be given as the mp4 itself.
"""
import argparse
import csv
import json
import os
import sys
import time
from collections import Counter

import cv2
import numpy as np


def rec_paths(p):
    if os.path.isfile(p):
        return p, os.path.dirname(os.path.abspath(p))
    return os.path.join(p, "cam5.mp4"), p


def median_frame(mp4, n):
    cap = cv2.VideoCapture(mp4)
    if not cap.isOpened():
        sys.exit(f"cannot open {mp4}")
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frames = []
    if total > 0:
        # skip the first 10 % (AE / AF settling) and sample evenly
        idx = np.unique(np.linspace(total * 0.1, total - 1, n).astype(int))
        for i in idx:
            cap.set(cv2.CAP_PROP_POS_FRAMES, int(i))
            ok, f = cap.read()
            if ok:
                frames.append(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
    if not frames:  # seeking failed / unknown count: read sequentially
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        allf = []
        while True:
            ok, f = cap.read()
            if not ok:
                break
            allf.append(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
        total = len(allf)
        frames = allf[int(total * 0.1)::max(1, (total - int(total * 0.1)) // n)][:n]
    cap.release()
    if not frames:
        sys.exit(f"no frames decoded from {mp4}")
    return np.median(np.stack(frames), axis=0).astype(np.uint8), total, len(frames)


def crop_info(folder):
    out = {}
    res = os.path.join(folder, "results_cam5.csv")
    if os.path.exists(res):
        with open(res, newline="") as f:
            r = csv.DictReader(f)
            if r.fieldnames and "crop_region" in r.fieldnames:
                c = Counter(row["crop_region"] for row in r)
                out["crop_region_counts"] = dict(c.most_common())
            else:
                out["crop_region_counts"] = "column missing (recorded before arlat 0.6.5)"
    meta = os.path.join(folder, "meta.json")
    if os.path.exists(meta):
        with open(meta) as f:
            m = json.load(f)
        for k in ("session", "app_version", "width", "height", "fps", "stream_config", "active_array",
                  "pixel_array", "pre_correction_active_array", "ae", "exposure_ns_set", "iso_set"):
            if k in m:
                out[k] = m[k]
        cap = m.get("capture", {})
        for k in ("first_crop_region", "first_zoom_ratio"):
            if k in cap:
                out[k] = cap[k]
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("a", help="recording A (e.g. hs 240)")
    ap.add_argument("b", help="recording B (e.g. std 60)")
    ap.add_argument("--frames", type=int, default=30, help="frames per recording for the median (default 30)")
    ap.add_argument("--ratio", type=float, default=0.75, help="Lowe ratio test (default 0.75)")
    ap.add_argument("--ransac", type=float, default=2.0, help="RANSAC reprojection threshold px (default 2)")
    ap.add_argument("--out", help="write the result as JSON here")
    ap.add_argument("--png", help="write A warped onto B (red/cyan overlay) here")
    args = ap.parse_args()

    (mp4a, da), (mp4b, db) = rec_paths(args.a), rec_paths(args.b)
    A, na, ua = median_frame(mp4a, args.frames)
    B, nb, ub = median_frame(mp4b, args.frames)
    ha, wa = A.shape
    hb, wb = B.shape

    sift = cv2.SIFT_create(nfeatures=8000)
    ka, fa = sift.detectAndCompute(A, None)
    kb, fb = sift.detectAndCompute(B, None)
    if fa is None or fb is None:
        sys.exit("no SIFT features (blank / dark frames?)")
    knn = cv2.BFMatcher(cv2.NORM_L2).knnMatch(fa, fb, k=2)
    good = [m for m, n in (p for p in knn if len(p) == 2) if m.distance < args.ratio * n.distance]
    if len(good) < 10:
        sys.exit(f"only {len(good)} matches after the ratio test: not the same scene?")
    pa = np.float32([ka[m.queryIdx].pt for m in good])
    pb = np.float32([kb[m.trainIdx].pt for m in good])

    S, inl = cv2.estimateAffinePartial2D(pa, pb, method=cv2.RANSAC, ransacReprojThreshold=args.ransac, maxIters=20000, confidence=0.999)
    F, inlf = cv2.estimateAffine2D(pa, pb, method=cv2.RANSAC, ransacReprojThreshold=args.ransac, maxIters=20000, confidence=0.999)
    if S is None:
        sys.exit("similarity fit failed")
    inl = inl.ravel().astype(bool)
    s = float(np.hypot(S[0, 0], S[1, 0]))
    rot = float(np.degrees(np.arctan2(S[1, 0], S[0, 0])))
    ca = np.array([(wa - 1) / 2, (ha - 1) / 2, 1.0])
    cb = np.array([(wb - 1) / 2, (hb - 1) / 2])
    shift = S @ ca - cb
    resid = np.linalg.norm((pa[inl] @ S[:, :2].T + S[:, 2]) - pb[inl], axis=1)
    # A's field of view expressed in B's: A's width maps to s*wa B pixels
    fov_h, fov_v = s * wa / wb, s * ha / hb
    aff = None
    if F is not None:
        sx, sy = float(np.hypot(F[0, 0], F[1, 0])), float(np.hypot(F[0, 1], F[1, 1]))
        aff = {"sx": sx, "sy": sy, "sx_minus_sy_pct": 100 * (sx - sy) / ((sx + sy) / 2),
               "fov_ratio_h": sx * wa / wb, "fov_ratio_v": sy * ha / hb,
               "inliers": int(inlf.sum()), "matrix": F.tolist()}

    result = {
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        "inputs": {"a": os.path.abspath(mp4a), "b": os.path.abspath(mp4b), "frames_median": [ua, ub],
                   "frames_total": [na, nb], "size_a": [wa, ha], "size_b": [wb, hb]},
        "matches": len(good), "inliers": int(inl.sum()), "rms_px": float(np.sqrt(np.mean(resid ** 2))),
        "similarity": {"scale": s, "scale_pct": 100 * (s - 1), "rotation_deg": rot,
                       "shift_px": [float(shift[0]), float(shift[1])], "fov_ratio_h": fov_h, "fov_ratio_v": fov_v,
                       "matrix": S.tolist()},
        "affine": aff,
        "a_info": crop_info(da), "b_info": crop_info(db),
    }

    print(f"A {mp4a}  {wa}x{ha}  median of {ua}/{na} frames")
    print(f"B {mp4b}  {wb}x{hb}  median of {ub}/{nb} frames")
    print(f"SIFT matches {len(good)}, inliers {int(inl.sum())}, rms {result['rms_px']:.2f} px")
    print(f"similarity A->B: scale {100 * (s - 1):+.2f} %  rotation {rot:+.3f} deg  "
          f"shift of A centre in B ({shift[0]:+.1f}, {shift[1]:+.1f}) px")
    print(f"field of view A / B: horizontal {fov_h:.4f}  vertical {fov_v:.4f}  (1 = same; < 1 = A sees less)")
    if aff:
        print(f"affine: sx {aff['sx']:.4f} sy {aff['sy']:.4f} (sx-sy {aff['sx_minus_sy_pct']:+.2f} %), inliers {aff['inliers']}")
    for name, info in (("A", result["a_info"]), ("B", result["b_info"])):
        print(f"{name}: session {info.get('session', '?')}  stream {info.get('stream_config', '?')}")
        print(f"   active_array {info.get('active_array', '?')}  first_crop {info.get('first_crop_region', '?')}"
              f"  zoom {info.get('first_zoom_ratio', '?')}")
        print(f"   crop_region per frame: {info.get('crop_region_counts', 'no results_cam5.csv')}")

    if args.out:
        with open(args.out, "w") as f:
            json.dump(result, f, indent=1)
        print("->", args.out)
    if args.png:
        W = cv2.warpAffine(A, S, (wb, hb))
        cv2.imwrite(args.png, cv2.merge([B, B, W]))   # A warped = red channel, B = blue/green: grey where they agree
        print("->", args.png)


if __name__ == "__main__":
    main()
