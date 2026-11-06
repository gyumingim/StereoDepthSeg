#!/usr/bin/env python3
"""mvggt: dump the BA keyframes as undistorted 960x540 PNGs, exactly like scan_offline (INTER_AREA resize, initUndistortRectifyMap
with the GT lens K*0.5 and dist, new K = fx for both axes, same cx cy). Also writes the model-input subset (every k-th keyframe so
that <= --cap frames), the BA / GT poses of those frames and the K.
usage: mv_dump.py REC --gt imu_gt_spl --ba poses_ba.jsonl.gz --out DIR [--cap 400]
"""
import argparse
import gzip
import json
import time
from pathlib import Path

import cv2
import numpy as np


def load_poses(p):
    t, T = [], []
    for line in gzip.open(p, "rt"):
        d = json.loads(line)
        if d.get("type") == "pose" and d.get("T_world_cam0") and d.get("status") == "tracking":
            t.append(int(d["t_ns"])); T.append(np.array(d["T_world_cam0"], float).reshape(4, 4))
    o = np.argsort(t)
    return np.array(t, np.int64)[o], np.array(T)[o]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("--gt", required=True)
    ap.add_argument("--ba", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--cap", type=int, default=400)
    ap.add_argument("--scale", type=float, default=0.5)
    a = ap.parse_args()
    rec, out = Path(a.rec), Path(a.out)
    (out / "img").mkdir(parents=True, exist_ok=True)
    g = np.load(rec / a.gt / "gt.npz")
    s = a.scale
    K = g["K"].copy(); K[:2] *= s
    dist = g["dist"].astype(np.float64)
    W0, H0 = int(round(1920 * s)), int(round(1080 * s))
    Kr = np.array([[K[0, 0], 0, K[0, 2]], [0, K[0, 0], K[1, 2]], [0, 0, 1.0]])
    und = cv2.initUndistortRectifyMap(K, dist, np.eye(3), Kr, (W0, H0), cv2.CV_32FC1)
    tb, Tb = load_poses(a.ba)
    gt_t = g["t_ns"].astype(np.int64)
    j = np.searchsorted(gt_t, tb).clip(1, len(gt_t) - 1)
    j = np.where(np.abs(gt_t[j - 1] - tb) < np.abs(gt_t[j] - tb), j - 1, j)
    good = np.abs(gt_t[j] - tb) < 8e6      # same 8 ms nearest-row rule as scan_offline (VIO t_ns is ~4.7 ms off t_mid)
    assert good.all(), f"{(~good).sum()} BA poses without a recording row within 8 ms"
    assert len(np.unique(j)) == len(j)
    k = int(np.ceil(len(tb) / a.cap))
    sub = np.arange(0, len(tb), k)
    want = {int(j[q]): q for q in sub}
    cap = cv2.VideoCapture(str(rec / "cam5.mp4"))
    fi, nw, last = 0, 0, max(want)
    t0 = time.time()
    while fi <= last:
        r, f = cap.read()
        if not r:
            break
        if fi in want:
            q = want[fi]
            if (f.shape[1], f.shape[0]) != (W0, H0):
                f = cv2.resize(f, (W0, H0), interpolation=cv2.INTER_AREA)
            cv2.imwrite(str(out / "img" / f"{tb[q]}.png"), cv2.remap(f, *und, cv2.INTER_LINEAR))
            nw += 1
        fi += 1
    cap.release()
    assert nw == len(sub), (nw, len(sub))
    np.savetxt(out / "kf_subset.txt", gt_t[j[sub]], fmt="%d")   # recording-row t_ns (mv_offline matches within 1 ms)
    np.savez(out / "frames.npz", t_ns=tb[sub], t_row=gt_t[j[sub]], row=j[sub], T_ba=Tb[sub], T_gt=np.stack(
        [np.block([[g["R_bc"][r], g["p_bc"][r][:, None]], [np.zeros((1, 3)), np.ones((1, 1))]]) for r in j[sub]]),
        gt_ok=g["ok"][j[sub]], K=Kr, size=np.array([W0, H0]))
    meta = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), rec=str(rec), gt=a.gt, ba=a.ba, n_ba_kf=int(len(tb)), every_k=k,
                n_subset=int(len(sub)), dropped=int(len(tb) - len(sub)), size=[W0, H0], K=Kr.tolist(),
                K_lens=K.tolist(), dist=dist.tolist(), runtime_s=round(time.time() - t0, 1))
    (out / "meta.json").write_text(json.dumps(meta, indent=1))
    print(json.dumps(meta))


if __name__ == "__main__":
    main()
