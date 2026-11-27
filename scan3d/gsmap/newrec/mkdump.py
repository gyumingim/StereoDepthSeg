#!/usr/bin/env python3
"""2DGS input for a new recording (night 11-27): RUN/dump_ba/img/KF.png (undistorted 960x540 keyframes, the gsplat dump convention:
frame resized to 960x540 INTER_AREA, cv2.remap with cams K (960 scale), dist -> Kr) and FUSED/KF.npz (Z, F = 1 FFS, S) from the
gsm_ffs_pairs2 fuse output s12 (Z, V, S). CPU.

  python mkdump.py RUN REC S12_DIR FUSED_OUT
"""
import json, sys, time
from pathlib import Path

import cv2
import numpy as np


def main():
    run, rec, s12, fo = (Path(x) for x in sys.argv[1:5])
    c = np.load(run / "dump_ba/cams.npz"); kf = [int(x) for x in c["kf"]]; W, H = [int(v) for v in c["size"]]
    mx, my = cv2.initUndistortRectifyMap(c["K"], c["dist"], np.eye(3), c["Kr"], (W, H), cv2.CV_32FC1)
    (run / "dump_ba/img").mkdir(parents=True, exist_ok=True); fo.mkdir(parents=True, exist_ok=True)
    want = set(kf); cap = cv2.VideoCapture(str(rec / "cam5.mp4")); i = 0; n = 0
    while n < len(want):
        r, f = cap.read()
        if not r:
            break
        if i in want:
            if f.shape[1] != W:
                f = cv2.resize(f, (W, H), interpolation=cv2.INTER_AREA)
            cv2.imwrite(str(run / "dump_ba/img" / f"{i:05d}.png"), cv2.remap(f, mx, my, cv2.INTER_LINEAR)); n += 1
        i += 1
    cap.release()
    nz = 0
    for f in kf:
        q = s12 / f"{f:05d}.npz"
        if q.exists():
            d = np.load(q); V = d["V"] & np.isfinite(d["Z"]); nz += 1
            np.savez_compressed(fo / f"{f:05d}.npz", Z=np.where(V, d["Z"], 0).astype(np.float32), F=V.astype(np.uint8), S=np.where(V, d["S"], np.inf).astype(np.float32))
        else:
            np.savez_compressed(fo / f"{f:05d}.npz", Z=np.zeros((H, W), np.float32), F=np.zeros((H, W), np.uint8), S=np.full((H, W), np.inf, np.float32))
    info = dict(made=time.strftime("%F %T"), run=str(run), rec=str(rec), s12=str(s12), n_kf=len(kf), images=n, kf_with_ffs=nz, note="F = 1 FFS only (no fill)")
    (fo / "info.json").write_text(json.dumps(info, indent=1)); print(json.dumps(info))


if __name__ == "__main__":
    main()
