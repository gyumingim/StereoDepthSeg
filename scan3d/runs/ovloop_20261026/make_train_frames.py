#!/usr/bin/env python3
"""ovloop 2026-10-26: decode training frames for the loop-closure vocabulary (read-only on recordings/).

Writes /tmp/ovloop/train/<rec>/frames_960x540_train/c_0.u8 (raw uint8 (n, 540, 960), the bench's gray + undistort to
960x540 = what ov_bridge sees) with every --step-th video frame. Recordings used for the loop-closure EVALUATION are
refused (EVAL below) so the vocabulary never saw them.
  ~/scanner/StereoDepthSeg/venv_ffs/bin/python make_train_frames.py --step 15 REC...
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

MAIN = Path("/home/system/scanner/StereoDepthSeg/scan3d")
sys.path.insert(0, str(MAIN / "imu_learn"))
EVAL = {"142148", "142610", "151646", "165506", "173432",                       # std60 loops + std60 GT
        "133820", "134550", "134204", "214550", "213341", "213955", "215348",   # HS 42-cell GT set (+ 213955)
        "214138", "221052", "221925", "222747", "223815", "224323", "131326", "154320"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recs", nargs="+")
    ap.add_argument("--step", type=int, default=15)
    ap.add_argument("--out", default="/tmp/ovloop/train")
    o = ap.parse_args()
    import os
    os.chdir(MAIN)
    import vio_bench_hs as vb
    for r in o.recs:
        rec = Path("recordings") / r
        if r.split("_")[2] in EVAL:
            sys.exit(f"{r} is an evaluation recording - refused")
        meta = json.load(open(rec / "meta.json"))
        m1, m2, _, (W, H) = vb.undistort_maps(rec, meta, "960x540")
        od = Path(o.out) / r / f"frames_{W}x{H}_train"
        od.mkdir(parents=True, exist_ok=True)
        cap = cv2.VideoCapture(str(rec / "cam5.mp4"))
        n = 0
        fi = 0
        with open(od / "c_0.u8", "wb") as f:
            while cap.grab():
                if fi % o.step == 0:
                    _, fr = cap.retrieve()
                    g = cv2.remap(cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY), m1, m2, cv2.INTER_LINEAR)
                    f.write(np.ascontiguousarray(g).tobytes())
                    n += 1
                fi += 1
        json.dump(dict(rec=r, step=o.step, frames=n, video_frames=fi, W=W, H=H), open(od / "info.json", "w"))
        print(r, n, "frames of", fi, flush=True)


if __name__ == "__main__":
    main()
