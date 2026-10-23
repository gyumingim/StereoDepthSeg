#!/usr/bin/env python3
"""Check imu_gt.board_poses(chunks=N) == serial board_poses (2026-10-23). Uses the first --frames frames of a recording
(board_poses max_frames), RS derotation + keep_corners like the GT runs.
  ../venv_ffs/bin/python imu_learn/imu_gt_chunk_check.py recordings/hs_... --frames 400 --chunks 4 --board B --lens L
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import imu_gt  # noqa: E402
import lens_calib  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("--frames", type=int, default=400)
    ap.add_argument("--chunks", type=int, default=4)
    ap.add_argument("--board", required=True)
    ap.add_argument("--lens", required=True)
    ap.add_argument("--out", default="runs/lens_switch/chunk_check.json")
    a = ap.parse_args()
    rec = Path(a.rec)
    vid = rec / "cam5.mp4"
    imu_gt.BOARD_JSON = Path(a.board)
    K, dist, _ = lens_calib.load(a.lens)
    w_cam = imu_gt.gyro_cam(rec, json.load(open(rec / "imu_gt" / "align.json")))
    t0 = time.time()
    S = imu_gt.board_poses(vid, rec / "frames_cam5.csv", K, dist, w_cam=w_cam, keep_corners=True, max_frames=a.frames)
    ts = time.time() - t0
    t0 = time.time()
    C = imu_gt.board_poses(vid, rec / "frames_cam5.csv", K, dist, w_cam=w_cam, keep_corners=True, chunks=a.chunks,
                           max_frames=a.frames)
    tc = time.time() - t0
    res = dict(made=time.strftime("%F %T"), rec=rec.name, frames=[len(S["t_ns"]), len(C["t_ns"])], chunks=a.chunks,
               serial_s=round(ts, 1), chunked_s=round(tc, 1), keys_equal=sorted(S) == sorted(C))
    diff = {}
    for k in ("t_ns", "n_corners", "ok"):
        diff[k] = bool(np.array_equal(S[k], C[k]))
    for k in ("R_bc", "p_bc", "rms_px"):
        x, y = S[k].astype(float), C[k].astype(float)
        diff[k] = dict(nan_pattern_equal=bool(np.array_equal(np.isnan(x), np.isnan(y))),
                       max_abs_diff=float(np.nanmax(np.abs(x - y))) if x.size else 0.0)
    diff["corners"] = dict(ids_equal=all(np.array_equal(p, q) for p, q in zip(S["corner_ids"], C["corner_ids"])),
                           max_abs_diff_px=max((float(np.abs(p - q).max()) for p, q in zip(S["corners"], C["corners"])
                                                if p.size), default=0.0))
    res["diff"] = diff
    res["n_ok"] = int(S["ok"].sum())
    json.dump(res, open(a.out, "w"), indent=1)
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
