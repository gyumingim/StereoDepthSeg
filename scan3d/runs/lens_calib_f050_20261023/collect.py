"""Detect ChArUco corners in EVERY frame of one tripod calibration recording (std60, 0.5 dpt, 2026-10-23).

One static pose per recording, so all frames are (nearly) the same view. We keep every frame's corners and pick the
steady ones later (select.py). Board: first 4 recordings 6x4, rest 5x3 (laptop screen).
  ../venv_ffs/bin/python runs/lens_calib_f050_20261023/collect.py REC_NAME
→ views/<rec>.pkl  {rec, board, size, frames: [(idx, t_mid_ns, ids (N,), corners (N,2), sharpness)], n_frames}
  peek/<rec>.jpg   (middle frame with detections, half size)
"""
import csv
import pickle
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
S3D = HERE.parents[1]
sys.path.insert(0, str(S3D))
import aruco_scale  # noqa: E402

BOARD_6x4 = {"hs_20261023_201830_std60", "hs_20261023_203443_std60", "hs_20261023_204636_std60",
             "hs_20261023_204819_std60"}


def board_for(rec):
    return "board_laptop_6x4.json" if rec in BOARD_6x4 else "board_laptop_5x3.json"


def main(rec):
    bj = board_for(rec)
    board = aruco_scale.Board(aruco_scale.load_board_spec(str(S3D / "config" / "charuco" / bj)))
    if board.sy % 2 == 0:
        # aruco_scale turns on the legacy pattern for even rows (printed 4.5 boards), but charuco_laptop_6x4.png was made
        # with the new pattern → 0 corners with legacy, 15 without (checked on all 4 recordings, 2026-10-23).
        board.board.setLegacyPattern(False)
        d = board.detector
        board.detector = cv2.aruco.CharucoDetector(board.board, d.getCharucoParameters(), d.getDetectorParameters(),
                                                   cv2.aruco.RefineParameters())
    rd = S3D / "recordings" / rec
    rows = list(csv.DictReader(open(rd / "frames_cam5.csv")))
    t = [int(r["t_mid_ns"]) for r in rows]
    cap = cv2.VideoCapture(str(rd / "cam5.mp4"))
    frames, i, size, mid = [], 0, None, None
    n_total = len(rows)
    while True:
        ok, f = cap.read()
        if not ok:
            break
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        size = g.shape[::-1]
        c, ids, nm = board.detect(g)
        sharp = float(cv2.Laplacian(g, cv2.CV_64F).var())
        frames.append((i, t[i] if i < len(t) else -1, np.asarray(ids, int).ravel(), np.asarray(c, float).reshape(-1, 2),
                       sharp, int(nm), float(g.mean()), float((g >= 250).mean())))
        if i == min(n_total, 10 ** 9) // 2:
            mid = (f.copy(), c, ids)
        i += 1
    (HERE / "views").mkdir(exist_ok=True)
    (HERE / "peek").mkdir(exist_ok=True)
    pickle.dump(dict(rec=rec, board=bj, size=size, frames=frames, n_frames=i, n_csv=len(rows)),
                open(HERE / "views" / f"{rec}.pkl", "wb"))
    if mid is not None:
        f, c, ids = mid
        for p in np.asarray(c).reshape(-1, 2):
            cv2.circle(f, (int(p[0]), int(p[1])), 8, (0, 0, 255), 2)
        cv2.imwrite(str(HERE / "peek" / f"{rec}.jpg"), cv2.resize(f, None, fx=0.5, fy=0.5), [cv2.IMWRITE_JPEG_QUALITY, 80])
    n = [len(fr[2]) for fr in frames]
    print(f"{rec} {bj}: frames {i} (csv {len(rows)}), with corners {sum(k > 0 for k in n)}, corners max {max(n or [0])}"
          f" median {int(np.median(n or [0]))}")


if __name__ == "__main__":
    main(sys.argv[1])
