"""imu_gt 결과 (imu_gt/gt.npz) → 영상 위에 그릴 프레임별 보드 표시 (imu_gt/overlay.json, docs/rec_view.html 이 읽음).

프레임마다: 영상 시각 (pts, 초), 찾은 모서리 수, 재투영 오차, 정답 OK 여부, 보드 안쪽 모서리 점들 (자세로 다시 투영한 화면 좌표, 화면 밖은 null).
  python imu_learn/gt_overlay.py recordings/hs_20261019_114000 [--board config/charuco/board_monitor_6x6.json]
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import aruco_scale  # noqa: E402
import imu_gt  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("--cam", default="5")
    ap.add_argument("--board", default=str(imu_gt.BOARD_JSON))
    a = ap.parse_args()
    rec = Path(a.rec)
    spec = aruco_scale.load_board_spec(a.board)
    sx, sy, sq = spec["squares_x"], spec["squares_y"], spec["square_m"]
    # inner chessboard corners (the ChArUco points). Only points in front of the lens and inside the view are kept:
    # the distortion polynomial blows up outside the view, so a projected board outline was garbage (board > view).
    grid = np.array([[i * sq, j * sq, 0] for j in range(1, sy) for i in range(1, sx)], np.float64)
    g = dict(np.load(rec / "imu_gt" / "gt.npz"))   # dict: NpzFile re-reads the array on every index
    K, dist = g["K"], g["dist"]
    rows = list(csv.DictReader(open(rec / f"frames_cam{a.cam}.csv")))
    pts = np.array([int(r["pts_us"]) for r in rows], np.int64)
    meta = json.load(open(rec / "meta.json"))
    Wimg, Himg = int(meta["width"]), int(meta["height"])
    n = min(len(pts), len(g["ok"]))
    quads = []
    for i in range(n):
        if not (g["n_corners"][i] >= 8 and np.isfinite(g["rms_px"][i])):
            quads.append(None)
            continue
        R_cb = g["R_bc"][i].T
        t_cb = -R_cb @ g["p_bc"][i]
        Xc = grid @ R_cb.T + t_cb
        z = Xc[:, 2]
        x, y = Xc[:, 0] / np.maximum(z, 1e-6), Xc[:, 1] / np.maximum(z, 1e-6)
        vis = (z > 0) & (x > -K[0, 2] / K[0, 0] * 1.02) & (x < (Wimg - K[0, 2]) / K[0, 0] * 1.02)             & (y > -K[1, 2] / K[1, 1] * 1.02) & (y < (Himg - K[1, 2]) / K[1, 1] * 1.02)
        q, _ = cv2.projectPoints(grid, cv2.Rodrigues(R_cb)[0], t_cb, K, dist)
        q = q.reshape(-1, 2)
        quads.append([None if not v else [round(float(q[k, 0]), 1), round(float(q[k, 1]), 1)] for k, v in enumerate(vis)])
    rms = g["rms_px"][:n]
    out = dict(rec=rec.name, width=Wimg, height=Himg, gx=sx - 1, gy=sy - 1,
               t=[round((int(p) - int(pts[0])) / 1e6, 6) for p in pts[:n]],
               n=[int(v) for v in g["n_corners"][:n]], ok=[int(v) for v in g["ok"][:n]],
               rms=[None if not np.isfinite(v) else round(float(v), 2) for v in rms], quad=quads)
    (rec / "imu_gt" / "overlay.json").write_text(json.dumps(out, separators=(",", ":")))
    ok = np.asarray(out["ok"], bool)
    print(f"overlay.json: 프레임 {n}, 정답 OK {ok.sum()} ({ok.mean() * 100:.1f} %)")


if __name__ == "__main__":
    main()
