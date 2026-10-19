"""imu_gt 결과 (imu_gt/gt.npz) → 영상 위에 그릴 프레임별 보드 표시 (imu_gt/overlay.json, docs/rec_view.html 이 읽음).

프레임마다: 영상 시각 (pts, 초), 찾은 모서리 수, 재투영 오차, 정답 OK 여부, 보드 바깥 테두리 4점 (자세로 다시 투영한 화면 좌표).
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
    W, H = spec["squares_x"] * spec["square_m"], spec["squares_y"] * spec["square_m"]
    outline = np.array([[0, 0, 0], [W, 0, 0], [W, H, 0], [0, H, 0]], np.float64)
    g = np.load(rec / "imu_gt" / "gt.npz")
    K, dist = g["K"], g["dist"]
    rows = list(csv.DictReader(open(rec / f"frames_cam{a.cam}.csv")))
    pts = np.array([int(r["pts_us"]) for r in rows], np.int64)
    n = min(len(pts), len(g["ok"]))
    quads = []
    for i in range(n):
        if not (g["n_corners"][i] >= 8 and np.isfinite(g["rms_px"][i])):
            quads.append(None)
            continue
        R_cb = g["R_bc"][i].T
        t_cb = -R_cb @ g["p_bc"][i]
        q, _ = cv2.projectPoints(outline, cv2.Rodrigues(R_cb)[0], t_cb, K, dist)
        quads.append([round(float(v), 1) for v in q.reshape(-1)])
    rms = g["rms_px"][:n]
    out = dict(rec=rec.name, width=int(json.load(open(rec / "meta.json"))["width"]),
               height=int(json.load(open(rec / "meta.json"))["height"]),
               t=[round((int(p) - int(pts[0])) / 1e6, 6) for p in pts[:n]],
               n=[int(v) for v in g["n_corners"][:n]], ok=[int(v) for v in g["ok"][:n]],
               rms=[None if not np.isfinite(v) else round(float(v), 2) for v in rms], quad=quads)
    (rec / "imu_gt" / "overlay.json").write_text(json.dumps(out, separators=(",", ":")))
    ok = np.asarray(out["ok"], bool)
    print(f"overlay.json: 프레임 {n}, 정답 OK {ok.sum()} ({ok.mean() * 100:.1f} %)")


if __name__ == "__main__":
    main()
