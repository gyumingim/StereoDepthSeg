"""고속 녹화의 자기 보정 (렌즈 값) 이 얼마나 흔들리는지: 느린 (자이로 작은) 프레임만 골라 겹치지 않는 묶음 3 개로 따로 보정.
  python imu_learn/calib_check.py recordings/hs_... [--board ...] [--per 60]
f 가 묶음마다 1 % 넘게 다르면 imu_gt 의 80 프레임 자기 보정을 믿기 어렵다는 뜻.
"""
import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import aruco_scale  # noqa: E402
from imu_gt import read_imu  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("--board", default="config/charuco/board_monitor_6x6.json")
    ap.add_argument("--per", type=int, default=60)
    ap.add_argument("--max-w", type=float, default=0.3, help="이보다 느린 프레임만 (rad/s)")
    a = ap.parse_args()
    rec = Path(a.rec)
    board = aruco_scale.Board(aruco_scale.load_board_spec(a.board))
    rows = list(csv.DictReader(open(rec / "frames_cam5.csv")))
    t = np.array([int(r["t_mid_ns"]) for r in rows])
    _, gyr = read_imu(rec / "imu.csv")
    w = np.linalg.norm(np.stack([np.interp(t, gyr[:, 0], gyr[:, 1 + j]) for j in range(3)], 1), axis=1)
    slow = np.where(w < a.max_w)[0]
    want = set(slow[np.linspace(0, len(slow) - 1, a.per * 3 * 3).astype(int)].tolist())
    cap = cv2.VideoCapture(str(rec / "cam5.mp4"))
    obj, img, idx, size = [], [], [], None
    i = 0
    while True:
        r, f = cap.read()
        if not r:
            break
        if i in want:
            g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
            size = g.shape[::-1]
            c, ids, _ = board.detect(g)
            if len(ids) >= 12:
                obj.append(board.obj[ids].astype(np.float32))
                img.append(c.astype(np.float32).reshape(-1, 1, 2))
                idx.append(i)
        i += 1
    print(f"{rec.name}: 느린 프레임 {len(slow)} 중 보드 {len(obj)} 장 (|ω| < {a.max_w} rad/s)")
    order = np.arange(len(obj))
    groups = [order[k::3][:a.per] for k in range(3)] + [order]
    for k, gsel in enumerate(groups):
        rms, K, dist, _, _ = cv2.calibrateCamera([obj[j] for j in gsel], [img[j] for j in gsel], size, None, None,
                                                 flags=cv2.CALIB_FIX_K3)
        name = f"묶음 {k + 1}" if k < 3 else "전부"
        print(f"  {name} ({len(gsel)} 장): rms {rms:.2f} px, f {K[0, 0]:.1f} {K[1, 1]:.1f}, c {K[0, 2]:.1f} {K[1, 2]:.1f}, "
              f"dist {np.round(dist.ravel()[:4], 3).tolist()}")


if __name__ == "__main__":
    main()
