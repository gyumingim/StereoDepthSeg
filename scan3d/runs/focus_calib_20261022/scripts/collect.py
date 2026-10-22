"""Copy of lens_calib.collect that also keeps frame index, gyro rate and Laplacian sharpness per view.
usage: collect.py REC OUT.pkl"""
import csv, pickle, sys
from pathlib import Path
import cv2, numpy as np
S3 = Path.home() / "scanner/StereoDepthSeg/scan3d"
sys.path[:0] = [str(S3), str(S3 / "imu_learn")]
import aruco_scale
from imu_gt import read_imu

rec, out = Path(sys.argv[1]), sys.argv[2]
board = aruco_scale.Board(aruco_scale.load_board_spec(str(S3 / "config/charuco/board_monitor_6x6.json")))
rows = list(csv.DictReader(open(rec / "frames_cam5.csv")))
t = np.array([int(r["t_mid_ns"]) for r in rows])
_, gyr = read_imu(rec / "imu.csv")
w = np.linalg.norm(np.stack([np.interp(t, gyr[:, 0], gyr[:, 1 + j]) for j in range(3)], 1), axis=1)
slow = np.where(w < 0.3)[0]
want = set(slow[np.linspace(0, len(slow) - 1, min(1500, len(slow))).astype(int)].tolist())
cap = cv2.VideoCapture(str(rec / "cam5.mp4"))
views, size, i = [], None, 0
while True:
    r, f = cap.read()
    if not r:
        break
    if i in want:
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        size = g.shape[::-1]
        c, ids, _ = board.detect(g)
        if len(ids) >= 20:
            obj = board.obj[ids].astype(np.float32)
            img = c.astype(np.float32).reshape(-1, 1, 2)
            H, _ = cv2.findHomography(obj[:, :2], img.reshape(-1, 2))
            tilt = 0.0 if H is None else float(np.degrees(np.arctan(np.hypot(H[2, 0], H[2, 1]) * 0.3)))
            x0, y0 = np.floor(img.reshape(-1, 2).min(0)).astype(int)
            x1, y1 = np.ceil(img.reshape(-1, 2).max(0)).astype(int)
            roi = g[max(y0, 0):y1, max(x0, 0):x1]
            sharp = float(cv2.Laplacian(roi, cv2.CV_64F).var()) if roi.size else 0.0
            views.append(dict(obj=obj, img=img, cx=float(img[:, 0, 0].mean()), cy=float(img[:, 0, 1].mean()),
                              tilt=tilt, fi=i, w=float(w[i]), sharp=sharp, ids=np.asarray(ids)))
    i += 1
pickle.dump(dict(views=views, size=size, n_slow=len(slow), n_frames=len(rows)), open(out, "wb"))
print(rec.name, "slow", len(slow), "views", len(views), "size", size)
