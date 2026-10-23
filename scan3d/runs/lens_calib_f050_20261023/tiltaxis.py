"""Tilt direction of each pose (board normal in camera frame, k1_prior full fit) + coverage picture → results/tiltaxis.json, results/coverage.png"""
import json, sys
from pathlib import Path
import cv2, numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import calib
views, clusters, members = calib.build()
r = calib.fit(views, "k1_prior")
out = {}
for v, rv, tv in zip(views, r[3], r[4]):
    R, _ = cv2.Rodrigues(rv)
    n = R[:, 2] * np.sign(R[2, 2])
    out.setdefault(v["rec"], []).append([float(np.degrees(np.arctan2(n[0], n[2]))), float(np.degrees(np.arctan2(n[1], n[2])))])
res = {k: dict(yaw_deg=round(float(np.mean([a[0] for a in x])), 1), pitch_deg=round(float(np.mean([a[1] for a in x])), 1)) for k, x in out.items()}
for k, x in res.items(): print(k[12:18], x)
json.dump(res, open(HERE / "results" / "tiltaxis.json", "w"), indent=1)
img = np.full((1080, 1920, 3), 255, np.uint8)
for i in range(1, 4): cv2.line(img, (480 * i, 0), (480 * i, 1079), (180, 180, 180), 2)
for j in range(1, 3): cv2.line(img, (0, 360 * j), (1919, 360 * j), (180, 180, 180), 2)
cols = {}
for v in views:
    t = calib.tilt_dist(*[x for x in (r[3][views.index(v)], r[4][views.index(v)])])[0]
    c = (0, 0, 220) if t > 25 else (200, 120, 0)
    for x, y in v["img"].reshape(-1, 2): cv2.circle(img, (int(x), int(y)), 6, c, -1)
for v in views[::5]:
    x, y = v["img"].reshape(-1, 2).mean(0); cv2.putText(img, v["rec"][12:18], (int(x) - 40, int(y) - 18), 0, 0.7, (0, 0, 0), 2)
cv2.putText(img, "red = tilted > 25 deg, blue = frontal; grid 4x3", (20, 1060), 0, 1.0, (0, 0, 0), 2)
cv2.imwrite(str(HERE / "results" / "coverage.png"), cv2.resize(img, (960, 540)))
