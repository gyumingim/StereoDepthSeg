"""Save a few frames with detected corners drawn (visual check). usage: peek.py REC_SHORT N"""
import pickle, sys
from pathlib import Path
import cv2, numpy as np
D = Path(__file__).resolve().parent
S3 = Path.home() / "scanner/StereoDepthSeg/scan3d"
r, n = sys.argv[1], int(sys.argv[2])
P = pickle.load(open(D / "views" / f"{r}.pkl", "rb"))
V = P["views"]
tl = np.array([v["tilt"] for v in V])
order = np.argsort(tl)
pick = sorted(set([V[k]["fi"] for k in order[np.linspace(0, len(order) - 1, n).astype(int)]]))
byfi = {v["fi"]: v for v in V}
cap = cv2.VideoCapture(str(S3 / f"recordings/hs_20261022_{r}/cam5.mp4"))
(D / "peek").mkdir(exist_ok=True)
i = 0
while i <= pick[-1]:
    if i in pick:
        ok, f = cap.read()
        v = byfi[i]
        for (x, y), cid in zip(v["img"].reshape(-1, 2), v["ids"]):
            cv2.circle(f, (int(x), int(y)), 6, (0, 0, 255), 2)
            cv2.putText(f, str(cid), (int(x) + 5, int(y) - 5), 0, 0.6, (0, 255, 255), 2)
        cv2.putText(f, f"fi {i} tilt {v['tilt']:.0f} roll {v['roll']:.0f} d {v['dist_m0']:.2f}", (30, 50), 0, 1.2, (0, 255, 0), 3)
        cv2.imwrite(str(D / "peek" / f"{r}_{i}.jpg"), cv2.resize(f, (960, 540)))
    else:
        cap.grab()
    i += 1
print(pick)
