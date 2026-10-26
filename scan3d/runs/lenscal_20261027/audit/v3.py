import json, os, numpy as np, cv2
d = json.load(open(os.path.expanduser("~/scanner/tmp_lenscal/views_calcap.json")))
K = np.array([[1286.25,0,965.2],[0,1286.4,553.2],[0,0,1.]])
s = []
for v in d["views"]:
    o = np.array(v["obj"], float).reshape(-1,3); q = np.array(v["img"], float).reshape(-1,2)
    ok, r, t = cv2.solvePnP(o, q, K, None)
    R, _ = cv2.Rodrigues(r); s.append(float(R[:,2] @ t.ravel()))
s = np.array(s); print("bowscan obj frame: frac board +z away from camera %.2f (n %d)" % (np.mean(s>0), len(s)))
print("obj x range", np.min([np.min(np.array(v["obj"])[:,0]) for v in d["views"]]), np.max([np.max(np.array(v["obj"])[:,0]) for v in d["views"]]), "sq", d["board_sq"])
