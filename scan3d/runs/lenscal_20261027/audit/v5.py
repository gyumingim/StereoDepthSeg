import json, numpy as np, cv2
base = "/home/system/scanner/StereoDepthSeg/scan3d/config/phone_ov_main/"
v = json.load(open(base + "variants.json"))
K = np.array(v["K_raw_used"]); dist = np.array(v["dist"]); Kn = np.array(v["variant_sizes"]["c4"]["Knew"])
raw = open(base + "c4/remap_1920x1080.bin", "rb").read()
w, h = np.frombuffer(raw[:8], np.int32); n = w * h
mx = np.frombuffer(raw[8:8+4*n], np.float32).reshape(h, w); my = np.frombuffer(raw[8+4*n:8+8*n], np.float32).reshape(h, w)
print("map", w, h, "src", np.frombuffer(raw[8+8*n:], np.int32), "cv2", cv2.__version__)
fx, fy = cv2.initUndistortRectifyMap(K, dist, None, Kn, (int(w), int(h)), cv2.CV_32FC1)
d = np.hypot(fx - mx, fy - my); print("float map vs file: max %.4f px  p99 %.4f  mean %.4f" % (d.max(), np.percentile(d, 99), d.mean()))
m1, m2 = cv2.initUndistortRectifyMap(K, dist, None, Kn, (int(w), int(h)), cv2.CV_16SC2)
qx = m1[..., 0] + (m2 & 31) / 32.0; qy = m1[..., 1] + (m2 >> 5) / 32.0
print("fixed-point regenerated vs file: identical", np.array_equal(qx.astype(np.float32), mx) and np.array_equal(qy.astype(np.float32), my), "max %.4f" % np.max(np.hypot(qx - mx, qy - my)))
Kv, _ = cv2.getOptimalNewCameraMatrix(K, dist, (1920, 1080), 0, (960, 540)); print("Knew recomputed - stored max", np.abs(Kv - Kn).max())
# top-level
raw = open(base + "remap_1920x1080.bin", "rb").read(); print("top-level remap_1920x1080 map size", np.frombuffer(raw[:8], np.int32), "src", np.frombuffer(raw[-8:], np.int32), "bytes", len(raw))
