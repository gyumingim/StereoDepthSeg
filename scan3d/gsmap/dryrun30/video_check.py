import cv2, numpy as np, json, time
A = "/home/system/scanner/StereoDepthSeg/scan3d/recordings/hs_20261112_121801_std60/cam5.mp4"
B = "/home/system/scanner/StereoDepthSeg-gsmap/scan3d/runs/dryrun30_121801_20261127/rec_hs_20261112_121801_std30/cam5.mp4"
ca, cb = cv2.VideoCapture(A), cv2.VideoCapture(B)
check = set(range(0, 1733, 97)) | {1732}
na = nb = 0; maxd = {}; fa = None
while True:
    ra, a = ca.read()
    if not ra: break
    if na % 2 == 0:
        rb, b = cb.read()
        if not rb: break
        k = na // 2
        if k in check: maxd[k] = int(np.abs(a.astype(int) - b.astype(int)).max())
        nb += 1
    na += 1
while cb.read()[0]: nb += 1
res = dict(made=time.strftime("%F %T"), src_frames=na, dec_frames=nb, max_abs_diff_bgr=maxd)
print(json.dumps(res))
json.dump(res, open("/home/system/scanner/StereoDepthSeg-gsmap/scan3d/runs/dryrun30_121801_20261127/video_check.json", "w"), indent=1)
