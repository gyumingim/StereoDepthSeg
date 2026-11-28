#!/usr/bin/env python3
"""vc_holdout 2026-11-28: helper -- grid overlay of keyframe photos (for picking regions by eye). usage: look.py OUT.jpg KF [KF ...]"""
import sys
import cv2
import numpy as np

D = "/home/system/scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2/dump_ba/img/"
ims = []
for k in sys.argv[2:]:
    im = cv2.imread(D + f"{int(k):05d}.png")
    for x in range(0, 960, 50):
        cv2.line(im, (x, 0), (x, 539), (0, 255, 255) if x % 100 == 0 else (0, 128, 128), 1)
    for y in range(0, 540, 50):
        cv2.line(im, (0, y), (959, y), (0, 255, 255) if y % 100 == 0 else (0, 128, 128), 1)
    cv2.putText(im, f"kf {k}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    ims.append(im)
cv2.imwrite(sys.argv[1], np.vstack(ims), [cv2.IMWRITE_JPEG_QUALITY, 85])
