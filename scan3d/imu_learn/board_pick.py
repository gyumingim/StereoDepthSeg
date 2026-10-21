#!/usr/bin/env python3
"""Pick the ChArUco board json of a recording from the ArUco marker ids seen in cam5.mp4 (overnight 2026-10-21).
ids 0-21  -> board_laptop_6x6.json (laptop screen, cropped monitor image, 42.86 mm) unless LOW_BOARD says otherwise
             (e.g. config/charuco/board_monitor_6x6.json when the input audit / IMU scale check shows the office monitor)
ids 100-121 -> board_laptop_fit_6x6.json (laptop board, 32.03 mm)
Samples N frames spread over the video, raw marker detection (DICT_6X6_250, no board model). Prints the json path on the
last stdout line; writes <rec>/board_pick.json (counts, decision). Exit 1 if no markers or both id ranges are seen.
  ../venv_ffs/bin/python imu_learn/board_pick.py recordings/hs_... [--n 40]
"""
import argparse
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

LOW = os.environ.get('LOW_BOARD', 'config/charuco/board_laptop_6x6.json')
HIGH = 'config/charuco/board_laptop_fit_6x6.json'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('rec')
    ap.add_argument('--n', type=int, default=40)
    o = ap.parse_args()
    rec = Path(o.rec)
    cap = cv2.VideoCapture(str(rec / 'cam5.mp4'))
    nf = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    p = cv2.aruco.DetectorParameters()
    p.minMarkerPerimeterRate = 0.01
    det = cv2.aruco.ArucoDetector(d, p)
    low = high = other = frames = 0
    for fi in np.linspace(0, max(nf - 1, 0), o.n).astype(int):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(fi))
        r, f = cap.read()
        if not r:
            continue
        frames += 1
        _, ids, _ = det.detectMarkers(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
        for i in ([] if ids is None else ids.ravel().tolist()):
            if 0 <= i <= 21:
                low += 1
            elif 100 <= i <= 121:
                high += 1
            else:
                other += 1
    res = dict(made=time.strftime('%F %T'), rec=rec.name, frames_sampled=frames, video_frames=nf, ids_0_21=low,
               ids_100_121=high, ids_other=other, low_board=LOW, high_board=HIGH)
    ok = (low + high) > 0 and min(low, high) <= 0.05 * max(low, high)
    res['board'] = (HIGH if high > low else LOW) if ok else None
    res['ok'] = ok
    json.dump(res, open(rec / 'board_pick.json', 'w'), indent=1)
    print(json.dumps(res), file=sys.stderr)
    if not ok:
        sys.exit(1)
    print(res['board'])


if __name__ == '__main__':
    main()
