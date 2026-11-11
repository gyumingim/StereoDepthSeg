"""kfonly 2026-11-11: frame source for kf_ba.py / scan_offline.py.

iter_frames(rec, wanted) yields (frame_index, BGR image) for the wanted recording frame indices, in increasing order.
  * REC/frames_kf/ exists (a reduced "phone sends keyframes only" recording, scan3d/runs/kfonly_20261111/README.md):
    reads only REC/frames_kf/NNNNN.jpg; a wanted frame that is not there is an error (exit) -- nothing falls back to a video.
  * otherwise: sequential decode of REC/cam5.mp4 (the original code path, unchanged behaviour).
"""
import sys
from pathlib import Path

import cv2


def iter_frames(rec, wanted):
    rec = Path(rec)
    wanted = sorted(set(int(i) for i in wanted))
    kd = rec / "frames_kf"
    if kd.is_dir():
        if (rec / "cam5.mp4").exists():
            sys.exit(f"{rec}: has both frames_kf/ and cam5.mp4 -- a reduced recording must not hold the video")
        miss = [i for i in wanted if not (kd / f"{i:05d}.jpg").exists()]
        if miss:
            sys.exit(f"{rec}/frames_kf: {len(miss)} wanted frames were not sent (first {miss[:10]})")
        for i in wanted:
            f = cv2.imread(str(kd / f"{i:05d}.jpg"), cv2.IMREAD_COLOR)
            if f is None:
                sys.exit(f"{kd}/{i:05d}.jpg unreadable")
            yield i, f
        return
    ws = set(wanted)
    last = wanted[-1] if wanted else -1
    cap = cv2.VideoCapture(str(rec / "cam5.mp4"))
    fi = 0
    while fi <= last:
        r_, f = cap.read()
        if not r_:
            break
        if fi in ws:
            yield fi, f
        fi += 1
    cap.release()
