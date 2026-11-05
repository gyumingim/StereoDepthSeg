#!/usr/bin/env python3
"""scalehunt frame cache: decode cam5.mp4 ONCE exactly like lp_replay (OpenCV FFmpeg, 2 threads, BGR -> GRAY) and store
the gray frames lp_replay would feed (fed_selection fps 60, start_off 0, max_secs 60) -> CACHE/<rec>/{idx.npy, gray.u8}.

  python sh_cache.py REC CACHE_ROOT
sh_replay.py --cache CACHE_ROOT then serves those frames instead of decoding (identity gate: README 3.2).
"""
import csv, json, sys, time
from pathlib import Path
import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parents[1]
sys.path.insert(0, str(SCAN3D / "imu_learn")); sys.path.insert(0, str(SCAN3D))
import vio_bench_hs as vb  # noqa: E402

rec, root = Path(sys.argv[1]).resolve(), Path(sys.argv[2])
meta = json.load(open(rec / "meta.json"))
rows = list(csv.DictReader(open(rec / "frames_cam5.csv")))
fs = vb.fed_selection(rec, meta, rows, 60.0, 0.0, 60.0)
sel = np.asarray(fs["sel"]); t_frames = fs["t_frames"]
sel = sel[t_frames[sel] <= fs["t_end"]]
W, H = int(meta["width"]), int(meta["height"])
od = root / rec.name; od.mkdir(parents=True, exist_ok=True)
mm = np.lib.format.open_memmap(od / "gray.npy", mode="w+", dtype=np.uint8, shape=(len(sel), H, W))
cap = cv2.VideoCapture(str(rec / "cam5.mp4"), cv2.CAP_FFMPEG, [cv2.CAP_PROP_N_THREADS, 2])
want = {int(f): k for k, f in enumerate(sel)}
t0, n = time.time(), 0
for fi in range(int(sel[-1]) + 1):
    if not cap.grab():
        sys.exit(f"eof at {fi}")
    if fi in want:
        _, f = cap.retrieve()
        mm[want[fi]] = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY); n += 1
mm.flush()
np.save(od / "idx.npy", sel)
json.dump(dict(rec=rec.name, made=time.strftime("%F %T"), n=n, last_fi=int(sel[-1]), sel_args="fps 60 start_off 0 max_secs 60",
               wall_s=round(time.time() - t0, 1)), open(od / "cache.json", "w"))
print(rec.name, n, round(time.time() - t0, 1))
