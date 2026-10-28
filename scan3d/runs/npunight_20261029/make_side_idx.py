#!/usr/bin/env python3
"""npunight 2026-10-29: STREAM.bin (PC tee of the bridge stdin) + REC/frames_cam5.csv -> OUTDIR/side.bin (camera pixels removed,
20 B header kept) + OUTDIR/idx.txt (source frame index in cam5.mp4 decode order per camera record) + OUTDIR/frames_ref.npz
(every 100th camera frame from the stream, to compare with the phone rebuild).
Matching: camera record t_ns = t_mid_ns + c (c = constant align offset); c is taken from the first record vs every frame and
kept if ALL records then match a frame within 0.3 ms (unique: frames are >= 4.1 ms apart).
  make_side_idx.py STREAM.bin REC OUTDIR"""
import csv
import json
import struct
import sys
from pathlib import Path

import numpy as np

st, rec, out = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
out.mkdir(parents=True, exist_ok=True)
tm = np.array([int(r["t_mid_ns"]) for r in csv.DictReader(open(rec / "frames_cam5.csv"))], np.int64)
ts, keep = [], {}
with open(st, "rb") as f, open(out / "side.bin", "wb") as g:
    while True:
        h = f.read(12)
        if len(h) < 12:
            break
        m, t, L = struct.unpack("<III", h)
        if t == 2:
            p = f.read(20)
            tns, w, hh = struct.unpack_from("<qII", p)
            if len(ts) % 100 == 0:
                keep[str(len(ts))] = np.frombuffer(f.read(w * hh), np.uint8).reshape(hh, w)
            else:
                f.seek(L - 20, 1)
            ts.append(tns)
            g.write(struct.pack("<III", m, 2, 20) + p)
        else:
            g.write(h + f.read(L))
ts = np.array(ts, np.int64)
best = None
for k in range(len(tm)):
    c = ts[0] - tm[k]
    j = np.clip(np.searchsorted(tm, ts - c), 1, len(tm) - 1)
    j = np.where(np.abs(tm[j - 1] + c - ts) < np.abs(tm[j] + c - ts), j - 1, j)
    err = np.abs(tm[j] + c - ts)
    if err.max() < 300_000:
        best = (k, int(c), j, int(err.max()))
        break
assert best is not None, "no constant offset matches all records"
k, c, j, emax = best
assert np.all(np.diff(j) > 0), "indices not rising"
(out / "idx.txt").write_text("\n".join(str(int(x)) for x in j) + "\n")
np.savez_compressed(out / "frames_ref.npz", **keep)
json.dump(dict(stream=str(st), rec=str(rec), camera_records=len(ts), offset_ns=c, first_idx=int(j[0]), last_idx=int(j[-1]),
               max_err_ns=emax, frames_in_csv=len(tm)), open(out / "idx.json", "w"), indent=1)
print(json.dumps(dict(camera_records=len(ts), offset_ms=c / 1e6, first_idx=int(j[0]), last_idx=int(j[-1]), max_err_us=emax / 1e3)))
