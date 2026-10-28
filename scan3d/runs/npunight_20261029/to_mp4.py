#!/usr/bin/env python3
"""Split a raw bridge stream into (a) side.bin = same records with camera pixel payloads removed (20 B header kept) and
(b) frames.mp4 = the gray frames as H.264 (yuv420p, chroma 128) so the phone can rebuild the raw stream locally.
  to_mp4.py IN.bin OUTDIR [--crf 14] [--max-frames N]
Also writes check.json: mean abs error of the decoded (PC) frames vs the originals (every 50th frame)."""
import argparse
import json
import struct
from pathlib import Path

import av
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("inp"); ap.add_argument("out")
ap.add_argument("--crf", default="14"); ap.add_argument("--max-frames", type=int, default=0)
a = ap.parse_args()
out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
oc = av.open(str(out / "frames.mp4"), "w")
st = oc.add_stream("libx264", rate=30)
st.width, st.height, st.pix_fmt = 1920, 1080, "yuv420p"
st.options = {"crf": a.crf, "preset": "medium", "profile": "high", "level": "4.2", "bf": "0", "g": "60"}
n = 0
keep = {}
with open(a.inp, "rb") as f, open(out / "side.bin", "wb") as g:
    while True:
        h = f.read(12)
        if len(h) < 12:
            break
        m, t, L = struct.unpack("<III", h)
        p = f.read(L)
        if t == 2:
            if a.max_frames and n >= a.max_frames:
                break
            _, w, hh, nc = struct.unpack_from("<qIIB", p)
            y = np.frombuffer(p, np.uint8, w * hh, 20).reshape(hh, w)
            yuv = np.full((hh * 3 // 2, w), 128, np.uint8)
            yuv[:hh] = y
            fr = av.VideoFrame.from_ndarray(yuv, format="yuv420p")
            for pk in st.encode(fr):
                oc.mux(pk)
            if n % 50 == 0:
                keep[n] = y.copy()
            n += 1
            g.write(struct.pack("<III", m, 2, 20) + p[:20])
        else:
            g.write(h + p)
for pk in st.encode():
    oc.mux(pk)
oc.close()
err = {}
ic = av.open(str(out / "frames.mp4"))
for i, fr in enumerate(ic.decode(video=0)):
    if i in keep:
        err[i] = float(np.abs(fr.to_ndarray(format="yuv420p")[:1080].astype(int) - keep[i]).mean())
json.dump(dict(frames=n, crf=a.crf, size_mb=(out / "frames.mp4").stat().st_size / 1e6, mae=err), open(out / "check.json", "w"))
print("frames", n, "mp4 MB", round((out / "frames.mp4").stat().st_size / 1e6, 1), "MAE mean", round(np.mean(list(err.values())), 2))
