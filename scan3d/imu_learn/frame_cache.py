#!/usr/bin/env python3
"""Frame cache for vio_bench_hs.py --frame-cache (2026-10-23; off by default = bench unchanged).

Stores exactly the image the bench sends to the bridge for a frame index fi of cam5.mp4:
    g = cv2.remap(cv2.cvtColor(BGR frame of the OpenCV FFmpeg decoder, BGR2GRAY), m1, m2, INTER_LINEAR)
with the bench's default undistort maps (m1, m2) for the fed size (no --focus-comp: that changes the map per frame, so
the bench bypasses the cache then). The cache is per recording and per map: recordings/<rec>/cache/frames_<W>x<H>_<key>/
key = sha256 of (cv2 version, conversion recipe, sha256 of m1/m2 bytes = lens K/dist/Knew/size, frames_cam5.csv sha256,
video quick id = size + sha256 of its first and last 8 MiB). index.json additionally keeps the full video sha256 and the
time the cache was made. Frames are stored sparsely in chunks (c_<id>.u8 = raw uint8 (n, H, W), c_<id>.idx.npy = frame
indices); a bench run that misses any needed frame decodes as before and appends the frames it lacked as a new chunk
(parallel runs are safe: each writes its own chunk, then renames it into place; duplicates are harmless).
eof_<fi>.json records that cap.grab() failed at frame fi (video shorter than frames_cam5.csv), as the decode loop saw.

  ../venv_ffs/bin/python imu_learn/frame_cache.py build recordings/hs_... [--offsets 0,1,2] [--max-secs 0] [--threads 8]
  ../venv_ffs/bin/python imu_learn/frame_cache.py info recordings/hs_...
"""
import argparse
import csv
import hashlib
import json
import os
import sys
import time
import uuid
from pathlib import Path

import cv2
import numpy as np

RECIPE = "cvtColor(BGR2GRAY of cv2 FFmpeg decode) -> remap(m1,m2 CV_16SC2, INTER_LINEAR); uint8 HxW"
QUICK = 8 << 20


def sha256_file(p, limit=None):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        if limit is None:
            for b in iter(lambda: f.read(16 << 20), b""):
                h.update(b)
        else:
            h.update(f.read(limit))
    return h.hexdigest()


def video_quick_id(p):
    st = os.stat(p)
    h = hashlib.sha256(str(st.st_size).encode())
    with open(p, "rb") as f:
        h.update(f.read(QUICK))
        f.seek(max(0, st.st_size - QUICK))
        h.update(f.read(QUICK))
    return dict(size=st.st_size, head_tail_sha256=h.hexdigest())


class FrameCache:
    def __init__(self, rec, size, m1, m2):
        rec = Path(rec)
        self.W, self.H = size
        self.params = dict(recipe=RECIPE, cv2=cv2.__version__, size=[self.W, self.H],
                           maps_sha256=hashlib.sha256(np.ascontiguousarray(m1).tobytes()
                                                      + np.ascontiguousarray(m2).tobytes()).hexdigest(),
                           frames_csv_sha256=sha256_file(rec / "frames_cam5.csv"),
                           video=video_quick_id(rec / "cam5.mp4"))
        self.key = hashlib.sha256(json.dumps(self.params, sort_keys=True).encode()).hexdigest()[:16]
        self.dir = rec / "cache" / f"frames_{self.W}x{self.H}_{self.key}"
        self.video = rec / "cam5.mp4"
        self.load()

    def load(self):
        """(Re)read the chunk list: fi -> (memmap, row); eof = smallest frame index where grab() failed (None = unseen)."""
        self.where, self.mm, self.eof = {}, [], None
        if not self.dir.exists():
            return
        for idx in sorted(self.dir.glob("c_*.idx.npy")):
            fis = np.load(idx)
            data = idx.with_name(idx.name[:-len(".idx.npy")] + ".u8")
            if not len(fis) or data.stat().st_size != len(fis) * self.W * self.H:
                continue
            m = np.memmap(data, np.uint8, "r", shape=(len(fis), self.H, self.W))
            self.mm.append(m)
            for r, fi in enumerate(fis.tolist()):
                self.where.setdefault(fi, (m, r))
        e = [int(p.stem[4:]) for p in self.dir.glob("eof_*.json")]
        self.eof = min(e) if e else None

    def needed(self, sel):
        return [int(fi) for fi in sel if int(fi) not in self.where and not (self.eof is not None and fi >= self.eof)]

    def has_all(self, sel):
        return not self.needed(sel)

    def get(self, fi):
        m, r = self.where[fi]
        return np.asarray(m[r])

    def info(self):
        return dict(dir=str(self.dir), key=self.key, frames=len(self.where), chunks=len(self.mm), eof=self.eof,
                    bytes=sum(m.size for m in self.mm))

    def _ensure_dir(self):
        self.dir.mkdir(parents=True, exist_ok=True)
        ix = self.dir / "index.json"
        if not ix.exists():
            tmp = self.dir / f".index_{uuid.uuid4().hex}.tmp"
            tmp.write_text(json.dumps(dict(made=time.strftime("%F %T"), **self.params,
                                           video_sha256=sha256_file(self.video)), indent=1))
            os.replace(tmp, ix)

    def writer(self):
        return Writer(self)


class Writer:
    """Collects frames the cache lacks; close() publishes them as one chunk (rename = atomic for readers)."""

    def __init__(self, fc):
        self.fc, self.f, self.fis, self.id = fc, None, [], f"{time.strftime('%Y%m%d%H%M%S')}_{uuid.uuid4().hex[:8]}"

    def add(self, fi, g):
        if fi in self.fc.where:
            return
        if g.shape != (self.fc.H, self.fc.W) or g.dtype != np.uint8:
            raise ValueError(f"frame cache: unexpected frame {g.shape} {g.dtype}")
        if self.f is None:
            self.fc._ensure_dir()
            self.tmp = self.fc.dir / f".c_{self.id}.u8.tmp"
            self.f = open(self.tmp, "wb")
        self.f.write(np.ascontiguousarray(g).tobytes())
        self.fis.append(int(fi))

    def eof(self, fi):
        self.fc._ensure_dir()
        (self.fc.dir / f"eof_{int(fi)}.json").write_text(json.dumps(dict(made=time.strftime("%F %T"))))

    def close(self):
        if self.f is None:
            return 0
        self.f.close()
        self.f = None
        os.replace(self.tmp, self.fc.dir / f"c_{self.id}.u8")
        tmp = self.fc.dir / f".c_{self.id}.idx.tmp.npy"
        np.save(tmp, np.asarray(self.fis, np.int64))
        os.replace(tmp, self.fc.dir / f"c_{self.id}.idx.npy")
        return len(self.fis)


def build(rec, size="640x360", fps=60.0, offsets=(0.0,), max_secs=0.0, threads=0):
    """Decode once and cache the union of the bench's fed-frame selections (same vb.fed_selection) for the offsets."""
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import vio_bench_hs as vb
    rec = Path(rec)
    meta = json.load(open(rec / "meta.json"))
    m1, m2, _, (W, H) = vb.undistort_maps(rec, meta, size)
    rows = list(csv.DictReader(open(rec / "frames_cam5.csv")))
    need = set()
    for off in offsets:
        need.update(int(i) for i in vb.fed_selection(rec, meta, rows, fps, off, max_secs)["sel"])
    fc = FrameCache(rec, (W, H), m1, m2)
    todo = set(fc.needed(sorted(need)))
    t0 = time.time()
    out = dict(rec=rec.name, need=len(need), todo=len(todo))
    if todo:
        cap = (cv2.VideoCapture(str(rec / "cam5.mp4"), cv2.CAP_FFMPEG, [cv2.CAP_PROP_N_THREADS, threads]) if threads
               else cv2.VideoCapture(str(rec / "cam5.mp4")))
        w = fc.writer()
        last = max(todo)
        try:
            for fi in range(len(rows)):
                if not cap.grab():
                    w.eof(fi)
                    break
                if fi in todo:
                    _, f = cap.retrieve()
                    w.add(fi, cv2.remap(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), m1, m2, cv2.INTER_LINEAR))
                if fi >= last:
                    break
        finally:
            out["written"] = w.close()
        fc.load()
    out.update(secs=round(time.time() - t0, 1), **fc.info())
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["build", "info"])
    ap.add_argument("rec")
    ap.add_argument("--size", default="640x360")
    ap.add_argument("--fps", type=float, default=60)
    ap.add_argument("--offsets", default="0", help="start offsets s (comma list) whose fed frames are cached")
    ap.add_argument("--max-secs", type=float, default=0)
    ap.add_argument("--threads", type=int, default=0, help="decode threads (0 = OpenCV default; frames identical)")
    o = ap.parse_args()
    offs = [float(x) for x in o.offsets.split(",") if x.strip()]
    if o.cmd == "build":
        print(json.dumps(build(o.rec, o.size, o.fps, offs, o.max_secs, o.threads)), flush=True)
    else:
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        import vio_bench_hs as vb
        rec = Path(o.rec)
        m1, m2, _, (W, H) = vb.undistort_maps(rec, json.load(open(rec / "meta.json")), o.size)
        print(json.dumps(FrameCache(rec, (W, H), m1, m2).info()))


if __name__ == "__main__":
    main()
