#!/usr/bin/env python3
"""rec960 2026-11-16 (B): emulate a 960x540 recording from a 1920x1080 std60 recording (offline, for the VIO 960-vs-1920 check).

  python make960.py SRC_REC DST_REC [--max-frames N]

cam5.mp4 (phone H.264 1920x1080) -> OpenCV/FFmpeg decode (BGR) -> cv2.resize 960x540 INTER_AREA (= exact 2x2 box average, the same
filter as the app's std960 GPU path) -> FFV1 (lossless) in DST/cam5.mp4, every decoded frame in order (frame count must equal the
source decode count; checked). All other files are copied; meta.json gets width/height 960x540 + an "emulated_960" block.

ASSUMPTION (not equal to a real std960 recording): the real app path is camera YUV -> GPU RGB 2x2 box -> phone H.264 at 40 Mbps.
Here the 1920 frames already went through the phone's 160 Mbps H.264 once, then a lossless 960 copy is made: no second lossy
encode (no H.264 encoder in this venv), no GPU colour round trip. Sensor binning / ISP are identical (same 1920 camera stream).
"""
import argparse, hashlib, json, shutil, sys, time
from pathlib import Path
import cv2, numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src"); ap.add_argument("dst")
    ap.add_argument("--max-frames", type=int, default=0, help="0 = all (test only)")
    a = ap.parse_args()
    src, dst = Path(a.src), Path(a.dst)
    t0 = time.time()
    if dst.exists():
        sys.exit(f"{dst} exists")
    tmp = dst.with_name(dst.name + ".part")
    shutil.rmtree(tmp, ignore_errors=True); tmp.mkdir(parents=True)
    for f in src.iterdir():
        if f.name in ("cam5.mp4", "scanA_cache", "cache", "meta.json"):
            continue
        (shutil.copytree if f.is_dir() else shutil.copy2)(f, tmp / f.name)
    cap = cv2.VideoCapture(str(src / "cam5.mp4"), cv2.CAP_FFMPEG)
    W, H = 960, 540
    wr = cv2.VideoWriter(str(tmp / "cam5.mp4"), cv2.CAP_FFMPEG, cv2.VideoWriter_fourcc(*"FFV1"), 60, (W, H))
    if not wr.isOpened():
        sys.exit("FFV1 writer not available")
    n, keep = 0, []
    while True:
        r, f = cap.read()
        if not r or (a.max_frames and n >= a.max_frames):
            break
        assert f.shape[:2] == (1080, 1920), f.shape
        s = cv2.resize(f, (W, H), interpolation=cv2.INTER_AREA)
        wr.write(s)
        if n % 200 == 0:
            keep.append((n, cv2.cvtColor(s, cv2.COLOR_BGR2GRAY)))
        n += 1
    cap.release(); wr.release()
    # round-trip check: decode the written file, same frame count, gray identical on the sampled frames
    cap = cv2.VideoCapture(str(tmp / "cam5.mp4"), cv2.CAP_FFMPEG)
    m, diffs, ki = 0, [], 0
    while True:
        r, f = cap.read()
        if not r:
            break
        if ki < len(keep) and keep[ki][0] == m:
            g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
            diffs.append(float(np.abs(g.astype(int) - keep[ki][1].astype(int)).max())); ki += 1
        m += 1
    cap.release()
    rows = sum(1 for _ in open(src / "frames_cam5.csv")) - 1
    if m != n:
        sys.exit(f"written {n} frames, decoded back {m}")
    meta = json.load(open(src / "meta.json"))
    meta.update(width=W, height=H, session_option="std960_emulated_offline", emulated_960=dict(
        made=time.strftime("%F %T"), src=str(src), src_mp4_bytes=(src / "cam5.mp4").stat().st_size,
        filter="OpenCV-FFmpeg decode BGR -> cv2.resize INTER_AREA 960x540 (exact 2x2 box)",
        codec="FFV1 lossless (OpenCV-FFmpeg writer), container cam5.mp4", frames=n, frames_csv_rows=rows,
        roundtrip_gray_maxdiff_sampled=max(diffs) if diffs else None, cv2=cv2.__version__,
        note="assumption: approximates, not equals, a native std960 recording (no 2nd H.264 at 40 Mbps, no GPU colour path)"))
    json.dump(meta, open(tmp / "meta.json", "w"), indent=1)
    tmp.rename(dst)
    info = dict(meta["emulated_960"], dst=str(dst), dst_mp4_bytes=(dst / "cam5.mp4").stat().st_size, wall_s=round(time.time() - t0, 1))
    print(json.dumps(info))


if __name__ == "__main__":
    main()
