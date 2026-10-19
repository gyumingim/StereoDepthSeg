"""240 fps 고속 녹화 (arlat hs) 의 프레임 시각을 results_cam5.csv (카메라가 준 시각 전부) 로 다시 짝지음.

녹화 앱은 영상 시각 (pts + clock_offset) 에서 2 ms 안의 센서 시각을 찾는데, 찾을 표를 8 초마다 비워서 (meta.clear)
짝을 못 찾은 프레임 (hs_20261019_114000: 525 장) 은 앞 프레임의 차이를 빌려 썼음 → 대부분 수십 µs, 드물게 프레임 하나 이상 틀림.
여기서는 녹화 끝난 뒤 전체 표로 다시 짝짓고, 고친 만큼 frames_cam5.csv 와 imu_gt/gt.npz 의 t_ns 를 옮김 (원본은 *.orig 로 남김).
  python imu_learn/hs_fix_ts.py recordings/hs_20261019_114000 [...]
"""
import csv
import json
import shutil
import sys
from pathlib import Path

import numpy as np


def fix(rec):
    rec = Path(rec)
    fc, rc = rec / "frames_cam5.csv", rec / "results_cam5.csv"
    if not rc.exists():
        print(f"{rec.name}: results_cam5.csv 없음 — 건너뜀")
        return
    off = json.load(open(rec / "meta.json"))["clock_offset_ns"]
    orig = rec / "frames_cam5.orig.csv"
    if not orig.exists():
        shutil.copy(fc, orig)
    rows = list(csv.DictReader(open(orig)))
    res = {}
    for r in csv.DictReader(open(rc)):
        if r["sensor_ts_ns"] in ("", "null"):
            continue
        res.setdefault(int(r["sensor_ts_ns"]), (int(r["exposure_ns"]), int(r["rolling_shutter_skew_ns"])))
    R = np.array(sorted(res))
    guess = np.array([int(r["pts_us"]) * 1000 + off for r in rows])
    i = np.clip(np.searchsorted(R, guess), 1, len(R) - 1)
    near = np.where(np.abs(R[i] - guess) < np.abs(R[i - 1] - guess), R[i], R[i - 1])
    hit = np.abs(near - guess) < 2_000_000
    old = np.array([int(r["physical_ts_ns"]) for r in rows])
    new = np.where(hit, near, old)
    delta = new - old
    with open(fc, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(list(rows[0].keys()))
        for k, r in enumerate(rows):
            ts = int(new[k])
            e, s = res.get(ts, (int(r["exposure_ns"]), int(r["rolling_shutter_skew_ns"])))
            w.writerow([r["frame_idx"], ts + e // 2 + s // 2, ts, ts, e, s, r["pts_us"]])
    gp = rec / "imu_gt" / "gt.npz"
    if gp.exists():
        go = rec / "imu_gt" / "gt.orig.npz"
        if not go.exists():
            shutil.copy(gp, go)
        g = dict(np.load(go))
        n = min(len(g["t_ns"]), len(delta))
        t = g["t_ns"].copy()
        m = t[:n] > 0
        t[:n][m] += delta[:n][m]
        g["t_ns"] = t
        np.savez(gp, **g)
    a = np.abs(delta) / 1e3
    print(f"{rec.name}: 프레임 {len(rows)}, 짝 찾음 {hit.sum()} (못 찾음 {(~hit).sum()}), 바뀐 시각 {(a > 0.5).sum()} 장 "
          f"(중앙 {np.median(a[a > 0.5]) if (a > 0.5).any() else 0:.1f} µs, 최대 {a.max():.1f} µs, 1 ms 넘게 {(a > 1000).sum()} 장)"
          + (" · gt.npz t_ns 도 옮김" if gp.exists() else ""))


if __name__ == "__main__":
    for r in sys.argv[1:]:
        fix(r)
