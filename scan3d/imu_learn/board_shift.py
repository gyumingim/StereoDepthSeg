"""Find the moment the monitor board itself moved (e.g. browser window → full screen at the start of a recording) and
write imu_gt/trim.json so everything before it is ignored (train_imu, batch_sync, overlays).

A board move on the screen is a sudden jump of the camera position in board coordinates that the phone cannot make:
between two ground-truth frames less than `max_gap` apart the camera would have to move faster than `vmax`.
Usually there is also a short gap in the ground truth while the browser redraws.
  python imu_learn/board_shift.py recordings/hs_... [--within 60] [--write]
"""
import argparse
import json
from pathlib import Path

import numpy as np


def find_shift(g, within_s=60.0, step_mm=8.0, side_s=0.15, fit_mm=2.0, rot_deg=0.5, max_gap_ms=None):
    """A board move = clean step: on each side of a frame boundary the camera path is smooth (straight line over
    side_s fits within fit_mm), the two lines disagree at the boundary by > step_mm, and orientation does not step.
    Fast hand motion with noisy poses fails the smooth-sides test (checked on 5 recordings without a board move).
    max_gap_ms: ignore boundaries with a longer GT gap — extrapolating straight lines over seconds while the board is out
    of view is not a step test (hs_20261020_214138: 4 false 'shifts' over 1.2-5.5 s gaps with the gyro turning 19-69°)."""
    from scipy.spatial.transform import Rotation as Rot
    ok = g["ok"].astype(bool) & (g["t_ns"] > 0)
    t = (g["t_ns"][ok] - g["t_ns"][ok][0]) / 1e9
    p = g["p_bc"][ok] * 1000
    R = g["R_bc"][ok]
    out = []
    for k in range(1, len(t)):
        if t[k] > within_s:
            break
        if max_gap_ms is not None and (t[k] - t[k - 1]) * 1000 > max_gap_ms:
            continue
        A =np.arange(np.searchsorted(t, t[k - 1] - side_s), k)
        B = np.arange(k, np.searchsorted(t, t[k] + side_s, side="right"))
        if len(A) < 15 or len(B) < 15:
            continue
        fa = np.polyfit(t[A], p[A], 1)
        fb = np.polyfit(t[B], p[B], 1)
        ra = np.sqrt(np.mean(np.sum((p[A] - (np.outer(t[A], fa[0]) + fa[1])) ** 2, 1)))
        rb = np.sqrt(np.mean(np.sum((p[B] - (np.outer(t[B], fb[0]) + fb[1])) ** 2, 1)))
        tm = (t[k - 1] + t[k]) / 2
        jump = np.linalg.norm((fb[0] * tm + fb[1]) - (fa[0] * tm + fa[1]))
        if jump < step_mm or ra > fit_mm or rb > fit_mm:
            continue
        dr = np.degrees((Rot.from_matrix(R[k - 1]).inv() * Rot.from_matrix(R[k])).magnitude())
        if dr > rot_deg + 60 * (t[k] - t[k - 1]):                 # allow normal rotation over the gap (60 deg/s)
            continue
        out.append(dict(index=int(k), t_ns=int(g["t_ns"][ok][k]), t_s=round(float(t[k]), 3), jump_mm=round(float(jump), 1),
                        gap_ms=round(float((t[k] - t[k - 1]) * 1000), 1), fit_mm=round(float(max(ra, rb)), 2)))
    # one move often shows on 2-3 neighbouring boundaries: keep the largest per 0.5 s
    merged = []
    for x in out:
        if merged and x["t_s"] - merged[-1]["t_s"] < 0.5:
            if x["jump_mm"] > merged[-1]["jump_mm"]:
                merged[-1] = x
        else:
            merged.append(x)
    return merged


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recs", nargs="+")
    ap.add_argument("--within", type=float, default=60.0, help="only look in the first N seconds")
    ap.add_argument("--write", action="store_true", help="write imu_gt/trim.json (start after the last shift found)")
    ap.add_argument("--max-gap-ms", type=float, default=None,
                    help="ignore boundaries with a longer GT gap (board out of view; default: no limit = old behaviour)")
    a = ap.parse_args()
    for rec in a.recs:
        rec = Path(rec)
        g = dict(np.load(rec / "imu_gt" / "gt.npz"))
        s = find_shift(g, a.within, max_gap_ms=a.max_gap_ms)
        tp = rec / "imu_gt" / "trim.json"
        if a.write and not s and tp.exists() and "board_shift" in json.load(open(tp)).get("reason", ""):
            tp.rename(tp.with_suffix(".json.old"))                      # stale trim from an earlier run
            print("  (old trim.json → trim.json.old)")
        print(f"{rec.name}: {len(s)} board shift(s) in the first {a.within:.0f} s " +
              " ".join(f"[{x['t_s']} s: {x['jump_mm']} mm in {x['gap_ms']} ms]" for x in s))
        if a.write and s:
            last = s[-1]
            json.dump(dict(start_ns=last["t_ns"] + 200_000_000, reason="board moved on screen (board_shift.py)",
                           shifts=s), open(rec / "imu_gt" / "trim.json", "w"), indent=1)
            print(f"  → trim.json: ignore the first {last['t_s'] + 0.2:.1f} s")


if __name__ == "__main__":
    main()
