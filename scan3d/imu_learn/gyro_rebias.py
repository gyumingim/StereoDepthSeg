#!/usr/bin/env python3
"""Causal streaming gyro bias re-estimation at every still (docs/gyro_rebias_apply_plan.md).

Standard IMU pre-processing step (user decision 2026-10-22, condition "D" of docs/imu_calib_gain.md + the D2 gate).
Input: raw gyro samples (Android type 16, values 0..2, rad/s) with timestamps (ns), one at a time.
Output: corrected gyro (w - bias) and the current bias. Uses past samples only; no numpy needed (portable to the
phone / STM32: fixed-size state, see docs/gyro_rebias_apply.md "porting").

Rule (all thresholds below, rad/s unless noted):
- Time is cut into 0.1 s blocks from the first sample; each block keeps n, sum, sum of squares per axis.
- A window = the last 5 blocks (0.5 s, every 0.1 s). Window is "still" if it has >= 20 samples, the per-axis std
  < STD_T and
    * before any bias exists: |mean| (norm) < INIT_MEAN_T  (bias unknown, so a loose absolute test)
    * after:  |mean - bias| < MEAN_T on every axis           (tight: catches slow rotation, 222747 y -47 vs bias 195)
- Consecutive still windows = a still segment [first window start, last window end].
- Bias update at the END of a segment (first non-still window) that lasted >= UPDATE_MIN_S: mean of the segment
  without its first TRIM_S (set-down shake, docs/endstill_*), i.e. >= 2 s of data. Short stills (1.5-4 s) are not
  used once a bias exists: slow hand rotation (< 100 mdeg/s) looks like a short still (222747 9.2-11.1 s,
  215348 141.6-144 s) and the real bias drifts only ~1 mdeg/s per s, so nothing is lost by waiting. Gate: rejected if any axis differs from
  the current bias by more than GATE (slow rotation taken for still); the gate is skipped while no bias exists.
- Start still (no bias yet): once the segment has lasted MIN_STILL_S the bias follows its running mean every block
  (so a filter started during the first still is not handed a 0.25 deg/s step later); at its end the trimmed mean
  replaces it and it becomes the first committed bias.
- Before any bias: bias = 0 (raw passes through).

  ../venv_ffs/bin/python imu_learn/gyro_rebias.py recordings/hs_...   # print segments and bias updates
"""
import math
import sys

BLOCK_S = 0.1          # window step = block length
WIN_BLOCKS = 5         # 0.5 s window (same as i3_confirm.still_runs)
MIN_N = 20             # samples per window (~500 Hz -> ~250 normally)
STD_T = 0.01           # rad/s (573 mdeg/s) per-axis std; true stills <= 502 mdeg/s on y (8 recs, 2026-10-20/21)
INIT_MEAN_T = 0.0175   # rad/s (1000 mdeg/s) |mean| norm before a bias exists (was 0.03 = 1700; S24 bias norm ~400)
MEAN_T = 0.00175       # rad/s (100 mdeg/s) per-axis |mean - bias| once a bias exists
MIN_STILL_S = 1.5      # minimum start still (first bias)
UPDATE_MIN_S = 4.0     # minimum still segment for a later update
TRIM_S, TRIM_MIN_S = 2.0, 4.0  # drop the first 2 s of a segment >= 4 s
GATE = 0.00175         # rad/s (100 mdeg/s) max per-axis jump of a bias update


class GyroRebias:
    def __init__(self):
        self.t0 = None                 # ns, block grid origin
        self.k = 0                     # index of the block being filled
        self.cur = [0, [0.0] * 3, [0.0] * 3]          # n, sum, sumsq
        self.ring = []                 # last WIN_BLOCKS completed blocks: (k, n, sum, sumsq)
        self.bias = None               # committed bias (rad/s) or None
        self.prov = None               # provisional bias during the start still
        self.seg = None                # [k_first, k_last, n, sum, n_trim, sum_trim] of the open still segment
        self.log = []                  # (t_ns, event, values) for inspection

    # --- public
    def update(self, t_ns, w):
        """Feed one raw sample; returns (corrected w, bias in use). w: 3 floats rad/s."""
        if self.t0 is None:
            self.t0 = t_ns
        kb = int((t_ns - self.t0) // int(BLOCK_S * 1e9))
        while kb > self.k:             # close finished blocks (also empty ones across gaps)
            self._close_block(t_ns)
        c = self.cur
        c[0] += 1
        for i in range(3):
            c[1][i] += w[i]
            c[2][i] += w[i] * w[i]
        b = self.current_bias()
        return [w[i] - b[i] for i in range(3)], b

    def current_bias(self):
        b = self.bias if self.bias is not None else self.prov
        return b if b is not None else [0.0, 0.0, 0.0]

    # --- internals
    def _close_block(self, t_ns):
        n, s, q = self.cur
        self.ring.append((self.k, n, s, q))
        if len(self.ring) > WIN_BLOCKS:
            self.ring.pop(0)
        self.k += 1
        self.cur = [0, [0.0] * 3, [0.0] * 3]
        if len(self.ring) == WIN_BLOCKS and self.ring[0][0] == self.k - WIN_BLOCKS:
            self._window(t_ns)

    def _window(self, t_ns):
        n = sum(r[1] for r in self.ring)
        still = False
        if n >= MIN_N:
            m = [sum(r[2][i] for r in self.ring) / n for i in range(3)]
            sd = [math.sqrt(max(sum(r[3][i] for r in self.ring) / n - m[i] * m[i], 0.0)) for i in range(3)]
            if max(sd) < STD_T:
                if self.bias is None:
                    still = math.sqrt(sum(x * x for x in m)) < INIT_MEAN_T
                else:
                    still = all(abs(m[i] - self.bias[i]) < MEAN_T for i in range(3))
        if still:
            if self.seg is None:       # new segment: all 5 blocks of this window
                k_first = self.ring[0][0]
                self.seg = [k_first, k_first - 1, 0, [0.0] * 3, 0, [0.0] * 3]
                for r in self.ring:
                    self._seg_add(r)
            else:                      # extend by the newest block
                self._seg_add(self.ring[-1])
            if self.bias is None and self._seg_len() >= MIN_STILL_S - 1e-9 and self.seg[2]:
                self.prov = [x / self.seg[2] for x in self.seg[3]]
        elif self.seg is not None:
            self._seg_end(t_ns)

    def _seg_add(self, r):
        k, n, s, _ = r
        g = self.seg
        g[1] = k
        g[2] += n
        for i in range(3):
            g[3][i] += s[i]
        if k >= g[0] + round(TRIM_S / BLOCK_S):
            g[4] += n
            for i in range(3):
                g[5][i] += s[i]

    def _seg_len(self):
        return (self.seg[1] + 1 - self.seg[0]) * BLOCK_S

    def _seg_end(self, t_ns):
        g, L = self.seg, self._seg_len()
        self.seg = None
        if L < (MIN_STILL_S if self.bias is None else UPDATE_MIN_S) - 1e-9:
            if self.bias is None:
                self.prov = None
            return
        n, s = (g[4], g[5]) if (L >= TRIM_MIN_S - 1e-9 and g[4]) else (g[2], g[3])
        if not n:
            return
        cand = [x / n for x in s]
        if self.bias is not None and any(abs(cand[i] - self.bias[i]) > GATE for i in range(3)):
            self.log.append((t_ns, "gated", L, cand))
            return
        self.bias, self.prov = cand, None
        self.log.append((t_ns, "update", L, cand))


def correct_array(t_ns, w):
    """Run the stream over arrays (t_ns (n,), w (n,3)); returns (w_corrected (n,3), bias (n,3), log). numpy only here."""
    import numpy as np
    g = GyroRebias()
    out, bias = np.empty_like(np.asarray(w, float)), np.empty((len(t_ns), 3))
    for j in range(len(t_ns)):
        c, b = g.update(int(t_ns[j]), [float(x) for x in w[j]])
        out[j], bias[j] = c, b
    return out, bias, g.log


if __name__ == "__main__":
    from pathlib import Path
    import numpy as np
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from imu_gt import read_imu
    _, gyr = read_imu(Path(sys.argv[1]) / "imu.csv", raw=True)
    _, b, lg = correct_array(gyr[:, 0].astype(np.int64), gyr[:, 1:4])
    for t, ev, L, v in lg:
        print(f"{(t - gyr[0, 0]) / 1e9:7.1f} s {ev:6s} seg {L:5.1f} s bias mdeg/s {np.round(np.degrees(v) * 1e3, 1)}")
    print("final bias mdeg/s", np.round(np.degrees(b[-1]) * 1e3, 1))
