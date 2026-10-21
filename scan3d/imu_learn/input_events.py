#!/usr/bin/env python3
"""Amendment 3 of docs/i3_confirm_plan.md (2026-10-21 23:45, post-hoc: decided after seeing 213341/213955 results, before
214550/215348): input-event detection on the raw accelerometer (type 35).

Window = [end of the start still, start of the first set-down still] on the IMU clock. Stills come from the SAME detector
as amendments 1/2 (i3_confirm.still_runs, imported, not re-implemented); start-still / set-down selection follows
amendment 2 (i3_confirm.inputs). If runs/i3_confirm/inputs/<rec>.json exists its intervals are used (and the recomputed
ones must match); otherwise the recomputed ones are used and marked provisional.
If there is no set-down still, the window runs to the last frame.

Event = any sample in the window with |a| > 39 m/s^2 (4 g), or a run of consecutive samples with |a| < 2 m/s^2 lasting
> 0.1 s (duration = t_last - t_first of the run; a sample gap > 20 ms breaks a run).

  ../venv_ffs/bin/python imu_learn/input_events.py [recs...]      # -> docs/i3_input_events.json
"""
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import i3_confirm as ic  # noqa: E402  (chdir to scan3d)
from imu_gt import read_imu  # noqa: E402

A_HI = 39.0          # m/s^2 (4 g)
A_LO = 2.0           # m/s^2
FF_MIN_S = 0.1
GAP_NS = 20_000_000
OUT_JSON = Path("docs/i3_input_events.json")


def _intervals(rec, gyr_t, gyr):
    """start still and first set-down still (IMU ns), same rules as i3_confirm.inputs (amendment 2)."""
    R = Path("recordings") / rec
    t_frames = np.array([int(r["t_mid_ns"]) for r in csv.DictReader(open(R / "frames_cam5.csv"))])
    al = R / "imu_gt" / "align.json"
    dt = int(json.load(open(al))["dt_ns"]) if al.exists() else 0
    f0, fl = int(t_frames[0] - dt), int(t_frames[-1] - dt)
    runs = ic.still_runs(gyr_t, gyr)
    st = next(([a, b] for a, b in runs if a <= f0 + ic.START_SEARCH_S * 1e9 and b > f0), None)
    sd = []
    for g0, g1 in ic.SETDOWN_GUIDE:
        cand = [[a, b] for a, b in runs if (b - f0) / 1e9 > g0 and (a - f0) / 1e9 < g1 and [a, b] != st]
        sd.append(max(cand, key=lambda x: x[1] - x[0]) if cand else None)
    src = "recomputed (provisional, no inputs json)" + ("" if al.exists() else ", dt=0 (no align.json)")
    fi = ic.OUT / "inputs" / f"{rec}.json"
    if fi.exists():
        inp = json.load(open(fi))
        st_i, sd_i = inp["start_still_imu_ns"], [w["still_imu_ns"] for w in inp["setdown"]]
        if st_i != st or sd_i != sd or inp["first_frame_imu_ns"] != f0:
            raise RuntimeError(f"{rec}: recomputed stills differ from inputs json: {st}/{sd} vs {st_i}/{sd_i}")
        src = "inputs json (recomputed: identical)"
    return f0, fl, st, sd, src


def detect(rec):
    R = Path("recordings") / rec
    acc, gyr = read_imu(R / "imu.csv", raw=True)
    meta = json.load(open(R / "meta.json"))
    t0m = int(meta["start_elapsed_ns"])
    f0, fl, st, sd, src = _intervals(rec, gyr[:, 0].astype(np.int64), gyr[:, 1:4])
    w0 = st[1] if st else f0
    first_sd = next((x for x in sd if x is not None), None)
    w1 = first_sd[0] if first_sd else fl
    ta = acc[:, 0].astype(np.int64)
    an = np.linalg.norm(acc[:, 1:4], axis=1)
    m = (ta >= w0) & (ta <= w1)
    tw, aw = ta[m], an[m]

    def s_f(t):   # seconds after first frame
        return round(float(t - f0) / 1e9, 3)

    def s_m(t):   # seconds after meta.start_elapsed_ns (clock of docs/collapse_213955.md)
        return round(float(t - t0m) / 1e9, 3)

    hi = np.nonzero(aw > A_HI)[0]
    low = aw < A_LO
    ff = []
    k = 0
    while k < len(low):
        if not low[k]:
            k += 1
            continue
        k2 = k
        while k2 + 1 < len(low) and low[k2 + 1] and tw[k2 + 1] - tw[k2] <= GAP_NS:
            k2 += 1
        ff.append((tw[k], tw[k2], (tw[k2] - tw[k]) / 1e9))
        k = k2 + 1
    ff_long = [x for x in ff if x[2] > FF_MIN_S + 1e-12]
    longest = max(ff, key=lambda x: x[2]) if ff else None
    imax = int(np.argmax(aw)) if len(aw) else None
    event = bool(len(hi) or ff_long)
    times = sorted({s_m(tw[i]) for i in hi} | {s_m(a) for a, _, _ in ff_long})
    clusters = []                      # event times grouped when < 0.5 s apart (meta clock)
    for x in times:
        if clusters and x - clusters[-1][1] < 0.5:
            clusters[-1][1] = x
        else:
            clusters.append([x, x])
    out = dict(rec=rec, event=event, interval_source=src,
               window_s_first_frame=[s_f(w0), s_f(w1)], window_s_meta=[s_m(w0), s_m(w1)],
               window_imu_ns=[int(w0), int(w1)], window_end="first set-down still" if first_sd else "last frame",
               n_acc=int(m.sum()), max_abs_a=round(float(aw.max()), 2) if len(aw) else None,
               max_abs_a_t_meta=s_m(tw[imax]) if imax is not None else None,
               max_abs_a_t_first_frame=s_f(tw[imax]) if imax is not None else None,
               n_over_4g=int(len(hi)),
               over_4g_span_meta=[s_m(tw[hi[0]]), s_m(tw[hi[-1]])] if len(hi) else None,
               over_4g_span_first_frame=[s_f(tw[hi[0]]), s_f(tw[hi[-1]])] if len(hi) else None,
               longest_freefall_s=round(longest[2], 3) if longest else 0.0,
               longest_freefall_meta=[s_m(longest[0]), s_m(longest[1])] if longest else None,
               freefall_runs_meta=[[s_m(a), s_m(b), round(d, 3)] for a, b, d in ff],
               n_freefall_over_0_1s=len(ff_long),
               event_times_meta=times[:50], event_clusters_meta=clusters,
               first_frame_minus_meta_s=round((f0 - t0m) / 1e9, 3))
    reasons = []
    if len(hi):
        reasons.append(f"|a| > {A_HI} m/s^2 x{len(hi)}")
    if ff_long:
        reasons.append(f"|a| < {A_LO} m/s^2 for > {FF_MIN_S} s x{len(ff_long)}")
    out["reasons"] = reasons
    return out


def main(recs):
    res = dict(made=time.strftime("%F %T"), rule="docs/i3_confirm_plan.md amendment 3 (post-hoc)",
               thresholds=dict(a_hi=A_HI, a_lo=A_LO, ff_min_s=FF_MIN_S, gap_ns=GAP_NS),
               imu="raw type 35 (imu_gt.read_imu raw=True)", recs={})
    for r in recs:
        res["recs"][r] = detect(r)
        d = res["recs"][r]
        print(f"{r}: event {d['event']} {d['reasons']} clusters(meta) {d['event_clusters_meta']} window(meta) {d['window_s_meta']} max|a| {d['max_abs_a']} "
              f"@{d['max_abs_a_t_meta']} longest ff {d['longest_freefall_s']} s @{d['longest_freefall_meta']} "
              f"[{d['interval_source']}]", flush=True)
    return res


if __name__ == "__main__":
    recs = sys.argv[1:] or ic.RECS
    res = main(recs)
    if not sys.argv[1:]:
        OUT_JSON.write_text(json.dumps(res, indent=1, ensure_ascii=False), encoding="utf-8")
        print("wrote", OUT_JSON)
