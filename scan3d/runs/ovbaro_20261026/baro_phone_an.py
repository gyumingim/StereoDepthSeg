#!/usr/bin/env python3
"""ovbaro 2026-10-26: analyse the phone barometer tests (README "폰 단계").

  static FILE.csv          ImuLog CSV (type,t_ns,v0..; type 6 = hPa): rate, white noise (diff std / sqrt 2), linear drift,
                           detrended std, Allan deviation (cm) -> FILE.static.json
  hand   FILE.csv          ImuLog CSV of the 1 m raise/lower test: height trace (20 C hypsometric), the level of each
                           still plateau (gyro < 0.05 rad/s for >= 2 s) -> plateau heights + step sizes -> FILE.hand.json
  live   RUN_DIR           arlat run with intent baro 1: baro_log.csv (bridge) -> baro height vs VIO z_state: scale (slope),
                           lag (cross-correlation of the derivatives, +-1 s), residual std, status counts -> RUN_DIR/baro_an.json
"""
import json
import sys
import time
from pathlib import Path

import numpy as np

K = 287.05 * 293.15 / 9.80665


def load_imulog(f):
    P, G = [], []
    for line in open(f):
        s = line.split(",")
        if s[0] == "6":
            P.append((int(s[1]), float(s[2])))
        elif s[0] == "4":
            G.append((int(s[1]), float(s[2]), float(s[3]), float(s[4])))
    return np.array(P), np.array(G)


def adev(x, dt, taus):
    th = np.cumsum(np.concatenate([[0.0], x])) * dt
    out = []
    for tau in taus:
        m = int(round(tau / dt))
        if m < 1 or 2 * m >= len(x):
            continue
        a = (th[2 * m:] - 2 * th[m:-m] + th[:-2 * m]) / (m * dt)
        out.append((round(m * dt, 2), round(float(np.sqrt(0.5 * np.mean(a ** 2))) * 100, 3)))
    return out


def static(f):
    P, _ = load_imulog(f)
    t = (P[:, 0] - P[0, 0]) / 1e9
    h = K * np.log(P[0, 1] / P[:, 1])
    dt = float(np.median(np.diff(t)))
    c = np.polyfit(t, h, 1)
    r = h - np.polyval(c, t)
    res = dict(made=time.strftime("%F %T"), file=str(f), n=len(P), dur_s=round(float(t[-1]), 1), dt_med_s=round(dt, 4),
               white_cm=round(float(np.std(np.diff(h)) / np.sqrt(2)) * 100, 3), drift_cm_per_min=round(float(c[0]) * 6000, 3),
               detrended_std_cm=round(float(r.std()) * 100, 3), ptp_cm=round(float(np.ptp(h)) * 100, 2),
               allan_cm=adev(h, dt, [0.08, 0.16, 0.32, 0.64, 1.28, 2.56, 5.12, 10.24, 20.48, 40.96, 81.92]))
    json.dump(res, open(str(f) + ".static.json", "w"), indent=1)
    print(json.dumps(res, indent=1))


def hand(f):
    P, G = load_imulog(f)
    t = (P[:, 0] - P[0, 0]) / 1e9
    h = K * np.log(P[0, 1] / P[:, 1])
    gt = (G[:, 0] - P[0, 0]) / 1e9
    gn = np.linalg.norm(G[:, 1:], axis=1)
    win = lambda x: gn[(gt > x - 1) & (gt < x + 1)]
    still = np.array([len(win(x)) > 10 and win(x).max() < 0.05 for x in t])
    plats, i = [], 0
    while i < len(t):
        if still[i]:
            j = i
            while j + 1 < len(t) and still[j + 1]:
                j += 1
            if t[j] - t[i] >= 2.0:
                plats.append(dict(t0=round(float(t[i]), 1), t1=round(float(t[j]), 1), h_cm=round(float(np.median(h[i:j + 1])) * 100, 1),
                                  std_cm=round(float(np.std(h[i:j + 1])) * 100, 2)))
            i = j + 1
        else:
            i += 1
    steps = [round(b["h_cm"] - a["h_cm"], 1) for a, b in zip(plats, plats[1:])]
    res = dict(made=time.strftime("%F %T"), file=str(f), plateaus=plats, steps_cm=steps,
               note="expected steps = the tape heights of the test (e.g. +100 / -100 cm); scale = step / tape")
    json.dump(res, open(str(f) + ".hand.json", "w"), indent=1)
    print(json.dumps(res, indent=1))


def live(d):
    import csv
    d = Path(d)
    rows = list(csv.DictReader(open(d / "baro_log.csv")))
    st = {}
    for r in rows:
        st[r["status"]] = st.get(r["status"], 0) + 1
    ok = [r for r in rows if r["z_state"] not in ("nan", "") and r["h"] not in ("nan", "")]
    res = dict(made=time.strftime("%F %T"), run=str(d), n=len(rows), status=st)
    if len(ok) > 50:
        t = np.array([float(r["t"]) for r in ok])
        h = np.array([float(r["h"]) for r in ok])
        z = np.array([float(r["z_state"]) for r in ok])
        A = np.stack([z - z.mean(), np.ones_like(z)], 1)
        slope = float(np.linalg.lstsq(A, h - h.mean(), rcond=None)[0][0])
        tg = np.arange(t[0], t[-1], 0.02)
        hz, zz = np.gradient(np.interp(tg, t, h), 0.02), np.gradient(np.interp(tg, t, z), 0.02)
        lags = np.arange(-50, 51)
        cc = [np.corrcoef(hz[max(0, k):len(hz) + min(0, k)], zz[max(0, -k):len(zz) - max(0, k)])[0, 1] for k in lags]
        res.update(scale_baro_per_vio=round(slope, 4), lag_ms_baro_after_vio=int(lags[int(np.nanargmax(cc))] * 20),
                   corr_max=round(float(np.nanmax(cc)), 3), z_range_m=round(float(np.ptp(z)), 3),
                   resid_std_cm=round(float(np.std((h - h.mean()) - slope * (z - z.mean()))) * 100, 2))
    json.dump(res, open(d / "baro_an.json", "w"), indent=1)
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    {"static": static, "hand": hand, "live": live}[sys.argv[1]](sys.argv[2])
