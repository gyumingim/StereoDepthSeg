#!/usr/bin/env python3
"""ovbaro 2026-10-26: real S24 barometer (LPS22DFTR, Android type 6) noise / drift from the old recordings that logged it
(rec_202610{16,18}_*, app 1.8-subsample, imu.csv type 6, hPa, SensorEvent.timestamp = same clock as the IMU).

Static segments = gyro |w| < 0.03 rad/s and |a| within 0.15 m/s^2 of its median for >= MIN_S s (phone lying still).
Per segment: linear trend (Pa/s), residual std, lag-1/lag-5 autocorrelation of the residual, first-difference std.
Allan deviation (overlapping) of height on the longest static segments.  Height = hypsometric 8.44 m/hPa (20 C).
Writes baro_noise.json (+ static residual series baro_static_resid.npz used by the synthetic test).

    python3 baro_noise.py
"""
import json
import time
from pathlib import Path

import numpy as np

REC = Path("/home/system/scanner/StereoDepthSeg/scan3d/recordings")
OUT = Path(__file__).resolve().parent
MIN_S = 20.0
M_PER_HPA = 287.05 * 293.15 / 9.80665 / 1013.0   # dh/dp at 20 C, 1013 hPa (= 8.47 m/hPa; 1017 hPa -> 8.44)


def load(rec):
    P, G, A = [], [], []
    for line in open(rec / "imu.csv"):
        s = line.split(",")
        if s[0] == "6":
            P.append((int(s[1]), float(s[2])))
        elif s[0] == "4":
            G.append((int(s[1]), float(s[2]), float(s[3]), float(s[4])))
        elif s[0] == "1":
            A.append((int(s[1]), float(s[2]), float(s[3]), float(s[4])))
    return np.array(P), np.array(G), np.array(A)


def static_mask(tp, G, A):
    """True for pressure samples whose surrounding 1 s of IMU is still."""
    gn = np.linalg.norm(G[:, 1:], axis=1)
    an = np.linalg.norm(A[:, 1:], axis=1)
    amed = np.median(an)
    m = np.zeros(len(tp), bool)
    for i, t in enumerate(tp):
        g = gn[(G[:, 0] > t - 5e8) & (G[:, 0] < t + 5e8)]
        a = an[(A[:, 0] > t - 5e8) & (A[:, 0] < t + 5e8)]
        m[i] = len(g) > 10 and len(a) > 10 and g.max() < 0.03 and np.abs(a - amed).max() < 0.15
    return m


def runs(m):
    out, i = [], 0
    while i < len(m):
        if m[i]:
            j = i
            while j + 1 < len(m) and m[j + 1]:
                j += 1
            out.append((i, j + 1))
            i = j + 1
        else:
            i += 1
    return out


def adev(x, dt, taus):
    """overlapping Allan deviation of a rate-free series x (height, m) sampled every dt."""
    res = []
    th = np.cumsum(np.concatenate([[0.0], x])) * dt  # integral -> Allan of the averaged value
    N = len(x)
    for tau in taus:
        m = int(round(tau / dt))
        if m < 1 or 2 * m >= N:
            continue
        a = (th[2 * m:] - 2 * th[m:-m] + th[:-2 * m]) / (m * dt)
        res.append((m * dt, float(np.sqrt(0.5 * np.mean(a ** 2)))))
    return res


def main():
    segs, resid_store = [], {}
    for rec in sorted(REC.glob("rec_*")):
        if not (rec / "imu.csv").exists():
            continue
        P, G, A = load(rec)
        if len(P) < 50 or len(G) < 100:
            continue
        tp = P[:, 0]
        m = static_mask(tp, G, A)
        for i0, i1 in runs(m):
            t = (tp[i0:i1] - tp[i0]) / 1e9
            if t[-1] < MIN_S:
                continue
            p = P[i0:i1, 1] * 100.0  # Pa
            c = np.polyfit(t, p, 1)
            r = p - np.polyval(c, t)
            d = np.diff(p)
            ac = lambda k: float(np.corrcoef(r[:-k], r[k:])[0, 1])
            segs.append(dict(rec=rec.name, t0_s=round((tp[i0] - tp[0]) / 1e9, 1), dur_s=round(float(t[-1]), 1), n=int(i1 - i0),
                             dt_med_s=round(float(np.median(np.diff(t))), 4), p_mean_hpa=round(float(p.mean() / 100), 3),
                             slope_pa_per_min=round(float(c[0] * 60), 3), resid_std_pa=round(float(r.std()), 4),
                             resid_std_cm=round(float(r.std() / 100 * M_PER_HPA * 100), 2),
                             diff_std_pa=round(float(d.std()), 4), ac1=round(ac(1), 3), ac5=round(ac(5), 3),
                             ptp_pa=round(float(np.ptp(p)), 3)))
            resid_store[f"{rec.name}_{i0}"] = np.stack([t, p - p[0]], 1)
    segs.sort(key=lambda s: -s["dur_s"])
    taus = [0.08, 0.16, 0.32, 0.64, 1.28, 2.56, 5.12, 10.24, 20.48, 40.96, 81.92, 163.84, 327.68]
    allan = {}
    for s in segs[:4]:
        k = next(k for k in resid_store if k.startswith(s["rec"] + "_") and len(resid_store[k]) == s["n"])
        x = resid_store[k][:, 1] / 100 * M_PER_HPA  # m
        allan[k] = [(round(a, 2), round(b * 100, 3)) for a, b in adev(x, s["dt_med_s"], taus)]
    st = np.array([s["resid_std_cm"] for s in segs])
    summ = dict(n_segments=len(segs), total_static_s=round(sum(s["dur_s"] for s in segs), 1),
                resid_std_cm_med=round(float(np.median(st)), 2) if len(st) else None,
                resid_std_cm_p90=round(float(np.percentile(st, 90)), 2) if len(st) else None,
                diff_std_pa_med=round(float(np.median([s["diff_std_pa"] for s in segs])), 4) if segs else None,
                ac1_med=round(float(np.median([s["ac1"] for s in segs])), 3) if segs else None,
                slope_pa_per_min_abs_med=round(float(np.median([abs(s["slope_pa_per_min"]) for s in segs])), 3) if segs else None,
                m_per_hpa=M_PER_HPA)
    res = dict(made=time.strftime("%F %T"), script=str(Path(__file__)), source=str(REC) + "/rec_*/imu.csv type 6",
               static_rule=f"gyro<0.03 rad/s, |a|-median<0.15 m/s2 within +-0.5 s, >= {MIN_S} s",
               summary=summ, allan_dev_cm=allan, segments=segs)
    json.dump(res, open(OUT / "baro_noise.json", "w"), indent=1)
    np.savez_compressed(OUT / "baro_static_resid.npz", **{k: v for k, v in resid_store.items()})
    print(json.dumps(summ, indent=1))
    for s in segs[:12]:
        print(s)
    for k, v in allan.items():
        print(k, v)


if __name__ == "__main__":
    main()
