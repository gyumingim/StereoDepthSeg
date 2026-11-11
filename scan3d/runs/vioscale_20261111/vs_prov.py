#!/usr/bin/env python3
"""vioscale: data provenance of one recording (IMU types/rates/units, still-phase |a| and gyro bias, timestamps, meta).

  python vs_prov.py REC OUT.json
Still phase = from the first IMU sample until the motion onset (first 0.25 s window whose gyro rms > 0.05 rad/s or acc std > 0.15 m/s^2,
after a 0.5 s guard). |a| in the still phase, per IMU type (35 raw, 1 calibrated if present) -> accelerometer scale check vs local g.
"""
import csv, json, sys, time
from pathlib import Path
import numpy as np

G_LOCAL = 9.7973        # Busan estimate used in scalehunt GR_9797 (가정: latitude formula + ~0 m altitude)


def main():
    rec, out = Path(sys.argv[1]), Path(sys.argv[2])
    meta = json.load(open(rec / "meta.json"))
    rows = {}
    bad = 0
    for r in csv.reader(open(rec / "imu.csv")):
        if not r or r[0] == "type":
            continue
        try:
            rows.setdefault(int(r[0]), []).append([float(x) for x in r[1:]])
        except ValueError:
            bad += 1
    T = {k: np.array([v[:4] + ([0.0] * 3 if len(v) < 7 else v[4:7]) for v in vs if len(v) >= 4]) for k, vs in rows.items()}
    res = dict(rec=rec.name, made=time.strftime("%F %T"), bad_rows=bad, types={})
    for k, a in T.items():
        t = a[:, 0].astype(np.int64)
        dt = np.diff(t) / 1e6
        res["types"][k] = dict(n=len(a), rate_hz=round(len(a) / ((t[-1] - t[0]) / 1e9), 2), dt_ms_med=round(float(np.median(dt)), 4),
                               dt_ms_max=round(float(dt.max()), 3), n_gap_gt_5ms=int((dt > 5).sum()), nonmono=int((dt <= 0).sum()),
                               extra_cols_mean=[round(float(v), 6) for v in a[:, 4:7].mean(0)],
                               t_first=int(t[0]), t_last=int(t[-1]))
    fr = list(csv.DictReader(open(rec / "frames_cam5.csv")))
    tf = np.array([int(r["t_mid_ns"]) for r in fr])
    ph = np.array([int(r["physical_ts_ns"]) for r in fr])
    ex = np.array([int(r["exposure_ns"]) for r in fr]); sk = np.array([int(r["rolling_shutter_skew_ns"]) for r in fr])
    res["frames"] = dict(n=len(fr), fps=round(len(fr) / ((tf[-1] - tf[0]) / 1e9), 3), dt_ms_max=round(float(np.diff(tf).max() / 1e6), 3),
                         t_mid_minus_phys_ms=sorted(set(np.round((tf - ph) / 1e6, 4).tolist()))[:5],
                         exposure_ms=sorted(set((ex / 1e6).tolist()))[:5], rs_skew_ms=sorted(set((sk / 1e6).tolist()))[:5],
                         t0_minus_imu0_s=round((tf[0] - T[min(T)][0, 0]) / 1e9, 3))
    res["meta"] = {k: meta.get(k) for k in ("timestamp_ref", "clock_offset_ns", "frames_unmatched", "start_elapsed_ns", "imu_samples",
                                            "app_version", "imu_thread", "frames", "fps", "save_fps")}
    # still phase from the raw gyro (16) / raw acc (35)
    g = T.get(16, T.get(4)); a = T.get(35, T.get(1))
    tg = g[:, 0]
    # 11-12 fix: the IMU starts ~3 s before frame 0 (AE phase, phone handled) -> search from frame 0 - 1 s; onset = gyro only
    # (a tap on the desk gives an acc burst without rotation, 202906 at +2.0..3.0 s, and must not count as the motion onset)
    t0 = max(tg[0], tf[0] - 0.8e9)
    onset = None
    w = 0.25e9
    tt = t0
    while tt + w < tg[-1]:
        mg = (tg >= tt) & (tg < tt + w)
        ma = (a[:, 0] >= tt) & (a[:, 0] < tt + w)
        grms = float(np.sqrt((g[mg, 1:4] ** 2).sum(1).mean())) if mg.any() else 0
        astd = float(np.linalg.norm(a[ma, 1:4].std(0))) if ma.sum() > 5 else 0
        if grms > 0.05:
            onset = tt; break
        tt += w
    res["onset_fix"] = "11-12 gyro-only from frame0-0.8s"
    res["onset_s_from_imu0"] = round((onset - t0) / 1e9, 3) if onset else None
    res["onset_s_from_frame0"] = round((onset - tf[0]) / 1e9, 3) if onset else None
    still_hi = (onset - 0.25e9) if onset else t0 + 3e9
    st = {}
    for k in (35, 1, 16, 4):
        if k not in T:
            continue
        m = (T[k][:, 0] >= t0) & (T[k][:, 0] < still_hi)
        v = T[k][m, 1:4]
        if not len(v):
            continue
        d = dict(n=int(m.sum()), dur_s=round((still_hi - t0) / 1e9, 2), mean=[round(float(x), 5) for x in v.mean(0)],
                 std=[round(float(x), 5) for x in v.std(0)])
        if k in (35, 1):
            nrm = np.linalg.norm(v, axis=1)
            d.update(norm_mean=round(float(nrm.mean()), 5), norm_of_mean=round(float(np.linalg.norm(v.mean(0))), 5),
                     ratio_to_g_local=round(float(np.linalg.norm(v.mean(0)) / G_LOCAL), 5), g_local=G_LOCAL)
        st[k] = d
    res["still"] = st
    # dynamic content after onset (for scale observability): linear-acc rms of raw acc minus its 1 s running mean
    if onset:
        m = a[:, 0] >= onset
        aa = a[m, 1:4]
        k = 250
        ker = np.ones(k) / k
        lo = np.stack([np.convolve(np.pad(aa[:, j], k // 2, mode="edge"), ker, mode="valid")[:len(aa)] for j in range(3)], 1)
        res["dyn_acc_hp_rms"] = round(float(np.sqrt(((aa - lo) ** 2).sum(1).mean())), 4)
        mg = g[:, 0] >= onset
        res["dyn_gyro_rms"] = round(float(np.sqrt((g[mg, 1:4] ** 2).sum(1).mean())), 4)
    json.dump(res, open(out, "w"), indent=1)
    print(json.dumps(res)[:3000])


if __name__ == "__main__":
    main()
