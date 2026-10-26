#!/usr/bin/env python3
"""ovbaro 2026-10-26: synthetic but realistic barometer stream for an HS GT recording.

Height truth = IMU position of the board GT (imu_gt_spl_cal1022: camera pose in the board frame + lever arm
config/imu_cam_lever.json) projected on "up" (= direction of the mean specific force over the whole recording in the board
frame: the mean acceleration of a bounded hand motion is ~0, so mean(f) = -g; checked |mean f| ~ 9.8).
Pressure = 1013 hPa * exp(-h / Hs), Hs at 25 C (the bridge assumes 20 C -> 1.7 % height-scale error on purpose).
Noise = REAL static S24 barometer series (baro_static_resid.npz from baro_noise.py: still segments of the old rec_ recordings,
Pa, incl. the slow indoor fluctuation), segments concatenated end-to-end (continuous) in a seeded random order.
Sample times = 80 ms grid (LPS22DF 12.5 Hz) + real jitter (+-0.2 ms), IMU clock (= GT camera time - align dt_ns).
Scenarios: s0 = the above; s1 = s0 + permanent +1.5 hPa step 25 s after the first IMU sample (door / HVAC);
s2 = s0 + 6 prop-wash bursts (+-0.03..0.3 hPa, 0.3..1.5 s, 2-sample edges) at seeded random times.

    gen(rec, scen, out_csv) -> dict(info);  python3 gen_baro.py REC SCEN OUT.csv
"""
import json
import sys
import zlib
from pathlib import Path

import numpy as np

MAIN = Path("/home/system/scanner/StereoDepthSeg/scan3d")
HERE = Path(__file__).resolve().parent
GT = "imu_gt_spl_cal1022"
HS_GEN = 287.05 * (25.0 + 273.15) / 9.80665


def read_acc(rec):
    t, a = [], []
    for line in open(rec / "imu.csv"):
        s = line.split(",")
        if s[0] == "1":
            t.append(int(s[1]))
            a.append((float(s[2]), float(s[3]), float(s[4])))
    return np.array(t, np.int64), np.array(a)


def truth(recname):
    rec = MAIN / "recordings" / recname
    g = np.load(rec / GT / "gt.npz")
    al = json.load(open(rec / "imu_gt" / "align.json"))
    dt_ns, R_ci = int(al["dt_ns"]), np.asarray(al["R_ci"])
    r = np.asarray(json.load(open(MAIN / "config" / "imu_cam_lever.json"))["r_cm"], float) / 100.0
    ok = g["ok"].astype(bool)
    t_cam, R_bc, p_bc = g["t_ns"][ok], g["R_bc"][ok], g["p_bc"][ok]
    p_i = p_bc + np.einsum("nij,j->ni", R_bc, r)
    t_imu = t_cam - dt_ns
    ta, a = read_acc(rec)
    ai = np.stack([np.interp(t_imu, ta, a[:, k]) for k in range(3)], 1)
    best = None
    for name, R in (("R_ci", R_ci), ("R_ci^T", R_ci.T)):
        f = np.einsum("nij,nj->ni", R_bc, ai @ R.T)
        m = f.mean(0)
        if best is None or np.linalg.norm(m) > best[2]:
            best = (name, m / np.linalg.norm(m), float(np.linalg.norm(m)))
    up = best[1]
    h = p_i @ up
    return dict(t_imu=t_imu, h=h, up=up, up_rot=best[0], up_norm=best[2], dt_ns=dt_ns, t_imu0=int(ta[0]), t_imu1=int(ta[-1]))


def noise_series(n, seed):
    z = np.load(HERE / "baro_static_resid.npz")
    segs = [z[k][:, 1] for k in sorted(z.files)]
    rng = np.random.default_rng(seed)
    out, last = [], 0.0
    while sum(len(x) for x in out) < n:
        s = segs[rng.integers(len(segs))]
        s = s - s[0] + last
        out.append(s)
        last = s[-1]
    x = np.concatenate(out)[:n]
    return (x - x[0]) / 100.0  # Pa -> hPa


def gen(recname, scen, out_csv):
    T = truth(recname)
    seed = zlib.crc32(recname.encode()) & 0xFFFFFFF
    rng = np.random.default_rng(seed + 7)
    t = np.arange(T["t_imu0"], T["t_imu1"], 80_000_000, dtype=np.int64)
    t = t + rng.integers(-200_000, 200_000, len(t))
    h = np.interp(t, T["t_imu"], T["h"])
    h = h - h[0]
    p = 1013.0 * np.exp(-h / HS_GEN) + noise_series(len(t), seed)
    ts = (t - t[0]) / 1e9
    ev = []
    if scen == "s1":
        p = p + np.where(ts >= 25.0, 1.5, 0.0)
        ev.append(dict(kind="step", t_s=25.0, hpa=1.5))
    elif scen == "s2":
        r2 = np.random.default_rng(seed + 11)
        for _ in range(6):
            t0 = r2.uniform(10.0, ts[-1] - 5.0)
            dur = r2.uniform(0.3, 1.5)
            amp = r2.uniform(0.03, 0.3) * (1 if r2.random() < 0.5 else -1)
            edge = 0.16
            w = np.clip(np.minimum((ts - t0) / edge, (t0 + dur - ts) / edge), 0.0, 1.0)
            p = p + amp * w
            ev.append(dict(kind="burst", t_s=round(t0, 3), dur_s=round(dur, 3), hpa=round(amp, 4)))
    elif scen != "s0":
        raise ValueError(scen)
    Path(out_csv).parent.mkdir(parents=True, exist_ok=True)
    with open(out_csv, "w") as f:
        for a, b in zip(t, p):
            f.write(f"{int(a)},{b:.5f}\n")
    info = dict(rec=recname, scen=scen, n=len(t), t0_ns=int(t[0]), up=T["up"].tolist(), up_rot=T["up_rot"],
                up_norm=round(T["up_norm"], 4), h_range_m=round(float(np.ptp(h)), 3), events=ev, seed=seed,
                hs_gen_m=HS_GEN, noise="real static S24 series (baro_static_resid.npz)")
    json.dump(info, open(str(out_csv) + ".json", "w"), indent=1)
    return info


if __name__ == "__main__":
    print(json.dumps(gen(sys.argv[1], sys.argv[2], sys.argv[3]), indent=1))
