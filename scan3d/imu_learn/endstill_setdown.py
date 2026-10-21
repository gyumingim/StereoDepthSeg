#!/usr/bin/env python3
"""Set-down failure diagnosis (2026-10-21, docs/init_window_plan.md experiment 2). No new VIO runs: uses the full-length
raw-IMU runs in runs/endstill_vio/<rec6>_loose_raw_o{0,1,2} (OpenVINS DEBUG log + pose stream) and the recording itself.

Per recording (224323, 223815), time = s since the first IMU sample of the recording (same as docs/endstill_check.html):
  vio   per run, per 0.5 s bin: GT error after one SE3 fit over the run (first-GT-match base as errtime.py), 1 s relative
        position error vs GT (RPE, GT-free of global drift), |v| VIO, features used in MSCKF / SLAM updates, ZUPT
        disparity and tracked-feature count (DEBUG log lines per [DIAGT] frame), bias bg / ba.
  imu   raw types 16/35: per 0.1 s max |a| - g, max |w|, overall max vs sensor range (sensors.json of this phone:
        accel 156.9 m/s^2 = 16 g, gyro 34.9 rad/s = 2000 dps) -> saturation / impact spikes.
  video cam5.mp4 every 0.25 s from T0 to the end: sharpness (variance of Laplacian, 640x360 gray), mean brightness,
        Shi-Tomasi corner count (500 max, q 0.01, 8 px), scene change (share of pixels whose gray level moved > 25 vs
        the sample 0.25 s earlier; with the phone still this is motion IN the scene), and board PnP coverage per 0.5 s
        (imu_gt/gt.npz ok, n_corners).
        Contact sheet PNG of frames every 1 s.
  -> runs/setdown/<rec6>.json, <rec6>_timeline.png, <rec6>_frames.jpg

  ../venv_ffs/bin/python imu_learn/endstill_setdown.py [--t0 95]
"""
import argparse
import csv
import json
import re
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import c5_overnight_analyse as an  # noqa: E402
import collapse_signals as cs  # noqa: E402
from imu_gt import read_imu  # noqa: E402

SCAN3D = HERE.parent
OUT = SCAN3D / "runs" / "setdown"
RECS = {"224323": "hs_20261020_224323", "223815": "hs_20261020_223815"}
ACC_RANGE, GYR_RANGE, G = 156.9064, 34.906, 9.80665
ANSI = re.compile(r"\x1b\[[0-9;]*m")


def parse_frames(fn):
    """[DIAGT] t -> zupt disparity / tracked feats / pass, msckf feats, slam feats, slam delayed init feats, |v| (zupt line)."""
    import gzip
    out, cur = [], None
    for line in gzip.open(fn, "rt", errors="replace"):
        line = ANSI.sub("", line)
        if "[DIAGT]" in line:
            cur = dict(t=float(line.split("[DIAGT]")[1]))
            out.append(cur)
        elif cur is None:
            continue
        elif "[ZUPT]: passed disparity" in line or "[ZUPT]: failed disparity" in line:
            m = re.search(r"\(([-0-9.]+) [<>] [0-9.]+, (\d+) features\)", line)
            cur.update(zdisp=float(m[1]), zfeat=int(m[2]), zpass="passed" in line)
        elif "[ZUPT]: accepted" in line:
            cur["zacc"] = True
        elif "seconds for MSCKF update" in line:
            cur["msckf"] = int(re.search(r"\((\d+) feats\)", line)[1])
        elif "seconds for SLAM update" in line:
            cur["slam"] = int(re.search(r"\((\d+) feats\)", line)[1])
        elif "seconds for SLAM delayed init" in line:
            cur["slam_init"] = int(re.search(r"\((\d+) feats\)", line)[1])
    return out


def binned(t, v, edges, fn=np.nanmean):
    r = []
    for a, b in zip(edges[:-1], edges[1:]):
        m = (t >= a) & (t < b) & np.isfinite(v)
        r.append(float(fn(v[m])) if m.any() else None)
    return r


def vio_run(prefix, t_imu0, edges):
    d = json.load(open(prefix + ".json"))
    run = cs.load(prefix)
    rec = d["rec"]
    gt_t, gt_p, _ = an.gt(rec, d.get("gt") or "imu_gt_spl")
    trk = (run["status"] == "tracking") & np.isfinite(run["T"][:, 0])
    vt = run["t_ns"] + int(d["align_dt_ms"] * 1e6)          # VIO frame time on the GT (camera) clock, as errtime.py
    j, mm = an.match(vt[trk], gt_t)
    P = run["T"][trk].reshape(-1, 4, 4)[:, :3, 3][mm]
    Pg = gt_p[j[mm]]
    tt = run["t_ns"][trk][mm]
    R, _, tr = an.vb.umeyama(P, Pg)
    Pa = P @ R.T + tr
    e = np.linalg.norm(Pa - Pg, axis=1) * 100
    ts = (tt - t_imu0) / 1e9
    # 1 s relative error (aligned rotation): |(Pa(t+1)-Pa(t)) - (Pg(t+1)-Pg(t))|
    k = np.searchsorted(tt, tt + int(1e9))
    ok = (k < len(tt))
    k = np.minimum(k, len(tt) - 1)
    ok &= np.abs(tt[k] - tt - 1e9) < 0.1e9
    rpe = np.where(ok, np.linalg.norm((Pa[k] - Pa) - (Pg[k] - Pg), axis=1) * 100, np.nan)
    ta = (run["t_ns"] - t_imu0) / 1e9
    vn = np.linalg.norm(run["v"], axis=1)
    fr = parse_frames(prefix + ".stderr.log.gz")
    # bridge time -> recording time via the pose lines (t = bridge s, t_ns = IMU clock ns)
    off = float(np.nanmedian(run["t_br"] - (run["t_ns"] - t_imu0) / 1e9))
    ft = np.array([f["t"] for f in fr]) - off

    def fcol(k, dflt=np.nan):
        return np.array([f.get(k, dflt) for f in fr], float)
    res = dict(name=Path(prefix).name, ate_cm=d.get("ate_cm"), gt_first_s=float(ts[0]), gt_last_s=float(ts[-1]),
               err_cm=binned(ts, e, edges), rpe1s_cm=binned(ts, rpe, edges), v_ms=binned(ta, vn, edges),
               v_max_ms=binned(ta, vn, edges, np.nanmax),
               msckf=binned(ft, fcol("msckf"), edges), slam=binned(ft, fcol("slam"), edges),
               zdisp=binned(ft, fcol("zdisp"), edges), zfeat=binned(ft, fcol("zfeat"), edges),
               zupt_acc_frac=binned(ft, fcol("zacc", 0.0), edges),
               bg_mdps=[binned(ta, run["bg"][:, i] * 180 / np.pi * 1000, edges) for i in range(3)],
               ba_mg=[binned(ta, run["ba"][:, i] / G * 1000, edges) for i in range(3)],
               n_slam=binned(ta, run["n_slam"], edges), n_msckf=binned(ta, run["n_msckf"], edges),
               path_cm=binned(ta[1:], np.linalg.norm(np.diff(run["p"], axis=0), axis=1) * 100, edges, np.nansum))
    # first time after 100 s where the 1 s relative error exceeds 3 / 5 cm (descriptive only)
    for thr in (3, 5):
        w = np.nonzero((ts > 100) & (rpe > thr))[0]
        res[f"rpe_over{thr}_first_s"] = float(ts[w[0]]) if len(w) else None
    return res


def imu_part(rec, t_imu0, edges):
    acc, gyr = read_imu(SCAN3D / "recordings" / rec / "imu.csv", raw=True)
    ta, tg = (acc[:, 0] - t_imu0) / 1e9, (gyr[:, 0] - t_imu0) / 1e9
    an_ = np.linalg.norm(acc[:, 1:4], axis=1)
    gn = np.linalg.norm(gyr[:, 1:4], axis=1)
    per_axis_max_acc = np.abs(acc[:, 1:4]).max(0)
    per_axis_max_gyr = np.abs(gyr[:, 1:4]).max(0)
    return dict(acc_dev_max=binned(ta, np.abs(an_ - G), edges, np.nanmax), gyr_max=binned(tg, gn, edges, np.nanmax),
                acc_axis_abs_max=per_axis_max_acc.tolist(), gyr_axis_abs_max=per_axis_max_gyr.tolist(),
                acc_range=ACC_RANGE, gyr_range=GYR_RANGE,
                acc_frac_of_range=float(per_axis_max_acc.max() / ACC_RANGE),
                gyr_frac_of_range=float(per_axis_max_gyr.max() / GYR_RANGE),
                n_acc_over_95pct=int((np.abs(acc[:, 1:4]) > 0.95 * ACC_RANGE).sum()),
                n_gyr_over_95pct=int((np.abs(gyr[:, 1:4]) > 0.95 * GYR_RANGE).sum()),
                t_end_s=float(tg[-1]))


def video_part(rec, t_imu0, t0, edges, sheet_path):
    rd = SCAN3D / "recordings" / rec
    rows = list(csv.DictReader(open(rd / "frames_cam5.csv")))
    al = json.load(open(rd / "imu_gt" / "align.json"))
    tf = (np.array([int(r["t_mid_ns"]) for r in rows]) - int(al["dt_ns"]) - t_imu0) / 1e9   # IMU clock s
    want = np.arange(t0, tf[-1], 0.25)
    idx = sorted({int(np.argmin(np.abs(tf - w))) for w in want})
    cap = cv2.VideoCapture(str(rd / "cam5.mp4"))
    pos = 0          # decode from frame 0: CAP_PROP_POS_FRAMES seeking landed ~5 s off on these 240 fps files (first try)
    out, thumbs = [], []
    last_sheet = -9
    prev = None
    for i in idx:
        while pos < i:
            cap.grab()
            pos += 1
        okf, fr = cap.read()
        pos += 1
        if not okf:
            break
        g = cv2.cvtColor(cv2.resize(fr, (640, 360), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2GRAY)
        c = cv2.goodFeaturesToTrack(g, 500, 0.01, 8)
        # scene change vs the previous sample (0.25 s earlier): share of pixels whose gray level moved > 25
        chg = None if prev is None else float((cv2.absdiff(g, prev) > 25).mean())
        prev = g
        out.append(dict(t=float(tf[i]), sharp=float(cv2.Laplacian(g, cv2.CV_64F).var()), bright=float(g.mean()),
                        corners=int(0 if c is None else len(c)), change=chg))
        if tf[i] - last_sheet >= 1.0 - 1e-6:
            last_sheet = tf[i]
            th = cv2.resize(fr, (320, 180), interpolation=cv2.INTER_AREA)
            cv2.putText(th, f"{tf[i]:.1f}s", (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            thumbs.append(th)
    if thumbs:
        cols = 6
        while len(thumbs) % cols:
            thumbs.append(np.zeros_like(thumbs[0]))
        sheet = np.vstack([np.hstack(thumbs[r * cols:(r + 1) * cols]) for r in range(len(thumbs) // cols)])
        cv2.imwrite(str(sheet_path), sheet, [cv2.IMWRITE_JPEG_QUALITY, 80])
    t = np.array([o["t"] for o in out])
    g = np.load(rd / "imu_gt" / "gt.npz")
    tg = (g["t_ns"] - int(al["dt_ns"]) - t_imu0) / 1e9
    okb = g["ok"].astype(bool)
    return dict(samples=out, sharp=binned(t, np.array([o["sharp"] for o in out]), edges),
                bright=binned(t, np.array([o["bright"] for o in out]), edges),
                corners=binned(t, np.array([o["corners"] for o in out], float), edges),
                change=binned(t, np.array([np.nan if o["change"] is None else o["change"] for o in out]), edges, np.nanmax),
                board_ok_frac=binned(tg, okb.astype(float), edges),
                board_corners=binned(tg, np.where(okb, g["n_corners"], 0).astype(float), edges),
                board_last_ok_s=float(tg[okb][-1]))


def plot(rec6, res, edges, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    tc = (edges[:-1] + edges[1:]) / 2
    f, ax = plt.subplots(6, 1, figsize=(11, 13), sharex=True)
    for r in res["vio"]:
        ax[0].plot(tc, [np.nan if v is None else v for v in r["err_cm"]], label=r["name"])
        ax[1].plot(tc, [np.nan if v is None else v for v in r["rpe1s_cm"]])
        ax[2].plot(tc, [np.nan if v is None else v for v in r["v_ms"]])
        ax[3].plot(tc, [np.nan if v is None else v for v in r["msckf"]], "-")
        ax[3].plot(tc, [np.nan if v is None else v for v in r["slam"]], "--")
    ax[0].set_ylabel("GT err cm"); ax[0].legend(fontsize=7)
    ax[1].set_ylabel("1 s rel err cm"); ax[2].set_ylabel("|v| VIO m/s")
    ax[3].set_ylabel("feats in update\n(- msckf, -- slam)")
    v = res["video"]
    ax[4].plot(tc, [np.nan if x is None else x for x in v["corners"]], label="corners (GFTT)")
    ax[4].plot(tc, [np.nan if x is None else x / 10 for x in v["sharp"]], label="sharpness/10")
    ax[4].plot(tc, [np.nan if x is None else 100 * x for x in v["board_ok_frac"]], label="board seen %")
    ax[4].plot(tc, [np.nan if x is None else 1000 * x for x in v["change"]], label="scene change (px share x1000)")
    ax[4].legend(fontsize=7)
    i = res["imu"]
    ax[5].plot(tc, [np.nan if x is None else x for x in i["acc_dev_max"]], label="max ||a|-g| m/s2")
    ax[5].plot(tc, [np.nan if x is None else x for x in i["gyr_max"]], label="max |w| rad/s")
    ax[5].legend(fontsize=7); ax[5].set_xlabel("s since recording IMU start")
    f.suptitle(f"{rec6} set-down diagnosis")
    f.tight_layout()
    f.savefig(path, dpi=80)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--t0", type=float, default=95.0)
    o = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    for rec6, rec in RECS.items():
        acc, gyr = read_imu(SCAN3D / "recordings" / rec / "imu.csv", raw=True)
        t_imu0 = int(min(acc[0, 0], gyr[0, 0]))
        ec = json.load(open(SCAN3D / "runs" / "endstill_tmp" / "endstill_check.json"))["recs"][rec]
        t_imu0_check = int(ec["t0_ns"])
        t_end = (gyr[-1, 0] - t_imu0) / 1e9
        edges = np.arange(o.t0, t_end + 0.5, 0.5)
        res = dict(rec=rec, t0_ns=t_imu0, t0_ns_endstill_check=t_imu0_check, end_still_used=ec["used"].get("end"),
                   edges=edges.tolist(),
                   vio=[vio_run(str(SCAN3D / "runs" / "endstill_vio" / f"{rec6}_loose_raw_o{k}"), t_imu0, edges)
                        for k in range(3)],
                   imu=imu_part(rec, t_imu0, edges),
                   video=video_part(rec, t_imu0, o.t0, edges, OUT / f"{rec6}_frames.jpg"))
        json.dump(res, open(OUT / f"{rec6}.json", "w"), indent=1, default=float)
        plot(rec6, res, edges, OUT / f"{rec6}_timeline.png")
        print(rec6, "done", {k: res["imu"][k] for k in ("acc_frac_of_range", "gyr_frac_of_range")},
              "board last", res["video"]["board_last_ok_s"], [(r["name"], r["rpe_over3_first_s"], r["gt_last_s"]) for r in res["vio"]])


if __name__ == "__main__":
    main()
