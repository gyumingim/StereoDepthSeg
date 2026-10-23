#!/usr/bin/env python3
"""Collapse review data + overlay videos for divergent ZUPT-off (V4) phone60 runs (2026-10-23).

Per run (runs/phone60/<name>.json + .poses.jsonl.gz, read only):
  - VIO p aligned to the camera-spline GT (recordings/<rec>/imu_gt_spl/gt.npz) by SE3 over the first ALIGN_S after init
    (so the error grows from ~0 at init and the divergence onset is visible; the bench ATE aligns over the whole run).
  - onset = first time the aligned error exceeds ONSET_M (GT-matched frames only).
  - GT speed / angular rate, VIO speed, n_slam / n_msckf, bg / ba, status, IMU (the same source the run fed: raw 16/35
    or Android 1/4) norms + timestamp gaps, fed-frame gaps, image brightness / Laplacian variance / FAST count and a
    reference KLT track survival (own tracker, NOT OpenVINS's tracks).
  - H.264 mp4 (yuv420p, faststart, crf 28) of the fed frames (frame cache = exactly the bench input) with overlay.
Outputs: <out>/<name>.mp4 and <out>/data.js (window.COLLAPSE = {...}).

  ../venv_ffs/bin/python tools/collapse_review.py --out /mnt/c/.../review/collapse [--names a,b] [--no-video]
"""
import argparse
import csv
import gzip
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

cv2.setNumThreads(4)
SCAN3D = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCAN3D / "imu_learn"))
import vio_bench_hs as vb  # noqa: E402
from frame_cache import FrameCache  # noqa: E402
from imu_gt import read_imu  # noqa: E402

RUN = SCAN3D / "runs" / "phone60"
ALIGN_S = 3.0
ONSET_M = 0.30
DEFAULT = ["131326_o0_V4", "131326_o1_V4", "131326_o2_V4", "224323_o0_V4", "224323_o1_V4", "224323_o2_V4",
           "131326_o0_base", "224323_o0_phone60"]


def umeyama_se3(A, B):
    ma, mb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((B - mb).T @ (A - ma))
    S = np.eye(3)
    S[2, 2] = np.sign(np.linalg.det(U @ Vt))
    R = U @ S @ Vt
    return R, mb - R @ ma


def so3_log(R):
    c = np.clip((np.trace(R) - 1) / 2, -1, 1)
    th = np.arccos(c)
    if th < 1e-9:
        return np.zeros(3)
    return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * np.sin(th)) * th


def rnd(a, d=3):
    a = np.asarray(a, float)
    return [None if not np.isfinite(x) else round(float(x), d) for x in a]


_img_cache = {}


def frame_metrics(fc, fis):
    """Per fed frame: mean, Laplacian var, FAST(15) count, reference KLT survival vs previous fed frame + points."""
    fast = cv2.FastFeatureDetector_create(15)
    out, prev, pts_prev = [], None, None
    for fi in fis:
        g = fc.get(int(fi))
        key = int(fi)
        if key in _img_cache:
            m = _img_cache[key]
        else:
            m = dict(mean=float(g.mean()), lap=float(cv2.Laplacian(g, cv2.CV_64F).var()), fast=len(fast.detect(g)))
            _img_cache[key] = m
        surv, pts = np.nan, None
        if prev is not None and pts_prev is not None and len(pts_prev):
            p1, st, _ = cv2.calcOpticalFlowPyrLK(prev, g, pts_prev, None, winSize=(21, 21), maxLevel=3)
            p0b, st2, _ = cv2.calcOpticalFlowPyrLK(g, prev, p1, None, winSize=(21, 21), maxLevel=3)
            ok = (st[:, 0] == 1) & (st2[:, 0] == 1) & (np.linalg.norm(p0b - pts_prev, axis=2)[:, 0] < 1.0)
            surv = float(ok.mean())
            pts = p1[ok].reshape(-1, 2)
        nxt = cv2.goodFeaturesToTrack(g, 200, 0.01, 10)
        pts_prev = nxt if nxt is not None else None
        prev = g
        out.append(dict(m, klt=surv, pts=pts))
    return out


def analyse(name, out_dir, video=True):
    j = json.load(open(RUN / f"{name}.json"))
    rec = SCAN3D / "recordings" / j["rec"]
    P = [json.loads(x) for x in gzip.open(RUN / f"{name}.poses.jsonl.gz", "rt") if x.strip()]
    P = [p for p in P if p.get("type") == "pose"]
    t = np.array([p["t"] for p in P])
    off0 = P[0]["t_ns"] - P[0]["t"] * 1e9                       # pose t = (t_ns - off0)/1e9
    adt = j["align_dt_ms"] * 1e6
    # fed frame index per pose row: frames t_mid = pose t_ns + align_dt (checked: residual reported)
    rows = list(csv.DictReader(open(rec / "frames_cam5.csv")))
    tf = np.array([int(r["t_mid_ns"]) for r in rows])
    tq = np.array([p["t_ns"] for p in P]) + adt
    k = np.clip(np.searchsorted(tf, tq), 1, len(tf) - 1)
    k = np.where(np.abs(tf[k - 1] - tq) < np.abs(tf[k] - tq), k - 1, k)
    fi = k
    fi_res_ms = float(np.max(np.abs(tf[fi] - tq)) / 1e6)
    status = [p.get("status") for p in P]
    trk = np.array([s in ("tracking", "lost") and p.get("p") is not None for s, p in zip(status, P)])
    pv = np.array([p["p"] if p.get("p") else [np.nan] * 3 for p in P])
    vv = np.array([p["v"] if p.get("v") else [np.nan] * 3 for p in P])
    bg = np.array([p["bg"] if p.get("bg") else [np.nan] * 3 for p in P])
    ba = np.array([p["ba"] if p.get("ba") else [np.nan] * 3 for p in P])
    nsl = np.array([p.get("n_slam") or 0 for p in P])
    nms = np.array([p.get("n_msckf") or 0 for p in P])
    i_init = int(np.argmax(trk)) if trk.any() else None
    t_init = float(t[i_init]) if i_init is not None else None
    lost_t = next((float(tt) for tt, s in zip(t, status) if s == "lost"), None)
    # GT
    g = np.load(rec / "imu_gt_spl" / "gt.npz", allow_pickle=True)
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    tg_all = (g["t_ns"] - off0 - adt) / 1e9
    tg, cg, Rg = tg_all[ok], g["p_bc"][ok], g["R_bc"][ok]
    o = np.argsort(tg)
    tg, cg, Rg = tg[o], cg[o], Rg[o]
    kk = np.clip(np.searchsorted(tg, t), 1, len(tg) - 1)
    kk = np.where(np.abs(tg[kk - 1] - t) < np.abs(tg[kk] - t), kk - 1, kk)
    gmatch = np.abs(tg[kk] - t) <= 0.002
    gp = np.where(gmatch[:, None], cg[kk], np.nan)
    # GT speed / angular rate (consecutive ok GT samples <= 10 ms apart), on the pose grid
    dtg = np.diff(tg)
    sp = np.linalg.norm(np.diff(cg, axis=0), axis=1) / np.maximum(dtg, 1e-6)
    wr = np.array([np.linalg.norm(so3_log(Rg[i].T @ Rg[i + 1])) for i in range(len(Rg) - 1)]) / np.maximum(dtg, 1e-6)
    tm = (tg[1:] + tg[:-1]) / 2
    good = dtg < 0.010
    def smooth_on(x, tt, win=0.1):
        if not good.any():
            return np.full(len(tt), np.nan)
        c = np.convolve(x[good], np.ones(5) / 5, mode="same")
        y = np.interp(tt, tm[good], c)
        near = np.abs(tm[good][np.clip(np.searchsorted(tm[good], tt), 0, good.sum() - 1)] - tt) < win
        return np.where(near, y, np.nan)
    gspeed = smooth_on(sp, t)
    gw = smooth_on(wr, t)
    # alignment over [t_init, t_init+ALIGN_S]
    vspeed = np.linalg.norm(vv, axis=1)
    sp_bad = np.nonzero(trk & (vspeed > np.nan_to_num(gspeed, nan=0) + 1.0))[0]
    t_vblow = float(t[sp_bad[0]]) if len(sp_bad) else None
    err = np.full(len(t), np.nan)
    pa = np.full((len(t), 3), np.nan)
    onset = e10 = None
    align_n = 0
    align_s = ALIGN_S                       # a run that blows up within ALIGN_S: align on half the time before blow-up
    if i_init is not None and t_vblow is not None and t_vblow - t_init < 2 * ALIGN_S:
        align_s = float(np.clip((t_vblow - t_init) / 2, 0.5, ALIGN_S))
    if i_init is not None:
        m = trk & gmatch & (t >= t_init) & (t <= t_init + align_s)
        align_n = int(m.sum())
        if align_n >= 20:
            R, tr = umeyama_se3(pv[m], gp[m])
            pa = (R @ np.nan_to_num(pv).T).T + tr
            pa[~trk] = np.nan
            err = np.linalg.norm(pa - gp, axis=1)
            over = np.nonzero(trk & gmatch & (err > ONSET_M) & (t > t_init))[0]
            onset = float(t[over[0]]) if len(over) else None
            o10 = np.nonzero(trk & gmatch & (err > 0.10) & (t > t_init))[0]
            e10 = float(t[o10[0]]) if len(o10) else None
    # IMU (same source as the run)
    raw = j.get("imu_source") == "raw_16_35"
    acc, gyr = read_imu(rec / "imu.csv", raw=raw)
    ti = (gyr[:, 0] - off0) / 1e9
    ta = (acc[:, 0] - off0) / 1e9
    gn = np.linalg.norm(gyr[:, 1:4], axis=1)
    an = np.linalg.norm(acc[:, 1:4], axis=1)
    dti = np.diff(gyr[:, 0]) / 1e6
    imu = dict(source=j.get("imu_source"), n_gyr=len(gyr), n_acc=len(acc), gyr_dt_med_ms=float(np.median(dti)),
               gyr_dt_max_ms=float(dti.max()), gyr_gaps_gt5ms=int((dti > 5).sum()), nonmono=int((dti <= 0).sum()),
               acc_absmax=rnd(np.abs(acc[:, 1:4]).max(0), 2), gyr_absmax=rnd(np.abs(gyr[:, 1:4]).max(0), 2))
    # saturation: count samples at the per-axis extreme value (flat top)
    sat = []
    for X, nm in ((acc, "acc"), (gyr, "gyr")):
        for q in range(3):
            v = np.abs(X[:, 1 + q])
            sat.append(f"{nm}{'xyz'[q]}:{int((v >= v.max() * 0.999).sum())}")
    imu["at_axis_max"] = sat
    # IMU per pose row (max norm over the preceding frame interval)
    gi = np.searchsorted(ti, t)
    gmax = np.array([gn[gi[i - 1]:max(gi[i], gi[i - 1] + 1)].max() if i else gn[0] for i in range(len(t))])
    ai = np.searchsorted(ta, t)
    amax = np.array([an[ai[i - 1]:max(ai[i], ai[i - 1] + 1)].max() if i else an[0] for i in range(len(t))])
    imu_gap = np.array([(dti[max(gi[i - 1] - 1, 0):max(gi[i] - 1, gi[i - 1])].max() if i and gi[i] > gi[i - 1]
                         else np.nan) for i in range(len(t))])
    # init window: true motion before init (GT) and IMU mean gyro in the 1 s before init
    init = {}
    if t_init is not None:
        w = (ti >= t_init - 1.0) & (ti <= t_init)
        wo = (ti > t_init - 1.0) & (ti <= t_init - 0.5)       # OpenVINS static init (wait_for_jerk): bg = mean gyro here
        init = dict(t_init=round(t_init, 3), bg0=rnd(bg[i_init], 4), ba0=rnd(ba[i_init], 3),
                    ov_oldhalf_gyro_mean=rnd(gyr[wo, 1:4].mean(0), 4), ov_oldhalf_gyro_absmean=round(float(gn[wo].mean()), 3),
                    gyro_mean_1s_before=rnd(gyr[w, 1:4].mean(0), 4), gyro_std_1s_before=rnd(gyr[w, 1:4].std(0), 4),
                    acc_mean_1s_before=rnd(acc[(ta >= t_init - 1) & (ta <= t_init), 1:4].mean(0), 3),
                    gt_angrate_med_1s_before=round(float(np.nanmedian(gw[(t >= t_init - 1) & (t <= t_init)])), 4)
                    if np.isfinite(gw[(t >= t_init - 1) & (t <= t_init)]).any() else None,
                    gt_speed_med_1s_before=round(float(np.nanmedian(gspeed[(t >= t_init - 1) & (t <= t_init)])), 4)
                    if np.isfinite(gspeed[(t >= t_init - 1) & (t <= t_init)]).any() else None,
                    bg_3s_after=rnd(bg[min(np.searchsorted(t, t_init + 3), len(t) - 1)], 4),
                    bg_end=rnd(bg[trk][-1], 4), ba_end=rnd(ba[trk][-1], 3))
    still_t = None                          # first time after init with |gyro| < 0.03 rad/s for the next 2 s (phone put down)
    if t_init is not None:
        for i in np.nonzero(t > t_init)[0][::6]:
            w = (ti >= t[i]) & (ti <= t[i] + 2.0)
            if w.sum() > 100 and gn[w].max() < 0.03:
                still_t = float(t[i])
                break
    # frames
    fed_dt = np.r_[np.nan, np.diff(tf[fi]) / 1e6]
    cache_dir = j["frame_cache"]["dir"]
    meta = json.load(open(rec / "meta.json"))
    m1, m2, _, (W, H) = vb.undistort_maps(rec, meta, j["size"])
    fc = FrameCache(rec, (W, H), m1, m2)
    assert str(fc.dir).endswith(cache_dir.split("/")[-1]), (fc.dir, cache_dir)
    fm = frame_metrics(fc, fi)
    mean = np.array([x["mean"] for x in fm]); lap = np.array([x["lap"] for x in fm])
    fastn = np.array([x["fast"] for x in fm]); klt = np.array([x["klt"] for x in fm])

    def around(x, t0, a=-1.0, b=1.0):
        if t0 is None:
            return None
        m = (t >= t0 + a) & (t <= t0 + b)
        return round(float(np.nanmedian(x[m])), 3) if m.any() and np.isfinite(x[m]).any() else None

    def amax_(x, t0, a=-1.0, b=1.0):
        if t0 is None:
            return None
        m = (t >= t0 + a) & (t <= t0 + b)
        return round(float(np.nanmax(x[m])), 3) if m.any() and np.isfinite(x[m]).any() else None
    ref = onset
    at_onset = dict(gt_speed_med=around(gspeed, ref), gt_speed_max=amax_(gspeed, ref), gt_angrate_med=around(gw, ref),
                    gt_angrate_max=amax_(gw, ref), gyro_max=amax_(gmax, ref), acc_max=amax_(amax, ref),
                    n_slam_med=around(nsl.astype(float), ref), n_msckf_med=around(nms.astype(float), ref),
                    fed_dt_max_ms=amax_(fed_dt, ref), imu_gap_max_ms=amax_(imu_gap, ref), mean=around(mean, ref),
                    lap=around(lap, ref), fast=around(fastn.astype(float), ref), klt=around(klt, ref),
                    gt_cover=round(float(gmatch[(t >= (ref or 0) - 1) & (t <= (ref or 0) + 1)].mean()), 3) if ref else None)
    # whole-run medians (tracking part) for comparison
    allm = trk if trk.any() else np.ones(len(t), bool)
    whole = dict(gt_speed_med=round(float(np.nanmedian(gspeed[allm])), 3), gt_angrate_med=round(float(np.nanmedian(gw[allm])), 3),
                 gt_angrate_p95=round(float(np.nanpercentile(gw[allm], 95)), 3),
                 n_slam_med=float(np.median(nsl[allm])), n_msckf_med=float(np.median(nms[allm])),
                 mean=round(float(np.median(mean)), 1), lap=round(float(np.median(lap)), 1),
                 fast=float(np.median(fastn)), klt=round(float(np.nanmedian(klt)), 3),
                 fed_dt_gt25ms=int((fed_dt > 25).sum()), gt_cover=round(float(gmatch.mean()), 3))
    # first 3 s after init (early window)
    early = {}
    if t_init is not None:
        early = dict(gt_speed_med=around(gspeed, t_init, 0, 3), gt_angrate_med=around(gw, t_init, 0, 3),
                     gt_angrate_max=amax_(gw, t_init, 0, 3), n_slam_med=around(nsl.astype(float), t_init, 0, 3),
                     n_msckf_med=around(nms.astype(float), t_init, 0, 3), klt=around(klt, t_init, 0, 3),
                     fast=around(fastn.astype(float), t_init, 0, 3), fed_dt_max_ms=amax_(fed_dt, t_init, 0, 3))
    summ = dict(name=name, rec=j["rec"], cond=j.get("p60_cond"), imu_source=j.get("imu_source"), ate_cm=j.get("ate_cm"),
                scale=j.get("scale"), tracked_frac=j.get("tracked_frac"), t_init=t_init, onset=onset, err10_t=e10,
                vblow_t=t_vblow, lost_t=lost_t, still_t=still_t, align_s=align_s, align_n=align_n, fi_res_ms=round(fi_res_ms, 3), init=init,
                at_onset=at_onset, early=early, whole=whole, imu=imu, frames_rec=j["frames"]["rec_all"],
                frames_fed=j["frames"]["fed"])
    # series for the page (every frame -> too big; every 2nd row ~30 Hz)
    s = slice(None, None, 2)
    ser = dict(t=rnd(t[s], 3), gx=rnd(gp[s, 0]), gy=rnd(gp[s, 1]), gz=rnd(gp[s, 2]), vx=rnd(pa[s, 0]), vy=rnd(pa[s, 1]),
               vz=rnd(pa[s, 2]), err=rnd(err[s]), gspeed=rnd(gspeed[s], 2), vspeed=rnd(np.where(trk, vspeed, np.nan)[s], 2),
               gw=rnd(gw[s], 2), gyro=rnd(gmax[s], 2), nslam=nsl[s].tolist(), nmsckf=nms[s].tolist(),
               klt=rnd(klt[s], 2), lap=rnd(lap[s], 0), fast=fastn[s].tolist(), mean=rnd(mean[s], 0))
    vt = rnd(t, 3)                                               # video frame k -> run time t[k] (60 fps CFR video)
    if video:
        write_video(out_dir / f"{name}.mp4", fc, fi, t, status, nsl, nms, err, onset, t_init, fm, fed_dt)
    return summ, ser, vt


def write_video(path, fc, fi, t, status, nsl, nms, err, onset, t_init, fm, fed_dt):
    import av
    cont = av.open(str(path), "w", options={"movflags": "+faststart"})
    st = cont.add_stream("libx264", rate=60)
    st.width, st.height, st.pix_fmt = 640, 360, "yuv420p"
    st.options = {"crf": "28", "preset": "medium", "threads": "8"}
    font = cv2.FONT_HERSHEY_SIMPLEX
    for k, f in enumerate(fi):
        img = cv2.cvtColor(fc.get(int(f)), cv2.COLOR_GRAY2BGR)
        pts = fm[k]["pts"]
        if pts is not None:
            for x, y in pts:
                cv2.circle(img, (int(x), int(y)), 2, (0, 255, 0), -1, cv2.LINE_AA)
        if onset is not None and t[k] >= onset:
            cv2.rectangle(img, (0, 0), (640, 30), (0, 0, 200), -1)
            cv2.putText(img, f"DIVERGED  onset {onset:.2f}s  (+{t[k] - onset:.1f}s)", (8, 21), font, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
        stt = status[k] or ""
        e = err[k]
        lines = [f"t {t[k]:7.2f}s  frame {k}  fi {int(f)}",
                 f"{stt.upper():9s} slam {nsl[k]:3d} msckf {nms[k]:3d}" + (f"  err {e * 100:6.1f}cm" if np.isfinite(e) else ""),
                 f"KLT ref {fm[k]['klt'] * 100:3.0f}%  blur(lap) {fm[k]['lap']:5.0f}" if np.isfinite(fm[k]["klt"]) else ""]
        if np.isfinite(fed_dt[k]) and fed_dt[k] > 25:
            lines.append(f"FRAME GAP {fed_dt[k]:.0f} ms")
        if t_init is not None and t_init <= t[k] < t_init + 1.0:
            lines.append("INIT")
        y0 = 360 - 10 - 20 * (len(lines) - 1)
        for i, s in enumerate(lines):
            if not s:
                continue
            col = (0, 200, 255) if s.startswith("FRAME GAP") or s == "INIT" else (255, 255, 255)
            cv2.putText(img, s, (8, y0 + 20 * i), font, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img, s, (8, y0 + 20 * i), font, 0.5, col, 1, cv2.LINE_AA)
        fr = av.VideoFrame.from_ndarray(img, format="bgr24")
        for pk in st.encode(fr):
            cont.mux(pk)
    for pk in st.encode():
        cont.mux(pk)
    cont.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--names", default=",".join(DEFAULT))
    ap.add_argument("--no-video", action="store_true")
    o = ap.parse_args()
    out = Path(o.out)
    out.mkdir(parents=True, exist_ok=True)
    D = dict(made=time.strftime("%F %T"), align_s=ALIGN_S, onset_m=ONSET_M, runs={})
    for n in o.names.split(","):
        t0 = time.time()
        summ, ser, vt = analyse(n, out, not o.no_video)
        D["runs"][n] = dict(summary=summ, series=ser, vt=vt)
        print(n, f"{time.time() - t0:.0f}s", json.dumps({k: summ[k] for k in ("t_init", "onset", "err10_t", "vblow_t", "lost_t", "still_t", "align_s")}), flush=True)
    (out / "data.js").write_text("window.COLLAPSE = " + json.dumps(D, ensure_ascii=False) + ";\n")
    (out / "summary.json").write_text(json.dumps({n: r["summary"] for n, r in D["runs"].items()}, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
