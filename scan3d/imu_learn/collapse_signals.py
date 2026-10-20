#!/usr/bin/env python3
"""GT-free collapse signals after OpenVINS initialization (runs from collapse_run.py).

For every run: the per-frame pose stream (p, q, v, bg, ba, sig15, n_track, n_slam, n_msckf) and the same IMU the run fed
(Android types 1/4 + the ablated calibration in the run JSON) -> signals in the first 2/5/10 s after init, and online
(running) versions for detection time / false alarms. GT is used ONLY for the label (collapsed / healthy), never in a signal.

Label (GT, camera-only spline GT as in the bench): Sim3 fit over [t_init, t_init + 10 s] -> window scale c10;
collapsed if c10 < 0.5 or c10 > 2 or SE3 ATE over the window > 10 cm.

  ../venv_ffs/bin/python imu_learn/collapse_signals.py runs/collapse/*.json --out runs/collapse/signals.json
"""
import argparse
import gzip
import json
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import vio_bench_hs as vb  # noqa: E402
from imu_gt import read_imu  # noqa: E402

G = np.array([0.0, 0.0, 9.80])        # = gravity_mag of config/mono/estimator_config.yaml (bench template), world z up
WINS = (2, 5, 10)


def quat_R(q):                         # Hamilton x,y,z,w -> R (world <- IMU)
    x, y, z, w = q.T
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
                     np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
                     np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], -2)


def expm(w):
    th = np.linalg.norm(w)
    K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    if th < 1e-12:
        return np.eye(3) + K
    return np.eye(3) + np.sin(th) / th * K + (1 - np.cos(th)) / th ** 2 * K @ K


def load(prefix):
    d = json.load(open(prefix + ".json"))
    P = [json.loads(line) for line in gzip.open(prefix + ".poses.jsonl.gz", "rt")]
    t = np.array([m["t_ns"] for m in P], np.int64)
    st = np.array([m.get("status", "") for m in P])

    def col(k, n):
        return np.array([m[k] if m.get(k) is not None else [np.nan] * n for m in P], float).reshape(len(P), n)
    r = dict(meta=d, t_ns=t, status=st, p=col("p", 3), q=col("q", 4), v=col("v", 3), bg=col("bg", 3), ba=col("ba", 3),
             sig=col("sig15", 15), T=col("T_world_cam0", 16),
             n_track=np.array([m.get("n_track", 0) for m in P], float),
             n_slam=np.array([m.get("n_slam", 0) for m in P], float),
             n_msckf=np.array([m.get("n_msckf", 0) for m in P], float),
             dt=np.array([m.get("dt_cam_imu") if m.get("dt_cam_imu") is not None else np.nan for m in P], float))
    r["t_br"] = np.array([m.get("t", np.nan) for m in P], float)       # bridge time base (= OpenVINS camera time)
    r.update(parse_log(prefix + ".stderr.log.gz"))
    return r


ANSI = re.compile(r"\x1b\[[0-9;]*m")
NUM = r"(-?(?:inf|nan|[0-9.]+(?:e[-+]?\d+)?))"


def parse_log(fn):
    """OpenVINS DEBUG log -> init facts (method, disparity, excitation, static bias, dynamic |g|/cond) of the SUCCESSFUL
    attempt, and per-frame events keyed by [DIAGT] frame time (diag library build only): ZUPT accepted, MSCKF/SLAM
    chi2 test in/rejected/mean chi2 ratio."""
    init, cur, frames, ft = {}, {}, {}, None
    done = False
    for line in gzip.open(fn, "rt", errors="replace"):
        line = ANSI.sub("", line)
        if not done:
            if "USING STATIC" in line:      # follows the 'disparity is' line of the same attempt
                cur["method"] = "static"
            elif "USING DYNAMIC" in line:
                cur["method"] = "dynamic"
            elif "[init]: disparity is" in line:
                m = re.search(r"disparity is " + NUM + "," + NUM, line)
                cur = dict(disp_old=float(m[1]), disp_new=float(m[2]))
            elif "IMU excitation stats" in line:
                m = re.search(r"stats: " + NUM + "," + NUM, line)
                cur.update(acc_var_old=float(m[1]), acc_var_new=float(m[2]))
            elif "[init-d]: gravity in I0" in line:
                cur["dyn_g"] = float(re.search(r"\|g\| = " + NUM, line)[1])
            elif "[init-d]: CM cond" in line:
                cur["dyn_cond"] = float(re.search(r"cond = " + NUM, line)[1])
            elif "[init-d]: |theta_I|" in line:
                m = re.search(r"\|theta_I\| = " + NUM + r" deg and \|accel\| = " + NUM, line)
                cur.update(dyn_theta_deg=float(m[1]), dyn_accel=float(m[2]))
            elif "successful initialization" in line:
                init = dict(cur)
                done = True
            continue
        if "[init]: bias gyro =" in line:
            init["bg_init"] = [float(x) for x in re.findall(NUM, line.split("=")[1])]
        elif "[init]: velocity =" in line:
            init["v_init"] = [float(x) for x in re.findall(NUM, line.split("=")[1])]
        elif "[init]: bias accel =" in line:
            init["ba_init"] = [float(x) for x in re.findall(NUM, line.split("=")[1])]
        elif "[DIAGT]" in line:
            ft = round(float(line.split("]")[-1]), 4)
            frames[ft] = dict(zupt=0, zupt_v=np.nan, m_in=0, m_rej=0, m_ratio=np.nan, s_in=0, s_rej=0, s_ratio=np.nan)
        elif ft is not None and "[ZUPT]: accepted" in line:
            frames[ft]["zupt"] = 1
            frames[ft]["zupt_v"] = float(re.search(r"v_IinG\| = " + NUM, line)[1])
        elif ft is not None and "[CHI2]" in line:
            m = re.search(r"\[CHI2\] (msckf|slam) in (\d+) rej (\d+) mean_ratio " + NUM, line)
            k = "m" if m[1] == "msckf" else "s"            # SLAM update runs in chunks -> several lines per frame: sum
            f = frames[ft]
            n_old, n_new = f[f"{k}_in"], int(m[2])
            if n_new:
                r_old = 0.0 if not np.isfinite(f[f"{k}_ratio"]) else f[f"{k}_ratio"]
                f[f"{k}_ratio"] = (r_old * n_old + float(m[4]) * n_new) / (n_old + n_new)
            f[f"{k}_in"] = n_old + n_new
            f[f"{k}_rej"] += int(m[3])
    return dict(init_info=init, frame_log=frames)


def imu_for(run):
    d = run["meta"]
    rec = vb.SCAN3D / "recordings" / d["rec"]
    acc, gyr = read_imu(rec / "imu.csv")
    a_i = np.stack([np.interp(gyr[:, 0], acc[:, 0], acc[:, 1 + j]) for j in range(3)], 1)
    g_i = gyr[:, 1:4].copy()
    cal = d.get("calib_used")
    if cal:
        g_i = g_i @ np.asarray(cal["G"]).T + np.asarray(cal["bg"])
        a_i = a_i @ np.asarray(cal["A"]).T + np.asarray(cal["ba"])
    return gyr[:, 0].astype(np.int64), g_i, a_i


def corrections(run, ti, gi, ai, i0):
    """Per frame k -> k+1 after init: IMU-only prediction of the velocity change from the state at k (R_k, bg_k, ba_k),
    compared to the reported v_{k+1} - v_k -> dv_corr (= what the camera update changed, m/s). Also position."""
    # OpenVINS integrates IMU up to t_cam + dt_cam_imu (online time offset) -> shift each frame time onto the IMU clock
    dts = np.nan_to_num(np.array([run["dt"][i] for i in range(len(run["t_ns"]))], float))
    t, n = run["t_ns"] + (dts * 1e9).astype(np.int64), len(run["t_ns"])
    R = quat_R(run["q"])
    dv = np.full((n, 3), np.nan)
    dp = np.full((n, 3), np.nan)
    for k in range(i0, n - 1):
        if not (np.isfinite(run["q"][k]).all() and np.isfinite(run["q"][k + 1]).all()):
            continue
        a, b = np.searchsorted(ti, t[k], "right"), np.searchsorted(ti, t[k + 1], "right")
        Rk, vel, pos = R[k].copy(), run["v"][k].copy(), run["p"][k].copy()
        tt = t[k]
        j = a - 1
        while True:                                   # piecewise-constant IMU between samples (zero-order hold)
            tn = ti[j + 1] if j + 1 < b else t[k + 1]
            h = (tn - tt) / 1e9
            if h > 0 and j >= 0:
                acc_w = Rk @ (ai[j] - run["ba"][k]) - G
                pos = pos + vel * h + 0.5 * acc_w * h * h
                vel = vel + acc_w * h
                Rk = Rk @ expm((gi[j] - run["bg"][k]) * h)
            tt = tn
            j += 1
            if tn >= t[k + 1]:
                break
        dv[k + 1] = run["v"][k + 1] - vel
        dp[k + 1] = run["p"][k + 1] - pos
    return dv, dp


def label(run, i0):
    d = run["meta"]
    rec = vb.SCAN3D / "recordings" / d["rec"]
    g = np.load(vb.pick_gt(rec, "auto") / "gt.npz")
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    gt_t, gt_p = g["t_ns"][ok], g["p_bc"][ok]
    m = (run["status"] == "tracking") & np.isfinite(run["T"][:, 0])
    vt = run["t_ns"] + int(d["align_dt_ms"] * 1e6)
    t0 = run["t_ns"][i0]
    out = {}
    for W in (5, 10):
        w = m & (run["t_ns"] >= t0) & (run["t_ns"] <= t0 + W * 1e9)
        P = run["T"][w].reshape(-1, 4, 4)[:, :3, 3]
        vv = vt[w]
        j = np.clip(np.searchsorted(gt_t, vv), 1, len(gt_t) - 1)
        j = np.where(np.abs(gt_t[j - 1] - vv) < np.abs(gt_t[j] - vv), j - 1, j)
        mm = np.abs(gt_t[j] - vv) < 2e6
        if mm.sum() < 30:
            continue
        P, Pg = P[mm], gt_p[j[mm]]
        Rs, c, ts = vb.umeyama(P, Pg, scale=True)
        R1, _, t1 = vb.umeyama(P, Pg)
        e = np.linalg.norm(P @ R1.T + t1 - Pg, axis=1)
        out[f"scale{W}"] = float(c)
        out[f"ate{W}_cm"] = float(np.sqrt((e ** 2).mean()) * 100)
        out[f"path_ratio{W}"] = float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum() /
                                      max(1e-6, np.linalg.norm(np.diff(Pg, axis=0), axis=1).sum()))
    c = out.get("scale10", out.get("scale5", np.nan))
    a = out.get("ate10_cm", out.get("ate5_cm", np.nan))
    out["collapsed"] = None if not np.isfinite(c) else bool((c < 0.5) or (c > 2) or (a > 10))   # None = no label
    return out


def analyse(prefix):
    run = load(prefix)
    st = run["status"]
    ini = np.nonzero(st != "init")[0]
    if not len(ini):
        return dict(name=Path(prefix).name, error="no init")
    i0 = int(ini[0])
    t = (run["t_ns"] - run["t_ns"][i0]) / 1e9
    ti, gi, ai = imu_for(run)
    dv, dp = corrections(run, ti, gi, ai, i0)
    # pre-init motion (GT-free): gyro / accel in the 1.5 s before init (= OpenVINS init window)
    t0 = run["t_ns"][i0]
    pre = (ti > t0 - 1.5e9) & (ti <= t0)
    res = dict(name=Path(prefix).name, rec=run["meta"]["rec"], parts=run["meta"].get("parts"),
               start_off=run["meta"].get("start_off", 0.0),
               t_init_s=round(float((t0 - run["t_ns"][0]) / 1e9), 3),
               init_method=run["init_info"].get("method", "?"),
               init_info={k: v for k, v in run["init_info"].items() if k != "method"},
               bg_init_dps=(float(np.degrees(np.linalg.norm(run["init_info"]["bg_init"])))
                            if "bg_init" in run["init_info"] else None),
               v_at_init=float(np.linalg.norm(run["v"][i0])),
               pre_gyro_mean_dps=float(np.degrees(np.linalg.norm(gi[pre].mean(0)))),
               pre_gyro_max_dps=float(np.degrees(np.linalg.norm(gi[pre], axis=1).max())),
               pre_acc_std=float(np.linalg.norm(ai[pre].std(0))),
               bench_scale=run["meta"].get("scale"), bench_ate_cm=run["meta"].get("ate_cm"))
    res.update(label(run, i0))
    vn = np.linalg.norm(run["v"], axis=1)
    bgn = np.degrees(np.linalg.norm(run["bg"], axis=1))
    ban = np.linalg.norm(run["ba"], axis=1)
    dvn = np.linalg.norm(dv, axis=1)
    dpn = np.linalg.norm(dp, axis=1)
    sig = run["sig"]
    sv = np.linalg.norm(sig[:, 6:9], axis=1)
    sp = np.linalg.norm(sig[:, 3:6], axis=1)
    syaw = np.degrees(sig[:, 2])
    sbg = np.degrees(np.linalg.norm(sig[:, 9:12], axis=1))
    sba = np.linalg.norm(sig[:, 12:15], axis=1)
    used = run["n_slam"] + run["n_msckf"]
    fl = run["frame_log"]                                   # per-frame log events (diag library runs only)
    has_log = len(fl) > 0
    ev = {k: np.full(len(t), np.nan) for k in ("zupt", "zupt_v", "m_in", "m_rej", "m_ratio", "s_in", "s_rej", "s_ratio")}
    if has_log:
        for i, tb in enumerate(run["t_br"]):
            e = fl.get(round(float(tb), 4)) if np.isfinite(tb) else None
            if e is not None:
                for kk in ev:
                    ev[kk][i] = e[kk]
    res["has_chi2_log"] = bool(has_log and np.isfinite(ev["m_in"][t >= 0]).any())
    for W in WINS:
        w = (t >= 0) & (t <= W)
        dt = np.diff(run["t_ns"][w]) / 1e9
        path = np.linalg.norm(np.diff(run["p"][w], axis=0), axis=1).sum()
        dvw = dv[w]
        okc = np.isfinite(dvw[:, 0])
        T = max(1e-3, t[w][-1] - t[w][0])
        res[f"w{W}"] = dict(
            v_max=float(np.nanmax(vn[w])), v_med=float(np.nanmedian(vn[w])), path_m=float(path),
            bg_max_dps=float(np.nanmax(bgn[w])), bg_end_dps=float(bgn[w][-1]), ba_max=float(np.nanmax(ban[w])),
            dv_rms=float(np.sqrt(np.nanmean(dvn[w] ** 2))), dv_sum_acc=float(np.linalg.norm(np.nansum(dvw[okc], 0)) / T),
            dp_rms_mm=float(np.sqrt(np.nanmean(dpn[w] ** 2)) * 1e3),
            dv_nis=float(np.nanmedian(dvn[w] / np.maximum(sv[w], 1e-6))),
            sig_v_end=float(sv[w][-1]), sig_p_end=float(sp[w][-1]), sig_yaw_end_deg=float(syaw[w][-1]),
            sig_bg_end_dps=float(sbg[w][-1]), sig_ba_end=float(sba[w][-1]),
            sig_v_max=float(np.nanmax(sv[w])),
            n_track_med=float(np.median(run["n_track"][w])), used_med=float(np.median(used[w])),
            used_ratio=float(np.median(used[w] / np.maximum(run["n_track"][w], 1))),
            zero_upd_frac=float((used[w] == 0).mean()), fps=float(len(dt) / max(1e-3, dt.sum())))
        if res["has_chi2_log"]:
            nin = np.nansum(ev["m_in"][w]) + np.nansum(ev["s_in"][w])
            nrej = np.nansum(ev["m_rej"][w]) + np.nansum(ev["s_rej"][w])
            res[f"w{W}"].update(zupt_frac=float(np.nanmean(ev["zupt"][w])), chi2_in=float(nin),
                                chi2_rej_rate=float(nrej / max(nin, 1)),
                                msckf_ratio_med=float(np.nanmedian(ev["m_ratio"][w])) if np.isfinite(ev["m_ratio"][w]).any() else None)
    # online series (for detection time / false alarms): 1-s running RMS of dv_corr, running |bg|, |v|, sig_v ...
    k = np.nonzero(t >= 0)[0]
    tt = t[k]
    def run_rms(x, L=1.0):
        o = np.full(len(k), np.nan)
        for n_, i in enumerate(k):
            w = (t > t[i] - L) & (t <= t[i]) & (t >= 0)
            o[n_] = np.sqrt(np.nanmean(x[w] ** 2))
        return o
    def run_mean(x, L=1.0):
        o = np.full(len(k), np.nan)
        for n_, i in enumerate(k):
            w = (t > t[i] - L) & (t <= t[i]) & (t >= 0)
            o[n_] = np.nanmean(x[w])
        return o
    res["series"] = dict(t=tt.round(3).tolist(), dv_rms1=run_rms(dvn).round(5).tolist(),
                         dv_acc1=run_mean_vec(dv, t, k).round(4).tolist(),
                         bg_dps=bgn[k].round(4).tolist(), ba=ban[k].round(4).tolist(), v=vn[k].round(4).tolist(),
                         sig_v=sv[k].round(5).tolist(), sig_yaw_deg=syaw[k].round(4).tolist(),
                         used=run_mean(used, 0.5).round(1).tolist(), n_track=run["n_track"][k].tolist(),
                         dv_nis1=run_mean(dvn / np.maximum(sv, 1e-6)).round(3).tolist())
    # 'moving without camera correction': time since the last camera update (chi2-accepted MSCKF/SLAM features > 0, from
    # the log), counting only frames where ZUPT was NOT accepted (image says 'still' -> no camera update expected, timer
    # pauses but does not reset; a verifier showed a reset hid 13-14 s gaps behind one ZUPT frame per second)
    nu = np.zeros(len(t))
    acc_t = 0.0
    for i in range(len(t)):
        if t[i] < 0:
            continue
        cam_upd = (ev["m_in"][i] - ev["m_rej"][i] + ev["s_in"][i] - ev["s_rej"][i] > 0) if has_log else used[i] > 0
        if cam_upd:
            acc_t = 0.0
        elif i > 0 and t[i - 1] >= 0 and not (has_log and ev["zupt"][i] == 1):
            acc_t += t[i] - t[i - 1]
        nu[i] = acc_t
    res["series"]["noupd_s"] = nu[k].round(3).tolist()
    res["series"]["sig_p"] = sp[k].round(4).tolist()
    if res["has_chi2_log"]:
        rej = np.nan_to_num(ev["m_rej"]) + np.nan_to_num(ev["s_rej"])
        nin = np.nan_to_num(ev["m_in"]) + np.nan_to_num(ev["s_in"])
        o = np.full(len(k), np.nan)
        for n_, i in enumerate(k):
            w = (t > t[i] - 1.0) & (t <= t[i]) & (t >= 0)
            o[n_] = rej[w].sum() / max(nin[w].sum(), 1)
        res["series"]["chi2_rej1"] = o.round(4).tolist()
        res["series"]["zupt"] = np.nan_to_num(ev["zupt"][k]).tolist()
        zv = np.full(len(k), 0.0)        # largest |v| the filter had when the image said 'standing still' (last 2 s)
        for n_, i in enumerate(k):
            w = (t > t[i] - 2.0) & (t <= t[i]) & (t >= 0) & np.isfinite(ev["zupt_v"])
            zv[n_] = ev["zupt_v"][w].max() if w.any() else 0.0
        res["series"]["zupt_v2"] = zv.round(4).tolist()
    return res


def run_mean_vec(dv, t, k, L=1.0):
    """|sum of velocity corrections over the last L s| / L (m/s^2) = the constant specific-force error that would explain
    them (gravity/scale/accel-bias inconsistency between IMU and camera)."""
    o = np.full(len(k), np.nan)
    for n_, i in enumerate(k):
        w = (t > t[i] - L) & (t <= t[i]) & (t >= 0) & np.isfinite(dv[:, 0])
        span = max(min(L, t[i]), 0.1)
        o[n_] = np.linalg.norm(dv[w].sum(0)) / span
    return o


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    res = []
    for f in a.runs:
        pre = f[:-5] if f.endswith(".json") else f
        if not Path(pre + ".poses.jsonl.gz").exists():
            continue
        r = analyse(pre)
        res.append(r)
        print(r["name"], r.get("init_method"), "t_init", r.get("t_init_s"), "scale10", r.get("scale10"),
              "collapsed", r.get("collapsed"), file=sys.stderr)
    json.dump(res, open(a.out, "w"), indent=0)


if __name__ == "__main__":
    main()
