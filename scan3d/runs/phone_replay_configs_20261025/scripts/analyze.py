#!/usr/bin/env python3
"""Per-run metrics of a replay run dir (tl.csv = bridge --timing-log, po.jsonl = pose lines, ov_timing.txt = OpenVINS
record_timing). analyze.py RUN_DIR [RUN_DIR ...] -> RUN_DIR/metrics.json + one summary line each.
proc_ms = remap_ms + feed_ms (bridge per frame). Accuracy: camera position (T_world_cam0) vs board GT
imu_gt_spl_cal1022 of the recording (pose t_ns = camera time on IMU clock, + align dt = GT time), nearest GT <2 ms,
SE3 ATE (cm) + Sim3 scale."""
import csv
import json
import sys
from pathlib import Path

import numpy as np

REC = Path("/home/system/scanner/StereoDepthSeg/scan3d/recordings/hs_20261021_213341")
STAGES = ["tracking", "propagation", "msckf update", "slam update", "slam delayed", "re-tri & marg", "total"]
BINS = [(0, 0), (1, 25), (26, 50), (51, 75), (76, 100)]


def pc(a, q):
    return round(float(np.percentile(a, q)), 2) if len(a) else None


def umeyama(src, dst, scale=False):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    X, Y = src - mu_s, dst - mu_d
    U, S, Vt = np.linalg.svd(Y.T @ X / len(src))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    c = np.trace(np.diag(S) @ D) / X.var(0).sum() if scale else 1.0
    return R, c, mu_d - c * R @ mu_s


_gt = None


def gt():
    global _gt
    if _gt is None:
        g = dict(np.load(REC / "imu_gt_spl_cal1022" / "gt.npz", allow_pickle=True))
        ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
        dt = int(json.load(open(REC / "imu_gt" / "align.json"))["dt_ns"])
        _gt = (g["t_ns"][ok], g["p_bc"][ok], dt)
    return _gt


def accuracy(poses):
    tr = [m for m in poses if m.get("status") == "tracking" and m.get("T_world_cam0")]
    if len(tr) < 50:
        return dict(error="too few tracked")
    gt_t, gt_p, dt = gt()
    vt = np.array([m["t_ns"] for m in tr]) + dt
    P = np.array([np.asarray(m["T_world_cam0"]).reshape(4, 4)[:3, 3] for m in tr])
    j = np.clip(np.searchsorted(gt_t, vt), 1, len(gt_t) - 1)
    j = np.where(np.abs(gt_t[j - 1] - vt) < np.abs(gt_t[j] - vt), j - 1, j)
    m = np.abs(gt_t[j] - vt) < 2_000_000
    if m.sum() < 50:
        return dict(error="too few matched")
    A, B = P[m], gt_p[j[m]]
    R, _, t = umeyama(A, B)
    e = np.linalg.norm(A @ R.T + t - B, axis=1)
    Rs, c, ts = umeyama(A, B, True)
    es = np.linalg.norm(c * A @ Rs.T + ts - B, axis=1)
    path = float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum())
    return dict(matched=int(m.sum()), tracked=len(tr), ate_cm=round(float(np.sqrt((e ** 2).mean())) * 100, 2),
                ate_sim3_cm=round(float(np.sqrt((es ** 2).mean())) * 100, 2), scale=round(float(c), 4),
                path_m=round(path, 2), max_err_cm=round(float(e.max()) * 100, 1))


def run(d):
    d = Path(d)
    rows = list(csv.DictReader(open(d / "tl.csv" if (d / "tl.csv").exists() else d / "bridge_timing.csv")))
    f = lambda k: np.array([float(r[k]) for r in rows])
    t, rm, fe, init, ns, nm = f("t"), f("remap_ms"), f("feed_ms"), f("init"), f("n_slam"), f("n_msckf")
    proc = rm + fe
    tr = init > 0
    out = dict(run=d.name, frames=len(rows), frames_tracking=int(tr.sum()),
               t_init_s=round(float(t[tr][0] - t[0]), 2) if tr.any() else None)
    for name, msk in (("all", np.ones(len(rows), bool)), ("trk", tr)):
        p = proc[msk]
        out[name] = dict(proc_p50=pc(p, 50), proc_p95=pc(p, 95), proc_max=round(float(p.max()), 1) if len(p) else None,
                         over33_pct=round(float((p > 33.3).mean() * 100), 2) if len(p) else None,
                         remap_p50=pc(rm[msk], 50), feed_p50=pc(fe[msk], 50), feed_p95=pc(fe[msk], 95),
                         wait_p50=pc(f("wait_ms")[msk], 50), wait_p95=pc(f("wait_ms")[msk], 95))
    out["n_slam_med"] = float(np.median(ns[tr])) if tr.any() else None
    out["n_slam_max"] = int(ns.max())
    out["n_msckf_med"] = float(np.median(nm[tr])) if tr.any() else None
    out["cam_dropped"] = int(f("cam_dropped")[-1])
    # lag behind real time: (bridge receive time - camera time) minus its minimum (= the on-time offset), + this
    # frame's remap + wait + feed  ->  how late the pose of each frame comes out vs the earliest possible (ms)
    if "rx_boot_ns" in rows[0]:
        tn = np.array([int(r["t_ns"]) for r in rows], dtype=np.int64)
        rx = np.array([int(r["rx_boot_ns"]) for r in rows], dtype=np.int64)
        speed = 1.0                                   # feeder speed factor (run_all2 run.txt), stream time / speed
        if (d / "run.txt").exists():
            kv = dict(x.split("=", 1) for x in open(d / "run.txt").read().split())
            speed = float(kv.get("speed", 1.0))
        out["speed"] = speed
        lag = ((rx - rx[0]) - (tn - tn[0]) / speed) / 1e6
        lag = lag - lag.min()
        e2e = lag + rm + f("wait_ms") + fe
        out["late_ms"] = dict(p50=pc(e2e[tr], 50), p95=pc(e2e[tr], 95), max=round(float(e2e.max()), 1),
                              last=round(float(e2e[-1]), 1))
    # n_slam over time (5 s windows from init)
    if tr.any():
        t0 = t[tr][0]
        out["n_slam_5s"] = [[int(k * 5), float(np.median(ns[tr & (t >= t0 + 5 * k) & (t < t0 + 5 * k + 5)]))]
                            for k in range(int((t[-1] - t0) // 5) + 1)
                            if (tr & (t >= t0 + 5 * k) & (t < t0 + 5 * k + 5)).any()]
        out["proc_p50_5s"] = [[int(k * 5), pc(proc[tr & (t >= t0 + 5 * k) & (t < t0 + 5 * k + 5)], 50)]
                              for k in range(int((t[-1] - t0) // 5) + 1)
                              if (tr & (t >= t0 + 5 * k) & (t < t0 + 5 * k + 5)).any()]
    # OpenVINS stages
    ot = d / "ov_timing.txt"
    if ot.exists():
        O = np.array([[float(v) for v in l.split(",")] for l in open(ot) if l[0] != "#" and l.strip()])
        if len(O):
            st = {}
            for i, k in enumerate(STAGES):
                v = O[:, 1 + i] * 1000
                st[k] = dict(p50=pc(v, 50), p95=pc(v, 95), mean=round(float(v.mean()), 2))
            out["ov_rows"] = len(O)
            out["ov_stage_ms"] = st
            # join to tl rows by time -> stage cost vs n_slam (and proc vs n_slam)
            jj = np.clip(np.searchsorted(t, O[:, 0]), 1, len(t) - 1)
            jj = np.where(np.abs(t[jj - 1] - O[:, 0]) < np.abs(t[jj] - O[:, 0]), jj - 1, jj)
            okj = np.abs(t[jj] - O[:, 0]) < 0.004
            out["ov_join_frac"] = round(float(okj.mean()), 3)
            bins = []
            for lo, hi in BINS:
                b = okj & (ns[jj] >= lo) & (ns[jj] <= hi)
                if b.sum() < 5:
                    continue
                e = dict(n_slam=f"{lo}-{hi}", n=int(b.sum()), proc_p50=pc(proc[jj[b]], 50), proc_p95=pc(proc[jj[b]], 95),
                         over33_pct=round(float((proc[jj[b]] > 33.3).mean() * 100), 1))
                for i, k in enumerate(STAGES):
                    e[k] = pc(O[b, 1 + i] * 1000, 50)
                bins.append(e)
            out["by_n_slam"] = bins
    if (d / "po.jsonl").exists():
        poses = [json.loads(l) for l in open(d / "po.jsonl") if '"pose"' in l]
        out["acc"] = accuracy(poses)
    else:
        out["acc"] = {}
    json.dump(out, open(d / "metrics.json", "w"), indent=1)
    a, s = out["trk"], out.get("ov_stage_ms", {})
    g = lambda k: s.get(k, {}).get("p50")
    print(f"{d.name:28s} init {out['t_init_s']}s trk {out['frames_tracking']} | proc p50 {a['proc_p50']} p95 {a['proc_p95']} "
          f"max {a['proc_max']} >33 {a['over33_pct']}% | remap {a['remap_p50']} | trk {g('tracking')} msckf {g('msckf update')} "
          f"slam {g('slam update')} dly {g('slam delayed')} marg {g('re-tri & marg')} tot {g('total')} | nslam med "
          f"{out['n_slam_med']} max {out['n_slam_max']} | ATE {out['acc'].get('ate_cm')} s {out['acc'].get('scale')}")
    return out


if __name__ == "__main__":
    for d in sys.argv[1:]:
        run(d)
