#!/usr/bin/env python3
"""On-phone OpenVINS timing report for arlat 0.6.7 ov runs (files/ov/run_<time>_.../, pulled to the PC).

  venv_ffs/bin/python scan3d/tools/ov_phone_timing.py RUN_DIR [RUN_DIR ...] [--seg still0:0:8,hand:8:68,still:68:128]
                                                     [--budget-ms 16.7] [--all-frames]

Per run (docs/phone_session_60fps_plan.md "On-phone OpenVINS timing"):
  per-frame processing = remap_ms + feed_ms from bridge_timing.csv (ov_bridge --timing-log; feed = OpenVINS
    feed_measurement_camera = track + propagate + update + marginalise): p50 / p95 / p99 / max and the fraction of
    frames over the budget (16.7 ms = 60 fps) and over 33.3 ms (30 fps)
  OpenVINS split from ov_timing.txt (record_timing_information, seconds -> ms): tracking, propagation, msckf update,
    slam update, slam delayed, re-tri & marg, total
  wait_ms (record received -> fed: IMU wait + backlog), end-to-end latency t_out - t_sensor (poses.csv)
  arlat 0.6.17 --ei predict 1|2 (poses_pred.csv, IMU forward-predicted poses): latency of the predicted pose
    t_out - t_pred (t_pred = predicted instant = newest IMU sample, IMU/sensor clock; assumes IMU and camera timestamps
    share elapsedRealtimeNanos) and the horizon t_pred - t_cam (t_cam = mid-exposure frame time it starts from), per src
    (frame = once per frame update, rate = between frames, mode 2); tracking lines only
  drops: camera gaps in frames.csv (sensor timestamps > 1.5 periods apart -> frames the camera / ImageReader skipped),
    Java pool / queue drops (frames.csv fate 1 / 2), bridge queue drops (bridge_timing.csv cam_dropped), and fed frames
    per second vs the requested fps
  temperatures: battery max, thermal status max (temps.csv); SKIN comes from the PC thermal log (thermal_*.txt) if present
By default only frames after OpenVINS initialised (init = 1) count (--all-frames: every fed frame). Segments are seconds
from the first fed frame. Writes RUN_DIR/timing_report.json (made time + inputs) and prints a table.
"""
import argparse
import csv
import json
import re
import sys
import time
from pathlib import Path

import numpy as np

OV_COLS = ["tracking", "propagation", "msckf update", "slam update", "slam delayed", "re-tri & marg", "total"]


def read_csv(p):
    if not p.exists():
        return None
    with open(p) as f:
        rows = list(csv.DictReader(f))
    return rows


def stats(x, budget=None):
    x = np.asarray([v for v in x if np.isfinite(v)], float)
    if len(x) == 0:
        return dict(n=0)
    d = dict(n=int(len(x)), p50=round(float(np.percentile(x, 50)), 3), p95=round(float(np.percentile(x, 95)), 3),
             p99=round(float(np.percentile(x, 99)), 3), max=round(float(x.max()), 3), mean=round(float(x.mean()), 3))
    if budget is not None:
        d["frac_over_budget"] = round(float((x > budget).mean()), 4)
        d["frac_over_33ms"] = round(float((x > 33.3).mean()), 4)
    return d


def read_ov_timing(p):
    """ov_timing.txt: '# timestamp (sec),tracking,...' then rows; seconds. -> {col: array ms}, timestamps (s, bridge t)."""
    if not p.exists():
        return None, None
    hdr, rows = None, []
    for line in open(p):
        line = line.strip()
        if not line:
            continue
        if line.startswith("#"):
            hdr = [h.strip() for h in line[1:].split(",")]
            continue
        try:
            rows.append([float(v) for v in line.split(",")])
        except ValueError:
            continue
    if not rows or hdr is None:
        return None, None
    a = np.array([r for r in rows if len(r) == len(hdr)])
    out = {h: a[:, i] * 1e3 for i, h in enumerate(hdr) if i > 0}
    return out, a[:, 0]


def camera_gaps(t_ns, fps):
    t = np.sort(np.asarray(t_ns, np.int64))
    if len(t) < 2:
        return dict(n=len(t))
    period = 1e9 / fps
    d = np.diff(t)
    miss = np.clip(np.round(d / period) - 1, 0, None)
    return dict(n=int(len(t)), secs=round(float((t[-1] - t[0]) / 1e9), 2), fps_measured=round(float((len(t) - 1) / ((t[-1] - t[0]) / 1e9)), 2),
                gaps=int((d > 1.5 * period).sum()), missing_est=int(miss.sum()), max_gap_ms=round(float(d.max() / 1e6), 2),
                median_period_ms=round(float(np.median(d) / 1e6), 3))


def skin_log(run):
    """Optional PC thermal poll (thermal_*.txt next to the run, lines '<unix> SKIN <C> ...'): max SKIN."""
    vals = []
    for p in list(run.glob("thermal*.txt")) + list(run.parent.glob(run.name + "_thermal*.txt")):
        for line in open(p, errors="ignore"):
            m = re.search(r"SKIN\D+([0-9]+\.?[0-9]*)", line)
            if m:
                vals.append(float(m.group(1)))
    return max(vals) if vals else None


def segments(spec, t_end):
    if not spec:
        return [("all", 0.0, t_end + 1)]
    out = [("all", 0.0, t_end + 1)]
    for s in spec.split(","):
        name, a, b = s.split(":")
        out.append((name, float(a), float(b)))
    return out


def report(run, a):
    meta = json.load(open(run / "meta.json")) if (run / "meta.json").exists() else {}
    fps = float(meta.get("fps") or a.fps)
    bt = read_csv(run / "bridge_timing.csv")
    if not bt:
        sys.exit(f"{run}: no bridge_timing.csv (ov_bridge --timing-log)")
    t_ns = np.array([int(r["t_ns"]) for r in bt], np.int64)
    t_rel = (t_ns - t_ns[0]) / 1e9
    remap = np.array([float(r["remap_ms"]) for r in bt])
    feed = np.array([float(r["feed_ms"]) for r in bt])
    wait = np.array([float(r["wait_ms"]) for r in bt])
    init = np.array([int(r["init"]) for r in bt]).astype(bool)
    proc = remap + feed
    ov, ov_t = read_ov_timing(run / "ov_timing.txt")
    fr = read_csv(run / "frames.csv")
    po = read_csv(run / "poses.csv")
    pp = read_csv(run / "poses_pred.csv")          # arlat 0.6.17 --ei predict
    tp = read_csv(run / "temps.csv")
    res = dict(run=str(run), made=time.strftime("%Y-%m-%d %H:%M:%S"), script="scan3d/tools/ov_phone_timing.py",
               inputs=[p.name for p in run.iterdir() if p.is_file()], budget_ms=a.budget_ms, fps=fps,
               only_initialised=not a.all_frames,
               meta={k: meta.get(k) for k in ("app_version", "cam", "size", "fps", "cfg", "preset", "imu", "exp_us", "iso",
                                               "focus_dpt", "yuv_min_frame_duration_ns", "first_result", "stop_reason",
                                               "secs", "bridge_exit", "device", "perf")},
               frames_fed=int(len(bt)), frames_initialised=int(init.sum()),
               t_init_s=round(float(t_rel[init][0]), 2) if init.any() else None)
    # arlat 0.6.14: ov_bridge "[ov_bridge] affinity <tag>: ..." lines (--cpus pinning, per-thread sched_getaffinity)
    se = run / "bridge_stderr.log"
    res["bridge_affinity"] = [l.strip() for l in open(se, errors="replace") if l.startswith("[ov_bridge] affinity")] \
        if se.exists() else []
    segs = {}
    t_end = float(t_rel[-1])
    for name, s0, s1 in segments(a.seg, t_end):
        m = (t_rel >= s0) & (t_rel < s1)
        if not a.all_frames:
            m &= init
        dur = float(t_rel[m][-1] - t_rel[m][0]) if m.sum() > 1 else 0.0
        sg = dict(t0=s0, t1=min(s1, t_end), frames=int(m.sum()),
                  fed_fps=round((m.sum() - 1) / dur, 2) if dur > 0 else None,
                  per_frame_ms=stats(proc[m], a.budget_ms), feed_ms=stats(feed[m]), remap_ms=stats(remap[m]),
                  wait_ms=stats(wait[m]),
                  bridge_cam_dropped=int(bt[int(np.nonzero(m)[0][-1])]["cam_dropped"]) - int(bt[int(np.nonzero(m)[0][0])]["cam_dropped"]) if m.any() else 0)
        if ov is not None:
            ot = ov_t - float(bt[0]["t"])                 # same bridge time base (t0 = first record) as bridge_timing t
            om = (ot >= s0) & (ot < s1)
            sg["openvins_ms"] = {c: stats(ov[c][om], a.budget_ms if c == "total" else None) for c in OV_COLS if c in ov}
        if po:
            tl = [(int(r["t_ns"]), (int(r["t_out_ns"]) - int(r["t_sensor_ns"])) / 1e6) for r in po if int(r["t_sensor_ns"]) > 0]
            if tl:
                tt = (np.array([x[0] for x in tl]) - t_ns[0]) / 1e9
                lat = np.array([x[1] for x in tl])
                sg["latency_sensor_to_pose_ms"] = stats(lat[(tt >= s0) & (tt < s1)])
        if pp:
            for src in ("frame", "rate"):
                rr = [r for r in pp if r["src"] == src and r["status"] == "tracking"]
                if not rr:
                    continue
                tc = (np.array([int(r["t_cam_ns"]) for r in rr]) - t_ns[0]) / 1e9
                mm = (tc >= s0) & (tc < s1)
                tpred = np.array([int(r["t_pred_ns"]) for r in rr], np.int64)
                lat = (np.array([int(r["t_out_ns"]) for r in rr], np.int64) - tpred) / 1e6
                hor = (tpred - np.array([int(r["t_cam_ns"]) for r in rr], np.int64)) / 1e6
                sg[f"pred_{src}"] = dict(latency_ms=stats(lat[mm]), horizon_ms=stats(hor[mm]),
                                         n_imu=stats([float(r["n_imu"]) for r, k in zip(rr, mm) if k]))
        segs[name] = sg
    res["segments"] = segs
    if fr:
        ts = [int(r["t_sensor_ns"]) for r in fr]
        t_first = int(t_ns[init][0]) if (init.any() and not a.all_frames) else -1   # drops over the judged frames only
        fate = np.array([int(r["fate"]) for r in fr if int(r["t_send_ns"]) >= t_first])
        res["camera"] = camera_gaps(ts, fps)
        res["java_pool_dropped"] = int((fate == 1).sum())
        res["java_queue_dropped"] = int((fate == 2).sum())
        res["java_copy_us"] = stats([float(r["copy_us"]) for r in fr if int(r["fate"]) == 0])
        res["frames_received"] = len(fr)
    if tp:
        res["battery_c_max"] = max(float(r["battery_c"]) for r in tp)
        res["thermal_status_max"] = max(int(r["thermal_status"]) for r in tp)
    res["skin_c_max_pc_log"] = skin_log(run)
    s = segs["all"]["per_frame_ms"]
    drops = res.get("java_pool_dropped", 0) + res.get("java_queue_dropped", 0) + segs["all"]["bridge_cam_dropped"]
    if s.get("n"):
        ok = s["p95"] <= a.budget_ms and drops == 0
        res["verdict"] = (f"{'REAL TIME' if ok else 'NOT real time'} at {a.budget_ms} ms: per-frame p95 {s['p95']} ms, "
                          f"{100 * s['frac_over_budget']:.1f} % over, drops {drops}, camera missing {res.get('camera', {}).get('missing_est')}")
    (run / "timing_report.json").write_text(json.dumps(res, indent=1))
    return res


def show(res):
    print(f"\n== {res['run']}")
    m = res["meta"]
    print(f"   {m.get('app_version')} cam {m.get('cam')} {m.get('size')}@{m.get('fps')} cfg {m.get('cfg')} preset {m.get('preset')} "
          f"imu {m.get('imu')} stop {m.get('stop_reason')}")
    if m.get("perf"):          # app 0.6.13+: thread priorities / hint session / sustained mode
        pf = m["perf"]
        print(f"   perf: on {pf.get('perf_on')} sustained supported {pf.get('sustained_mode_supported')} set {pf.get('sustained_mode_set')}; "
              f"hint {pf.get('hint')} reports {pf.get('hint_reports')}; bridge prio {pf.get('bridge_prio')}")
        print(f"   perf knobs: hint_ms {pf.get('hint_ms', '-')} (target_ns {pf.get('hint_target_ns')}) cpus {pf.get('cpus', '-')}")
        if "ov_threads" in pf or "bridge_cv_threads" in pf:   # arlat 0.6.15: OpenCV thread pool of the bridge
            print(f"   threads: ov_threads {pf.get('ov_threads', '-')} -> bridge cv_threads {pf.get('bridge_cv_threads', '-')}"
                  f" ({pf.get('bridge_cv_parallel', '?')}), klt_overlap {pf.get('bridge_klt_overlap', pf.get('klt_overlap', '-'))}"
                  f", idle_spin {pf.get('ov_idle_spin', '-')} -> bridge {pf.get('bridge_idle_spin', '-')}"
                  f" (stripes {pf.get('bridge_idle_spin_stripes', '-')})"      # arlat 0.6.16
                  f", predict {pf.get('ov_predict', '-')} -> bridge {pf.get('bridge_predict', '-')}")   # arlat 0.6.17
        if pf.get("bridge_affinity"):
            print(f"   bridge affinity (pose 30): {pf.get('bridge_affinity')}")
            print(f"   app affinity (pose 30): {pf.get('app_affinity')}")
    for l in res.get("bridge_affinity", []):
        print(f"   {l if len(l) <= 400 else l[:400] + ' ...'}")
    print(f"   fed {res['frames_fed']} frames, initialised {res['frames_initialised']} (init at {res['t_init_s']} s)")
    if "camera" in res:
        c = res["camera"]
        print(f"   camera: {c.get('fps_measured')} fps measured, missing {c.get('missing_est')} (gaps {c.get('gaps')}, max {c.get('max_gap_ms')} ms); "
              f"Java drops pool {res['java_pool_dropped']} queue {res['java_queue_dropped']}; copy p50 {res['java_copy_us'].get('p50')} us")
    print(f"   battery max {res.get('battery_c_max')} C, thermal status max {res.get('thermal_status_max')}, SKIN max (PC log) {res.get('skin_c_max_pc_log')}")
    print(f"   {'segment':10s} {'frames':>6s} {'fps':>6s} | per-frame ms p50 / p95 / max | >{res['budget_ms']} | >33.3 | track p50/p95 | update ms p50 | wait p95 | latency p50/p95")
    for name, s in res["segments"].items():
        p = s["per_frame_ms"]
        if not p.get("n"):
            print(f"   {name:10s} {s['frames']:6d}  (no frames)")
            continue
        ov = s.get("openvins_ms", {})
        tr = ov.get("tracking", {})
        up = {k: ov.get(k, {}) for k in ("msckf update", "slam update", "slam delayed")}
        upd = " / ".join(f"{v.get('p50', '-')}" for v in up.values())
        lat = s.get("latency_sensor_to_pose_ms", {})
        print(f"   {name:10s} {s['frames']:6d} {s['fed_fps'] or 0:6.1f} | {p['p50']:6.2f} / {p['p95']:6.2f} / {p['max']:7.2f} | "
              f"{100 * p['frac_over_budget']:5.1f} % | {100 * p['frac_over_33ms']:5.1f} % | {tr.get('p50', '-')}/{tr.get('p95', '-')} | "
              f"msckf/slam/delay p50 {upd} | {s['wait_ms'].get('p95')} | {lat.get('p50', '-')}/{lat.get('p95', '-')}")
    for name, s in res["segments"].items():   # arlat 0.6.17 --ei predict: IMU forward-predicted poses
        for src in ("frame", "rate"):
            pr = s.get(f"pred_{src}")
            if pr and pr["latency_ms"].get("n"):
                la, ho = pr["latency_ms"], pr["horizon_ms"]
                lp = s.get("latency_sensor_to_pose_ms", {})
                print(f"   predicted pose [{src}] {name:8s} n {la['n']}: latency t_out - t_pred p50 {la['p50']} / p95 {la['p95']} / max {la['max']} ms"
                      f" (pose: {lp.get('p50', '-')} / {lp.get('p95', '-')} / {lp.get('max', '-')}) | horizon t_pred - t_cam p50 {ho['p50']}"
                      f" / p95 {ho['p95']} / max {ho['max']} ms")
    print(f"   -> {res.get('verdict')}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--seg", default="", help="name:t0:t1,... seconds from the first fed frame (default: whole run)")
    ap.add_argument("--budget-ms", type=float, default=16.7)
    ap.add_argument("--fps", type=float, default=60.0, help="if meta.json is missing")
    ap.add_argument("--all-frames", action="store_true", help="also frames before OpenVINS initialised")
    a = ap.parse_args()
    for r in a.runs:
        show(report(Path(r), a))


if __name__ == "__main__":
    main()
