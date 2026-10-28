#!/usr/bin/env python3
"""npunight 2026-10-29 scorer (README pre-registered metrics), same code for phone replay runs and the PC reference.
  score_rep.py phone <tag_dir> <rec_dir> <pc_json>     (phone run: <tag_dir>/run_*/poses_raw.jsonl, loop.jsonl, npu.jsonl, meta.json)
  score_rep.py pc <prefix> <rec_dir>                     (PC mkstream output: <prefix>.poses.jsonl, .loop.jsonl, .json)
GT = <rec>/imu_gt/gt.npz (ok & |p_bc| < 1.2, like vio_bench_hs.evaluate), VIO time + align_dt (PC json) -> nearest GT < 2 ms.
ATE = RMSE of camera positions after a rigid (rotation + translation, no scale) Umeyama fit, tracking poses only.
map ATE = same on pose['map']['T_world_cam0'] (loop-corrected) where present. End error = |mean of the last 1 s| after the fit.
Loops: 'loop' lines; correct = GT camera centres < 0.5 m apart and optical axes (R[:,2]) < 30 deg at t_cur / t_old.
Writes <tag_dir>/score.json (phone) or <prefix>.score.json (pc); prints one line."""
import glob
import json
import math
import os
import re
import sys
import time

import numpy as np


def umeyama_rmse(P, G):
    mp, mg = P.mean(0), G.mean(0)
    H = (P - mp).T @ (G - mg)
    U, _, Vt = np.linalg.svd(H)
    D = np.eye(3)
    D[2, 2] = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ D @ U.T
    Q = (R @ (P - mp).T).T + mg
    e = np.linalg.norm(Q - G, axis=1)
    return float(np.sqrt((e ** 2).mean())), Q, e


def load_gt(rec):
    g = dict(np.load(os.path.join(rec, "imu_gt", "gt.npz"), allow_pickle=True))
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    return g["t_ns"][ok].astype(np.int64), g["p_bc"][ok], g["R_bc"][ok]


def near(gt_t, vt):
    j = np.clip(np.searchsorted(gt_t, vt), 1, len(gt_t) - 1)
    j = np.where(np.abs(gt_t[j - 1] - vt) < np.abs(gt_t[j] - vt), j - 1, j)
    return j, np.abs(gt_t[j] - vt) < 2_000_000


def ate(poses, key, gt, dt_ns, off):
    gt_t, gt_p, _ = gt
    rows = []
    for m in poses:
        if m.get("status") != "tracking":
            continue
        T = m.get("T_world_cam0") if key == "vio" else (m.get("map") or {}).get("T_world_cam0")
        if not T:
            continue
        rows.append((int(m["t_ns"]) - off + dt_ns, np.asarray(T, float).reshape(4, 4)[:3, 3]))
    if len(rows) < 50:
        return dict(n=len(rows), error="too few poses")
    vt = np.array([r[0] for r in rows], np.int64)
    P = np.array([r[1] for r in rows])
    j, ok = near(gt_t, vt)
    if ok.sum() < 50:
        return dict(n=int(ok.sum()), error="too few matched")
    rmse, Q, e = umeyama_rmse(P[ok], gt_p[j[ok]])
    tv = vt[ok]
    last = tv >= tv[-1] - 1_000_000_000
    end = float(np.linalg.norm((Q[last] - gt_p[j[ok]][last]).mean(0)))
    return dict(n=int(ok.sum()), ate_cm=round(rmse * 100, 3), end_cm=round(end * 100, 3), max_cm=round(float(e.max()) * 100, 3))


def loops(lines, gt, dt_ns, off):
    gt_t, gt_p, gt_R = gt
    acc = [l for l in lines if l.get("type") == "loop"]
    res = []
    for l in acc:
        tc, to = int(l["t_cur_ns"]) - off + dt_ns, int(l["t_old_ns"]) - off + dt_ns
        (jc, oc), (jo, oo) = near(gt_t, np.array([tc])), near(gt_t, np.array([to]))
        if not (oc[0] and oo[0]):
            res.append(None)
            continue
        d = float(np.linalg.norm(gt_p[jc[0]] - gt_p[jo[0]]))
        a = float(np.degrees(np.arccos(np.clip(gt_R[jc[0]][:, 2] @ gt_R[jo[0]][:, 2], -1, 1))))
        res.append(dict(d_m=round(d, 3), ang_deg=round(a, 1), ok=d < 0.5 and a < 30))
    kf = [l for l in lines if l.get("type") == "kf"]
    xn = [c for l in kf for c in (l.get("cand") or [])]
    return dict(n_loop=len(acc), n_loop_gt=sum(1 for r in res if r), n_loop_correct=sum(1 for r in res if r and r["ok"]),
                n_kf=len(kf), n_cand=len(xn), n_opt=sum(1 for l in lines if l.get("type") == "opt"),
                xf_down=sum(1 for l in lines if "xf_down" in l), loops=res)


def pct(v, q):
    return round(float(np.percentile(v, q)), 3) if len(v) else None


def npu_stats(path):
    if not os.path.isfile(path):
        return None
    F, M1, M2 = [], [], []
    for s in open(path, errors="replace"):
        try:
            j = json.loads(s)
        except Exception:
            continue
        if j.get("op") == "F" and "us" in j:
            F.append(j["us"])
        elif j.get("op") == "M" and "us" in j:
            (M1 if j.get("mode") == 1 else M2).append(j)
    o = dict(n_F=len(F), n_M1=len(M1), n_M2=len(M2))
    for k in ("prep", "npu", "post_q", "tot", "recv"):
        v = [u[k] / 1e3 for u in F if k in u]
        o["F_%s_ms_p50" % k] = pct(v, 50)
        o["F_%s_ms_p95" % k] = pct(v, 95)
    for name, L in (("M1", M1), ("M2", M2)):
        for k in ("pack", "npu", "post", "tot"):
            v = [m["us"][k] / 1e3 for m in L if k in m["us"]]
            o["%s_%s_ms_p50" % (name, k)] = pct(v, 50)
        o["%s_n_p50" % name] = pct([m.get("n", 0) for m in L], 50)
    sk = [j for j in [json.loads(s) for s in open(path, errors="replace") if s.startswith("{")] if j.get("skin") is not None]
    o["skin_max"] = max([j["skin"] for j in sk], default=None)
    o["ap_max"] = max([j["ap"] for j in sk if j.get("ap") is not None], default=None)
    return o


def read_jsonl(p):
    out = []
    if os.path.isfile(p):
        for s in open(p, errors="replace"):
            s = s.strip()
            if s.startswith("{"):
                try:
                    out.append(json.loads(s))
                except Exception:
                    pass
    return out


mode = sys.argv[1]
if mode == "pc":
    pre, rec = sys.argv[2], sys.argv[3]
    pj = json.load(open(pre + ".json"))
    dt_ns = int(round(pj["align_dt_ms"] * 1e6))
    poses, ll, off, meta, run = read_jsonl(pre + ".poses.jsonl"), read_jsonl(pre + ".loop.jsonl"), 0, {}, pre
    outp = pre + ".score.json"
    be = pre + ".bridge.err"
else:
    tag, rec, pcj = sys.argv[2], sys.argv[3], sys.argv[4]
    dt_ns = int(round(json.load(open(pcj))["align_dt_ms"] * 1e6))
    runs = sorted(glob.glob(os.path.join(tag, "run_*")))
    if not runs:
        json.dump(dict(error="no run folder"), open(os.path.join(tag, "score.json"), "w"))
        print(tag, "no run folder")
        sys.exit(0)
    run = runs[-1]
    meta = json.load(open(os.path.join(run, "meta.json"))) if os.path.isfile(os.path.join(run, "meta.json")) else {}
    off = int(meta.get("replay_offset_ns") or 0)
    poses, ll = read_jsonl(os.path.join(run, "poses_raw.jsonl")), read_jsonl(os.path.join(run, "loop.jsonl"))
    outp = os.path.join(tag, "score.json")
    be = os.path.join(run, "bridge_stderr.log")
gt = load_gt(rec)
ms = [m["ms"] for m in poses if m.get("status") == "tracking" and "ms" in m]
r = dict(made=time.strftime("%F %T"), mode=mode, src=run, rec=os.path.basename(rec), dt_ns=dt_ns, offset_ns=off, poses=len(poses),
         tracking=sum(1 for m in poses if m.get("status") == "tracking"),
         vio=ate(poses, "vio", gt, dt_ns, off), map=ate(poses, "map", gt, dt_ns, off), loop=loops(ll, gt, dt_ns, off),
         vio_ms_p50=pct(ms, 50), vio_ms_p95=pct(ms, 95), vio_ms_max=pct(ms, 100))
if mode == "phone":
    r["npu"] = npu_stats(os.path.join(run, "npu.jsonl"))
    keys = ("app_version", "npu_loop", "run_valid", "npu_failed", "bridge_exit", "stop_reason", "replay_records", "replay_images",
            "replay_late_max_ms", "replay_late_over5ms", "npu_cpus", "secs", "battery_c_start", "battery_c_end")
    r["meta"] = {k: meta.get(k) for k in keys}
    txt = open(be, errors="replace").read() if os.path.isfile(be) else ""
    mm = re.search(r"keyframes pushed (\d+) dropped (\d+)", txt)
    r["kf_pushed_dropped"] = [int(mm.group(1)), int(mm.group(2))] if mm else None
    nrec = meta.get("replay_records") or 0
    tdir = os.path.dirname(run)
    valid = (bool(meta) and str(meta.get("bridge_exit")) == "0" and meta.get("stop_reason") == "replay end"
             and (meta.get("npu_loop") in (None, "") or meta.get("run_valid") is True) and r["loop"]["xf_down"] == 0
             and not os.path.exists(os.path.join(tdir, "SAFETY_STOP.txt")) and (meta.get("replay_late_over5ms") or 0) <= 0.01 * max(nrec, 1))
    r["valid"] = valid
json.dump(r, open(outp, "w"), indent=1)
print(json.dumps(dict(src=os.path.basename(run), valid=r.get("valid"), vio_ate=r["vio"].get("ate_cm"), map_ate=r["map"].get("ate_cm"),
                      map_end=r["map"].get("end_cm"), loops=r["loop"]["n_loop"], correct=r["loop"]["n_loop_correct"],
                      vio_ms_p50=r["vio_ms_p50"], vio_ms_p95=r["vio_ms_p95"])))
