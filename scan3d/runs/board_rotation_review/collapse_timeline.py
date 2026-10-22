#!/usr/bin/env python3
"""133820 VIO collapse: per-5 s timeline of VIO log (ZUPT, init, features) vs GT (speed, distance, camera roll).
Read-only. Run from scan3d/: ../venv_ffs/bin/python runs/board_rotation_review/collapse_timeline.py [run ...]"""
import gzip, json, re, sys
from pathlib import Path
import numpy as np

OUT = Path("runs/board_rotation")
runs = sys.argv[1:] or ["133820_base_o0", "133820_base_o1", "133820_base_o2", "134204_base_o0", "134550_base_o1"]
res = {}
for n in runs:
    rec = "hs_20261022_" + n[:6]
    j = json.load(open(OUT / f"{n}.json"))
    ev = []   # (t, kind, val)
    t = None
    for line in gzip.open(OUT / f"{n}.stderr.log.gz", "rt", errors="replace"):
        m = re.search(r"\[DIAGT\] ([\d.]+)", line)
        if m:
            t = float(m.group(1)); continue
        if t is None:
            continue
        m = re.search(r"ZUPT\]: (passed|failed) disparity \(([\d.]+) [<>] [\d.]+, (\d+) features\)", line)
        if m:
            ev.append((t, "zdisp_" + m.group(1), float(m.group(2)), int(m.group(3))))
        m = re.search(r"ZUPT\]: (accepted|rejected) \|v_IinG\| = ([\d.]+)", line)
        if m:
            ev.append((t, "zupt_" + m.group(1), float(m.group(2)), 0))
        if "not enough feats" in line or "failed initialization" in line:
            ev.append((t, "initfail", 0, 0))
        m = re.search(r"SLAM update \((\d+) feats", line)
        if m:
            ev.append((t, "slamfeats", int(m.group(1)), 0))
    poses = [json.loads(x) for x in gzip.open(OUT / f"{n}.poses.jsonl.gz", "rt") if x.strip()]
    pt = np.array([p["t"] for p in poses])
    tr = [p for p in poses if p.get("T_world_cam0")]
    vt = np.array([p["t"] for p in tr]); vp = np.array([np.asarray(p["T_world_cam0"]).reshape(4, 4)[:3, 3] for p in tr])
    nms = np.array([p.get("n_msckf", 0) for p in tr]); nsl = np.array([p.get("n_slam", 0) for p in tr])
    # GT: distance, speed, camera roll (gravity in camera frame from board-frame u_g)
    g = np.load(Path("recordings") / rec / "imu_gt_spl" / "gt.npz", allow_pickle=True)
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    gt_t = g["t_ns"][ok]; gp = g["p_bc"][ok]; gR = g["R_bc"][ok]
    ug = np.array(json.load(open(OUT / "analysis.json"))["recs"][rec]["u_g_board"])
    gcam = np.einsum("nji,j->ni", gR, ug)          # R_bc^T u_g = gravity-up in camera frame
    roll = np.degrees(np.arctan2(gcam[:, 0], -gcam[:, 1]))   # 0 = camera -y up (landscape as held before)
    tilt = np.degrees(np.arccos(np.clip(-gcam[:, 2], -1, 1)))  # angle of optical axis from vertical-down... diagnostic
    # convert GT time to bench clip time: pose t_ns <-> t
    p0 = poses[0]; off_ns = p0["t_ns"] - p0["t"] * 1e9 + j["align_dt_ms"] * 1e6
    gtt = (gt_t - off_ns) / 1e9
    sp = np.r_[0, np.linalg.norm(np.diff(gp, axis=0), axis=1) / np.maximum(np.diff(gtt), 1e-3)]
    rows = []
    for a in range(0, int(pt.max()) + 5, 5):
        b = a + 5
        E = [e for e in ev if a <= e[0] < b]
        c = lambda k: sum(1 for e in E if e[1] == k)
        zf = [e[3] for e in E if e[1].startswith("zdisp")]
        sl = [e[2] for e in E if e[1] == "slamfeats"]
        mv = (vt >= a) & (vt < b); mg = (gtt >= a) & (gtt < b)
        rows.append(dict(t=a, zupt_acc=c("zupt_accepted"), zupt_rej=c("zupt_rejected"), initfail=c("initfail"),
                         zfeat_med=int(np.median(zf)) if zf else None,
                         msckf_med=int(np.median(nms[mv])) if mv.any() else None, slam_med=int(np.median(nsl[mv])) if mv.any() else None,
                         vio_dist_m=round(float(np.median(np.linalg.norm(vp[mv] - vp[0], axis=1))), 3) if mv.any() else None,
                         gt_dist_m=round(float(np.median(np.linalg.norm(gp[mg], axis=1))), 3) if mg.any() else None,
                         gt_speed_cms=round(float(np.median(sp[mg])) * 100, 1) if mg.any() else None,
                         roll_deg=round(float(np.median(roll[mg])), 1) if mg.any() else None,
                         gt_ok_frac=round(float(mg.sum()) / 240 / 5, 2)))
    res[n] = dict(t_init=j["t_init_clip_s"], rows=rows)
    print("==", n, "init", j["t_init_clip_s"])
    print(" t   zAcc zRej initF zFeat msckf slam vioD  gtD  gtV  roll gtOK")
    for r in rows:
        print(" %-3d %4d %4d %5d %5s %5s %4s %6s %5s %5s %6s %4s" % (r["t"], r["zupt_acc"], r["zupt_rej"], r["initfail"], r["zfeat_med"],
              r["msckf_med"], r["slam_med"], r["vio_dist_m"], r["gt_dist_m"], r["gt_speed_cms"], r["roll_deg"], r["gt_ok_frac"]))
json.dump(res, open("runs/board_rotation_review/collapse_timeline.json", "w"), indent=1)
