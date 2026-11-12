#!/usr/bin/env python3
"""phonevio 11-12: phone offline VIO poses vs the laptop/PC lp_replay poses of the same recording.
  cmp_phonevio.py REC PHONE_OUT_DIR PC_PREFIX OUT_JSON
ATE after SE3 alignment (Umeyama, no scale) of matched tracking positions (same t_ns), Sim3 scale ratio phone/PC,
rotation difference after orientation alignment, init time, loop closures, counts. Also lp_score.score of both (same scorer)."""
import gzip, json, sys, time
from pathlib import Path
import numpy as np
sys.path.insert(0, "/home/system/scanner/StereoDepthSeg-fixes/scan3d/runs/livepose_20261030")
sys.path.insert(0, "/home/system/scanner/StereoDepthSeg-fixes/scan3d/imu_learn"); sys.path.insert(0, "/home/system/scanner/StereoDepthSeg-fixes/scan3d")
import lp_score

def load(p):
    with gzip.open(p, "rt") as f:
        return [json.loads(l) for l in f if l.strip()]

def T(m, key):
    d = m if key == "vio" else m.get("map") or {}
    v = d.get("T_world_cam0")
    return None if v is None else np.array(v, float).reshape(4, 4)

def umeyama(P, Q, scale=False):
    mp, mq = P.mean(0), Q.mean(0)
    X, Y = P - mp, Q - mq
    U, s, Vt = np.linalg.svd(Y.T @ X / len(P))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    c = (s * np.diag(D)).sum() / (X ** 2).sum(1).mean() if scale else 1.0
    return c, R, mq - c * R @ mp

def loops(p):
    n = acc = 0
    if Path(p).exists():
        for l in open(p):
            try:
                m = json.loads(l)
            except Exception:
                continue
            n += m.get("type") == "kf"; acc += int(m.get("acc", 0) or 0)
    return dict(kf=n, accepted=acc)

rec, pdir, pcp, outj = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3], Path(sys.argv[4])
ph, pc = load(pdir / "poses.jsonl.gz"), load(pcp + ".poses.jsonl.gz")
run = json.load(open(pdir / "run.json")); pcj = json.load(open(pcp + ".json"))
res = dict(made=time.strftime("%F %T"), rec=rec.name, phone_dir=str(pdir), pc_prefix=pcp, phone_run=run.get("status"))
res["score_phone"] = lp_score.score(rec, ph, 0, run["t_first_fed"], run["t_last_fed"], gt="auto")
res["score_pc"] = pcj["score"]
res["loops_phone"], res["loops_pc"] = loops(pdir / "loop.jsonl"), loops(pcp + ".loop.jsonl")
trp = {int(m["t_ns"]): m for m in ph if m.get("status") == "tracking" and m.get("T_world_cam0")}
trc = {int(m["t_ns"]): m for m in pc if m.get("status") == "tracking" and m.get("T_world_cam0")}
common = sorted(set(trp) & set(trc))
res.update(poses_phone=len(ph), poses_pc=len(pc), tracking_phone=len(trp), tracking_pc=len(trc), matched=len(common),
           same_t_ns_all=sorted(int(m["t_ns"]) for m in ph) == sorted(int(m["t_ns"]) for m in pc))
for key in ("vio", "map"):
    A = [T(trp[t], key) for t in common]; B = [T(trc[t], key) for t in common]
    if not common or any(x is None for x in A + B):
        continue
    A, B = np.array(A), np.array(B)
    P, Q = A[:, :3, 3], B[:, :3, 3]
    _, R, t = umeyama(P, Q)
    e = np.linalg.norm(P @ R.T + t - Q, axis=1)
    c, R2, t2 = umeyama(P, Q, scale=True)
    e2 = np.linalg.norm(c * P @ R2.T + t2 - Q, axis=1)
    Ra = np.einsum("ij,njk->nik", R, A[:, :3, :3])
    ang = np.degrees(np.arccos(np.clip((np.einsum("nij,nij->n", Ra, B[:, :3, :3]) - 1) / 2, -1, 1)))
    raw = np.linalg.norm(P - Q, axis=1)
    path = float(np.linalg.norm(np.diff(Q, axis=0), axis=1).sum())
    res[key] = dict(ate_se3_cm=round(float(np.sqrt((e ** 2).mean())) * 100, 3), ate_se3_max_cm=round(float(e.max()) * 100, 3),
                    ate_sim3_cm=round(float(np.sqrt((e2 ** 2).mean())) * 100, 3), scale_phone_over_pc=round(1 / c, 5),
                    rot_med_deg=round(float(np.median(ang)), 4), rot_max_deg=round(float(ang.max()), 4),
                    raw_pos_diff_med_cm=round(float(np.median(raw)) * 100, 3), raw_pos_diff_max_cm=round(float(raw.max()) * 100, 3),
                    path_pc_m=round(path, 3))
res["timing"] = {k: run.get(k) for k in ("rec_len_s", "setup_s", "feed_s", "wall_s", "wall_over_rec", "decode_ms_per_frame",
                                         "convert_ms_per_frame", "write_ms_per_frame", "battery_c_start", "battery_c_end", "battery_c_max",
                                         "thermal_status_start", "thermal_status_end", "thermal_status_max", "decoder", "bridge_sha")}
res["timing"]["pc_wall_s"] = pcj.get("wall_s")
res["phone_ms_med"] = float(np.median([m["ms"] for m in ph if "ms" in m]))
res["pc_ms_med"] = float(np.median([m["ms"] for m in pc if "ms" in m]))
outj.write_text(json.dumps(res, indent=1))
print(json.dumps(res, indent=1))
