#!/usr/bin/env python3
"""lgin 2026-10-29: loop-candidate level statistics for one PC replay (same GT / time alignment as npunight score_rep.py pc).
  cand_stats.py <prefix> <rec_dir>   -> <prefix>.cand.json, one summary line
Candidate = one entry of a kf line "cand" (BoW candidate that went to LighterGlue + PnP). GT revisit = GT camera centres < 0.5 m
and optical axes < 30 deg at the two keyframe times (= score_rep.py 'correct' rule). Pass = geometric verification ok (status "ok").
Accepted loop = 'loop' line (after the temporal-consistency step).
Relative pose error (RPE) of accepted loops: the loop's T_oldI_curI (q_oc, t_oc, IMU frame) is moved to the camera frame with the
c4f60 T_imu_cam (T_oC_cC = T_IC^-1 T_oI_cI T_IC) and compared with the GT T_oC_cC = T_bo^-1 T_bc (gt.npz R_bc, p_bc = camera in board
frame; ASSUMPTION: R_bc maps camera axes to board axes, as score_rep.py uses R[:,2] as the optical axis).
rot err = angle(R_gt^T R_loop) deg, trans err = |t_loop - t_gt| cm. Pre-registered convention check: if the median rot err of GT-correct
loops > 10 deg the RPE is marked 'convention_suspect'."""
import json
import math
import os
import sys
import time

import numpy as np

T_IC = np.array([[0.020201228802, -0.99977773579, -0.00603236055105, 0.0458238306231],
                 [-0.999795522195, -0.0201953741747, -0.00102988426744, 0.00581655355542],
                 [0.000907829582544, 0.00605193199493, -0.999981274807, -0.0113826293871],
                 [0, 0, 0, 1]])   # ~/scanner/tmp_lens05/new_gen/c4f60/kalibr_imucam_chain.yaml T_imu_cam


def qmat(q):  # x y z w
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rang(R):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))


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


def main():
    pre, rec = sys.argv[1], sys.argv[2]
    pj = json.load(open(pre + ".json"))
    dt_ns = int(round(pj["align_dt_ms"] * 1e6))
    g = dict(np.load(os.path.join(rec, "imu_gt", "gt.npz"), allow_pickle=True))
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    gt_t, gt_p, gt_R = g["t_ns"][ok].astype(np.int64), g["p_bc"][ok], g["R_bc"][ok]

    def gtat(t):
        t = t + dt_ns
        j = int(np.clip(np.searchsorted(gt_t, t), 1, len(gt_t) - 1))
        if abs(gt_t[j - 1] - t) < abs(gt_t[j] - t):
            j -= 1
        return j if abs(gt_t[j] - t) < 2_000_000 else None

    def rel(jc, jo):
        d = float(np.linalg.norm(gt_p[jc] - gt_p[jo]))
        a = math.degrees(math.acos(float(np.clip(gt_R[jc][:, 2] @ gt_R[jo][:, 2], -1, 1))))
        return d, a

    L = read_jsonl(pre + ".loop.jsonl")
    kfs = [l for l in L if l.get("type") == "kf"]
    t_of = {(l.get("session", 0), l["idx"]): int(l["t_ns"]) for l in kfs}
    path_of = {(l.get("session", 0), l["idx"]): float(l.get("path") or 0) for l in kfs}
    cands = []
    for l in kfs:
        tc = int(l["t_ns"])
        for c in l.get("cand") or []:
            oi, st, nm, ni = c[0], c[4], c[2], c[3]
            to = t_of.get((l.get("session", 0), oi))
            e = dict(cur=l["idx"], old=oi, status=st, nm=nm, ni=ni, xn=c[10] if len(c) > 10 else None,
                     dt_s=round((tc - to) / 1e9, 2) if to else None,
                     path_m=round(path_of[(l.get("session", 0), l["idx"])] - path_of.get((l.get("session", 0), oi), 0), 2))
            jc, jo = gtat(tc), (gtat(to) if to else None)
            if jc is not None and jo is not None:
                d, a = rel(jc, jo)
                e.update(gt_d=round(d, 3), gt_ang=round(a, 1), gt_rev=d < 0.5 and a < 30)
            cands.append(e)
    loops = []
    Tci = np.linalg.inv(T_IC)
    for l in L:
        if l.get("type") != "loop":
            continue
        tc, to = int(l["t_cur_ns"]), int(l["t_old_ns"])
        e = dict(cur=l["cur"], old=l["old"], n_inl=l.get("n_inl"), n_match=l.get("n_match"), path_between=l.get("path_between"),
                 dt_s=round((tc - to) / 1e9, 2))
        jc, jo = gtat(tc), gtat(to)
        if jc is not None and jo is not None:
            d, a = rel(jc, jo)
            T = np.eye(4)
            T[:3, :3] = qmat(l["q_oc"])
            T[:3, 3] = l["t_oc"]
            Tc = Tci @ T @ T_IC                       # T_oC_cC from the loop
            Ro, Rc, po, pc = gt_R[jo], gt_R[jc], gt_p[jo], gt_p[jc]
            Rg = Ro.T @ Rc
            tg = Ro.T @ (pc - po)
            e.update(gt_d=round(d, 3), gt_ang=round(a, 1), correct=d < 0.5 and a < 30,
                     rot_err_deg=round(rang(Rg.T @ Tc[:3, :3]), 3), trans_err_cm=round(float(np.linalg.norm(Tc[:3, 3] - tg)) * 100, 2),
                     gt_rel_t_cm=round(float(np.linalg.norm(tg)) * 100, 1), gt_rel_rot_deg=round(rang(Rg), 1))
        loops.append(e)

    def med(v):
        return round(float(np.median(v)), 3) if len(v) else None

    def mx(v):
        return round(float(np.max(v)), 3) if len(v) else None
    cg = [c for c in cands if "gt_rev" in c]
    pas = [c for c in cg if c["status"] == "ok"]
    lg = [l for l in loops if "correct" in l]
    lc = [l for l in lg if l["correct"]]
    rot = [l["rot_err_deg"] for l in lc]
    tr = [l["trans_err_cm"] for l in lc]
    xf = read_jsonl(pre + ".xf.jsonl")
    m2 = [x for x in xf if x.get("op") == "M" and x.get("mode") == 2]
    nv = [sum(x["nv"]) / 2 for x in m2 if x.get("nv")]
    r = dict(made=time.strftime("%F %T"), prefix=pre, rec=os.path.basename(rec), dt_ns=dt_ns,
             n_kf=len(kfs), n_kf_cand=sum(1 for l in kfs if l.get("cand")), n_cand=len(cands), n_cand_gt=len(cg),
             n_cand_gt_rev=sum(1 for c in cg if c["gt_rev"]), n_pass=sum(1 for c in cands if c["status"] == "ok"), n_pass_gt=len(pas),
             n_pass_gt_rev=sum(1 for c in pas if c["gt_rev"]),
             pass_precision=round(sum(1 for c in pas if c["gt_rev"]) / len(pas), 3) if pas else None,
             pass_recall=round(sum(1 for c in pas if c["gt_rev"]) / max(1, sum(1 for c in cg if c["gt_rev"])), 3) if cg else None,
             n_loop=len(loops), n_loop_gt=len(lg), n_loop_correct=len(lc), loop_correct_rate=round(len(lc) / len(lg), 3) if lg else None,
             rpe_rot_med_deg=med(rot), rpe_rot_max_deg=mx(rot), rpe_trans_med_cm=med(tr), rpe_trans_max_cm=mx(tr),
             convention_suspect=bool(rot) and float(np.median(rot)) > 10,
             loop_path_between_max_m=mx([l["path_between"] for l in loops if l.get("path_between") is not None]),
             loop_dt_max_s=mx([l["dt_s"] for l in loops]),
             xn_med_true=med([c["xn"] for c in cg if c["gt_rev"] and c["xn"] is not None]),
             xn_med_false=med([c["xn"] for c in cg if not c["gt_rev"] and c["xn"] is not None]),
             ni_med_true=med([c["ni"] for c in cg if c["gt_rev"]]), ni_med_false=med([c["ni"] for c in cg if not c["gt_rev"]]),
             lg_calls=len(m2), lg_nv_med=med(nv), lg_ms_med=med([x["ms"] for x in m2]),
             status_counts={s: sum(1 for c in cands if c["status"] == s) for s in sorted({c["status"] for c in cands})},
             cands=cands, loops=loops)
    json.dump(r, open(pre + ".cand.json", "w"), indent=1)
    print(json.dumps({k: r[k] for k in ("rec", "n_cand", "n_cand_gt_rev", "n_pass", "pass_precision", "n_loop", "n_loop_correct",
                                       "rpe_rot_med_deg", "rpe_trans_med_cm", "lg_nv_med")}))


if __name__ == "__main__":
    main()
