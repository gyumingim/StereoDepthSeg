#!/usr/bin/env python3
"""scanA 2026-11-05 step 2 metric: keyframe-pair relative pose error vs board GT (pre-registered, README 2).

  python scan3d/scanA/rel_eval.py REC --kf-step 10 --pose NAME=POSES.jsonl.gz [--pose ...] --out OUT.json

Pair set (fixed once from GT, same for every pose source): keyframes i, j (every kf-step-th frame) with GT ok in both and
GT baseline 4-30 cm, and a pose in EVERY given source (common set). Error of T_ci_cj = inv(T_w_ci) T_w_cj:
translation |t - t_gt| (mm, camera-i frame), rotation angle (deg). Also |t| length error (scale) and direction error (deg).
Reference rows: 'gt_nospline' = imu_gt (no spline) vs imu_gt_spl on the same pairs (GT noise floor, reference only).
Absolute ATE (SE3, cm) of the keyframe poses vs GT for reference.
"""
import argparse, gzip, json, time
from pathlib import Path
import numpy as np


def load(p):
    d = {}
    for line in gzip.open(p, "rt"):
        m = json.loads(line)
        if m.get("type") == "pose" and m.get("T_world_cam0") and m.get("status") == "tracking":
            d[int(m["t_ns"])] = np.array(m["T_world_cam0"], float).reshape(4, 4)
    return d


def gt_T(g):
    T = np.zeros((len(g["ok"]), 4, 4)); T[:, :3, :3] = g["R_bc"]; T[:, :3, 3] = g["p_bc"]; T[:, 3, 3] = 1
    return T


def stats(x):
    x = np.asarray(x, float)
    return dict(n=int(len(x)), mean=float(x.mean()), median=float(np.median(x)), p90=float(np.percentile(x, 90)),
                p10=float(np.percentile(x, 10)), max=float(x.max()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec"); ap.add_argument("--kf-step", type=int, default=10)
    ap.add_argument("--gt", default="imu_gt_spl"); ap.add_argument("--pose", action="append", default=[])
    ap.add_argument("--bmin", type=float, default=0.04); ap.add_argument("--bmax", type=float, default=0.30)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    rec = Path(a.rec)
    g = np.load(rec / a.gt / "gt.npz"); t_ns = g["t_ns"]; ok = g["ok"]; Tg = gt_T(g)
    g2 = np.load(rec / "imu_gt" / "gt.npz")
    src = {}
    for kv in a.pose:
        n, p = kv.split("=", 1)
        d = load(p)
        tt = np.array(sorted(d)); src[n] = {}
        for i in range(0, len(t_ns), a.kf_step):
            k = np.searchsorted(tt, t_ns[i]).clip(1, len(tt) - 1)
            k = k - 1 if abs(tt[k - 1] - t_ns[i]) < abs(tt[k] - t_ns[i]) else k
            if abs(tt[k] - t_ns[i]) < 4e6:
                src[n][i] = d[tt[k]]
    # GT no-spline: same frame index (assumes same t_ns grid; checked)
    T2 = gt_T(g2); same_grid = bool(len(g2["t_ns"]) == len(t_ns) and np.all(np.abs(g2["t_ns"] - t_ns) < 1e6))
    kf = [i for i in range(0, len(t_ns), a.kf_step) if ok[i] and all(i in s for s in src.values())]
    pairs = []
    for x, i in enumerate(kf):
        for j in kf[x + 1:]:
            if j - i > 300:
                break
            B = np.linalg.norm(Tg[i][:3, 3] - Tg[j][:3, 3])
            if a.bmin <= B <= a.bmax:
                pairs.append((i, j))
    res = dict(made=time.strftime("%F %T"), rec=str(rec.resolve()), gt=a.gt, args=vars(a), n_kf_common=len(kf),
               n_pairs=len(pairs), sources={})

    def rel(T, i, j):
        return np.linalg.inv(T[i]) @ T[j]

    def err(Ts, use_g2=False):
        et, er, el, ed = [], [], [], []
        for i, j in pairs:
            if use_g2 and not (g2["ok"][i] and g2["ok"][j]):
                continue
            Rg = rel(Tg, i, j)
            Re = rel(T2, i, j) if use_g2 else rel(Ts, i, j)
            et.append(np.linalg.norm(Re[:3, 3] - Rg[:3, 3]) * 1000)
            dR = Re[:3, :3].T @ Rg[:3, :3]
            er.append(np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))))
            el.append((np.linalg.norm(Re[:3, 3]) - np.linalg.norm(Rg[:3, 3])) * 1000)
            ed.append(np.degrees(np.arccos(np.clip(Re[:3, 3] @ Rg[:3, 3] / np.linalg.norm(Re[:3, 3]) / np.linalg.norm(Rg[:3, 3]), -1, 1))))
        return dict(trans_mm=stats(et), rot_deg=stats(er), len_err_mm=stats(el), abs_len_err_mm=stats(np.abs(el)),
                    dir_deg=stats(ed))
    for n, s in src.items():
        r = err(s)
        A = np.array([s[i][:3, 3] for i in kf]); Bp = np.array([Tg[i][:3, 3] for i in kf])
        ma, mb = A.mean(0), Bp.mean(0)
        U, S_, Vt = np.linalg.svd((Bp - mb).T @ (A - ma)); D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
        Ra = U @ D @ Vt
        e = np.linalg.norm((Ra @ (A - ma).T).T + mb - Bp, axis=1)
        r["ate_kf_se3_cm"] = float(np.sqrt((e ** 2).mean()) * 100)
        res["sources"][n] = r
    if same_grid:
        res["sources"]["ref_gt_nospline"] = err(None, use_g2=True)
    else:
        res["gt_nospline_note"] = f"skipped (same_grid {same_grid})"
    Path(a.out).write_text(json.dumps(res, indent=1))
    print(f"pairs {len(pairs)} kf {len(kf)}")
    for n, r in res["sources"].items():
        print(f"{n:18s} trans mm mean {r['trans_mm']['mean']:.2f} med {r['trans_mm']['median']:.2f} p90 {r['trans_mm']['p90']:.2f} | "
              f"rot deg med {r['rot_deg']['median']:.3f} p90 {r['rot_deg']['p90']:.3f} | |len| med {r['abs_len_err_mm']['median']:.2f} | "
              f"dir med {r['dir_deg']['median']:.2f} | ATE {r.get('ate_kf_se3_cm', float('nan')):.2f} cm")


if __name__ == "__main__":
    main()
