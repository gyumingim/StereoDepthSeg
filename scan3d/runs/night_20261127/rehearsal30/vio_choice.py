#!/usr/bin/env python3
"""Pick the VIO config for 30 fps recordings (night 11-27). CPU. Rule written BEFORE looking at the arms (23:18):
 arms on the 30 fps decimated 121801: c4f60 (121801 config, track 60 Hz), c4 (same, track 30 Hz = single variable), fps30 (640x360 base variant,
 track 30 Hz, 11 clones, histogram on = 4 variables). Reference = the official 60 fps VIO of 121801 (NOT ground truth; consistency only).
 Pick = tracked share >= 0.92 AND lowest Sim3 ATE (median) vs the 60 fps reference; if within 0.5 cm of the best, prefer c4 then c4f60
 (fewest changes from the proven 60 fps config). Also report keyframes and stage-A plan numbers per arm.
"""
import gzip, json, time
from pathlib import Path

import numpy as np

D = Path(__file__).resolve().parent
REF = Path("/home/system/scanner/StereoDepthSeg-fixes/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan/vio/B_o0.poses.jsonl.gz")


def load(p):
    t, P, st = [], [], []
    for line in gzip.open(p, "rt"):
        d = json.loads(line)
        if d.get("type") == "pose":
            st.append(d.get("status"))
            if d.get("status") == "tracking" and d.get("T_world_cam0"):
                t.append(int(d["t_ns"])); P.append(np.array(d["T_world_cam0"], float).reshape(4, 4)[:3, 3])
    return np.array(t), np.array(P), np.mean(np.array(st) == "tracking")


def umeyama(A, B):   # B ~ s R A + t
    ma, mb = A.mean(0), B.mean(0); X, Y = A - ma, B - mb
    U, S, Vt = np.linalg.svd(Y.T @ X / len(A)); Dg = np.eye(3); Dg[2, 2] = np.sign(np.linalg.det(U @ Vt))
    R = U @ Dg @ Vt; s = np.trace(np.diag(S) @ Dg) / X.var(0).sum(); t = mb - s * R @ ma
    return s, R, t


def main():
    tr, Pr, _ = load(REF); idx = {t: i for i, t in enumerate(tr)}; res = dict(made=time.strftime("%F %T"), ref=str(REF), rule=__doc__, arms={})
    for arm in ("c4f60", "c4", "fps30"):
        v = D / arm / "scan/vio/B_o0.poses.jsonl.gz"
        if not v.exists():
            res["arms"][arm] = dict(missing=True); continue
        t, P, trk = load(v); m = [(i, idx[x]) for i, x in enumerate(t) if x in idx]
        a = np.array([P[i] for i, _ in m]); b = np.array([Pr[j] for _, j in m])
        s, R, tt = umeyama(a, b); e = np.linalg.norm((s * (R @ a.T)).T + tt - b, axis=1) * 100
        path = float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum())
        ca = json.loads((D / arm / "check_a.json").read_text()) if (D / arm / "check_a.json").exists() else {}
        res["arms"][arm] = dict(tracked_frac=round(float(trk), 3), n_common=len(m), sim3_scale=round(float(s), 4), ate_med_cm=round(float(np.median(e)), 2),
                                ate_rms_cm=round(float(np.sqrt(np.mean(e ** 2))), 2), ate_max_cm=round(float(e.max()), 2), path_m=round(path, 3),
                                n_kf=ca.get("kf_in_manifest"), strict_ge2_pct=ca.get("strict_ge2_pct"), strict_zero_pct=ca.get("strict_zero_pct"),
                                check_a_pass=ca.get("PASS"), warnings=ca.get("warnings"))
    ok = {k: v for k, v in res["arms"].items() if not v.get("missing") and v["tracked_frac"] >= 0.92}
    if ok:
        best = min(v["ate_med_cm"] for v in ok.values())
        for pref in ("c4", "c4f60", "fps30"):
            if pref in ok and ok[pref]["ate_med_cm"] <= best + 0.5:
                res["pick"] = pref; break
    else:
        res["pick"] = None
    (D / "vio_choice.json").write_text(json.dumps(res, indent=1)); print(json.dumps({k: v for k, v in res.items() if k != "rule"}, indent=1))


if __name__ == "__main__":
    main()
