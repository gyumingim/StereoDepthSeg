#!/usr/bin/env python3
"""ovrefine POST-HOC diagnostic (16:10, after the dev verdict; dev cells only; not a verdict): are the refined states in
the middle of the window (delayed output, smoothing) better than the newest one (what --refine publishes)?
From the R10w runs (= R10 + log_win=1): lag-L trajectory = for every snapshot s, the refined pose of s from the accepted
window in which s is L snapshots before the newest; frames after s use that correction (T = dT(s) T_ov) until the next
snapshot. L = 0 must reproduce the published R10 "ref" trajectory. Scored with the same ruler (cam_metrics, rpe1).
  python3 lag_diag.py   -> lag_diag.json (runs dir)
"""
import gzip
import importlib.util
import json
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("ovr", HERE / "ovrefine_run.py")
ovr = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ovr)
LAGS = (0, 3, 5, 9)


def quat_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_quat(R):
    w = np.sqrt(max(1e-12, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = (R[2, 1] - R[1, 2]) / (4 * w)
    y = (R[0, 2] - R[2, 0]) / (4 * w)
    z = (R[1, 0] - R[0, 1]) / (4 * w)
    return [x, y, z, w]


def lag_poses(P, wst, L):
    corr = {}  # snapshot t_ns -> (dR, dt)
    for w in wst:
        if not w["ok"]:
            continue
        s = w["s"]
        i = len(s) - 1 - L
        if i < 0:
            continue
        e = s[i]
        p_r, q_r, p_o, q_o = np.array(e[1:4]), e[4:8], np.array(e[8:11]), e[11:15]
        Rr, Ro = quat_R(q_r), quat_R(q_o)
        dR = Rr @ Ro.T
        corr[int(e[0])] = (dR, p_r - dR @ p_o)
    # every snapshot time (window newest t_ns) in order; a snapshot without an accepted lag-L value -> no correction
    snaps = sorted({int(w["t_ns"]) for w in wst})
    out, k, cur = [], 0, None
    for m in P:
        t = int(m.get("t_ns", -1))
        while k < len(snaps) and snaps[k] <= t:
            cur = corr.get(snaps[k])
            k += 1
        if m.get("ref") is not None and m.get("T_world_cam0") is not None:
            cur_use = cur  # (resets inside a run are not replayed: the L = 0 check shows their effect)
            m2 = dict(m)
            if cur_use is not None:
                dR, dt = cur_use
                R = dR @ quat_R(m["q"])
                p = dR @ np.array(m["p"]) + dt
                R_ItoC = np.array(m["R_ItoC0"]).reshape(3, 3)
                p_IinC = np.array(m["p_IinC0"])
                Rc = R @ R_ItoC.T
                T = np.eye(4)
                T[:3, :3] = Rc
                T[:3, 3] = p - Rc @ p_IinC
                m2.update(p=p.tolist(), q=R_quat(R), T_world_cam0=T.reshape(-1).tolist())
            m2.pop("ref", None)
            out.append(m2)
        else:
            m2 = dict(m)
            m2.pop("ref", None)
            out.append(m2)
    return out


def main():
    import lens_switch_gtscore as gs
    gs.OUT = ovr.OUT / "score_tmp_lag"
    gs.OUT.mkdir(parents=True, exist_ok=True)
    rows = []
    for r, o in ovr.cells("dev"):
        k = ovr.key_of(r, o, "dev")
        pre = ovr.OUT / "R10w" / f"{k}_R10w"
        bpre = ovr.OUT / "B60" / f"{k}_B60"
        if not Path(str(pre) + ".json").exists():
            continue
        P = ovr.load_P(pre)
        wst = [json.loads(x) for x in open(str(pre) + ".refine.jsonl") if '"wst"' in x]
        row = dict(key=k, rec=r, n_wst=len(wst))
        mb = gs.cam_metrics(bpre, r, ovr.GT_HS)
        row["B60"] = dict(ate_w=mb.get("ate_w"), div=bool(mb.get("divergent")), rpe1=ovr.rpe1(bpre, r, ovr.GT_HS))
        # published ref (from the run itself) for the L = 0 check
        dref, _ = ovr.write_ref(pre, ovr.OUT / "R10w_ref")
        mr = gs.cam_metrics(dref, r, ovr.GT_HS)
        row["ref"] = dict(ate_w=mr.get("ate_w"), rpe1=ovr.rpe1(dref, r, ovr.GT_HS))
        Pref = ovr.load_P(dref)
        for L in LAGS:
            Q = lag_poses(P, wst, L)
            if L == 0:  # max camera-position difference vs the published ref
                d = [np.abs(np.asarray(a["T_world_cam0"]).reshape(4, 4)[:3, 3] - np.asarray(b["T_world_cam0"]).reshape(4, 4)[:3, 3]).max()
                     for a, b in zip(Q, Pref) if a.get("T_world_cam0") is not None and b.get("T_world_cam0") is not None]
                row["lag0_vs_ref_max_m"] = float(max(d)) if d else None
            dd = ovr.OUT / f"R10w_lag{L}"
            dd.mkdir(parents=True, exist_ok=True)
            dst = dd / pre.name
            json.dump(json.load(open(str(pre) + ".json")), open(str(dst) + ".json", "w"))
            with gzip.open(str(dst) + ".poses.jsonl.gz", "wt") as f:
                for m in Q:
                    f.write(json.dumps(m) + "\n")
            mk = gs.cam_metrics(dst, r, ovr.GT_HS)
            row[f"lag{L}"] = dict(ate_w=mk.get("ate_w"), div=bool(mk.get("divergent")), rpe1=ovr.rpe1(dst, r, ovr.GT_HS))
        rows.append(row)
        print(k, json.dumps({x: row[x] for x in row if x not in ("rec",)}), flush=True)
    summ = {}
    for name in ["ref"] + [f"lag{L}" for L in LAGS]:
        for met in ("ate_w", "rpe1"):
            d = [x[name][met] - x["B60"][met] for x in rows if x[name].get(met) is not None and x["B60"].get(met) is not None]
            recs = sorted({x["rec"] for x in rows})
            per = [np.mean([x[name][met] - x["B60"][met] for x in rows if x["rec"] == rr]) for rr in recs]
            summ[f"{name}.{met}"] = dict(n=len(d), d_med=round(float(np.median(d)), 4), better=int(sum(v < 0 for v in d)),
                                         rec_better=f"{sum(v < 0 for v in per)}/{len(per)}")
    for k, v in summ.items():
        print(k, v)
    res = dict(made=time.strftime("%F %T"), doc=__doc__, plan_commit=ovr.git("log", "-1", "--format=%H", "--", "runs/ovrefine_20261027/PLAN.md"),
               bridge=json.load(open(ovr.OUT / "bin" / "bridge.json")), summary=summ, rows=rows)
    json.dump(res, open(ovr.RD / "lag_diag.json", "w"), indent=1)
    print("saved", ovr.RD / "lag_diag.json")


if __name__ == "__main__":
    main()
