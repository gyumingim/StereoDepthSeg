#!/usr/bin/env python3
"""livepose scorer (rules pre-registered in README.md section 2). score(rec, poses, shift_ns, t_first_fed, t_last_fed).

Ruler = camera position (memory rule): VIO T_world_cam0 translation vs board GT p_bc (imu_gt_spl if present else imu_gt,
camera-only GT, ok & |p_bc| < 1.2 m, nearest GT < 2 ms), SE3 Umeyama (no scale) over all matched tracking poses.
Recordings without GT: start-end (loop) error = fi_limits_run.loop_eval rule (start = first tracked pose, end = mean of
+-10 poses at motion end + 0.5 s, path = every 6th pose).
"""
import csv, json, sys
from pathlib import Path
import numpy as np

DIVERGE_CM = 50.0


def umeyama(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((dst - mu_d).T @ (src - mu_s) / len(src))
    S = np.eye(3); S[2, 2] = np.sign(np.linalg.det(U @ Vt))
    R = U @ S @ Vt
    return R, mu_d - R @ mu_s


def pick_gt(rec):
    for n in ("imu_gt_spl", "imu_gt"):
        if (rec / n / "gt.npz").exists():
            return rec / n
    return None


def _T(m, key):
    v = (m.get("map") or {}).get("T_world_cam0") if key == "map" else m.get("T_world_cam0")
    return None if v is None else np.asarray(v, float).reshape(4, 4)


def path_len(P, step=6):
    q = P[::step]
    return float(np.linalg.norm(np.diff(q, axis=0), axis=1).sum()) if len(q) > 1 else 0.0


def drift_per_m(P, Pg, tt):
    """median over start poses (every 0.5 s) of |dP_vio - dP_gt| / D, D = GT path 1.0 m (0.5 m if total path < 2 m)."""
    cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(Pg, axis=0), axis=1))])
    # GT path on a 10 Hz subsample to keep board jitter out of the length
    sub = np.arange(0, len(Pg), 6)
    cs = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(Pg[sub], axis=0), axis=1))])
    cum = np.interp(np.arange(len(Pg)), sub, cs)
    D = 1.0 if cum[-1] >= 2.0 else 0.5
    out, i, last_t = [], 0, None
    for i in range(len(Pg)):
        if last_t is not None and tt[i] - last_t < 5e8:
            continue
        j = int(np.searchsorted(cum, cum[i] + D))
        if j >= len(Pg):
            break
        last_t = tt[i]
        e = np.linalg.norm((P[j] - P[i]) - (Pg[j] - Pg[i]))
        out.append(e / (cum[j] - cum[i]) * 100)
    return (round(float(np.median(out)), 2) if out else None), D, len(out)


def score(rec, poses, shift_ns, t_first_fed, t_last_fed, gt="auto"):
    rec = Path(rec)
    tr = [m for m in poses if m.get("status") == "tracking" and m.get("T_world_cam0")]
    st = {}
    for m in poses:
        st[m.get("status")] = st.get(m.get("status"), 0) + 1
    res = dict(poses=len(poses), tracked=len(tr), tracked_frac=round(len(tr) / max(1, len(poses)), 3), status_counts=st,
               clip_s=round((t_last_fed - t_first_fed) / 1e9, 2))
    dts = [m["dt_cam_imu"] for m in tr if m.get("dt_cam_imu") is not None]
    if dts:
        res.update(dt_init_ms=round(dts[0] * 1e3, 3), dt_final_ms=round(dts[-1] * 1e3, 3))
    ms = [m["ms"] for m in poses if "ms" in m]
    if ms:
        res.update(pc_ms_med=round(float(np.median(ms)), 2))
    nl = [m["map"].get("n_loop", 0) for m in tr if m.get("map")]
    res["n_loop"] = int(max(nl)) if nl else 0
    if len(tr) < 50:
        res.update(status="no_init", init_s=None)
        return res
    res["init_s"] = round((int(tr[0]["t_ns"]) - t_first_fed) / 1e9, 2)
    gdir = pick_gt(rec) if gt != "none" else None
    if gdir is None:
        res.update(loop_eval(rec, tr))
        le = res.get("loop_err_cm")
        res["status"] = "ok" if le is not None and le < DIVERGE_CM else "diverged"
        return res
    g = dict(np.load(gdir / "gt.npz", allow_pickle=True))
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    gt_t, gt_p, gt_R = g["t_ns"][ok], g["p_bc"][ok], g["R_bc"][ok]
    vt = np.array([int(m["t_ns"]) for m in tr]) + shift_ns
    j = np.clip(np.searchsorted(gt_t, vt), 1, len(gt_t) - 1)
    j = np.where(np.abs(gt_t[j - 1] - vt) < np.abs(gt_t[j] - vt), j - 1, j)
    mm = np.abs(gt_t[j] - vt) < 2_000_000
    res.update(gt=gdir.name, matched=int(mm.sum()))
    if mm.sum() < 50:
        res.update(status="no_gt_match")
        return res
    Pg, Rg, tt = gt_p[j[mm]], gt_R[j[mm]], vt[mm]
    idx = np.nonzero(mm)[0]
    for key in ("vio", "map"):
        T = [_T(tr[i], key) for i in idx]
        if any(t is None for t in T):
            continue
        T = np.array(T)
        P = T[:, :3, 3]
        R, t = umeyama(P, Pg)
        Pa = P @ R.T + t
        e = np.linalg.norm(Pa - Pg, axis=1)
        res[f"ate_{key}_cm"] = round(float(np.sqrt(np.mean(e ** 2))) * 100, 2)
        res[f"end_{key}_cm"] = round(float(np.linalg.norm(Pa[-30:].mean(0) - Pg[-30:].mean(0))) * 100, 2)
        if key == "vio":
            res["drift_cm_per_m"], res["drift_D_m"], res["drift_n"] = drift_per_m(Pa, Pg, tt)
            # rotation: orientation-only alignment (as vio_bench_hs.evaluate)
            Rv = T[:, :3, :3]
            U, _, Vt = np.linalg.svd(np.einsum("nij,nkj->ik", Rg, Rv))
            Rr = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
            ang = np.degrees(np.arccos(np.clip((np.einsum("nij,nij->n", np.einsum("ij,njk->nik", Rr, Rv), Rg) - 1) / 2, -1, 1)))
            res["rot_med_deg"] = round(float(np.median(ang)), 3)
            # scale (Sim3) for reference
            mu_s, mu_d = P.mean(0), Pg.mean(0)
            U2, s2, Vt2 = np.linalg.svd((Pg - mu_d).T @ (P - mu_s) / len(P))
            res["scale_sim3"] = round(float(s2.sum() / np.mean(np.sum((P - mu_s) ** 2, 1))), 4)
    # ruler check (added 18:10, after the pre-registration, reference only): same VIO poses vs the other GT variants
    for alt in ("imu_gt", "imu_gt_rs"):
        if alt == gdir.name or not (rec / alt / "gt.npz").exists():
            continue
        h = dict(np.load(rec / alt / "gt.npz", allow_pickle=True))
        ok2 = h["ok"].astype(bool) & (np.linalg.norm(h["p_bc"], axis=1) < 1.2)
        t2, p2 = h["t_ns"][ok2], h["p_bc"][ok2]
        k = np.clip(np.searchsorted(t2, vt), 1, len(t2) - 1)
        k = np.where(np.abs(t2[k - 1] - vt) < np.abs(t2[k] - vt), k - 1, k)
        m2 = np.abs(t2[k] - vt) < 2_000_000
        if m2.sum() >= 50:
            P2 = np.array([_T(tr[i], "vio")[:3, 3] for i in np.nonzero(m2)[0]])
            R2, tt2 = umeyama(P2, p2[k[m2]])
            res[f"ate_vio_cm_{alt}"] = round(float(np.sqrt(np.mean(np.sum((P2 @ R2.T + tt2 - p2[k[m2]]) ** 2, 1)))) * 100, 2)
    L = path_len(Pg)
    res["path_m"] = round(L, 3)
    res["gt_span_s"] = round((tt[-1] - tt[0]) / 1e9, 2)
    if "ate_vio_cm" in res:
        res["ate_pct"] = round(res["ate_vio_cm"] / max(L * 100, 1e-6) * 100, 2)
        res["status"] = "ok" if res["ate_vio_cm"] < DIVERGE_CM else "diverged"
    return res


def loop_eval(rec, tr):
    g = [x for x in csv.DictReader(open(rec / "imu.csv")) if x["type"] == "16"]
    gtt = np.array([int(x["t_ns"]) for x in g], float)
    gw = np.linalg.norm(np.array([[float(x["v0"]), float(x["v1"]), float(x["v2"])] for x in g]), axis=1)
    gw -= np.median(gw[: max(50, len(gw) // 20)])
    moving = np.convolve(np.abs(gw) > 0.08, np.ones(100), "same") > 0
    t_end = gtt[np.where(moving)[0][-1]] if moving.any() else gtt[-1]
    out = {}
    for key in ("vio", "map"):
        T = [_T(m, key) for m in tr]
        if any(t is None for t in T):
            continue
        tv = np.array([int(m["t_ns"]) for m in tr], float)
        p = np.array([t[:3, 3] for t in T])
        i = min(int(np.searchsorted(tv, t_end + 0.5e9)), len(p) - 1)
        put = p[max(0, i - 10): i + 10].mean(0)
        L = float(np.linalg.norm(np.diff(p[: i + 1: 6], axis=0), axis=1).sum())
        e = float(np.linalg.norm(put - p[0]))
        sfx = "" if key == "vio" else "_map"
        out[f"loop_err{sfx}_cm"] = round(e * 100, 2)
        if key == "vio":
            out.update(path_m=round(L, 3), loop_pct=round(e / max(L, 1e-6) * 100, 2))
    return out


if __name__ == "__main__":   # rescore: lp_score.py OUT.json  (reads OUT.poses.jsonl.gz)
    import gzip
    for f in sys.argv[1:]:
        r = json.load(open(f))
        P = [json.loads(l) for l in gzip.open(f[:-5] + ".poses.jsonl.gz", "rt")]
        shift = int(r["gt_align_dt_ms"] * 1e6) if r["args"]["time"] == "oracle" else 0
        r["score"] = score(Path(r["args"]["rec"]), P, shift, r["t_first_fed"], r["t_last_fed"], gt=r["args"]["gt"])
        json.dump(r, open(f, "w"), indent=1)
        print(f, json.dumps(r["score"]))
