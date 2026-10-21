#!/usr/bin/env python3
"""Self-check of runs/i3_confirm/summary.json: recompute the verdict numbers from the raw run files (json, poses, GT npz,
inputs json) WITHOUT i3_confirm / c5_overnight_analyse / z2_regress code (own Umeyama, own GT speed, own gate), compare.
Writes runs/i3_confirm/check.json; exit 1 on any mismatch.
  ../venv_ffs/bin/python imu_learn/i3_confirm_check.py [runs/i3_confirm]
"""
import gzip
import json
import sys
import time
from pathlib import Path

import numpy as np

SCAN3D = Path(__file__).resolve().parent.parent


def umeyama(X, Y, scale):
    mx, my = X.mean(0), Y.mean(0)
    Xc, Yc = X - mx, Y - my
    U, S, Vt = np.linalg.svd(Yc.T @ Xc / len(X))
    D = np.eye(3)
    D[2, 2] = np.sign(np.linalg.det(U @ Vt))
    R = U @ D @ Vt
    c = float(np.trace(np.diag(S) @ D) / (Xc ** 2).sum(1).mean()) if scale else 1.0
    return R, c, my - c * R @ mx


def gt_load(rec):
    g = np.load(SCAN3D / "recordings" / rec / "imu_gt_spl" / "gt.npz", allow_pickle=True)
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    return g["t_ns"][ok].astype(np.int64), g["p_bc"][ok], g["n_corners"][ok]


def nearest(tg, tq):
    j = np.clip(np.searchsorted(tg, tq), 1, len(tg) - 1)
    return np.where(np.abs(tg[j - 1] - tq) <= np.abs(tg[j] - tq), j - 1, j)


def poses(prefix):
    P = [json.loads(x) for x in gzip.open(str(prefix) + ".poses.jsonl.gz", "rt")]
    tr = [m for m in P if m.get("status") == "tracking" and m.get("T_world_cam0")]
    return (np.array([m["t_ns"] for m in tr], np.int64),
            np.array([np.asarray(m["T_world_cam0"], float).reshape(4, 4)[:3, 3] for m in tr]).reshape(-1, 3),
            max((m["t_ns"] for m in P if "t_ns" in m), default=None))


def status(prefix, d, gt):
    t, X, _ = poses(prefix)
    if not len(t):
        return "noinit"
    if d.get("ate_cm") is None or d.get("scale") is None:
        return "unscored"
    dt = int(round(d["align_dt_ms"] * 1e6))
    tg, pg, _ = gt
    coll = None
    for W in (10, 5):
        w = t <= t[0] + W * 1e9
        vt = t[w] + dt
        j = nearest(tg, vt)
        m = np.abs(tg[j] - vt) < 2e6
        if m.sum() < 30:
            continue
        A, B = X[w][m], pg[j[m]]
        R, _, tt = umeyama(A, B, False)
        ate = float(np.sqrt((np.linalg.norm(A @ R.T + tt - B, axis=1) ** 2).mean()) * 100)
        _, c, _ = umeyama(A, B, True)
        coll = c < 0.5 or c > 2 or ate > 10
        break
    if coll or not (0.5 <= d["scale"] <= 2) or d["ate_cm"] > 10:
        return "collapse"
    return "ok"


def held(prefix, d, gt):
    tg, pg, nc = gt
    dt = int(round(d["align_dt_ms"] * 1e6))
    _, _, tl = poses(prefix)
    n = fast = 0
    vmax, untested = None, 0
    for s, e in d.get("hold_segments_imu_ns") or []:
        e = tl if e is None else e
        idx = np.nonzero((tg >= s + dt) & (tg < e + dt))[0]
        k = 0
        for i in idx:
            ia, ib = nearest(tg, np.array([tg[i] + 100_000_000]))[0], nearest(tg, np.array([tg[i] - 100_000_000]))[0]
            if abs(tg[ia] - tg[i] - 1e8) >= 2e7 or abs(tg[i] - tg[ib] - 1e8) >= 2e7:
                continue
            if min(nc[i], nc[ia], nc[ib]) < 12:
                continue
            v = np.linalg.norm(pg[ia] - pg[ib]) / ((tg[ia] - tg[ib]) / 1e9)
            n += 1
            k += 1
            fast += v > 0.02
            vmax = v * 100 if vmax is None else max(vmax, v * 100)
        untested += k == 0
    return n, fast, vmax, untested


def drift(prefix, d, gt, st):
    tg = gt[0]
    dt = int(round(d["align_dt_ms"] * 1e6))
    t, X, _ = poses(prefix)
    w0, w1 = st[0] + 5e8, st[1] - 5e8
    ng = int(((tg >= w0 + dt) & (tg <= w1 + dt)).sum())
    m = (t >= w0) & (t <= w1)
    if ng < 5 or m.sum() < 2:
        return None
    return float(np.linalg.norm(X[m][-1] - X[m][0]) * 100)


def main(out="runs/i3_confirm"):
    out = SCAN3D / out
    S = json.load(open(out / "summary.json"))
    mism, n = [], 0

    def cmp(key, a, b, tol=0.011):
        nonlocal n
        n += 1
        if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool):
            if abs(a - b) > tol:
                mism.append(f"{key}: summary {a} vs check {b}")
        elif a != b:
            mism.append(f"{key}: summary {a} vs check {b}")

    signs, adopt_parts = {}, dict(c1=True, c2=True, c3=True)
    judged = []
    for rec, R in S["recs"].items():
        if "c1" not in R:
            continue
        judged.append(rec)
        inp = json.load(open(out / "inputs" / f"{rec}.json"))
        gt = gt_load(rec)
        st = {}
        ates = {}
        H = dict(n=0, fast=0, vmax=None, untested=0, held=0)
        dr = {}
        for c in ("base", "i3"):
            for k in range(3):
                p = out / f"{rec[-6:]}_{c}_o{k}"
                d = json.load(open(str(p) + ".json"))
                cmp(f"{rec} {c} o{k} start_off", d["start_off"], inp["offsets_s"][k], 1e-6)
                cmp(f"{rec} {c} o{k} hold", d.get("still_hold"), "1.5:0.05:0.05:i3" if c == "i3" else None)
                cmp(f"{rec} {c} o{k} threads", d.get("cfg_over"), "num_opencv_threads=2")
                cmp(f"{rec} {c} o{k} imu", d.get("imu_source"), "raw_16_35")
                st[(c, k)] = status(p, d, gt)
                ates[(c, k)] = d.get("ate_cm")
                if c == "i3":
                    a, f, v, u = held(p, d, gt)
                    H["n"] += a
                    H["fast"] += int(f)
                    H["vmax"] = v if H["vmax"] is None else (H["vmax"] if v is None else max(H["vmax"], v))
                    H["untested"] += u
                    H["held"] += d.get("frames_held") or 0
                for wi, w in enumerate(inp["setdown"]):
                    if w["still_imu_ns"] is not None:
                        dr[(c, k, wi)] = drift(p, d, gt, w["still_imu_ns"])
        cmp(f"{rec} base status", R["c2"]["base_status"], [st[("base", k)] for k in range(3)])
        cmp(f"{rec} i3 status", R["c2"]["i3_status"], [st[("i3", k)] for k in range(3)])
        new = [k for k in range(3) if st[("base", k)] == "ok" and st[("i3", k)] != "ok"]
        cmp(f"{rec} new collapses", len(R["c2"]["new_collapses"]), len(new))
        bm = np.median([ates[("base", k)] for k in range(3) if st[("base", k)] in ("ok", "collapse")] or [np.nan])
        zm = np.median([ates[("i3", k)] for k in range(3) if st[("i3", k)] in ("ok", "collapse")] or [np.nan])
        ok1 = bool(zm <= max(bm * 1.1, bm + 0.2) + 1e-9) if np.isfinite(bm) and np.isfinite(zm) else None
        cmp(f"{rec} c1 base_med", R["c1"]["base_med"], float(bm))
        cmp(f"{rec} c1 i3_med", R["c1"]["i3_med"], float(zm))
        cmp(f"{rec} c1 ok", R["c1"]["ok"], ok1)
        cmp(f"{rec} c3 gated_n", R["c3"]["gated_n"], H["n"], 0)
        cmp(f"{rec} c3 gated_fast", R["c3"]["gated_fast"], H["fast"], 0)
        if H["vmax"] is not None or R["c3"]["vmax_cms"] is not None:
            cmp(f"{rec} c3 vmax", R["c3"]["vmax_cms"], H["vmax"], 0.01)
        cmp(f"{rec} c3 untested segs", len(R["c3"]["untested"]), H["untested"], 0)
        if H["held"] == 0:
            v3 = "n/a (0 held)"
        elif H["n"] and (H["fast"] / H["n"] > 0.005 + 1e-12 or H["vmax"] > 3.0 + 1e-9):
            v3 = "fail"
        elif H["untested"]:
            v3 = "partly not tested"
        elif H["n"] == 0:
            v3 = "not tested"
        else:
            v3 = "pass"
        cmp(f"{rec} c3 verdict", R["c3"]["verdict"], v3)
        sb = sz = 0.0
        tested = 0
        for wi, w in enumerate(inp["setdown"]):
            if w["still_imu_ns"] is None:
                continue
            b = [dr[("base", k, wi)] for k in range(3) if dr[("base", k, wi)] is not None]
            z = [dr[("i3", k, wi)] for k in range(3) if dr[("i3", k, wi)] is not None]
            if b and z:
                sb += float(np.median(b))
                sz += float(np.median(z))
                tested += 1
        sign = "not tested" if not tested else ("+" if sz < sb - 0.05 else "-" if sz > sb + 0.05 else "=")
        cmp(f"{rec} c4 sign", R["c4"]["sign"], sign)
        if tested:
            cmp(f"{rec} c4 base_sum", R["c4"]["base_sum"], sb, 0.01)
            cmp(f"{rec} c4 i3_sum", R["c4"]["i3_sum"], sz, 0.01)
        signs[rec] = sign
        adopt_parts["c1"] &= ok1 is True
        adopt_parts["c2"] &= not new
        adopt_parts["c3"] &= v3 in ("pass", "n/a (0 held)")
    npl, nmi = sum(s == "+" for s in signs.values()), sum(s == "-" for s in signs.values())
    nt = sum(s in "+-=" for s in signs.values())
    c4 = "not tested" if nt == 0 else ("improve" if npl > nmi else "not improve")
    adopt = bool(judged) and all(adopt_parts.values()) and c4 == "improve"
    cmp("c4 overall", S["criteria"]["c4"], c4)
    cmp("adopt", S["criteria"]["adopt"], adopt)
    cmp("n judged", S["criteria"]["n"], len(judged), 0)
    res = dict(made=time.strftime("%F %T"), n_checked=n, mismatch=mism, adopt=adopt, c4=c4, judged=judged)
    json.dump(res, open(out / "check.json", "w"), indent=1, default=float)
    print(json.dumps(res, indent=1, default=float))
    return 1 if mism else 0


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:]))
