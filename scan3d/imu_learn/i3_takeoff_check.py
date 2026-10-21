#!/usr/bin/env python3
"""Independent re-computation for the i3 takeoff test (docs/i3_takeoff_plan.md). Does NOT import i3_takeoff,
i3_takeoff_analyse, i3_confirm or c5_overnight_analyse: reads imu.csv, frames_cam5.csv, imu_gt/align.json,
imu_gt_spl/gt.npz and the run files directly, with its own still detector, own Umeyama, own GT matching.
Compares with runs/i3_takeoff/summary.json -> runs/i3_takeoff/check.json.

  ../venv_ffs/bin/python imu_learn/i3_takeoff_check.py
"""
import csv
import gzip
import json
import time
from pathlib import Path

import numpy as np

S = Path(__file__).resolve().parent.parent
RUNS = S / "runs/i3_takeoff"
SRC = S / "runs/i3_confirm"
RECS = ["hs_20261021_213341", "hs_20261021_213955", "hs_20261021_214550", "hs_20261021_215348"]


def imu(rec, typ):
    rows = []
    with open(S / "recordings" / rec / "imu.csv") as f:
        for r in csv.reader(f):
            if len(r) < 5 or r[0] != str(typ):
                continue
            try:
                rows.append([int(r[1]), float(r[2]), float(r[3]), float(r[4])])
            except ValueError:
                pass
    a = np.array(rows)
    return a[:, 0].astype(np.int64), a[:, 1:4]


def stills(t, g):
    ts = (t - t[0]) / 1e9
    st = np.arange(0, ts[-1] - 0.5, 0.1)
    ok = []
    for s in st:
        i, j = np.searchsorted(ts, s), np.searchsorted(ts, s + 0.5)
        w = g[i:j]
        ok.append(j - i >= 20 and bool((w.std(0) < 0.01).all()) and np.linalg.norm(w.mean(0)) < 0.03)
    out, k = [], 0
    while k < len(ok):
        if ok[k]:
            k2 = k
            while k2 + 1 < len(ok) and ok[k2 + 1]:
                k2 += 1
            if st[k2] + 0.5 - st[k] >= 1.5 - 1e-9:
                out.append((int(t[0] + st[k] * 1e9), int(t[0] + (st[k2] + 0.5) * 1e9)))
            k = k2 + 1
        else:
            k += 1
    return out


def rec_info(rec):
    R = S / "recordings" / rec
    tf = [int(r["t_mid_ns"]) for r in csv.DictReader(open(R / "frames_cam5.csv"))]
    dt = int(json.load(open(R / "imu_gt/align.json"))["dt_ns"])
    f0, fl = tf[0] - dt, tf[-1] - dt
    tg, g = imu(rec, 16)
    ss = stills(tg, g)
    start = next(((a, b) for a, b in ss if a <= f0 + 10e9 and b > f0), None)
    sd = []
    for g0, g1 in ((50.0, 75.0), (115.0, 1e9)):
        c = [(a, b) for a, b in ss if (b - f0) / 1e9 > g0 and (a - f0) / 1e9 < g1 and (a, b) != start]
        sd.append(max(c, key=lambda x: x[1] - x[0]) if c else None)
    return dict(f0=f0, fl=fl, start=start, sd=sd)


def gt(rec):
    g = np.load(S / "recordings" / rec / "imu_gt_spl/gt.npz", allow_pickle=True)
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    return g["t_ns"][ok].astype(np.int64), g["p_bc"][ok]


def poses(prefix):
    t, X, v, ba = [], [], [], []
    for line in gzip.open(str(prefix) + ".poses.jsonl.gz", "rt"):
        m = json.loads(line)
        if m.get("type", "pose") != "pose" or m.get("status") != "tracking" or not m.get("T_world_cam0"):
            continue
        T = m["T_world_cam0"]
        t.append(m["t_ns"])
        X.append([T[3], T[7], T[11]])
        v.append(np.linalg.norm(m["v"]) if m.get("v") else np.nan)
        ba.append(np.linalg.norm(m["ba"]) if m.get("ba") else np.nan)
    return np.array(t, np.int64), np.array(X, float).reshape(-1, 3), np.array(v), np.array(ba)


def kabsch(P, Q, scale):
    mp, mq = P.mean(0), Q.mean(0)
    A, B = P - mp, Q - mq
    U, D, Vt = np.linalg.svd(B.T @ A / len(P))
    E = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ E @ Vt
    c = np.trace(np.diag(D) @ E) / (A ** 2).sum(1).mean() if scale else 1.0
    e = np.linalg.norm(c * A @ R.T - B, axis=1)
    return float(c), float(np.sqrt((e ** 2).mean()) * 100)


def nearest(gt_t, q):
    j = np.clip(np.searchsorted(gt_t, q), 1, len(gt_t) - 1)
    j = np.where(np.abs(gt_t[j - 1] - q) < np.abs(gt_t[j] - q), j - 1, j)
    return j, np.abs(gt_t[j] - q) < 2e6


def win_status(prefix, rec, t_dep, G):
    d = json.load(open(str(prefix) + ".json"))
    t, X, _, _ = poses(prefix)
    if not len(t):
        return dict(status="noinit")
    dt = int(d["align_dt_ms"] * 1e6)
    m = (t >= t_dep) & (t <= t_dep + 10e9)
    j, ok = nearest(G[0], t[m] + dt)
    if ok.sum() < 30:
        return dict(status="unscored")
    P, Q = X[m][ok], G[1][j[ok]]
    _, a = kabsch(P, Q, False)
    c, _ = kabsch(P, Q, True)
    return dict(status="collapse" if (c < 0.5 or c > 2 or a > 10) else "ok", ate_cm=a, scale=c)


def main():
    summ = json.load(open(RUNS / "summary.json"))
    mism, n = [], 0

    def cmp(tag, mine, theirs, tol=None):
        nonlocal n
        n += 1
        if tol is None:
            same = mine == theirs
        else:
            same = (mine is None and theirs is None) or (mine is not None and theirs is not None and abs(mine - theirs) <= tol)
        if not same:
            mism.append(f"{tag}: check {mine} vs analyse {theirs}")

    info = {r: rec_info(r) for r in RECS}
    G = {r: gt(r) for r in RECS}
    rows = {x["key"]: x for x in summ["B"]["rows"]}
    tested, mine_rows = [], {}
    for rec in RECS:
        I = info[rec]
        ta, a = imu(rec, 35)
        an = np.linalg.norm(a, axis=1)
        for si, st in enumerate(I["sd"]):
            for d in (1, 5, 10):
                key = f"{rec[-6:]}_s{si}_d{d}"
                off = round((st[0] - I["f0"]) / 1e9 + d, 3)
                cmp(f"{key} off", off, rows[key]["off"], 1e-6)
                inside = st[1] + 10e9 <= min(I["fl"], I["f0"] + (off + 40) * 1e9)
                cmp(f"{key} inside", inside, rows[key]["window_inside"])
                mm = (ta >= st[1] - 0.5e9) & (ta <= st[1] + 1e9)
                cmp(f"{key} impact_g", round(float(an[mm].max()) / 9.81, 2), rows[key]["impact_g"], 0.011)
                res = {}
                for c in ("base", "i3"):
                    p = RUNS / f"B_{key}_{c}"
                    if not Path(str(p) + ".json").exists():
                        mism.append(f"{key} {c}: run missing")
                        continue
                    w = win_status(p, rec, st[1], G[rec])
                    res[c] = w
                    th = rows[key].get(c) or {}
                    cmp(f"{key} {c} status", w["status"], th.get("status"))
                    if "ate_cm" in w:
                        cmp(f"{key} {c} ate", round(w["ate_cm"], 3), th.get("ate_cm"), 0.002)
                        cmp(f"{key} {c} scale", round(w["scale"], 4), th.get("scale"), 2e-4)
                mine_rows[key] = res
                if inside and len(res) == 2:
                    tested.append((rec, key, res))
    new = sorted(k for _, k, r in tested if r["base"]["status"] == "ok" and r["i3"]["status"] != "ok")
    prev = sorted(k for _, k, r in tested if r["base"]["status"] == "collapse" and r["i3"]["status"] == "ok")
    both = [r for _, _, r in tested if "ate_cm" in r["base"] and "ate_cm" in r["i3"]]
    bm = float(np.median([r["base"]["ate_cm"] for r in both])) if both else None
    zm = float(np.median([r["i3"]["ate_cm"] for r in both])) if both else None
    lim = None if bm is None else max(1.1 * bm, bm + 0.2)
    C = summ["B"]["criteria"]
    cmp("n_tested", len(tested), C["n_tested"])
    cmp("i_new", new, sorted(C["i_new"]))
    cmp("ii_prevented", prev, sorted(C["ii_prevented"]))
    cmp("base_collapses", sorted(k for _, k, r in tested if r["base"]["status"] == "collapse"), sorted(C["base_collapses"]))
    cmp("iii_base_med", bm, C["iii_base_med"], 0.003)
    cmp("iii_i3_med", zm, C["iii_i3_med"], 0.003)
    cmp("iii_ok", None if lim is None else bool(zm <= lim + 1e-9), C["iii_ok"])
    # A: criterion-4 window values (set-down stills trimmed 0.5 s, first/last tracked pose |dP|, >= 5 GT samples)
    rec = "hs_20261021_215348"
    I = info[rec]
    dmax = 0.0
    for c in ("base", "i3"):
        pre = [SRC / f"215348_{c}_o1"] + [RUNS / f"A_215348_{c}_o1_r{r}" for r in (1, 2)]
        pre = [p for p in pre if Path(str(p) + ".json").exists()]
        vals = []
        for p in pre:
            d = json.load(open(str(p) + ".json"))
            dt = int(round(d["align_dt_ms"] * 1e6))
            t, X, _, _ = poses(p)
            vv = []
            for st in I["sd"]:
                w0, w1 = st[0] + 0.5e9, st[1] - 0.5e9
                ng = int(((G[rec][0] >= w0 + dt) & (G[rec][0] <= w1 + dt)).sum())
                m = (t >= w0) & (t <= w1)
                vv.append(None if ng < 5 or m.sum() < 2 else float(np.linalg.norm(X[m][-1] - X[m][0]) * 100))
            vals.append(vv)
        th = summ["A"]["conds"][c]["drift"]
        for i, (a_, b_) in enumerate(zip(vals, th)):
            for k, (x, y) in enumerate(zip(a_, b_)):
                cmp(f"A {c} run{i} win{k}", None if x is None else round(x, 3), y, 0.0011)
        for i in range(len(vals)):
            for k in range(i + 1, len(vals)):
                for x, y in zip(vals[i], vals[k]):
                    if x is not None and y is not None:
                        dmax = max(dmax, abs(x - y))
    cmp("A d_rep_max", round(dmax, 3), summ["A"]["d_rep_max"], 0.0011)
    # C: 214550 pre-departure |ba|, |v| medians, ratio medians
    rec = "hs_20261021_214550"
    t_dep = info[rec]["start"][1]
    rat = {"ba": [], "v": []}
    for k in range(3):
        val = {}
        for c in ("base", "i3"):
            t, _, v, ba = poses(SRC / f"214550_{c}_o{k}")
            m = (t >= t_dep - 0.5e9) & (t < t_dep)
            val[c] = (float(np.median(ba[m])), float(np.median(v[m])))
            cmp(f"C {c}_o{k} ba", round(val[c][0], 5), round(summ["C"]["runs"][f"{c}_o{k}"]["ba_pre"], 5), 2e-5)
            cmp(f"C {c}_o{k} v", round(val[c][1], 5), round(summ["C"]["runs"][f"{c}_o{k}"]["v_pre"], 5), 2e-5)
        rat["ba"].append(val["base"][0] / val["i3"][0])
        rat["v"].append(val["base"][1] / val["i3"][1])
    hit = max(np.median(rat["ba"]), np.median(rat["v"])) >= 2
    cmp("C hypothesis", hit, summ["C"]["verdict"].startswith("정지 중"))
    out = dict(made=time.strftime("%F %T"), summary_made=summ.get("made"), n_checked=n, mismatch=mism,
               mine=dict(new=new, prevented=prev, base_med=bm, i3_med=zm, lim=lim, d_rep_max=dmax,
                         c_ratio_med=dict(ba=float(np.median(rat["ba"])), v=float(np.median(rat["v"])))))
    json.dump(out, open(RUNS / "check.json", "w"), indent=1, ensure_ascii=False, default=str)
    print(f"checked {n}, mismatch {len(mism)}")
    for x in mism[:40]:
        print(" ", x)


if __name__ == "__main__":
    main()
