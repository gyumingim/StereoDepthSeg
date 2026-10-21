#!/usr/bin/env python3
"""Independent checker for the G1.0 test (docs/g10_plan.md). Does NOT import g10, g10_analyse, i3_takeoff(_analyse),
z2_regress, c5_overnight_analyse or vio_bench_hs: recomputes from the raw run files (json, poses.jsonl.gz, stderr.log.gz),
the GT (imu_gt_spl/gt.npz) and the input json (runs/i3_confirm/inputs), then compares with runs/g10/summary.json.

  ../venv_ffs/bin/python imu_learn/g10_check.py      # -> runs/g10/check.json, prints mismatches
"""
import glob
import gzip
import json
import re
from pathlib import Path

import numpy as np

S = Path(__file__).resolve().parent.parent
G = S / "runs" / "g10"
TK = S / "runs" / "i3_takeoff"
CF = S / "runs" / "i3_confirm"
RECS = {"213341": "hs_20261021_213341", "213955": "hs_20261021_213955", "214550": "hs_20261021_214550",
        "215348": "hs_20261021_215348"}
ANSI = re.compile(r"\x1b\[[0-9;]*m")


def J(p):
    return json.load(open(str(p) + ".json"))


def have(p):
    return Path(str(p) + ".json").exists()


def poses(p):
    t, tc, X, trk = [], [], [], []
    for line in gzip.open(str(p) + ".poses.jsonl.gz", "rt"):
        m = json.loads(line)
        if m.get("type", "pose") != "pose":
            continue
        T = m.get("T_world_cam0")
        t.append(int(m["t_ns"]))
        tc.append(m.get("t", np.nan))
        ok = m.get("status") == "tracking" and T is not None
        trk.append(ok)
        X.append([T[3], T[7], T[11]] if T is not None else [np.nan] * 3)
    return np.array(t, np.int64), np.array(tc, float), np.array(X, float), np.array(trk, bool)


_GT = {}


def gt(rec):
    if rec not in _GT:
        g = np.load(S / "recordings" / rec / "imu_gt_spl" / "gt.npz", allow_pickle=True)
        ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
        _GT[rec] = (g["t_ns"][ok].astype(np.int64), g["p_bc"][ok], g["n_corners"][ok])
    return _GT[rec]


def nearest(ts, q):
    j = np.clip(np.searchsorted(ts, q), 1, len(ts) - 1)
    return np.where(np.abs(ts[j - 1] - q) <= np.abs(ts[j] - q), j - 1, j)


def align(A, B, scale):
    ma, mb = A.mean(0), B.mean(0)
    a, b = A - ma, B - mb
    U, D, Vt = np.linalg.svd(b.T @ a / len(A))
    E = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        E[2, 2] = -1
    R = U @ E @ Vt
    c = (np.trace(np.diag(D) @ E) / a.var(0).sum()) if scale else 1.0
    t = mb - c * R @ ma
    return R, c, t


def window(p, rec, t_dep):
    t, _, X, trk = poses(p)
    if not trk.any():
        return dict(status="noinit")
    dt = int(J(p)["align_dt_ms"] * 1e6)
    w = trk & (t >= t_dep) & (t <= t_dep + 10e9)
    gt_t, gt_p, _ = gt(rec)
    q = t[w] + dt
    if not len(q):
        return dict(status="unscored")
    j = nearest(gt_t, q)
    mm = np.abs(gt_t[j] - q) < 2e6
    if mm.sum() < 30:
        return dict(status="unscored")
    A, B = X[w][mm], gt_p[j[mm]]
    R, _, tt = align(A, B, False)
    e = np.sqrt((np.linalg.norm(A @ R.T + tt - B, axis=1) ** 2).mean()) * 100
    _, c, _ = align(A, B, True)
    return dict(status="collapse" if (c < 0.5 or c > 2 or e > 10) else "ok", ate=float(e), scale=float(c))


def gt_speed_at(rec, q):
    """GT speed by +-0.1 s central difference at GT samples, sampled at the nearest GT sample to q (<= 20 ms)."""
    gt_t, gt_p, nc = gt(rec)
    ts = gt_t / 1e9
    ja = nearest(ts, ts + 0.1)
    jb = nearest(ts, ts - 0.1)
    ok = (np.abs(ts[ja] - ts - 0.1) < 0.02) & (np.abs(ts[jb] - ts + 0.1) < 0.02)
    v = np.full(len(ts), np.nan)
    v[ok] = np.linalg.norm(gt_p[ja[ok]] - gt_p[jb[ok]], axis=1) / (ts[ja[ok]] - ts[jb[ok]])
    j = nearest(gt_t, q)
    near = np.abs(gt_t[j] - q) < 0.02e9
    return np.where(near, v[j], np.nan), nc[j]


def false_pass(p, rec):
    t, tc, _, _ = poses(p)
    ok = np.isfinite(tc)
    o = np.argsort(tc[ok])
    tcs, tns = tc[ok][o], t[ok][o].astype(float)
    cur, pt = None, []
    txt = gzip.open(str(p) + ".stderr.log.gz", "rt", errors="replace").read()
    txt = re.sub(r"\[G10\] [-0-9.e+]+ thr [0-9.]+\n", "", txt)     # G1.0 bridge lines can split other log lines
    for line in txt.splitlines():
        line = ANSI.sub("", line)
        if "[DIAGT]" in line:
            cur = float(line.split("[DIAGT]")[1])
        elif cur is not None and "[ZUPT]: passed disparity" in line:
            pt.append(cur)
    if not pt:
        return 0
    q = np.interp(np.array(pt), tcs, tns) + J(p)["align_dt_ms"] * 1e6
    v, nc = gt_speed_at(rec, q)
    return int(np.sum((v > 0.02) & (nc >= 12)))


def inp(rec):
    return json.load(open(CF / "inputs" / f"{rec}.json"))


def dep_prefixes():
    """key -> (rec, {cond: prefix}, max_secs) for every departure cell."""
    out = {}
    for r6, rec in RECS.items():
        for d in (1, 5, 10):
            k = f"B_{r6}_s0_d{d}"
            out[k] = (rec, {"base": TK / f"{k}_base", "i3": TK / f"{k}_i3", "G": G / f"{k}_G"}, 40)
    for k in range(3):
        out[f"BA_214550_o{k}"] = (RECS["214550"], {c: CF / f"214550_{c}_o{k}" for c in ("base", "i3")} |
                                  {"G": G / f"BA_214550_o{k}_G"}, 0)
    for L in (3, 6):
        out[f"BL_214550_L{L}"] = (RECS["214550"], {c: TK / f"B2_214550_L{L}_{c}" for c in ("base", "i3")} |
                                  {"G": G / f"BL_214550_L{L}_G"}, 40)
    for d in (1, 5):
        for s in ("m025", "p025"):
            k = f"N_213341_s0_d{d}_{s}"
            out[k] = (RECS["213341"], {c: G / f"{k}_{c}" for c in ("base", "i3", "G")}, 40)
    return out


def main():
    summ = json.load(open(G / "summary.json"))
    mism, rows = [], {}
    for k, (rec, pre, ms) in dep_prefixes().items():
        x = inp(rec)
        st = x["setdown"][0]["still_imu_ns"]
        t_dep = st[1]
        row = {}
        for c, p in pre.items():
            if not have(p):
                row[c] = None
                continue
            w = window(p, rec, t_dep)
            w["fp"] = false_pass(p, rec)
            if c == "base":
                so = J(p)["start_off"]
                lim = min(x["last_frame_imu_ns"], x["first_frame_imu_ns"] + int((so + ms) * 1e9)) if ms else x["last_frame_imu_ns"]
                row["inside"] = bool(t_dep + 10e9 <= lim)
            row[c] = w
        b = row.get("base")
        row["tested"] = bool(b and b["status"] in ("ok", "collapse") and row.get("inside"))
        rows[k] = row
    gates = json.load(open(G / "gates.json"))["gates"]
    for k, (gp, _) in gates.items():
        row = {"gate_possible": gp}
        for c in ("base", "G"):
            p = G / f"{k}_{c}"
            if not have(p):
                row[c] = None
                continue
            d = J(p)
            trk = poses(p)[3].any()
            ate, sc = d.get("ate_cm"), d.get("scale")
            stt = ("noinit" if not trk else "unscored" if ate is None or sc is None else
                   "ok" if (0.5 <= sc <= 2 and ate <= 10) else "collapse")
            row[c] = dict(status=stt, ate=ate, scale=sc, fp=false_pass(p, d["rec"]), gate_open=d.get("g10_gate_open_frames"))
        row["tested"] = bool(gp and row["base"] and row["base"]["status"] != "noinit" and row["G"])
        rows[k] = row
    # compare per-run values with the summary
    sd = {r["key"]: r for r in summ["dep_rows"]}
    for k, row in rows.items():
        if k.startswith("R_"):
            continue
        s = sd.get(k)
        for c in ("base", "i3", "G"):
            a, b = row.get(c), (s or {}).get(c)
            if (a is None) != (b is None):
                mism.append(f"{k} {c}: presence {a is not None} vs {b is not None}")
                continue
            if a is None:
                continue
            if a["status"] != b["status"]:
                mism.append(f"{k} {c}: status {a['status']} vs {b['status']}")
            if a.get("ate") is not None and (b.get("ate_cm") is None or abs(a["ate"] - b["ate_cm"]) > 0.002):
                mism.append(f"{k} {c}: ate {a.get('ate')} vs {b.get('ate_cm')}")
            if a["fp"] != b.get("false_pass"):
                mism.append(f"{k} {c}: false_pass {a['fp']} vs {b.get('false_pass')}")
        if bool(row["tested"]) != bool(s.get("tested")):
            mism.append(f"{k}: tested {row['tested']} vs {s.get('tested')}")
    sr = {r["key"]: r for r in summ["r_rows"]}
    for k, row in rows.items():
        if not k.startswith("R_"):
            continue
        s = sr[k]
        for c in ("base", "G"):
            a, b = row.get(c), s.get(c)
            if (a is None) != (b is None):
                mism.append(f"{k} {c}: presence")
                continue
            if a is None:
                continue
            for ka, kb in (("status", "status"), ("fp", "false_pass"), ("gate_open", "gate_open_frames")):
                if a[ka] != b.get(kb):
                    mism.append(f"{k} {c}: {ka} {a[ka]} vs {b.get(kb)}")
    # criteria recomputed
    fpw, newc = [], []
    for k, row in rows.items():
        if row.get("tested") and row.get("G") and row.get("base"):
            if row["G"]["fp"] > row["base"]["fp"]:
                fpw.append(k)
            if row["base"]["status"] == "ok" and row["G"]["status"] != "ok":
                newc.append(k)
    c1 = not fpw and not newc
    bandv = []
    for d in (1, 5):
        for c in ("base", "i3"):
            v = [rows[k][c]["ate"] for k in (f"B_213341_s0_d{d}", f"N_213341_s0_d{d}_m025", f"N_213341_s0_d{d}_p025")
                 if rows.get(k, {}).get(c) and rows[k][c].get("ate") is not None]
            if len(v) >= 2:
                bandv.append(max(v) - min(v))
    band = max(bandv) if bandv else None
    rd = [rows[k]["G"]["ate"] - rows[k]["base"]["ate"] for k in rows if k.startswith("R_") and rows[k]["tested"]
          and (rows[k]["G"]["gate_open"] or 0) > 0 and rows[k]["base"]["status"] in ("ok", "collapse")
          and rows[k]["G"]["status"] in ("ok", "collapse")]
    c2 = None if (not rd or band is None) else bool(np.median(rd) <= 0.1 + 1e-9 and max(rd) <= band + 1e-9)
    t41 = [k for k in rows if "214550" in k and rows[k].get("tested") and rows[k].get("G")]
    d41 = [rows[k]["G"]["ate"] - rows[k]["base"]["ate"] for k in t41
           if rows[k]["G"].get("ate") is not None and rows[k]["base"].get("ate") is not None]
    prev = [k for k in t41 if rows[k]["base"]["status"] == "collapse" and rows[k]["G"]["status"] == "ok"]
    c3 = None if not d41 else bool(np.median(d41) < 0 and len(prev) >= 2)
    v = summ["verdict"]
    mine = dict(c1=c1, c1_fp_worse=sorted(fpw), c1_new=sorted(newc), c2=c2, c2_n=len(rd),
                c2_median=None if not rd else float(np.median(rd)), c2_worst=None if not rd else float(max(rd)), band=band,
                c3=c3, c3_n=len(t41), c3_median=None if not d41 else float(np.median(d41)), c3_prevented=sorted(prev))
    for a, b in (("c1", v["c1_safety"]["ok"]), ("c2", v["c2_regression"]["ok"]), ("c3", v["c3_gain_214550"]["ok"])):
        if mine[a] != b:
            mism.append(f"criterion {a}: checker {mine[a]} vs analysis {b}")
    for a, b in (("c2_median", v["c2_regression"]["median"]), ("band", v["c2_regression"]["band"]),
                 ("c3_median", v["c3_gain_214550"]["median"])):
        if (mine[a] is None) != (b is None) or (mine[a] is not None and abs(mine[a] - b) > 0.005):
            mism.append(f"{a}: checker {mine[a]} vs analysis {b}")
    if sorted(prev) != sorted(v["c3_gain_214550"]["prevented"]):
        mism.append(f"prevented: {prev} vs {v['c3_gain_214550']['prevented']}")
    res = dict(checker=mine, n_rows=len(rows), n_compared_runs=sum(1 for r in rows.values() for c in ("base", "i3", "G")
                                                                    if isinstance(r.get(c), dict)),
               mismatches=mism)
    json.dump(res, open(G / "check.json", "w"), indent=1, default=float)
    print(json.dumps(mine, indent=1, default=float))
    print(f"runs compared {res['n_compared_runs']}, mismatches {len(mism)}")
    for m in mism:
        print("  ", m)


if __name__ == "__main__":
    main()
