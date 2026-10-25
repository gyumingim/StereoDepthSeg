#!/usr/bin/env python3
"""ovrobust offline analysis (2026-10-25) of /tmp/ovrobust/ate cells (ate_run.py), on top of `ate_run.py score`:
  - init: first tracking pose time from the first fed frame (s), per condition (H = current jerk init, SI = --init-still)
  - still before motion: from the IMU (raw gyro norm, 0.2 s mean > 0.15 rad/s = moving) after the first fed frame
  - H identity: poses (without "h") identical to the ovopt B0 cell (/tmp/ovopt/ate/B0)?
  - health: share of frames per state (from the "h" object of the pose lines)
  - blank (BL*/BLR*): SE3-aligned camera-position error (GT imu_gt_spl_cal1022, alignment on all matched poses except
    the blank window + 3 s) -> max error in [blank start, blank end + 3 s], ATE of the part after it (blank end + 3 s ..
    end) in the joint alignment and aligned on its own; the same windows on the unblanked H run as reference.
  python3 ana.py [conds]   -> OUT/ana_<conds>.json + tables
"""
import gzip
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ate_run as ar  # noqa: E402  (chdir to the main checkout, OUT, cells)
import vio_bench_hs as vb  # noqa: E402

OUT = ar.OUT
STRIP = ("ms", "remap_ms", "wait_ms", "h", "blank")


def poses(path):
    return [json.loads(x) for x in gzip.open(path, "rt") if x.strip()]


def phash(P):
    h = hashlib.sha256()
    for m in P:
        h.update(json.dumps({k: v for k, v in m.items() if k not in STRIP}, sort_keys=True).encode())
    return h.hexdigest()[:16]


_still = {}


def still_s(rec, t_first_ns, old):
    """seconds without rotation after the first fed frame (t_first_ns = camera time on the IMU clock), raw gyro"""
    key = (rec, t_first_ns)
    if key not in _still:
        acc, gyr = vb.read_imu(Path(f"recordings/{rec}") / "imu.csv", raw=rec not in old)
        t = gyr[:, 0].astype(np.int64)
        w = np.linalg.norm(gyr[:, 1:4], axis=1)
        i0 = int(np.searchsorted(t, t_first_ns))
        dt = float(np.median(np.diff(t[i0:i0 + 1000]))) / 1e9
        k = max(1, int(0.2 / dt))
        mv = np.convolve(w[i0:], np.ones(k) / k, mode="valid") > 0.15
        j = int(np.argmax(mv)) if mv.any() else len(mv) - 1
        _still[key] = round(float((t[i0 + j] - t[i0]) / 1e9), 2)
    return _still[key]


def blank_err(rec, P, t0_rel, dur):
    al = json.load(open(f"recordings/{rec}/imu_gt/align.json"))
    dt_ns = int(al["dt_ns"])
    gd = vb.pick_gt(Path(f"recordings/{rec}"), ar.GT)
    g = dict(np.load(gd / "gt.npz", allow_pickle=True))
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    gt_t, gt_p = g["t_ns"][ok], g["p_bc"][ok]
    tr = [m for m in P if m.get("status") == "tracking" and m.get("T_world_cam0")]
    if len(tr) < 50:
        return None
    t_first = int(P[0]["t_ns"])
    vt = np.array([int(m["t_ns"]) for m in tr]) + dt_ns
    T = np.array([np.asarray(m["T_world_cam0"], float).reshape(4, 4) for m in tr])
    j = np.clip(np.searchsorted(gt_t, vt), 1, len(gt_t) - 1)
    j = np.where(np.abs(gt_t[j - 1] - vt) < np.abs(gt_t[j] - vt), j - 1, j)
    m = np.abs(gt_t[j] - vt) < 2_000_000
    rel = (vt - dt_ns - t_first) / 1e9
    Pv, Pg, rel = T[m, :3, 3], gt_p[j[m]], rel[m]
    b0, b1 = t0_rel, t0_rel + dur
    fit = (rel < b0) | (rel > b1 + 3.0)
    if fit.sum() < 50:
        return None
    R, _, t = vb.umeyama(Pv[fit], Pg[fit])
    e = np.linalg.norm(Pv @ R.T + t - Pg, axis=1)
    w = (rel >= b0) & (rel <= b1 + 3.0)
    after = rel > b1 + 3.0
    before = rel < b0
    rms = lambda x: round(float(np.sqrt(np.mean(x ** 2))) * 100, 2) if len(x) else None
    ate_after_self = None
    if after.sum() > 50:
        R2, _, t2 = vb.umeyama(Pv[after], Pg[after])
        ate_after_self = rms(np.linalg.norm(Pv[after] @ R2.T + t2 - Pg[after], axis=1))
    return dict(max_err_blank_cm=round(float(e[w].max()) * 100, 2) if w.any() else None,
                ate_before_cm=rms(e[before]), ate_after_cm=rms(e[after]), ate_after_self_cm=ate_after_self,
                ate_all_cm=rms(e[fit]), n=int(m.sum()))


def main():
    conds = (sys.argv[1] if len(sys.argv) > 1 else "H,SI,BL1,BL2,BLR1,BLR2,SIR").split(",")
    allc, drop, old = ar.cells()
    rows = []
    for rec, off in allc:
        key = f"{rec[-6:]}_o{off}"
        row = dict(rec=rec, off=off, key=key)
        b0p = ar.OVOPT_B0 / f"{key}_B0.poses.jsonl.gz"
        B0 = poses(b0p) if b0p.exists() else None
        if B0:
            row["B0_hash"] = phash(B0)
            tr = [m for m in B0 if m.get("status") == "tracking"]
            row["B0_init_s"] = round((int(tr[0]["t_ns"]) - int(B0[0]["t_ns"])) / 1e9, 2) if tr else None
            try:
                row["still_s"] = still_s(rec, int(B0[0]["t_ns"]), old)
            except Exception as e:  # noqa: BLE001
                row["still_s"] = f"err {e}"
        for c in conds:
            f = OUT / c / f"{key}_{c}.poses.jsonl.gz"
            if not f.exists():
                continue
            P = poses(f)
            tr = [m for m in P if m.get("status") == "tracking"]
            hs = [m["h"]["state"] for m in P if "h" in m]
            d = dict(hash=phash(P), init_s=round((int(tr[0]["t_ns"]) - int(P[0]["t_ns"])) / 1e9, 2) if tr else None,
                     health={s: round(hs.count(s) / max(1, len(hs)), 4) for s in ("INIT", "OK", "DEGRADED", "LOST")},
                     segs=max([m.get("seg", 0) for m in P] or [0]), n_poses=len(P))
            if B0:
                d["same_as_B0"] = d["hash"] == row["B0_hash"]
            if c.startswith("BL"):
                spec = ar.CONDS[c]["args"].split("--test-blank ")[1].split()[0]
                t0b, dur = (float(x) for x in spec.split(":"))
                d["blank"] = blank_err(rec, P, t0b, dur)
                fh = OUT / "H" / f"{key}_H.poses.jsonl.gz"
                if fh.exists():
                    row.setdefault("H_blank_ref", {})[f"{dur:g}"] = blank_err(rec, poses(fh), t0b, dur)
            row[c] = d
        rows.append(row)
    use = [r for r in rows if r["rec"] != drop]
    summ = {}
    for c in conds:
        cc = [r for r in use if c in r]
        if not cc:
            continue
        iv = [r[c]["init_s"] for r in cc if r[c]["init_s"] is not None]
        s = dict(n=len(cc), init_s_med=round(float(np.median(iv)), 2) if iv else None,
                 init_s_max=round(float(np.max(iv)), 2) if iv else None, never_init=sum(r[c]["init_s"] is None for r in cc),
                 same_as_B0=sum(bool(r[c].get("same_as_B0")) for r in cc),
                 health_med={k: round(float(np.median([r[c]["health"][k] for r in cc])), 4) for k in ("INIT", "OK", "DEGRADED", "LOST")},
                 segs_cells=sum(r[c]["segs"] > 0 for r in cc))
        if c.startswith("BL"):
            # cells that initialize later than 2 s before the blank (214138, 221052: init 17-19 s) are excluded: there
            # the blank hits the initialization, not a running filter (BLANK_T0 = 15 s was fixed before this was known)
            okc = [r for r in cc if r.get("B0_init_s") is not None and r["B0_init_s"] <= ar.BLANK_T0 - 2.0]
            s["blank_cells_used"] = len(okc)
            s["blank_cells_excluded"] = [r["key"] for r in cc if r not in okc]
            bl = [r[c]["blank"] for r in okc if r[c].get("blank")]
            for k in ("max_err_blank_cm", "ate_after_cm", "ate_after_self_cm", "ate_all_cm"):
                v = [b[k] for b in bl if b.get(k) is not None]
                s[k + "_med"] = round(float(np.median(v)), 2) if v else None
                s[k + "_max"] = round(float(np.max(v)), 2) if v else None
            s["after_gt_10cm"] = sum(1 for b in bl if (b.get("ate_after_cm") or 0) > 10)
        summ[c] = s
    for dur in ("1", "2"):
        ref = [r["H_blank_ref"][dur] for r in use if r.get("H_blank_ref", {}).get(dur)
               and r.get("B0_init_s") is not None and r["B0_init_s"] <= ar.BLANK_T0 - 2.0]
        if ref:
            summ[f"H_ref_blank{dur}"] = {k + "_med": round(float(np.median([b[k] for b in ref if b.get(k) is not None])), 2)
                                         for k in ("max_err_blank_cm", "ate_after_cm", "ate_after_self_cm", "ate_all_cm")}
    out = dict(made=time.strftime("%F %T"), conds=conds, summary=summ, cells=rows, doc=__doc__)
    tag = "_".join(conds)
    json.dump(out, open(OUT / f"ana_{tag}.json", "w"), indent=1)
    print(json.dumps(summ, indent=1))
    print("| cell | still s | B0 init s | " + " | ".join(f"{c} init s" for c in conds) + " |")
    for r in rows:
        cols = []
        for c in conds:
            if c not in r:
                cols.append("-")
                continue
            x = str(r[c]["init_s"])
            if c == "H" and not r[c].get("same_as_B0", True):
                x += " DIFF"
            cols.append(x)
        print(f"| {r['key']} | {r.get('still_s')} | {r.get('B0_init_s')} | " + " | ".join(cols) + " |")
    print("saved", OUT / f"ana_{tag}.json")


if __name__ == "__main__":
    main()
