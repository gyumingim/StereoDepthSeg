#!/usr/bin/env python3
"""Overnight GT status: per recording (steps done, corners/frame, batch_sync, spline corner px, IMU scale, still start,
warnings) + across recordings (square-size consistency) → runs/overnight_gt_status.json. Read-only on the recordings.
  ../venv_ffs/bin/python imu_learn/gt_status.py recordings/hs_A [...]      (adds/updates those entries)
Method bias of the scale check: imu_learn/scale_ls.py on the monitor recordings (known square 63.403 mm)
→ runs/overnight_gt/scale_ref_monitor_imu_gt_rs.json (inverse-variance mean = ref_s, between-recording sd = systematic).
"""
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
sys.path.insert(0, str(HERE))
from imu_gt import read_imu  # noqa: E402

OUT = SCAN3D / "runs" / "overnight_gt_status.json"
OD = SCAN3D / "runs" / "overnight_gt"


def jload(p):
    try:
        return json.load(open(p))
    except (OSError, ValueError):
        return None


def ref_scale(gt_dir="imu_gt_rs"):
    r = jload(OD / f"scale_ref_monitor_{gt_dir}.json")
    if not r:
        return None
    s = np.array([x["s"] for x in r if "s" in x])
    sd = np.array([max(x["s_boot_std"], 1e-4) for x in r if "s" in x])
    # random effects (DerSimonian-Laird): s_i = ref + systematic (sd tau, recording-to-recording) + bootstrap noise
    w = 1 / sd ** 2
    m0 = (w * s).sum() / w.sum()
    Q = float((w * (s - m0) ** 2).sum())
    tau2 = max(0.0, (Q - (len(s) - 1)) / (w.sum() - (w ** 2).sum() / w.sum()))
    ws = 1 / (sd ** 2 + tau2)
    m = float((ws * s).sum() / ws.sum())
    return dict(gt_dir=gt_dir, n=len(s), ref_s=round(m, 4), ref_se=round(float(1 / np.sqrt(ws.sum())), 4),
                median=round(float(np.median(s)), 4), between_sd=round(float(np.sqrt(tau2)), 4),
                min=round(float(s.min()), 4), max=round(float(s.max()), 4), method="DerSimonian-Laird random effects",
                recs=[x["rec"] for x in r])


def still_start(rec, raw):
    """Initial still period from the IMU (0.25 s bins: gyro std < 0.02, |ω| < 0.05 rad/s, |a| std < 0.05 m/s²)."""
    acc, gyr = read_imu(rec / "imu.csv", raw=raw)
    t0 = gyr[0, 0]
    tg = (gyr[:, 0] - t0) / 1e9
    ta = (acc[:, 0] - t0) / 1e9
    w = gyr[:, 1:4] - (np.median(gyr[:200, 1:4], 0) if raw else 0)
    an = np.linalg.norm(acc[:, 1:4], axis=1)
    b = 0.25
    n = int(tg[-1] / b)
    flags = []                                           # per bin: True still, False moving, None no data (IMU gap)
    for k in range(n):
        mg = (tg >= k * b) & (tg < (k + 1) * b)
        ma = (ta >= k * b) & (ta < (k + 1) * b)
        if mg.sum() < 20 or ma.sum() < 20:
            flags.append(None)
            continue
        flags.append(bool(w[mg].std(0).max() < 0.02 and np.linalg.norm(w[mg], axis=1).mean() < 0.05 and an[ma].std() < 0.05))
    # first still run starting in the first 2 s (gap bins inside a run do not end it)
    k0 = next((k for k, f in enumerate(flags[:8]) if f), None)
    still, still_end = 0.0, 0.0
    if k0 is not None:
        k = k0
        while k < len(flags) and flags[k] is not False:
            k += 1
        still_end = k * b
        still = still_end - k0 * b
    # still that overlaps the video (IMU starts ~1.5-2 s before the first frame)
    rows = list(csv.DictReader(open(rec / "frames_cam5.csv")))
    tv = (int(rows[0]["t_mid_ns"]) - t0) / 1e9 if rows else 0.0
    tm = np.array([int(r["t_mid_ns"]) for r in rows]) if rows else np.zeros(1)
    d = np.diff(tm) / 1e6
    return dict(still_start_s=round(still, 2), video_starts_at_imu_s=round(tv, 2),
                still_end_s=round(still_end, 2),
                still_overlapping_video_s=round(max(0.0, still_end - tv), 2)), \
        dict(frames=len(rows), dur_s=round((tm[-1] - tm[0]) / 1e9, 1) if len(tm) > 1 else 0,
             gaps_gt6ms=int((d > 6).sum()), missing_est=int(np.round(d[d > 6] / np.median(d) - 1).sum()) if len(d) else 0,
             max_gap_ms=round(float(d.max()), 1) if len(d) else 0)


def corner_stats(gp):
    g = np.load(gp, allow_pickle=True)
    n = g["n_corners"]
    ok = g["ok"].astype(bool)
    out = dict(frames=int(len(ok)), ok=int(ok.sum()), ok_frac=round(float(ok.mean()), 3),
               corners_median_all=float(np.median(n)), corners_median_ok=float(np.median(n[ok])) if ok.any() else 0,
               frac_ge8=round(float((n >= 8).mean()), 3), frac_ge16=round(float((n >= 16).mean()), 3),
               rms_px_median=round(float(np.nanmedian(g["rms_px"][ok])), 3) if ok.any() else None,
               dist_m_median=round(float(np.median(np.linalg.norm(g["p_bc"][ok], axis=1))), 3) if ok.any() else None,
               square_mm=float(g["square_mm"]) if "square_mm" in g else None)
    if "corner_ids" in g:
        ids = np.concatenate([np.asarray(x).ravel() for x in g["corner_ids"] if len(x)]) if len(g["corner_ids"]) else []
        out["corner_ids_seen"] = int(len(np.unique(ids)))
    return out


def one(rec):
    rec = Path(rec)
    st = dict(rec=rec.name, updated=time.strftime("%Y-%m-%d %H:%M:%S"), steps=[], warnings=[])
    W = st["warnings"]
    al = jload(rec / "imu_gt" / "align.json")
    if (rec / "imu_gt" / "gt.npz").exists() and al:
        st["steps"].append("imu_gt")
        st["imu"] = "raw35/16" if al.get("imu_raw") else "calibrated1/4"
        st["imu_gt"] = corner_stats(rec / "imu_gt" / "gt.npz")
        st["align_initial"] = dict(dt_ns=al.get("dt_ns_inc", al.get("dt_ns")), omega_rms=al.get("omega_rms"))
    bi = jload(rec / "imu_gt" / "board.json")
    if bi:
        st["board"] = dict(name=bi.get("board_name"), square_mm=bi.get("square_mm"), source=bi.get("square_mm_source"),
                           estimated=bi.get("square_estimated"), code=bi.get("git_head"))
    tr = jload(rec / "imu_gt" / "trim.json")
    if (OD / f"{rec.name}_board_shift.log").exists():
        st["steps"].append("board_shift")
        st["trim"] = dict(start_ns=tr["start_ns"], shifts=tr.get("shifts")) if tr else None
        if tr:
            W.append(f"trim.json: GT before {tr['start_ns']} ignored ({tr.get('reason')})")
        import re
        mm = re.search(r"(\d+) board shift\(s\)", open(OD / f"{rec.name}_board_shift.log").read())
        st["board_shift_candidates"] = int(mm.group(1)) if mm else None
        if mm and int(mm.group(1)):
            W.append(f"board_shift found {mm.group(1)} possible board move(s) (not trimmed — check)")
    if al and al.get("method", "").startswith("whole-recording"):
        st["steps"].append("batch_sync")
        st["batch_sync"] = dict(dt_ms=round(al["dt_ns"] / 1e6, 2), frame_res_deg_med=round(al.get("frame_res_deg", np.nan), 4),
                                frame_res_deg_p90=al.get("frame_res_deg_p90"), inc_res_deg=al.get("inc_res_deg"),
                                bias_mean=np.round(np.mean(al["gyro_bias_knots"], 0), 5).tolist(),
                                dt_change_ms=round((al["dt_ns"] - al.get("dt_ns_inc", al["dt_ns"])) / 1e6, 2))
        if al.get("frame_res_deg", 0) > 0.5:
            W.append(f"batch_sync frame residual {al['frame_res_deg']:.2f}° (> 0.5°)")
    if (rec / "imu_gt_rs" / "gt.npz").exists():
        st["steps"].append("imu_gt_rs")
        st["imu_gt_rs"] = c = corner_stats(rec / "imu_gt_rs" / "gt.npz")
        if c["ok_frac"] < 0.5:
            W.append(f"only {c['ok_frac'] * 100:.0f} % frames with a trusted pose")
        if c["corners_median_ok"] and c["corners_median_ok"] < 14:
            W.append(f"few corners per frame (median {c['corners_median_ok']:.0f} of 32) — partial board, PnP tilt less stable")
    sb = jload(rec / "imu_gt_spl" / "board.json")
    if (rec / "imu_gt_spl" / "gt.npz").exists():
        st["steps"].append("spline")
        g = np.load(rec / "imu_gt_spl" / "gt.npz", allow_pickle=True)
        st["spline"] = dict(knot=float(g["knot"]), ok=int(g["ok"].sum()), frames=int(len(g["ok"])),
                            corner_px_median=sb.get("corner_px_median") if sb else None,
                            corner_px_p90=sb.get("corner_px_p90") if sb else None,
                            square_mm=float(g["square_mm"]) if "square_mm" in g else None)
        if sb and sb.get("corner_px_median", 0) > 0.6:
            W.append(f"spline corner residual median {sb['corner_px_median']:.2f} px (> 0.6)")
    pv = {}
    for gd in ("imu_gt", "imu_gt_rs", "imu_gt_spl"):
        b = jload(rec / gd / "board.json")
        if b:
            pv[gd] = dict(git_head=b.get("git_head"), git_dirty=b.get("git_dirty"), code=b.get("code_sha256"), made=b.get("made"))
    if al and al.get("batch_sync_provenance"):
        bp = al["batch_sync_provenance"]
        pv["batch_sync"] = dict(git_head=bp.get("git_head"), git_dirty=bp.get("git_dirty"), code=bp.get("code_sha256"), made=bp.get("made"))
    tl = OD / f"{rec.name}_board_shift.log"
    pv["trim_used"] = (rec / "imu_gt" / "trim.json").exists()
    st["pipeline"] = pv
    sc = {}
    for gd in ("rs", "spl"):
        r = jload(OD / f"{rec.name}_scale_{gd}.json")
        if r and "s" in r[0]:
            ref = ref_scale("imu_gt_" + gd if gd == "rs" else "imu_gt_spl")
            x = r[0]
            e = dict(s=x["s"], s_ci95=x["s_ci95"], windows=x["windows"], g_norm=x["g_norm"], res_cm=x["res_cm"],
                     acc_dyn_rms_med=x["acc_dyn_rms_med"], gt_square_mm=x["gt_square_mm"], imu=x["imu"])
            if ref and x["gt_square_mm"]:
                q = x["gt_square_mm"] / ref["ref_s"]
                e.update(ref_s=ref["ref_s"], ref_between_sd=ref["between_sd"],
                         square_mm_est=round(x["s"] * q, 2), square_mm_ci95_stat=[round(v * q, 2) for v in x["s_ci95"]],
                         square_mm_sd_total=round(x["gt_square_mm"] * float(np.hypot(x["s_boot_std"], ref["between_sd"])) / ref["ref_s"], 2))
            sc[gd] = e
        elif r:
            sc[gd] = r[0]
    if sc:
        st["steps"].append("scale")
        st["scale"] = sc
    try:
        ss, fr = still_start(rec, bool(al and al.get("imu_raw")))
        st.update(ss)
        st["frames"] = fr
        if ss["still_overlapping_video_s"] < 2:
            W.append(f"no ≥2 s still at the start of the video ({ss['still_overlapping_video_s']} s) — strict still init not testable")
    except Exception as ex:  # noqa: BLE001
        W.append(f"still/frames stats failed: {ex}")
    meta = jload(rec / "meta.json") or {}
    st["app_version"] = meta.get("app_version")
    st["stop_reason"] = meta.get("stop_reason")
    st["frames_unmatched_app"] = meta.get("frames_unmatched")
    if meta.get("app_version") and "nroff" not in meta.get("app_version", "") and "still" not in meta.get("app_version", ""):
        W.append(f"app {meta.get('app_version')}: noise reduction / edge enhancement ON (FAST)")
    if meta.get("frames_unmatched"):
        W.append(f"{meta['frames_unmatched']} frames matched by the app with a borrowed offset (hs_fix_ts not run: results "
                 "has 2 rows/frame; error ≤ tens of µs)")
    W.append("square size is an IMU estimate (board_laptop_6x6.json) — positions scale with it; fix with rescale_gt.py")
    return st


def summary(recs, ref=None):
    rows = []
    for k, v in recs.items():
        e = (v.get("scale") or {}).get("rs")
        if e and "square_mm_est" in e:
            rows.append((k, e["square_mm_est"], e["square_mm_sd_total"], e["square_mm_ci95_stat"]))
    if not rows:
        return {}
    x = np.array([r[1] for r in rows])
    sd = np.array([max(r[2], 1e-3) for r in rows])
    w = 1 / sd ** 2
    m = float((w * x).sum() / w.sum())
    se_stat = float(1 / np.sqrt(w.sum()))
    ref_rel = (ref["ref_se"] / ref["ref_s"]) if ref else 0.0          # common to all recordings (same reference)
    se = float(np.hypot(se_stat, m * ref_rel))
    chi2 = float((((x - m) / sd) ** 2).sum())
    flag = [r[0] for r, xi, si in zip(rows, x, sd) if abs(xi - m) > 2 * si]
    return dict(square_mm_pooled=round(m, 2), square_mm_pooled_se=round(se, 2), square_mm_pooled_ci95=[round(m - 1.96 * se, 2), round(m + 1.96 * se, 2)],
                se_recordings=round(se_stat, 2), se_reference=round(m * ref_rel, 2), n=len(rows),
                chi2=round(chi2, 2), dof=len(rows) - 1, disagree=flag,
                per_rec={r[0]: dict(square_mm_est=r[1], sd_total=r[2], ci95_stat=r[3]) for r in rows},
                note="sd_total = bootstrap sd (within recording) ⊕ between-recording sd of the method on the monitor "
                     "recordings (known square). Estimate only — measure the screen width (square = W / 7.07).")


def main():
    d = jload(OUT) or {}
    d.setdefault("recordings", {})
    for r in sys.argv[1:]:
        st = one(r)
        d["recordings"][st["rec"]] = st
        print(f"{st['rec']}: steps {','.join(st['steps'])}; " +
              (f"spline px {st.get('spline', {}).get('corner_px_median')}; " if "spline" in st else "") +
              (f"square est {st['scale']['rs'].get('square_mm_est')} mm; " if st.get("scale", {}).get("rs") else "") +
              f"still {st.get('still_overlapping_video_s')} s; warnings {len(st['warnings'])}")
    d["reference"] = dict(rs=ref_scale("imu_gt_rs"), spl=ref_scale("imu_gt_spl"))
    d["square_consistency"] = summary(d["recordings"], d["reference"]["rs"])
    d["updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
    json.dump(d, open(OUT, "w"), indent=1, ensure_ascii=False)
    sc = d["square_consistency"]
    if sc:
        print(f"pooled square {sc['square_mm_pooled']} ± {sc['square_mm_pooled_se']} mm over {sc['n']} rec, "
              f"chi2 {sc['chi2']}/{sc['dof']}, disagree {sc['disagree']}")


if __name__ == "__main__":
    main()
