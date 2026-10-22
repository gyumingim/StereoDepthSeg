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


def dl_mean(s, sd):
    """random effects (DerSimonian-Laird): s_i = mean + systematic (sd tau, recording-to-recording) + bootstrap noise.
    Returns mean, se, tau."""
    s, sd = np.asarray(s, float), np.maximum(np.asarray(sd, float), 1e-4)
    w = 1 / sd ** 2
    m0 = (w * s).sum() / w.sum()
    Q = float((w * (s - m0) ** 2).sum())
    tau2 = max(0.0, (Q - (len(s) - 1)) / (w.sum() - (w ** 2).sum() / w.sum())) if len(s) > 1 else 0.0
    ws = 1 / (sd ** 2 + tau2)
    return float((ws * s).sum() / ws.sum()), float(1 / np.sqrt(ws.sum())), float(np.sqrt(tau2))


def ref_scale(gt_dir="imu_gt_rs", key="s"):
    """method bias on the 10-19 monitor recordings; key 's' (old, no lever) or 's_fixedlever' (2026-10-22)"""
    r = jload(OD / f"scale_ref_monitor_{gt_dir}.json")
    r = [x for x in (r or []) if key in x]
    if not r:
        return None
    s = np.array([x[key] for x in r])
    m, se, tau = dl_mean(s, [x["s_boot_std" if key == "s" else key + "_boot_std"] for x in r])
    return dict(gt_dir=gt_dir, key=key, n=len(s), ref_s=round(m, 4), ref_se=round(se, 4),
                median=round(float(np.median(s)), 4), between_sd=round(tau, 4),
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
    if al and al.get("method", "").startswith("borrowed"):
        st["steps"].append("batch_sync(borrowed)")
        W.append("alignment: " + al["method"])
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
            for k in ("s_lever", "s_lever_ci95", "lever_fit_cm", "s_fixedlever", "s_fixedlever_ci95",
                      "s_fixedlever_boot_std", "lever_fixed_cm", "check", "check_uses"):
                if k in x:
                    e[k] = x[k]
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


def summary_by_board(recs, ref=None):
    """2026-10-22: pooled per board config (name + GT square), on s_fixedlever (lever arm from config/imu_cam_lever.json;
    spl GT, rs if no spl). The old summary() mixed boards (laptop 42.86, laptop_fit 32.03, monitor) in one mean."""
    groups = {}
    for k, v in recs.items():
        sc = v.get("scale") or {}
        e = sc.get("spl") if "s_fixedlever" in (sc.get("spl") or {}) else sc.get("rs")
        if not e or "s_fixedlever" not in e or not e.get("gt_square_mm"):
            continue
        name = (v.get("board") or {}).get("name") or "unknown"
        groups.setdefault(f"{name} {e['gt_square_mm']}", []).append((k, e))
    out = {}
    for g, rows in sorted(groups.items()):
        s = [e["s_fixedlever"] for _, e in rows]
        m, se, tau = dl_mean(s, [e["s_fixedlever_boot_std"] for _, e in rows])
        sq = rows[0][1]["gt_square_mm"]
        o = dict(n=len(rows), s_fixedlever_pooled=round(m, 4), se=round(se, 4), between_sd=round(tau, 4),
                 s_lever_mean=round(float(np.mean([e["s_lever"] for _, e in rows])), 4),
                 s_nolever_mean=round(float(np.mean([e["s"] for _, e in rows])), 4),
                 gt_square_mm=sq, square_mm_imu=round(sq * m, 2), recs=[k for k, _ in rows])
        if ref:
            o.update(ref_s_fixedlever=ref["ref_s"], square_mm_est_refcorr=round(sq * m / ref["ref_s"], 2))
        out[g] = o
    return dict(by_board=out, check_uses="s_fixedlever",
                note="Estimate only. Per-board s differs by 0.6-1.7 % between methods (lever fitted/fixed, q2), and the "
                     "monitor board itself gives s 1.01-1.03, so a ~1-2 % method bias remains "
                     "(runs/ate_floor_review/judge.md). square_mm_imu = gt_square_mm * s_fixedlever (no method-bias "
                     "correction); square_mm_est_refcorr divides by the 10-19 monitor s_fixedlever.")


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
    d["reference"] = dict(rs=ref_scale("imu_gt_rs"), spl=ref_scale("imu_gt_spl"),
                          spl_fixedlever=ref_scale("imu_gt_spl", "s_fixedlever"))
    d.pop("square_consistency", None)                # old mixed-board pool (no lever), replaced 2026-10-22
    d["square_by_board"] = summary_by_board(d["recordings"], d["reference"]["spl_fixedlever"])
    d["updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
    json.dump(d, open(OUT, "w"), indent=1, ensure_ascii=False)
    rf = d["reference"]["spl_fixedlever"]
    for g, o in d["square_by_board"]["by_board"].items():
        print(f"pooled square [{g}] s_fixedlever {o['s_fixedlever_pooled']} ± {o['se']} (between sd {o['between_sd']}, "
              f"n {o['n']}; s_lever mean {o['s_lever_mean']}, old s mean {o['s_nolever_mean']}) -> IMU {o['square_mm_imu']} mm"
              + (f", /monitor-ref {rf['ref_s']} -> {o['square_mm_est_refcorr']} mm" if rf else "") + " (estimate, method bias ~1-2 %)")


if __name__ == "__main__":
    main()
