#!/usr/bin/env python3
"""Post-recording auto check (night 11-27, item 4). CPU only. Thresholds come from judge_criteria.json and are PROPOSALS,
not validated on a good (new-method) recording.

  python postrec_check.py plan --plan PLAN.json --rec REC --vio VIO.jsonl.gz --scan SCANDIR --out OUT.json
        stage A (before FFS, minutes): strict partners per keyframe (>= 2 share, 0 share, fallback share), zero-partner time ranges,
        VIO tracking share + fast rotation time, keyframe count / spacing / kf-win vs fps, VIO config track_frequency vs fps,
        covisibility check active?, pair profile (new = low-angle backdoors off). -> PASS / FAIL (+ warnings).
  python postrec_check.py ffs --pairs PAIRS_DIR --out OUT.json [--plan PLAN.json]
        stage B (after FFS): within-keyframe partner disagreement in disparity px: for pair p, |f B_p sin_p (1/Z_p - 1/Z_ref)| with
        Z_ref = per-pixel median of the OTHER pairs of the keyframe; per-keyframe median over pairs. Share of keyframes > line (0.3 px).
  python postrec_check.py validate121801 --out OUT.json
        sanity: the same disagreement on 121801 by pair class (base strict / base fallback / low / relax-wide) vs strict consensus,
        and stage A on 121801's official plan (must FAIL: strict >= 2 is 75 %, 0 partners 17 %).
"""
import argparse, gzip, json, sys, time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
CRIT = HERE / "judge_criteria.json"


def crit():
    return json.loads(CRIT.read_text())


def vio_poses(p):
    t, T, st = [], [], []
    for line in gzip.open(p, "rt"):
        d = json.loads(line)
        if d.get("type") == "pose":
            t.append(int(d["t_ns"])); st.append(d.get("status")); T.append(np.array(d["T_world_cam0"], float).reshape(4, 4) if d.get("T_world_cam0") else np.full((4, 4), np.nan))
    return np.array(t), np.array(T), np.array(st, dtype=object)


def ranges(idx_sorted, t_s, gap=1.0):
    out = []
    for i, ts in zip(idx_sorted, t_s):
        if out and ts - out[-1][1] <= gap:
            out[-1][1] = ts; out[-1][2] += 1
        else:
            out.append([ts, ts, 1])
    return [dict(t0_s=round(a, 2), t1_s=round(b, 2), n_kf=n) for a, b, n in out]


def stage_a(plan_json, rec, vio, scan, C):
    P = json.loads(Path(plan_json).read_text()); ch = P["chosen"]; n = ch["n_kf"]; pl = {int(k): v for k, v in ch["plan"].items()}
    meta = json.loads((Path(rec) / "meta.json").read_text()); fps = float(meta.get("save_fps") or meta.get("fps"))
    strict = {k: [q for q in v if q[1] != "fallback"] for k, v in pl.items()}
    n2 = sum(len(v) >= 2 for v in strict.values()); n1 = sum(len(v) >= 1 for v in strict.values()); n0 = n - n1
    pairs = sum(len(v) for v in pl.values()); fb = sum(q[1] == "fallback" for v in pl.values() for q in v)
    # keyframe list: from the scan manifest (all keyframes, also those without partners)
    man = None
    for m_ in Path(scan).glob("red/*_kf/manifest.json"):
        man = json.loads(m_.read_text())
    g = np.load(Path(rec) / "lensonly" / "gt.npz"); t_ns = g["t_ns"]; t0 = float(t_ns[0])
    kfl = [int(x) for x in man["keyframes"]] if man else sorted(pl)
    zero = [f for f in kfl if len(strict.get(f, [])) == 0]
    zr = ranges(zero, [(t_ns[f] - t0) * 1e-9 for f in zero])
    # VIO
    tv, Tv, st = vio_poses(vio); trk = st == "tracking"
    ok = trk & np.isfinite(Tv[:, 0, 0]); tt = tv[ok] * 1e-9; R = Tv[ok, :3, :3]
    w = []
    step = max(1, int(round(0.5 * fps)))
    for i in range(0, len(R) - step, step):
        dR = R[i].T @ R[i + step]; ang = np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))); dt = tt[i + step] - tt[i]
        if dt > 0:
            w.append(ang / dt)
    w = np.array(w); fast = float((w > C["stage_a"]["fast_rot_deg_s"]).mean()) if len(w) else None
    ks = (man or {}).get("kf_select", {}) or {}
    kf_win_expect = max(1, int(round(4 * fps / 60)))
    summ = json.loads((Path(scan) / "scan_summary.json").read_text()) if (Path(scan) / "scan_summary.json").exists() else {}
    cfg = summ.get("vio_cfg"); trk_hz = None
    if cfg and (Path(cfg) / "estimator_config.yaml").exists():
        for line in (Path(cfg) / "estimator_config.yaml").read_text().splitlines():
            if line.strip().startswith("track_frequency:"):
                trk_hz = float(line.split(":")[1])
    dur = (t_ns[-1] - t_ns[0]) * 1e-9
    covis = P.get("whole_video", {}).get("covis_pts_src", {})
    r = dict(n_kf=n, kf_in_manifest=len(kfl), duration_s=round(dur, 1), fps=fps, kf_per_s=round(len(kfl) / max(dur, 1e-9), 2),
             strict_ge2_pct=round(100 * n2 / n, 1), strict_ge1_pct=round(100 * n1 / n, 1), strict_zero_pct=round(100 * n0 / n, 1),
             pairs=pairs, fallback_pairs=fb, fallback_pct=round(100 * fb / max(pairs, 1), 1), zero_ranges=zr[:40],
             vio_tracked_frac=round(float(trk.mean()), 3), rot_speed_med_deg_s=float(np.median(w)) if len(w) else None,
             rot_fast_time_frac=fast, kf_select=ks, kf_win=ks.get("win"), kf_win_expected=kf_win_expect,
             vio_cfg=cfg, vio_track_hz=trk_hz, profile=P.get("rate", {}).get("profile"), rate=P.get("rate"), covis_pts_src=covis)
    c = C["stage_a"]; fails, warns = [], []
    if r["strict_ge2_pct"] < c["strict_ge2_min_pct"]:
        fails.append(f"strict >= 2 partners {r['strict_ge2_pct']} % < {c['strict_ge2_min_pct']} %")
    if r["strict_zero_pct"] > c["zero_max_pct"]:
        fails.append(f"0 strict partners {r['strict_zero_pct']} % > {c['zero_max_pct']} %")
    if r["vio_tracked_frac"] < c["vio_tracked_min"]:
        fails.append(f"VIO tracked {r['vio_tracked_frac']} < {c['vio_tracked_min']}")
    if fb > 0 and r["profile"] == "new":
        fails.append("fallback (low-angle) pairs present under profile new")
    if r["profile"] != "new":
        warns.append(f"pair profile is {r['profile']} (low-angle backdoors ON); new recordings should use --profile new")
    if fast is not None and fast > c["fast_rot_time_max"]:
        warns.append(f"rotation > {c['fast_rot_deg_s']} deg/s for {100 * fast:.0f} % of the time")
    if r["kf_win"] is not None and r["kf_win"] != kf_win_expect:
        warns.append(f"kf-win {r['kf_win']} but {kf_win_expect} expected at {fps:g} fps (keyframes drop ~30 % with the 60 fps window at 30 fps)")
    if trk_hz is not None and abs(trk_hz - fps) > 1:
        warns.append(f"VIO config track_frequency {trk_hz:g} Hz != recording {fps:g} fps (check the chosen config)")
    if not covis or set(covis) <= {"none"}:
        warns.append("covisibility check inactive (no prior depth for this run: every partner passes the 60 % covis test)")
    r.update(PASS=not fails, fails=fails, warnings=warns, thresholds=c, note="thresholds = proposal, not validated on a good recording")
    return r


def disagreement(Zs, Bs, fs, sins, ref_mask=None):
    """per pair p: median |f B sin (1/Z_p - 1/Z_ref)| over px where p and >= 1 other (ref) pair are valid. Z_ref = median of others."""
    m = len(Zs); out = []
    V = np.isfinite(Zs) & (Zs > 0.05)
    for p in range(m):
        oth = [q for q in range(m) if q != p and (ref_mask is None or ref_mask[q])]
        if not oth:
            out.append(None); continue
        with np.errstate(all="ignore"):
            Zr = np.nanmedian(np.where(V[oth], Zs[oth], np.nan), 0)
        ok = V[p] & np.isfinite(Zr)
        if ok.sum() < 500:
            out.append(None); continue
        s = sins[p][ok] if np.ndim(sins[p]) else sins[p]
        d = np.abs(fs[p] * Bs[p] * s * (1.0 / Zs[p][ok] - 1.0 / Zr[ok]))
        out.append(dict(med_px=float(np.median(d)), p90_px=float(np.percentile(d, 90)), n_px=int(ok.sum())))
    return out


def stage_b(pairs_dir, C):
    rows = []
    for f in sorted(Path(pairs_dir).glob("*.npz")):
        d = np.load(f); Z = d["Z"].astype(np.float32); m = len(d["B"])
        if m < 2:
            rows.append(dict(kf=int(f.stem), pairs=m, med_px=None)); continue
        sins = np.sin(np.radians(d["perp"])) if "perp" in d.files else np.ones(m)
        dis = disagreement(Z, d["B"], d["f"], sins)
        v = [x["med_px"] for x in dis if x]
        rows.append(dict(kf=int(f.stem), pairs=m, med_px=float(np.median(v)) if v else None, per_pair=[x["med_px"] if x else None for x in dis],
                         src=[str(s) for s in d["src"]] if "src" in d.files else None))
    line = C["stage_b"]["kf_disagree_px_line"]; vals = [r["med_px"] for r in rows if r["med_px"] is not None]
    share = float(np.mean([v > line for v in vals])) if vals else None
    r = dict(n_kf=len(rows), n_kf_multi=len(vals), kf_med_px_median=float(np.median(vals)) if vals else None,
             kf_med_px_p90=float(np.percentile(vals, 90)) if vals else None, share_kf_over_line=share, line_px=line, per_kf=rows)
    fails = []
    if share is None:
        fails.append("no keyframe with >= 2 pairs")
    elif share > C["stage_b"]["share_kf_over_line_max"]:
        fails.append(f"{100 * share:.0f} % of keyframes disagree > {line} px (max {100 * C['stage_b']['share_kf_over_line_max']:.0f} %)")
    r.update(PASS=not fails, fails=fails, note="line and share = proposal, not validated on a good recording")
    return r


def validate121801(out):
    V2 = Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2"
    sys.path.insert(0, str(V2.parent.parent / "gsmap")); sys.path.insert(0, str(V2 / "diag_plane_20261127/scripts"))
    import gsm_ffs_4m as M
    c = np.load(V2 / "ba_partners2/run/dump_ba/cams.npz"); kf = c["kf"]; W, H = [int(v) for v in c["size"]]
    import gsm_ba_partners as BP
    g = np.load(Path(BP.R1) / "rec" / "lensonly" / "gt.npz"); Kfull = g["K"].copy(); Kfull[:2] *= W / 1920.0
    base_c = json.loads((V2 / "ffs_relax/src/plan_4m.json").read_text())["base_c"]
    P2D = V2 / "ba_partners/ffs_pairs2/pairs"; P3D = V2 / "ba_partners/ffs_pairs3/extra"; SRC = V2 / "ffs_relax/src/wide"
    acc = {}; t0 = time.time(); nk = 0
    rng = np.random.default_rng(0); idx = list(range(len(kf)))
    for ci in idx:
        fi = int(kf[ci]); L = []
        fb = P2D / f"{fi:05d}.npz"
        if not fb.exists():
            continue
        d = np.load(fb); cmap = {int(r[0]): r[1:] for r in base_c.get(str(fi), [])}
        srcs = [str(s) for s in d["src"]] if "src" in d.files else ["?"] * len(d["B"])
        for p in range(len(d["B"])):
            j = int(d["partner"][p])
            if j not in cmap:
                continue
            L.append((d["Z"][p].astype(np.float32), float(d["f"][p]), float(d["B"][p]), M.sin_map(Kfull, cmap[j], W, H).astype(np.float32),
                      "base_fb" if srcs[p] == "fallback" else "base"))
        for tag, fe in (("low", P3D / f"{fi:05d}.npz"), ("relax_wide", SRC / f"{fi:05d}.npz")):
            if fe.exists():
                e = np.load(fe)
                tg = [str(x) for x in e["tag"]] if "tag" in e.files else ["wide?"] * len(e["B"])
                for p in range(len(e["B"])):
                    if tag == "low" and tg[p] != "low":
                        continue
                    L.append((e["Z"][p].astype(np.float32), float(e["f"][p]), float(e["B"][p]), e["sin"][p].astype(np.float32), "low" if tag == "low" else "relax_wide:" + tg[p]))
        strict = [k for k, q in enumerate(L) if q[4] == "base"]
        if len(strict) < 2:
            continue
        nk += 1
        Zs = np.stack([q[0] for q in L]); ref = np.array([q[4] == "base" for q in L])
        dis = disagreement(Zs, [q[2] for q in L], [q[1] for q in L], [q[3] for q in L], ref_mask=ref)
        for q, x in zip(L, dis):
            if x:
                acc.setdefault(q[4], []).append(x["med_px"])
        if nk % 50 == 0:
            print(nk, {k: (len(v), round(float(np.median(v)), 3)) for k, v in acc.items()}, f"{time.time() - t0:.0f}s", flush=True)
    by = {k: dict(n_pairs=len(v), med_px=float(np.median(v)), p25=float(np.percentile(v, 25)), p75=float(np.percentile(v, 75)),
                  share_over_0p3=float(np.mean(np.array(v) > 0.3))) for k, v in acc.items()}
    C = crit()
    sa = stage_a(V2 / "ffs_pairs2/plan.json", V2 / "rec", "/home/system/scanner/StereoDepthSeg-fixes/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan/vio/B_o0.poses.jsonl.gz",
                 V2.parent / "dryrun30_121801_20261127/scan60", C)
    res = dict(made=time.strftime("%F %T"), script=str(Path(__file__).resolve()), n_recordings=1,
               disagreement_by_class=by, n_kf_with_2_strict=nk,
               note="per pair: median |f B sin (1/Z - 1/Z_ref)| px, Z_ref = per-pixel median of the OTHER strict base pairs of the keyframe; "
                    "strict pairs are compared with the other strict pairs, so their number includes the reference's own error",
               stage_a_121801=sa, runtime_s=round(time.time() - t0, 1))
    Path(out).write_text(json.dumps(res, indent=1, default=float))
    print(json.dumps(dict(by=by, stage_a_PASS=sa["PASS"], fails=sa["fails"], warns=sa["warnings"]), indent=1, default=float))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("cmd", choices=["plan", "ffs", "validate121801"])
    ap.add_argument("--plan"); ap.add_argument("--rec"); ap.add_argument("--vio"); ap.add_argument("--scan"); ap.add_argument("--pairs"); ap.add_argument("--out", required=True)
    a = ap.parse_args(); C = crit()
    if a.cmd == "plan":
        r = stage_a(a.plan, a.rec, a.vio, a.scan, C)
    elif a.cmd == "ffs":
        r = stage_b(a.pairs, C)
    else:
        validate121801(a.out); sys.exit(0)
    r.update(made=time.strftime("%F %T"), args=vars(a), criteria_file=str(CRIT))
    Path(a.out).write_text(json.dumps(r, indent=1, default=float))
    print(json.dumps({k: v for k, v in r.items() if k not in ("per_kf", "zero_ranges", "kf_select")}, indent=1, default=float))
    sys.exit(0 if r["PASS"] else 3)
