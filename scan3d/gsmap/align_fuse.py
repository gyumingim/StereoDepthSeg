#!/usr/bin/env python3
"""gsmap 2026-11-25: align AI depth to FFS per keyframe (scale + shift), report residuals, write fused depth.

  python scan3d/gsmap/align_fuse.py --dump DUMP --ai da3=DIR moge=DIR --out RUN/fuse

Per keyframe with >= --min-px valid FFS pixels: robust LSQ  Z_ffs ~ s * Z_ai + b  (3 rounds, drop |r| > 3 * 1.4826 MAD).
Residual stats on the FFS-valid pixels (inliers AND outliers, i.e. all valid pixels) after the fit:
  |r| median / p90 (cm), |r|/Z median / p90 (%). Also 'global': one (median s, median b) for all keyframes (= what a
  keyframe without FFS gets) and for da3 'raw' (s=1, b=0: DA3 already scaled to our pose units).
Keyframes without enough FFS get the global (median) s, b.
Fused depth per variant: Z = FFS where valid, else aligned AI (fill) where AI is valid, 0.05 < Z < --zmax-fill and not on a
depth edge (|Z - median5(Z)| / Z > --edge-rel -> dropped: AI depth smears across occlusion edges -> floaters).
Writes OUT/<variant>/<kf>.npz  Z (float32, NaN none), F (uint8 0 none / 1 FFS / 2 filled) + OUT/align_stats.json.
Variant 'ffs' = FFS only. Variant names for AI = the --ai keys.
"""
import argparse, json, time
from pathlib import Path

import cv2
import numpy as np


def robust_fit(x, y, rounds=3):
    m = np.ones(len(x), bool)
    s, b = 1.0, 0.0
    for _ in range(rounds):
        A = np.stack([x[m], np.ones(m.sum())], 1)
        s, b = np.linalg.lstsq(A, y[m], rcond=None)[0]
        r = y - (s * x + b)
        mad = 1.4826 * np.median(np.abs(r[m] - np.median(r[m]))) + 1e-6
        m = np.abs(r) < 3 * mad
        if m.sum() < 100:
            break
    return float(s), float(b)


def rstats(r, z):
    a = np.abs(r)
    return dict(abs_med_cm=float(np.median(a) * 100), abs_p90_cm=float(np.percentile(a, 90) * 100),
                rel_med_pct=float(np.median(a / z) * 100), rel_p90_pct=float(np.percentile(a / z, 90) * 100), n=int(len(a)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dump", required=True); ap.add_argument("--ai", nargs="+", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--min-px", type=int, default=2000)
    ap.add_argument("--zmax-fill", type=float, default=4.0)
    ap.add_argument("--edge-rel", type=float, default=0.03)
    a = ap.parse_args()
    dump, out = Path(a.dump), Path(a.out); out.mkdir(parents=True, exist_ok=True)
    cams = np.load(dump / "cams.npz"); kf = cams["kf"]
    W, H = [int(v) for v in cams["size"]]
    ai = dict(s.split("=", 1) for s in a.ai)
    ffs = {}
    for i in kf:
        p = dump / "ffs" / f"{i:05d}.npz"
        if p.exists():
            f = np.load(p); Z = f["Z"]; V = f["V"] & np.isfinite(Z) & (Z > 0.05)
            ffs[int(i)] = np.where(V, Z, np.nan).astype(np.float32)
    stats = dict(made=time.strftime("%F %T"), dump=str(dump), ai=ai, args=vars(a), n_kf=int(len(kf)),
                 n_kf_ffs=int(sum(int(np.isfinite(z).sum()) >= a.min_px for z in ffs.values())), models={})
    # FFS-only variant
    (out / "ffs").mkdir(exist_ok=True)
    for i in kf:
        Z = ffs.get(int(i), np.full((H, W), np.nan, np.float32))
        np.savez_compressed(out / "ffs" / f"{i:05d}.npz", Z=Z, F=np.isfinite(Z).astype(np.uint8))
    for name, d in ai.items():
        d = Path(d)
        fits, per = {}, []
        Zai = {int(i): np.load(d / f"{i:05d}.npz")["Z"].astype(np.float32) for i in kf}
        for i in kf:
            i = int(i); Zf = ffs.get(i)
            if Zf is None:
                continue
            m = np.isfinite(Zf) & np.isfinite(Zai[i]) & (Zai[i] > 0)
            if m.sum() < a.min_px:
                continue
            x, y = Zai[i][m].astype(np.float64), Zf[m].astype(np.float64)
            s, b = robust_fit(x, y)
            fits[i] = (s, b)
            per.append(dict(kf=i, s=s, b=b, **rstats(y - (s * x + b), y)))
        sg = float(np.median([v[0] for v in fits.values()])); bg = float(np.median([v[1] for v in fits.values()]))
        glob, raw = [], []
        for i, (s, b) in fits.items():
            Zf = ffs[i]; m = np.isfinite(Zf) & np.isfinite(Zai[i]) & (Zai[i] > 0)
            x, y = Zai[i][m].astype(np.float64), Zf[m].astype(np.float64)
            glob.append(rstats(y - (sg * x + bg), y)); raw.append(rstats(y - x, y))
        agg = lambda L, k: float(np.median([e[k] for e in L]))  # noqa: E731
        keys = ["abs_med_cm", "abs_p90_cm", "rel_med_pct", "rel_p90_pct"]
        stats["models"][name] = dict(
            n_fit=len(fits), global_s=sg, global_b=bg,
            s_p10_p90=[float(np.percentile([v[0] for v in fits.values()], q)) for q in (10, 90)],
            per_kf_fit={k: agg(per, k) for k in keys}, global_fit={k: agg(glob, k) for k in keys},
            no_fit={k: agg(raw, k) for k in keys}, per_kf=per)
        # fused variant
        vo = out / name; vo.mkdir(exist_ok=True)
        fill_frac = []
        for i in kf:
            i = int(i)
            s, b = fits.get(i, (sg, bg))
            Za = s * Zai[i] + b
            med = cv2.medianBlur(np.nan_to_num(Za, nan=0).astype(np.float32), 5)
            ok = np.isfinite(Za) & (Za > 0.05) & (Za < a.zmax_fill) & (np.abs(Za - med) / np.maximum(Za, 1e-3) < a.edge_rel)
            Zf = ffs.get(i, np.full((H, W), np.nan, np.float32))
            hasf = np.isfinite(Zf)
            Z = np.where(hasf, Zf, np.where(ok, Za, np.nan)).astype(np.float32)
            F = np.where(hasf, 1, np.where(ok, 2, 0)).astype(np.uint8)
            fill_frac.append(float((F == 2).mean()))
            np.savez_compressed(vo / f"{i:05d}.npz", Z=Z, F=F, Zai=Za.astype(np.float16))
        stats["models"][name]["fill_frac_mean"] = float(np.mean(fill_frac))
        print(name, json.dumps({k: v for k, v in stats["models"][name].items() if k != "per_kf"}), flush=True)
    (out / "align_stats.json").write_text(json.dumps(stats, indent=1))


if __name__ == "__main__":
    main()
