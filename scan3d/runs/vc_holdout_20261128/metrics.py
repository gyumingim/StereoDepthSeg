#!/usr/bin/env python3
"""vc_holdout 2026-11-28 step 3 (CPU, gsmap_env): arms A0n / A0m / B / B_raw / wrong-colour control, hole-only metrics, gate.
Rules fixed in GATE.md. Eye table read from eye.json (written by hand after looking at media/*_crop.png) if present.

  nice -n 10 ~/scanner/gsmap_env/venv/bin/python metrics.py
"""
import json, time
from pathlib import Path

import cv2
import numpy as np
import torch
import lpips

HERE = Path(__file__).resolve().parent
torch.set_num_threads(8)
LP = lpips.LPIPS(net="alex", spatial=True, verbose=False)


def lp(a, b, H):
    """mean of the spatial LPIPS map over H; outside H both images are set to b's reference (the caller passes GT-masked)."""
    t = lambda im: torch.from_numpy(im).permute(2, 0, 1)[None].float() / 127.5 - 1  # noqa: E731
    with torch.no_grad():
        m = LP(t(a), t(b))[0, 0].numpy()
    return float(m[H].mean())


def masked(im, gt, H):
    o = gt.copy(); o[H] = im[H]; return o


def psnr(a, b, H):
    e = ((a.astype(np.float64) - b.astype(np.float64)) ** 2)[H].mean(); return float(10 * np.log10(255 ** 2 / max(e, 1e-9)))


def nearest_fill(img, cov):
    _, lab = cv2.distanceTransformWithLabels((~cov).astype(np.uint8), cv2.DIST_L2, 5, labelType=cv2.DIST_LABEL_PIXEL)
    v, u = np.nonzero(cov); lut = np.zeros((lab.max() + 1, 3), np.uint8); lut[lab[cov]] = img[v, u]
    o = img.copy(); o[~cov] = lut[lab[~cov]]; return o


def main():
    t0 = time.time(); res = {}; imgs = {}
    for reg in ("R1", "R2"):
        pd = HERE / "prep" / reg; info = json.loads((pd / "info.json").read_text())
        gt = cv2.cvtColor(cv2.imread(str(pd / "gt.png")), cv2.COLOR_BGR2RGB); M = cv2.imread(str(pd / "region_mask.png"), 0) > 0
        for c in info["clips"]:
            name = c["clip"]; f = np.load(HERE / "clips" / name / "frames.npz")
            cond, vc, cov = f["render"][-1], f["diffusion"][-1], f["cover"][-1]
            H = M & ~cov; Hd = cv2.dilate(H.astype(np.uint8), np.ones((17, 17), np.uint8)) > 0
            a0n = nearest_fill(cond, cov)
            ring = (cv2.dilate(H.astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25))) > 0) & cov
            a0m = cond.copy(); a0m[~cov] = np.median(cond[ring], 0).astype(np.uint8)
            fit = cov & ~Hd; X = vc[fit].astype(np.float64); Y = cond[fit].astype(np.float64); g = []
            for k in range(3):
                A = np.stack([X[:, k], np.ones(len(X))], 1); g.append(np.linalg.lstsq(A, Y[:, k], rcond=None)[0])
            vcc = np.clip(np.stack([vc[..., k] * g[k][0] + g[k][1] for k in range(3)], -1), 0, 255).astype(np.uint8)
            B = cond.copy(); B[~cov] = vcc[~cov]; Braw = cond.copy(); Braw[~cov] = vc[~cov]
            arms = dict(A0n=a0n, A0m=a0m, B=B, B_raw=Braw, W_A0n=255 - a0n, W_A0m=255 - a0m, GT=gt)
            r = dict(H_px=int(H.sum()), H_frac_of_region=round(float(H.sum() / M.sum()), 4), uncovered_frac=round(float(1 - cov.mean()), 4),
                     colour_fit=[[round(float(x), 4) for x in gk] for gk in g])
            for k, im in arms.items():
                r[f"lpips_{k}"] = round(lp(masked(im, gt, H), gt, H), 4); r[f"psnr_{k}"] = round(psnr(im, gt, H), 2)
            r["covered_change_absdiff"] = round(float(np.abs(vc.astype(float) - cond.astype(float))[cov].mean()), 2)
            r["covered_change_psnr"] = round(psnr(vc, cond, cov), 2)
            r["covered_change_lpips"] = round(lp(masked(vc, cond, cov), cond, cov), 4)
            res[name] = dict(region=reg, start_kf=c["start_kf"], seed=c["seed"], **r)
            imgs[name] = dict(B=B, H=H, gt=gt, cond=cond, a0n=a0n, a0m=a0m, Braw=Braw, vc=vc, M=M)
    # A0 choice per region (GATE: lower mean LPIPS over the region's 4 clips)
    gate = {}
    eye = json.loads((HERE / "eye.json").read_text()) if (HERE / "eye.json").exists() else None
    for reg in ("R1", "R2"):
        cl = [k for k in res if res[k]["region"] == reg]
        mn = {a: float(np.mean([res[k][f"lpips_{a}"] for k in cl])) for a in ("A0n", "A0m")}
        a0 = min(mn, key=mn.get)
        rows = []
        for k in cl:
            d = res[k][f"lpips_{a0}"] - res[k]["lpips_B"]; w = res[k][f"lpips_W_{a0}"] - res[k][f"lpips_{a0}"]
            res[k]["A0_used"] = a0; res[k]["lpips_A0"] = res[k][f"lpips_{a0}"]; res[k]["gain_A0_minus_B"] = round(d, 4)
            res[k]["control_W_minus_A0"] = round(w, 4); rows.append((d >= 0.05, w >= 0.05, res[k]["lpips_GT"] < 1e-6))
        wins = int(sum(x[0] for x in rows)); ctrl = all(x[1] for x in rows) and all(x[2] for x in rows)
        inv = None if eye is None else int(sum(bool(eye[k]["invented_object"]) for k in cl))
        # the positive control only guards the LPIPS half of the rule; the eye half (invented <= 1 of 4) is independent of it
        if inv is not None and inv > 1:
            verdict = "NO-GO (invented objects %d/4%s)" % (inv, "; LPIPS ruler failed its control" if not ctrl else "")
        elif not ctrl:
            verdict = "UNDECIDED (ruler blind)"
        elif inv is None:
            verdict = "PENDING eye table"
        else:
            verdict = "GO" if wins >= 3 else "NO-GO"
        # consistency
        seed_pairs, start_pairs = [], []
        starts = sorted({res[k]["start_kf"] for k in cl}); seeds = sorted({res[k]["seed"] for k in cl})
        nm = lambda s, sd: f"{reg}_s{s}_seed{sd}"  # noqa: E731
        for s in starts:
            a, b = imgs[nm(s, seeds[0])], imgs[nm(s, seeds[1])]; H = a["H"]
            seed_pairs.append(dict(start_kf=s, lpips=round(lp(masked(a["B"], a["gt"], H), masked(b["B"], a["gt"], H), H), 4)))
        for sd in seeds:
            a, b = imgs[nm(starts[0], sd)], imgs[nm(starts[1], sd)]; H = a["H"]
            start_pairs.append(dict(seed=sd, lpips=round(lp(masked(a["B"], a["gt"], H), masked(b["B"], a["gt"], H), H), 4)))
        gate[reg] = dict(A0_choice=a0, A0_mean_lpips=round(mn["A0n"], 4) if a0 == "A0n" else round(mn["A0m"], 4), A0_means={k: round(v, 4) for k, v in mn.items()},
                         B_mean_lpips=round(float(np.mean([res[k]["lpips_B"] for k in cl])), 4),
                         clips_B_beats_A0_by_005=wins, positive_control_ok=bool(ctrl),
                         invented_object_clips=inv, verdict=verdict, consistency_seed_to_seed=seed_pairs, consistency_start_to_start=start_pairs)
    # media
    md = HERE / "media"; md.mkdir(exist_ok=True)
    lab = lambda im, s: cv2.putText(cv2.cvtColor(im, cv2.COLOR_RGB2BGR), s, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)  # noqa: E731
    for k, d in imgs.items():
        cnt, _ = cv2.findContours(d["M"].astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        a0 = d["a0n"] if res[k]["A0_used"] == "A0n" else d["a0m"]
        tiles = [lab(d["gt"], "real photo (GT)"), lab(a0, f"A0 ({res[k]['A0_used']}) {res[k]['lpips_A0']:.3f}"),
                 lab(d["B"], f"B ViewCrafter {res[k]['lpips_B']:.3f}"), lab(d["cond"], "condition render")]
        tiles = [cv2.drawContours(t, cnt, -1, (0, 0, 255), 1) for t in tiles]
        cv2.imwrite(str(md / f"{k}_side.jpg"), np.hstack(tiles), [cv2.IMWRITE_JPEG_QUALITY, 88])
        x, y, w, h = cv2.boundingRect(d["M"].astype(np.uint8)); p = 10
        crop = lambda im: cv2.resize(cv2.cvtColor(im, cv2.COLOR_RGB2BGR)[max(0, y - p):y + h + p, max(0, x - p):x + w + p], None, fx=2, fy=2)  # noqa: E731
        cv2.imwrite(str(md / f"{k}_crop.jpg"), np.hstack([crop(d["gt"]), crop(d["B"]), crop(d["vc"])]), [cv2.IMWRITE_JPEG_QUALITY, 90])
        cv2.imwrite(str(md / f"{k}_vcraw.jpg"), cv2.cvtColor(d["vc"], cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 88])
    infos = {r: json.loads((HERE / "prep" / r / "info.json").read_text()) for r in ("R1", "R2")}
    out = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), gate_file=str(HERE / "GATE.md"),
               inputs=dict(prep={r: {k: infos[r][k] for k in ("made", "kf", "rect", "inputs", "starts", "n_removed", "n_points")} for r in infos},
                           clips=str(HERE / "clips"), eye=str(HERE / "eye.json") if eye else None),
               lpips="alex, spatial map averaged over H (H = region rectangle AND uncovered); outside H set to GT", gate=gate, clips=res,
               runtime_s=round(time.time() - t0, 1), note="n small: 1 recording, 2 regions, 2 starts x 2 seeds each")
    (HERE / "results.json").write_text(json.dumps(out, indent=1))
    print(json.dumps(gate, indent=1))
    for k, r in res.items():
        print(k, {x: r[x] for x in ("H_px", "lpips_A0n", "lpips_A0m", "lpips_B", "lpips_B_raw", "lpips_W_A0n", "lpips_W_A0m", "psnr_A0n", "psnr_B",
                                     "gain_A0_minus_B", "covered_change_absdiff", "covered_change_psnr")})


if __name__ == "__main__":
    main()
