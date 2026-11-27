#!/usr/bin/env python3
"""gsmap 11-27 dc: FFS-guided AI depth COMPLETION (Prior Depth Anything v1.1, vitb+vitb; FFS fused depth = prior, RGB) for 121801.

Runs in its own venv (~/scanner/priorda_env/venv, torch 2.11 cu128), started with python -I from outside the download dir:
  ~/scanner/priorda_env/venv/bin/python -I scan3d/gsmap/gsm_dc.py probe OUTDIR [--kf-idx 50,150,...] [--hide 0.2 --block 24]
  ~/scanner/priorda_env/venv/bin/python -I scan3d/gsmap/gsm_dc.py run OUTDIR [--k-sigma 3 --edge-rel 0.05]
Model code = ~/scanner/priorda_env/dl/Prior-Depth-Anything (git 8c029cb, Apache-2.0; HF Rain729/Prior-Depth-Anything card apache-2.0;
the frozen DA-v2 vitb upstream is CC-BY-NC-4.0 -> research use only). torch_cluster.knn replaced by dc_shim (scipy cKDTree).

probe: per keyframe, valid FFS = fused F == 1 inside the 7x7-eroded undistortion mask. Hide 20 % of the valid FFS pixels in random
  BLOCKxBLOCK blocks (holes, harder than random pixels), complete from the rest (PriorDA) and fit DA3 affine (align_fuse robust_fit on the
  kept pixels, = last night's fill), score both on the hidden pixels. Second pass with the full FFS prior: score both vs the BA2 sparse
  depth targets on FFS-EMPTY pixels (w5/sparse: BA2 tracks >= 3 obs, reproj < 1 px, tri > 2 deg, 3x3 splat, only F == 0).
  Reference level: fused FFS vs the same BA tracks (w5 sparse.json sanity, median 0.73 %).
run: all keyframes, full FFS prior. OUT/fused = copy of the FFS fused files (unchanged); OUT/dc_sparse = completed depth (Z, S) on
  F == 0 pixels only (NaN / inf elsewhere) for gsm_train.py --sparse-depth (FFS pixels keep FFS).
  S(completed) = k_sigma x median FFS sigma at that depth (w5 sigma table). Dropped: depth-edge pixels (|Z - median5(Z)| / Z > edge_rel),
  Z outside (0.05, zmax), outside the valid mask.
"""
import argparse, json, sys, time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
PDA = Path.home() / "scanner/priorda_env/dl/Prior-Depth-Anything"
MDC = Path.home() / "scanner/priorda_env/dl/Marigold-DC"   # git c6d756b, Apache-2.0; weights prs-eth/marigold-depth-v1-0 (card apache-2.0)
sys.path.insert(0, str(HERE / "dc_shim")); sys.path.insert(0, str(PDA)); sys.path.insert(0, str(MDC))
V2 = HERE.parent / "runs/gsmap_20261125_121801_v2"
DUMP = V2 / "ba_partners2/run/dump_ba"; FUSED = V2 / "ffsonly_ba2/fused"; DA3 = V2 / "depth_da3"; SPARSE = V2 / "w5/sparse"
AISLE = set(range(314, 332))
MN = dict(pda="PriorDA", mdc="Marigold-DC")   # picture labels (probe_mdc jpgs made before this fix say "PriorDA")


def robust_fit(x, y, rounds=3):   # = align_fuse.robust_fit
    m = np.ones(len(x), bool); s, b = 1.0, 0.0
    for _ in range(rounds):
        A = np.stack([x[m], np.ones(m.sum())], 1); s, b = np.linalg.lstsq(A, y[m], rcond=None)[0]
        r = y - (s * x + b); mad = 1.4826 * np.median(np.abs(r[m] - np.median(r[m]))) + 1e-6; m = np.abs(r) < 3 * mad
        if m.sum() < 100:
            break
    return float(s), float(b)


def err(p, g):
    p = np.asarray(p, np.float64); g = np.asarray(g, np.float64)
    if not len(g):
        return dict(n=0)
    r = (p - g) / g; a = np.abs(r)
    return dict(n=int(len(a)), absrel_med_pct=float(np.median(a) * 100), absrel_mean_pct=float(a.mean() * 100),
                p90_pct=float(np.percentile(a, 90) * 100), gt5_pct=float((a > 0.05).mean() * 100), signed_med_pct=float(np.median(r) * 100))


class Ctx:
    def __init__(s):
        c = np.load(DUMP / "cams.npz"); s.kf = c["kf"]; s.W, s.H = [int(v) for v in c["size"]]; s.f = float(c["Kr"][0, 0])
        mx, my = cv2.initUndistortRectifyMap(c["K"], c["dist"], None, c["Kr"], (s.W, s.H), cv2.CV_32FC1)
        s.vmask = cv2.erode(((mx >= 0) & (mx <= s.W - 1) & (my >= 0) & (my <= s.H - 1)).astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)

    def load(s, i):
        rgb = cv2.cvtColor(cv2.imread(str(DUMP / f"img/{i:05d}.png")), cv2.COLOR_BGR2RGB)
        d = np.load(FUSED / f"{i:05d}.npz"); F = np.where(s.vmask, d["F"], 0)
        return rgb, d["Z"].astype(np.float32), F, d["S"].astype(np.float32)


def model(dev="cuda:0"):
    import torch
    if GEO["model"] == "mdc":
        torch.set_num_threads(8)
        from diffusers import DDIMScheduler
        from marigold_dc import MarigoldDepthCompletionPipeline
        p = MarigoldDepthCompletionPipeline.from_pretrained("prs-eth/marigold-depth-v1-0", prediction_type="depth").to(dev, dtype=torch.bfloat16)
        p.scheduler = DDIMScheduler.from_config(p.scheduler.config, timestep_spacing="trailing"); return p
    from prior_depth_anything import PriorDepthAnything
    torch.set_num_threads(8)
    return PriorDepthAnything(device=dev, version="1.1", frozen_model_size="vitb", conditioned_model_size="vitb")


GEO = dict(src="", dg=False, model="pda", steps=50, ens=1, res=768)   # --geometric da3: DA3-LARGE depth replaces the frozen DA-v2 vitb prediction (PriorDA 'geometric' input)


def complete(m, rgb, prior, i=None):
    import torch
    if GEO["model"] == "mdc":
        from PIL import Image
        out = np.asarray(m(image=Image.fromarray(rgb), sparse_depth=np.ascontiguousarray(prior.astype(np.float32)), num_inference_steps=GEO["steps"],
                           ensemble_size=GEO["ens"], processing_resolution=GEO["res"]), np.float32)
        assert out.shape == prior.shape, out.shape
        torch.cuda.empty_cache(); return out
    geo = None
    if GEO["src"] == "da3":
        g = np.load(DA3 / f"{i:05d}.npz")["Z"].astype(np.float32); geo = np.where(np.isfinite(g) & (g > 0), g, np.nanmedian(g)).astype(np.float32)
    out = m.infer_one_sample(image=rgb, prior=np.ascontiguousarray(prior.astype(np.float32)), geometric=geo, double_global=GEO["dg"], visualize=False)
    out = out.float().cpu().numpy(); assert out.shape == prior.shape, out.shape
    torch.cuda.empty_cache(); return out


def hide_blocks(valid, frac, block, rng):
    H, W = valid.shape
    blocks = [(y, x) for y in range(0, H, block) for x in range(0, W, block) if valid[y:y + block, x:x + block].any()]
    rng.shuffle(blocks); hid = np.zeros_like(valid); need = frac * valid.sum(); n = 0
    for y, x in blocks:
        v = valid[y:y + block, x:x + block]; hid[y:y + block, x:x + block] = v; n += int(v.sum())
        if n >= need:
            break
    return hid


def shade(Z, f):   # normal-shading picture of a depth map (edges / flatness check)
    Zs = np.where(np.isfinite(Z) & (Z > 0), Z, np.nan).astype(np.float64)
    gy, gx = np.gradient(Zs); n = np.dstack([-gx * f / Zs, -gy * f / Zs, np.ones_like(Zs)])
    n /= np.linalg.norm(n, axis=2, keepdims=True); img = (n * 0.5 + 0.5) * 255
    img[~np.isfinite(Zs)] = 0; return np.nan_to_num(img).astype(np.uint8)


def dcol(Z, lo, hi):
    v = np.clip((np.nan_to_num(Z, nan=0) - lo) / (hi - lo), 0, 1); c = cv2.applyColorMap((255 * (1 - v)).astype(np.uint8), cv2.COLORMAP_TURBO)
    c[~(np.isfinite(Z) & (Z > 0))] = 0; return c


def ecol(e):
    c = cv2.applyColorMap((255 * np.clip(np.nan_to_num(e) / 0.1, 0, 1)).astype(np.uint8), cv2.COLORMAP_INFERNO); c[~np.isfinite(e)] = 0; return c


def label(img, t):
    img = img.copy(); cv2.putText(img, t, (8, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA); return img


def cmd_probe(a):
    out = Path(a.out); assert not (out / "probe.json").exists(), "never overwrite"; out.mkdir(parents=True, exist_ok=True)
    cx = Ctx(); m = model(); rng = np.random.default_rng(0); t0 = time.time()
    idxs = [int(v) for v in a.kf_idx.split(",")]
    per = {}; acc = {}
    add = lambda k, v: acc.setdefault(k, []).append(np.asarray(v, np.float64))
    for j in idxs:
        i = int(cx.kf[j]); rgb, Z, F, S = cx.load(i); valid = (F == 1) & np.isfinite(Z) & (Z > 0)
        hid = hide_blocks(valid, a.hide, a.block, rng); keep = valid & ~hid
        t1 = time.time(); Dh = complete(m, rgb, np.where(keep, Z, 0), i); Df = complete(m, rgb, np.where(valid, Z, 0), i); tt = time.time() - t1
        za = np.load(DA3 / f"{i:05d}.npz")["Z"].astype(np.float64); okA = np.isfinite(za) & (za > 0)
        s1, b1 = robust_fit(za[keep & okA], Z[keep & okA].astype(np.float64)); Ah = s1 * za + b1
        s2, b2 = robust_fit(za[valid & okA], Z[valid & okA].astype(np.float64)); Af = s2 * za + b2
        hm = hid & okA & np.isfinite(Dh)
        sfx = "_a" if j in AISLE else "_o"
        r = dict(idx=j, kf=i, aisle=j in AISLE, ffs_px=int(valid.sum()), ffs_pct=round(100 * valid.sum() / cx.vmask.sum(), 2),
                 hidden_px=int(hm.sum()), sec_2runs=round(tt, 2), hidden=dict(pda=err(Dh[hm], Z[hm]), da3_affine=err(Ah[hm], Z[hm])),
                 ffs_sigma_rel_med_pct_hidden=float(np.median(S[hm] / Z[hm]) * 100) if hm.any() else None)
        add("h_pda" + sfx, Dh[hm]); add("h_da3" + sfx, Ah[hm]); add("h_g" + sfx, Z[hm])
        sp = SPARSE / f"{i:05d}.npz"
        if sp.exists():
            Zb = np.load(sp)["Z"]; bm = np.isfinite(Zb) & (Zb > 0) & (F == 0) & cx.vmask & okA & np.isfinite(Df)
            r["ba_empty"] = dict(pda=err(Df[bm], Zb[bm]), da3_affine=err(Af[bm], Zb[bm]))
            add("b_pda" + sfx, Df[bm]); add("b_da3" + sfx, Af[bm]); add("b_g" + sfx, Zb[bm])
        per[i] = r; print(json.dumps(r), flush=True)
        np.savez_compressed(out / f"{i:05d}.npz", Dh=Dh.astype(np.float32), Df=Df.astype(np.float32), hid=hid, Ah=Ah.astype(np.float32), Af=Af.astype(np.float32))
        lo, hi = np.percentile(Z[valid], 2), np.percentile(Z[valid], 98)
        comp = np.where(valid, Z, np.where(cx.vmask, Df, np.nan)); da3c = np.where(valid, Z, np.where(cx.vmask, Af, np.nan))
        e_p = np.where(hm, np.abs(Dh - Z) / Z, np.nan); e_a = np.where(hm, np.abs(Ah - Z) / Z, np.nan)
        row1 = np.hstack([label(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), f"kf {i} idx {j}"), label(dcol(np.where(valid, Z, np.nan), lo, hi), f"FFS fused {r['ffs_pct']}%"),
                          label(dcol(comp, lo, hi), f"FFS + {MN[a.model]} fill"), label(dcol(da3c, lo, hi), "FFS + DA3 affine fill")])
        row2 = np.hstack([label(ecol(e_p), f"hidden |err| {MN[a.model]} (0-10%)"), label(ecol(e_a), "hidden |err| DA3 (0-10%)"),
                          label(shade(np.where(cx.vmask, Df, np.nan), cx.f), f"shading {MN[a.model]} (raw)"), label(shade(np.where(cx.vmask, Af, np.nan), cx.f), "shading DA3 affine")])
        cv2.imwrite(str(out / f"cmp_{i:05d}.jpg"), cv2.resize(np.vstack([row1, row2]), None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA), [cv2.IMWRITE_JPEG_QUALITY, 88])
    cat = lambda *ks: np.concatenate([np.concatenate(acc[k]) for k in ks if k in acc]) if any(k in acc for k in ks) else np.zeros(0)
    pooled = {}
    for nm, sfxs in (("all", ("_a", "_o")), ("aisle", ("_a",)), ("other", ("_o",))):
        pooled[nm] = dict(hidden=dict(pda=err(cat(*["h_pda" + s for s in sfxs]), cat(*["h_g" + s for s in sfxs])),
                                      da3_affine=err(cat(*["h_da3" + s for s in sfxs]), cat(*["h_g" + s for s in sfxs]))),
                          ba_empty=dict(pda=err(cat(*["b_pda" + s for s in sfxs]), cat(*["b_g" + s for s in sfxs])),
                                        da3_affine=err(cat(*["b_da3" + s for s in sfxs]), cat(*["b_g" + s for s in sfxs]))))
    sj = json.loads((SPARSE / "sparse.json").read_text())
    res = dict(made=time.strftime("%F %T"), n_recordings=1, model=("Marigold-DC git c6d756b, prs-eth/marigold-depth-v1-0 bf16" if a.model == "mdc" else "Prior Depth Anything v1.1 (vitb frozen DA-v2 + vitb conditioned), git 8c029cb"),
               inputs=dict(dump=str(DUMP), fused=str(FUSED), da3=str(DA3), ba_sparse=str(SPARSE)),
               args=vars(a), hide_rule=f"{a.hide:.0%} of valid FFS px in random {a.block}x{a.block} blocks, seed 0",
               ffs_reference=dict(note="fused FFS vs the same BA2 tracks on FFS pixels (w5 sparse.json sanity, all keyframes)",
                                  med_abs_rel_pct=sj["sanity"]["med_abs_rel_pct"], p90_abs_rel_pct=sj["sanity"]["p90_abs_rel_pct"]),
               pooled=pooled, per_kf=per, runtime_s=round(time.time() - t0, 1))
    (out / "probe.json").write_text(json.dumps(res, indent=1)); print(json.dumps(res["pooled"], indent=1))


def cmd_run(a):
    out = Path(a.out); fo, so, ro = out / "fused", out / "dc_sparse", out / "raw"
    assert not (out / "run.json").exists(), "never overwrite"
    for d in (fo, so, ro):
        d.mkdir(parents=True, exist_ok=True)
    cx = Ctx(); m = model(); t0 = time.time()
    st = json.loads((SPARSE / "sparse.json").read_text())["sigma_table_ffs"]; bc = np.array(st["depth_m"]); bm = np.array(st["ffs_sigma_med_cm"]) / 100

    def ffs_sigma_at(zz):   # = gsm_w5_sparse ffs_sigma_at
        s_ = np.interp(zz, bc, bm); hi_ = zz > bc[-1]; s_[hi_] = bm[-1] * (zz[hi_] / bc[-1]) ** 2; return s_
    per = {}; tot = dict(ffs=0, dc=0, edge=0, nonffs=0)
    for j, i in enumerate(cx.kf):
        i = int(i)
        rgb, Z, F, S = cx.load(i); valid = (F == 1) & np.isfinite(Z) & (Z > 0)
        src = np.load(FUSED / f"{i:05d}.npz"); np.savez_compressed(fo / f"{i:05d}.npz", **{k: src[k] for k in src.files})
        Zs = np.full(Z.shape, np.nan, np.float32); Ss = np.full(Z.shape, np.inf, np.float32); ne = 0
        if valid.sum() >= 100:
            D = complete(m, rgb, np.where(valid, Z, 0), i)
            med = cv2.medianBlur(D.astype(np.float32), 5); edge = np.abs(D - med) / np.maximum(D, 1e-3) > a.edge_rel
            ok = (~valid) & cx.vmask & np.isfinite(D) & (D > 0.05) & (D < a.zmax) & ~edge
            ne = int(((~valid) & cx.vmask & edge).sum())
            Zs[ok] = D[ok]; Ss[ok] = (a.k_sigma * ffs_sigma_at(D[ok].astype(np.float64))).astype(np.float32)
            np.savez_compressed(ro / f"{i:05d}.npz", D=D.astype(np.float32))
        np.savez_compressed(so / f"{i:05d}.npz", Z=Zs, S=Ss)
        n = int(np.isfinite(Zs).sum()); per[i] = dict(idx=j, ffs_px=int(valid.sum()), dc_px=n, edge_drop_px=ne)
        tot["ffs"] += int(valid.sum()); tot["dc"] += n; tot["edge"] += ne; tot["nonffs"] += int(((~valid) & cx.vmask).sum())
        if j % 25 == 0:
            print(f"{time.strftime('%T')} {j}/{len(cx.kf)} kf {i} ffs {valid.sum()} dc {n}", flush=True)
    res = dict(made=time.strftime("%F %T"), n_recordings=1, model=("Marigold-DC git c6d756b bf16" if a.model == "mdc" else "Prior Depth Anything v1.1 vitb/vitb git 8c029cb"), args=vars(a),
               inputs=dict(dump=str(DUMP), fused=str(FUSED), sigma_table=str(SPARSE / "sparse.json")), totals=tot,
               dc_share_of_nonffs_pct=100 * tot["dc"] / max(1, tot["nonffs"]), per_kf=per, runtime_s=round(time.time() - t0, 1))
    (out / "run.json").write_text(json.dumps(res, indent=1)); print(json.dumps({k: res[k] for k in ("totals", "dc_share_of_nonffs_pct", "runtime_s")}))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("cmd", choices=["probe", "run"]); ap.add_argument("out")
    ap.add_argument("--kf-idx", default="50,150,250,400,470,314,318,322,326,330"); ap.add_argument("--hide", type=float, default=0.2)
    ap.add_argument("--block", type=int, default=24); ap.add_argument("--k-sigma", type=float, default=3.0)
    ap.add_argument("--edge-rel", type=float, default=0.05); ap.add_argument("--zmax", type=float, default=8.0)
    ap.add_argument("--geometric", default="", choices=["", "da3"]); ap.add_argument("--double-global", type=int, default=0)
    ap.add_argument("--model", default="pda", choices=["pda", "mdc"], help="pda = Prior Depth Anything v1.1; mdc = Marigold-DC")
    ap.add_argument("--steps", type=int, default=50); ap.add_argument("--ens", type=int, default=1); ap.add_argument("--res", type=int, default=768)
    a = ap.parse_args(); GEO.update(src=a.geometric, dg=bool(a.double_global), model=a.model, steps=a.steps, ens=a.ens, res=a.res); {"probe": cmd_probe, "run": cmd_run}[a.cmd](a)
