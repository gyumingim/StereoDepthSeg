#!/usr/bin/env python3
"""gsmap 2026-11-25 (added after seeing gsB): multi-view free-space check of the AI-filled pixels (anti ghost layer).

  ~/scanner/gsmap_env/venv/bin/python scan3d/gsmap/fill_check.py --dump DUMP --fused FUSE/da3 --out FUSE/da3c [--tol 0.03]

A filled pixel (F == 2) of keyframe c is back-projected with its aligned AI depth and projected into every other keyframe j
that has FFS depth. If it lands inside j's image on a valid FFS pixel and lies IN FRONT of that FFS surface by more than
max(tol, tol * Z_ffs) -> violation (FFS saw empty space there). Filled pixels with >= --min-viol violations AND >= --min-frac of the checkable views are dropped (F -> 0).
FFS pixels are never changed. Writes the same npz layout + check_stats.json.
"""
import argparse, json, sys, time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "gsplat"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dump", required=True); ap.add_argument("--fused", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--tol", type=float, default=0.03); ap.add_argument("--min-viol", type=int, default=2)
    ap.add_argument("--min-frac", type=float, default=0.3, help="and violations >= this share of the views where the point was checkable")
    ap.add_argument("--cand", type=int, default=2, help="2 = check filled pixels (normal); 1 = control: FFS pixels vs other views FFS (stats only)")
    a = ap.parse_args()
    import torch
    t0 = time.time()
    dump, fu, out = Path(a.dump), Path(a.fused), Path(a.out); out.mkdir(parents=True, exist_ok=True)
    cams = np.load(dump / "cams.npz"); kf, T, Kr = cams["kf"], cams["T"], cams["Kr"]
    W, H = [int(v) for v in cams["size"]]
    d = [np.load(fu / f"{i:05d}.npz") for i in kf]
    Z = [x["Z"] for x in d]; F = [x["F"] for x in d]
    dev = "cuda"
    ffs = torch.stack([torch.from_numpy(np.where(f == 1, z, np.nan)) for z, f in zip(Z, F)]).to(dev)   # (n, H, W)
    has = [bool(torch.isfinite(ffs[j]).any()) for j in range(len(kf))]
    Tw = torch.from_numpy(np.linalg.inv(T)).float().to(dev); Tc = torch.from_numpy(T).float().to(dev)
    fx, fy, cx, cy = [float(v) for v in (Kr[0, 0], Kr[1, 1], Kr[0, 2], Kr[1, 2])]
    st = []
    for c in range(len(kf)):
        Fc = F[c].copy()
        v, u = np.nonzero(Fc == a.cand)
        if len(v):
            z = torch.from_numpy(Z[c][v, u]).float().to(dev); uu = torch.from_numpy(u).float().to(dev); vv = torch.from_numpy(v).float().to(dev)
            X = torch.stack([(uu - cx) / fx * z, (vv - cy) / fy * z, z], 1) @ Tc[c, :3, :3].T + Tc[c, :3, 3]
            viol = torch.zeros(len(z), dtype=torch.int32, device=dev); chk = torch.zeros_like(viol)
            for j in range(len(kf)):
                if j == c or not has[j]:
                    continue
                Xj = X @ Tw[j, :3, :3].T + Tw[j, :3, 3]; zj = Xj[:, 2]
                pu = torch.round(fx * Xj[:, 0] / zj.clamp(min=1e-3) + cx).long(); pv = torch.round(fy * Xj[:, 1] / zj.clamp(min=1e-3) + cy).long()
                ok = (zj > 0.05) & (pu >= 0) & (pu < W) & (pv >= 0) & (pv < H)
                zr = torch.full_like(zj, float("nan")); zr[ok] = ffs[j][pv[ok], pu[ok]]
                chk += torch.isfinite(zr).int()
                viol += (torch.isfinite(zr) & (zj < zr - torch.clamp(a.tol * zr, min=a.tol))).int()
            drop = ((viol >= a.min_viol) & (viol >= a.min_frac * chk)).cpu().numpy()
            if a.cand == 2:
                Fc[v[drop], u[drop]] = 0
            st.append(dict(kf=int(kf[c]), fill=int(len(v)), dropped=int(drop.sum())))
        Zc = np.where(Fc > 0, Z[c], np.nan).astype(np.float32)
        np.savez_compressed(out / f"{kf[c]:05d}.npz", Z=Zc, F=Fc)
    tot_f = sum(s["fill"] for s in st); tot_d = sum(s["dropped"] for s in st)
    res = dict(made=time.strftime("%F %T"), fused=str(fu), args=vars(a), fill_px=tot_f, dropped_px=tot_d,
               dropped_frac=tot_d / max(1, tot_f), runtime_s=round(time.time() - t0, 1), per_kf=st)
    (out / "check_stats.json").write_text(json.dumps(res, indent=1))
    print(json.dumps({k: v for k, v in res.items() if k != "per_kf"}), flush=True)


if __name__ == "__main__":
    import gpu_guard
    o = sys.argv[sys.argv.index("--out") + 1]
    with gpu_guard.job("gsmap fill_check " + o, out_dir=o, torch_cap=True):
        main()
