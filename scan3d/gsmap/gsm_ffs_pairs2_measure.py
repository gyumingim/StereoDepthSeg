#!/usr/bin/env python3
"""gsmap 2026-11-26: measure old dense FFS vs pairs2 step 1 (s1) vs step 1+2 (s12), same rules, same keyframes.

  GSMAP_RUN=<v2 run> python scan3d/gsmap/gsm_ffs_pairs2_measure.py OLD_FFS_DIR PAIRS2_DIR --out OUT [--fig FIG.jpg] [--fig-kf 642,649,656,X]
Per set: keyframes with FFS, median valid fraction, multi-view fail % (gsm_filtmesh rule: violated by >= 2 views and >= 30 % of the
checkable views, margin max(3 cm, 3 %), undistortion border masked) with (a) the set itself as reference (as gsm_ffs_dense) and (b) the
OLD set as a fixed reference (same ruler for all), on the common keyframes and overall; depth p2-p98; agreement with eval2 gold
(old-run filtered FFS confirmed by >= 3 views; biased toward the old pairs). Query pixels subsampled every 2nd px (both axes).
Sets 's1_strict' / 's12_strict' drop the fallback keyframes (no perp>=40 pair, old +-2 s rule used).
"""
import argparse, json, os, sys, time
from pathlib import Path

import cv2
import numpy as np


def main():
    import torch
    ap = argparse.ArgumentParser(); ap.add_argument("old"); ap.add_argument("p2"); ap.add_argument("--out", required=True)
    ap.add_argument("--fig", default=""); ap.add_argument("--fig-kf", default="642,649,656")
    a = ap.parse_args(); t0 = time.time()
    R = Path(os.environ["GSMAP_RUN"]); old, p2, out = Path(a.old), Path(a.p2), Path(a.out)
    c = np.load(R / "dump_ba" / "cams.npz"); kf, T, Kr = c["kf"], c["T"], c["Kr"]; W, H = [int(v) for v in c["size"]]; n = len(kf)
    fx, fy, cx, cy = [float(v) for v in (Kr[0, 0], Kr[1, 1], Kr[0, 2], Kr[1, 2])]
    mx, my = cv2.initUndistortRectifyMap(c["K"], c["dist"], None, Kr, (W, H), cv2.CV_32FC1)
    vm = cv2.erode(((mx >= 0) & (mx <= W - 1) & (my >= 0) & (my <= H - 1)).astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)
    info = json.loads((p2 / "s1" / "ffs_info.json").read_text()); fbk = set(info["fallback_kf"])

    def load(d, keep=None):
        Z = np.full((n, H, W), np.nan, np.float32)
        for ci, i in enumerate(kf):
            f = d / f"{int(i):05d}.npz"
            if f.exists() and (keep is None or keep(int(i))):
                Z[ci] = np.load(f)["Z"]
        return Z
    sets = {"old": load(old), "s1": load(p2 / "s1"), "s12": load(p2 / "s12")}
    sets["s1_strict"] = np.where(np.array([int(i) not in fbk for i in kf])[:, None, None], sets["s1"], np.nan)
    sets["s12_strict"] = np.where(np.array([int(i) not in fbk for i in kf])[:, None, None], sets["s12"], np.nan)
    has = {k: np.isfinite(Z).reshape(n, -1).any(1) for k, Z in sets.items()}
    dev = "cuda"; Tw = torch.from_numpy(np.linalg.inv(T)).float().to(dev); Tc = torch.from_numpy(T).float().to(dev)
    gold = np.load(R / "eval2" / "ref" / "gold_ffs.npz")["Z"].astype(np.float32)

    def mvcheck(Z, ref, href):
        """per keyframe (bad, total) on the 2x-subsampled valid pixels."""
        rt = torch.from_numpy(ref).to(dev); res = np.zeros((n, 2), np.int64)
        js = [j for j in range(n) if href[j]]; jt = torch.tensor(js, device=dev)
        for ci in range(n):
            Zs = Z[ci, ::2, ::2]; val = np.isfinite(Zs) & vm[::2, ::2]
            if not val.any():
                continue
            v, u = np.nonzero(val); z = torch.from_numpy(Zs[v, u]).float().to(dev)
            ut = torch.from_numpy(u * 2).float().to(dev); vt = torch.from_numpy(v * 2).float().to(dev)
            X = torch.stack([(ut - cx) / fx * z, (vt - cy) / fy * z, z], 1) @ Tc[ci, :3, :3].T + Tc[ci, :3, 3]
            viol = torch.zeros(len(z), dtype=torch.int32, device=dev); chk = torch.zeros_like(viol)
            for k0 in range(0, len(js), 48):
                jj = jt[k0:k0 + 48]; jj = jj[jj != ci]
                if len(jj) == 0:
                    continue
                Xj = torch.einsum("vij,mj->vmi", Tw[jj, :3, :3], X) + Tw[jj, None, :3, 3]; zj = Xj[..., 2]
                pu = torch.round(fx * Xj[..., 0] / zj.clamp(min=1e-3) + cx).long(); pv = torch.round(fy * Xj[..., 1] / zj.clamp(min=1e-3) + cy).long()
                ok = (zj > 0.05) & (pu >= 0) & (pu < W) & (pv >= 0) & (pv < H)
                zr = rt[jj[:, None].expand_as(pu), pv.clamp(0, H - 1), pu.clamp(0, W - 1)]; zr = torch.where(ok, zr, torch.full_like(zr, float("nan")))
                fin = torch.isfinite(zr)
                chk += fin.sum(0).int(); viol += (fin & (zj < zr - torch.clamp(0.03 * zr, min=0.03))).sum(0).int()
            res[ci] = [int(((viol >= 2) & (viol >= 0.3 * chk)).sum()), len(z)]
        return res

    mv_self = {k: mvcheck(Z, Z, has[k]) for k, Z in sets.items()}
    mv_old = {k: (mv_self["old"] if k == "old" else mvcheck(Z, sets["old"], has["old"])) for k, Z in sets.items()}
    common = has["old"] & has["s1"] & has["s12"]; common_strict = common & has["s1_strict"]
    pct = lambda r, m: round(100 * r[m, 0].sum() / max(1, r[m, 1].sum()), 2)
    kmed = lambda r, m: round(100 * float(np.median(r[m, 0] / np.maximum(r[m, 1], 1))), 2)
    res = dict(made=time.strftime("%F %T"), inputs=dict(old=str(old), p2=str(p2), gold=str(R / "eval2" / "ref" / "gold_ffs.npz")),
               n_kf=n, n_common=int(common.sum()), n_common_strict=int(common_strict.sum()), n_fallback_kf=len(fbk),
               note="mv = gsm_filtmesh rule on 2x-subsampled pixels; gold = old-run filtered FFS (biased toward old pairs)", sets={})
    for k, Z in sets.items():
        V = np.isfinite(Z); vf = V.reshape(n, -1).mean(1); zz = Z[V][::50]
        g = {}
        Zs = Z[:, ::2, ::2]; gm = gold > 0; both = gm & np.isfinite(Zs)
        if both.any():
            e = np.abs(Zs[both] - gold[both]); g = dict(gold_cover=round(float(both.sum() / gm.sum()), 4), within_2cm_1pct=round(float((e < 0.02 + 0.01 * gold[both]).mean()), 4),
                                                         med_abs_rel=round(float(np.median(e / gold[both])), 4))
        res["sets"][k] = dict(kf_with_ffs=int(has[k].sum()), kf_lost_vs_old=int((has["old"] & ~has[k]).sum()),
                              valid_frac_median_with_ffs=round(float(np.median(vf[has[k]])), 4), valid_frac_median_common=round(float(np.median(vf[common])), 4),
                              mv_self_pct_all=pct(mv_self[k], has[k]), mv_self_pct_common=pct(mv_self[k], common), mv_self_pct_common_strict=pct(mv_self[k], common_strict),
                              mv_self_kfmedian_common=kmed(mv_self[k], common),
                              mv_oldref_pct_all=pct(mv_old[k], has[k]), mv_oldref_pct_common=pct(mv_old[k], common), mv_oldref_pct_common_strict=pct(mv_old[k], common_strict),
                              depth_p2_p98=[round(float(x), 3) for x in np.percentile(zz, [2, 98])] if len(zz) else None, gold=g)
    fbm = np.array([int(i) in fbk for i in kf])
    for k in ("s1", "s12"):
        res["sets"][k]["mv_self_pct_fallback_kf"] = pct(mv_self[k], has[k] & fbm); res["sets"][k]["mv_self_pct_strict_kf"] = pct(mv_self[k], has[k] & ~fbm)
        res["sets"][k]["mv_oldref_pct_fallback_kf"] = pct(mv_old[k], has[k] & fbm)
    res["sets"]["old"]["mv_self_pct_fallback_kf"] = pct(mv_self["old"], has["old"] & fbm); res["sets"]["old"]["mv_self_pct_strict_kf"] = pct(mv_self["old"], has["old"] & ~fbm)
    res["runtime_s"] = round(time.time() - t0, 1)
    (out / "measure.json").write_text(json.dumps(res, indent=1)); print(json.dumps(res, indent=1), flush=True)
    if a.fig:
        rows = []
        for fk in [int(x) for x in a.fig_kf.split(",")]:
            ci = int(np.nonzero(kf == fk)[0][0]); img = cv2.imread(str(R / "dump_ba" / "img" / f"{fk:05d}.png"))
            tiles = [cv2.resize(img, (480, 270))]
            for k in ("old", "s1", "s12"):
                Z = sets[k][ci]; d = np.clip((Z - 0.3) / 1.7, 0, 1); col = cv2.applyColorMap((255 * np.nan_to_num(d)).astype(np.uint8), cv2.COLORMAP_TURBO)
                col[~np.isfinite(Z)] = 0; tl = cv2.resize(col, (480, 270), interpolation=cv2.INTER_NEAREST)
                r_ = mv_self[k][ci]; txt = f"{k} kf{fk} valid {np.isfinite(Z).mean():.2f} mv {100 * r_[0] / max(1, r_[1]):.0f}%"
                cv2.putText(tl, txt, (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2); tiles.append(tl)
            cv2.putText(tiles[0], f"kf {fk}{' (fallback)' if fk in fbk else ''}", (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            rows.append(np.hstack(tiles))
        fig = np.vstack(rows); cv2.putText(fig, "turbo 0.3-2.0 m, black = none", (6, fig.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imwrite(a.fig, fig, [cv2.IMWRITE_JPEG_QUALITY, 88]); print("fig", a.fig)


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "gsplat"))
    import gpu_guard
    with gpu_guard.job("gsmap ffs_pairs2 measure", torch_cap=True):
        main()
