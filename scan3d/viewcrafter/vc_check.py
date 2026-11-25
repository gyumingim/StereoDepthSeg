#!/usr/bin/env python3
"""viewcrafter 2026-11-25: known-answer check. Last frame of the anchor -> held-out keyframe path vs the real photo.

  ~/scanner/gsmap_env/venv/bin/python scan3d/viewcrafter/vc_check.py WORK DUMP OUT_MEDIA_DIR

All at 512x320 (864x540 centre crop resized). Pixels of the black undistortion border of the real photo are ignored.
PSNR + LPIPS (alex) for: ViewCrafter (DUSt3R PC), ViewCrafter (our PC), gsA2 splat, and the condition renders.
Also a 'copy anchor' baseline (the anchor photo itself) to show how much the view changes.
"""
import json, sys
from pathlib import Path

import cv2
import numpy as np


def main():
    wk, dump, out = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
    meta = json.loads(str(np.load(wk / "traj_check.npz")["meta"]))
    prep = lambda im: cv2.resize(im[:, (im.shape[1] - 864) // 2:][:, :864], (512, 320), interpolation=cv2.INTER_AREA)  # noqa: E731
    real = prep(cv2.imread(str(dump / "img" / f"{meta['target_kf']:05d}.png")))
    anc = prep(cv2.imread(str(dump / "img" / f"{meta['anchor_kf']:05d}.png")))
    mask = cv2.erode((real.max(-1) > 8).astype(np.uint8), np.ones((7, 7), np.uint8)) > 0
    c = {}
    for pc in ("dust3r", "ours"):
        f = np.load(wk / f"chk_sparse_{pc}" / "frames.npz")
        c[f"ViewCrafter ({pc} PC)"] = cv2.cvtColor(f["diffusion"][-1], cv2.COLOR_RGB2BGR)
        c[f"condition render ({pc} PC)"] = cv2.cvtColor(f["render"][-1], cv2.COLOR_RGB2BGR)
    c["gsA2 splat"] = prep(cv2.imread(str(wk / "chk_splat_gsA2" / "frames" / "24.png")))
    c["anchor photo copied"] = anc
    import torch, lpips
    lp = lpips.LPIPS(net="alex", verbose=False)
    t = lambda im: torch.from_numpy(cv2.cvtColor(im, cv2.COLOR_BGR2RGB)).permute(2, 0, 1)[None].float() / 127.5 - 1  # noqa: E731
    res = {}
    for k, im in c.items():
        e = ((im.astype(np.float32) - real.astype(np.float32)) ** 2)[mask].mean()
        im_m = im.copy(); im_m[~mask] = real[~mask]
        with torch.no_grad():
            l = float(lp(t(im_m), t(real)).item())
        res[k] = dict(psnr=round(float(10 * np.log10(255 ** 2 / e)), 2), lpips=round(l, 3))
        print(f"{k:32s} PSNR {res[k]['psnr']:.2f}  LPIPS {res[k]['lpips']:.3f}")
    lab = lambda im, s: cv2.putText(im.copy(), s, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)  # noqa: E731
    row1 = np.hstack([lab(real, f"REAL held-out photo kf {meta['target_kf']}"), lab(c["ViewCrafter (dust3r PC)"], "ViewCrafter (DUSt3R PC)"),
                      lab(c["ViewCrafter (ours PC)"], "ViewCrafter (our PC)")])
    row2 = np.hstack([lab(c["gsA2 splat"], "gsA2 splat"), lab(c["condition render (dust3r PC)"], "condition DUSt3R PC"),
                      lab(c["condition render (ours PC)"], "condition our PC")])
    cv2.imwrite(str(out / "check_heldout.jpg"), np.vstack([row1, row2]), [cv2.IMWRITE_JPEG_QUALITY, 88])
    (out / "check_heldout.json").write_text(json.dumps(dict(meta=meta, results=res), indent=1))


if __name__ == "__main__":
    main()
