#!/usr/bin/env python3
"""dpvofast 2026-10-26: FLOPs per frame of DPVO's network parts (torch FlopCounterMode, counts matmul/conv only)
+ edge counts during a real run (std60 165506_o0 frames 0..600) -> mobile estimate input.
  patchify (fnet + inet BasicEncoder4) at 960x528 / 640x352 / 480x264
  update operator per edge (measured with E edges) -> GFLOP/frame = per-edge x mean edges after init
  corr (custom cuda_corr, not counted by torch): analytic 2 levels x (2r+2)^2 x P^2 x 128 MAC per edge
dpvo env: python dpvofast_flops.py [patches]"""
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import torch
from torch.utils.flop_counter import FlopCounterMode

RD = Path(__file__).resolve().parent
os.environ.setdefault("DPVO_OUT", "/home/system/scanner/tmp_dpvofast/out")
sys.path.insert(0, str(RD.parent / "dpvo_20261026"))
import dpvo_run as R  # noqa: E402
sys.path.insert(0, str(R.DPVO_SRC))
from dpvo.config import cfg  # noqa: E402
from dpvo.dpvo import DPVO  # noqa: E402

M = int(sys.argv[1]) if len(sys.argv) > 1 else 96
cfg.merge_from_file(str(R.DPVO_SRC / "config/default.yaml"))
cfg.merge_from_list(["PATCHES_PER_FRAME", str(M)])
out = dict(patches=M, made=time.strftime("%F %T"))
with torch.no_grad():
    for (w, h) in ((960, 528), (640, 352), (480, 264)):
        slam = DPVO(cfg, str(R.DPVO_SRC / "dpvo.pth"), ht=h, wd=w, viz=False)
        img = torch.rand(1, 1, 3, h, w, device="cuda")
        fc = FlopCounterMode(display=False)
        with fc, torch.autocast("cuda", dtype=torch.half):
            slam.network.patchify(img, patches_per_image=M, centroid_sel_strat="RANDOM", return_color=True)
        out[f"patchify_gflop_{w}x{h}"] = round(fc.get_total_flops() / 1e9, 2)
    # update operator per edge: run a real cell and record edge counts
    c = json.load(open(R.OUT / "cells" / "s165506_o0_f60.json"))
    W, H = c["W"], c["H"]
    mms = [np.memmap(f, np.uint8, "r").reshape(-1, H, W) for f in c["chunks"]]
    K = np.asarray(c["K"])
    intr = torch.tensor([K[0, 0], K[1, 1], K[0, 2], K[1, 2]], dtype=torch.float32, device="cuda")
    torch.manual_seed(0)
    slam = DPVO(cfg, str(R.DPVO_SRC / "dpvo.pth"), ht=528, wd=960, viz=False)
    edges, kfs = [], []
    for k, (ci, r_) in enumerate(c["rows"][:700]):
        im = torch.from_numpy(np.ascontiguousarray(mms[ci][r_][:528, :960])).cuda()[None].expand(3, 528, 960).contiguous()
        slam(k / 60.0, im, intr)
        if slam.is_initialized:
            edges.append(int(slam.pg.ii.numel()))
            kfs.append(int(slam.n))
    E = int(np.median(edges))
    out.update(edges_med=E, edges_p95=int(np.percentile(edges, 95)), edges_max=int(max(edges)), frames_after_init=len(edges))
    net = slam.pg.net
    ctx = slam.imap[:, slam.pg.kk % (slam.M * slam.pmem)]
    coords = slam.reproject()
    corr = slam.corr(coords)
    fc = FlopCounterMode(display=False)
    with fc, torch.autocast("cuda", dtype=torch.half):
        slam.network.update(net, ctx, corr, None, slam.pg.ii, slam.pg.jj, slam.pg.kk)
    e_now = int(slam.pg.ii.numel())
    per_edge = fc.get_total_flops() / e_now
    out.update(update_flop_per_edge=round(per_edge), update_gflop_per_frame_at_med_edges=round(per_edge * E / 1e9, 2))
    corr_mac = 2 * (2 * 3 + 2) ** 2 * 9 * 128
    out.update(corr_mac_per_edge=corr_mac, corr_gflop_per_frame=round(2 * corr_mac * E / 1e9, 2))
    # memory touched by corr per frame (gathers from fmap pyramid, fp16): each edge reads (8x8 + 3x3) x 128 x 2 B per level (upper bound w/o cache)
    out["corr_gather_MB_per_frame_upper"] = round(E * 2 * (64 + 9) * 128 * 2 / 2**20, 1)
print(json.dumps(out, indent=1))
json.dump(out, open(RD / f"flops_p{M}.json", "w"), indent=1)
