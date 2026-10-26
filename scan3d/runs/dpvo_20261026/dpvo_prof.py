#!/usr/bin/env python3
"""dpvo 2026-10-26: where does DPVO's per-frame time go on this PC (WSL2, RTX 5080)? torch.profiler over frames
200..299 of one cell (after init/warm-up): GPU kernel time per frame vs wall time per frame, top CPU/CUDA ops.
  dpvo env: python dpvo_prof.py <cell> [cfg]   (cells json from dpvo_run.py prep, OUT = DPVO_OUT or ~/scanner/tmp_dpvo/out)"""
import json, os, sys, time
from pathlib import Path
import numpy as np
import torch
sys.path.insert(0, str(Path(__file__).resolve().parent))
import dpvo_run as R
sys.path.insert(0, str(R.DPVO_SRC))
from dpvo.config import cfg
from dpvo.dpvo import DPVO
name = sys.argv[1]; cfgname = sys.argv[2] if len(sys.argv) > 2 else "base"
c = json.load(open(R.OUT / "cells" / f"{name}.json"))
cf = R.CFGS[cfgname]; cfg.merge_from_file(str(R.DPVO_SRC / cf["file"]))
if cf["opts"]: cfg.merge_from_list(cf["opts"])
W, H = c["W"], c["H"]; Hc, Wc = H - H % 16, W - W % 16
mms = [np.memmap(f, np.uint8, "r").reshape(-1, H, W) for f in c["chunks"]]
K = np.asarray(c["K"]); intr = torch.tensor([K[0, 0], K[1, 1], K[0, 2], K[1, 2]], dtype=torch.float32, device="cuda")
torch.manual_seed(0); np.random.seed(0)
slam = DPVO(cfg, str(R.DPVO_SRC / "dpvo.pth"), ht=Hc, wd=Wc, viz=False)
t_ns = np.asarray(c["t_ns"]); t0 = t_ns[0]
A, B = 200, 300
from torch.profiler import profile, ProfilerActivity
wall = []
with torch.no_grad():
    for k, (ci, r_) in enumerate(c["rows"][:B]):
        img = torch.from_numpy(np.ascontiguousarray(mms[ci][r_][:Hc, :Wc])).cuda()[None].expand(3, Hc, Wc).contiguous()
        if k == A:
            prof = profile(activities=[ProfilerActivity.CPU, ProfilerActivity.CUDA]); prof.__enter__()
        torch.cuda.synchronize(); a = time.perf_counter()
        slam((t_ns[k] - t0) / 1e9, img, intr)
        torch.cuda.synchronize(); wall.append((time.perf_counter() - a) * 1e3)
    prof.__exit__(None, None, None)
ev = prof.key_averages()
cuda_us = sum(e.self_device_time_total for e in ev)
n = B - A
out = dict(cell=name, cfg=cfgname, frames=n, wall_ms_med=float(np.median(wall[A:])), wall_ms_mean=float(np.mean(wall[A:])),
           wall_ms_med_unprofiled=float(np.median(wall[100:A])),
           gpu_kernel_ms_per_frame=cuda_us / 1e3 / n, n_cuda_launch_per_frame=sum(e.count for e in ev if e.key in ("cudaLaunchKernel", "cuLaunchKernel", "cudaLaunchKernelExC")) / n,
           load=list(os.getloadavg()), made=time.strftime("%F %T"))
print(json.dumps(out))
print(ev.table(sort_by="self_cuda_time_total", row_limit=12))
print(ev.table(sort_by="self_cpu_time_total", row_limit=12))
