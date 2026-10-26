#!/usr/bin/env python3
"""dpvofast 2026-10-26: how fast can DPVO really be? Clean timing split + speed knobs (time AND accuracy).

Reuses dpvo_20261026 (dpvo_run.py: same cells = byte-identical 960x540 OpenVINS bench frames, same scoring).
Modes (dpvo env ~/scanner/dpvo_ext/mamba/envs/dpvo/bin/python):
  job  <cell> --tag T [knobs]          run DPVO on one cell, write OUT/T/<cell>_T.{json,causal,final,ms.npy}
  split <cell> --tag T [knobs] [--a 200 --b 500]   per-region cuda-synced time split (patchify, reproject, corr,
                                       update-net, BA, point cloud, keyframe, rest) + unsynced wall + torch.profiler
                                       GPU-kernel time and launch counts over frames a..b (after init)
Knobs:
  --patches N   PATCHES_PER_FRAME (default 96)
  --win W       OPTIMIZATION_WINDOW (default 10);  --removal R  REMOVAL_WINDOW (22);  --life L  PATCH_LIFETIME (13)
  --ba_it K     Gauss-Newton iterations inside each BA call (DPVO hard-codes 2)
  --fp32        MIXED_PRECISION False (default yaml = True = fp16 autocast already)
  --scale S     resize input on GPU (area) by S (e.g. 0.6667 -> 640x360, crop to /16), K scaled
  --stride k    feed every k-th frame of the 60 fps cell (k=2/3/6 -> 30/20/10 Hz)
  --compile     torch.compile the update operator + patchify encoders (dynamic shapes)
  --nosync      one host sync less per frame (keyframe motionmag .item() x2 fused into one)
  --seed s      torch/np seed (default 0)
OUT = ~/scanner/tmp_dpvofast/out (cells -> symlink to ~/scanner/tmp_dpvo/out/cells).
"""
import argparse
import gzip
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

RD = Path(__file__).resolve().parent
os.environ.setdefault("DPVO_OUT", "/home/system/scanner/tmp_dpvofast/out")
sys.path.insert(0, str(RD.parent / "dpvo_20261026"))
import dpvo_run as R  # noqa: E402

OUT = Path(os.environ["DPVO_OUT"])
SYNC = {"on": False}
ACC = {}


def knob_opts(a):
    o = []
    if a.patches:
        o += ["PATCHES_PER_FRAME", str(a.patches)]
    if a.win:
        o += ["OPTIMIZATION_WINDOW", str(a.win)]
    if a.removal:
        o += ["REMOVAL_WINDOW", str(a.removal)]
    if a.life:
        o += ["PATCH_LIFETIME", str(a.life)]
    if a.fp32:
        o += ["MIXED_PRECISION", "False"]
    return o


def setup(a):
    import torch
    sys.path.insert(0, str(R.DPVO_SRC))
    from dpvo.config import cfg
    import dpvo.dpvo as D
    cfg.merge_from_file(str(R.DPVO_SRC / "config/default.yaml"))
    o = knob_opts(a)
    if o:
        cfg.merge_from_list(o)
    if a.ba_it:
        orig = D.fastba.BA

        def BA(*args, **kw):
            kw["iterations"] = a.ba_it
            return orig(*args, **kw)
        D.fastba.BA = BA
    if a.nosync:
        patch_nosync(D)
    return torch, cfg, D


def patch_nosync(D):
    """Fewer host syncs per frame, same math:
    keyframe(): the two motionmag(...).item() calls -> one GPU sum + one .item() (1 host sync less per frame)."""
    import torch
    from dpvo import projective_ops as pops
    from dpvo.lietorch import SE3

    def motionmag_t(self, i, j):
        k = (self.pg.ii == i) & (self.pg.jj == j)
        flow, _ = pops.flow_mag(SE3(self.poses), self.patches, self.intrinsics, self.pg.ii[k], self.pg.jj[k], self.pg.kk[k], beta=0.5)
        return flow.mean()

    orig_kf = D.DPVO.keyframe

    def keyframe(self):
        i = self.n - self.cfg.KEYFRAME_INDEX - 1
        j = self.n - self.cfg.KEYFRAME_INDEX + 1
        m = float((motionmag_t(self, i, j) + motionmag_t(self, j, i)).item())
        self._mm_cached = m
        return orig_kf(self)

    def motionmag(self, i, j):   # called twice by orig keyframe: return half of the fused value each time
        return self._mm_cached / 2.0
    D.DPVO.keyframe = keyframe
    D.DPVO.motionmag = motionmag


# ---------------------------------------------------------------------------------------------- region timers
def timed(name, fn):
    import torch

    def w(*args, **kw):
        if not SYNC["on"]:
            return fn(*args, **kw)
        torch.cuda.synchronize()
        t = time.perf_counter()
        r = fn(*args, **kw)
        torch.cuda.synchronize()
        ACC[name] = ACC.get(name, 0.0) + (time.perf_counter() - t) * 1e3
        return r
    return w


def install_timers(D, slam):
    pm, um = slam.network.patchify, slam.network.update
    pm.forward = timed("patchify(CNN)", pm.forward)
    um.forward = timed("update_net(MLP/GRU/scatter)", um.forward)
    D.fastba.BA = timed("BA(cuda_ba)", D.fastba.BA)
    D.pops.point_cloud = timed("point_cloud", D.pops.point_cloud)
    for meth, nm in (("corr", "corr(cuda_corr)"), ("reproject", "reproject(lietorch)"), ("keyframe", "keyframe(+motionmag)"),
                     ("motion_probe", "motion_probe(init)")):
        f = getattr(slam, meth)
        setattr(slam, meth, timed(nm, f))


# ---------------------------------------------------------------------------------------------- frames
def frames(c, a, torch):
    W, H = c["W"], c["H"]
    mms = [np.memmap(f, np.uint8, "r").reshape(-1, H, W) for f in c["chunks"]]
    K = np.asarray(c["K"], float).copy()
    s = a.scale or 1.0
    Ws, Hs = (int(round(W * s)), int(round(H * s))) if s != 1.0 else (W, H)
    if s != 1.0:
        K[0, :] *= Ws / W
        K[1, :] *= Hs / H
    Hc, Wc = Hs - Hs % 16, Ws - Ws % 16
    rows = c["rows"][::a.stride]
    t_ns = np.asarray(c["t_ns"], np.int64)[::a.stride]
    intr = torch.tensor([K[0, 0], K[1, 1], K[0, 2], K[1, 2]], dtype=torch.float32, device="cuda")

    def get(k):
        ci, r_ = rows[k]
        img = torch.from_numpy(np.ascontiguousarray(mms[ci][r_])).cuda()
        if s != 1.0:
            img = torch.nn.functional.interpolate(img[None, None].float(), size=(Hs, Ws), mode="area")[0, 0].round().clamp(0, 255).to(torch.uint8)
        return img[:Hc, :Wc][None].expand(3, Hc, Wc).contiguous()
    return rows, t_ns, intr, Hc, Wc, get


def make_slam(a, torch, cfg, D, Hc, Wc):
    slam = D.DPVO(cfg, str(R.DPVO_SRC / "dpvo.pth"), ht=Hc, wd=Wc, viz=False)
    if a.compile:
        slam.network.update = torch.compile(slam.network.update, dynamic=True)
        slam.network.patchify.fnet = torch.compile(slam.network.patchify.fnet)
        slam.network.patchify.inet = torch.compile(slam.network.patchify.inet)
    return slam


def job(name, a):
    torch, cfg, D = setup(a)
    from dpvo.lietorch import SE3
    torch.manual_seed(a.seed)
    np.random.seed(a.seed)
    c = json.load(open(OUT / "cells" / f"{name}.json"))
    rows, t_ns, intr, Hc, Wc, get = frames(c, a, torch)
    t0 = int(t_ns[0])
    slam, causal, ms, t_init_ns = None, [], [], None
    torch.cuda.reset_peak_memory_stats()
    wall0 = time.time()
    err = None
    term_ms = None
    try:
        with torch.no_grad():
            for k in range(len(rows)):
                torch.cuda.synchronize()
                q = time.perf_counter()
                image = get(k)
                if slam is None:
                    slam = make_slam(a, torch, cfg, D, Hc, Wc)
                    torch.cuda.synchronize()
                    q = time.perf_counter()
                slam((int(t_ns[k]) - t0) / 1e9, image, intr)
                torch.cuda.synchronize()
                ms.append((time.perf_counter() - q) * 1e3)
                if slam.is_initialized and t_init_ns is None:
                    t_init_ns = int(t_ns[k])
                if slam.is_initialized:
                    P = SE3(slam.pg.poses_[slam.n - 1]).inv().data.cpu().numpy()
                    causal.append(R.pose_line(t_ns[k], R.T_from_xyzq(P)))
                else:
                    causal.append(dict(type="pose", status="init", t_ns=int(t_ns[k]), t=int(t_ns[k]) / 1e9))
            q = time.perf_counter()
            poses, _ = slam.terminate()
            torch.cuda.synchronize()
            term_ms = (time.perf_counter() - q) * 1e3
    except Exception as e:  # noqa: BLE001
        err = f"{type(e).__name__}: {e}"
        poses = None
    od = OUT / a.tag
    od.mkdir(parents=True, exist_ok=True)
    pre = od / f"{name}_{a.tag}"
    with gzip.open(str(pre) + ".causal.poses.jsonl.gz", "wt") as f:
        for m in causal:
            f.write(json.dumps(m) + "\n")
    n_final = 0
    if poses is not None:
        with gzip.open(str(pre) + ".final.poses.jsonl.gz", "wt") as f:
            for k in range(len(poses)):
                tn = int(t_ns[k])
                st = "tracking" if (t_init_ns is not None and tn >= t_init_ns) else "init"
                m = R.pose_line(tn, R.T_from_xyzq(poses[k]), st) if st == "tracking" else dict(type="pose", status="init", t_ns=tn, t=tn / 1e9)
                n_final += st == "tracking"
                f.write(json.dumps(m) + "\n")
    msa = np.asarray(ms)
    st = msa[30:] if len(msa) > 60 else msa   # warm-up (cuDNN autotune, torch.compile recompiles) excluded
    res = dict(name=name, cfg=a.tag, knobs={k: getattr(a, k) for k in ("patches", "win", "removal", "life", "ba_it", "fp32", "scale",
                                                                     "stride", "compile", "nosync", "seed")},
               dpvo_cfg={k: (v if not hasattr(v, "items") else dict(v)) for k, v in cfg.items()},
               rec=c["rec"], off=c["off"], which=c["which"], fps=c["fps"] / a.stride, frames_fed=len(rows), size=f"{Wc}x{Hc}",
               t_init_clip_s=round((t_init_ns - t0) / 1e9, 3) if t_init_ns else None,
               tracked_causal=sum(m["status"] == "tracking" for m in causal), tracked_final=n_final,
               n_keyframes=int(slam.n) if slam is not None else None, error=err,
               ms_med=round(float(np.median(st)), 2), ms_p95=round(float(np.percentile(st, 95)), 2),
               ms_max=round(float(st.max()), 2), ms_mean=round(float(st.mean()), 2),
               terminate_ms=round(term_ms, 1) if term_ms else None, wall_s=round(time.time() - wall0, 1),
               gpu_mem_alloc_max_mb=round(torch.cuda.max_memory_allocated() / 2**20, 1),
               gpu_mem_reserved_max_mb=round(torch.cuda.max_memory_reserved() / 2**20, 1),
               gpu=torch.cuda.get_device_name(0), torch=torch.__version__, load=list(os.getloadavg()),
               dpvo_commit=subprocess.run(["git", "-C", str(R.DPVO_SRC), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip(),
               cell_json_sha256=hashlib.sha256(open(OUT / "cells" / f"{name}.json", "rb").read()).hexdigest()[:16],
               made=time.strftime("%F %T"), runner=str(Path(__file__).resolve()))
    np.save(str(pre) + ".ms.npy", msa)
    json.dump(res, open(str(pre) + ".json", "w"), indent=1)
    print(json.dumps({k: res[k] for k in ("name", "cfg", "frames_fed", "size", "t_init_clip_s", "tracked_causal", "ms_med", "ms_p95",
                                         "error", "load")}), flush=True)


def split(name, a):
    torch, cfg, D = setup(a)
    torch.manual_seed(a.seed)
    np.random.seed(a.seed)
    c = json.load(open(OUT / "cells" / f"{name}.json"))
    rows, t_ns, intr, Hc, Wc, get = frames(c, a, torch)
    t0 = int(t_ns[0])
    slam = make_slam(a, torch, cfg, D, Hc, Wc)
    # measurement window = a..b frames AFTER initialisation (found by a dry pass over the same frames, same seed)
    with torch.no_grad():
        k_init = None
        for k in range(len(rows)):
            slam((int(t_ns[k]) - t0) / 1e9, get(k), intr)
            if slam.is_initialized:
                k_init = k
                break
    torch.manual_seed(a.seed)
    np.random.seed(a.seed)
    slam = make_slam(a, torch, cfg, D, Hc, Wc)
    A = k_init + a.a
    B = min(k_init + a.b, len(rows))
    install_timers(D, slam)
    wall_unsync, wall_sync, per = [], [], []
    from torch.profiler import profile, ProfilerActivity
    prof = None
    PA = B + (B - A)          # frames B..PA: torch.profiler window (unsynced timers off)
    PB = min(PA, len(rows))
    with torch.no_grad():
        for k in range(PB):
            image = get(k)
            torch.cuda.synchronize()
            SYNC["on"] = A <= k < B and (k - A) % 2 == 1     # odd frames: region timers; even frames: clean wall
            ACC.clear()
            if k == B:
                prof = profile(activities=[ProfilerActivity.CPU, ProfilerActivity.CUDA])
                prof.__enter__()
            q = time.perf_counter()
            slam((int(t_ns[k]) - t0) / 1e9, image, intr)
            torch.cuda.synchronize()
            dt = (time.perf_counter() - q) * 1e3
            if A <= k < B:
                if SYNC["on"]:
                    wall_sync.append(dt)
                    per.append(dict(ACC))
                else:
                    wall_unsync.append(dt)
        if prof is not None:
            prof.__exit__(None, None, None)
    SYNC["on"] = False
    keys = sorted({k for p in per for k in p})
    reg = {k: round(float(np.mean([p.get(k, 0.0) for p in per])), 3) for k in keys}
    reg["rest(python glue, indexing, cat, sync)"] = round(float(np.mean(wall_sync)) - sum(reg.values()), 3)
    out = dict(cell=name, tag=a.tag, knobs={k: getattr(a, k) for k in ("patches", "win", "ba_it", "fp32", "scale", "stride", "compile", "nosync")},
               size=f"{Wc}x{Hc}", k_init=k_init, frames=[A, B], n_keyframes=int(slam.n), edges_now=int(slam.pg.ii.numel()),
               wall_unsynced_ms_med=round(float(np.median(wall_unsync)), 3), wall_unsynced_ms_mean=round(float(np.mean(wall_unsync)), 3),
               wall_regiontimed_ms_mean=round(float(np.mean(wall_sync)), 3), regions_ms_mean=reg, load=list(os.getloadavg()),
               made=time.strftime("%F %T"))
    if prof is not None:
        ev = prof.key_averages()
        n = PB - B
        from torch.autograd import DeviceType
        kev = [e for e in ev if e.device_type == DeviceType.CUDA]   # kernels / memcpy only (op-level rows repeat their kernels' time)
        cuda_us = sum(e.self_device_time_total for e in kev)
        launches = sum(e.count for e in ev if e.key in ("cudaLaunchKernel", "cuLaunchKernel", "cudaLaunchKernelExC", "cudaLaunchKernelEx"))
        syncs = {e.key: e.count / n for e in ev if any(s in e.key for s in ("Synchronize", "cudaMemcpy", "aten::item", "aten::_local_scalar_dense",
                                                                             "aten::unique", "aten::_unique"))}
        # kernel classes: group device time by kernel name families
        fam = {}
        for e in kev:
            if e.self_device_time_total <= 0:
                continue
            kname = e.key
            f_ = ("conv/cudnn" if any(s in kname.lower() for s in ("conv", "cudnn", "implicit", "winograd", "sm80_xmma", "sm90", "fprop", "nhwc")) else
                  "corr" if "corr" in kname.lower() else
                  "memcpy/memset" if kname.startswith("Memcpy") or kname.startswith("Memset") else
                  "BA" if any(s in kname for s in ("reprojection_residuals", "pose_retr", "patch_retr", "cholesky", "potrf", "potrs", "trsm",
                                                    "accum_", "EEt", "Ev", "Etv", "EfficentE")) else
                  "gemm" if any(s in kname.lower() for s in ("gemm", "cutlass", "ampere", "sm90_", "cublas")) else
                  "scatter" if "scatter" in kname.lower() else
                  "lietorch" if any(s in kname.lower() for s in ("lie", "se3", "exp_", "log_", "act_", "inv_", "mul_forward")) else
                  "elementwise/reduce/other")
            fam[f_] = fam.get(f_, 0.0) + e.self_device_time_total / 1e3 / n
        out.update(prof_frames=[B, PB], gpu_kernel_ms_per_frame=round(cuda_us / 1e3 / n, 3), kernel_launches_per_frame=round(launches / n, 1),
                   gpu_ms_by_family={k: round(v, 3) for k, v in sorted(fam.items(), key=lambda x: -x[1])},
                   host_sync_like_calls_per_frame={k: round(v, 2) for k, v in syncs.items()})
        tdir = OUT / "split"
        tdir.mkdir(parents=True, exist_ok=True)
        open(tdir / f"{name}_{a.tag}.prof_cuda.txt", "w").write(ev.table(sort_by="self_cuda_time_total", row_limit=40))
        open(tdir / f"{name}_{a.tag}.prof_cpu.txt", "w").write(ev.table(sort_by="self_cpu_time_total", row_limit=40))
    d = RD / "split"
    d.mkdir(exist_ok=True)
    json.dump(out, open(d / f"{name}_{a.tag}.json", "w"), indent=1)
    print(json.dumps(out, indent=1), flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["job", "split"])
    ap.add_argument("cell")
    ap.add_argument("--tag", required=True)
    ap.add_argument("--patches", type=int, default=0)
    ap.add_argument("--win", type=int, default=0)
    ap.add_argument("--removal", type=int, default=0)
    ap.add_argument("--life", type=int, default=0)
    ap.add_argument("--ba_it", type=int, default=0)
    ap.add_argument("--fp32", action="store_true")
    ap.add_argument("--scale", type=float, default=0.0)
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--compile", action="store_true")
    ap.add_argument("--nosync", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--a", type=int, default=100)
    ap.add_argument("--b", type=int, default=400)
    a = ap.parse_args()
    (job if a.cmd == "job" else split)(a.cell, a)
