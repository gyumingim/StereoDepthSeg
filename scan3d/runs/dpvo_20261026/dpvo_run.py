#!/usr/bin/env python3
"""dpvo 2026-10-26: DPVO / DPV-SLAM on our recordings, scored with the SAME ruler as ov60eval / integ2 (OpenVINS).

Three stages, two Python environments:
  prep  (venv_ffs = the bench env, cv2 5.0.0): per cell, the exact frames the OpenVINS bench feeds (vio_bench_hs
        undistort_maps + fed_selection + FrameCache: same lens file, 960x540 gray, same 1/fps time grid, same start
        offset, same eof stop) -> OUT/cells/<cell>.json = frame index list, cache chunk + row per frame, t_ns on the IMU
        clock (t_cam - dt, = the OpenVINS pose t_ns), Knew. Frames are read later straight from the cache memmaps
        (uint8, byte-identical to the bridge input). A cache miss is reported, never silently decoded.
  job   (dpvo env, torch 2.7.1+cu128): DPVO on one cell, gray replicated to 3 channels, image cropped to H - H%16
        (540 -> 528 rows, top-left kept, so K is unchanged; DPVO's own stream does the same). Seeded (torch/np 0).
        Writes two trajectories in the OpenVINS pose-line format (type pose, status, t_ns, t, T_world_cam0):
          causal = pose of the newest frame right after it was processed (what a live system could use),
          final  = DPVO terminate(): 12 more BA iterations + non-keyframes interpolated (the paper's evaluation).
        Frames before DPVO's initialisation (8 keyframes with motion) are status "init" (not scored), like OpenVINS.
        Timing: cuda-synchronised wall ms per frame (incl. host->GPU copy), torch max_memory_allocated / reserved.
  score (venv_ffs): per output and GT cell:
        sim3   = vio_bench_hs.evaluate() Sim3 ATE on the raw DPVO trajectory (= OpenVINS 'sim3' column, same code)
        ate_w / div / rel5_max = zupt_ob_score.run_metrics (via lens_switch_gtscore.cam_metrics, SE3 window ATE,
                 divergence ate > 50 cm or 5 s rel > 30 cm) on the trajectory scaled by its full Sim3 scale (oracle scale)
        ate_w_ow = same, restricted to the OpenVINS reference run's tracking window (same cell, same span)
        s5     = scale from the first 5 s of tracking only (Sim3 fit) -> ate_w_s5 (realistic: scale known early,
                 then the VO's scale drift shows); scale_drift = Sim3 scale (2nd half) / (1st half) - 1
        loops  = fi_limits_run.loop_eval (start-end error / path, loop_pct is scale-free)

  ~/scanner/StereoDepthSeg/venv_ffs/bin/python dpvo_run.py prep hs:30,60;std:30,60;loop:30,60
  ~/scanner/dpvo_ext/mamba/envs/dpvo/bin/python dpvo_run.py run hs:30 --cfg base [--par 1]
  ~/scanner/StereoDepthSeg/venv_ffs/bin/python dpvo_run.py score
OUT = ~/scanner/tmp_dpvo/out (env DPVO_OUT). Raw poses stay there (not committed); score.json is copied next to this file.
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

MAIN = Path("/home/system/scanner/StereoDepthSeg/scan3d")
RD = Path(__file__).resolve().parent
OUT = Path(os.environ.get("DPVO_OUT", "/home/system/scanner/tmp_dpvo/out"))  # /tmp wiped 10-26 (WSL crash)
DPVO_SRC = Path("/home/system/scanner/dpvo_ext/DPVO")
PY_FFS = "/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python"
PY_DPVO = "/home/system/scanner/dpvo_ext/mamba/envs/dpvo/bin/python"
GT = "imu_gt_spl_cal1022"
STD_GT = "imu_gt_spl"
OFFS = (0, 1, 2)
STD = ["hs_20261024_165506_std60", "hs_20261024_173432_std60"]
LOOP = ["hs_20261024_142148_std60", "hs_20261024_142610_std60", "hs_20261024_151646_std60"]
SIZE = "960x540"
# DPVO configs: base = repo config/default.yaml (96 patches, window 10, the paper's default); fast = config/fast.yaml
# if present; slam = base + proximity loop closure (DPV-SLAM); p48 = base with 48 patches (light reference)
CFGS = {"base": dict(file="config/default.yaml", opts=[]),
        "slam": dict(file="config/default.yaml", opts=["LOOP_CLOSURE", "True"]),
        "p48": dict(file="config/default.yaml", opts=["PATCHES_PER_FRAME", "48"])}
# OpenVINS references (same cells): integ2 0.6.27 profile (gyro_klt=5) and ov60eval
OVR = "/home/system/scanner/tmp_dpvo/ov"   # own re-run of the integ2 0.6.27 profile (ov_ref_run.py), all four cells sets
REF = {("hs", 30): (OVR, "GL30"), ("hs", 60): (OVR, "GL60"), ("std", 30): (OVR, "GL30"), ("std", 60): (OVR, "GL60"),
       ("loop", 30): (OVR, "GL30"), ("loop", 60): (OVR, "GL60")}


def cells(which):
    if which == "hs":
        sys.path.insert(0, str(MAIN / "imu_learn"))
        from phone60_run import RAW, OLD  # noqa: E402
        return [(r, o) for r in RAW + OLD for o in OFFS]
    if which == "std":
        return [(r, o) for r in STD for o in OFFS]
    if which == "loop":
        return [(r, 0) for r in LOOP]
    raise ValueError(which)


def key_of(rec, off, which):
    hh = rec.split("_")[2]
    return {"hs": f"{hh}_o{off}", "std": f"s{hh}_o{off}", "loop": f"l{hh}_o{off}"}[which]


def cell_name(rec, off, which, fps):
    return f"{key_of(rec, off, which)}_f{int(fps)}"


def which_of(key):
    return {"s": "std", "l": "loop"}.get(key[0], "hs")


# ----------------------------------------------------------------------------------------------- prep (venv_ffs)
def prep(spec):
    sys.path.insert(0, str(MAIN / "imu_learn"))
    sys.path.insert(0, str(MAIN))
    os.chdir(MAIN)
    import csv
    import vio_bench_hs as vb
    from frame_cache import FrameCache
    (OUT / "cells").mkdir(parents=True, exist_ok=True)
    for part in spec.split(";"):
        which, fpss = part.split(":")
        for fps in (float(x) for x in fpss.split(",")):
            for rec, off in cells(which):
                name = cell_name(rec, off, which, fps)
                dst = OUT / "cells" / f"{name}.json"
                if dst.exists():
                    continue
                R = Path(f"recordings/{rec}")
                meta = json.load(open(R / "meta.json"))
                vb.set_lens_override(None)
                m1, m2, Knew, (W, H) = vb.undistort_maps(R, meta, SIZE)
                rows = list(csv.DictReader(open(R / "frames_cam5.csv")))
                dt_ns = int(json.load(open(R / "imu_gt" / "align.json"))["dt_ns"])
                fs = vb.fed_selection(R, meta, rows, fps, float(off), 0.0)
                fc = FrameCache(R, (W, H), m1, m2)
                sel = [int(i) for i in fs["sel"] if fc.eof is None or int(i) < fc.eof]
                miss = [i for i in sel if i not in fc.where]
                chunks, rowsel = [], []
                files = {}
                for fi in sel:
                    if fi in miss:
                        continue
                    mm, r_ = fc.where[fi]
                    fn = str(mm.filename)
                    if fn not in files:
                        files[fn] = len(chunks)
                        chunks.append(fn)
                    rowsel.append([files[fn], int(r_)])
                t = [int(fs["t_frames"][i]) - dt_ns for i in sel]
                d = dict(name=name, rec=rec, off=off, which=which, fps=fps, W=W, H=H, K=np.asarray(Knew).tolist(),
                         lens=str(vb.lens_path(R, meta)), dt_ns=dt_ns, fi=sel, t_ns=t, cache_dir=str(fc.dir),
                         cache_key=fc.key, chunks=chunks, rows=rowsel, missing=miss, eof=fc.eof,
                         made=time.strftime("%F %T"), note="frames = FrameCache rows (bench input bytes); t_ns = IMU clock")
                json.dump(d, open(str(dst) + ".tmp", "w"))
                os.replace(str(dst) + ".tmp", dst)
                print(f"{name}: {len(sel)} frames, missing {len(miss)}, chunks {len(chunks)}", flush=True)


# ----------------------------------------------------------------------------------------------- job (dpvo env)
def pose_line(t_ns, T, status="tracking"):
    return dict(type="pose", status=status, t_ns=int(t_ns), t=t_ns / 1e9, T_world_cam0=np.asarray(T).reshape(-1).tolist())


def T_from_xyzq(v):
    from scipy.spatial.transform import Rotation as Rot
    T = np.eye(4)
    T[:3, :3] = Rot.from_quat(v[3:7]).as_matrix()   # x y z w
    T[:3, 3] = v[:3]
    return T


def job(name, cfgname):
    import torch
    sys.path.insert(0, str(DPVO_SRC))
    from dpvo.config import cfg
    from dpvo.dpvo import DPVO
    from dpvo.lietorch import SE3
    torch.manual_seed(0)
    np.random.seed(0)
    c = json.load(open(OUT / "cells" / f"{name}.json"))
    if c["missing"]:
        raise SystemExit(f"{name}: {len(c['missing'])} frames not in the cache -> run the bench / frame_cache build first")
    cf = CFGS[cfgname]
    cfg.merge_from_file(str(DPVO_SRC / cf["file"]))
    if cf["opts"]:
        cfg.merge_from_list(cf["opts"])
    W, H = c["W"], c["H"]
    mms = [np.memmap(f, np.uint8, "r").reshape(-1, H, W) for f in c["chunks"]]
    K = np.asarray(c["K"])
    intr = torch.tensor([K[0, 0], K[1, 1], K[0, 2], K[1, 2]], dtype=torch.float32, device="cuda")
    Hc = H - H % 16
    Wc = W - W % 16
    net = str(DPVO_SRC / "dpvo.pth")
    slam = None
    t_ns = np.asarray(c["t_ns"], np.int64)
    t0 = int(t_ns[0])
    causal, ms, t_init_ns = [], [], None
    lc_prev, n_lc, lc_at = -1000, 0, []   # DPV-SLAM proximity loop closure: times last_global_ba moved (= loop factors added)
    torch.cuda.reset_peak_memory_stats()
    wall0 = time.time()
    err = None
    try:
        with torch.no_grad():
            for k, (ci, r_) in enumerate(c["rows"]):
                g = mms[ci][r_]
                img = torch.from_numpy(np.ascontiguousarray(g[:Hc, :Wc]))
                torch.cuda.synchronize()
                a = time.perf_counter()
                image = img.cuda()[None].expand(3, Hc, Wc).contiguous()
                if slam is None:
                    slam = DPVO(cfg, net, ht=Hc, wd=Wc, viz=False)
                    torch.cuda.synchronize()
                    a = time.perf_counter()        # network load is not per-frame time
                slam((int(t_ns[k]) - t0) / 1e9, image, intr)
                torch.cuda.synchronize()
                ms.append((time.perf_counter() - a) * 1e3)
                lg = getattr(slam, "last_global_ba", None)
                if lg is not None and lg != lc_prev:
                    n_lc, lc_prev = n_lc + 1, lg
                    lc_at.append(round((int(t_ns[k]) - t0) / 1e9, 2))
                if slam.is_initialized and t_init_ns is None:
                    t_init_ns = int(t_ns[k])
                if slam.is_initialized:   # after init every frame is appended as the newest (keyframe() may drop an older one)
                    P = SE3(slam.pg.poses_[slam.n - 1]).inv().data.cpu().numpy()   # world->cam stored; cam->world out
                    causal.append(pose_line(t_ns[k], T_from_xyzq(P)))
                else:
                    causal.append(dict(type="pose", status="init", t_ns=int(t_ns[k]), t=int(t_ns[k]) / 1e9))
            fin_t = time.perf_counter()
            poses, tst = slam.terminate()
            torch.cuda.synchronize()
            term_ms = (time.perf_counter() - fin_t) * 1e3
    except Exception as e:   # noqa: BLE001  (record DPVO failures as results)
        err = f"{type(e).__name__}: {e}"
        poses, tst, term_ms = None, None, None
    od = OUT / cfgname
    od.mkdir(parents=True, exist_ok=True)
    pre = od / f"{name}_{cfgname}"
    with gzip.open(str(pre) + ".causal.poses.jsonl.gz", "wt") as f:
        for m in causal:
            f.write(json.dumps(m) + "\n")
    n_final = 0
    if poses is not None:
        with gzip.open(str(pre) + ".final.poses.jsonl.gz", "wt") as f:
            for k in range(len(poses)):
                tn = int(t_ns[k])
                st = "tracking" if (t_init_ns is not None and tn >= t_init_ns) else "init"
                m = pose_line(tn, T_from_xyzq(poses[k]), st) if st == "tracking" else dict(type="pose", status="init", t_ns=tn, t=tn / 1e9)
                n_final += st == "tracking"
                f.write(json.dumps(m) + "\n")
    msa = np.asarray(ms)
    st = msa[10:] if len(msa) > 20 else msa   # first 10 calls = CUDA / cuDNN warm-up (seconds), not steady state
    gpu = torch.cuda.get_device_name(0)
    res = dict(name=name, cfg=cfgname, cfg_def=cf, dpvo_cfg={k: (v if not hasattr(v, "items") else dict(v)) for k, v in cfg.items()},
               rec=c["rec"], off=c["off"], which=c["which"], fps=c["fps"], frames_fed=len(c["rows"]), size=f"{Wc}x{Hc}",
               t_init_clip_s=round((t_init_ns - t0) / 1e9, 3) if t_init_ns else None,
               tracked_causal=sum(m["status"] == "tracking" for m in causal), tracked_final=n_final,
               n_keyframes=int(slam.n) if slam is not None else None, error=err, n_loop_ba=n_lc, loop_ba_t_s=lc_at[:50],
               ms_med=round(float(np.median(st)), 2) if len(st) else None, ms_p95=round(float(np.percentile(st, 95)), 2) if len(st) else None,
               ms_max=round(float(st.max()), 2) if len(st) else None, ms_mean=round(float(st.mean()), 2) if len(st) else None,
               terminate_ms=round(term_ms, 1) if term_ms else None, wall_s=round(time.time() - wall0, 1),
               gpu_mem_alloc_max_mb=round(torch.cuda.max_memory_allocated() / 2**20, 1),
               gpu_mem_reserved_max_mb=round(torch.cuda.max_memory_reserved() / 2**20, 1),
               gpu=gpu, torch=torch.__version__, cuda=torch.version.cuda, load=list(os.getloadavg()),
               dpvo_commit=subprocess.run(["git", "-C", str(DPVO_SRC), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip(),
               weights_sha256=hashlib.sha256(open(net, "rb").read()).hexdigest()[:16],
               cell_json_sha256=hashlib.sha256(open(OUT / "cells" / f"{name}.json", "rb").read()).hexdigest()[:16],
               made=time.strftime("%F %T"), runner=str(Path(__file__).resolve()))
    np.save(str(pre) + ".ms.npy", msa)
    json.dump(res, open(str(pre) + ".json", "w"), indent=1)
    print(json.dumps({k: res[k] for k in ("name", "cfg", "frames_fed", "t_init_clip_s", "tracked_causal", "ms_med", "ms_p95",
                                         "gpu_mem_alloc_max_mb", "error")}), flush=True)


def run_sched(spec, cfgname, par):
    q = []
    if spec.startswith("names="):   # explicit cell list (pre-declared 60 fps subset, PLAN.md)
        spec, q = "", spec[6:].split(",")
    for part in filter(None, spec.split(";")):
        which, fpss = part.split(":")
        offs = None
        if "@" in which:   # e.g. hs@0:60 = offset 0 cells only (time budget)
            which, o_ = which.split("@")
            offs = {int(x) for x in o_.split("+")}
        q += [cell_name(r, o, which, float(f)) for f in fpss.split(",") for r, o in cells(which) if offs is None or o in offs]
    q = [n for n in q if not (OUT / cfgname / f"{n}_{cfgname}.json").exists()]
    (OUT / cfgname).mkdir(parents=True, exist_ok=True)
    running = {}
    print(f"{time.strftime('%F %T')} start {spec} cfg {cfgname} {len(q)} jobs par {par}", flush=True)
    while q or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is not None:
                del running[n]
                ok = (OUT / cfgname / f"{n}_{cfgname}.json").exists()
                print(f"{time.strftime('%F %T')} done {n} {time.time() - t0:.0f} s {'ok' if ok else 'FAILED'}", flush=True)
        while q and len(running) < par:
            n = q.pop(0)
            err = open(OUT / cfgname / f"{n}_{cfgname}.err", "w")
            running[n] = (subprocess.Popen([PY_DPVO, str(Path(__file__).resolve()), "job", n, "--cfg", cfgname],
                                           stdout=err, stderr=subprocess.STDOUT), time.time())
        time.sleep(1)
    print(f"{time.strftime('%F %T')} end", flush=True)


# ----------------------------------------------------------------------------------------------- score (venv_ffs)
def load_lines(p):
    return [json.loads(x) for x in gzip.open(p, "rt") if x.strip()] if Path(p).exists() else []


def gt_match(rec, gtname, P, dt_ns):
    """Same matching as vio_bench_hs.evaluate: GT ok & |p| < 1.2 m, nearest GT within 2 ms of t_ns + dt."""
    g = dict(np.load(MAIN / "recordings" / rec / gtname / "gt.npz", allow_pickle=True))
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    gt_t, gt_p = g["t_ns"][ok], g["p_bc"][ok]
    tr = [m for m in P if m.get("status") == "tracking" and m.get("T_world_cam0")]
    if len(tr) < 50:
        return None
    vt = np.array([m["t_ns"] for m in tr]) + dt_ns
    X = np.array([np.asarray(m["T_world_cam0"]).reshape(4, 4)[:3, 3] for m in tr])
    j = np.clip(np.searchsorted(gt_t, vt), 1, len(gt_t) - 1)
    j = np.where(np.abs(gt_t[j - 1] - vt) < np.abs(gt_t[j] - vt), j - 1, j)
    m = np.abs(gt_t[j] - vt) < 2_000_000
    return vt[m], X[m], gt_p[j[m]]


def sim3_scale(X, Y):
    import vio_bench_hs as vb
    _, c, _ = vb.umeyama(X, Y, scale=True)
    return float(c)


def scaled(P, s, t_from=None):
    out = []
    for m in P:
        m = dict(m)
        if t_from is not None and m["t_ns"] < t_from:
            m["status"] = "init"
        if m.get("T_world_cam0") and m["status"] == "tracking":
            T = np.asarray(m["T_world_cam0"], float).reshape(4, 4).copy()
            T[:3, 3] *= s
            m["T_world_cam0"] = T.reshape(-1).tolist()
        out.append(m)
    return out


def write_pre(dst, P, rec, dt_ns, fps, extra=None):
    dst.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(str(dst) + ".poses.jsonl.gz", "wt") as f:
        for m in P:
            f.write(json.dumps(m) + "\n")
    d = dict(rec=rec, ate_cm=0.0, align_dt_ms=dt_ns / 1e6, fps=fps)
    d.update(extra or {})
    json.dump(d, open(str(dst) + ".json", "w"))


def score_one(res, kind, gs, ref_dir):
    """kind = causal | final. Returns metrics dict for one DPVO run (GT cells)."""
    import vio_bench_hs as vb
    rec, which = res["rec"], res["which"]
    gtname = GT if which == "hs" else STD_GT
    pre = OUT / res["cfg"] / f"{res['name']}_{res['cfg']}"
    P = load_lines(f"{pre}.{kind}.poses.jsonl.gz")
    dt_ns = int(json.load(open(MAIN / "recordings" / rec / "imu_gt" / "align.json"))["dt_ns"])
    out = dict(tracked=sum(m.get("status") == "tracking" for m in P), error=res.get("error"))
    mt = gt_match(rec, gtname, P, dt_ns)
    if mt is None or len(mt[0]) < 50:
        return dict(out, div=True, why="no tracking / few GT matches")
    tt, X, Y = mt
    # (a) Sim3 ATE with the bench code (evaluate) on the raw trajectory
    a = argparse.Namespace(gt=gtname, fps=res["fps"], size=SIZE, npts=0, clones=0, noise="cfg", imu_calib=None, lever=None,
                           dropout="", imu_raw=True, noise_scale=1.0)
    os.chdir(MAIN)
    ev = vb.evaluate(Path(f"recordings/{rec}"), a, P, res["frames_fed"], [], dt_ns, 0.0, {}, None, {})
    out.update(sim3=ev.get("ate_sim3_cm"), scale=ev.get("scale"), matched=ev.get("matched"), rpe1s_sim3=None)
    s = ev.get("scale")
    if not s:
        return dict(out, div=True, why="evaluate failed: " + str(ev.get("error")))
    # (b) oracle-scaled window ATE + divergence with the OpenVINS window metric
    tmp = OUT / "score_tmp" / res["cfg"] / kind
    dst = tmp / f"{res['name']}_{res['cfg']}"
    write_pre(dst, scaled(P, s), rec, dt_ns, res["fps"])
    mc = gs.cam_metrics(dst, rec, gtname)
    out.update(div=bool(mc.get("divergent")), ate_w=mc.get("ate_w"), rel5_max=mc.get("rel5_max"), window=mc.get("window"),
               why=mc.get("why"))
    # (c) OpenVINS reference window (same cell): first tracking t of the reference run
    if ref_dir is not None:
        rp = load_lines(str(ref_dir) + ".poses.jsonl.gz")
        rt = [m["t_ns"] for m in rp if m.get("status") == "tracking"]
        if rt:
            Pw = scaled(P, 1.0, t_from=rt[0])
            mw = gt_match(rec, gtname, Pw, dt_ns)
            if mw is not None and len(mw[0]) >= 50:
                sw = sim3_scale(mw[1], mw[2])
                dstw = tmp / f"{res['name']}_{res['cfg']}_ow"
                write_pre(dstw, scaled(Pw, sw), rec, dt_ns, res["fps"])
                mcw = gs.cam_metrics(dstw, rec, gtname)
                out.update(ate_w_ow=mcw.get("ate_w"), div_ow=bool(mcw.get("divergent")), ref_t0=int(rt[0]))
    # (d) scale from the first 5 s of tracking, (e) scale drift halves
    t0 = tt[0]
    e5 = tt <= t0 + 5e9
    if e5.sum() >= 30:
        s5 = sim3_scale(X[e5], Y[e5])
        dst5 = tmp / f"{res['name']}_{res['cfg']}_s5"
        write_pre(dst5, scaled(P, s5), rec, dt_ns, res["fps"])
        mc5 = gs.cam_metrics(dst5, rec, gtname)
        out.update(s5_over_s=round(s5 / s, 4), ate_w_s5=mc5.get("ate_w"), div_s5=bool(mc5.get("divergent")))
    h = len(tt) // 2
    if h >= 50:
        s1, s2 = sim3_scale(X[:h], Y[:h]), sim3_scale(X[h:], Y[h:])
        out.update(scale_drift=round(s2 / s1 - 1, 4))
    return out


def score():
    sys.path.insert(0, str(MAIN / "imu_learn"))
    sys.path.insert(0, str(MAIN))
    os.chdir(MAIN)
    import lens_switch_gtscore as gs
    from fi_limits_run import loop_eval
    gs.OUT = OUT / "score_tmp"
    gs.OUT.mkdir(parents=True, exist_ok=True)
    rows = []
    for cfgd in sorted(p for p in OUT.iterdir() if p.is_dir() and p.name in CFGS):
        for jf in sorted(cfgd.glob("*.json")):
            res = json.load(open(jf))
            fps = int(res["fps"])
            row = dict(name=res["name"], cfg=res["cfg"], rec=res["rec"], off=res["off"], which=res["which"], fps=fps,
                       key=res["name"].rsplit("_f", 1)[0], ms_med=res["ms_med"], ms_p95=res["ms_p95"], ms_max=res["ms_max"],
                       mem_mb=res["gpu_mem_alloc_max_mb"], mem_res_mb=res["gpu_mem_reserved_max_mb"], t_init=res["t_init_clip_s"],
                       error=res.get("error"), load1=res["load"][0], frames=res["frames_fed"], n_kf=res["n_keyframes"],
                       n_loop_ba=res.get("n_loop_ba"), loop_ba_t_s=res.get("loop_ba_t_s"))
            for kind in ("causal", "final"):
                if res["which"] == "loop":
                    P = load_lines(OUT / res["cfg"] / f"{res['name']}_{res['cfg']}.{kind}.poses.jsonl.gz")
                    tr = [m for m in P if m.get("status") == "tracking"]
                    tmpj = OUT / "score_tmp" / "loops" / f"{res['name']}_{res['cfg']}_{kind}.json"
                    tmpj.parent.mkdir(parents=True, exist_ok=True)
                    json.dump(dict(rec=res["rec"], tracked=len(tr), t_init_clip_s=res["t_init_clip_s"],
                                   traj=[[m["t_ns"]] + np.asarray(m["T_world_cam0"]).reshape(4, 4)[:3, 3].tolist() + [0, 0] for m in tr]),
                              open(tmpj, "w"))
                    e = loop_eval(tmpj)
                    row[kind] = {k: e.get(k) for k in ("loop_err_cm", "path_m", "loop_pct", "tracked")}
                    continue
                rd, rc = REF.get((res["which"], fps), (None, None))
                which = res["which"]
                k6 = row["key"]
                ref = Path(rd) / rc / f"{k6}_{rc}" if rd else None
                row[kind] = score_one(res, kind, gs, ref if ref is not None and Path(str(ref) + ".poses.jsonl.gz").exists() else None)
            rows.append(row)
            c = row.get("final", {})
            print(row["name"], row["cfg"], "final:", c.get("div"), c.get("sim3"), c.get("ate_w"), c.get("ate_w_ow"),
                  "causal:", row.get("causal", {}).get("ate_w"), flush=True)
    json.dump(dict(made=time.strftime("%F %T"), doc=__doc__, ref={f"{k[0]}{k[1]}": v for k, v in REF.items()}, rows=rows), open(OUT / "score.json", "w"), indent=1)
    json.dump(dict(made=time.strftime("%F %T"), doc=__doc__, ref={f"{k[0]}{k[1]}": v for k, v in REF.items()}, rows=rows), open(RD / "score.json", "w"), indent=1)
    print("saved", OUT / "score.json")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["prep", "job", "run", "score"])
    ap.add_argument("arg", nargs="?", default="")
    ap.add_argument("--cfg", default="base")
    ap.add_argument("--par", type=int, default=1)
    a = ap.parse_args()
    if a.cmd == "prep":
        prep(a.arg)
    elif a.cmd == "job":
        job(a.arg, a.cfg)
    elif a.cmd == "run":
        run_sched(a.arg, a.cfg, a.par)
    else:
        score()
