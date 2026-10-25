#!/usr/bin/env python3
"""ovacc 2026-10-25: offline GT accuracy of OpenVINS parameter / calibration variants of the live C4 setup.

Protocol = ovopt_20261025/ate_run.py ("fps30 C4 protocol", copied here with only these changes):
42 HS cells (14 recordings x start offsets 0/1/2; 213955 = phone drop, run but excluded from the summary), 30 fps,
960x540, clones 15, pts 200, init window 1.0 s / disparity 22.5 px, cfg_over "num_opencv_threads=1,try_zupt=false,
fi_max_cond_number=1e6,fi_max_baseline=200", lever, frame cache, strict init, per-recording R_ci / dt from
imu_gt/align.json; scored with lens_switch_gtscore (camera-position ruler, GT imu_gt_spl_cal1022).
Bridge = the SAME frozen binary as ovopt (sha256 71bb15d9..., copied from /tmp/ovopt/ate/bin), no --ovopt options
(B0 of ovopt = fps30 C4 poses in 45/45 cells). Single-threaded = deterministic, 1 run per cell.

Changes vs ate_run.py:
  - OUT = /tmp/ovacc/ate (env OVACC_ATE_OUT); B0 cells are copied from /tmp/ovopt/ate/B0 (same binary + protocol;
    3 cells re-run as B0R to confirm identity).
  - "L" family = live-like: histogram_method "NONE" (the live c4 config; the fps30 protocol used the template's
    HISTOGRAM). L0 = baseline of every L* condition.
  - per-condition overrides applied by wrapping vio_bench_hs.write_config (nothing in the main checkout is modified):
      dt_ms   : add to timeshift_cam_imu (s = ms/1000) = camera-IMU time-offset error of the prior (online calib on)
      fx      : scale fx, fy written to the config (image / remap unchanged) = focal-length error of the lens file
      live_cal: R_ci = chordal mean, dt = median of the 15 recordings (config/phone_ov_main/variants.json), p_CinI from
                that R_ci + lever = what the phone config uses (instead of the per-recording GT alignment)
      noise_scale / noise_allan: IMU noise densities (cfg = make_ov_config.IMU_NOISE; allan = runs/gyro_bias_stability)

  python3 acc_run.py freeze | run L0,LC [--par 6] | score L0,LC --ref L0
"""
import argparse
import gzip
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

MAIN = Path("/home/system/scanner/StereoDepthSeg/scan3d")
WT = Path("/home/system/scanner/StereoDepthSeg-ovacc/scan3d")
OUT = Path(os.environ.get("OVACC_ATE_OUT", "/tmp/ovacc/ate"))
SRC_BIN = Path("/tmp/ovopt/ate/bin/ov_bridge_71bb15d95c1f")
SRC_SHA = "71bb15d95c1feb12db9d9c66dfe0cbf5bae9b407f911144087cfeab73f330a2a"
PY = str(MAIN.parent / "venv_ffs" / "bin" / "python")
sys.path.insert(0, str(MAIN / "imu_learn"))
sys.path.insert(0, str(MAIN))
os.chdir(MAIN)  # the bench resolves recordings/, config/ relative to the main checkout

GT = "imu_gt_spl_cal1022"
OFFS = (0, 1, 2)
LEVER = "config/imu_cam_lever.json"
BASE_CFG = "num_opencv_threads=1,try_zupt=false,fi_max_cond_number=1e6,fi_max_baseline=200"  # = fps30 C4
LIVE = 'histogram_method="NONE"'
VARJSON = WT / "config" / "phone_ov_main" / "variants.json"


def L(**kw):
    """live-like condition: histogram NONE + extra cfg"""
    d = dict(kw)
    d["cfg"] = LIVE + ("," + kw["cfg"] if kw.get("cfg") else "")
    return d


CONDS = {
    "B0": dict(),                                   # fps30 C4 protocol (HISTOGRAM), copied from ovopt
    "B0R": dict(),                                  # re-run of B0 (identity check, a few cells)
    "L0": L(),                                      # live-like baseline (histogram NONE)
    # calibration sensitivity (on L0)
    "LC": L(live_cal=True),                         # live R_ci / dt instead of per-recording GT alignment
    "DTp1": L(dt_ms=1.0), "DTm1": L(dt_ms=-1.0), "DTp2": L(dt_ms=2.0), "DTm2": L(dt_ms=-2.0),
    "FXp": L(fx=1.003), "FXm": L(fx=0.997),
    # estimator parameters (on L0)
    "F10": L(cfg="fast_threshold=10"), "F20": L(cfg="fast_threshold=20"), "F25": L(cfg="fast_threshold=25"),
    "G85": L(cfg="grid_x=8,grid_y=5"), "MD15": L(cfg="min_px_dist=15"), "MD7": L(cfg="min_px_dist=7"),
    "SG15": L(cfg="up_msckf_sigma_px=1.5,up_slam_sigma_px=1.5"), "SG07": L(cfg="up_msckf_sigma_px=0.7,up_slam_sigma_px=0.7"),
    "SM15": L(cfg="up_msckf_sigma_px=1.5"), "SS15": L(cfg="up_slam_sigma_px=1.5"),
    "CH2": L(cfg="up_msckf_chi2_multipler=2,up_slam_chi2_multipler=2"),
    "NC05": L(noise_scale=0.5), "NC2": L(noise_scale=2.0), "NA10": L(noise_allan=10.0),
    "NG05": L(noise_gyro=0.5), "NAC05": L(noise_acc=0.5),
    "M60": L(cfg="max_msckf_in_update=60"), "M80": L(cfg="max_msckf_in_update=80"),
    "S70": L(cfg="max_slam=70"),
    "N300M80": L(npts=300, cfg="max_msckf_in_update=80"),
    "C12": L(clones=12), "C18": L(clones=18),
    "DS2": L(cfg="dt_slam_delay=2"),
    "F30": L(cfg="fast_threshold=30"),
    "F40": L(cfg="fast_threshold=40"),
}


def cells():
    from phone60_run import RAW, OLD, DROP  # noqa: E402
    return [(r, o) for r in RAW + OLD for o in OFFS], DROP, set(OLD)


# supplementary live-like set (NOT in the 42-cell summary): std60 board recordings = live camera mode (focus 0.5, 2 ms,
# auto-ISO-then-fixed, std lens std_1920x1080_f0.50_cal20261024d via the bench's shared-lens rule), 60 fps saved, fed at
# 30; GT imu_gt_spl (board, 60.40 mm square); docs/std60_board_check_20261024.md. 173432 does not initialise with the
# 1.0 s window in the earlier check.
STD = ["hs_20261024_165506_std60", "hs_20261024_173432_std60"]
STD_GT = "imu_gt_spl"
STD_SUMMARY = MAIN / "runs" / "std60_board_20261024" / "summary.json"


def cells_std():
    return [(r, o) for r in STD for o in OFFS]


def wrapper(cond):
    return OUT / "bin" / f"br_{cond}.sh"


def freeze():
    (OUT / "bin").mkdir(parents=True, exist_ok=True)
    h = hashlib.sha256(open(SRC_BIN, "rb").read()).hexdigest()
    if h != SRC_SHA:
        sys.exit(f"frozen ovopt binary changed: {h}")
    dst = OUT / "bin" / f"ov_bridge_{h[:12]}"
    if not dst.exists():
        shutil.copy2(SRC_BIN, dst)
    git = subprocess.run(["git", "-C", str(WT), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    json.dump(dict(src=str(SRC_BIN), sha256=h, copy=str(dst), frozen=time.strftime("%F %T"), worktree_head=git,
                   note="same binary as ovopt B0 (/tmp/ovopt/ate/bin/bridge.json, built from ovopt e1429b9)"),
              open(OUT / "bin" / "bridge.json", "w"), indent=1)
    for c in CONDS:
        w = wrapper(c)
        w.write_text(f"#!/bin/bash\nexec {dst} \"$@\"\n")
        w.chmod(0o755)
    # B0 = ovopt's B0 cells (same binary, same protocol)
    (OUT / "B0").mkdir(exist_ok=True)
    for f in Path("/tmp/ovopt/ate/B0").iterdir():
        if f.name.endswith(".json") or f.name.endswith(".poses.jsonl.gz"):
            if not (OUT / "B0" / f.name).exists():
                shutil.copy2(f, OUT / "B0" / f.name)
    print("frozen", dst, h)


def name_of(rec, off, cond):
    return f"{rec.split('_')[2]}_o{off}_{cond}"   # HHMMSS (= rec[-6:] for HS, also right for *_std60)


def job(name):
    import vio_bench_hs as vb
    import make_ov_config
    rec6, o, cond = name.split("_")
    allc, drop, old = cells()
    rec = next(r for r, _ in allc + cells_std() if r.endswith(rec6) or r.endswith(rec6 + "_std60"))
    off = int(o[1:])
    d = CONDS[cond]
    vb.BRIDGE = wrapper(cond)
    vb.TEMPLATE = MAIN / "config" / "mono" / "estimator_config.yaml"
    import tempfile as _tf
    import types
    (OUT / "tmp").mkdir(parents=True, exist_ok=True)
    vb.tempfile = types.SimpleNamespace(mkdtemp=lambda prefix="vio_", dir=None: _tf.mkdtemp(prefix=prefix, dir=str(OUT / "tmp")))
    applied = {}
    orig_wc = vb.write_config

    def wc(dd, K, size, fps, R_ci, p_CinI, timeshift, noise, npts, clones, init_cfg="strict", init_over=None):
        K = np.array(K, float)
        if d.get("fx"):
            K[0, 0] *= d["fx"]
            K[1, 1] *= d["fx"]
        if d.get("live_cal"):
            vj = json.load(open(VARJSON))
            al = json.load(open(MAIN / "recordings" / rec / "imu_gt" / "align.json"))
            R_live = np.asarray(vj["R_ci"], float)
            r_cm = np.asarray(json.load(open(MAIN / LEVER))["r_cm"], float)
            p_CinI = -R_live.T @ (r_cm / 100.0)
            # frames are fed at t_cam - dt_rec; live: OpenVINS t_imu = t_cam + timeshift_live (= -dt_live)
            dt_live_ns = -float(vj["timeshift_cam_imu_s"]) * 1e9      # = median of vj["dt_ns"] (list of 15)
            timeshift = timeshift + (int(al["dt_ns"]) - dt_live_ns) * 1e-9
            dR = np.degrees(np.arccos(np.clip((np.trace(R_live.T @ np.asarray(R_ci)) - 1) / 2, -1, 1)))
            applied.update(live_R_diff_deg=float(dR), live_dt_minus_rec_ms=(dt_live_ns - int(al["dt_ns"])) * 1e-6)
            R_ci = R_live
        if d.get("dt_ms"):
            timeshift = timeshift + d["dt_ms"] * 1e-3
        noise = dict(noise)
        if d.get("noise_scale"):
            noise = {k: v * d["noise_scale"] for k, v in noise.items()}
        if d.get("noise_allan"):
            noise = {k: v * d["noise_allan"] for k, v in vb.allan_noise().items()}
        if d.get("noise_gyro"):
            for k in ("gyroscope_noise_density", "gyroscope_random_walk"):
                noise[k] *= d["noise_gyro"]
        if d.get("noise_acc"):
            for k in ("accelerometer_noise_density", "accelerometer_random_walk"):
                noise[k] *= d["noise_acc"]
        applied.update(timeshift=timeshift, fx=float(K[0, 0]), fy=float(K[1, 1]), noise=noise,
                       R_ci=np.asarray(R_ci).tolist(), p_CinI=np.asarray(p_CinI).tolist())
        return orig_wc(dd, K, size, fps, R_ci, p_CinI, timeshift, noise, npts, clones, init_cfg, init_over)
    vb.write_config = wc
    cfg = BASE_CFG + ("," + d["cfg"] if d.get("cfg") else "")
    a = argparse.Namespace(rec=f"recordings/{rec}", fps=30.0, size="960x540", npts=d.get("npts", 200),
                           clones=d.get("clones", 15), noise="cfg", noise_scale=1.0, imu_calib=None, lever=LEVER,
                           lens_file=None, dropout="", max_secs=0.0, imu_raw=rec not in old, gt="auto", out=None,
                           init_cfg="strict", start_off=float(off), decode_threads=2, imu_lead=2.0, init_window=1.0,
                           init_disp=22.5, frame_cache=True, gyro_rebias=False, cfg_over=cfg, still_hold=None)
    saved = {}
    orig = vb.evaluate

    def ev(rec_, a_, poses, *rest):
        saved["poses"] = poses
        return orig(rec_, a_, poses, *rest)
    vb.evaluate = ev
    t0 = time.time()
    try:
        r = vb.run(a)
    finally:
        vb.evaluate = orig
        vb.write_config = orig_wc
    br = json.load(open(OUT / "bin" / "bridge.json"))
    r.update(ovacc_made=time.strftime("%F %T"), ovacc_job_s=round(time.time() - t0, 1), ovacc_runner=str(Path(__file__)),
             ovacc_cond=cond, ovacc_cond_def=d, ovacc_applied=applied, ovacc_bridge_sha256=br["sha256"],
             ovacc_worktree_head=br["worktree_head"], ovacc_load=list(os.getloadavg()))
    od = OUT / cond
    od.mkdir(parents=True, exist_ok=True)
    with gzip.open(str(od / name) + ".poses.jsonl.gz", "wt") as f:
        for m in saved.get("poses", []):
            f.write(json.dumps(m) + "\n")
    json.dump(r, open(od / f"{name}.json", "w"), indent=1, ensure_ascii=False)


def run_sched(conds, par):
    allc, _, _ = cells()
    q = [name_of(r, o, c) for c in conds for r, o in allc]
    q = [n for n in q if not (OUT / n.split("_")[2] / f"{n}.json").exists()]
    running, t_all = {}, time.time()
    print(f"{time.strftime('%F %T')} start {conds} {len(q)} jobs PAR {par}", flush=True)
    while q or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is not None:
                del running[n]
                ok = (OUT / n.split("_")[2] / f"{n}.json").exists()
                print(f"{time.strftime('%F %T')} done {n} {time.time() - t0:.0f} s {'ok' if ok else 'FAILED'}", flush=True)
        while q and len(running) < par:
            n = q.pop(0)
            (OUT / n.split("_")[2]).mkdir(parents=True, exist_ok=True)
            err = open(OUT / n.split("_")[2] / f"{n}.err", "w")
            running[n] = (subprocess.Popen(["nice", "-n", "5", PY, str(Path(__file__).resolve()), "job", n],
                                           stdout=subprocess.DEVNULL, stderr=err), time.time())
        time.sleep(2)
    print(f"{time.strftime('%F %T')} end {conds} wall {time.time() - t_all:.0f} s", flush=True)


def score(conds, ref):
    import lens_switch_gtscore as gs
    gs.OUT = OUT / "score_tmp"
    gs.OUT.mkdir(parents=True, exist_ok=True)
    allc, drop, _ = cells()
    rows = []
    for r, o in allc:
        row = dict(rec=r, off=o, key=f"{r[-6:]}_o{o}")
        for c in conds:
            pre = OUT / c / name_of(r, o, c)
            if not Path(str(pre) + ".json").exists():
                continue
            st = json.load(open(str(pre) + ".json"))
            mc = gs.cam_metrics(pre, r, GT)
            be = gs.bench_eval(pre, r, GT)
            P = [json.loads(x) for x in gzip.open(str(pre) + ".poses.jsonl.gz", "rt") if x.strip()]
            ms = np.array([m["ms"] for m in P if "ms" in m], float)
            h = hashlib.sha256()
            for m in P:
                h.update(json.dumps({k: v for k, v in m.items() if k not in ("ms", "remap_ms", "wait_ms")},
                                    sort_keys=True).encode())
            row[c] = dict(div=bool(mc.get("divergent")), ate_w=mc.get("ate_w"), why=mc.get("why"),
                          sim3=be.get("ate_sim3_cm"), scale=be.get("scale"), tracked=st.get("tracked"),
                          n_slam_med=float(np.median([m.get("n_slam", 0) for m in P if m.get("status") == "tracking"] or [0])),
                          ms_med=round(float(np.median(ms)), 2) if len(ms) else None,
                          ms_p95=round(float(np.percentile(ms, 95)), 2) if len(ms) else None, poses_hash=h.hexdigest()[:16])
        rows.append(row)
    use = [x for x in rows if x["rec"] != drop]
    summ = {}
    for c in conds:
        cc = [x for x in use if c in x]
        ok = [x for x in cc if not x[c]["div"]]
        med = lambda f: round(float(np.median([f(x[c]) for x in ok if f(x[c]) is not None])), 4) if ok else None
        s = dict(n=len(cc), div=sum(x[c]["div"] for x in cc), div_cells=[x["key"] for x in cc if x[c]["div"]],
                 ate_w_med=med(lambda y: y["ate_w"]), sim3_med=med(lambda y: y["sim3"]), scale_med=med(lambda y: y["scale"]),
                 ms_med_of_cells=round(float(np.median([x[c]["ms_med"] for x in cc])), 2) if cc else None,
                 ms_p95_of_cells=round(float(np.median([x[c]["ms_p95"] for x in cc])), 2) if cc else None)
        if ref in conds and c != ref:
            both = [x for x in cc if ref in x and not x[c]["div"] and not x[ref]["div"]]
            dd = [x[c]["ate_w"] - x[ref]["ate_w"] for x in both]
            s.update(paired_n=len(both), d_med=round(float(np.median(dd)), 4) if dd else None,
                     d_mean=round(float(np.mean(dd)), 4) if dd else None, n_better=int(sum(v < 0 for v in dd)),
                     identical_cells=sum(1 for x in cc if ref in x and x[c]["poses_hash"] == x[ref]["poses_hash"]),
                     big_changes=[(x["key"], round(x[ref]["ate_w"], 3), round(x[c]["ate_w"], 3)) for x in both
                                  if abs(x[c]["ate_w"] - x[ref]["ate_w"]) > 0.3])
        summ[c] = s
    out = dict(made=time.strftime("%F %T"), conds=conds, ref=ref, protocol=__doc__.split("\n\n")[1], summary=summ, cells=rows,
               bridge=json.load(open(OUT / "bin" / "bridge.json")))
    tag = "_".join(conds)
    json.dump(out, open(OUT / f"score_{tag}.json", "w"), indent=1, ensure_ascii=False)
    print(f"| cond | div | ate_w med | sim3 med | scale med | vs {ref}: paired n, d_med (mean) cm, better | identical cells | ms med / p95 (cell medians) |")
    print("|---|---|---|---|---|---|---|---|")
    for c in conds:
        s = summ[c]
        pr = f"{s.get('paired_n')}, {s.get('d_med')} ({s.get('d_mean')}), {s.get('n_better')}" if c != ref else "-"
        print(f"| {c} | {s['div']} {s['div_cells']} | {s['ate_w_med']} | {s['sim3_med']} | {s['scale_med']} | {pr} | "
              f"{s.get('identical_cells', '-')} | {s['ms_med_of_cells']} / {s['ms_p95_of_cells']} |")
    print("saved", OUT / f"score_{tag}.json")




if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["freeze", "run", "job", "score"])
    ap.add_argument("arg", nargs="?", default="")
    ap.add_argument("--par", type=int, default=6)
    ap.add_argument("--ref", default="L0")
    ap.add_argument("--only", default="", help="comma list of cell keys (e.g. 213341_o0) to restrict 'run'")
    a = ap.parse_args()
    if a.cmd == "freeze":
        freeze()
    elif a.cmd == "run":
        if a.only:
            _cells = cells
            keep = set(a.only.split(","))

            def cells():  # noqa: F811
                allc, drop, old = _cells()
                return [(r, o) for r, o in allc if f"{r[-6:]}_o{o}" in keep], drop, old
            globals()["cells"] = cells
        run_sched(a.arg.split(","), a.par)
    elif a.cmd == "job":
        job(a.arg)
    else:
        score(a.arg.split(","), a.ref)
