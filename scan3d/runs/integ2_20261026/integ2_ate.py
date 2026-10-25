#!/usr/bin/env python3
"""integ2-20261026: copy of runs/ov60eval_20261026/ov60_run.py (same runner, same ruler: 42 HS cells, GT imu_gt_spl_cal1022,
lens_switch_gtscore) with only the bridge and the conditions changed: 30 fps, new rec20261025 profile (N = + sym_inplace,
ekf_sparse) vs old rec20261025 profile (O = --ovopt all_exact,ekf_gemm,qr_compress,ekf_chol given explicitly), both with
the integ2 PC build and --drop-late 0; P = bench template (HISTOGRAM) as ov60eval P30, L = histogram NONE as L30.
Original ov60eval docstring follows.

ov60eval 2026-10-26: offline accuracy of 60 fps vs 30 fps on the same GT data with the integ live profiles.

Protocol = runs/integ_20261025/ate_run.py (= ovopt / fps30 C4 protocol) with only these changes:
  - bridge = this branch's PC build (ov60eval-20261026 = integ-20261025 14de3b8, no code change, -DOV_FULL_SOURCE=ON,
    frozen copy in OUT/bin, sha256 in OUT/bin/bridge.json)
  - fps 30 or 60 (the bench picks the frame nearest to each 1/fps time step from the 240 fps HS recordings / 60 fps std60)
  - bridge args = the integ profile of that rate + "--drop-late 0": rec20261025 at 30 fps, rec60 at 60 fps.
    --drop-late is wall-clock based (CLOCK_BOOTTIME - frame t); offline the bench is not paced, so on a slow / loaded
    PC it would drop frames non-deterministically -> switched off (explicit flag wins over the profile). All other
    profile parts (W3 exact / near-exact math, idle spin, health, predict 2 @200 Hz + pred thread) do not change poses
    (integ identity checks) or are timing-only.
  - two config families: P = bench template (histogram_method HISTOGRAM) = the established 42-cell ruler
    (integ PR / ovopt B0, W3); L = live-like (histogram_method "NONE", the phone's c4 / c4f60 config) = ovacc L0.
Common: 960x540, pts 200, clones 15 (c4f60 keeps 15), init window 1.0 s / disparity 22.5 px, cfg_over
"num_opencv_threads=1,try_zupt=false,fi_max_cond_number=1e6,fi_max_baseline=200", lever, frame cache, strict init,
per-recording R_ci / dt (imu_gt/align.json). Single-threaded = deterministic; repeats = the 3 start offsets (0/1/2 s).

Sets:
  hs   : 42 HS cells (14 recordings x offsets 0/1/2) + 213955 (phone drop, run but excluded from the summary),
         GT imu_gt_spl_cal1022, scorer lens_switch_gtscore (camera-position ruler): divergence flag, window ATE, Sim3, scale
  std  : std60 board recordings 165506, 173432 (live camera mode, native 60 fps) x offsets 0/1/2, GT imu_gt_spl
  loop : std60 hand loops 142148, 142610, 151646 (native 60 fps, no GT), offset 0: loop_eval (start-end error / path)
Conditions: P30, P60, L30, L60 (the comparison) + P60C30, L60C30 (informative: 60 fps with clones 30 = the same 0.5 s
clone window as 30 fps x 15; not a phone setting, ~+3 ms/frame estimated from clones +5 ~ +1 ms).

  python3 ov60_run.py freeze
  python3 ov60_run.py run hs:P30,P60 [--par 6]      # set:conds
  python3 ov60_run.py score                          # -> OUT/score.json + RD/score.json + table
OUT = /tmp/ov60eval/ate (env OV60_OUT).
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
WT = Path("/home/system/scanner/StereoDepthSeg-integ2/scan3d")
RD = WT / "runs" / "integ2_20261026"
OUT = Path(os.environ.get("OV60_OUT", "/tmp/integ2/ate"))
SRC = WT / "ov_bridge" / "build_integ2_full" / "ov_bridge"
PY = str(MAIN.parent / "venv_ffs" / "bin" / "python")
sys.path.insert(0, str(MAIN / "imu_learn"))
sys.path.insert(0, str(MAIN))
os.chdir(MAIN)  # the bench resolves recordings/, config/ relative to the main checkout (read only)

GT = "imu_gt_spl_cal1022"
STD_GT = "imu_gt_spl"
OFFS = (0, 1, 2)
LEVER = "config/imu_cam_lever.json"
BASE_CFG = "num_opencv_threads=1,try_zupt=false,fi_max_cond_number=1e6,fi_max_baseline=200"
LIVE = 'histogram_method="NONE"'
STD = ["hs_20261024_165506_std60", "hs_20261024_173432_std60"]
LOOP = ["hs_20261024_142148_std60", "hs_20261024_142610_std60", "hs_20261024_151646_std60"]
P30 = "--profile rec20261025 --drop-late 0"
P60 = "--profile rec60 --drop-late 0"
OLDP = "--profile rec20261025 --drop-late 0 --ovopt all_exact,ekf_gemm,qr_compress,ekf_chol"  # integ-20261025 profile
NEWP = "--profile rec20261025 --drop-late 0"  # integ2: profile ovopt = all_exact,ekf_gemm,qr_compress,ekf_chol,sym_inplace,ekf_sparse
CONDS = {
    "O30": dict(fps=30.0, args=OLDP),
    "N30": dict(fps=30.0, args=NEWP),
    "OL30": dict(fps=30.0, args=OLDP, cfg=LIVE),
    "NL30": dict(fps=30.0, args=NEWP, cfg=LIVE),
    # gyro (user 2026-10-26 morning): profile now has gyro_klt=5. Run with OV60_OUT=/tmp/integ2/ate_g and the new build;
    # N30 / NL30 there = copies of the /tmp/integ2/ate results made with the 0.6.26 build (profile without gyro_klt),
    # L60 there = copies of ov60eval L60 (integ 14de3b8 build, old rec60 profile) = reference only.
    "G30": dict(fps=30.0, args=NEWP),
    "GL30": dict(fps=30.0, args=NEWP, cfg=LIVE),
    "L60": dict(fps=60.0, args=P60, cfg=LIVE),
    "NL60": dict(fps=60.0, args=P60 + " --ovopt all_exact,ekf_gemm,qr_compress,ekf_chol,sym_inplace,ekf_sparse", cfg=LIVE),
    "GL60": dict(fps=60.0, args=P60, cfg=LIVE),
}
PAIRS = [("O30", "N30"), ("OL30", "NL30"), ("N30", "G30"), ("NL30", "GL30"), ("L60", "NL60"), ("NL60", "GL60")]


def hs_cells():
    from phone60_run import RAW, OLD, DROP  # noqa: E402
    return [(r, o) for r in RAW + OLD for o in OFFS], DROP, set(OLD)


def cells(which):
    if which == "hs":
        return hs_cells()[0]
    if which == "std":
        return [(r, o) for r in STD for o in OFFS]
    if which == "loop":
        return [(r, 0) for r in LOOP]
    raise ValueError(which)


def key_of(rec, off, which):
    hh = rec.split("_")[2]
    return {"hs": f"{hh}_o{off}", "std": f"s{hh}_o{off}", "loop": f"l{hh}_o{off}"}[which]


def name_of(rec, off, cond, which):
    return f"{key_of(rec, off, which)}_{cond}"


def wrapper(cond):
    return OUT / "bin" / f"br_{cond}.sh"


def freeze():
    (OUT / "bin").mkdir(parents=True, exist_ok=True)
    h = hashlib.sha256(open(SRC, "rb").read()).hexdigest()
    dst = OUT / "bin" / f"ov_bridge_{h[:12]}"
    if not dst.exists():
        shutil.copy2(SRC, dst)
    git = subprocess.run(["git", "-C", str(WT), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(["git", "-C", str(WT), "status", "--short", "ov_bridge"], capture_output=True, text=True).stdout
    json.dump(dict(src=str(SRC), sha256=h, copy=str(dst), frozen=time.strftime("%F %T"), worktree_head=git,
                   ov_bridge_dirty=[x for x in dirty.splitlines() if "build_integ2_full" not in x]),
              open(OUT / "bin" / "bridge.json", "w"), indent=1)
    for c, d in CONDS.items():
        wrapper(c).write_text(f"#!/bin/bash\nexec {dst} \"$@\" {d['args']}\n")
        wrapper(c).chmod(0o755)
    print("frozen", dst, h)


def which_of(key):
    return {"s": "std", "l": "loop"}.get(key[0], "hs")


def job(name):
    import vio_bench_hs as vb
    key6, o, cond = name.split("_")
    which = which_of(key6)
    rec = next(r for r, oo in cells(which) if r.split("_")[2] == key6.lstrip("sl"))
    off = int(o[1:])
    _, _, old = hs_cells()
    d = CONDS[cond]
    vb.BRIDGE = wrapper(cond)
    vb.TEMPLATE = MAIN / "config" / "mono" / "estimator_config.yaml"
    import tempfile as _tf
    import types
    (OUT / "tmp").mkdir(parents=True, exist_ok=True)
    vb.tempfile = types.SimpleNamespace(mkdtemp=lambda prefix="vio_", dir=None: _tf.mkdtemp(prefix=prefix, dir=str(OUT / "tmp")))
    cfg = BASE_CFG + ("," + d["cfg"] if d.get("cfg") else "")
    a = argparse.Namespace(rec=f"recordings/{rec}", fps=d["fps"], size="960x540", npts=200, clones=d.get("clones", 15),
                           noise="cfg", noise_scale=1.0, imu_calib=None, lever=LEVER, lens_file=None, dropout="",
                           max_secs=0.0, imu_raw=rec not in old, gt="none" if which == "loop" else "auto", out=None,
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
    br = json.load(open(OUT / "bin" / "bridge.json"))
    r.update(o60_made=time.strftime("%F %T"), o60_job_s=round(time.time() - t0, 1), o60_runner=str(Path(__file__)),
             o60_cond=cond, o60_cond_def=d, o60_set=which, o60_bridge_sha256=br["sha256"],
             o60_worktree_head=br["worktree_head"], o60_load=list(os.getloadavg()))
    od = OUT / cond
    od.mkdir(parents=True, exist_ok=True)
    if saved.get("poses") is not None:   # gt none: evaluate is not called -> poses come from r["traj"] only
        with gzip.open(str(od / name) + ".poses.jsonl.gz", "wt") as f:
            for m in saved["poses"]:
                f.write(json.dumps(m) + "\n")
    json.dump(r, open(od / f"{name}.json", "w"), indent=1, ensure_ascii=False)


def run_sched(spec, par):
    q = []
    for part in spec.split(";"):
        which, conds = part.split(":")
        q += [name_of(r, o, c, which) for c in conds.split(",") for r, o in cells(which)]
    q = [n for n in q if not (OUT / n.split("_")[2] / f"{n}.json").exists()]
    running, t_all = {}, time.time()
    print(f"{time.strftime('%F %T')} start {spec} {len(q)} jobs PAR {par}", flush=True)
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
    print(f"{time.strftime('%F %T')} end {spec} wall {time.time() - t_all:.0f} s", flush=True)


def ms_stats(P):
    a = np.array([m["ms"] for m in P if "ms" in m], float)
    return (round(float(np.median(a)), 2), round(float(np.percentile(a, 95)), 2)) if len(a) else (None, None)


def load_P(pre):
    f = Path(str(pre) + ".poses.jsonl.gz")
    return [json.loads(x) for x in gzip.open(f, "rt") if x.strip()] if f.exists() else []


def score_hs_std(which, conds, gs):
    rows = []
    gt = GT if which == "hs" else STD_GT
    for r, o in cells(which):
        row = dict(rec=r, off=o, key=key_of(r, o, which))
        for c in conds:
            pre = OUT / c / name_of(r, o, c, which)
            if not Path(str(pre) + ".json").exists():
                continue
            st = json.load(open(str(pre) + ".json"))
            mc = gs.cam_metrics(pre, r, gt)
            be = gs.bench_eval(pre, r, gt) if which == "hs" else {}
            P = load_P(pre)
            trk = [m for m in P if m.get("status") == "tracking"]
            h = hashlib.sha256()
            for m in P:
                h.update(json.dumps({k: v for k, v in m.items() if k not in ("ms", "remap_ms", "wait_ms")},
                                    sort_keys=True).encode())
            msm, msp = ms_stats(P)
            row[c] = dict(div=bool(mc.get("divergent")), ate_w=mc.get("ate_w"), rel5_max=mc.get("rel5_max"), why=mc.get("why"),
                          window=mc.get("window"), sim3=be.get("ate_sim3_cm"), scale=be.get("scale") or st.get("scale"),
                          tracked=st.get("tracked"), frames_fed=st.get("frames_fed"), t_init_clip_s=st.get("t_init_clip_s"),
                          n_slam_med=float(np.median([m.get("n_slam", 0) for m in trk] or [0])),
                          late_dropped=max([m.get("late_dropped", 0) for m in P] or [0]),
                          pc_ms_med=msm, pc_ms_p95=msp, poses_hash=h.hexdigest()[:16])
        rows.append(row)
    return rows


def summarize(rows, conds, drop):
    use = [x for x in rows if x["rec"] != drop]
    S = {}
    for c in conds:
        cc = [x for x in use if c in x]
        if not cc:
            continue
        ok = [x for x in cc if not x[c]["div"] and x[c]["ate_w"] is not None]
        med = lambda f: round(float(np.median([f(x[c]) for x in ok if f(x[c]) is not None])), 4) if ok else None
        S[c] = dict(n=len(cc), div=sum(x[c]["div"] for x in cc), div_cells=[x["key"] for x in cc if x[c]["div"]],
                    noinit=[x["key"] for x in cc if not x[c]["tracked"]],
                    ate_w_med=med(lambda y: y["ate_w"]), ate_w_mean=round(float(np.mean([x[c]["ate_w"] for x in ok])), 4) if ok else None,
                    sim3_med=med(lambda y: y["sim3"]), scale_med=med(lambda y: y["scale"]),
                    t_init_med=med(lambda y: y["t_init_clip_s"]), n_slam_med=med(lambda y: y["n_slam_med"]),
                    late_dropped_max=max(x[c]["late_dropped"] for x in cc),
                    per_off={o: dict(div=sum(x[c]["div"] for x in cc if x["off"] == o),
                                     ate_w_med=round(float(np.median([x[c]["ate_w"] for x in ok if x["off"] == o])), 4)
                                     if any(x["off"] == o for x in ok) else None) for o in OFFS},
                    pc_ms_med_of_cells=round(float(np.median([x[c]["pc_ms_med"] for x in cc if x[c]["pc_ms_med"]])), 2))
    P = {}
    for a, b in PAIRS:
        if a not in S or b not in S:
            continue
        both = [x for x in use if a in x and b in x and not x[a]["div"] and not x[b]["div"]
                and x[a]["ate_w"] is not None and x[b]["ate_w"] is not None]
        d = np.array([x[b]["ate_w"] - x[a]["ate_w"] for x in both])
        recs = sorted({x["rec"] for x in both})
        per_rec = [float(np.mean([x[b]["ate_w"] - x[a]["ate_w"] for x in both if x["rec"] == r])) for r in recs]
        P[f"{a}->{b}"] = dict(n_both=len(both), d_med=round(float(np.median(d)), 4) if len(d) else None,
                              d_mean=round(float(np.mean(d)), 4) if len(d) else None, n_b_better=int((d < 0).sum()),
                              rec_b_better=f"{sum(v < 0 for v in per_rec)}/{len(per_rec)}",
                              div_only_a=[x["key"] for x in use if a in x and b in x and x[a]["div"] and not x[b]["div"]],
                              div_only_b=[x["key"] for x in use if a in x and b in x and x[b]["div"] and not x[a]["div"]],
                              big=[(x["key"], round(x[a]["ate_w"], 2), round(x[b]["ate_w"], 2)) for x in both
                                   if abs(x[b]["ate_w"] - x[a]["ate_w"]) > 0.3])
    return S, P


def score_loops(conds):
    from fi_limits_run import loop_eval  # noqa: E402
    out = {}
    for r, o in cells("loop"):
        k = key_of(r, o, "loop")
        for c in conds:
            pre = OUT / c / name_of(r, o, c, "loop")
            if Path(str(pre) + ".json").exists():
                st = json.load(open(str(pre) + ".json"))
                e = loop_eval(Path(str(pre) + ".json"))
                out.setdefault(k, {})[c] = dict({x: e.get(x) for x in ("loop_err_cm", "path_m", "loop_pct", "tracked",
                                                                       "n_slam_motion_med", "n_slam_motion_frac0",
                                                                       "drift_final_still_cm", "t_init_clip_s")},
                                                frames_fed=st.get("frames_fed"))
    return out


def score():
    import lens_switch_gtscore as gs
    gs.OUT = OUT / "score_tmp"
    gs.OUT.mkdir(parents=True, exist_ok=True)
    conds = list(CONDS)
    _, drop, _ = hs_cells()
    hs = score_hs_std("hs", conds, gs)
    Shs, Phs = summarize(hs, conds, drop)
    loops = {}  # integ2: HS 42 cells (+ std60 GT cells for the rec60 gyro sanity check, conds present only)
    std = score_hs_std("std", conds, gs)
    Sstd, Pstd = summarize(std, conds, None)
    res = dict(made=time.strftime("%F %T"), protocol=__doc__, conds=CONDS, bridge=json.load(open(OUT / "bin" / "bridge.json")),
               hs=dict(summary=Shs, paired=Phs, excluded=drop, cells=hs), std=dict(summary=Sstd, paired=Pstd, cells=std),
               loops=loops)
    json.dump(res, open(OUT / "score.json", "w"), indent=1, ensure_ascii=False)
    RD.mkdir(parents=True, exist_ok=True)
    rd_score = RD / "score.json" if OUT == Path("/tmp/integ2/ate") else RD / "gyro" / "score_gyro.json"  # keep the 0.6.26 score
    rd_score.parent.mkdir(parents=True, exist_ok=True)
    json.dump(res, open(rd_score, "w"), indent=1, ensure_ascii=False)
    for nm, S, P in (("HS 42", Shs, Phs), ("std60 GT", Sstd, Pstd)):
        print(f"\n## {nm}\n| cond | n | div | ate_w med (mean) cm | sim3 med | scale med | init s med | n_slam med | pc ms med |")
        print("|---|---|---|---|---|---|---|---|---|")
        for c, s in S.items():
            print(f"| {c} | {s['n']} | {s['div']} {s['div_cells']} | {s['ate_w_med']} ({s['ate_w_mean']}) | {s['sim3_med']} | "
                  f"{s['scale_med']} | {s['t_init_med']} | {s['n_slam_med']} | {s['pc_ms_med_of_cells']} |")
        print("| pair | n both | d_med (mean) cm | b better | recs b better | div only a / only b |")
        print("|---|---|---|---|---|---|")
        for k, p in P.items():
            print(f"| {k} | {p['n_both']} | {p['d_med']} ({p['d_mean']}) | {p['n_b_better']} | {p['rec_b_better']} | "
                  f"{p['div_only_a']} / {p['div_only_b']} |")
    print("\n## std60 loops (no GT)")
    for k, v in loops.items():
        print(k, {c: (x["loop_err_cm"], x["path_m"], x["tracked"]) for c, x in v.items()})
    print("saved", OUT / "score.json", rd_score)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["freeze", "run", "job", "score"])
    ap.add_argument("arg", nargs="?", default="")
    ap.add_argument("--par", type=int, default=6)
    a = ap.parse_args()
    if a.cmd == "freeze":
        freeze()
    elif a.cmd == "run":
        run_sched(a.arg, a.par)
    elif a.cmd == "job":
        job(a.arg)
    else:
        score()
