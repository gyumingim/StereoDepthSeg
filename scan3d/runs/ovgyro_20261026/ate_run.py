#!/usr/bin/env python3
"""[ovgyro-20261026 copy of integ_20261025/ate_run.py: WT = ovgyro worktree, build_ovgyro_full, OUT /tmp/ovgyro/ate,
conditions B0K/PRK (= B0/PR + --klt-stats 1, output only; PR with --drop-late 0 so PC load cannot drop frames) and gyro_klt
conditions; score also summarises the per-frame "klt" counters.]
[integ-20261025 copy of ovopt_20261025/ate_run.py: WT = integ worktree, build_integ_full, OUT /tmp/integ/ate,
condition PR = --profile rec20261025 (extra bridge args via "args"); everything else unchanged.]
Offline accuracy check for ov_bridge --ovopt near-exact options and Phase 4 configs (2026-10-25, ovopt-20261025).

Same protocol as imu_learn/fps30_run.py condition C4 (the live config): 42 HS cells = 14 recordings x start offsets
0/1/2 (RAW + OLD of phone60_run, 213955 = phone drop excluded from the summary), 30 fps, 960x540, clones 15, pts 200,
init window 1.0 s / disparity 22.5 px, cfg_over "num_opencv_threads=1,try_zupt=false,fi_max_cond_number=1e6,
fi_max_baseline=200", lever, frame cache, strict init; scored with lens_switch_gtscore (camera-position ruler, GT
imu_gt_spl_cal1022): divergence flag, window ATE (ate_w), Sim3 ATE, scale. The bench and scorer modules are imported
read-only from the main checkout (~/scanner/StereoDepthSeg/scan3d); recordings / frame caches are only read there.
The bridge is a frozen copy of this branch's PC build (-DOV_FULL_SOURCE=ON) in OUT/bin, called through one wrapper
script per condition that appends the condition's --ovopt list. Single-threaded (num_opencv_threads=1) = deterministic,
so 1 run per cell (the 3 start offsets are the repeats, as in the earlier studies).

  python3 ate_run.py freeze                      # copy the current build to OUT/bin (sha256 in OUT/bin/bridge.json)
  python3 ate_run.py run B0,G,Q [--par 8]        # all 42+3 cells of these conditions (skips finished cells)
  python3 ate_run.py score B0,G,Q --ref B0       # -> OUT/score_<conds>.json + table on stdout
OUT = /tmp/ovopt/ate (env OVOPT_ATE_OUT).
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
WT = Path("/home/system/scanner/StereoDepthSeg-ovgyro/scan3d")
OUT = Path(os.environ.get("OVOPT_ATE_OUT", "/tmp/ovgyro/ate"))
PY = str(MAIN.parent / "venv_ffs" / "bin" / "python")
sys.path.insert(0, str(MAIN / "imu_learn"))
sys.path.insert(0, str(MAIN))
os.chdir(MAIN)  # the bench resolves recordings/, config/ relative to the main checkout

GT = "imu_gt_spl_cal1022"
OFFS = (0, 1, 2)
LEVER = "config/imu_cam_lever.json"
BASE_CFG = "num_opencv_threads=1,try_zupt=false,fi_max_cond_number=1e6,fi_max_baseline=200"  # = fps30 C4
# name: dict(ovopt=..., cfg=extra cfg_over, npts=, clones=)
CONDS = {
    # ovgyro 2026-10-26
    "B0K": dict(args="--klt-stats 1"),
    "PRK": dict(args="--profile rec20261025 --drop-late 0 --klt-stats 1"),
    "PG1": dict(args="--profile rec20261025 --drop-late 0 --klt-stats 1 --ovopt all_exact,ekf_gemm,qr_compress,ekf_chol,gyro_klt=1"),
    "PG3": dict(args="--profile rec20261025 --drop-late 0 --klt-stats 1 --ovopt all_exact,ekf_gemm,qr_compress,ekf_chol,gyro_klt=3"),
    "PG4": dict(args="--profile rec20261025 --drop-late 0 --klt-stats 1 --ovopt all_exact,ekf_gemm,qr_compress,ekf_chol,gyro_klt=4"),
    "BG1": dict(args="--klt-stats 1 --ovopt gyro_klt=1"),
    "BG3": dict(args="--klt-stats 1 --ovopt gyro_klt=3"),
    "B0": dict(),
    "PR": dict(args="--profile rec20261025"),                  # integ: recommended live profile                                              # C4, this branch's bridge, no options
    "X": dict(ovopt="all_exact"),                              # exact options (must equal B0 bit for bit)
    "G": dict(ovopt="ekf_gemm"),                               # near-exact: one GEMM for M = P H^T
    "Q": dict(ovopt="qr_compress"),                            # near-exact: Householder measurement compression
    "GQ": dict(ovopt="ekf_gemm,qr_compress"),
    "K": dict(ovopt="ekf_chol"),                               # near-exact: no S^-1 / K (W = M U^-1)
    "P": dict(ovopt="par_cov"),                                # near-exact: covariance update in 2 parts
    "W": dict(ovopt="all_exact,ekf_gemm,qr_compress,ekf_chol,par_cov"),  # every option together
    "W2": dict(ovopt="all_exact,ekf_gemm,ekf_chol,par_cov"),   # without qr_compress (givens_rm exact instead)
    "W3": dict(ovopt="all_exact,ekf_gemm,qr_compress,ekf_chol"),  # without par_cov (slower on the phone, 2026-10-25 L2)
    # Phase 4 configs (accuracy only; exact options do not change results)
    "S70": dict(cfg="max_slam=70"),
    "S100": dict(cfg="max_slam=100"),
    "S100U50": dict(cfg="max_slam=100,max_slam_in_update=50"),
    "U50": dict(cfg="max_slam_in_update=50"),
    "M60": dict(cfg="max_msckf_in_update=60"),
    "N400": dict(npts=400),
    "N400M80": dict(npts=400, cfg="max_msckf_in_update=80"),
    "C20": dict(clones=20),
    "N400S100C20": dict(npts=400, clones=20, cfg="max_slam=100"),
    "N300S70": dict(npts=300, cfg="max_slam=70"),
}


def cells():
    from phone60_run import RAW, OLD, DROP  # noqa: E402
    return [(r, o) for r in RAW + OLD for o in OFFS], DROP, set(OLD)


def wrapper(cond):
    return OUT / "bin" / f"br_{cond}.sh"


def freeze():
    src = WT / "ov_bridge" / "build_ovgyro_full" / "ov_bridge"
    (OUT / "bin").mkdir(parents=True, exist_ok=True)
    h = hashlib.sha256(open(src, "rb").read()).hexdigest()
    dst = OUT / "bin" / f"ov_bridge_{h[:12]}"
    if not dst.exists():
        shutil.copy2(src, dst)
    git = subprocess.run(["git", "-C", str(WT), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(["git", "-C", str(WT), "status", "--short", "scan3d/ov_bridge"], capture_output=True, text=True).stdout
    json.dump(dict(src=str(src), sha256=h, copy=str(dst), frozen=time.strftime("%F %T"), worktree_head=git,
                   ov_bridge_dirty=dirty.splitlines()), open(OUT / "bin" / "bridge.json", "w"), indent=1)
    for c, d in CONDS.items():
        w = wrapper(c)
        extra = f" --ovopt {d['ovopt']}" if d.get("ovopt") else ""
        extra += f" {d['args']}" if d.get("args") else ""
        w.write_text(f"#!/bin/bash\nexec {dst} \"$@\"{extra}\n")
        w.chmod(0o755)
    print("frozen", dst, h)


def name_of(rec, off, cond):
    return f"{rec[-6:]}_o{off}_{cond}"


def job(name):
    import vio_bench_hs as vb
    rec6, o, cond = name.split("_")
    allc, drop, old = cells()
    rec = next(r for r, _ in allc if r.endswith(rec6))
    off = int(o[1:])
    d = CONDS[cond]
    vb.BRIDGE = wrapper(cond)
    vb.TEMPLATE = MAIN / "config" / "mono" / "estimator_config.yaml"
    # the bench makes its temp config dir under <main checkout>/scan3d/runs/vio_*: keep ours out of the shared checkout
    import tempfile as _tf
    import types
    (OUT / "tmp").mkdir(parents=True, exist_ok=True)
    vb.tempfile = types.SimpleNamespace(mkdtemp=lambda prefix="vio_", dir=None: _tf.mkdtemp(prefix=prefix, dir=str(OUT / "tmp")))
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
    br = json.load(open(OUT / "bin" / "bridge.json"))
    r.update(ovopt_made=time.strftime("%F %T"), ovopt_job_s=round(time.time() - t0, 1), ovopt_runner=str(Path(__file__)),
             ovopt_cond=cond, ovopt_cond_def=d, ovopt_bridge_sha256=br["sha256"], ovopt_worktree_head=br["worktree_head"],
             ovopt_load=list(os.getloadavg()))
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


def klt_sum(P):
    """--klt-stats per-frame counters: [in, LK ok, kept, mean pred shift, fb try, fb ok, rot, |res|, |flow|, ...]"""
    K = np.array([m["klt"][:9] for m in P if "klt" in m and m["klt"][0] > 0], float)
    if not len(K):
        return None
    lost = 1.0 - K[:, 2] / K[:, 0]
    fast = K[:, 8] > np.percentile(K[:, 8], 90)
    return dict(frames=len(K), kept_pct=round(100 * K[:, 2].sum() / K[:, 0].sum(), 3),
                lkok_pct=round(100 * K[:, 1].sum() / K[:, 0].sum(), 3),
                lost_p95_pct=round(100 * float(np.percentile(lost, 95)), 2), lost_max_pct=round(100 * float(lost.max()), 2),
                fast_kept_pct=round(100 * K[fast, 2].sum() / K[fast, 0].sum(), 3),
                res_px=round(float(np.average(K[:, 7], weights=K[:, 1])), 3), flow_px=round(float(np.average(K[:, 8], weights=K[:, 1])), 3),
                tracked_med=float(np.median(K[:, 2])))


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
                h.update(json.dumps({k: v for k, v in m.items() if k not in ("ms", "remap_ms", "wait_ms", "klt")},
                                    sort_keys=True).encode())
            row[c] = dict(div=bool(mc.get("divergent")), ate_w=mc.get("ate_w"), why=mc.get("why"),
                          sim3=be.get("ate_sim3_cm"), scale=be.get("scale"), tracked=st.get("tracked"),
                          n_slam_med=float(np.median([m.get("n_slam", 0) for m in P if m.get("status") == "tracking"] or [0])),
                          klt=klt_sum(P),
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
        kk = [x[c]["klt"] for x in cc if x[c].get("klt")]
        if kk:
            s["klt"] = {f: round(float(np.median([k[f] for k in kk])), 3) for f in kk[0]}

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
    for c in conds:
        print("klt", c, summ[c].get("klt"))
    print("saved", OUT / f"score_{tag}.json")


def check_fps30(cond="B0"):
    """Reproducibility: this branch's bridge (no options) vs the established fps30 C4 run of each cell
    (main checkout runs/fps30/<key>_C4.poses.jsonl.gz, made 2026-10-24 with the bridge of that day)."""
    allc, _, _ = cells()
    same = diff = missing = 0
    for r, o in allc:
        key = f"{r[-6:]}_o{o}"
        a = MAIN / "runs" / "fps30" / f"{key}_C4.poses.jsonl.gz"
        b = OUT / cond / f"{key}_{cond}.poses.jsonl.gz"
        if not a.exists() or not b.exists():
            missing += 1
            continue
        strip = lambda m: {k: v for k, v in m.items() if k not in ("ms", "remap_ms", "wait_ms")}
        A = [strip(json.loads(x)) for x in gzip.open(a, "rt") if x.strip()]
        B = [strip(json.loads(x)) for x in gzip.open(b, "rt") if x.strip()]
        ok = A == B
        same += ok
        diff += not ok
        if not ok:
            first = next((i for i, (x, y) in enumerate(zip(A, B)) if x != y), min(len(A), len(B)))
            print(f"{key}: DIFFERENT (poses {len(A)} vs {len(B)}, first diff {first})")
    print(f"fps30 C4 vs {cond}: identical cells {same}, different {diff}, missing {missing}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["freeze", "run", "job", "score", "check"])
    ap.add_argument("arg", nargs="?", default="")
    ap.add_argument("--par", type=int, default=8)
    ap.add_argument("--ref", default="B0")
    a = ap.parse_args()
    if a.cmd == "freeze":
        freeze()
    elif a.cmd == "run":
        run_sched(a.arg.split(","), a.par)
    elif a.cmd == "job":
        job(a.arg)
    elif a.cmd == "check":
        check_fps30(a.arg or "B0")
    else:
        score(a.arg.split(","), a.ref)
