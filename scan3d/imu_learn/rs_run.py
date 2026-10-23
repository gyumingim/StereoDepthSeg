#!/usr/bin/env python3
"""Rolling-shutter (RS) per-feature gyro correction: sanity checks and the pre-registered comparison
(docs/rs_correction_plan.md, 2026-10-23).

Bridge = tools/ov_bridge_rs (stock ov_bridge/main.cpp copy) linked to ~/scanner/src/open_vins_rs
(= open_vins_zbase + tools/ov_patches/rs.diff). Two new estimator keys, both optional:
  rs_readout_time  s, signed (> 0 = top row of the fed image first). 0 = off (default) -> same code path as the base.
  rs_row_scale     multiplier on (v/H - 0.5): fed rows -> raw sensor rows after the undistort crop. Set here from the lens
                   (fy_raw * H_fed / (fy_fed * H_raw)), geometry only, not tuned.
The estimator config is the bench's (vio_bench_hs.write_config) + these two lines appended (only when rs != 0).

  ../venv_ffs/bin/python imu_learn/rs_run.py bitcheck [REC] [SECS]   # RS bridge rs=0 vs stock bridge: poses identical?
  ../venv_ffs/bin/python imu_learn/rs_run.py sanity [REC] [SECS]     # off / 2.94 / 2.205 / -2.94 ms: [RS] + [CHI2] stats
  ../venv_ffs/bin/python imu_learn/rs_run.py job NAME | run | score  # full comparison (NOT run yet; plan doc first)
"""
import argparse
import gzip
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
os.chdir(SCAN3D)
sys.path.insert(0, str(HERE))

PY = "../venv_ffs/bin/python"
OUT = Path("runs/rs_corr")
PAR = int(os.environ.get("PAR", 2))
STOCK_BRIDGE = SCAN3D / "ov_bridge" / "build" / "ov_bridge"
RS_BRIDGE = SCAN3D / "tools" / "ov_bridge_rs" / "build" / "ov_bridge"
SKEW_REPORTED = 2.94e-3                                       # SENSOR_ROLLING_SHUTTER_SKEW, HS 1920x1080 @240 (CSV 2.9386 ms)
CONDS = {"base": 0.0, "rs294": SKEW_REPORTED, "rs221": SKEW_REPORTED * 0.75}
# base template: config/base_zob (= config/mono + zupt_only_at_beginning: true, user decision 2026-10-23). Until that dir
# exists the same thing is made from config/mono + cfg_over (same file content except that one line).
BASE_TDIR = "config/base_zob"
RAW = ["hs_20261022_133820", "hs_20261022_134550", "hs_20261022_134204",
       "hs_20261021_214550", "hs_20261021_213341", "hs_20261021_213955", "hs_20261021_215348",
       "hs_20261020_214138", "hs_20261020_221052", "hs_20261020_221925", "hs_20261020_222747", "hs_20261020_223815",
       "hs_20261020_224323"]
OLD = ["hs_20261019_131326", "hs_20261019_154320"]          # no raw IMU -> calibrated types 1/4 (same for all conds)
DROP = "hs_20261021_213955"


def template():
    if (SCAN3D / BASE_TDIR / "estimator_config.yaml").exists():
        return BASE_TDIR, ""
    return "config/mono", "zupt_only_at_beginning=true"


def row_scale(rec, size="640x360"):
    import json as _j
    import vio_bench_hs as vb
    meta = _j.load(open(Path(rec) / "meta.json"))
    K, _ = vb.lens_for(Path(rec), meta)
    _, _, Knew, (W, H) = vb.undistort_maps(Path(rec), meta, size)
    return float(K[1, 1] * H / (Knew[1, 1] * int(meta["height"])))


def replay(rec, off, rs, bridge, max_secs=0.0, verbose=False, err_path=None, frame_cache=True):
    """One bench run. Returns (result dict, poses list). err_path: bridge stderr (OpenVINS log) goes there."""
    import vio_bench_hs as vb
    tdir, over = template()
    vb.TEMPLATE = SCAN3D / tdir / "estimator_config.yaml"
    vb.BRIDGE = bridge
    rsc = row_scale(rec)
    orig_wc = vb.write_config

    def wc(d, *args, **kw):
        used = orig_wc(d, *args, **kw)
        if rs != 0.0:
            with open(d / "estimator_config.yaml", "a") as f:
                f.write(f"rs_readout_time: {rs:.9f}\nrs_row_scale: {rsc:.6f}\n")
        return used
    vb.write_config = wc
    orig_popen = vb.subprocess.Popen

    def popen(cmd, *a, **kw):
        if err_path and str(cmd[0]) == str(bridge):
            kw["stderr"] = open(err_path, "w")
        return orig_popen(cmd, *a, **kw)
    vb.subprocess.Popen = popen
    cfg_over = ",".join(x for x in ["num_opencv_threads=2", over, 'verbosity="INFO"' if verbose else ""] if x)
    a = argparse.Namespace(rec=str(rec), fps=60.0, size="640x360", npts=200, clones=11, noise="cfg",
                           noise_scale=1.0, imu_calib=None, dropout="", max_secs=float(max_secs),
                           imu_raw=Path(rec).name not in OLD, gt="auto", out=None, init_cfg="strict",
                           start_off=float(off), decode_threads=2, imu_lead=2.0, init_window=1.0, init_disp=15.0,
                           frame_cache=frame_cache, gyro_rebias=False, cfg_over=cfg_over, still_hold=None)
    saved = {}
    orig_ev = vb.evaluate

    def ev(rec_, a_, poses, *rest):
        saved["poses"] = poses
        return orig_ev(rec_, a_, poses, *rest)
    vb.evaluate = ev
    try:
        r = vb.run(a)
    finally:
        vb.write_config, vb.subprocess.Popen, vb.evaluate = orig_wc, orig_popen, orig_ev
    r.update(rs_readout_time=rs, rs_row_scale=rsc if rs != 0.0 else None, rs_bridge=str(bridge), rs_template=tdir,
             rs_cfg_over=cfg_over)
    return r, saved.get("poses", [])


def parse_log(path):
    """[RS] last summary line and [CHI2] msckf/slam statistics (zbase diag lines) from an INFO-level bridge log."""
    rs, chi = None, {"msckf": [], "slam": []}
    for line in open(path, errors="replace"):
        m = re.search(r"\[RS\] frames (\d+) feats (\d+) mean_shift_px ([\d.]+) max_shift_px ([\d.]+) mean_w_rad_s ([\d.]+)", line)
        if m:
            rs = dict(frames=int(m[1]), feats=int(m[2]), mean_shift_px=float(m[3]), max_shift_px=float(m[4]),
                      mean_w_rad_s=float(m[5]))
        m = re.search(r"\[CHI2\] (msckf|slam) in (\d+) rej (\d+) mean_ratio ([\d.]+)", line)
        if m and int(m[2]) > 0:
            chi[m[1]].append((int(m[2]), int(m[3]), float(m[4])))
    out = dict(rs_stats=rs)
    for k, v in chi.items():
        n_in = sum(x[0] for x in v)
        out[k] = dict(updates=len(v), feats_in=n_in, rej=sum(x[1] for x in v),
                      rej_frac=round(sum(x[1] for x in v) / n_in, 4) if n_in else None,
                      mean_chi2_ratio=round(sum(x[0] * x[2] for x in v) / n_in, 4) if n_in else None)
    return out


def bitcheck(rec="recordings/hs_20261022_133820", secs=30.0):
    OUT.mkdir(parents=True, exist_ok=True)
    rA, pA = replay(rec, 0, 0.0, STOCK_BRIDGE, secs, frame_cache=False)
    rB, pB = replay(rec, 0, 0.0, RS_BRIDGE, secs, frame_cache=False)
    strip = lambda P: [{k: v for k, v in m.items() if k not in ("ms", "wall_ms")} for m in P]
    same = strip(pA) == strip(pB)
    res = dict(made=time.strftime("%F %T"), rec=rec, secs=secs, n_poses=[len(pA), len(pB)], identical=same,
               ate=[rA.get("ate_cm"), rB.get("ate_cm")], keys_ignored=["ms", "wall_ms"])
    if not same:
        diff = next((i for i, (x, y) in enumerate(zip(strip(pA), strip(pB))) if x != y), None)
        res["first_diff"] = diff
        res["keys"] = sorted(pA[0].keys()) if pA else []
    json.dump(res, open(OUT / "bitcheck.json", "w"), indent=1)
    print(json.dumps(res, indent=1))


def sanity(rec="recordings/hs_20261022_133820", secs=30.0):
    OUT.mkdir(parents=True, exist_ok=True)
    res = dict(made=time.strftime("%F %T"), rec=rec, secs=secs, row_scale=row_scale(rec), runs={})
    for name, rs in [("off", 0.0), ("rs294", SKEW_REPORTED), ("rs221", SKEW_REPORTED * 0.75), ("flip294", -SKEW_REPORTED)]:
        err = OUT / f"sanity_{name}.log"
        t0 = time.time()
        r, P = replay(rec, 0, rs, RS_BRIDGE, secs, verbose=True, err_path=err, frame_cache=False)
        st = parse_log(err)
        res["runs"][name] = dict(rs=rs, wall_s=round(time.time() - t0, 1), n_poses=len(P),
                                 n_tracking=sum(1 for m in P if m.get("status") == "tracking"),
                                 ate_cm_short=r.get("ate_cm"), **st)
        print(name, json.dumps(res["runs"][name]), flush=True)
        json.dump(res, open(OUT / "sanity.json", "w"), indent=1)


def jobs():
    return [dict(rec=r, off=o, cond=c, name=f"{r[-6:]}_o{o}_{c}") for r in RAW + OLD for o in (0, 1, 2) for c in CONDS]


def job(name):
    j = next(x for x in jobs() if x["name"] == name)
    t0 = time.time()
    r, P = replay(f"recordings/{j['rec']}", j["off"], CONDS[j["cond"]], RS_BRIDGE)
    r.update(rs_cond=j["cond"], rs_made=time.strftime("%F %T"), rs_job_s=round(time.time() - t0, 1))
    with gzip.open(str(OUT / name) + ".poses.jsonl.gz", "wt") as f:
        for m in P:
            f.write(json.dumps(m) + "\n")
    json.dump(r, open(OUT / f"{name}.json", "w"), indent=1, ensure_ascii=False)


def run_sched():
    dirty = subprocess.run(["git", "status", "--porcelain", "imu_learn/vio_bench_hs.py", "imu_learn/rs_run.py",
                            "config/mono", BASE_TDIR, "tools/ov_bridge_rs", "tools/ov_patches/rs.diff"],
                           capture_output=True, text=True).stdout
    if dirty.strip():
        sys.exit(f"REFUSED: uncommitted {dirty}")
    q = [j for j in jobs() if not (OUT / f"{j['name']}.json").exists()]
    running = {}
    print(f"{time.strftime('%F %T')} start {len(q)} jobs PAR {PAR}", flush=True)
    while q or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is not None:
                del running[n]
                ok = (OUT / f"{n}.json").exists()
                print(f"{time.strftime('%F %T')} done {n} {time.time() - t0:.0f} s {'ok' if ok else 'FAILED'}", flush=True)
        while q and len(running) < PAR:
            j = q.pop(0)
            running[j["name"]] = (subprocess.Popen(["nice", "-n", "5", PY, "imu_learn/rs_run.py", "job", j["name"]],
                                                   stdout=subprocess.DEVNULL,
                                                   stderr=open(OUT / f"{j['name']}.err", "w")), time.time())
        time.sleep(2)


def score():
    import numpy as np
    import zupt_ob_score as zs
    os.chdir(SCAN3D)
    runs = {}
    for j in jobs():
        pre = OUT / j["name"]
        d = json.load(open(str(pre) + ".json")) if Path(str(pre) + ".json").exists() else {}
        m = zs.run_metrics(pre, j["rec"]) if d else dict(divergent=True, why="missing")
        runs[j["name"]] = dict(rec=j["rec"], key=j["name"].rsplit("_", 1)[0], cond=j["cond"], div=bool(m.get("divergent")),
                               ate_w=m.get("ate_w"), rel5_max=m.get("rel5_max"), dt_final_ms=d.get("dt_cam_imu_final_ms"))
    res = dict(made=time.strftime("%F %T"), runs=runs, conds={}, criteria={})
    for c in CONDS:
        R = [r for r in runs.values() if r["cond"] == c]
        res["conds"][c] = dict(n_div_excl_drop=sum(r["div"] for r in R if r["rec"] != DROP),
                               n_div_drop=sum(r["div"] for r in R if r["rec"] == DROP))
    keys = sorted({r["key"] for r in runs.values() if r["rec"] != DROP})
    for c in ("rs294", "rs221"):
        normal = [k for k in keys if not runs[f"{k}_base"]["div"] and not runs[f"{k}_{c}"]["div"]]
        mb = float(np.median([runs[f"{k}_base"]["ate_w"] for k in normal]))
        mc = float(np.median([runs[f"{k}_{c}"]["ate_w"] for k in normal]))
        better = sum(runs[f"{k}_{c}"]["ate_w"] < runs[f"{k}_base"]["ate_w"] for k in normal)
        a = res["conds"][c]["n_div_excl_drop"] <= res["conds"]["base"]["n_div_excl_drop"]
        b = mc <= 1.2 * mb + 0.3
        p = mc <= 0.9 * mb and better > len(normal) / 2
        res["criteria"][c] = dict(a_div=a, b_ate_not_worse=b, c_primary=p, n_normal=len(normal), med_base=round(mb, 2),
                                  med_rs=round(mc, 2), n_cells_better=better, adopt=a and b and p)
    json.dump(res, open(OUT / "score.json", "w"), indent=1, ensure_ascii=False)
    print(json.dumps(dict(conds=res["conds"], criteria=res["criteria"]), indent=1))


def main():
    cmd = sys.argv[1]
    if cmd == "job":
        OUT.mkdir(parents=True, exist_ok=True)
        return job(sys.argv[2])
    if cmd in ("bitcheck", "sanity"):
        rec = sys.argv[2] if len(sys.argv) > 2 else "recordings/hs_20261022_133820"
        secs = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0
        return {"bitcheck": bitcheck, "sanity": sanity}[cmd](rec, secs)
    OUT.mkdir(parents=True, exist_ok=True)
    {"run": run_sched, "score": score}[cmd]()


if __name__ == "__main__":
    main()
