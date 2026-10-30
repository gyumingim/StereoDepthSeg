#!/usr/bin/env python3
"""livepose batch: conditions x recordings (README.md section 2, pre-registered). Skips finished cells (OUT.json exists).

  python lp_batch.py run COND[,COND..] [--sets S,L,H] [-j 6]
  python lp_batch.py list
Outputs ~/scanner/tmp_livepose/runs/<COND>/<recshort>.{json,poses.jsonl.gz,loop.jsonl,bridge.err,log}
"""
import argparse, json, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
TMP = Path.home() / "scanner/tmp_livepose"
REC = Path.home() / "scanner/StereoDepthSeg/scan3d/recordings"
PY = str(Path.home() / "scanner/StereoDepthSeg/venv_ffs/bin/python")
LIVE = TMP / "cfg/live_c4f60"
CFG_D = TMP / "cfg/gen_cal1024d/c4f60"
CFG_HS = TMP / "cfg/gen_hs222/c4f60"

SETS = {
    # live-equivalent: std60 camera session, 60 fps native, focus 0.5 dpt
    "S": ["hs_20261024_165506_std60", "hs_20261024_173432_std60"],                       # board GT
    "L": ["hs_20261024_142148_std60", "hs_20261024_142610_std60", "hs_20261024_151646_std60"],   # hand loops, no GT
    # not equivalent: HS 240 fps session, focus 2.22 dpt, fed at 60 fps (nearest frame); ov60eval/phone60 RAW set minus 213955
    "H": ["hs_20261022_133820", "hs_20261022_134550", "hs_20261022_134204", "hs_20261021_214550", "hs_20261021_213341",
          "hs_20261021_215348", "hs_20261020_214138", "hs_20261020_221052", "hs_20261020_221925", "hs_20261020_222747",
          "hs_20261020_223815", "hs_20261020_224323"],
}
OVOPT = "all_exact,ekf_gemm,qr_compress,ekf_chol,sym_inplace,ekf_sparse"
CONDS = {   # name: (cfg dir, extra lp_replay args, sets)
    "B": (LIVE, [], "SLH"),
    "B_o1": (LIVE, ["--start-off", "1"], "SLH"),
    "B_o2": (LIVE, ["--start-off", "2"], "SLH"),
    "L_d": (CFG_D, [], "SLH"),                       # other 0.5 dpt board calibration (cal20261024d)
    "L_hs": (CFG_HS, [], "H"),                       # HS: the recording's own 2.22 dpt board lens
    "I_cal": (LIVE, ["--imu", "cal"], "SLH"),        # Android-calibrated IMU (types 1/4)
    "N_half": (LIVE, ["--noise-scale", "0.5"], "SLH"),
    "N_x2": (LIVE, ["--noise-scale", "2"], "SLH"),
    "T_or": (LIVE, ["--time", "oracle"], "SLH"),     # GT align dt (oracle), yaml timeshift 0
    "T_fix": (LIVE, ["--yaml", "calib_cam_timeoffset=false"], "SLH"),
    "T_p5": (LIVE, ["--chain", "timeshift_cam_imu=0.007172"], "SLH"),   # +5 ms initial offset error
    "R_rs": (LIVE, ["--extra", f"--ovopt {OVOPT},gyro_klt=5,rs_derot"], "SLH"),
    "K_off": (LIVE, ["--extra", f"--ovopt {OVOPT}"], "SLH"),          # gyro_klt off
    "D10": (LIVE, ["--drop-every", "10"], "SLH"),    # 10 % frames dropped (timing stress)
    "Lp0": (LIVE, ["--loop", "0"], "S"),             # loop off: VIO pose must be identical to B
    # README 2.6 exploratory (registered 18:20 after part of B was seen; not used for verdicts)
    "X_win05": (LIVE, ["--yaml", "init_window_time=0.5"], "SL"),
    "X_pts300": (LIVE, ["--yaml", "num_pts=300"], "SL"),
    "X_fast10": (LIVE, ["--yaml", "fast_threshold=10"], "SL"),
}


def short(r):
    return r.split("_")[2] + ("s" if r.endswith("std60") else "")


def job(cond, rec):
    cfg, extra, _ = CONDS[cond]
    od = TMP / "runs" / cond; od.mkdir(parents=True, exist_ok=True)
    out = od / short(rec)
    if (Path(str(out) + ".json")).exists():
        return cond, rec, "skip"
    t0 = time.time()
    with open(str(out) + ".log", "w") as lg:
        rc = subprocess.call(["nice", "-n", "5", PY, str(HERE / "lp_replay.py"), str(REC / rec), str(cfg), str(out)] + extra,
                             stdout=lg, stderr=subprocess.STDOUT)
    last = open(str(out) + ".log").read().strip().splitlines()[-1:] or [""]
    print(time.strftime("%T"), cond, rec, f"rc {rc}", f"{time.time() - t0:.0f}s", last[0][:300], flush=True)
    return cond, rec, rc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd"); ap.add_argument("conds", nargs="?", default="B")
    ap.add_argument("--sets", default=""); ap.add_argument("-j", type=int, default=6)
    a = ap.parse_args()
    if a.cmd == "list":
        for c, v in CONDS.items():
            print(c, v[2], v[1])
        return
    jobs = []
    for c in a.conds.split(","):
        sets = a.sets or CONDS[c][2]
        # longest first (HS) so the pool stays full
        for s in "HSL":
            if s in sets and s in CONDS[c][2]:
                jobs += [(c, r) for r in SETS[s]]
    print(time.strftime("%F %T"), "jobs", len(jobs), flush=True)
    with ThreadPoolExecutor(a.j) as ex:
        list(ex.map(lambda cr: job(*cr), jobs))
    print(time.strftime("%F %T"), "done", flush=True)


if __name__ == "__main__":
    main()
