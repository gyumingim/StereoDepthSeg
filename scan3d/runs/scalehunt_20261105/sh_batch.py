#!/usr/bin/env python3
"""scalehunt batch (README 3): conditions x 13 GT recordings, offset 0 (B also 1, 2). Skips finished cells.

  python sh_batch.py run COND[,COND..]|all [-j 14]
Outputs ~/scanner/build_livepose/scalehunt/runs/<COND>/<recshort>.{json,poses.jsonl.gz,...}
"""
import argparse, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
W = HERE.parents[2]
BL = Path.home() / "scanner/build_livepose"
ROOT = BL / "scalehunt"
REC = Path.home() / "scanner/StereoDepthSeg/scan3d/recordings"
PY = str(Path.home() / "scanner/StereoDepthSeg/venv_ffs/bin/python")
LIVE = BL / "cfg_gen/c4f60"
CFG = ROOT / "cfg"
BRIDGE = ["--bridge", str(BL / "bridge/ov_bridge"), "--vocab", str(W / "scan3d/ov_bridge/loop/voc_k10L4.voc")]
RECS = ["hs_20261024_165506_std60",
        "hs_20261022_133820", "hs_20261022_134550", "hs_20261022_134204", "hs_20261021_214550", "hs_20261021_213341",
        "hs_20261021_215348", "hs_20261020_214138", "hs_20261020_221052", "hs_20261020_221925", "hs_20261020_222747",
        "hs_20261020_223815", "hs_20261020_224323"]
OVOPT = "all_exact,ekf_gemm,qr_compress,ekf_chol,sym_inplace,ekf_sparse"
TS0 = 0.002172
C = {"B": (LIVE, []), "B60": (LIVE, []),      "L_d": (CFG / "L_d/c4f60", []),
     "T_m3": (LIVE, ["--chain", f"timeshift_cam_imu={TS0 - 0.003:.6f}"]),
     "T_p3": (LIVE, ["--chain", f"timeshift_cam_imu={TS0 + 0.003:.6f}"]),
     "T_fixp3": (LIVE, ["--yaml", "calib_cam_timeoffset=false", "--chain", f"timeshift_cam_imu={TS0 + 0.003:.6f}"]),
     "E_fix": (LIVE, ["--yaml", "calib_cam_extrinsics=false"]),
     "G_s+1": (LIVE, ["--gyro-scale", "1.01"]), "G_s-1": (LIVE, ["--gyro-scale", "0.99"]),
     "A_s+1": (LIVE, ["--acc-scale", "1.01"]),
     "N_ax2": (LIVE, ["--chain", "accelerometer_noise_density=2.0e-02"]),
     "N_ax.5": (LIVE, ["--chain", "accelerometer_noise_density=5.0e-03"]),
     "N_gx2": (LIVE, ["--chain", "gyroscope_noise_density=2.0e-03"]),
     "N_gx.5": (LIVE, ["--chain", "gyroscope_noise_density=5.0e-04"]),
     "N_awx10": (LIVE, ["--chain", "accelerometer_random_walk=1.0e-02"]),
     "GR_9797": (LIVE, ["--yaml", "gravity_mag=9.7973"]), "GR_970": (LIVE, ["--yaml", "gravity_mag=9.70"]),
     "R_rs": (LIVE, ["--extra", f"--ovopt {OVOPT},gyro_klt=5,rs_derot"]),
     "I_w05": (LIVE, ["--yaml", "init_window_time=0.5"]), "I_w2": (LIVE, ["--yaml", "init_window_time=2.0"]),
     "F_p300": (LIVE, ["--yaml", "num_pts=300"]), "F_p100": (LIVE, ["--yaml", "num_pts=100"]),
     "F_s0": (LIVE, ["--yaml", "max_slam=0"]), "F_s100": (LIVE, ["--yaml", "max_slam=100"]),
     "F_sg05": (LIVE, ["--yaml", "up_msckf_sigma_px=0.5,up_slam_sigma_px=0.5"]),
     "F_sg2": (LIVE, ["--yaml", "up_msckf_sigma_px=2,up_slam_sigma_px=2"]),
     "F_c11": (LIVE, ["--yaml", "max_clones=11"]), "F_c25": (LIVE, ["--yaml", "max_clones=25"]),
     }
for n in ("K_f+2", "K_f-2", "K_fx+2", "K_cx+5", "K_cy+5", "D_off", "E_rx1", "E_ry1", "E_rz1", "E_p0", "E_px2", "E_py2", "E_pz2"):
    C[n] = (CFG / n, [])
ORDER = ["B", "B60", "K_f+2", "K_f-2", "A_s+1", "G_s+1", "E_p0", "T_p3", "T_fixp3", "E_rx1", "E_ry1", "E_rz1", "E_fix", "K_fx+2",
         "K_cx+5", "K_cy+5", "D_off", "L_d", "E_px2", "E_py2", "E_pz2", "T_m3", "G_s-1", "N_ax2", "N_ax.5", "N_gx2", "N_gx.5",
         "N_awx10", "GR_9797", "GR_970", "R_rs", "I_w05", "I_w2", "F_p300", "F_p100", "F_s0", "F_s100", "F_sg05", "F_sg2",
         "F_c11", "F_c25"]
assert set(ORDER) == set(C), set(C) ^ set(ORDER)


def short(r):
    return r.split("_")[2] + ("s" if r.endswith("std60") else "")


def job(cond, rec):
    cfg, extra = C[cond]
    if cond != "B":                      # README 3.1: sweep clips capped at 60 s
        extra = extra + ["--max-secs", "60", "--cache", str(ROOT / "cache")]   # README 3.2
    od = ROOT / "runs" / cond; od.mkdir(parents=True, exist_ok=True)
    out = od / short(rec)
    if Path(str(out) + ".json").exists():
        return
    t0 = time.time()
    with open(str(out) + ".log", "w") as lg:
        rc = subprocess.call(["nice", "-n", "5", PY, str(HERE / "sh_replay.py"), str(REC / rec), str(cfg), str(out)] + BRIDGE + extra,
                             stdout=lg, stderr=subprocess.STDOUT, env={**__import__("os").environ, "OPENCV_FOR_THREADS_NUM": "1"})
    last = (open(str(out) + ".log").read().strip().splitlines() or [""])[-1]
    print(time.strftime("%T"), cond, short(rec), f"rc {rc}", f"{time.time() - t0:.0f}s", last[:200], flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd"); ap.add_argument("conds", nargs="?", default="B"); ap.add_argument("-j", type=int, default=14)
    a = ap.parse_args()
    conds = ORDER if a.conds == "all" else a.conds.split(",")
    # by recording (all conditions of one recording run together -> its frame cache stays in the page cache; 47 GB of caches
    # > 30 GB RAM thrashed at 200 MB/s when 14 recordings streamed at once, 16:15)
    jobs = [(c, r) for r in RECS for c in conds]
    print(time.strftime("%F %T"), "jobs", len(jobs), flush=True)
    with ThreadPoolExecutor(a.j) as ex:
        list(ex.map(lambda cr: job(*cr), jobs))
    print(time.strftime("%F %T"), "done", flush=True)


if __name__ == "__main__":
    main()
