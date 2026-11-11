#!/usr/bin/env python3
"""vioscale batch (README 1): one-factor VIO variants on ONE recording, then the rulers.

  python vs_batch.py REC OUT_DIR [--conds a,b,..] [-j 4] [--rulers red,imu | board,imu]
Runs on the laptop (cwd = worktree). Per condition: OUT_DIR/runs/<cond>.{poses.jsonl.gz,json,log,...};
scores -> OUT_DIR/scores.jsonl (one line per cond and ruler), provenance -> OUT_DIR/prov.json.
"""
import argparse, json, os, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
WT = HERE.parents[2]
PY = "/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python"
BL = Path("/home/system/scanner/build_livepose")
LIVE = BL / "cfg_gen/c4f60"
BRIDGE = ["--bridge", str(BL / "bridge/ov_bridge"), "--vocab", str(WT / "scan3d/ov_bridge/loop/voc_k10L4.voc")]
OVOPT = "all_exact,ekf_gemm,qr_compress,ekf_chol,sym_inplace,ekf_sparse"
TS0 = 0.002172
G_LOCAL = 9.7973


def conds(prov):
    a_still = prov["still"].get("35", {}).get("norm_of_mean")
    onset = prov.get("onset_s_from_frame0") or 5.0
    C = {"B": []}
    for o in (0.05, 0.1, 0.2, 0.5, 1.0, 2.0):                       # null floor: start still inside the still phase
        if o < onset - 1.5:
            C[f"S_{o:g}"] = ["--start-off", f"{o}"]
    C.update({
        "I_w05": ["--yaml", "init_window_time=0.5"], "I_w15": ["--yaml", "init_window_time=1.5"], "I_w2": ["--yaml", "init_window_time=2.0"],
        "I_th015": ["--yaml", "init_imu_thresh=0.15"], "I_th06": ["--yaml", "init_imu_thresh=0.6"],
        "I_audit": ["--yaml", "init_window_time=1.5,init_max_disparity=2.30"],
        "I_dyn": ["--yaml", "init_dyn_use=true"],
        "I_dyn_after": ["--yaml", "init_dyn_use=true", "--start-off", f"{onset + 0.5:.2f}"],
        "I_stat_after": ["--start-off", f"{onset + 0.5:.2f}"],
        "N_ax2": ["--chain", "accelerometer_noise_density=2.0e-02"], "N_ax.5": ["--chain", "accelerometer_noise_density=5.0e-03"],
        "N_gx2": ["--chain", "gyroscope_noise_density=2.0e-03"], "N_gx.5": ["--chain", "gyroscope_noise_density=5.0e-04"],
        "N_awx10": ["--chain", "accelerometer_random_walk=1.0e-02"], "N_awx.1": ["--chain", "accelerometer_random_walk=1.0e-04"],
        "N_gwx10": ["--chain", "gyroscope_random_walk=1.0e-03"],
        "N_allx2": ["--noise-scale", "2"], "N_allx.5": ["--noise-scale", "0.5"],
        "C_imuintr": ["--yaml", "calib_imu_intrinsics=true"], "C_gsens": ["--yaml", "calib_imu_g_sensitivity=true"],
        "C_imuintr_gs": ["--yaml", "calib_imu_intrinsics=true,calib_imu_g_sensitivity=true"],
        "GR_9797": ["--yaml", "gravity_mag=9.7973"],
        "A_s+1": ["--acc-scale", "1.01"], "A_s-1": ["--acc-scale", "0.99"], "A_s-5": ["--acc-scale", "0.95"],
        "G_s+1": ["--gyro-scale", "1.01"], "G_s-1": ["--gyro-scale", "0.99"],
        "T_fix": ["--yaml", "calib_cam_timeoffset=false"], "T_0": ["--chain", "timeshift_cam_imu=0.0"],
        "T_m3": ["--chain", f"timeshift_cam_imu={TS0 - 0.003:.6f}"], "T_p3": ["--chain", f"timeshift_cam_imu={TS0 + 0.003:.6f}"],
        "T_m10": ["--chain", f"timeshift_cam_imu={TS0 - 0.010:.6f}"], "T_p10": ["--chain", f"timeshift_cam_imu={TS0 + 0.010:.6f}"],
        "E_fix": ["--yaml", "calib_cam_extrinsics=false"],
        "F_p100": ["--yaml", "num_pts=100"], "F_p300": ["--yaml", "num_pts=300"],
        "F_fast10": ["--yaml", "fast_threshold=10"], "F_fast25": ["--yaml", "fast_threshold=25"],
        "F_s100": ["--yaml", "max_slam=100"], "F_sg05": ["--yaml", "up_msckf_sigma_px=0.5,up_slam_sigma_px=0.5"],
        "F_sg2": ["--yaml", "up_msckf_sigma_px=2,up_slam_sigma_px=2"], "F_c11": ["--yaml", "max_clones=11"], "F_c25": ["--yaml", "max_clones=25"],
        "F_fidef": ["--yaml", "fi_max_baseline=40.0,fi_max_cond_number=10000.0"],
        "Z_on": ["--yaml", "try_zupt=true"],
        "L_off": ["--loop", "0"],
        "Q_gcal": ["--gyro-cal"], "Q_d2": ["--imu-decim", "2"],
        "R_rs": ["--extra", f"--ovopt {OVOPT},gyro_klt=5,rs_derot"],
    })
    if a_still:
        C["A_meas"] = ["--acc-scale", f"{G_LOCAL / a_still:.6f}"]         # accel rescaled so the still |a| = local g
        C["GR_meas"] = ["--yaml", f"gravity_mag={a_still:.5f}"]          # gravity magnitude = what this accelerometer reads still
    return C


def run_one(rec, out, cond, args):
    f = out / "runs" / cond
    if Path(str(f) + ".json").exists():
        return
    t0 = time.time()
    with open(str(f) + ".log", "w") as lg:
        rc = subprocess.call(["nice", "-n", "5", PY, str(HERE / "vs_replay.py"), str(rec), str(LIVE), str(f)] + BRIDGE + args,
                             stdout=lg, stderr=subprocess.STDOUT, env={**os.environ, "OPENCV_FOR_THREADS_NUM": "1"},
                             cwd=str(WT / "scan3d/runs/livepose_20261030"))
    last = (open(str(f) + ".log").read().strip().splitlines() or [""])[-1]
    print(time.strftime("%T"), cond, f"rc {rc}", f"{time.time() - t0:.0f}s", last[:220], flush=True)


def score(rec, out, cond, rulers, views, kfmap):
    f = out / "runs" / f"{cond}.poses.jsonl.gz"
    if not f.exists():
        return
    res = {}
    for r in rulers:
        if r == "red":
            cmd = ["red", views, kfmap, str(f)]
        elif r == "imu":
            cmd = ["imu", str(rec), str(f), str(LIVE / "kalibr_imucam_chain.yaml"), str(out / "imuwork" / cond)]
        else:
            cmd = ["board", str(rec), str(f)]
        p = subprocess.run([PY, str(HERE / "vs_score.py")] + cmd, capture_output=True, text=True)
        line = [l for l in p.stdout.splitlines() if l.startswith("RESULT ")]
        res[r] = json.loads(line[-1][7:]) if line else dict(error=(p.stderr or "")[-400:])
    j = json.load(open(out / "runs" / f"{cond}.json"))
    sc = j.get("score", {})
    rowd = dict(cond=cond, made=time.strftime("%F %T"), args=j["args"], **{k: sc.get(k) for k in
                ("status", "init_s", "path_m", "loop_err_cm", "dt_final_ms", "tracked_frac", "n_loop")}, rulers=res)
    with open(out / "scores.jsonl", "a") as fo:
        fo.write(json.dumps(rowd) + "\n")
    msg = " ".join(f"{r}:{v.get('s', v.get('k_rev', v.get('dtvio', {}).get('s_fixedlever') if isinstance(v.get('dtvio'), dict) else v.get('error', '')[:60]))}"
                   for r, v in res.items())
    print(time.strftime("%T"), "score", cond, rowd["status"], rowd["init_s"], msg, flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec"); ap.add_argument("out")
    ap.add_argument("--conds", default="all"); ap.add_argument("-j", type=int, default=4)
    ap.add_argument("--rulers", default="red,imu")
    ap.add_argument("--views", default=""); ap.add_argument("--kfmap", default="")
    a = ap.parse_args()
    rec, out = Path(a.rec), Path(a.out)
    (out / "runs").mkdir(parents=True, exist_ok=True)
    if not (out / "prov.json").exists():
        subprocess.run([PY, str(HERE / "vs_prov.py"), str(rec), str(out / "prov.json")], check=True, stdout=subprocess.DEVNULL)
    prov = json.load(open(out / "prov.json"))
    C = conds(prov)
    names = list(C) if a.conds == "all" else a.conds.split(",")
    json.dump(dict(made=time.strftime("%F %T"), rec=str(rec), conds={n: C[n] for n in names}), open(out / "conds.json", "w"), indent=1)
    print(time.strftime("%F %T"), "conds", len(names), "onset", prov.get("onset_s_from_frame0"), "a_still", prov["still"].get("35", {}).get("norm_of_mean"), flush=True)
    rulers = a.rulers.split(",")
    with ThreadPoolExecutor(a.j) as ex:
        list(ex.map(lambda n: run_one(rec, out, n, C[n]), names))
    print(time.strftime("%F %T"), "replays done", flush=True)
    with ThreadPoolExecutor(a.j) as ex:
        list(ex.map(lambda n: score(rec, out, n, rulers, a.views, a.kfmap), names))
    print(time.strftime("%F %T"), "all done", flush=True)


if __name__ == "__main__":
    main()
