#!/usr/bin/env python3
"""Init relax test, 2026-10-24 (plan docs/init_relax_plan.md, result docs/init_relax_result.md).

base = final baseline + F1 with the baseline init (window 1.0 s / disparity 15 px) = runs/fi_limits/<key>_F1 (reused;
       one cell re-run here as <key>_base__chk and compared pose-for-pose)
aud  = same, but init window 1.5 s / disparity 2.30 px (bench strict default = make_ov_config.init_params at f ~440)
Sets: the 15 HS recordings x o0/o1/o2 (new-K GT, camera-position ruler, 213955 excluded), the std60 loop recordings
(--gt none, loop error, o0) and the std60 board recordings (reused from runs/std60_board_20261024/<HHMMSS>/o*_base|aud).
Also: init time per cell and the motion at init (GT camera speed, gyro) -> 'init during motion' flags.

  (setsid nohup ../venv_ffs/bin/python imu_learn/init_relax_run.py run > runs/init_relax/sched.out 2>&1 &)   # PAR 8
  ../venv_ffs/bin/python imu_learn/init_relax_run.py job 133820_o0_aud
  ../venv_ffs/bin/python imu_learn/init_relax_run.py score     # -> runs/init_relax/score.json
"""
import argparse
import csv
import gzip
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
os.chdir(SCAN3D)
sys.path.insert(0, str(HERE))
from phone60_run import RAW, OLD, DROP  # noqa: E402
import fi_limits_run as fl  # noqa: E402

PY = "../venv_ffs/bin/python"
OUT = Path("runs/init_relax")
FIL = Path("runs/fi_limits")
BOARD = Path("runs/std60_board_20261024")
PAR = int(os.environ.get("PAR", 8))
THREADS = "1"                                  # same as runs/fi_limits (F1 cells were run with 1 OpenCV thread)
LEVER = "config/imu_cam_lever.json"
OFFS = (0, 1, 2)
STD = fl.STD
BOARD_RECS = ["hs_20261024_165506_std60", "hs_20261024_173432_std60"]
INIT = {"base": (1.0, 15.0), "aud": (1.5, 2.30)}
F1 = "fi_max_cond_number=1e6,fi_max_baseline=200"
GT = "imu_gt_spl_cal1022"
CHK = "133820_o0_base__chk"
V_MOVE, G_MOVE = 0.02, np.radians(3.0)        # flag: GT speed > 2 cm/s or |gyro| > 3 deg/s at init


def jobs():
    js = [dict(rec="hs_20261022_133820", off=0, cond="base", gt="auto", name=CHK)]
    js += [dict(rec=r, off=o, cond="aud", gt="auto", name=f"{r[-6:]}_o{o}_aud") for r in RAW + OLD for o in OFFS]
    js += [dict(rec=r, off=0, cond="aud", gt="none", name=f"std{r[-12:-6]}_aud") for r in STD]
    return js


def job(name):
    import vio_bench_hs as vb
    j = next(x for x in jobs() if x["name"] == name)
    iw, idp = INIT[j["cond"]]
    vb.TEMPLATE = SCAN3D / "config" / "mono" / "estimator_config.yaml"
    a = argparse.Namespace(rec=f"recordings/{j['rec']}", fps=60.0, size="640x360", npts=200, clones=11, noise="cfg",
                           noise_scale=1.0, imu_calib=None, lever=LEVER, lens_file=None, dropout="", max_secs=0.0,
                           imu_raw=j["rec"] not in OLD, gt=j["gt"], out=None, init_cfg="strict", start_off=float(j["off"]),
                           decode_threads=2, imu_lead=2.0, init_window=iw, init_disp=idp, frame_cache=True,
                           gyro_rebias=False, cfg_over=f"num_opencv_threads={THREADS},try_zupt=false,{F1}", still_hold=None)
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
    r.update(ir_made=time.strftime("%F %T"), ir_job_s=round(time.time() - t0, 1), ir_runner="imu_learn/init_relax_run.py",
             ir_cond=j["cond"], ir_init=[iw, idp], ir_threads=THREADS)
    with gzip.open(str(OUT / name) + ".poses.jsonl.gz", "wt") as f:
        for m in saved.get("poses", []):
            f.write(json.dumps(m) + "\n")
    json.dump(r, open(OUT / f"{name}.json", "w"), indent=1, ensure_ascii=False)


def run_sched():
    q = [j for j in jobs() if not (OUT / f"{j['name']}.json").exists()]
    q.sort(key=lambda j: (j["name"] != CHK, j["rec"] != "hs_20261020_224323"))
    running, t_all = {}, time.time()
    print(f"{time.strftime('%F %T')} start {len(q)} jobs PAR {PAR} threads {THREADS}", flush=True)
    while q or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is not None:
                del running[n]
                ok = (OUT / f"{n}.json").exists()
                print(f"{time.strftime('%F %T')} done {n} {time.time() - t0:.0f} s {'ok' if ok else 'FAILED'}", flush=True)
        while q and len(running) < PAR:
            j = q.pop(0)
            running[j["name"]] = (subprocess.Popen(["nice", "-n", "5", PY, "imu_learn/init_relax_run.py", "job", j["name"]],
                                                   stdout=subprocess.DEVNULL,
                                                   stderr=open(OUT / f"{j['name']}.err", "w")), time.time())
        time.sleep(2)
    print(f"{time.strftime('%F %T')} end wall {time.time() - t_all:.0f} s", flush=True)


# ---------------------------------------------------------------- motion at init
_cache = {}


def gz_poses(pre):
    with gzip.open(str(pre) + ".poses.jsonl.gz", "rt") as f:
        return [json.loads(x) for x in f]


def rec_motion(rec, gtdir):
    """First frame time (camera clock), dt_ns (IMU = cam - dt), gyro |w| (type 4, calibrated) and GT camera centre."""
    key = (rec, gtdir)
    if key in _cache:
        return _cache[key]
    R = Path("recordings") / rec
    t_first = int(next(csv.DictReader(open(R / "frames_cam5.csv")))["t_mid_ns"])
    al = R / "imu_gt" / "align.json"
    dt_ns = int(json.load(open(al))["dt_ns"]) if al.exists() else 0
    g = [x for x in csv.DictReader(open(R / "imu.csv")) if x["type"] == "4"]
    tg = np.array([int(x["t_ns"]) for x in g], float)
    wg = np.linalg.norm(np.array([[float(x["v0"]), float(x["v1"]), float(x["v2"])] for x in g]), axis=1)
    gt = None
    if gtdir and (R / gtdir / "gt.npz").exists():
        d = np.load(R / gtdir / "gt.npz")
        ok = d["ok"].astype(bool) & (np.linalg.norm(d["p_bc"], axis=1) < 1.2)
        gt = (d["t_ns"][ok].astype(float), d["p_bc"][ok].reshape(-1, 3))
    _cache[key] = dict(t_first=t_first, dt_ns=dt_ns, tg=tg, wg=wg, gt=gt)
    return _cache[key]


def speed_at(m, t_imu, half=0.05):
    """GT camera-centre speed (m/s) over t_imu +- half s = |slope| of a straight-line fit of position vs time (robust to
    per-frame GT noise). GT is on the camera clock: t_cam = t_imu + dt."""
    if m["gt"] is None:
        return None
    tc = t_imu + m["dt_ns"]
    t, p = m["gt"]
    s = (t >= tc - half * 1e9) & (t <= tc + half * 1e9)
    if s.sum() < 4:
        return None
    ts = (t[s] - tc) / 1e9
    return float(np.linalg.norm(np.polyfit(ts, p[s], 1)[0]))


def gyro_mean(m, t_imu, a=-0.05, b=0.05):
    s = (m["tg"] >= t_imu + a * 1e9) & (m["tg"] <= t_imu + b * 1e9)
    return float(np.mean(m["wg"][s])) if s.any() else None


def init_info(pre, rec, window, gtdir):
    """Init time (s since the recording's first frame) and motion at init and over the assumed-still part of the window
    (OpenVINS static init: older half of the window still, newer half 'jerk'; that split is from memory = 추정)."""
    P = [m for m in gz_poses(pre) if m.get("status") == "tracking"]
    if not P:
        return dict(t_init_rec_s=None, moving_at_init=None)
    m = rec_motion(rec, gtdir)
    ti = float(P[0]["t_ns"])
    v0, g0 = speed_at(m, ti), gyro_mean(m, ti)
    vs = [speed_at(m, ti - (window - k * window / 10) * 1e9, half=window / 20) for k in range(6)]
    vs = [v for v in vs if v is not None]
    s = (m["tg"] >= ti - window * 1e9) & (m["tg"] <= ti - window / 2 * 1e9)
    gs = float(np.mean(m["wg"][s])) if s.any() else None
    gmax = (float(np.max(np.convolve(m["wg"][s], np.ones(20) / 20, "valid"))) if s.sum() >= 20 else
            (float(m["wg"][s].max()) if s.any() else None))
    flag = bool((v0 is not None and v0 > V_MOVE) or (g0 is not None and g0 > G_MOVE))
    return dict(t_init_rec_s=round((ti + m["dt_ns"] - m["t_first"]) / 1e9, 3),
                v_init_cms=None if v0 is None else round(v0 * 100, 2),
                gyro_init_dps=None if g0 is None else round(float(np.degrees(g0)), 2), moving_at_init=flag,
                v_stillhalf_max_cms=round(max(vs) * 100, 2) if vs else None,
                gyro_stillhalf_mean_dps=None if gs is None else round(float(np.degrees(gs)), 2),
                gyro_stillhalf_max_dps=None if gmax is None else round(float(np.degrees(gmax)), 2))


def check_identity():
    a, b = gz_poses(FIL / "133820_o0_F1"), gz_poses(OUT / CHK)
    ka = [(m["t_ns"], m.get("status"), m.get("T_world_cam0")) for m in a]
    kb = [(m["t_ns"], m.get("status"), m.get("T_world_cam0")) for m in b]
    return dict(n_a=len(ka), n_b=len(kb), identical=ka == kb,
                n_diff=sum(x != y for x, y in zip(ka, kb)) + abs(len(ka) - len(kb)))


def paired(cells):
    both = [x for x in cells if not x["base"]["div"] and not x["aud"]["div"]]
    d = [x["aud"]["ate_w"] - x["base"]["ate_w"] for x in both]
    return dict(n_both_normal=len(both), d_med=round(float(np.median(d)), 4) if d else None,
                d_mean=round(float(np.mean(d)), 4) if d else None, n_aud_better=int(sum(v < 0 for v in d)),
                n_equal=int(sum(v == 0 for v in d)),
                med_base=round(float(np.median([x["base"]["ate_w"] for x in both])), 4) if both else None,
                med_aud=round(float(np.median([x["aud"]["ate_w"] for x in both])), 4) if both else None)


def score():
    import lens_switch_gtscore as gs
    os.chdir(SCAN3D)
    gs.OUT = OUT
    src = {"base": lambda k: FIL / f"{k}_F1", "aud": lambda k: OUT / f"{k}_aud"}
    rows = {}
    for r in RAW + OLD:
        for o in OFFS:
            key = f"{r[-6:]}_o{o}"
            row = dict(rec=r, key=key, off=o)
            for c in INIT:
                pre = src[c](key)
                if not Path(str(pre) + ".json").exists():
                    row[c] = None
                    continue
                mc = gs.cam_metrics(pre, r, GT)
                be = gs.bench_eval(pre, r, GT)
                st = json.load(open(str(pre) + ".json"))
                cu = st.get("config_used", {})
                row[c] = dict(div=bool(mc.get("divergent")), ate_w=mc.get("ate_w"), rel5_max=mc.get("rel5_max"), **be,
                              tracked_frac=st.get("tracked_frac"), t_init_clip_s=st.get("t_init_clip_s"),
                              init_used=[cu.get("init_window_time"), cu.get("init_max_disparity")],
                              fi_used=[cu.get("fi_max_cond_number"), cu.get("fi_max_baseline")],
                              **init_info(pre, r, INIT[c][0], GT))
            rows[key] = row
            print(key, {c: (('*' if v['div'] else '') + str(v['ate_w']) + f" t{v['t_init_rec_s']}" if v else None)
                        for c, v in row.items() if c in INIT}, flush=True)
    use = [x for x in rows.values() if x["rec"] != DROP and all(x[c] for c in INIT)]

    def summ(c, cells):
        ok = [x for x in cells if not x[c]["div"]]
        v = lambda f: round(float(np.median([f(x[c]) for x in ok])), 4) if ok else None  # noqa: E731
        ti = [x[c]["t_init_rec_s"] for x in cells if x[c]["t_init_rec_s"] is not None]
        return dict(div=sum(x[c]["div"] for x in cells), div_cells=[x["key"] for x in cells if x[c]["div"]], n=len(cells),
                    n_normal=len(ok), ate_w_med=v(lambda y: y["ate_w"]), sim3_med=v(lambda y: y["ate_sim3_cm"]),
                    scale_med=v(lambda y: y["scale"]), t_init_med=round(float(np.median(ti)), 3) if ti else None,
                    n_moving_at_init=sum(bool(x[c].get("moving_at_init")) for x in cells),
                    n_no_init=sum(x[c]["t_init_rec_s"] is None for x in cells))
    S = dict(per_config={c: summ(c, use) for c in INIT},
             per_offset={o: {c: summ(c, [x for x in use if x["off"] == o]) for c in INIT} for o in OFFS},
             paired_base_to_aud=paired(use))
    loops = {}
    for r in STD:
        for c, pre in (("base", FIL / f"std{r[-12:-6]}_F1"), ("aud", OUT / f"std{r[-12:-6]}_aud")):
            if Path(str(pre) + ".json").exists():
                loops[f"{r}_{c}"] = dict(fl.loop_eval(str(pre) + ".json"), **init_info(pre, r, INIT[c][0], None))
    board = {}
    for r in BOARD_RECS:
        d = BOARD / r.split("_")[2]
        sc = json.load(open(d / "score.json"))["rows"]
        for o in OFFS:
            for c in INIT:
                row = sc.get(f"o{o}_{c}", {})
                board[f"{r[-12:-6]}_o{o}_{c}"] = dict({k: row.get(k) for k in ("div", "ate_w", "ate_sim3_cm", "scale",
                                                                               "tracked_frac")},
                                                      **init_info(d / f"o{o}_{c}", r, INIT[c][0], "imu_gt_spl"))
    res = dict(made=time.strftime("%F %T"), plan="docs/init_relax_plan.md", gt=GT, excluded=DROP, threads=THREADS,
               init=INIT, base_source="runs/fi_limits/<key>_F1",
               identity_check=check_identity() if (OUT / f"{CHK}.json").exists() else None,
               summary=S, loops=loops, board=board, rows=rows)
    json.dump(res, open(OUT / "score.json", "w"), indent=1, ensure_ascii=False)
    print(json.dumps({k: res[k] for k in ("identity_check", "summary")}, ensure_ascii=False, indent=1))
    print(json.dumps(loops, ensure_ascii=False, indent=1))
    print(json.dumps(board, ensure_ascii=False, indent=1))


def main():
    cmd = sys.argv[1]
    OUT.mkdir(parents=True, exist_ok=True)
    if cmd == "job":
        return job(sys.argv[2])
    if cmd == "run":
        return run_sched()
    score()


if __name__ == "__main__":
    main()
