#!/usr/bin/env python3
"""IMU corrections vs base (docs/imucal_plan.md, 2026-10-23). o0 only, 15 recordings, one run per cell.

Base = runs/phone60/<rec>_o0_V4 (config/mono + try_zupt=false = config/base_nozupt, stock bridge, gyro_rebias False,
imu_calib None). Each variant changes exactly one input path of vio_bench_hs.run:
  C1 = 6-face accelerometer scale+bias  (--imu-calib JSON: G=I, bg=0, A=diag(1/s), ba=-b/s, r=0)
  C2 = --gyro-rebias                     (raw IMU only; the two old recordings have no raw gyro -> C2 = base input)
  C3 = C1 + C2                           (old recordings: = C1 input)
  C4 = camera-IMU lever arm              (--imu-calib JSON: G=I, bg=0, A=I, ba=0, r = p_CinI = -R_ci^T r_cm/100)
  id = identity JSON (all zero / identity) -> bit-check against stored V4

  ../venv_ffs/bin/python imu_learn/imucal_run.py calib          # write runs/imucal/calib/*.json
  ../venv_ffs/bin/python imu_learn/imucal_run.py bitcheck [KEY]  # id JSON vs stored V4 poses
  (setsid nohup ../venv_ffs/bin/python imu_learn/imucal_run.py run > runs/imucal/sched.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/imucal_run.py score          # -> runs/imucal/score.json
"""
import argparse
import gzip
import json
import os
import shutil
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

PY = "../venv_ffs/bin/python"
OUT = Path("runs/imucal")
CAL = OUT / "calib"
REF_DIR = Path("runs/phone60")
PAR = int(os.environ.get("PAR", 12))
SIXFACE = Path("config/imu_accel_6face_20261022.json")
LEVER = Path("config/imu_cam_lever.json")
# cond -> (calib json kind or None, gyro_rebias)
CONDS = {"C1": ("C1", False), "C2": (None, True), "C3": ("C1", True), "C4": ("C4", False), "id": ("id", False),
         "L": (None, False)}   # L = bench --lever option (added after the comparison) -> must equal C4 (levercheck)


def calib_files():
    CAL.mkdir(parents=True, exist_ok=True)
    sf = json.load(open(SIXFACE))
    s = 1.0 + np.asarray(sf["scale_minus_1_pct"]) / 100.0
    b = np.asarray(sf["bias_mps2"])
    # model a_meas = s*a_true + b  ->  a_true = a_meas/s - b/s ; bench applies a_i @ A.T + ba
    A = np.diag(1.0 / s)
    ba = -b / s
    I3 = np.eye(3).tolist()
    r_cm = np.asarray(json.load(open(LEVER))["r_cm"], float)
    info = {}
    for rec in RAW + OLD:
        R_ci = np.asarray(json.load(open(f"recordings/{rec}/imu_gt/align.json"))["R_ci"])
        p_CinI = -R_ci.T @ (r_cm / 100.0)       # r = IMU position in the camera frame (p_IinC) -> camera position in IMU frame
        base = dict(G=I3, bg=[0.0, 0.0, 0.0], A=I3, ba=[0.0, 0.0, 0.0])
        c1 = dict(base, A=A.tolist(), ba=ba.tolist())
        c4 = dict(base, r=p_CinI.tolist())
        src = dict(made=time.strftime("%F %T"), sixface=str(SIXFACE), lever=str(LEVER), rec=rec, R_ci=R_ci.tolist())
        json.dump(dict(src, kind="C1", calib=c1), open(CAL / f"{rec[-6:]}_C1.json", "w"), indent=1)
        json.dump(dict(src, kind="C4", r_cm=r_cm.tolist(), calib=c4), open(CAL / f"{rec[-6:]}_C4.json", "w"), indent=1)
        json.dump(dict(src, kind="id", calib=base), open(CAL / f"{rec[-6:]}_id.json", "w"), indent=1)
        info[rec[-6:]] = dict(p_CinI_cm=[round(x * 100, 3) for x in p_CinI])
    print(json.dumps(dict(A_diag=np.diag(A).tolist(), ba=ba.tolist(), lever=info), indent=1))


def replay(rec, off, cond):
    import vio_bench_hs as vb
    kind, rb = CONDS[cond]
    raw = Path(rec).name not in OLD
    if rb and not raw:
        raise SystemExit(f"{cond} on old recording {rec}: no raw gyro (handled in score as a copy)")
    vb.TEMPLATE = SCAN3D / "config" / "mono" / "estimator_config.yaml"
    a = argparse.Namespace(rec=str(rec), fps=60.0, size="640x360", npts=200, clones=11, noise="cfg",
                           noise_scale=1.0, imu_calib=str(CAL / f"{Path(rec).name[-6:]}_{kind}.json") if kind else None,
                           dropout="", max_secs=0.0, imu_raw=raw,
                           gt="auto", out=None, init_cfg="strict", start_off=float(off), decode_threads=2,
                           imu_lead=2.0, init_window=1.0, init_disp=15.0, frame_cache=True, gyro_rebias=rb,
                           cfg_over="num_opencv_threads=2,try_zupt=false", still_hold=None,
                           lever=str(LEVER) if cond == "L" else None)
    saved = {}
    orig_ev = vb.evaluate

    def ev(rec_, a_, poses, *rest):
        saved["poses"] = poses
        return orig_ev(rec_, a_, poses, *rest)
    vb.evaluate = ev
    try:
        r = vb.run(a)
    finally:
        vb.evaluate = orig_ev
    r.update(ic_cond=cond, ic_calib=a.imu_calib, ic_gyro_rebias=rb)
    return r, saved.get("poses", [])


def strip(P):
    return [{k: v for k, v in m.items() if k not in ("ms", "wall_ms")} for m in P]


def load_poses(pre):
    return [json.loads(x) for x in gzip.open(str(pre) + ".poses.jsonl.gz", "rt") if x.strip()]


def bitcheck(key="134550_o0"):
    rec = next(r for r in RAW + OLD if r.endswith(key.split("_")[0]))
    r, P = replay(f"recordings/{rec}", int(key.split("_o")[1]), "id")
    V = load_poses(REF_DIR / f"{key}_V4")
    same = strip(P) == strip(V)
    res = dict(made=time.strftime("%F %T"), key=key, ref=f"{REF_DIR}/{key}_V4", cond="id", n_poses=[len(P), len(V)],
               identical=same, ate=[r.get("ate_cm"), json.load(open(REF_DIR / f"{key}_V4.json")).get("ate_cm")],
               keys_ignored=["ms", "wall_ms"])
    if not same:
        res["first_diff"] = next((i for i, (x, y) in enumerate(zip(strip(P), strip(V))) if x != y), None)
    json.dump(res, open(OUT / "bitcheck.json", "w"), indent=1)
    print(json.dumps(res, indent=1))


def levercheck(key="134550_o0"):
    """bench --lever (new option) must reproduce the C4 JSON run exactly; no lever must still equal stored V4."""
    rec = next(r for r in RAW + OLD if r.endswith(key.split("_")[0]))
    res = dict(made=time.strftime("%F %T"), key=key)
    for cond, ref in (("L", OUT / f"{key}_C4"), ("id", REF_DIR / f"{key}_V4")):
        r, P = replay(f"recordings/{rec}", int(key.split("_o")[1]), cond)
        V = load_poses(ref)
        res[cond] = dict(ref=str(ref), n=[len(P), len(V)], identical=strip(P) == strip(V), ate=r.get("ate_cm"))
    json.dump(res, open(OUT / "levercheck.json", "w"), indent=1)
    print(json.dumps(res, indent=1))


def jobs():
    J = []
    for r in RAW + OLD:
        for c in ("C1", "C2", "C3", "C4"):
            if CONDS[c][1] and r in OLD:
                continue                      # no raw gyro: C2 = base input, C3 = C1 input (score copies)
            J.append(dict(rec=r, off=0, cond=c, name=f"{r[-6:]}_o0_{c}"))
    if os.environ.get("STAGE") == "1":        # plan stage 1: C4 on the 12 cells not divergent in V4
        J = [j for j in J if j["cond"] == "C4" and j["rec"][-6:] not in ("131326", "224323", "213955")]
    return J


def job(name):
    j = next(x for x in jobs() if x["name"] == name)
    t0 = time.time()
    r, P = replay(f"recordings/{j['rec']}", j["off"], j["cond"])
    r.update(ic_made=time.strftime("%F %T"), ic_job_s=round(time.time() - t0, 1))
    with gzip.open(str(OUT / name) + ".poses.jsonl.gz", "wt") as f:
        for m in P:
            f.write(json.dumps(m) + "\n")
    json.dump(r, open(OUT / f"{name}.json", "w"), indent=1, ensure_ascii=False)


def run_sched():
    dirty = subprocess.run(["git", "status", "--porcelain", "--untracked-files=no", "imu_learn/vio_bench_hs.py",
                            "imu_learn/gyro_rebias.py", "imu_learn/imucal_run.py", "config/mono", "ov_bridge/main.cpp",
                            "docs/imucal_plan.md", str(SIXFACE), str(LEVER)], capture_output=True, text=True).stdout
    if dirty.strip():
        sys.exit(f"REFUSED: uncommitted {dirty}")
    if not (OUT / "bitcheck.json").exists() or not json.load(open(OUT / "bitcheck.json")).get("identical"):
        sys.exit("REFUSED: bitcheck missing or not identical")
    q = [j for j in jobs() if not (OUT / f"{j['name']}.json").exists()]
    running, t_all = {}, time.time()
    print(f"{time.strftime('%F %T')} start {len(q)} jobs PAR {PAR}", flush=True)
    while q or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is not None:
                del running[n]
                ok = (OUT / f"{n}.json").exists()
                print(f"{time.strftime('%F %T')} done {n} {time.time() - t0:.0f} s {'ok' if ok else 'FAILED'}", flush=True)
        while q and len(running) < PAR:
            j = q.pop(0)
            running[j["name"]] = (subprocess.Popen(["nice", "-n", "5", PY, "imu_learn/imucal_run.py", "job", j["name"]],
                                                   stdout=subprocess.DEVNULL,
                                                   stderr=open(OUT / f"{j['name']}.err", "w")), time.time())
        time.sleep(2)
    print(f"{time.strftime('%F %T')} end wall {time.time() - t_all:.0f} s", flush=True)


def cam_metrics(pre, rec, zs):
    """zupt_ob_score.run_metrics on the CAMERA position (T_world_cam0 translation) instead of 'p' (= p_IinG, IMU).
    Needed for C4 (IMU != camera once p_CinI != 0); applied to base and every variant alike (plan).
    Kept as the file-rewrite used for the committed score; zupt_ob_score.run_metrics(pos='cam') is now the same (checked in score)."""
    tmp = OUT / "campos"
    tmp.mkdir(exist_ok=True)
    dst = tmp / Path(str(pre)).name
    shutil.copy(str(pre) + ".json", str(dst) + ".json")
    with gzip.open(str(dst) + ".poses.jsonl.gz", "wt") as f:
        for m in load_poses(pre):
            if m.get("T_world_cam0"):
                m["p"] = list(np.asarray(m["T_world_cam0"], float).reshape(4, 4)[:3, 3])
            f.write(json.dumps(m) + "\n")
    return zs.run_metrics(dst, rec)


def score():
    import zupt_ob_score as zs
    os.chdir(SCAN3D)
    rows = {}
    for r in RAW + OLD:
        key = f"{r[-6:]}_o0"
        src = {"base": REF_DIR / f"{key}_V4"}
        for c in ("C1", "C2", "C3", "C4"):
            if CONDS[c][1] and r in OLD:
                src[c] = src["base"] if c == "C2" else OUT / f"{key}_C1"     # identical inputs (documented)
            else:
                src[c] = OUT / f"{key}_{c}"
        row = dict(rec=r, key=key, old=r in OLD)
        for c, pre in src.items():
            d = json.load(open(str(pre) + ".json")) if Path(str(pre) + ".json").exists() else {}
            if not d:
                row[c] = dict(missing=True, div=True)
                continue
            mc = cam_metrics(pre, r, zs)
            mi = zs.run_metrics(pre, r, pos="imu")
            m2 = zs.run_metrics(pre, r)                      # new default (pos='cam') must equal cam_metrics
            assert (m2.get("ate_w"), m2.get("divergent")) == (mc.get("ate_w"), mc.get("divergent")), (pre, m2, mc)
            row[c] = dict(src=str(pre), div=bool(mc.get("divergent")), ate_w=mc.get("ate_w"), rel5_max=mc.get("rel5_max"),
                          div_imu=bool(mi.get("divergent")), ate_w_imu=mi.get("ate_w"),
                          ate_cm=d.get("ate_cm"), scale=d.get("scale"), t_init=d.get("t_init_clip_s"),
                          gyro_rebias=d.get("gyro_rebias"), imu_calib=d.get("imu_calib"), imu_source=d.get("imu_source"))
        rows[key] = row
    use = [x for x in rows.values() if x["rec"] != DROP]
    crit = {}
    nb = sum(x["base"]["div"] for x in use)
    for c in ("C1", "C2", "C3", "C4"):
        nc = sum(x[c]["div"] for x in use)
        normal = [x for x in use if not x["base"]["div"] and not x[c]["div"]]
        mb = float(np.median([x["base"]["ate_w"] for x in normal]))
        mv = float(np.median([x[c]["ate_w"] for x in normal]))
        sb = float(np.median([abs(x["base"]["scale"] - 1) for x in normal]))
        sv = float(np.median([abs(x[c]["scale"] - 1) for x in normal]))
        a_ = nc <= nb
        b_ = mv <= 1.2 * mb + 0.3
        c_ate = mv <= 0.9 * mb
        c_sc = sv <= 0.7 * sb
        crit[c] = dict(a=dict(div_base=nb, div_var=nc, pass_=a_),
                       b=dict(n_normal=len(normal), med_base=round(mb, 3), med_var=round(mv, 3),
                              limit=round(1.2 * mb + 0.3, 3), pass_=b_),
                       c=dict(ate_limit=round(0.9 * mb, 3), ate_pass=c_ate, scale_med_base=round(sb, 4),
                              scale_med_var=round(sv, 4), scale_limit=round(0.7 * sb, 4), scale_pass=c_sc,
                              pass_=c_ate or c_sc),
                       verdict=("개선" if a_ and b_ and (c_ate or c_sc) else ("해 없음 (개선 아님)" if a_ and b_ else "불통과")))
    res = dict(made=time.strftime("%F %T"), plan="docs/imucal_plan.md", base=f"{REF_DIR}/<key>_V4", rows=rows,
               criteria=crit, metric="window ATE / divergence on camera position (cam_metrics); ate_w_imu = old 'p' metric")
    json.dump(res, open(OUT / "score.json", "w"), indent=1, ensure_ascii=False)
    print(json.dumps(crit, ensure_ascii=False, indent=1))
    for k, x in sorted(rows.items()):
        print(k, " | ".join(f"{c} {'*' if x[c]['div'] else ''}{x[c].get('ate_w') and round(x[c]['ate_w'], 2)} "
                            f"s{x[c].get('scale')}" for c in ("base", "C1", "C2", "C3", "C4")))


def main():
    cmd = sys.argv[1]
    OUT.mkdir(parents=True, exist_ok=True)
    if cmd == "job":
        return job(sys.argv[2])
    if cmd == "levercheck":
        return levercheck(*sys.argv[2:3])
    if cmd == "bitcheck":
        return bitcheck(*sys.argv[2:3])
    {"run": run_sched, "score": score, "calib": calib_files}[cmd]()


if __name__ == "__main__":
    main()
