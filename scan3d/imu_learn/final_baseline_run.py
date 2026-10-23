#!/usr/bin/env python3
"""Final baseline check, 2026-10-23 (docs/final_baseline_20261023.md). User decision: baseline = base_nozupt + lever arm +
new lens (no 6-face accel calib, no gyro rebias).
  final = config/mono + try_zupt=false, stock bridge, clones 11, strict init, window 1.0, disparity 15, frame cache,
          opencv threads 2, gyro_rebias False, imu_calib None, lever config/imu_cam_lever.json, lens = bench default rule
          (2.22 dpt -> config/lens/hs_1920x1080_f2.22_cal20261022.json), 15 recordings x o0/o1/o2, one run per cell.
Comparison (re-scored, not re-run): runs/phone60/<key>_base (this morning: config/mono always-on ZUPT, old lens, no lever)
and runs/phone60/<key>_V4 (try_zupt=false, old lens, no lever). Score = imu_learn/lens_switch_gtscore.py functions
(window ATE / divergence on the camera position, bench evaluate for Sim3 ATE / scale) on the NEW GT imu_gt_spl_cal1022
(old GT imu_gt_spl also reported).

  (setsid nohup ../venv_ffs/bin/python imu_learn/final_baseline_run.py run > runs/final_baseline/sched.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/final_baseline_run.py score     # -> runs/final_baseline/score.json
"""
import argparse
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

PY = "../venv_ffs/bin/python"
OUT = Path("runs/final_baseline")
PAR = int(os.environ.get("PAR", 12))
LEVER = "config/imu_cam_lever.json"
OFFS = (0, 1, 2)
CMP = {"final": "runs/final_baseline/{k}_final", "morning": "runs/phone60/{k}_base", "V4": "runs/phone60/{k}_V4"}
GTS = {"newK": "imu_gt_spl_cal1022", "oldK": "imu_gt_spl"}


def jobs():
    return [dict(rec=r, off=o, name=f"{r[-6:]}_o{o}_final") for r in RAW + OLD for o in OFFS]


def job(name):
    import vio_bench_hs as vb
    j = next(x for x in jobs() if x["name"] == name)
    vb.TEMPLATE = SCAN3D / "config" / "mono" / "estimator_config.yaml"
    a = argparse.Namespace(rec=f"recordings/{j['rec']}", fps=60.0, size="640x360", npts=200, clones=11, noise="cfg",
                           noise_scale=1.0, imu_calib=None, lever=LEVER, lens_file=None, dropout="", max_secs=0.0,
                           imu_raw=j["rec"] not in OLD, gt="auto", out=None, init_cfg="strict", start_off=float(j["off"]),
                           decode_threads=2, imu_lead=2.0, init_window=1.0, init_disp=15.0, frame_cache=True,
                           gyro_rebias=False, cfg_over="num_opencv_threads=2,try_zupt=false", still_hold=None)
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
    r.update(fb_made=time.strftime("%F %T"), fb_job_s=round(time.time() - t0, 1), fb_runner="imu_learn/final_baseline_run.py")
    with gzip.open(str(OUT / name) + ".poses.jsonl.gz", "wt") as f:
        for m in saved.get("poses", []):
            f.write(json.dumps(m) + "\n")
    json.dump(r, open(OUT / f"{name}.json", "w"), indent=1, ensure_ascii=False)


def run_sched():
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
            running[j["name"]] = (subprocess.Popen(["nice", "-n", "5", PY, "imu_learn/final_baseline_run.py", "job", j["name"]],
                                                   stdout=subprocess.DEVNULL,
                                                   stderr=open(OUT / f"{j['name']}.err", "w")), time.time())
        time.sleep(2)
    print(f"{time.strftime('%F %T')} end wall {time.time() - t_all:.0f} s", flush=True)


def score():
    import lens_switch_gtscore as gs
    os.chdir(SCAN3D)
    gs.OUT = OUT
    rows, check = {}, []
    for r in RAW + OLD:
        for o in OFFS:
            key = f"{r[-6:]}_o{o}"
            row = dict(rec=r, key=key, off=o)
            for c, pat in CMP.items():
                pre = pat.format(k=key)
                stored = json.load(open(pre + ".json"))
                row[c] = {}
                for g, gd in GTS.items():
                    mc = gs.cam_metrics(pre, r, gd)
                    be = gs.bench_eval(pre, r, gd)
                    row[c][g] = dict(div=bool(mc.get("divergent")), ate_w=mc.get("ate_w"), rel5_max=mc.get("rel5_max"), **be)
                check.append(all(row[c]["oldK"].get(k) == stored.get(k) for k in ("ate_cm", "ate_sim3_cm", "scale")))
                row[c]["meta"] = {k: stored.get(k) for k in ("lens_file", "lens_source", "lens_sha256", "lever", "cfg_over",
                                                            "gyro_rebias", "imu_calib", "t_init_clip_s")}
            rows[key] = row
            print(key, {c: {g: (('*' if v['div'] else '') + (str(round(v['ate_w'], 2)) if v['ate_w'] is not None else 'None'),
                                v.get('scale')) for g, v in row[c].items() if g in GTS} for c in CMP}, flush=True)
    use = [x for x in rows.values() if x["rec"] != DROP]

    def summ(c, g, cells):
        ok = [x for x in cells if not x[c][g]["div"]]
        v = lambda f: round(float(np.median([f(x[c][g]) for x in ok])), 4) if ok else None  # noqa: E731
        return dict(div=sum(x[c][g]["div"] for x in cells), n=len(cells), n_normal=len(ok),
                    ate_w_med=v(lambda y: y["ate_w"]), sim3_med=v(lambda y: y["ate_sim3_cm"]),
                    scale_med=v(lambda y: y["scale"]), abs_scale_err_med=v(lambda y: abs(y["scale"] - 1)))

    def paired(a, b, g, cells):
        both = [x for x in cells if not x[a][g]["div"] and not x[b][g]["div"]]
        d = [x[b][g]["ate_w"] - x[a][g]["ate_w"] for x in both]
        return dict(n_both_normal=len(both), d_med=round(float(np.median(d)), 3) if d else None,
                    n_b_better=int(sum(v < 0 for v in d)),
                    med_a=round(float(np.median([x[a][g]["ate_w"] for x in both])), 4) if both else None,
                    med_b=round(float(np.median([x[b][g]["ate_w"] for x in both])), 4) if both else None)
    S = {g: dict(per_config={c: summ(c, g, use) for c in CMP},
                 per_offset={o: {c: summ(c, g, [x for x in use if x["off"] == o]) for c in CMP} for o in OFFS},
                 paired_morning_to_final=paired("morning", "final", g, use),
                 paired_V4_to_final=paired("V4", "final", g, use)) for g in GTS}
    ident = []
    for r in RAW + OLD:                     # o0 final must equal the lens A/B 'new' run except lens_file/source path text
        a_, b_ = OUT / f"{r[-6:]}_o0_final", Path(f"runs/lens_switch/{r[-6:]}_o0_new")
        if Path(str(a_) + ".poses.jsonl.gz").exists() and Path(str(b_) + ".poses.jsonl.gz").exists():
            strip = lambda P: [{k: v for k, v in m.items() if k not in ("ms", "wall_ms")} for m in P]  # noqa: E731
            ident.append(strip(gs.load_poses(a_)) == strip(gs.load_poses(b_)))
    res = dict(made=time.strftime("%F %T"), doc="docs/final_baseline_20261023.md", cmp=CMP, gts=GTS, excluded=DROP,
               reproduce_stored_with_oldK=f"{sum(check)}/{len(check)}",
               o0_identical_to_lens_switch_new=f"{sum(ident)}/{len(ident)}", summary=S, rows=rows)
    json.dump(res, open(OUT / "score.json", "w"), indent=1, ensure_ascii=False)
    print(json.dumps({k: res[k] for k in ("reproduce_stored_with_oldK", "o0_identical_to_lens_switch_new")}))
    print(json.dumps(S, ensure_ascii=False, indent=1))


def main():
    cmd = sys.argv[1]
    OUT.mkdir(parents=True, exist_ok=True)
    if cmd == "job":
        return job(sys.argv[2])
    {"run": run_sched, "score": score}[cmd]()


if __name__ == "__main__":
    main()
