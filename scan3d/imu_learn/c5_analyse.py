#!/usr/bin/env python3
"""C5 analysis (docs/c5_test_plan.md): per run init facts + collapse label (collapse_signals.label, unchanged), per
recording verdict per setting, and the pre-registered decision (setting (a) = cfg_strict).

  ../venv_ffs/bin/python imu_learn/c5_analyse.py runs/c5 --out runs/c5/summary.json
"""
import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import collapse_signals as cs  # noqa: E402

PREV_COLLAPSED = ["115620", "132803", "145816", "151358", "152631", "161035", "163345"]
PREV_HEALTHY = ["114000", "125714", "131326", "154320"]
SETTINGS = ["cfg_loose", "cfg_strict", "cfg_strict30", "m10_loose", "m10_strict", "cfg_strictdyn"]
PRIMARY = "cfg_strict"


def one(prefix):
    d = json.load(open(prefix + ".json"))
    out = dict(name=Path(prefix).name, rec=d["rec"][-6:], init_cfg=d.get("init_cfg"), clones=d.get("clones"),
               noise=d.get("noise"), noise_scale=d.get("noise_scale"), start_off=d.get("start_off"),
               clip_s=d.get("clip_s"), t_init_clip_s=d.get("t_init_clip_s"), covered_frac=d.get("covered_frac"),
               ate_cm=d.get("ate_cm"), scale=d.get("scale"), rpe1s_cm=d.get("rpe1s_cm"), rot_med_deg=d.get("rot_med_deg"),
               tracked_frac=d.get("tracked_frac"), ms_med=d.get("ms_med"), wall_s=d.get("wall_s"),
               dt_cam_imu_final_ms=d.get("dt_cam_imu_final_ms"), dt_cam_imu_med_ms=d.get("dt_cam_imu_med_ms"),
               bench_blob=d.get("bench_blob"), git_head=d.get("git_head"), bench_dirty=d.get("bench_dirty"),
               loadavg_start=d.get("loadavg_start"), error=d.get("error"), bridge_crash=d.get("bridge_crash"))
    run = cs.load(prefix)
    ini = np.nonzero(run["status"] != "init")[0]
    if not len(ini):
        out.update(initialized=False, collapsed=None)
        return out
    i0 = int(ini[0])
    info = run["init_info"]
    out.update(initialized=True, init_method=info.get("method", "?"),
               bg_init_dps=(float(np.degrees(np.linalg.norm(info["bg_init"]))) if "bg_init" in info else None),
               init_disp=[info.get("disp_old"), info.get("disp_new")])
    lab = cs.label(run, i0)
    out.update({k: (float(v) if isinstance(v, (float, np.floating)) else v) for k, v in lab.items()})
    return out


def rec_verdict(runs):
    labelled = [r for r in runs if r.get("initialized") and r.get("collapsed") is not None]
    n_col = sum(bool(r["collapsed"]) for r in labelled)
    if len(labelled) < 2:
        v = "untested"
    elif n_col == 0:
        v = "gone"
    else:
        v = "remains"
    return dict(verdict=v, n_runs=len(runs), n_init=sum(bool(r.get("initialized")) for r in runs),
                n_labelled=len(labelled), n_collapsed=n_col)


def med(x):
    x = [v for v in x if v is not None and not (isinstance(v, float) and math.isnan(v))]
    return round(float(np.median(x)), 3) if x else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--out", required=True)
    o = ap.parse_args()
    runs = []
    for f in sorted(Path(o.dir).glob("*.json")):
        if f.name in ("summary.json",) or not Path(str(f)[:-5] + ".poses.jsonl.gz").exists():
            continue
        try:
            runs.append(one(str(f)[:-5]))
        except Exception as e:   # noqa: BLE001
            runs.append(dict(name=f.stem, analyse_error=repr(e)))
    by = defaultdict(list)
    for r in runs:
        if r["name"].startswith("rep_") or "analyse_error" in r:
            continue
        rec, setting = r["name"].split("_", 1)
        setting = setting.rsplit("_o", 1)[0]
        by[(setting, rec)].append(r)
    table = {}
    for s in SETTINGS:
        recs = {}
        for rec in PREV_COLLAPSED + PREV_HEALTHY:
            rr = by.get((s, rec), [])
            if not rr:
                continue
            v = rec_verdict(rr)
            v.update(t_init_med_s=med([r.get("t_init_clip_s") for r in rr]),
                     covered_med=med([r.get("covered_frac") for r in rr]),
                     methods=sorted({r.get("init_method", "none") for r in rr}),
                     bg_init_med_dps=med([r.get("bg_init_dps") for r in rr]),
                     scale10_med=med([r.get("scale10") for r in rr]), ate10_med_cm=med([r.get("ate10_cm") for r in rr]),
                     ate_med_cm=med([r.get("ate_cm") for r in rr]))
            recs[rec] = v
        if not recs:
            continue
        gone = [k for k in PREV_COLLAPSED if recs.get(k, {}).get("verdict") == "gone"]
        ok_runs = [r for rec in recs for r in by[(s, rec)]]
        table[s] = dict(
            recs=recs, n_complete=sum(v["n_runs"] for v in recs.values()),
            prev_collapsed_gone=gone, n_gone=len(gone), n_gone_wo161035=len([k for k in gone if k != "161035"]),
            n_untested=len([k for k in PREV_COLLAPSED if recs.get(k, {}).get("verdict") == "untested"]),
            healthy_broken=[k for k in PREV_HEALTHY if recs.get(k, {}).get("verdict") == "remains"],
            runs_collapsed=sum(bool(r.get("collapsed")) for r in ok_runs),
            runs_labelled=sum(r.get("collapsed") is not None for r in ok_runs),
            runs_init=sum(bool(r.get("initialized")) for r in ok_runs), runs_total=len(ok_runs),
            bg_over_0p5=sum((r.get("bg_init_dps") or 0) > 0.5 for r in ok_runs),
            t_init_med_s=med([r.get("t_init_clip_s") for r in ok_runs]),
            covered_med=med([r.get("covered_frac") for r in ok_runs]))
    p = table.get(PRIMARY)
    decision = None
    if p:
        complete = all(p["recs"].get(k, {}).get("n_runs") == 3 for k in PREV_COLLAPSED)
        decision = dict(setting=PRIMARY, complete=complete, n_gone=p["n_gone"], of=7,
                        c5="kept" if p["n_gone"] >= 5 else "withdrawn",
                        wo161035=dict(n_gone=p["n_gone_wo161035"], of=6,
                                      c5="kept" if p["n_gone_wo161035"] >= 5 else "withdrawn"))
    blobs = sorted({r.get("bench_blob") for r in runs if r.get("bench_blob")})
    rep = [r for r in runs if r["name"].startswith("rep_")]
    det = []
    for r in rep:
        orig = next((x for x in runs if x["name"] == r["name"][4:]), None)
        if orig:
            det.append(dict(name=orig["name"], load=[orig.get("loadavg_start"), r.get("loadavg_start")],
                            t_init=[orig.get("t_init_clip_s"), r.get("t_init_clip_s")],
                            method=[orig.get("init_method"), r.get("init_method")],
                            scale10=[orig.get("scale10"), r.get("scale10")], ate_cm=[orig.get("ate_cm"), r.get("ate_cm")],
                            collapsed=[orig.get("collapsed"), r.get("collapsed")]))
    json.dump(dict(decision=decision, settings=table, bench_blobs=blobs, determinism=det, runs=runs),
              open(o.out, "w"), indent=1, ensure_ascii=False, default=float)
    print(json.dumps(dict(decision=decision, blobs=blobs, n_runs=len(runs),
                          per_setting={s: {k: t[k] for k in ("n_complete", "n_gone", "n_untested", "runs_collapsed",
                                                             "runs_labelled", "runs_init", "runs_total", "t_init_med_s",
                                                             "covered_med", "healthy_broken")} for s, t in table.items()}),
                     indent=1))


if __name__ == "__main__":
    main()
