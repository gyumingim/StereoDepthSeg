#!/usr/bin/env python3
"""Init window test analysis (docs/init_window_plan.md). Per run: TRUE init time from the DEBUG log ([DIAGT] frame time of
the 'successful initialization' line, bridge clock -> clip time via the first pose line), not the bench t_init_clip_s
(= first tracking pose = first visual update, comes only after motion when ZUPT holds the still). Labels:
  noinit   no successful init in the 20 s clip (safe)
  late     init later than the still left + 0.1 s (safe: the drone would not take off before init)
  collapse init before take-off and full-clip rule fails (Sim3 scale outside 0.5..2 or SE3 ATE > 10 cm) or unlabelled
  success  init before take-off and full-clip rule ok
still left = fine motion onset (0.1 s windows, c5_retest_analyse.fine_onset) - start_off.
Init bias error = |bg_init (log) - mean raw gyro of the start still [0.5 s, fine onset - 0.3 s]| in mdeg/s.

  ../venv_ffs/bin/python imu_learn/c5_initwin_analyse.py runs/init_window [--html docs/init_window_result.html --concl FILE]
"""
import argparse
import gzip
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import c5_overnight_analyse as an  # noqa: E402
from c5_retest_analyse import fine_onset  # noqa: E402
from imu_gt import read_imu  # noqa: E402

SCAN3D = HERE.parent
STILL = SCAN3D / "runs" / "overnight_vio" / "inputs"
NAME = re.compile(r"^(L0|L2)_S(\d)_(W05|W10|W15)_(D224|D5|D15)_(\d{6})_(dm025|d0|dp025)$")
WV = {"W05": 0.5, "W10": 1.0, "W15": 1.5}
DV = {"D224": 2.24, "D5": 5.0, "D15": 15.0}
TOL = 0.1
ANSI = re.compile(r"\x1b\[[0-9;]*m")
R2MD = 180 / np.pi * 1000


def true_init(prefix):
    """(bridge time of the init frame, bg at init rad/s) from the DEBUG log, or (None, None)."""
    t = None
    for line in gzip.open(prefix + ".stderr.log.gz", "rt", errors="replace"):
        line = ANSI.sub("", line)
        if "[DIAGT]" in line:
            t = float(line.split("[DIAGT]")[1])
        elif "successful initialization" in line:
            return t
    return None


def first_pose_t(prefix):
    with gzip.open(prefix + ".poses.jsonl.gz", "rt") as f:
        return float(json.loads(f.readline())["t"])


_BG = {}


def still_bias(rec):
    """mean raw gyro (type 16 v0..2, mdeg/s) over the start still [0.5 s, fine onset - 0.3 s] of the clip."""
    if rec not in _BG:
        s = json.load(open(STILL / f"{rec}_still.json"))
        fo = fine_onset(rec, s["motion_start_s"])
        _, gyr = read_imu(SCAN3D / "recordings" / rec / "imu.csv", raw=True)
        t = (gyr[:, 0] - s["clip_start_imu_ns"]) / 1e9
        m = (t >= 0.5) & (t <= fo - 0.3)
        _BG[rec] = dict(fine_s=fo, bg_mdps=(gyr[m, 1:4].mean(0) * R2MD).tolist(), n=int(m.sum()),
                        win=[0.5, round(fo - 0.3, 2)])
    return _BG[rec]


def label_run(prefix):
    m = NAME.match(Path(prefix).name)
    d = json.load(open(prefix + ".json"))
    r = an.analyse_run(prefix, k=1.0)
    rec = d["rec"]
    sb = still_bias(rec)
    r.update(lead=m[1], S=int(m[2]), W=m[3], D=m[4], dn=m[6], rec6=m[5])
    r["still_left_s"] = round(sb["fine_s"] - d["start_off"], 3)
    ti = true_init(prefix)
    r["t_init_true_s"] = None if ti is None else round(ti - first_pose_t(prefix), 3)
    info = d.get("init_info") or {}
    r["init_disp"] = [info.get("disp_old"), info.get("disp_new")]
    r["init_acc_var"] = [info.get("acc_var_old"), info.get("acc_var_new")]
    if "bg_init" in info:
        e = np.asarray(info["bg_init"]) * R2MD - np.asarray(sb["bg_mdps"])
        r["bg_err_mdps"] = float(np.linalg.norm(e))
        r["bg_err_axes_mdps"] = e.round(1).tolist()
    else:
        r["bg_err_mdps"] = None
    if r["t_init_true_s"] is None:
        r["label"] = "noinit"
    elif r["t_init_true_s"] > r["still_left_s"] + TOL:
        r["label"] = "late"
    elif not r.get("initialized") or r.get("collapsed_full") is None or r["collapsed_full"]:
        r["label"] = "collapse"
    else:
        r["label"] = "success"
    r["late_traj"] = (None if r["label"] != "late" else
                      ("no-track" if not r.get("initialized") else
                       "unlabelled" if r.get("collapsed_full") is None else
                       "collapsed" if r["collapsed_full"] else "ok"))
    r["version"] = {k: d.get(k) for k in ("bench_blob", "runner_blob", "imu_gt_blob", "make_ov_config_blob")}
    r["version"]["bin"] = sorted((d.get("binary_sha256") or {}).values())
    r["imu_source"] = d.get("imu_source")
    r["config_init"] = {k: v for k, v in (d.get("config_used") or {}).items()
                        if k in ("init_window_time", "init_max_disparity", "init_imu_thresh", "init_dyn_use")}
    return r


def med(x):
    x = [v for v in x if v is not None]
    return float(np.median(x)) if x else None


def summarize(d):
    runs, failed = [], []
    for f in sorted(Path(d).glob("*.json")):
        if not NAME.match(f.stem):
            continue
        try:
            runs.append(label_run(str(f)[:-5]))
        except Exception as e:  # noqa: BLE001
            failed.append(dict(name=f.stem, why=repr(e)))
    for e in Path(d).glob("*.err"):
        if NAME.match(e.stem) and not Path(str(e)[:-4] + ".json").exists():
            failed.append(dict(name=e.stem, why="no json (running or failed)"))
    cells = {}
    keyf = lambda r: f"{r['lead']}|S{r['S']}|{r['W']}|{r['D']}"  # noqa: E731
    by = defaultdict(list)
    for r in runs:
        by[keyf(r)].append(r)
    for k, rs in by.items():
        pre = [r for r in rs if r["label"] in ("success", "collapse")]
        cells[k] = dict(n=len(rs), **{lab: sum(r["label"] == lab for r in rs) for lab in ("noinit", "late", "collapse", "success")},
                        collapse_recs=sorted({r["rec6"] for r in rs if r["label"] == "collapse"}),
                        success_recs=sorted({r["rec6"] for r in rs if r["label"] == "success"}),
                        late_collapsed=sum(r["late_traj"] in ("collapsed", "unlabelled") for r in rs),
                        t_init_minus_still_med=med([None if r["t_init_true_s"] is None else r["t_init_true_s"] - r["still_left_s"]
                                                    for r in rs]),
                        bg_err_med_pre=med([r["bg_err_mdps"] for r in pre]),
                        bg_err_max_pre=max([r["bg_err_mdps"] for r in pre if r["bg_err_mdps"] is not None], default=None),
                        ate_med_success=med([r.get("ate_full_cm") for r in rs if r["label"] == "success"]))
    settings = {}
    for w in WV:
        for dd in DV:
            rs = [r for r in runs if r["W"] == w and r["D"] == dd]
            pre = [r for r in rs if r["label"] in ("success", "collapse")]
            settings[f"{w}|{dd}"] = dict(
                n=len(rs), collapse=sum(r["label"] == "collapse" for r in rs),
                collapse_recs=sorted({r["rec6"] for r in rs if r["label"] == "collapse"}),
                succ_L0_S2=sum(r["label"] == "success" for r in rs if r["lead"] == "L0" and r["S"] == 2),
                succ_L0_S1=sum(r["label"] == "success" for r in rs if r["lead"] == "L0" and r["S"] == 1),
                succ_L2_S2=sum(r["label"] == "success" for r in rs if r["lead"] == "L2" and r["S"] == 2),
                succ_L2_S1=sum(r["label"] == "success" for r in rs if r["lead"] == "L2" and r["S"] == 1),
                bg_err_med_pre=med([r["bg_err_mdps"] for r in pre]),
                recs_L0_S2_success=sorted({r["rec6"] for r in rs if r["lead"] == "L0" and r["S"] == 2 and r["label"] == "success"}))
    complete = all(v["n"] == 60 for v in settings.values()) and len(settings) == 9

    def order(st):
        return lambda k: (-st[k]["succ_L0_S2"], -st[k]["succ_L0_S1"],
                          st[k]["bg_err_med_pre"] if st[k]["bg_err_med_pre"] is not None else 1e9,
                          DV[k.split("|")[1]], -WV[k.split("|")[0]])
    # amendment 10:40: coarse pass = L0, d0 only (ranking for the repeat candidates)
    coarse = {}
    for w in WV:
        for dd in DV:
            rs = [r for r in runs if r["W"] == w and r["D"] == dd and r["lead"] == "L0" and r["dn"] == "d0"]
            pre = [r for r in rs if r["label"] in ("success", "collapse")]
            coarse[f"{w}|{dd}"] = dict(n=len(rs), collapse=sum(r["label"] == "collapse" for r in rs),
                                       succ_L0_S2=sum(r["label"] == "success" for r in rs if r["S"] == 2),
                                       succ_L0_S1=sum(r["label"] == "success" for r in rs if r["S"] == 1),
                                       bg_err_med_pre=med([r["bg_err_mdps"] for r in pre]))
    coarse_rank = sorted([k for k, v in coarse.items() if v["collapse"] == 0 and v["n"] == 10], key=order(coarse))
    safe = [k for k, v in settings.items() if v["collapse"] == 0 and v["n"] > 0]
    repeated = [k for k, v in settings.items() if v["n"] >= 30]
    rank = sorted([k for k in repeated if k in safe], key=order(settings))
    versions = sorted({json.dumps(r["version"], sort_keys=True) for r in runs})
    return dict(n_runs=len(runs), failed=failed, complete=complete, mixed_versions=len(versions) > 1, versions=versions,
                still=_BG, cells=cells, settings=settings, safe=safe, rank=rank, best=rank[0] if rank else None,
                coarse=coarse, coarse_rank=coarse_rank, repeated=repeated,
                least_collapse=sorted(settings, key=lambda k: settings[k]["collapse"])[:3],
                imu_sources=sorted({str(r["imu_source"]) for r in runs}),
                runs=[{k: r.get(k) for k in ("name", "rec6", "lead", "S", "W", "D", "dn", "start_off", "still_left_s",
                                             "t_init_true_s", "t_init_clip_s", "label", "late_traj", "init_disp",
                                             "init_acc_var", "bg_err_mdps", "bg_err_axes_mdps", "scale_full",
                                             "ate_full_cm", "scale10", "ate10_cm", "collapsed", "collapsed_full",
                                             "initialized", "config_init")} for r in runs])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--html", default=None)
    ap.add_argument("--concl", default=None)
    o = ap.parse_args()
    res = summarize(o.dir)
    json.dump(res, open(Path(o.dir) / "summary.json", "w"), indent=1, ensure_ascii=False, default=float)
    if o.html:
        import c5_initwin_html
        c5_initwin_html.write(res, o.html, Path(o.concl).read_text() if o.concl else "")
    print(json.dumps(dict(n_runs=res["n_runs"], failed=len(res["failed"]), complete=res["complete"],
                          mixed=res["mixed_versions"], imu=res["imu_sources"], safe=res["safe"], rank=res["rank"],
                          coarse_rank=res["coarse_rank"],
                          coarse={k: (v["n"], v["collapse"], v["succ_L0_S2"], v["succ_L0_S1"],
                                      None if v["bg_err_med_pre"] is None else round(v["bg_err_med_pre"]))
                                  for k, v in res["coarse"].items()},
                          settings={k: (v["n"], v["collapse"], v["succ_L0_S2"], v["succ_L0_S1"], v["succ_L2_S2"],
                                        v["succ_L2_S1"], None if v["bg_err_med_pre"] is None else round(v["bg_err_med_pre"]))
                                    for k, v in res["settings"].items()}), ensure_ascii=False, indent=1))
    for k in sorted(res["cells"]):
        c = res["cells"][k]
        print(k, c["n"], "noinit", c["noinit"], "late", c["late"], "coll", c["collapse"], c["collapse_recs"], "succ",
              c["success"], "dt", None if c["t_init_minus_still_med"] is None else round(c["t_init_minus_still_med"], 2),
              "bg", None if c["bg_err_med_pre"] is None else round(c["bg_err_med_pre"]))


if __name__ == "__main__":
    main()
