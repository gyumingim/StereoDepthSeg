#!/usr/bin/env python3
"""H tuning 2026-10-21 (docs/h_tune_plan.md): an IMU-only still-hold variant that keeps the set-down fix of H
(--still-hold 1.0:0.05:0.05, docs/z2_isolate.html) and passes the three z2_regression criteria in all 18 groups.
Same bench args as imu_learn/z2_regress.py (jobs()), only --still-hold changes, plus OpenVINS num_opencv_threads 2 if
the determinism check (stage det) shows it gives the same poses as the stored base (else 4 = unchanged).

  ../venv_ffs/bin/python imu_learn/h_tune.py mask                 # predicted held frames per run x variant (no VIO)
  (setsid nohup ../venv_ffs/bin/python imu_learn/h_tune.py run det > runs/h_tune/sched_det.out 2>&1 &)
  (setsid nohup ../venv_ffs/bin/python imu_learn/h_tune.py run coarse > runs/h_tune/sched_coarse.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/h_tune.py analyse
"""
import argparse
import csv
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
import z2_regress as zr  # noqa: E402
import z2_isolate as zi  # noqa: E402

PY = "../venv_ffs/bin/python"
OUT = Path("runs/h_tune")
LOG = OUT / "sched.log"
PAR = int(os.environ.get("PAR", 5))
THREADS = os.environ.get("OV_THREADS", "2")          # set to 4 (= template) if stage det fails
H0 = "1.0:0.05:0.05"
VARIANTS = {"H": H0, "Hm": H0 + ":m", "Ht5": H0 + ":t5", "Ht10": H0 + ":t10",
            "Hs": "1.0:0.03:0.03", "Hw2": "2.0:0.05:0.05", "Hms": "1.0:0.03:0.03:m", "Hmw2": "2.0:0.05:0.05:m",
            "Hm05": H0 + ":m0.5", "Hsm05": "1.0:0.03:0.03:m0.5", "Hw15m05": "1.5:0.05:0.05:m0.5",
            "Hw2m05": "2.0:0.05:0.05:m0.5"}
COARSE_CELLS = ["O_154320_o2", "F_222747_o1", "T_214138_S2_dm025", "T_222747_S1_d0", "T_224323_S1_dm025",
                "F_224323_o0", "F_223815_o0"]          # run order: the longest (O_154320, ~16 min) first
COARSE_VARIANTS = ["Hm05", "Hw15m05", "Hsm05", "Hw2m05"]
CONF_PAIRS = ["T_224323_S1_dm025", "F_222747_o1"]
FILES = zr.FILES[:3] + ("imu_learn/z2_regress.py", "imu_learn/z2_isolate.py", "imu_learn/h_tune.py") + zr.FILES[4:]
DET_CELL = "T_224323_S1_dm025"


def log(msg):
    OUT.mkdir(parents=True, exist_ok=True)
    line = f"{time.strftime('%F %T')} ht: {msg}"
    with open(LOG, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


def blobs():
    head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    b = {f: subprocess.run(["git", "hash-object", f], capture_output=True, text=True).stdout.strip() for f in FILES}
    dirty = subprocess.run(["git", "status", "--porcelain", *FILES], capture_output=True, text=True).stdout.strip()
    return head, b, bool(dirty)


def base_jobs():
    return {j["pair"]: j for j in zr.jobs() if j["cond"] == "base"}


def name(pair, var):
    g, o = pair.rsplit("_", 1)
    return f"{g}_{var}_{o}"


def mkjob(pair, var, threads=None):
    j = base_jobs()[pair]
    return dict(j, cond=var, name=name(pair, var), hold=VARIANTS.get(var, ""), threads=threads or THREADS)


# ---------------------------------------------------------------- prediction (no VIO): same decision code as the bench
def predict(j, spec):
    """Held fed frames of job j under spec: same t_start/selection lines as vio_bench_hs.run and the same
    vio_bench_hs.StillHold (checked against frames_held of every executed run)."""
    import vio_bench_hs as vb
    from imu_gt import read_imu
    a = j["args"]
    rec = SCAN3D / a[0]
    off = float(a[a.index("--start-off") + 1])
    max_secs = float(a[a.index("--max-secs") + 1])
    meta = json.load(open(rec / "meta.json"))
    t_frames = np.array([int(r["t_mid_ns"]) for r in csv.DictReader(open(rec / "frames_cam5.csv"))])
    al = json.load(open(rec / "imu_gt" / "align.json"))
    dt_ns = int(al["dt_ns"])
    t_start = t_frames[0]
    if (rec / "imu_gt" / "trim.json").exists():
        t_start = max(t_start, json.load(open(rec / "imu_gt" / "trim.json"))["start_ns"])
    t_start = t_start + int(off * 1e9)
    t_end = min(t_start + int(max_secs * 1e9) if max_secs else t_frames[-1], t_frames[-1])
    acc, gyr = read_imu(rec / "imu.csv", raw="--imu-raw" in a)
    a_i = np.stack([np.interp(gyr[:, 0], acc[:, 0], acc[:, 1 + k]) for k in range(3)], 1)
    g_i = gyr[:, 1:4].copy()
    t_imu = gyr[:, 0].astype(np.int64)
    period = 1e9 / float(meta.get("save_fps") or 240)
    sel = vb.pick_by_time(t_frames, t_start, t_end, 60.0, period)
    sh = vb.StillHold(spec, t_imu, g_i, a_i)
    held, segs, cur = 0, [], None
    for fi in sel:
        tc = t_frames[fi]
        h = sh.step(tc - dt_ns, (tc - t_start) / 1e9)
        held += h
        if h and cur is None:
            cur = [(tc - t_start) / 1e9, None]
            segs.append(cur)
        elif not h and cur is not None:
            cur[1] = (tc - t_start) / 1e9
            cur = None
    return dict(held=int(held), segs=[[round(s, 2), None if e is None else round(e, 2)] for s, e in segs],
                clip_s=round((t_frames[sel[-1]] - t_start) / 1e9, 2))


def mask():
    bj = base_jobs()
    res = json.load(open(OUT / "mask.json"))["runs"] if (OUT / "mask.json").exists() else {}
    for pr, j in bj.items():
        res.setdefault(pr, {}).update({v: predict(j, s) for v, s in VARIANTS.items() if v not in res.get(pr, {})})
        # check: H prediction vs the executed z2_isolate H run
        f = zi.OUT / (name(pr, "H") + ".json")
        if f.exists():
            res[pr]["H"]["actual_held"] = json.load(open(f)).get("frames_held")
        print(pr, {v: (x["held"], x["segs"][:4]) for v, x in res[pr].items()}, "actual H", res[pr]["H"].get("actual_held"),
              flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    json.dump(dict(made=time.strftime("%F %T"), variants=VARIANTS, blobs=blobs()[1], runs=res),
              open(OUT / "mask.json", "w"), indent=1)
    bad = [pr for pr, x in res.items() if "actual_held" in x["H"] and x["H"]["actual_held"] != x["H"]["held"]]
    print("H prediction mismatches:", bad)


# ---------------------------------------------------------------- runs
def cmd(j):
    over = f"num_opencv_threads={j['threads']}" if j["threads"] != "4" else ""
    return [PY, "imu_learn/z2_isolate.py", "job", "--cfg-over", over, "--still-hold", j["hold"], "--", *j["args"],
            "--out", str(OUT / j["name"])]


def stage_jobs(st):
    cp = json.load(open(OUT / "copyplan.json")) if (OUT / "copyplan.json").exists() else {}
    if st == "det":      # threads 2 + new bench: base and H on one input, compared with the stored runs
        return [dict(mkjob(DET_CELL, "base"), name=f"DET_{name(DET_CELL, 'base')}", hold=""),
                dict(mkjob(DET_CELL, "H"), name=f"DET_{name(DET_CELL, 'H')}")]
    if st == "conf":     # copy-rule check: zero-held Hm05 runs must equal base pose by pose
        return [dict(mkjob(pr, "Hm05"), name=f"CONF_{name(pr, 'Hm05')}") for pr in CONF_PAIRS]
    if st == "coarse":   # 1 offset per group; copy-rule cells are not run
        return [mkjob(c, v) for c in COARSE_CELLS for v in COARSE_VARIANTS if c not in cp.get(v, {})]
    if st.startswith("fine:"):          # fine:V1,V2 -> every pair (3 offsets, 18 groups) not covered by the copy rule
        js = [mkjob(pr, v) for v in st[5:].split(",") for pr in base_jobs() if pr not in cp.get(v, {})]
        return sorted(js, key=lambda j: not j["pair"].startswith("O_154320"))
    raise ValueError(st)


def run_queue(js):
    OUT.mkdir(parents=True, exist_ok=True)
    head, b, dirty = blobs()
    js = [j for j in js if not (OUT / f"{j['name']}.json").exists()]
    log(f"start git {head} dirty={dirty} PAR {PAR} threads {THREADS} jobs {len(js)} blobs {b}")
    running = {}
    while js or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is None:
                continue
            del running[n]
            try:
                d = json.load(open(OUT / f"{n}.json"))
                tail = f"init {d.get('t_init_clip_s')} ate {d.get('ate_cm')} held {d.get('frames_held')} err {d.get('error')}"
            except Exception:  # noqa: BLE001
                lines = (OUT / f"{n}.err").read_text(errors="replace").strip().splitlines()
                tail = "FAILED: " + (lines[-1][-300:] if lines else "")
            log(f"done {n} ({time.time() - t0:.0f} s, load {open('/proc/loadavg').read().split()[0]}) {tail}")
        while js and len(running) < PAR:
            j = js.pop(0)
            err = open(OUT / f"{j['name']}.err", "w")
            running[j["name"]] = (subprocess.Popen(["nice", "-n", "5"] + cmd(j), stdout=subprocess.DEVNULL, stderr=err),
                                  time.time())
            log(f"start {j['name']} ({len(js)} left) {' '.join(cmd(j)[3:8])}")
        time.sleep(5)
    log("end (queue done)")


# ---------------------------------------------------------------- analysis
HELD_FAST_SHARE = 0.005      # held-interval check (plan amendment, verifier 2026-10-21): GT samples > 2 cm/s share
HELD_FAST_MAX = 0.03         # and max GT speed (m/s) inside held intervals


def held_motion(prefix):
    """GT speed and |gyro| inside every held interval (hold_segments_imu_ns, IMU clock; open end = last pose).
    Replaces the structurally-zero false-accept count for hold variants (held frames give zero disparity)."""
    import gzip
    d = json.load(open(str(prefix) + ".json"))
    segs = d.get("hold_segments_imu_ns") or []
    if not segs:
        return dict(held_gt_n=0, held_gt_fast=0, held_gt_vmax_cms=None, held_gyro_max=None)
    gt_t, gv = zr.gt_speed(d["rec"])
    ti, gn = zr.gyro_norm(d["rec"], d.get("imu_source") == "raw_16_35")
    dt = d["align_dt_ms"] * 1e6
    t_last = max(m["t_ns"] for m in map(json.loads, gzip.open(str(prefix) + ".poses.jsonl.gz", "rt")) if "t_ns" in m)
    n = fast = 0
    vmax = gmax = 0.0
    for s, e in segs:
        e = t_last if e is None else e
        m = (gt_t >= s + dt) & (gt_t < e + dt) & np.isfinite(gv)
        n += int(m.sum())
        fast += int((gv[m] > zr.V_MOVE).sum())
        vmax = max(vmax, float(gv[m].max(initial=0)))
        gmax = max(gmax, float(gn[(ti >= s) & (ti < e)].max(initial=0)))
    return dict(held_gt_n=n, held_gt_fast=fast, held_gt_vmax_cms=round(vmax * 100, 2), held_gyro_max=round(gmax, 4))


def cell_of(prefix, j):
    r = zr.analyse_one(str(prefix))
    d = json.load(open(str(prefix) + ".json"))
    r.update(group=j["group"], cond=j["cond"], pair=j["pair"], frames_held=d.get("frames_held"),
             still_hold=d.get("still_hold"), threads=(d.get("config_used") or {}).get("num_opencv_threads"))
    if j["rec"][-6:] in zi.SETDOWN and j["pair"].startswith("F_"):
        r.update(zi.setdown_metrics(str(prefix)))
    r.update(held_motion(prefix))
    return r


def src_of(pr, var, copyplan):
    """Result file of (pair, variant): this folder; base/H from z2_regress/z2_isolate; zero-held pairs of a full-stage
    variant = the base run (copy rule of the plan, checked on executed zero-held runs)."""
    p = OUT / name(pr, var)
    if Path(str(p) + ".json").exists():
        return p, "this run"
    if var == "base":
        return zr.OUT / name(pr, "base"), "z2_regress"
    if var == "H":
        return zi.OUT / name(pr, "H"), "z2_isolate"
    cp = copyplan.get(var, {}).get(pr)
    if cp == "base":
        return zr.OUT / name(pr, "base"), "base copy (0 held)"
    if cp == "H":
        return zi.OUT / name(pr, "H"), "H copy (same held frames)"
    return None, None


def make_copyplan():
    """Plan rule: a (pair, variant) whose predicted held frames are 0 feeds exactly the base frames -> base result; one
    whose predicted hold segments equal H's feeds exactly the H frames -> H result. Valid only if stage det showed
    threads 2 == stored runs; the executed zero-held / H-equal runs check it."""
    mk = json.load(open(OUT / "mask.json"))["runs"]
    plan = {}
    for pr, x in mk.items():
        for v in VARIANTS:
            if v == "H" or v not in x:
                continue
            if x[v]["held"] == 0:
                plan.setdefault(v, {})[pr] = "base"
            elif x[v]["segs"] == x["H"]["segs"] and x[v]["held"] == x["H"]["held"]:
                plan.setdefault(v, {})[pr] = "H"
    json.dump(plan, open(OUT / "copyplan.json", "w"), indent=1)
    return plan


def analyse(variants):
    bj = base_jobs()
    copyplan = json.load(open(OUT / "copyplan.json")) if (OUT / "copyplan.json").exists() else {}
    runs = {}
    for pr, j in bj.items():
        for v in ["base"] + variants:
            p, src = src_of(pr, v, copyplan)
            if p is None or not Path(str(p) + ".json").exists():
                continue
            r = cell_of(p, dict(j, cond=v))
            r["src"] = src
            runs[(pr, v)] = r
    out = {}
    for v in variants:
        groups = {}
        for g in dict.fromkeys(j["group"] for j in bj.values()):
            cells = {}
            for c in ("base", v):
                xs = sorted([r for (pr, cc), r in runs.items() if cc == c and r["group"] == g], key=lambda r: r["pair"])
                if not xs:
                    continue
                sc = [r["ate_cm"] for r in xs if r["status"] in ("ok", "collapse")]
                cells[c] = dict(n=len(xs), pairs=[r["pair"] for r in xs], status=[r["status"] for r in xs],
                                ates=[r["ate_cm"] for r in xs], ate_med=zr.med(sc),
                                false_acc=sum(r["false_acc"] for r in xs), held=[r.get("frames_held") for r in xs],
                                lastwin_med=zr.med([r.get("lastwin_med_cm") for r in xs]),
                                drift_med=zr.med([r.get("drift_cm") for r in xs]), src=[r["src"] for r in xs],
                                held_gt_n=sum(r["held_gt_n"] for r in xs), held_gt_fast=sum(r["held_gt_fast"] for r in xs),
                                held_gt_vmax_cms=max([r["held_gt_vmax_cms"] or 0 for r in xs]),
                                held_gyro_max=max([r["held_gyro_max"] or 0 for r in xs]))
            if v not in cells or "base" not in cells:
                continue
            b, z = cells["base"], cells[v]
            ba = dict(zip(b["pairs"], b["ates"]))
            bs = dict(zip(b["pairs"], b["status"]))
            new_bad = [f"{pr}: {bs[pr]} -> {s}" for pr, s in zip(z["pairs"], z["status"])
                       if pr in bs and ((bs[pr] == "ok" and s != "ok") or (bs[pr] == "noinit" and s in ("collapse", "unscored")))]
            lim = None if b["ate_med"] is None or z["ate_med"] is None else max(b["ate_med"] * 1.10, b["ate_med"] + 0.2)
            dd = [(pr, round(a - ba[pr], 2)) for pr, a in zip(z["pairs"], z["ates"]) if ba.get(pr) is not None and a is not None]
            full3 = z["n"] == 3 and b["n"] == 3
            ate_ok = None if lim is None else z["ate_med"] <= lim + 1e-9
            # amendment 18:19 (before any new-variant result): share rule dropped -> info only (see plan)
            held_ok = z["held_gt_vmax_cms"] <= HELD_FAST_MAX * 100 + 1e-9
            groups[g] = dict(cells=cells, ate_limit=lim, ate_ok=ate_ok, new_collapses=new_bad, false_acc=z["false_acc"],
                             held_ok=held_ok,
                             worst_pair=max(dd, key=lambda x: x[1]) if dd else None, n=z["n"],
                             pass_=(ate_ok is not False) and not new_bad and z["false_acc"] == 0 and held_ok and full3,
                             pair_over=[pr for pr, a in zip(z["pairs"], z["ates"]) if ba.get(pr) is not None and a is not None
                                        and a > max(ba[pr] * 1.10, ba[pr] + 0.2) + 1e-9])
        sd = groups.get("F_224323", {}).get("cells", {})
        helps = None
        if sd.get(v) and sd[v]["lastwin_med"] is not None and sd[v]["n"] == 3:
            helps = (sd[v]["lastwin_med"] <= 0.5 * sd["base"]["lastwin_med"]
                     and sd[v]["drift_med"] <= 0.5 * sd["base"]["drift_med"])
        out[v] = dict(groups=groups, helps_setdown=helps, n_groups=len(groups),
                      n_complete=sum(g["n"] == 3 for g in groups.values()),
                      fails=[g for g, x in groups.items() if not x["pass_"]],
                      crit1=[g for g, x in groups.items() if x["ate_ok"] is False],
                      crit2=[g for g, x in groups.items() if x["new_collapses"]],
                      crit3=[g for g, x in groups.items() if x["false_acc"]],
                      crit3h=[g for g, x in groups.items() if not x["held_ok"]],
                      held_gt_n=sum(x["cells"][v]["held_gt_n"] for x in groups.values()),
                      held_gt_fast=sum(x["cells"][v]["held_gt_fast"] for x in groups.values()),
                      held_gt_vmax_cms=max([x["cells"][v]["held_gt_vmax_cms"] for x in groups.values()] or [0]))
    # coarse view: single cells vs base (pair-level ATE limit, used only for ranking in the coarse pass)
    coarse = {}
    for v in variants:
        rows = []
        for c in COARSE_CELLS:
            r, b = runs.get((c, v)), runs.get((c, "base"))
            if not r or not b:
                continue
            lim = max(b["ate_cm"] * 1.10, b["ate_cm"] + 0.2) if b["ate_cm"] is not None else None
            rows.append(dict(pair=c, base=b["ate_cm"], ate=r["ate_cm"], status=r["status"], base_status=b["status"],
                             held=r.get("frames_held"), lim=lim, over=(lim is not None and r["ate_cm"] is not None
                                                                         and r["ate_cm"] > lim + 1e-9),
                             fa=r["false_acc"], held_fast=r["held_gt_fast"], held_n=r["held_gt_n"],
                             held_vmax=r["held_gt_vmax_cms"], lastwin=r.get("lastwin_med_cm"), drift=r.get("drift_cm"),
                             base_lastwin=b.get("lastwin_med_cm"), base_drift=b.get("drift_cm")))
        coarse[v] = rows
    # determinism / prediction checks
    det = {}
    for v in ("base", "H"):
        p = OUT / f"DET_{name(DET_CELL, v)}"
        ref = (zr.OUT if v == "base" else zi.OUT) / name(DET_CELL, v)
        if Path(str(p) + ".json").exists():
            det[v] = dict(zi.poses_diff(str(ref), str(p)), ate=json.load(open(str(p) + ".json")).get("ate_cm"),
                          ref_ate=json.load(open(str(ref) + ".json")).get("ate_cm"))
    pred = {}
    if (OUT / "mask.json").exists():
        mk = json.load(open(OUT / "mask.json"))["runs"]
        for (pr, v), r in runs.items():
            if v in VARIANTS and r["src"] == "this run" and v in mk.get(pr, {}):
                pred[f"{pr}/{v}"] = (mk[pr][v]["held"], r.get("frames_held"))
    pred_bad = {k: x for k, x in pred.items() if x[0] != x[1]}
    vers = {}
    for r in runs.values():
        vers.setdefault(json.dumps({k: x for k, x in r["version"].items() if k != "dirty"}, sort_keys=True), []).append(
            r["name"])
    return dict(made=time.strftime("%F %T"), git=blobs()[0], blobs=blobs()[1], variants=variants, det=det,
                pred_checked=len(pred), pred_mismatch=pred_bad, code_versions=len(vers),
                version_counts=list(map(len, vers.values())), coarse=coarse, full=out,
                runs=[dict(r, key=f"{k[0]}/{k[1]}") for k, r in runs.items()])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["mask", "copyplan", "run", "list", "analyse"])
    ap.add_argument("stage", nargs="?")
    o, rest = ap.parse_known_args()
    if o.cmd == "mask":
        return mask()
    if o.cmd == "copyplan":
        pl = make_copyplan()
        print({v: (sum(c == "base" for c in x.values()), sum(c == "H" for c in x.values())) for v, x in pl.items()})
        return
    if o.cmd in ("run", "list"):
        js = stage_jobs(o.stage)
        if o.cmd == "list":
            for j in js:
                print(j["name"], " ".join(cmd(j)))
            return
        return run_queue(js)
    if o.cmd == "analyse":
        vs = [v for v in (o.stage or "").split(",") if v]
        res = analyse(vs)
        name_ = "summary" + ("_" + "_".join(vs) if vs else "") + ".json"
        json.dump(res, open(OUT / name_, "w"), indent=1, ensure_ascii=False, default=float)
        print(json.dumps({k: res[k] for k in ("det", "pred_checked", "pred_mismatch", "code_versions", "version_counts")},
                         default=float))
        for v, rows in res["coarse"].items():
            for r in rows:
                print(f"C {v:5s} {r['pair']:20s} base {r['base']} -> {r['ate']} {r['status']} held {r['held']} over {r['over']}"
                      f" fa {r['fa']} hf {r['held_fast']}/{r['held_n']} vmax {r['held_vmax']} lw {r['lastwin']} (b {r['base_lastwin']}) dr {r['drift']} (b {r['base_drift']})")
        for v, x in res["full"].items():
            print(f"F {v}: groups {x['n_groups']} complete {x['n_complete']} fails {x['fails']} helps {x['helps_setdown']}"
                  f" held fast {x['held_gt_fast']}/{x['held_gt_n']} vmax {x['held_gt_vmax_cms']}")
            for g, gg in x["groups"].items():
                print(f"   {g:16s} n {gg['n']} {gg['cells']['base']['ates']} -> {gg['cells'][v]['ates']} lim {gg['ate_limit']}"
                      f" pass {gg['pass_']} worst {gg['worst_pair']} {gg['new_collapses']} src {gg['cells'][v]['src']}")


if __name__ == "__main__":
    main()
