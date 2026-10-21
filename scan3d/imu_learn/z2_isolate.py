#!/usr/bin/env python3
"""Z2 isolation 2026-10-21 (docs/z2_isolate_plan.md): which part of Z2 (zupt_chi2_multipler 1 + zupt_noise_multiplier 1)
causes the 2 new collapses of docs/z2_regression.html (F_221052 o1, O_131326 o0), and does a variant fix the set-down
failure (224323) without regressions? Same bench path, args, run set and criteria as imu_learn/z2_regress.py.

Conditions (cfg overrides on the same template; H = external IMU stillness hold, no ZUPT config change):
  base | Z2 chi2 1 + noise 1 | Z1 chi2 1 | N1 noise 1 | H --still-hold 1.0:0.05:0.05 |
  Z2c chi2 0.5 + noise 1 (tighter IMU chi2 gate) | Z2v Z2 + zupt_max_velocity 0.05 (tighter velocity gate)
Phase 1: groups F_221052, O_131326, F_224323, F_223815, F_222747 (3 offsets), all conditions.
Phase 2: the phase-1 best condition (rule in the plan) on the rest of the z2_regress matrix (F_214138, F_221925, T_*, O_154320).
Reused (copied, sources untouched): base/Z2 runs from runs/z2_regress (all groups), Z1/H of 224323/223815 from
runs/setdown_mitig (same args and code blobs; determinism shown in z2_regression_plan.md).
Determinism: F_221052_Z2_o1 rerun twice (DET_a, DET_b) and compared pose-by-pose with the z2_regress run.

  (setsid nohup ../venv_ffs/bin/python imu_learn/z2_isolate.py run p1 > runs/z2_isolate/sched.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/z2_isolate.py analyse            # -> runs/z2_isolate/summary.json
  (setsid nohup ../venv_ffs/bin/python imu_learn/z2_isolate.py run p2 > runs/z2_isolate/sched2.out 2>&1 &)
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
import z2_regress as zr  # noqa: E402  (same job matrix / args / analysis as the regression check)

PY = "../venv_ffs/bin/python"
OUT = Path("runs/z2_isolate")
LOG = OUT / "sched.log"
PAR = int(os.environ.get("PAR", 3))
HOLD = "1.0:0.05:0.05"
CONDS = {"base": ("", None), "Z2": ("zupt_chi2_multipler=1,zupt_noise_multiplier=1", None),
         "Z1": ("zupt_chi2_multipler=1", None), "N1": ("zupt_noise_multiplier=1", None), "H": ("", HOLD),
         "Z2c": ("zupt_chi2_multipler=0.5,zupt_noise_multiplier=1", None),
         "Z2v": ("zupt_chi2_multipler=1,zupt_noise_multiplier=1,zupt_max_velocity=0.05", None)}
P1_GROUPS = ["F_221052", "O_131326", "F_224323", "F_223815", "F_222747"]   # run order = priority (222747 longest, last)
SETDOWN = ("224323", "223815")
DET = "F_221052_Z2_o1"
FILES = zr.FILES[:3] + ("imu_learn/z2_regress.py", "imu_learn/z2_isolate.py") + zr.FILES[4:]


def log(msg):
    OUT.mkdir(parents=True, exist_ok=True)
    line = f"{time.strftime('%F %T')} z2i: {msg}"
    with open(LOG, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


def blobs():
    head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    b = {f: subprocess.run(["git", "hash-object", f], capture_output=True, text=True).stdout.strip() for f in FILES}
    dirty = subprocess.run(["git", "status", "--porcelain", *FILES], capture_output=True, text=True).stdout.strip()
    return head, b, bool(dirty)


def base_jobs():
    """z2_regress base jobs = one entry per (group, pair) with its exact bench args."""
    return [j for j in zr.jobs() if j["cond"] == "base"]


def name_of(j, cond):
    return j["name"].replace("_base_", f"_{cond}_")


def reuse_src(j, cond):
    if cond in ("base", "Z2"):
        return zr.OUT / name_of(j, cond)
    rec6 = j["rec"][-6:]
    if cond in ("Z1", "H") and j["group"] in (f"F_{r}" for r in SETDOWN):
        return Path("runs/setdown_mitig") / f"{rec6}_{cond}_o{j['pair'][-1]}"
    return None


def jobs(phase, best=None):
    bj = base_jobs()
    if phase == "p1":
        conds = list(CONDS)
        sel = sorted([j for j in bj if j["group"] in P1_GROUPS], key=lambda j: (P1_GROUPS.index(j["group"]), j["pair"]))
        out = [dict(j, cond=c, name=name_of(j, c)) for c in conds for j in sel]
        # interleave conditions per input (so a stopped queue still has every condition on the early inputs); Z2v last
        out.sort(key=lambda j: (j["cond"] == "Z2v", P1_GROUPS.index(j["group"]), j["pair"], conds.index(j["cond"])))
        return out
    if phase == "p2":
        sel = [j for j in bj if j["group"] not in P1_GROUPS]
        return [dict(j, cond=c, name=name_of(j, c)) for c in ("base", "Z2", best) for j in sel]
    raise ValueError(phase)


def cmd(j):
    over, hold = CONDS[j["cond"]]
    return [PY, "imu_learn/z2_isolate.py", "job", "--cfg-over", over, "--still-hold", hold or "", "--", *j["args"],
            "--out", str(OUT / j["name"])]


def job(argv):
    """In-process c5_run with --cfg-over / --still-hold (same mechanism as setdown_mitig.job)."""
    i = argv.index("--")
    ap = argparse.ArgumentParser()
    ap.add_argument("--cfg-over", default="")
    ap.add_argument("--still-hold", default="")
    o = ap.parse_args(argv[:i])
    import c5_run
    import vio_bench_hs as vb
    orig = vb.run

    def run(a):
        a.cfg_over = o.cfg_over
        a.still_hold = o.still_hold or None
        return orig(a)
    vb.run = run
    sys.argv = ["c5_run.py"] + argv[i + 1:]
    c5_run.main()
    out = sys.argv[sys.argv.index("--out") + 1]
    d = json.load(open(out + ".json"))
    head, b, dirty = blobs()
    d.update(z2i_runner="imu_learn/z2_isolate.py", z2_blobs=b, z2_dirty=dirty, z2_head=head, z2_cfg_over=o.cfg_over,
             z2i_still_hold=o.still_hold or None)
    json.dump(d, open(out + ".json", "w"), indent=1, ensure_ascii=False)


def reuse(js):
    OUT.mkdir(parents=True, exist_ok=True)
    for j in js:
        src = reuse_src(j, j["cond"])
        dst = OUT / j["name"]
        if src is None or Path(str(dst) + ".json").exists():
            continue
        if not Path(str(src) + ".json").exists():
            log(f"reuse source missing {src} -> will run")
            continue
        for ext in (".json", ".poses.jsonl.gz", ".stderr.log.gz"):
            shutil.copy2(str(src) + ext, str(dst) + ext)
        log(f"reused {src} -> {dst}")


def run_queue(js):
    OUT.mkdir(parents=True, exist_ok=True)
    head, b, dirty = blobs()
    reuse(js)
    js = [j for j in js if not (OUT / f"{j['name']}.json").exists()]
    log(f"start git {head} dirty={dirty} PAR {PAR} jobs {len(js)} blobs {b}")
    running = {}
    while js or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is None:
                continue
            del running[n]
            try:
                d = json.load(open(OUT / f"{n}.json"))
                tail = f"init {d.get('t_init_clip_s')} ate {d.get('ate_cm')} scale {d.get('scale')} err {d.get('error')}"
            except Exception:  # noqa: BLE001
                lines = (OUT / f"{n}.err").read_text(errors="replace").strip().splitlines()
                tail = "FAILED: " + (lines[-1][-300:] if lines else "")
            log(f"done {n} ({time.time() - t0:.0f} s, load {open('/proc/loadavg').read().split()[0]}) {tail}")
        while js and len(running) < PAR:
            j = js.pop(0)
            err = open(OUT / f"{j['name']}.err", "w")
            running[j["name"]] = (subprocess.Popen(["nice", "-n", "5"] + cmd(j), stdout=subprocess.DEVNULL, stderr=err),
                                  time.time())
            log(f"start {j['name']} ({len(js)} left)")
        time.sleep(10)
    log("end (queue done)")


def det_jobs():
    j = next(x for x in base_jobs() if x["pair"] == "F_221052_o1")
    return [dict(j, cond="Z2", name=f"DET_{k}_{DET}") for k in ("a", "b")]


# ---------------------------------------------------------------- analysis
def setdown_metrics(prefix):
    import setdown_mitig as sm
    es = json.load(open(sm.ENDSTILL))
    r = sm.analyse_one(prefix, es)
    return {k: r.get(k) for k in ("lastwin_med_cm", "drift_cm", "ate100_cm", "scale100", "v_mean_cms")}


def timings(prefix):
    """Clip seconds (from the first fed frame) of: OpenVINS 'successful initialization', first frame with a visual
    (MSCKF+SLAM) update using > 0 feats, and ZUPT accepts between init and that first visual update (verifier request:
    separate 'noise x1' from 'later tracking start')."""
    import re
    t_first, t_init, t_upd, cur, z_pre, f, zc = None, None, None, None, 0, 0, False
    for line in gzip.open(prefix + ".stderr.log.gz", "rt", errors="replace"):
        line = zr.ANSI.sub("", line)
        if "[DIAGT]" in line:
            if cur is not None and t_init is not None and t_upd is None:
                if f > 0:
                    t_upd = cur
                elif zc:
                    z_pre += 1
            cur = float(line.split("[DIAGT]")[1])
            t_first = cur if t_first is None else t_first
            f, zc = 0, False
        elif "successful initialization" in line and t_init is None:
            t_init = cur
        elif "[ZUPT]: accepted" in line:
            zc = True
        elif "for MSCKF update (" in line or "for SLAM update (" in line:
            f += int(re.search(r"\((\d+) feats\)", line)[1])
    rel = (lambda x: None if x is None or t_first is None else round(x - t_first, 3))
    return dict(t_init_log_s=rel(t_init), t_first_vis_update_s=rel(t_upd), zupt_init_to_vis=z_pre)


def poses_diff(a, b):
    def load(p):
        return {m["t_ns"]: np.asarray(m["T_world_cam0"]).reshape(4, 4)[:3, 3]
                for m in map(json.loads, gzip.open(p + ".poses.jsonl.gz", "rt"))
                if m.get("status") == "tracking" and m.get("T_world_cam0")}
    A, B = load(a), load(b)
    common = sorted(set(A) & set(B))
    mx = max((float(np.linalg.norm(A[t] - B[t])) for t in common), default=None)
    return dict(n_a=len(A), n_b=len(B), n_common=len(common), max_pos_diff_m=mx)


def judge(cells, cond):
    """z2_regress criteria 1-3 for one group, cond vs base (same code path)."""
    b, z = cells["base"], cells[cond]
    new_bad = []
    zs = dict(zip(z["pairs"], z["status"]))
    for pr, sb in zip(b["pairs"], b["status"]):
        if pr not in zs:
            continue
        sz = zs[pr]
        if sb == "ok" and sz != "ok":
            new_bad.append(f"{pr}: base ok -> {cond} {sz}")
        elif sb == "noinit" and sz in ("collapse", "unscored"):
            new_bad.append(f"{pr}: base noinit -> {cond} {sz}")
    if b["ate_med"] is None or z["ate_med"] is None:
        ate_ok, lim = None, None
    else:
        lim = max(b["ate_med"] * 1.10, b["ate_med"] + 0.2)
        ate_ok = z["ate_med"] <= lim + 1e-9
    return dict(ate_limit=lim, ate_ok=ate_ok, new_collapses=new_bad, false_acc=z["false_acc"],
                pass_=(ate_ok is not False) and not new_bad and z["false_acc"] == 0 and z["n"] == b["n"] == 3)


def summarize():
    js = jobs("p1")
    best_prev = None
    if (OUT / "summary.json").exists():
        best_prev = json.load(open(OUT / "summary.json")).get("best")
    if best_prev and best_prev != "base":
        js += [j for j in jobs("p2", best_prev) if j["name"] not in {x["name"] for x in js}]
    runs, missing = {}, []
    for j in js:
        p = OUT / j["name"]
        if not Path(str(p) + ".json").exists():
            missing.append(j["name"])
            continue
        try:
            r = zr.analyse_one(str(p))
            r.update(group=j["group"], cond=j["cond"], pair=j["pair"])
            d = json.load(open(str(p) + ".json"))
            r["frames_held"] = d.get("frames_held")
            r.update(timings(str(p)))
            r["reused_from"] = ("z2_regress" if d.get("z2_runner") else "setdown_mitig" if d.get("mitig_runner") else
                                "this run")
            if j["rec"][-6:] in SETDOWN and j["name"].startswith("F_"):
                r.update(setdown_metrics(str(p)))
            runs[j["name"]] = r
        except Exception as e:  # noqa: BLE001
            missing.append(f"{j['name']}: {e!r}")
    groups = {}
    for g in dict.fromkeys(j["group"] for j in js):
        cells = {}
        for c in CONDS:
            xs = sorted([r for r in runs.values() if r["group"] == g and r["cond"] == c], key=lambda r: r["pair"])
            if not xs:
                continue
            sc = [r["ate_cm"] for r in xs if r["status"] in ("ok", "collapse")]
            cells[c] = dict(n=len(xs), pairs=[r["pair"] for r in xs], status=[r["status"] for r in xs],
                            ates=[r["ate_cm"] for r in xs], scales=[r["scale"] for r in xs], ate_med=zr.med(sc),
                            false_acc=sum(r["false_acc"] for r in xs), false_acc_runs=[r["false_acc"] for r in xs],
                            false_acc_v=[r["false_acc_v_cms"] for r in xs],
                            imu_only=sum(r["zupt_imu_only"] for r in xs), zupt_acc=sum(r["zupt_acc"] for r in xs),
                            zupt_moving_any=sum(r["zupt_moving_any"] for r in xs),
                            lastwin_med=zr.med([r.get("lastwin_med_cm") for r in xs]),
                            drift_med=zr.med([r.get("drift_cm") for r in xs]),
                            held=[r.get("frames_held") for r in xs], src=sorted({r["reused_from"] for r in xs}),
                            t_init_log=[r.get("t_init_log_s") for r in xs], t_track=[r.get("t_init_clip_s") for r in xs],
                            t_vis=[r.get("t_first_vis_update_s") for r in xs],
                            zupt_pre=[r.get("zupt_init_to_vis") for r in xs])
        judged = {c: judge(cells, c) for c in cells if c != "base" and "base" in cells}
        for c, jd in judged.items():                  # pairwise worst case (verifier request), not only medians
            ba = dict(zip(cells["base"]["pairs"], cells["base"]["ates"]))
            tri = [(pr, ba.get(pr), z) for pr, z in zip(cells[c]["pairs"], cells[c]["ates"])
                   if ba.get(pr) is not None and z is not None]
            dd = [(pr, round(z - b, 2)) for pr, b, z in tri]
            jd["worst_pair"] = max(dd, key=lambda x: x[1]) if dd else None
            jd["pair_worse_than_limit"] = [pr for pr, b, z in tri if z > max(b * 1.10, b + 0.2) + 1e-9]
        groups[g] = dict(cells=cells, judged=judged)
    # set-down help (setdown_mitig rule): 224323 last-window median <= 50 % base AND end-still drift median <= 50 % base
    sd = groups.get("F_224323", {}).get("cells", {})
    helps = {}
    for c in CONDS:
        if c == "base" or c not in sd or sd["base"]["lastwin_med"] is None or sd[c]["lastwin_med"] is None:
            continue
        helps[c] = (sd[c]["lastwin_med"] <= 0.5 * sd["base"]["lastwin_med"]
                    and sd[c]["drift_med"] <= 0.5 * sd["base"]["drift_med"])
    p1 = {}
    for c in CONDS:
        if c == "base":
            continue
        js1 = [groups[g]["judged"].get(c) for g in P1_GROUPS if g in groups]
        if len(js1) < len(P1_GROUPS) or any(x is None for x in js1):
            continue
        p1[c] = dict(n_fail=sum(not x["pass_"] for x in js1), n_new=sum(len(x["new_collapses"]) for x in js1),
                     n_fa=sum(x["false_acc"] for x in js1), helps=helps.get(c),
                     lastwin_224323=sd.get(c, {}).get("lastwin_med"),
                     fails=[g for g in P1_GROUPS if not groups[g]["judged"][c]["pass_"]])
    cand = [c for c in p1 if p1[c]["helps"]] or list(p1)
    best = min(cand, key=lambda c: (p1[c]["n_fail"], p1[c]["n_new"], p1[c]["n_fa"], p1[c]["lastwin_224323"] or 1e9)) \
        if cand else None
    allg = [g for g in groups if g not in P1_GROUPS]
    full = None
    if best and allg and all(best in groups[g]["judged"] for g in allg):
        fails = [g for g in groups if not groups[g]["judged"][best]["pass_"]]
        full = dict(cond=best, n_groups=len(groups), fails=fails, all_pass=not fails,
                    crit1=[g for g in groups if groups[g]["judged"][best]["ate_ok"] is False],
                    crit2=[g for g in groups if groups[g]["judged"][best]["new_collapses"]],
                    crit3=[g for g in groups if groups[g]["judged"][best]["false_acc"]])
    det = None
    if all(Path(str(OUT / j["name"]) + ".json").exists() for j in det_jobs()):
        ref = str(zr.OUT / DET)
        det = {j["name"]: dict(poses_diff(ref, str(OUT / j["name"])),
                               ate=json.load(open(str(OUT / j["name"]) + ".json")).get("ate_cm"),
                               scale=json.load(open(str(OUT / j["name"]) + ".json")).get("scale"))
               for j in det_jobs()}
        det["ref"] = dict(name=ref, ate=json.load(open(ref + ".json")).get("ate_cm"))
    head, b, dirty = blobs()
    vers = {}
    for r in runs.values():
        vers.setdefault(json.dumps({k: v for k, v in r["version"].items() if k != "dirty"}, sort_keys=True), []).append(
            r["name"])
    zc = {}
    for r in runs.values():
        zc.setdefault(r["cond"], set()).add(json.dumps(r["zupt_cfg"], sort_keys=True))
    return dict(made=time.strftime("%F %T"), git=head, blobs=b, dirty=dirty, n_runs=len(runs), missing=missing,
                code_versions=len(vers), version_counts=list(map(len, vers.values())),
                zupt_cfgs={k: sorted(v) for k, v in zc.items()}, helps=helps, phase1=p1, best=best, full=full,
                det=det, groups=groups, runs=list(runs.values()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["run", "job", "analyse", "list"])
    o, _ = ap.parse_known_args(sys.argv[1:2])
    if o.cmd == "job":
        return job(sys.argv[2:])
    if o.cmd in ("run", "list"):
        ph = sys.argv[2]
        if ph == "p1":
            js = jobs("p1")
        elif ph == "det":
            js = det_jobs()
        else:
            best = json.load(open(OUT / "summary.json"))["best"]
            js = jobs("p2", best)
        if o.cmd == "list":
            for j in js:
                print(j["name"], " ".join(cmd(j)), "| reuse", reuse_src(j, j["cond"]))
            return
        return run_queue(js)
    res = summarize()
    OUT.mkdir(parents=True, exist_ok=True)
    json.dump(res, open(OUT / "summary.json", "w"), indent=1, ensure_ascii=False, default=float)
    print(json.dumps({k: res[k] for k in ("n_runs", "missing", "code_versions", "zupt_cfgs", "helps", "phase1", "best",
                                          "full", "det")}, indent=1, ensure_ascii=False, default=float))
    for g, v in res["groups"].items():
        for c, x in v["cells"].items():
            jj = v["judged"].get(c, {})
            print(f"{g:16s} {c:5s} {x['status']} ate {x['ates']} med {x['ate_med']} FA {x['false_acc_runs']} "
                  f"lw {x['lastwin_med']} dr {x['drift_med']} ti {x['t_init_log']} tv {x['t_vis']} tt {x['t_track']} PASS {jj.get('pass_')} {jj.get('new_collapses', '')}")


if __name__ == "__main__":
    main()
