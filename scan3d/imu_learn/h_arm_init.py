#!/usr/bin/env python3
"""Still-hold armed only after OpenVINS initialization (docs/h_arm_init_plan.md, 2026-10-21).
--still-hold 1.5:0.05:0.05:iD (D = 0 / 3 / 10 s after the bridge's first tracking pose) vs base and the reference
Hw15m05 (1.5:0.05:0.05:m0.5), all re-run here with one code version and OpenVINS num_opencv_threads 2 (base too).
Same 54 bench jobs (18 groups x 3 offsets) as imu_learn/z2_regress.py.

  ../venv_ffs/bin/python imu_learn/h_arm_init.py mask          # predicted held frames (no VIO; init time from base)
  ../venv_ffs/bin/python imu_learn/h_arm_init.py copyplan
  (setsid nohup ../venv_ffs/bin/python imu_learn/h_arm_init.py run stage1 > runs/h_arm_init/sched_stage1.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/h_arm_init.py analyse
"""
import argparse
import csv
import fcntl
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
import z2_regress as zr  # noqa: E402
import h_tune as ht  # noqa: E402  (held_motion / cell_of / predict lines reused, its OUT is not written)

PY = "../venv_ffs/bin/python"
OUT = Path("runs/h_arm_init")
LOG = OUT / "sched.log"
PAR = int(os.environ.get("PAR", 5))
THREADS = "2"
VARIANTS = {"Hw15m05": "1.5:0.05:0.05:m0.5", "Hw15i10": "1.5:0.05:0.05:i10", "Hw15i3": "1.5:0.05:0.05:i3",
            "Hw15i0": "1.5:0.05:0.05:i0"}
CANDIDATES = ["Hw15i0", "Hw15i3", "Hw15i10"]
FILES = ht.FILES + ("imu_learn/h_arm_init.py",)
SHARE_MAX = 0.005            # ORIGINAL pre-registered 3': share of held GT samples > 2 cm/s <= 0.5 %
VMAX_CMS = 3.0               #                           and max held GT speed <= 3 cm/s


def log(msg):
    OUT.mkdir(parents=True, exist_ok=True)
    line = f"{time.strftime('%F %T')} hai: {msg}"
    with open(LOG, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


def blobs():
    head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    b = {f: subprocess.run(["git", "hash-object", f], capture_output=True, text=True).stdout.strip() for f in FILES}
    dirty = subprocess.run(["git", "status", "--porcelain", *FILES], capture_output=True, text=True).stdout.strip()
    return head, b, bool(dirty)


def base_jobs():
    return ht.base_jobs()


def coarse_pair(pr):
    return pr.endswith("_o0") or pr.endswith("_d0")


def mkjob(pair, var):
    j = base_jobs()[pair]
    return dict(j, cond=var, name=ht.name(pair, var), hold=VARIANTS.get(var, ""), threads=THREADS)


def init_ns(pair):
    """First tracking pose of the base run (new run here if present, else the stored z2_regress base: same input
    frames before arming, checked again after the new base runs). None = never initialized."""
    for d in (OUT, zr.OUT):
        f = Path(str(d / ht.name(pair, "base")) + ".poses.jsonl.gz")
        if f.exists():
            for line in gzip.open(f, "rt"):
                m = json.loads(line)
                if m.get("type") == "pose" and m.get("status") == "tracking":
                    return int(m["t_ns"]), str(d)
            return None, str(d)
    raise FileNotFoundError(pair)


def predict(j, spec, t_init):
    """ht.predict with the iD arming: same selection lines and the bench's StillHold; init_ns set up front (the
    bench learns it from the pose of the frame before the decision; step() only compares it with later frames)."""
    import vio_bench_hs as vb
    from imu_gt import read_imu
    a = j["args"]
    rec = SCAN3D / a[0]
    off = float(a[a.index("--start-off") + 1])
    max_secs = float(a[a.index("--max-secs") + 1])
    meta = json.load(open(rec / "meta.json"))
    t_frames = np.array([int(r["t_mid_ns"]) for r in csv.DictReader(open(rec / "frames_cam5.csv"))])
    dt_ns = int(json.load(open(rec / "imu_gt" / "align.json"))["dt_ns"])
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
    if sh.arm_init is not None:
        sh.init_ns = t_init
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
                t_init_clip_s=None if t_init is None else round((t_init + dt_ns - t_start) / 1e9, 3))


def mask():
    res = {}
    for pr, j in base_jobs().items():
        ti, src = init_ns(pr)
        res[pr] = {v: predict(j, s, ti) for v, s in VARIANTS.items()}
        res[pr]["_init"] = dict(init_ns=ti, src=src)
        print(pr, src, {v: (x["held"], x["segs"][:3]) for v, x in res[pr].items() if v[0] != "_"}, flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    json.dump(dict(made=time.strftime("%F %T"), variants=VARIANTS, blobs=blobs()[1], runs=res),
              open(OUT / "mask.json", "w"), indent=1)


def make_copyplan():
    """Copy rule (plan): same code version and threads here, so a (pair, variant) whose predicted input frames equal
    an executed run's input frames reuses it: held 0 -> this folder's base; same held segments as an earlier variant
    of VARIANTS order -> that variant."""
    mk = json.load(open(OUT / "mask.json"))["runs"]
    plan = {}
    for pr, x in mk.items():
        order = list(VARIANTS)
        for i, v in enumerate(order):
            if x[v]["held"] == 0:
                plan.setdefault(v, {})[pr] = "base"
                continue
            for u in order[:i]:
                if x[u]["held"] == x[v]["held"] and x[u]["segs"] == x[v]["segs"] and plan.get(u, {}).get(pr) is None:
                    plan.setdefault(v, {})[pr] = u
                    break
    json.dump(plan, open(OUT / "copyplan.json", "w"), indent=1)
    return plan


# ---------------------------------------------------------------- runs (global PAR over every scheduler of this folder)
def cmd(j):
    return [PY, "imu_learn/z2_isolate.py", "job", "--cfg-over", f"num_opencv_threads={j['threads']}",
            "--still-hold", j["hold"], "--", *j["args"], "--out", str(OUT / j["name"])]


DUR = {"O_": 1270, "F_214138": 690, "F_222747": 620, "F_221052": 500, "F_221925": 450}


def dur(j):
    return next((s for k, s in DUR.items() if j["pair"].startswith(k)), 380 if j["pair"].startswith("F_") else 190)


def stage_jobs(st):
    cp = json.load(open(OUT / "copyplan.json"))
    bj = base_jobs()
    if st == "stage1":   # coarse cells (base + every variant not copied) + conf, then every other cell (plan: the
        #                  3-offset pass is queued for all variants because copies leave only Hw15i0/Hw15m05 runs)
        co = [mkjob(pr, "base") for pr in bj if coarse_pair(pr)]
        co += [mkjob(pr, v) for v in VARIANTS for pr in bj if coarse_pair(pr) and pr not in cp.get(v, {})]
        conf = [dict(mkjob(pr, v), name="CONF_" + ht.name(pr, v)) for pr, v, _ in CONF]
        rest = [mkjob(pr, "base") for pr in bj if not coarse_pair(pr)]
        rest += [mkjob(pr, v) for v in VARIANTS for pr in bj if not coarse_pair(pr) and pr not in cp.get(v, {})]
        return sorted(co + conf, key=lambda j: -dur(j)) + sorted(rest, key=lambda j: -dur(j))
    if st.startswith("fine:"):
        js = [mkjob(pr, v) for v in st[5:].split(",") for pr in bj if not coarse_pair(pr) and pr not in cp.get(v, {})]
        return sorted(js, key=lambda j: -dur(j))
    raise ValueError(st)


# copy-rule checks (plan, from mask.json): (pair, variant, run it must equal pose by pose)
CONF = [("T_221052_S1_d0", "Hw15i10", "base"), ("F_224323_o0", "Hw15i10", "Hw15m05")]


def n_running():
    r = subprocess.run(["pgrep", "-fc", f"z2_isolate.py job .*--out {OUT}/"], capture_output=True, text=True)
    return int(r.stdout.strip() or 0)


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
                tail = (f"init {d.get('t_init_clip_s')} ate {d.get('ate_cm')} held {d.get('frames_held')} "
                        f"waitTO {d.get('hold_wait_timeouts')} err {d.get('error')}")
            except Exception:  # noqa: BLE001
                lines = (OUT / f"{n}.err").read_text(errors="replace").strip().splitlines()
                tail = "FAILED: " + (lines[-1][-300:] if lines else "")
            log(f"done {n} ({time.time() - t0:.0f} s, load {open('/proc/loadavg').read().split()[0]}) {tail}")
        if js:
            with open(OUT / ".sched.lock", "w") as lk:
                fcntl.flock(lk, fcntl.LOCK_EX)
                while js and n_running() < PAR:
                    j = js.pop(0)
                    if (OUT / f"{j['name']}.json").exists() or (OUT / f"{j['name']}.err").exists():
                        continue
                    err = open(OUT / f"{j['name']}.err", "w")
                    running[j["name"]] = (subprocess.Popen(["nice", "-n", "5"] + cmd(j), stdout=subprocess.DEVNULL,
                                                           stderr=err), time.time())
                    log(f"start {j['name']} ({len(js)} left) {j['hold'] or 'base'}")
                    time.sleep(0.5)
        time.sleep(5)
    log("end (queue done)")


# ---------------------------------------------------------------- analysis
def src_of(pr, var, cp):
    p = OUT / ht.name(pr, var)
    if Path(str(p) + ".json").exists():
        return p, "this run"
    c = cp.get(var, {}).get(pr)
    if c:
        q = OUT / ht.name(pr, c)
        if Path(str(q) + ".json").exists():
            return q, f"copy of {c} (same input)"
    return None, None


def analyse():
    bj = base_jobs()
    cp = json.load(open(OUT / "copyplan.json"))
    mk = json.load(open(OUT / "mask.json"))["runs"]
    runs = {}
    for pr, j in bj.items():
        for v in ["base"] + list(VARIANTS):
            p, src = src_of(pr, v, cp)
            if p is None:
                continue
            r = ht.cell_of(p, dict(j, cond=v))
            r["src"] = src
            r["pred_held"] = mk[pr][v]["held"] if v in VARIANTS else 0
            r["pred_segs"] = mk[pr][v]["segs"] if v in VARIANTS else []
            d = json.load(open(str(p) + ".json"))
            r["threads"] = d.get("cfg_over")          # config_used lists only init/zupt keys; the override is here
            if src == "this run":
                r.update(hold_wait_timeouts=d.get("hold_wait_timeouts"), hold_init_ns=d.get("hold_init_ns"))
            runs[(pr, v)] = r

    def group_verdict(v, pairs):
        groups = {}
        for g in dict.fromkeys(bj[pr]["group"] for pr in pairs):
            ps = [pr for pr in pairs if bj[pr]["group"] == g]
            b = [runs.get((pr, "base")) for pr in ps]
            z = [runs.get((pr, v)) for pr in ps]
            if any(x is None for x in b + z):
                continue
            bsc = [r["ate_cm"] for r in b if r["status"] in ("ok", "collapse")]
            zsc = [r["ate_cm"] for r in z if r["status"] in ("ok", "collapse")]
            bm, zm = zr.med(bsc), zr.med(zsc)
            lim = None if bm is None or zm is None else max(bm * 1.10, bm + 0.2)
            ate_ok = None if lim is None else zm <= lim + 1e-9
            new_bad = [f"{rb['pair']}: {rb['status']} -> {rz['status']}" for rb, rz in zip(b, z)
                       if (rb["status"] == "ok" and rz["status"] != "ok")
                       or (rb["status"] == "noinit" and rz["status"] in ("collapse", "unscored"))]
            fa = sum(r["false_acc"] for r in z)
            held = sum(r.get("frames_held") or 0 for r in z)
            n_gt = sum(r["held_gt_n"] for r in z)
            fast = sum(r["held_gt_fast"] for r in z)
            vmax = max([r["held_gt_vmax_cms"] or 0 for r in z])
            untested = [r["pair"] for r in z if (r.get("frames_held") or 0) > 0 and r["held_gt_n"] == 0]
            if held == 0:
                c3h = "n/a (0 held)"
            elif n_gt == 0:
                c3h = "not tested"
            else:
                c3h = "pass" if (fast / n_gt <= SHARE_MAX + 1e-12 and vmax <= VMAX_CMS + 1e-9) else "fail"
            c3h_amended = "n/a (0 held)" if held == 0 else ("not tested" if n_gt == 0 else
                                                           ("pass" if vmax <= VMAX_CMS + 1e-9 else "fail"))
            c123 = (ate_ok is not False) and not new_bad and fa == 0
            orig = "fail" if (not c123 or c3h == "fail") else ("not tested" if c3h == "not tested" else "pass")
            groups[g] = dict(n=len(ps), pairs=ps, base_ates=[r["ate_cm"] for r in b], ates=[r["ate_cm"] for r in z],
                             base_status=[r["status"] for r in b], status=[r["status"] for r in z],
                             base_med=bm, med=zm, lim=lim, ate_ok=ate_ok, new_bad=new_bad, false_acc=fa,
                             held=[r.get("frames_held") for r in z], held_gt_n=n_gt, held_gt_fast=fast,
                             held_share=None if not n_gt else round(fast / n_gt, 5), held_vmax_cms=vmax,
                             untested_pairs=untested, c3h=c3h, c3h_amended=c3h_amended, verdict_orig=orig,
                             verdict_amended="fail" if (not c123 or c3h_amended == "fail") else
                             ("not tested" if c3h_amended == "not tested" else "pass"),
                             src=[r["src"] for r in z],
                             pair_over=[r["pair"] for r, rb in zip(z, b) if r["ate_cm"] is not None and rb["ate_cm"]
                                        is not None and r["ate_cm"] > max(rb["ate_cm"] * 1.1, rb["ate_cm"] + 0.2) + 1e-9])
        sd = [runs.get((pr, x)) for pr in pairs if bj[pr]["group"] == "F_224323" for x in ("base", v)]
        helps = None
        if sd and all(sd):
            bb, vv = sd[0::2], sd[1::2]
            lw_b, lw_v = zr.med([r.get("lastwin_med_cm") for r in bb]), zr.med([r.get("lastwin_med_cm") for r in vv])
            dr_b, dr_v = zr.med([r.get("drift_cm") for r in bb]), zr.med([r.get("drift_cm") for r in vv])
            helps = dict(ok=bool(lw_v <= 0.5 * lw_b and dr_v <= 0.5 * dr_b), lastwin=(lw_b, lw_v), drift=(dr_b, dr_v))
        return dict(groups=groups, setdown=helps, n_groups=len(groups),
                    orig_fail=[g for g, x in groups.items() if x["verdict_orig"] == "fail"],
                    orig_untested=[g for g, x in groups.items() if x["verdict_orig"] == "not tested"],
                    amended_fail=[g for g, x in groups.items() if x["verdict_amended"] == "fail"],
                    pair_over=sum(len(x["pair_over"]) for x in groups.values()),
                    new_bad=sum(len(x["new_bad"]) for x in groups.values()),
                    c3h_fail=[g for g, x in groups.items() if x["c3h"] == "fail"])

    coarse_pairs = [pr for pr in bj if coarse_pair(pr)]
    coarse = {v: group_verdict(v, coarse_pairs) for v in VARIANTS}
    full = {v: group_verdict(v, list(bj)) for v in VARIANTS}
    full = {v: x for v, x in full.items() if x["n_groups"] == 18}
    # checks: prediction == executed held, init time of new base == stored base, conf copy run == base
    pred_bad = {f"{pr}/{v}": (r["pred_held"], r.get("frames_held")) for (pr, v), r in runs.items()
                if v in VARIANTS and r["src"] == "this run" and r["pred_held"] != r.get("frames_held")}
    init_chk = {}
    for pr in bj:
        a, b = (Path(str(d / ht.name(pr, "base")) + ".poses.jsonl.gz") for d in (OUT, zr.OUT))
        if a.exists():
            init_chk[pr] = init_ns(pr)[0] == mk[pr]["_init"]["init_ns"]
    conf = {}
    import z2_isolate as zi
    for pr, v, ref in CONF:
        p, q = OUT / ("CONF_" + ht.name(pr, v)), OUT / ht.name(pr, ref)
        if Path(str(p) + ".json").exists() and Path(str(q) + ".json").exists():
            d = json.load(open(str(p) + ".json"))
            conf[f"{pr}/{v} vs {ref}"] = dict(zi.poses_diff(str(q), str(p)), held=d.get("frames_held"),
                                              hold_init_ns=d.get("hold_init_ns"), base_init_ns=init_ns(pr)[0],
                                              wait_timeouts=d.get("hold_wait_timeouts"))
    timeouts = {f"{pr}/{v}": r.get("hold_wait_timeouts") for (pr, v), r in runs.items() if r.get("hold_wait_timeouts")}
    vers = {}
    for r in runs.values():
        vers.setdefault(json.dumps({k: x for k, x in r["version"].items() if k != "dirty"}, sort_keys=True), []).append(
            r["name"])
    return dict(made=time.strftime("%F %T"), git=blobs()[0], blobs=blobs()[1], pred_checked=sum(
        1 for (pr, v), r in runs.items() if v in VARIANTS and r["src"] == "this run"), pred_mismatch=pred_bad,
        init_same=init_chk, conf=conf, wait_timeouts=timeouts, code_versions=len(vers),
        threads=sorted({str(r.get("threads")) for r in runs.values()}), coarse=coarse, full=full,
        runs=[dict(r, key=f"{k[0]}/{k[1]}") for k, r in runs.items()])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["mask", "copyplan", "run", "list", "analyse"])
    ap.add_argument("stage", nargs="?")
    o = ap.parse_args()
    if o.cmd == "mask":
        return mask()
    if o.cmd == "copyplan":
        pl = make_copyplan()
        print({v: {c: sum(1 for x in pl.get(v, {}).values() if x == c) for c in ["base"] + list(VARIANTS)}
               for v in VARIANTS})
        return
    if o.cmd in ("run", "list"):
        js = stage_jobs(o.stage)
        if o.cmd == "list":
            tot = sum(dur(j) for j in js)
            for j in js:
                print(j["name"], j["hold"] or "base", dur(j))
            print(len(js), "jobs, est", round(tot / PAR / 60), "min at PAR", PAR)
            return
        return run_queue(js)
    res = analyse()
    json.dump(res, open(OUT / "summary.json", "w"), indent=1, ensure_ascii=False, default=float)
    print(json.dumps({k: res[k] for k in ("pred_checked", "pred_mismatch", "conf", "wait_timeouts", "code_versions",
                                          "threads")}, default=float))
    print("init same:", sum(res["init_same"].values()), "/", len(res["init_same"]))
    for lab, part in (("COARSE", res["coarse"]), ("FULL", res["full"])):
        for v, x in part.items():
            print(f"{lab} {v}: groups {x['n_groups']} orig_fail {x['orig_fail']} untested {x['orig_untested']} "
                  f"amended_fail {x['amended_fail']} pair_over {x['pair_over']} new_bad {x['new_bad']} setdown {x['setdown']}")
            for g, gg in x["groups"].items():
                print(f"   {g:14s} {gg['base_ates']} -> {gg['ates']} lim {gg['lim']} held {gg['held']} "
                      f"3' {gg['c3h']} ({gg['held_gt_fast']}/{gg['held_gt_n']} vmax {gg['held_vmax_cms']}) "
                      f"orig {gg['verdict_orig']} {gg['new_bad']} {gg['src']}")


if __name__ == "__main__":
    main()
