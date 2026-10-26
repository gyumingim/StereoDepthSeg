#!/usr/bin/env python3
"""ovswba 2026-10-26: VINS-Mono sliding-window estimator (--swba 1) next to OpenVINS, scored with the SAME ruler as the
OpenVINS GL60 reference (runs/dpvo_20261026/ov_ref_run.py = runs/integ2_20261026/integ2_ate.py = ov60eval protocol):
960x540, pts 200, clones 15, init window 1.0 s / 22.5 px, cfg_over "num_opencv_threads=1,try_zupt=false,
fi_max_cond_number=1e6,fi_max_baseline=200" + histogram_method "NONE" (live), lever, frame cache, strict init, per-recording
R_ci / dt, 60 fps, bridge "--profile rec60 --drop-late 0" (profile = gyro_klt=5 etc.). Only changes: the bridge is this
branch's build and "--loop 0" (integ2 0.6.28 put --loop 1 into rec60; the GL60 reference build had no loop code; --loop
never changes the VIO fields) + the condition's --swba options.

Conditions:
  Z60      : --swba 0 (identity: VIO fields must equal the GL60 reference runs in REF)
  W60      : --swba 1 --swba-sync 1, noise 1 (VINS EuRoC values), solver_time 1.0 s (iteration-bounded = deterministic)
  W60n2    : same with noise 2 (OpenVINS densities * sqrt(rate))
Each W run gives three trajectories scored with the same functions:
  vio  : the OpenVINS pose lines (must equal Z60 / GL60)
  swba : the corrected pose ("swba" object = Rz(yaw) p_vio + t, rate limited) at every frame (same window as vio)
  raw  : the VINS latest-frame pose (aligned to the OpenVINS world once) at the VINS frames only (10 Hz), compared with
         vio sub-sampled at the same frames (rawvio)
  python3 swba_run.py freeze
  python3 swba_run.py run "hs:W60;std:W60;loop:W60" [--par 5]
  python3 swba_run.py score [conds]
OUT = ~/scanner/tmp_ovswba/ate (env SWBA_OUT).
"""
import argparse
import copy
import gzip
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

MAIN = Path("/home/system/scanner/StereoDepthSeg/scan3d")
WT = Path("/home/system/scanner/StereoDepthSeg-ovswba/scan3d")
RD = WT / "runs" / "ovswba_20261026"
OUT = Path(os.environ.get("SWBA_OUT", "/home/system/scanner/tmp_ovswba/ate"))
SRC = WT / "ov_bridge" / "build_ovswba_full" / "ov_bridge"
REF = Path("/home/system/scanner/tmp_dpvo/ov/GL60")   # OpenVINS GL60 reference runs (dpvo ov_ref_run.py, bridge c640f534)
PY = str(MAIN.parent / "venv_ffs" / "bin" / "python")
sys.path.insert(0, str(MAIN / "imu_learn"))
sys.path.insert(0, str(MAIN))
os.chdir(MAIN)

GT = "imu_gt_spl_cal1022"
STD_GT = "imu_gt_spl"
OFFS = (0, 1, 2)
LEVER = "config/imu_cam_lever.json"
BASE_CFG = "num_opencv_threads=1,try_zupt=false,fi_max_cond_number=1e6,fi_max_baseline=200"
LIVE = 'histogram_method="NONE"'
STD = ["hs_20261024_165506_std60", "hs_20261024_173432_std60"]
LOOP = ["hs_20261024_142148_std60", "hs_20261024_142610_std60", "hs_20261024_151646_std60"]
P60 = "--profile rec60 --drop-late 0 --loop 0"
SW = "--swba 1 --swba-sync 1"
CONDS = {
    "Z60": dict(fps=60.0, args=P60),
    "W60": dict(fps=60.0, args=P60 + " " + SW, swcfg="noise=1,solver_time=1.0"),
    "W60n2": dict(fps=60.0, args=P60 + " " + SW, swcfg="noise=2,solver_time=1.0"),
}


def hs_cells():
    from phone60_run import RAW, OLD, DROP  # noqa: E402
    return [(r, o) for r in RAW + OLD for o in OFFS], DROP, set(OLD)


def cells(which):
    if which == "hs":
        return hs_cells()[0]
    if which == "std":
        return [(r, o) for r in STD for o in OFFS]
    if which == "loop":
        return [(r, 0) for r in LOOP]
    if which == "dev":   # pre-declared noise-choice cells (PLAN.md)
        return [("hs_20261022_133820", 0), ("hs_20261020_224323", 0)]
    raise ValueError(which)


def key_of(rec, off, which):
    hh = rec.split("_")[2]
    return {"hs": f"{hh}_o{off}", "dev": f"{hh}_o{off}", "std": f"s{hh}_o{off}", "loop": f"l{hh}_o{off}"}[which]


def which_of(key):
    return {"s": "std", "l": "loop"}.get(key[0], "hs")


def freeze():
    (OUT / "bin").mkdir(parents=True, exist_ok=True)
    h = hashlib.sha256(open(SRC, "rb").read()).hexdigest()
    dst = OUT / "bin" / f"ov_bridge_{h[:12]}"
    if not dst.exists():
        shutil.copy2(SRC, dst)
    git = subprocess.run(["git", "-C", str(WT), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(["git", "-C", str(WT), "status", "--short", "ov_bridge"], capture_output=True, text=True).stdout
    json.dump(dict(src=str(SRC), sha256=h, copy=str(dst), frozen=time.strftime("%F %T"), worktree_head=git,
                   ov_bridge_dirty=[x for x in dirty.splitlines() if "build_" not in x]),
              open(OUT / "bin" / "bridge.json", "w"), indent=1)
    w = OUT / "bin" / "br.sh"   # condition / per-job options come from the environment (set by job())
    w.write_text(f"#!/bin/bash\nexec {dst} \"$@\" $SWBA_ARGS\n")
    w.chmod(0o755)
    print("frozen", dst, h)


def job(name):
    import vio_bench_hs as vb
    key6, o, cond = name.split("_")
    which = which_of(key6)
    rec = next(r for r, oo in cells(which) if r.split("_")[2] == key6.lstrip("sl"))
    off = int(o[1:])
    _, _, old = hs_cells()
    d = CONDS[cond]
    od = OUT / cond
    od.mkdir(parents=True, exist_ok=True)
    args = d["args"]
    if "--swba 1" in args:
        args += f" --swba-log {od / name}.swba.jsonl"
        if d.get("swcfg"):
            args += f" --swba-cfg {d['swcfg']}"
    os.environ["SWBA_ARGS"] = args
    vb.BRIDGE = OUT / "bin" / "br.sh"
    vb.TEMPLATE = MAIN / "config" / "mono" / "estimator_config.yaml"
    import tempfile as _tf
    import types
    (OUT / "tmp").mkdir(parents=True, exist_ok=True)
    vb.tempfile = types.SimpleNamespace(mkdtemp=lambda prefix="vio_", dir=None: _tf.mkdtemp(prefix=prefix, dir=str(OUT / "tmp")))
    cfg = BASE_CFG + "," + LIVE
    a = argparse.Namespace(rec=f"recordings/{rec}", fps=d["fps"], size="960x540", npts=200, clones=15,
                           noise="cfg", noise_scale=1.0, imu_calib=None, lever=LEVER, lens_file=None, dropout="",
                           max_secs=0.0, imu_raw=rec not in old, gt="none" if which == "loop" else "auto", out=None,
                           init_cfg="strict", start_off=float(off), decode_threads=2, imu_lead=2.0, init_window=1.0,
                           init_disp=22.5, frame_cache=True, gyro_rebias=False, cfg_over=cfg, still_hold=None)
    saved = {}
    orig = vb.evaluate

    def ev(rec_, a_, poses, *rest):
        saved["poses"] = poses
        saved["rest"] = rest
        return orig(rec_, a_, poses, *rest)
    vb.evaluate = ev
    t0 = time.time()
    try:
        r = vb.run(a)
    finally:
        vb.evaluate = orig
    br = json.load(open(OUT / "bin" / "bridge.json"))
    r.update(sw_made=time.strftime("%F %T"), sw_job_s=round(time.time() - t0, 1), sw_runner=str(Path(__file__)), sw_cond=cond,
             sw_args=args, sw_set=which, sw_bridge_sha256=br["sha256"], sw_worktree_head=br["worktree_head"],
             sw_load=list(os.getloadavg()))
    if saved.get("poses") is not None:
        with gzip.open(str(od / name) + ".poses.jsonl.gz", "wt") as f:
            for m in saved["poses"]:
                f.write(json.dumps(m) + "\n")
    json.dump(r, open(od / f"{name}.json", "w"), indent=1, ensure_ascii=False)


def run_sched(spec, par):
    q = []
    for part in spec.split(";"):
        which, conds = part.split(":")
        q += [f"{key_of(r, o, which)}_{c}" for c in conds.split(",") for r, o in cells(which)]
    q = [n for n in q if not (OUT / n.split("_")[2] / f"{n}.json").exists()]
    running, t_all = {}, time.time()
    print(f"{time.strftime('%F %T')} start {spec} {len(q)} jobs PAR {par}", flush=True)
    while q or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is not None:
                del running[n]
                ok = (OUT / n.split("_")[2] / f"{n}.json").exists()
                print(f"{time.strftime('%F %T')} done {n} {time.time() - t0:.0f} s {'ok' if ok else 'FAILED'}", flush=True)
        while q and len(running) < par:
            n = q.pop(0)
            (OUT / n.split("_")[2]).mkdir(parents=True, exist_ok=True)
            err = open(OUT / n.split("_")[2] / f"{n}.err", "w")
            running[n] = (subprocess.Popen(["nice", "-n", "5", PY, str(Path(__file__).resolve()), "job", n],
                                           stdout=subprocess.DEVNULL, stderr=err), time.time())
        time.sleep(2)
    print(f"{time.strftime('%F %T')} end {spec} wall {time.time() - t_all:.0f} s", flush=True)


# ------------------------------------------------------------------ scoring
VOLATILE = ("ms", "remap_ms", "wait_ms", "map", "swba")


def load_P(pre):
    f = Path(str(pre) + ".poses.jsonl.gz")
    return [json.loads(x) for x in gzip.open(f, "rt") if x.strip()] if f.exists() else []


def vio_hash(P):
    h = hashlib.sha256()
    for m in P:
        h.update(json.dumps({k: v for k, v in m.items() if k not in VOLATILE}, sort_keys=True).encode())
    return h.hexdigest()[:16]


def quat_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def swapped(P, kind, swlog):
    """pose list for kind 'swba' (corrected pose at every frame; frames before the first VINS solve = VIO pose, as the
    published correction is not valid yet), 'raw' (VINS aligned pose at the VINS frames only) or 'rawvio' (VIO pose at the
    same frames)."""
    out = []
    if kind == "swba":
        for m in P:
            s = m.get("swba")
            if s and s.get("valid") and m.get("T_world_cam0") is not None:
                m2 = dict(m)
                for k in ("p", "q", "v", "T_world_cam0"):
                    m2[k] = s[k]
                out.append(m2)
            else:
                out.append(m)
        return out
    fr = {}
    for x in open(swlog):
        j = json.loads(x)
        if j.get("type") == "frame" and "p_al" in j:
            fr[int(j["t_ns"])] = j
    for m in P:
        j = fr.get(int(m.get("t_ns", -1)))
        if j is None or m.get("T_world_cam0") is None or m.get("status") != "tracking":
            continue
        if kind == "raw":
            R_ItoC = np.array(m["R_ItoC0"]).reshape(3, 3)
            p_IinC = np.array(m["p_IinC0"])
            R = quat_R(j["q_al"])
            p = np.array(j["p_al"])
            R_CtoG = R @ R_ItoC.T
            T = np.eye(4)
            T[:3, :3] = R_CtoG
            T[:3, 3] = p - R_CtoG @ p_IinC
            m2 = dict(m)
            m2["p"] = list(p)
            m2["T_world_cam0"] = T.tolist()
            out.append(m2)
        else:
            out.append(m)
    return out


def write_variant(pre, kind, dst_dir):
    """writes <dst_dir>/<name>.json + .poses.jsonl.gz for the variant; returns the new prefix (None if no swba log)."""
    swlog = Path(str(pre) + ".swba.jsonl")
    if not swlog.exists():
        return None
    P = load_P(pre)
    Q = swapped(P, kind, swlog)
    dst_dir.mkdir(parents=True, exist_ok=True)
    dst = dst_dir / Path(str(pre)).name
    r = json.load(open(str(pre) + ".json"))
    if r.get("gt") == "none":
        tr = [m for m in Q if m.get("status") == "tracking" and m.get("T_world_cam0")]
        r["traj"] = [[int(m["t_ns"])] + [float(v) for v in np.asarray(m["T_world_cam0"]).reshape(4, 4)[:3, 3]]
                     + [int(m.get("n_slam") or 0), int(m.get("n_msckf") or 0)] for m in tr]
    json.dump(r, open(str(dst) + ".json", "w"))
    with gzip.open(str(dst) + ".poses.jsonl.gz", "wt") as f:
        for m in Q:
            f.write(json.dumps(m) + "\n")
    return dst


def swba_stats(pre):
    swlog = Path(str(pre) + ".swba.jsonl")
    if not swlog.exists():
        return {}
    F = [json.loads(x) for x in open(swlog)]
    F = [j for j in F if j.get("type") == "frame"]
    nl = [j for j in F if j["flag"] == "nl"]
    ms = np.array([j["ms"] for j in nl]) if nl else np.array([0.0])
    first = next((j["t"] for j in nl), None)
    return dict(n_frames=len(F), n_nl=len(nl), n_fail=sum(j["flag"] == "fail" for j in F), n_kf=sum(j["kf"] for j in nl),
                t_first_nl=round(first - F[0]["t"], 2) if first is not None and F else None,
                ms_p50=round(float(np.median(ms)), 2), ms_p95=round(float(np.percentile(ms, 95)), 2),
                ms_max=round(float(ms.max()), 2),
                solve_p50=round(float(np.median([j["solve_ms"] for j in nl])), 2) if nl else None,
                margin_p50=round(float(np.median([j["margin_ms"] for j in nl])), 2) if nl else None,
                it_p50=float(np.median([j["it"] for j in nl])) if nl else None,
                n_feat_p50=float(np.median([j["n_feat"] for j in nl])) if nl else None)


def score_gt(which, conds, gs):
    gt = GT if which == "hs" else STD_GT
    rows = []
    for r, o in cells(which):
        k = key_of(r, o, which)
        row = dict(rec=r, off=o, key=k)
        ref = REF / f"{k}_GL60"
        if Path(str(ref) + ".json").exists():
            mc = gs.cam_metrics(ref, r, gt)
            row["GL60"] = dict(div=bool(mc.get("divergent")), ate_w=mc.get("ate_w"), hash=vio_hash(load_P(ref)))
        for c in conds:
            pre = OUT / c / f"{k}_{c}"
            if not Path(str(pre) + ".json").exists():
                continue
            P = load_P(pre)
            mc = gs.cam_metrics(pre, r, gt)
            e = dict(div=bool(mc.get("divergent")), ate_w=mc.get("ate_w"), hash=vio_hash(P))
            for kind in ("swba", "raw", "rawvio"):
                dp = write_variant(pre, kind, OUT / f"{c}_{kind}")
                if dp is None:
                    continue
                try:
                    mk = gs.cam_metrics(dp, r, gt)
                    e[kind] = dict(div=bool(mk.get("divergent")), ate_w=mk.get("ate_w"), why=mk.get("why"))
                except Exception as ex:  # noqa: BLE001
                    e[kind] = dict(err=str(ex)[:200])
            e["sw"] = swba_stats(pre)
            row[c] = e
        rows.append(row)
    return rows


def score_loops(conds):
    from fi_limits_run import loop_eval  # noqa: E402
    out = {}
    for r, o in cells("loop"):
        k = key_of(r, o, "loop")
        ref = REF / f"{k}_GL60"
        if Path(str(ref) + ".json").exists():
            e = loop_eval(Path(str(ref) + ".json"))
            out.setdefault(k, {})["GL60"] = {x: e.get(x) for x in ("loop_err_cm", "path_m", "loop_pct", "tracked")}
        for c in conds:
            pre = OUT / c / f"{k}_{c}"
            if not Path(str(pre) + ".json").exists():
                continue
            e = loop_eval(Path(str(pre) + ".json"))
            d = {x: e.get(x) for x in ("loop_err_cm", "path_m", "loop_pct", "tracked")}
            d["hash"] = vio_hash(load_P(pre))
            dp = write_variant(pre, "swba", OUT / f"{c}_swba")
            if dp is not None:
                es = loop_eval(Path(str(dp) + ".json"))
                d["swba"] = {x: es.get(x) for x in ("loop_err_cm", "path_m", "loop_pct", "tracked")}
            d["sw"] = swba_stats(pre)
            out.setdefault(k, {})[c] = d
    return out


def summarize(rows, c, drop, kind=None):
    use = [x for x in rows if x["rec"] != drop and c in x]
    get = (lambda x: x[c]) if kind is None else (lambda x: x[c].get(kind) or {})
    vals = [get(x) for x in use]
    ok = [v for v in vals if not v.get("div") and v.get("ate_w") is not None]
    return dict(n=len(use), div=sum(bool(v.get("div")) for v in vals),
                div_cells=[x["key"] for x in use if get(x).get("div")],
                ate_w_med=round(float(np.median([v["ate_w"] for v in ok])), 4) if ok else None,
                ate_w_mean=round(float(np.mean([v["ate_w"] for v in ok])), 4) if ok else None)


def paired(rows, a_get, b_get, drop):
    both = []
    for x in rows:
        if x["rec"] == drop:
            continue
        A, B = a_get(x), b_get(x)
        if A and B and not A.get("div") and not B.get("div") and A.get("ate_w") is not None and B.get("ate_w") is not None:
            both.append((x["key"], x["rec"], A["ate_w"], B["ate_w"]))
    if not both:
        return {}
    d = np.array([b - a for _, _, a, b in both])
    recs = sorted({r for _, r, _, _ in both})
    per_rec = [np.mean([b - a for _, rr, a, b in both if rr == r]) for r in recs]
    return dict(n=len(both), d_med=round(float(np.median(d)), 4), d_mean=round(float(np.mean(d)), 4),
                b_better=int((d < 0).sum()), rec_b_better=f"{sum(v < 0 for v in per_rec)}/{len(per_rec)}",
                big=[(k, round(a, 2), round(b, 2)) for k, _, a, b in both if abs(b - a) > 0.3])


def score(conds):
    import lens_switch_gtscore as gs
    gs.OUT = OUT / "score_tmp"
    gs.OUT.mkdir(parents=True, exist_ok=True)
    _, drop, _ = hs_cells()
    res = dict(made=time.strftime("%F %T"), protocol=__doc__, conds={c: CONDS[c] for c in conds},
               bridge=json.load(open(OUT / "bin" / "bridge.json")), ref=str(REF))
    for which in ("hs", "std"):
        rows = score_gt(which, conds, gs)
        S = {"GL60": summarize(rows, "GL60", drop if which == "hs" else None)}
        Pp = {}
        for c in conds:
            if not any(c in x for x in rows):
                continue
            S[c] = summarize(rows, c, drop if which == "hs" else None)
            for kind in ("swba", "raw", "rawvio"):
                S[f"{c}.{kind}"] = summarize(rows, c, drop if which == "hs" else None, kind)
            dd = drop if which == "hs" else None
            Pp[f"GL60->{c}.swba"] = paired(rows, lambda x: x.get("GL60"), lambda x, c=c: (x.get(c) or {}).get("swba"), dd)
            Pp[f"{c}.rawvio->{c}.raw"] = paired(rows, lambda x, c=c: (x.get(c) or {}).get("rawvio"),
                                               lambda x, c=c: (x.get(c) or {}).get("raw"), dd)
            Pp[f"identity {c} vs GL60"] = dict(
                same=sum(1 for x in rows if c in x and "GL60" in x and x[c]["hash"] == x["GL60"]["hash"]),
                n=sum(1 for x in rows if c in x and "GL60" in x))
        res[which] = dict(summary=S, paired=Pp, cells=rows)
    res["loops"] = score_loops(conds)
    tag = "_".join(conds)
    json.dump(res, open(OUT / f"score_{tag}.json", "w"), indent=1, ensure_ascii=False)
    RD.mkdir(parents=True, exist_ok=True)
    json.dump(res, open(RD / f"score_{tag}.json", "w"), indent=1, ensure_ascii=False)
    for which in ("hs", "std"):
        print(f"\n## {which}")
        for c, s in res[which]["summary"].items():
            print(f"  {c:14s} n {s['n']:2d} div {s['div']} {s['div_cells']} ate_w med {s['ate_w_med']} mean {s['ate_w_mean']}")
        for k, p in res[which]["paired"].items():
            print(f"  {k}: {p}")
    print("\n## loops")
    for k, v in res["loops"].items():
        print(" ", k, json.dumps(v))
    print("saved", RD / f"score_{tag}.json")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["freeze", "run", "job", "score"])
    ap.add_argument("arg", nargs="?", default="")
    ap.add_argument("--par", type=int, default=5)
    a = ap.parse_args()
    if a.cmd == "freeze":
        freeze()
    elif a.cmd == "run":
        run_sched(a.arg, a.par)
    elif a.cmd == "job":
        job(a.arg)
    else:
        score(a.arg.split(",") if a.arg else ["Z60", "W60"])
