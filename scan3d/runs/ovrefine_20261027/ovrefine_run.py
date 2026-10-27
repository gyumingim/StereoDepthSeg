#!/usr/bin/env python3
"""ovrefine 2026-10-27: offline evaluation of ov_bridge --refine 1 (OpenVINS-anchored window refinement, "ref" pose) on the
pre-registered xfopt split (split.json, PLAN.md). Job set-up = xfopt_run.py (lens05 lens, vio_bench_hs protocol), scoring =
ovswba swba_run.py (lens_switch_gtscore.cam_metrics, camera position, divergence ATE > 50 cm or 5 s rel > 30 cm).
  B60   : this branch's bridge, --refine 0 (baseline = OpenVINS GL60 live config, no loop closure)
  BASE  : the integ2 4035d59 build (no vins/ code; ov_bridge tree = lens05) -> identity of --refine 0 only
  R*    : --refine 1 --refine-cfg ... (VIO fields must equal B60; "ref" = candidate trajectory)
  python3 ovrefine_run.py freeze
  python3 ovrefine_run.py run "dev:B60,R10" [--par 3]        sets: dev, test, std, loop
  python3 ovrefine_run.py score B60,R10 [--sets dev]
OUT = /home/system/scanner/tmp_ovrefine/ate (env OVR_OUT).
"""
import argparse
import gzip
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

MAIN = Path("/home/system/scanner/StereoDepthSeg/scan3d")
WT = Path("/home/system/scanner/StereoDepthSeg-ovrefine/scan3d")
RD = WT / "runs" / "ovrefine_20261027"
OUT = Path(os.environ.get("OVR_OUT", "/home/system/scanner/tmp_ovrefine/ate"))
SRC = WT / "ov_bridge" / "build_ovrefine" / "ov_bridge"
BASE_SRC = Path("/home/system/scanner/StereoDepthSeg-integ2/scan3d/ov_bridge/build_integ2_full/ov_bridge")
PY = str(MAIN.parent / "venv_ffs" / "bin" / "python")
sys.path.insert(0, str(MAIN / "imu_learn"))
sys.path.insert(0, str(MAIN))
os.chdir(MAIN)

SPLIT = json.load(open(RD / "split.json"))
LENS_HS = str(WT / SPLIT["lens"]["hs"])
LENS_STD = str(WT / SPLIT["lens"]["std"])
OFFS = (0, 1, 2)
LEVER = "config/imu_cam_lever.json"
BASE_CFG = "num_opencv_threads=1,try_zupt=false,fi_max_cond_number=1e6,fi_max_baseline=200"
LIVE = 'histogram_method="NONE"'
OLD = {"hs_20261019_131326", "hs_20261019_154320"}
P60 = "--profile rec60 --drop-late 0 --loop 0"
GT_HS, GT_STD = "imu_gt_spl_cal1022", "imu_gt_spl"


def R(cfg=""):
    return dict(refine="n=10,freq=10,prior_scale=1,min_obs=3,iters=10" + ("," + cfg if cfg else ""))


CONDS = {
    "B60": dict(),
    "BASE": dict(base=True),
    "R10": R(),                    # PLAN 4 default
    "R5": R("n=5"),
    "R10p4": R("prior_scale=4"),
    "R10q": R("prior_scale=0.25"),
    "R10m2": R("min_obs=2"),
    "R30f30": R("n=30,freq=30"),   # PLAN 4 addition (14:10, before the dev grid): 1 s window at 30 Hz (uses 3x the frames)
    "R10v": R(),                   # 15:36: = R10 on the prior-fix build (check: the fix leaves R10 practically unchanged)
    "RX0": R("dp_max=0"),          # test (c): every window rejected -> ref == VIO
}


def cells(which):
    if which == "dev":
        return [(r, o) for r in SPLIT["dev"] for o in OFFS]
    if which == "test":
        return [(r, o) for r in SPLIT["test"] for o in OFFS]
    if which == "std":
        return [(r, o) for r in SPLIT["test_std"] for o in OFFS]
    if which == "loop":
        return [(r, 0) for r in SPLIT["test_loop"]]
    raise ValueError(which)


def key_of(rec, off, which):
    hh = rec.split("_")[2]
    return {"dev": f"{hh}_o{off}", "test": f"{hh}_o{off}", "std": f"s{hh}_o{off}", "loop": f"l{hh}_o{off}"}[which]


def which_of(key):
    if key[0] == "s":
        return "std"
    if key[0] == "l":
        return "loop"
    return "dev" if any(r.split("_")[2] == key.split("_")[0] for r in SPLIT["dev"]) else "test"


def git(*a):
    return subprocess.run(["git", "-C", str(WT), *a], capture_output=True, text=True).stdout.strip()


def plan_gate(sets, conds):
    for f in ("runs/ovrefine_20261027/PLAN.md", "runs/ovrefine_20261027/split.json"):
        if git("status", "--porcelain", f):
            raise SystemExit(f"plan gate: {f} has uncommitted changes")
    pc = git("log", "-1", "--format=%H", "--", "runs/ovrefine_20261027/PLAN.md")
    if not pc:
        raise SystemExit("plan gate: PLAN.md not committed")
    if set(sets) & {"test", "std", "loop"}:
        tf = "runs/ovrefine_20261027/test_conds.json"
        if not (WT / tf).exists() or git("status", "--porcelain", tf) or not git("log", "-1", "--format=%H", "--", tf):
            raise SystemExit("plan gate: test_conds.json missing / not committed")
        allowed = set(json.load(open(WT / tf))["conds"])
        bad = [c for c in conds if c not in allowed]
        if bad:
            raise SystemExit(f"plan gate: {bad} not in test_conds.json")
    return pc


def freeze():
    (OUT / "bin").mkdir(parents=True, exist_ok=True)
    info = dict(frozen=time.strftime("%F %T"), worktree_head=git("rev-parse", "HEAD"),
                ov_bridge_dirty=[x for x in git("status", "--short", "ov_bridge").splitlines() if "build_" not in x])
    for tag, src in (("main", SRC), ("base", BASE_SRC)):
        h = hashlib.sha256(open(src, "rb").read()).hexdigest()
        dst = OUT / "bin" / f"ov_bridge_{h[:12]}"
        if not dst.exists():
            shutil.copy2(src, dst)
        info[tag] = dict(src=str(src), sha256=h, copy=str(dst))
        w = OUT / "bin" / f"br_{tag}.sh"
        w.write_text(f"#!/bin/bash\nexec {dst} \"$@\" $OVR_ARGS\n")
        w.chmod(0o755)
    json.dump(info, open(OUT / "bin" / "bridge.json", "w"), indent=1)
    print("frozen", json.dumps(info, indent=1))


def job(name):
    import vio_bench_hs as vb
    key6, o, cond = name.split("_")
    which = which_of(key6)
    rec = next(r for r, oo in cells(which) if r.split("_")[2] == key6.lstrip("sl"))
    off = int(o[1:])
    d = CONDS[cond]
    od = OUT / cond
    od.mkdir(parents=True, exist_ok=True)
    args = P60
    if d.get("refine"):
        args += f" --refine 1 --refine-log {od / name}.refine.jsonl --refine-cfg {d['refine']}"
    os.environ["OVR_ARGS"] = args
    vb.BRIDGE = OUT / "bin" / ("br_base.sh" if d.get("base") else "br_main.sh")
    vb.TEMPLATE = MAIN / "config" / "mono" / "estimator_config.yaml"
    import tempfile as _tf
    import types
    (OUT / "tmp").mkdir(parents=True, exist_ok=True)
    vb.tempfile = types.SimpleNamespace(mkdtemp=lambda prefix="vio_", dir=None: _tf.mkdtemp(prefix=prefix, dir=str(OUT / "tmp")))
    cfg = BASE_CFG + "," + LIVE
    lens = LENS_STD if which in ("std", "loop") else LENS_HS
    a = argparse.Namespace(rec=f"recordings/{rec}", fps=60.0, size="960x540", npts=200, clones=15,
                           noise="cfg", noise_scale=1.0, imu_calib=None, lever=LEVER, lens_file=lens, dropout="",
                           max_secs=0.0, imu_raw=rec not in OLD, gt="none" if which == "loop" else "auto", out=None,
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
    r.update(ovr_made=time.strftime("%F %T"), ovr_job_s=round(time.time() - t0, 1), ovr_cond=cond, ovr_args=args, ovr_set=which,
             ovr_bridge_sha256=br["base" if d.get("base") else "main"]["sha256"], ovr_worktree_head=br["worktree_head"],
             ovr_lens=lens, ovr_runner_sha256=hashlib.sha256(open(__file__, "rb").read()).hexdigest(),
             plan_commit=os.environ.get("OVR_PLAN_COMMIT"), ovr_load=list(os.getloadavg()))
    if saved.get("poses") is not None:
        with gzip.open(str(od / name) + ".poses.jsonl.gz", "wt") as f:
            for m in saved["poses"]:
                f.write(json.dumps(m) + "\n")
    json.dump(r, open(od / f"{name}.json", "w"), indent=1, ensure_ascii=False)


def run_sched(spec, par):
    q, sets, conds = [], [], []
    for part in spec.split(";"):
        which, cs = part.split(":")
        sets.append(which)
        conds += cs.split(",")
        q += [f"{key_of(r, o, which)}_{c}" for c in cs.split(",") for r, o in cells(which)]
    for c in conds:
        if c not in CONDS:
            raise SystemExit(f"unknown condition {c}")
    pc = plan_gate(sets, conds)
    os.environ["OVR_PLAN_COMMIT"] = pc
    q = [n for n in q if not (OUT / n.split("_")[2] / f"{n}.json").exists()]
    running, t_all = {}, time.time()
    print(f"{time.strftime('%F %T')} start {spec} {len(q)} jobs PAR {par} plan {pc[:8]}", flush=True)
    hb = OUT / "heartbeat.txt"
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
        hb.write_text(f"{time.strftime('%F %T')} {spec} queue {len(q)} running {len(running)}\n")
        time.sleep(2)
    print(f"{time.strftime('%F %T')} end {spec} wall {time.time() - t_all:.0f} s", flush=True)
    hb.write_text(f"{time.strftime('%F %T')} {spec} END\n")


# ------------------------------------------------------------------ scoring
VOLATILE = ("ms", "remap_ms", "wait_ms", "ref")


def load_P(pre):
    f = Path(str(pre) + ".poses.jsonl.gz")
    return [json.loads(x) for x in gzip.open(f, "rt") if x.strip()] if f.exists() else []


def vio_hash(P):
    h = hashlib.sha256()
    for m in P:
        h.update(json.dumps({k: v for k, v in m.items() if k not in VOLATILE}, sort_keys=True).encode())
    return h.hexdigest()[:16]


def ref_poses(P):
    out = []
    for m in P:
        s = m.get("ref")
        if s and m.get("T_world_cam0") is not None:
            m2 = dict(m)
            for k in ("p", "q", "v", "T_world_cam0"):
                m2[k] = s[k]
            out.append(m2)
        else:
            out.append(m)
    return out


def write_ref(pre, dst_dir):
    P = load_P(pre)
    if not any("ref" in m for m in P):
        return None, None
    Q = ref_poses(P)
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
    # same trajectory as VIO? (all windows rejected -> must be identical camera positions)
    same = all(np.allclose(np.asarray(a["T_world_cam0"]), np.asarray(b["T_world_cam0"]), atol=0, rtol=0)
               for a, b in zip(P, Q) if a.get("T_world_cam0") is not None)
    return dst, same


def ref_stats(pre, P):
    lf = Path(str(pre) + ".refine.jsonl")
    if not lf.exists():
        return {}
    W = [json.loads(x) for x in open(lf) if x.strip()]
    win = [j for j in W if j.get("type") == "win"]
    ok = [j for j in win if j["ok"]]
    why = {}
    for j in win:
        if not j["ok"]:
            why[j["why"]] = why.get(j["why"], 0) + 1
    last = next((m["ref"] for m in reversed(P) if m.get("ref")), None)
    ms = np.array([j["ms"] for j in win]) if win else np.array([0.0])
    return dict(n_win=len(win), n_ok=len(ok), n_rej=len(win) - len(ok), why=why, n_reset=sum(j.get("type") == "reset" for j in W),
                pose_line_counts=[last.get("n_win"), last.get("n_ok"), last.get("n_rej")] if last else None,
                counts_match=bool(last and last.get("n_win") == len(win) and last.get("n_ok") == len(ok)),
                ms_p50=round(float(np.median(ms)), 2), ms_p95=round(float(np.percentile(ms, 95)), 2), ms_max=round(float(ms.max()), 2),
                feat_p50=float(np.median([j["feat"] for j in win])) if win else None,
                it_p50=float(np.median([j["it"] for j in win])) if win else None,
                dp_mm_ok_p50=round(float(np.median([j["dp_mm"] for j in ok])), 3) if ok else None,
                dp_mm_ok_p95=round(float(np.percentile([j["dp_mm"] for j in ok], 95)), 3) if ok else None,
                dth_ok_p50=round(float(np.median([j["dth_deg"] for j in ok])), 4) if ok else None)


def rpe1(pre, rec, gt):
    """1 s relative camera-position error, median cm (SE3-aligned window like cam_metrics; reference metric)."""
    import zupt_ob_score as zs
    j = json.load(open(str(pre) + ".json"))
    if j.get("ate_cm") is None:
        return None
    P = load_P(pre)
    T = [m for m in P if m.get("status") == "tracking" and m.get("T_world_cam0")]
    if len(T) < 100:
        return None
    p = np.array([np.asarray(m["T_world_cam0"], float).reshape(4, 4)[:3, 3] for m in T])
    t = np.array([m["t"] for m in T])
    g = np.load(MAIN / "recordings" / rec / gt / "gt.npz", allow_pickle=True)
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    off = P[0]["t_ns"] - P[0]["t"] * 1e9 + j["align_dt_ms"] * 1e6
    tg = (g["t_ns"][ok] - off) / 1e9
    cg = g["p_bc"][ok].reshape(-1, 3)
    o = np.argsort(tg)
    tg, cg = tg[o], cg[o]
    k = np.clip(np.searchsorted(tg, t), 1, len(tg) - 1)
    k = np.where(np.abs(tg[k - 1] - t) < np.abs(tg[k] - t), k - 1, k)
    good = np.abs(tg[k] - t) <= 0.002
    if good.sum() < 50:
        return None
    Rm, tr = zs.aa.umeyama_se3(p[good], cg[k[good]])
    pa = (Rm @ p[good].T).T + tr
    tt, gg = t[good], cg[k[good]]
    j1 = np.searchsorted(tt, tt + 1.0)
    m = j1 < len(tt)
    i0, i1 = np.nonzero(m)[0], j1[m]
    keep = np.abs(tt[i1] - tt[i0] - 1.0) < 0.05
    if keep.sum() < 10:
        return None
    e = np.linalg.norm((pa[i1[keep]] - pa[i0[keep]]) - (gg[i1[keep]] - gg[i0[keep]]), axis=1) * 100
    return round(float(np.median(e)), 4)


def score_cells(which, conds, gs):
    gt = GT_HS if which in ("dev", "test") else GT_STD
    rows = []
    for r, o in cells(which):
        k = key_of(r, o, which)
        row = dict(rec=r, off=o, key=k)
        for c in conds:
            pre = OUT / c / f"{k}_{c}"
            if not Path(str(pre) + ".json").exists():
                continue
            P = load_P(pre)
            mc = gs.cam_metrics(pre, r, gt)
            tr = [m for m in P if m.get("T_world_cam0") is not None]
            e = dict(div=bool(mc.get("divergent")), ate_w=mc.get("ate_w"), hash=vio_hash(P), rpe1=rpe1(pre, r, gt),
                     init_t_ns=tr[0]["t_ns"] if tr else None,
                     pose0_sha=hashlib.sha256(json.dumps(tr[0]["T_world_cam0"]).encode()).hexdigest()[:12] if tr else None,
                     plan_commit=json.load(open(str(pre) + ".json")).get("plan_commit"))
            dp, same = write_ref(pre, OUT / f"{c}_ref")
            if dp is not None:
                mk = gs.cam_metrics(dp, r, gt)
                e["ref"] = dict(div=bool(mk.get("divergent")), ate_w=mk.get("ate_w"), why=mk.get("why"), rpe1=rpe1(dp, r, gt),
                                same_as_vio=same)
                e["rs"] = ref_stats(pre, P)
            row[c] = e
        rows.append(row)
    return rows


def score_loops(conds):
    from fi_limits_run import loop_eval  # noqa: E402
    out = {}
    for r, o in cells("loop"):
        k = key_of(r, o, "loop")
        for c in conds:
            pre = OUT / c / f"{k}_{c}"
            if not Path(str(pre) + ".json").exists():
                continue
            P = load_P(pre)
            e = loop_eval(Path(str(pre) + ".json"))
            d = {x: e.get(x) for x in ("loop_err_cm", "path_m", "loop_pct", "tracked")}
            d["hash"] = vio_hash(P)
            dp, same = write_ref(pre, OUT / f"{c}_ref")
            if dp is not None:
                es = loop_eval(Path(str(dp) + ".json"))
                d["ref"] = {x: es.get(x) for x in ("loop_err_cm", "path_m", "loop_pct", "tracked")}
                d["ref"]["same_as_vio"] = same
                d["rs"] = ref_stats(pre, P)
            out.setdefault(k, {})[c] = d
    return out


def summarize(rows, get):
    vals = [get(x) for x in rows if get(x)]
    ok = [v for v in vals if not v.get("div") and v.get("ate_w") is not None]
    return dict(n=len(vals), div=sum(bool(v.get("div")) for v in vals),
                div_cells=[x["key"] for x in rows if get(x) and get(x).get("div")],
                ate_w_med=round(float(np.median([v["ate_w"] for v in ok])), 4) if ok else None,
                rpe1_med=round(float(np.median([v["rpe1"] for v in ok if v.get("rpe1") is not None])), 4) if ok else None)


def paired(rows, a_get, b_get, key="ate_w"):
    both = []
    for x in rows:
        A, B = a_get(x), b_get(x)
        if A and B and not A.get("div") and not B.get("div") and A.get(key) is not None and B.get(key) is not None:
            both.append((x["key"], x["rec"], A[key], B[key]))
    if not both:
        return {}
    d = np.array([b - a for _, _, a, b in both])
    recs = sorted({r for _, r, _, _ in both})
    per_rec = {r: round(float(np.mean([b - a for _, rr, a, b in both if rr == r])), 4) for r in recs}
    return dict(n=len(both), d_med=round(float(np.median(d)), 4), d_mean=round(float(np.mean(d)), 4),
                b_better=int((d < 0).sum()), b_worse=int((d > 0).sum()),
                rec_b_better=f"{sum(v < 0 for v in per_rec.values())}/{len(per_rec)}", per_rec=per_rec)


def score(conds, sets):
    import lens_switch_gtscore as gs
    gs.OUT = OUT / "score_tmp"
    gs.OUT.mkdir(parents=True, exist_ok=True)
    res = dict(made=time.strftime("%F %T"), protocol=__doc__, conds={c: CONDS[c] for c in conds},
               bridge=json.load(open(OUT / "bin" / "bridge.json")), plan_commit=git("log", "-1", "--format=%H", "--", "runs/ovrefine_20261027/PLAN.md"),
               runner_sha256=hashlib.sha256(open(__file__, "rb").read()).hexdigest(), sets=sets)
    for which in [s for s in sets if s != "loop"]:
        rows = score_cells(which, conds, gs)
        S, Pp = {}, {}
        for c in conds:
            if not any(c in x for x in rows):
                continue
            S[c] = summarize(rows, lambda x, c=c: x.get(c))
            if any((x.get(c) or {}).get("ref") for x in rows):
                S[f"{c}.ref"] = summarize(rows, lambda x, c=c: (x.get(c) or {}).get("ref"))
                Pp[f"B60->{c}.ref"] = paired(rows, lambda x: x.get("B60"), lambda x, c=c: (x.get(c) or {}).get("ref"))
                Pp[f"B60->{c}.ref rpe1"] = paired(rows, lambda x: x.get("B60"), lambda x, c=c: (x.get(c) or {}).get("ref"), "rpe1")
            if c != "B60":
                Pp[f"identity {c} vs B60"] = dict(
                    same=sum(1 for x in rows if c in x and "B60" in x and x[c]["hash"] == x["B60"]["hash"]),
                    n=sum(1 for x in rows if c in x and "B60" in x))
        res[which] = dict(summary=S, paired=Pp, cells=rows)
    if "loop" in sets:
        res["loops"] = score_loops(conds)
    tag = "_".join(conds) + "__" + "_".join(sets)
    json.dump(res, open(OUT / f"score_{tag}.json", "w"), indent=1, ensure_ascii=False)
    RD.mkdir(parents=True, exist_ok=True)
    json.dump(res, open(RD / f"score_{tag}.json", "w"), indent=1, ensure_ascii=False)
    for which in [s for s in sets if s != "loop"]:
        print(f"\n## {which}")
        for c, s in res[which]["summary"].items():
            print(f"  {c:12s} n {s['n']:2d} div {s['div']} {s['div_cells']} ate_w med {s['ate_w_med']} rpe1 med {s['rpe1_med']}")
        for k, p in res[which]["paired"].items():
            print(f"  {k}: {json.dumps(p)}")
        for x in res[which]["cells"]:
            for c in conds:
                if (x.get(c) or {}).get("rs"):
                    rs = x[c]["rs"]
                    print(f"    {x['key']} {c}: win {rs['n_win']} ok {rs['n_ok']} rej {rs['why']} match {rs['counts_match']} "
                          f"ms {rs['ms_p50']}/{rs['ms_p95']} dp_mm {rs['dp_mm_ok_p50']} same {x[c]['ref'].get('same_as_vio')}")
    if "loop" in sets:
        print("\n## loops")
        for k, v in res["loops"].items():
            print(" ", k, json.dumps({c: {kk: vv for kk, vv in d.items() if kk != "rs"} for c, d in v.items()}))
    print("saved", RD / f"score_{tag}.json")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["freeze", "run", "job", "score"])
    ap.add_argument("arg", nargs="?")
    ap.add_argument("--par", type=int, default=3)
    ap.add_argument("--sets", default="dev")
    o = ap.parse_args()
    if o.cmd == "freeze":
        freeze()
    elif o.cmd == "run":
        run_sched(o.arg, o.par)
    elif o.cmd == "job":
        job(o.arg)
    else:
        score(o.arg.split(","), o.sets.split(","))


if __name__ == "__main__":
    main()
