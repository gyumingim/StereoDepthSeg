#!/usr/bin/env python3
"""lgopt 2026-10-27 (copy of runs/xfopt_20261027/xfopt_run.py, same job / scoring code): LighterGlue redesign on the xfopt
DEV split only (PLAN.md). Changes vs xfopt_run: OUT = tmp_lgopt/ate, bridge = build_lgopt (+ xf_tri_mode), server = this
folder's xf_server.py (+ LG width / depth / prune options), HS lens = measured 2.22 dpt lens cal20261022 (PLAN 11 of xfopt),
sets = dev (15 cells) and hand (white-wall hand loops 142148 / 142610: already used once in the xfopt verdict = SEEN,
diagnostic only, never a verdict). test / std sets are refused (xfopt test split must not be reused).
  python3 lgopt_run.py freeze | run dev:B60,FNN;hand:B60,FNN [--par 8] | score
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

MAIN = Path("/home/system/scanner/StereoDepthSeg/scan3d")       # recordings + imu_learn (vio_bench_hs etc. = worktree copy, cmp-equal)
WT = Path("/home/system/scanner/StereoDepthSeg-xfopt/scan3d")
RD = WT / "runs" / "lgopt_20261027"
XRD = WT / "runs" / "xfopt_20261027"
OUT = Path(os.environ.get("LGOPT_OUT", "/home/system/scanner/tmp_lgopt/ate"))
SRC = Path(os.environ.get("LGOPT_SRC", str(WT / "ov_bridge" / "build_lgopt" / "ov_bridge")))
VOC = WT / "ov_bridge" / "loop" / "voc_k10L4.voc"
XFS = RD / "xf_server.py"
PY = str(MAIN.parent / "venv_ffs" / "bin" / "python")
sys.path.insert(0, str(MAIN / "imu_learn"))
sys.path.insert(0, str(MAIN))
os.chdir(MAIN)

SPLIT = json.load(open(XRD / "split.json"))   # xfopt split (dev part only is used here)
LENS_HS = os.environ.get("LGOPT_LENS_HS") or str(WT / "config/lens/hs_1920x1080_f2.22_cal20261022.json")
LENS_STD = str(WT / SPLIT["lens"]["std"])
OFFS = (0, 1, 2)
LEVER = "config/imu_cam_lever.json"
BASE_CFG = "num_opencv_threads=1,try_zupt=false,fi_max_cond_number=1e6,fi_max_baseline=200"
LIVE = 'histogram_method="NONE"'
OLD = {"hs_20261019_131326", "hs_20261019_154320"}            # no raw IMU (phone60_run OLD)
P60 = "--profile rec60 --drop-late 0"
XF_PY = f"PYTHONPATH=/home/system/scanner/tmp_ovxfeat/pylib {PY}"
LOOPARGS = "--loop 1 --loop-sync 1"


def X(mode=1, topk=1024, w=640, h=360, tri=1, cfg="", srv="", **_):
    """XFeat condition: server options + extra --loop-cfg (always log_cand=1, xf_both=1: both paths logged)."""
    return dict(xf=dict(mode=mode, srv=f"--topk {topk} --w {w} --h {h} {srv}".strip(),
                        cfg=f"xf_mode={mode},xf_tri={tri},log_cand=1,xf_both=1" + ("," + cfg if cfg else "")))


# conditions (cands.md has the reason for each; added before their first run)
CONDS = {
    "B60": dict(extra="--loop-cfg log_cand=1"),           # baseline: BoW + box-BRIEF (log_cand does not change decisions)
    "B60r": dict(extra=""),                               # B60 without log_cand (identity check only)
    "X1": X(1, 1024),                                     # = ovxfeat N1kt on this build (NN 1k, 640, tri 1 deg)
    "X1n": X(1, 1024, tri=0),                             # = N1k (no triangulation)
    "X1L": X(2, 1024),                                    # = L1kt (LighterGlue)
}
COND_FILE = RD / "conds.json"                       # later conditions (committed before running)
if COND_FILE.exists():
    for k, v in json.load(open(COND_FILE)).items():
        CONDS[k] = X(**v) if "extra" not in v else v


def cells(which):
    if which == "dev":
        return [(r, o) for r in SPLIT["dev"] for o in OFFS]
    if which == "hand":       # SEEN (xfopt verdict used them): diagnosis / white-wall recall only
        return [(r, 0) for r in SPLIT["test_loop"]]
    raise ValueError(which)


def key_of(rec, off, which):
    hh = rec.split("_")[2]
    return {"dev": f"{hh}_o{off}", "hand": f"l{hh}_o{off}"}[which]


def which_of(key):
    if key[0] == "l":
        return "hand"
    if any(r.split("_")[2] == key.split("_")[0] for r in SPLIT["dev"]):
        return "dev"
    raise SystemExit(f"lgopt: {key} is not a dev / hand cell (xfopt test split is not reused)")


def git(*a):
    return subprocess.run(["git", "-C", str(WT), *a], capture_output=True, text=True).stdout.strip()


def plan_gate(sets, conds):
    """lgopt PLAN.md + conds.json committed and clean; only dev / hand sets."""
    bad = set(sets) - {"dev", "hand"}
    if bad:
        raise SystemExit(f"plan gate: sets {bad} refused (xfopt test split already used)")
    for f in ("runs/lgopt_20261027/PLAN.md", "runs/lgopt_20261027/conds.json"):
        if not (WT / f).exists() or git("status", "--porcelain", f) or not git("log", "-1", "--format=%H", "--", f):
            raise SystemExit(f"plan gate: {f} missing / uncommitted")
    return git("log", "-1", "--format=%H", "--", "runs/lgopt_20261027/PLAN.md")


def freeze():
    (OUT / "bin").mkdir(parents=True, exist_ok=True)
    h = hashlib.sha256(open(SRC, "rb").read()).hexdigest()
    dst = OUT / "bin" / f"ov_bridge_{h[:12]}"
    if not dst.exists():
        shutil.copy2(SRC, dst)
    hv = hashlib.sha256(open(VOC, "rb").read()).hexdigest()
    vd = OUT / "bin" / f"voc_{hv[:12]}.voc"
    if not vd.exists():
        shutil.copy2(VOC, vd)
    dirty = git("status", "--short", "ov_bridge")
    json.dump(dict(src=str(SRC), sha256=h, copy=str(dst), voc=str(vd), voc_sha256=hv, frozen=time.strftime("%F %T"),
                   worktree_head=git("rev-parse", "HEAD"), ov_bridge_dirty=[x for x in dirty.splitlines() if "build_" not in x],
                   xf_server_sha256=hashlib.sha256(open(XFS, "rb").read()).hexdigest()),
              open(OUT / "bin" / "bridge.json", "w"), indent=1)
    w = OUT / "bin" / "br.sh"
    w.write_text("#!/bin/bash\n"
                 + 'if [ -n "$XFOPT_XF" ]; then exec ' + f'{dst} "$@" $XFOPT_PROFILE {LOOPARGS} --loop-vocab {vd} $OVLOOP_ARGS --loop-xf "$XFOPT_XF"; fi\n'
                 + f'exec {dst} "$@" $XFOPT_PROFILE {LOOPARGS} --loop-vocab {vd} $OVLOOP_ARGS\n')
    w.chmod(0o755)
    print("frozen", dst, h, "voc", vd)


def map_poses(poses):
    out = []
    for m in poses:
        mm = m.get("map")
        if mm and m.get("T_world_cam0") is not None:
            m2 = dict(m)
            for k in ("p", "q", "v", "T_world_cam0"):
                m2[k] = mm[k]
            out.append(m2)
        else:
            out.append(m)
    return out


def job(name):
    import vio_bench_hs as vb
    key6, o, cond = name.split("_")
    which = which_of(key6)
    rec = next(r for r, oo in cells(which) if r.split("_")[2] == key6.lstrip("sl"))
    off = int(o[1:])
    d = CONDS[cond]
    od = OUT / cond
    od.mkdir(parents=True, exist_ok=True)
    args = f"--loop-log {od / name}.loop.jsonl"
    os.environ["XFOPT_PROFILE"] = P60
    os.environ.pop("XFOPT_XF", None)
    if d.get("xf"):
        x = d["xf"]
        os.environ["XFOPT_XF"] = f"{XF_PY} {XFS} {x['srv']} --log {od / name}.xf.jsonl"
        args += f" --loop-cfg {x['cfg']}"
    if d.get("extra"):
        args += " " + d["extra"]
    os.environ["OVLOOP_ARGS"] = args
    vb.BRIDGE = OUT / "bin" / "br.sh"
    vb.TEMPLATE = MAIN / "config" / "mono" / "estimator_config.yaml"
    import tempfile as _tf
    import types
    (OUT / "tmp").mkdir(parents=True, exist_ok=True)
    vb.tempfile = types.SimpleNamespace(mkdtemp=lambda prefix="vio_", dir=None: _tf.mkdtemp(prefix=prefix, dir=str(OUT / "tmp")))
    cfg = BASE_CFG + "," + LIVE
    lens = LENS_STD if which == "hand" else LENS_HS
    a = argparse.Namespace(rec=f"recordings/{rec}", fps=60.0, size="960x540", npts=200, clones=15,
                           noise="cfg", noise_scale=1.0, imu_calib=None, lever=LEVER, lens_file=lens, dropout="",
                           max_secs=0.0, imu_raw=rec not in OLD, gt="none" if which == "hand" else "auto", out=None,
                           init_cfg="strict", start_off=float(off), decode_threads=2, imu_lead=2.0, init_window=1.0,
                           init_disp=22.5, frame_cache=True, gyro_rebias=False, cfg_over=cfg, still_hold=None)
    saved = {}
    orig = vb.evaluate

    def ev(rec_, a_, poses, *rest):
        saved["poses"] = poses
        saved["rest"] = rest
        r1 = orig(rec_, a_, poses, *rest)
        saved["map"] = orig(rec_, a_, map_poses(poses), *rest)
        return r1
    vb.evaluate = ev
    t0 = time.time()
    try:
        r = vb.run(a)
    finally:
        vb.evaluate = orig
    br = json.load(open(OUT / "bin" / "bridge.json"))
    r.update(ol_made=time.strftime("%F %T"), ol_job_s=round(time.time() - t0, 1), ol_cond=cond, ol_args=args, ol_set=which,
             ol_bridge_sha256=br["sha256"], ol_voc_sha256=br["voc_sha256"], ol_worktree_head=br["worktree_head"],
             ol_xf_server_sha256=br.get("xf_server_sha256"), ol_xf=d.get("xf"), ol_lens=lens,
             ol_runner_sha256=hashlib.sha256(open(__file__, "rb").read()).hexdigest(),
             plan_commit=os.environ.get("LGOPT_PLAN_COMMIT"), ol_load=list(os.getloadavg()),
             dt_ns=int(saved["rest"][2]) if saved.get("rest") else None)
    r["map_res"] = saved.get("map")
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
    os.environ["LGOPT_PLAN_COMMIT"] = pc
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
            if (OUT / n.split("_")[2] / f"{n}.json").exists():
                continue
            busy = subprocess.run(["pgrep", "-f", f"lgopt_run.py job {n}$"], capture_output=True, text=True).stdout.strip()
            if busy:
                print(f"{time.strftime('%F %T')} wait {n} (already running pid {busy.split()[0]}), re-queued", flush=True)
                q.append(n)
                break
            (OUT / n.split("_")[2]).mkdir(parents=True, exist_ok=True)
            err = open(OUT / n.split("_")[2] / f"{n}.err", "w")
            running[n] = (subprocess.Popen(["nice", "-n", "5", PY, str(Path(__file__).resolve()), "job", n],
                                           stdout=subprocess.DEVNULL, stderr=err), time.time())
        hb.write_text(f"{time.strftime('%F %T')} {spec} queue {len(q)} running {len(running)}\n")
        time.sleep(2)
    print(f"{time.strftime('%F %T')} end {spec} wall {time.time() - t_all:.0f} s", flush=True)
    hb.write_text(f"{time.strftime('%F %T')} {spec} END\n")


# ------------------------------------------------------------------ scoring helpers (also used by sim.py)
def quat_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class GT:
    """GT camera poses of a recording + extrinsics of a run (ovxfeat loop_edges_vs_gt, + board corners per GT frame)."""

    def __init__(self, res, rec, poses):
        import vio_bench_hs as vb
        g = dict(np.load(vb.pick_gt(MAIN / "recordings" / rec, "auto") / "gt.npz", allow_pickle=True))
        ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
        self.t, self.p, self.R, self.nc = g["t_ns"][ok], g["p_bc"][ok], g["R_bc"][ok], g["n_corners"][ok]
        self.dt = res.get("dt_ns") or 0
        P0 = next((m for m in poses if m.get("R_ItoC0") is not None), None)
        self.R_ItoC = np.array(P0["R_ItoC0"]).reshape(3, 3)
        self.p_IinC = np.array(P0["p_IinC0"])

    def at(self, t):
        t = t + self.dt
        j = int(np.clip(np.searchsorted(self.t, t), 1, len(self.t) - 1))
        j = j - 1 if abs(self.t[j - 1] - t) < abs(self.t[j] - t) else j
        return (self.R[j], self.p[j], int(self.nc[j])) if abs(self.t[j] - t) < 2_000_000 else None

    def err(self, t_old, t_cur, q_oc, t_oc):
        """-> (err_cm, err_deg, min corners) or None (no GT at either end)."""
        a, b = self.at(t_old), self.at(t_cur)
        if a is None or b is None:
            return None
        R_oc, t_oc = quat_R(q_oc), np.array(t_oc)
        R_c = self.R_ItoC @ R_oc @ self.R_ItoC.T
        t_c = self.R_ItoC @ (R_oc @ (-self.R_ItoC.T @ self.p_IinC) + t_oc) + self.p_IinC
        Rg = a[0].T @ b[0]
        tg = a[0].T @ (b[1] - a[1])
        e_cm = float(np.linalg.norm(t_c - tg)) * 100
        e_deg = float(np.degrees(np.arccos(np.clip((np.trace(Rg.T @ R_c) - 1) / 2, -1, 1))))
        return e_cm, e_deg, min(a[2], b[2])


MIN_CORNERS = 12   # PLAN 3: edges whose GT frame at either end has < 12 board corners are excluded from the verdict


def loop_edges_vs_gt(res, rec, poses, loopj):
    edges = [json.loads(x) for x in open(loopj) if '"type":"loop"' in x] if Path(loopj).exists() else []
    if not edges:
        return []
    G = GT(res, rec, poses)
    out = []
    for e in edges:
        row = dict(cur=e["cur"], old=e["old"], t_cur_ns=e["t_cur_ns"], t_old_ns=e["t_old_ns"],
                   dt_s=round((e["t_cur_ns"] - e["t_old_ns"]) / 1e9, 2), n_inl=e["n_inl"],
                   drift_cm=round(float(np.linalg.norm(e["drift"])) * 100, 2), dyaw_deg=round(float(np.degrees(e["dyaw"])), 3))
        r = G.err(e["t_old_ns"], e["t_cur_ns"], e["q_oc"], e["t_oc"])
        if r is None:
            row["gt"] = None
        else:
            row.update(gt=True, err_cm=round(r[0], 2), err_deg=round(r[1], 3), corners=r[2], gt_good=r[2] >= MIN_CORNERS,
                       false=bool(r[0] > 10 or r[1] > 3))
        out.append(row)
    return out


def score():
    from fi_limits_run import loop_eval  # noqa: E402
    rows = {}
    for cond in sorted(p.name for p in OUT.iterdir() if p.is_dir() and p.name in CONDS):
        for jf in sorted((OUT / cond).glob("*.json")):
            name = jf.stem
            r = json.load(open(jf))
            which = r.get("ol_set")
            row = dict(name=name, cond=cond, set=which, rec=r.get("rec"), plan_commit=r.get("plan_commit"),
                       bridge=(r.get("ol_bridge_sha256") or "")[:12], lens=r.get("ol_lens"))
            lj = OUT / cond / f"{name}.loop.jsonl"
            L = [json.loads(x) for x in open(lj)] if lj.exists() else []
            row["n_kf"] = sum(1 for x in L if x.get("type") == "kf")
            row["n_loop"] = sum(1 for x in L if x.get("type") == "loop")
            ms = [x["ms"][6] for x in L if x.get("type") == "kf"]
            row["kf_ms_p50_p95"] = [round(float(np.percentile(ms, 50)), 2), round(float(np.percentile(ms, 95)), 2)] if ms else None
            if which == "hand":
                with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
                    json.dump(r, f)
                ev = loop_eval(Path(f.name))
                mr = dict(r)
                mr["traj"] = r["map_res"]["traj"] if r.get("map_res") else []
                with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f2:
                    json.dump(mr, f2)
                em = loop_eval(Path(f2.name))
                row.update(vio_loop_cm=ev.get("loop_err_cm"), map_loop_cm=em.get("loop_err_cm"), path_m=ev.get("path_m"),
                           tracked=ev.get("tracked"))
            else:
                mres = r.get("map_res") or {}
                for k in ("ate_cm", "rpe1s_cm", "rot_med_deg", "ate_sim3_cm"):
                    row["vio_" + k] = r.get(k)
                    row["map_" + k] = mres.get(k)
                pz = OUT / cond / f"{name}.poses.jsonl.gz"
                if pz.exists():
                    poses = [json.loads(x) for x in gzip.open(pz, "rt")]
                    tr = [m for m in poses if m.get("T_world_cam0") is not None]
                    row["init_t_ns"] = tr[0]["t_ns"] if tr else None
                    row["pose0_sha"] = hashlib.sha256(json.dumps(tr[0]["T_world_cam0"]).encode()).hexdigest()[:12] if tr else None
                    if row["n_loop"]:
                        row["edges"] = loop_edges_vs_gt(r, r["rec"], poses, lj)
                        ch = [e for e in row["edges"] if e.get("gt")]
                        good = [e for e in ch if e["gt_good"]]
                        row.update(edges_checked=len(ch), edges_false=sum(e["false"] for e in ch), edges_good=len(good),
                                   edges_false_good=sum(e["false"] for e in good),
                                   edge_err_cm_max=max((e["err_cm"] for e in ch), default=None))
            rows[f"{cond}/{name}"] = row
    made = time.strftime("%F %T")
    json.dump(dict(made=made, out=str(OUT), rows=rows), open(OUT / "score.json", "w"), indent=1)
    shutil.copy2(OUT / "score.json", RD / "score.json")
    for k, v in rows.items():
        print(k, {x: v.get(x) for x in v if x not in ("edges", "name", "rec")})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["freeze", "run", "job", "score"])
    ap.add_argument("arg", nargs="?")
    ap.add_argument("--par", type=int, default=6)
    o = ap.parse_args()
    if o.cmd == "freeze":
        freeze()
    elif o.cmd == "run":
        run_sched(o.arg, o.par)
    elif o.cmd == "job":
        job(o.arg)
    else:
        score()


if __name__ == "__main__":
    main()
