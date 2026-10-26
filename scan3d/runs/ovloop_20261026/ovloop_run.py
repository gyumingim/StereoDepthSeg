#!/usr/bin/env python3
"""ovloop 2026-10-26: offline evaluation of ov_bridge --loop on the SAME data / protocol as ov60eval (the ruler).

Protocol = runs/ov60eval_20261026/ov60_run.py job() (= integ ate_run protocol): 960x540, pts 200, clones 15, init window
1.0 s / 22.5 px, cfg_over "num_opencv_threads=1,try_zupt=false,fi_max_cond_number=1e6,fi_max_baseline=200", lever,
frame cache, strict init, per-recording R_ci / dt, histogram NONE (= ov60eval "L" family, the live config), 30 fps,
bridge args "--profile rec20261025 --drop-late 0". Only change: the bridge is this branch's build with
"--loop 1 --loop-sync 1" (+ condition options). --loop never changes the VIO fields of a pose line (identity checks),
so each run gives BOTH the VIO trajectory (baseline) and the map-frame trajectory ("map" object) of the same run.

Conditions: B30 = BoW (own vocabulary loop/voc_k10L4.voc), Q30 / S30 = BoQ / SALAD global descriptors from
--loop-vpr files (vpr_desc.py on the keyframes dumped by B30; same keyframes because the keyframe rule only uses the
VIO pose, which is identical).
Scores per run (same functions for VIO and map):
  loop : loop_eval (fi_limits_run) start-end error / path for VIO and map trajectories
  std/hs (GT): vio_bench_hs.evaluate ate_cm / rpe1s_cm / rot_med_deg for VIO and map poses
  loop edges vs GT (std/hs): relative camera pose of each accepted loop edge vs GT relative pose (false loop if
  > 10 cm or > 3 deg; unchecked if GT missing at either time)
  python3 ovloop_run.py freeze
  python3 ovloop_run.py run loop:B30;std:B30 [--par 4]
  python3 ovloop_run.py score
OUT = /tmp/ovloop/ate (env OVLOOP_OUT).
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
import tempfile
import time
from pathlib import Path

import numpy as np

MAIN = Path("/home/system/scanner/StereoDepthSeg/scan3d")
WT = Path("/home/system/scanner/StereoDepthSeg-ovloop/scan3d")
RD = WT / "runs" / "ovloop_20261026"
OUT = Path(os.environ.get("OVLOOP_OUT", "/tmp/ovloop/ate"))
SRC = WT / "ov_bridge" / "build_ovloop_full" / "ov_bridge"
VOC = WT / "ov_bridge" / "loop" / "voc_k10L4.voc"
PY = str(MAIN.parent / "venv_ffs" / "bin" / "python")
sys.path.insert(0, str(MAIN / "imu_learn"))
sys.path.insert(0, str(MAIN))
os.chdir(MAIN)

OFFS = (0, 1, 2)
LEVER = "config/imu_cam_lever.json"
BASE_CFG = "num_opencv_threads=1,try_zupt=false,fi_max_cond_number=1e6,fi_max_baseline=200"
LIVE = 'histogram_method="NONE"'
STD = ["hs_20261024_165506_std60", "hs_20261024_173432_std60"]
LOOP = ["hs_20261024_142148_std60", "hs_20261024_142610_std60", "hs_20261024_151646_std60"]
P30 = "--profile rec20261025 --drop-late 0"
LOOPARGS = "--loop 1 --loop-sync 1"
CONDS = {
    "B30": dict(fps=30.0, extra=""),
    "B30f": dict(fps=30.0, extra="--loop-cfg fast_th=10,n_feat=1000,min_inliers=20"),
    "B30g": dict(fps=30.0, extra="--loop-cfg fast_th=10,n_feat=1000,min_inliers=15,ratio=0.85"),
    "Q30": dict(fps=30.0, extra="", vpr="boq"),
    "S30": dict(fps=30.0, extra="", vpr="salad"),
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
    raise ValueError(which)


def key_of(rec, off, which):
    hh = rec.split("_")[2]
    return {"hs": f"{hh}_o{off}", "std": f"s{hh}_o{off}", "loop": f"l{hh}_o{off}"}[which]


def which_of(key):
    return {"s": "std", "l": "loop"}.get(key[0], "hs")


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
    git = subprocess.run(["git", "-C", str(WT), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(["git", "-C", str(WT), "status", "--short", "ov_bridge"], capture_output=True, text=True).stdout
    json.dump(dict(src=str(SRC), sha256=h, copy=str(dst), voc=str(vd), voc_sha256=hv, frozen=time.strftime("%F %T"),
                   worktree_head=git, ov_bridge_dirty=[x for x in dirty.splitlines() if "build_" not in x]),
              open(OUT / "bin" / "bridge.json", "w"), indent=1)
    w = OUT / "bin" / "br.sh"   # condition / per-job options come from the environment (set by job())
    w.write_text(f"#!/bin/bash\nexec {dst} \"$@\" {P30} {LOOPARGS} --loop-vocab {vd} $OVLOOP_ARGS\n")
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
    _, _, old = hs_cells()
    d = CONDS[cond]
    od = OUT / cond
    od.mkdir(parents=True, exist_ok=True)
    args = f"--loop-log {od / name}.loop.jsonl"
    if cond.startswith("B30"):
        dump = OUT / "kf" / (name.rsplit("_", 1)[0] + ("" if cond == "B30" else "_" + cond))
        if dump.exists():
            shutil.rmtree(dump)
        dump.mkdir(parents=True)
        args += f" --loop-dump {dump}"
    if d.get("vpr"):
        vf = OUT / "vpr" / d["vpr"] / (name.rsplit("_", 1)[0] + ".ovpr")
        if not vf.exists():
            raise SystemExit(f"missing {vf}")
        args += f" --loop-vpr {vf}"
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
             ol_load=list(os.getloadavg()), dt_ns=int(saved["rest"][2]) if saved.get("rest") else None)
    r["map_res"] = saved.get("map")
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
def quat_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def loop_edges_vs_gt(res, rec, poses, loopj):
    """each accepted loop edge (T_oldI_curI) -> camera relative pose vs GT camera relative pose."""
    import vio_bench_hs as vb
    edges = [json.loads(x) for x in open(loopj) if '"type":"loop"' in x] if Path(loopj).exists() else []
    if not edges:
        return []
    gt_dir = vb.pick_gt(MAIN / "recordings" / rec, "auto")
    g = dict(np.load(gt_dir / "gt.npz", allow_pickle=True))
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    gt_t, gt_p, gt_R = g["t_ns"][ok], g["p_bc"][ok], g["R_bc"][ok]
    dt = res.get("dt_ns") or 0
    P0 = next((m for m in poses if m.get("R_ItoC0") is not None), None)
    R_ItoC = np.array(P0["R_ItoC0"]).reshape(3, 3)
    p_IinC = np.array(P0["p_IinC0"])

    def gt_at(t):
        t = t + dt
        j = int(np.clip(np.searchsorted(gt_t, t), 1, len(gt_t) - 1))
        j = j - 1 if abs(gt_t[j - 1] - t) < abs(gt_t[j] - t) else j
        return (gt_R[j], gt_p[j]) if abs(gt_t[j] - t) < 2_000_000 else None
    out = []
    for e in edges:
        a, b = gt_at(e["t_old_ns"]), gt_at(e["t_cur_ns"])
        R_oc = quat_R(e["q_oc"])
        t_oc = np.array(e["t_oc"])
        # camera relative pose: T_oldC_curC = T_CI T_oldI_curI T_IC
        R_c = R_ItoC @ R_oc @ R_ItoC.T
        t_c = R_ItoC @ (R_oc @ (-R_ItoC.T @ p_IinC) + t_oc) + p_IinC
        row = dict(cur=e["cur"], old=e["old"], dt_s=round((e["t_cur_ns"] - e["t_old_ns"]) / 1e9, 2), n_inl=e["n_inl"],
                   drift_cm=round(float(np.linalg.norm(e["drift"])) * 100, 2), dyaw_deg=round(float(np.degrees(e["dyaw"])), 3))
        if a is None or b is None:
            row["gt"] = None
        else:
            Rg = a[0].T @ b[0]                     # cam_old <- cam_cur from GT (R_bc = board <- cam)
            tg = a[0].T @ (b[1] - a[1])
            row["gt"] = True
            row["err_cm"] = round(float(np.linalg.norm(t_c - tg)) * 100, 2)
            row["err_deg"] = round(float(np.degrees(np.arccos(np.clip((np.trace(Rg.T @ R_c) - 1) / 2, -1, 1)))), 3)
            row["false"] = bool(row["err_cm"] > 10 or row["err_deg"] > 3)
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
            row = dict(name=name, cond=cond, set=which, rec=r.get("rec"))
            lj = OUT / cond / f"{name}.loop.jsonl"
            L = [json.loads(x) for x in open(lj)] if lj.exists() else []
            row["n_kf"] = sum(1 for x in L if x.get("type") == "kf")
            row["n_loop"] = sum(1 for x in L if x.get("type") == "loop")
            row["n_opt"] = sum(1 for x in L if x.get("type") == "opt")
            ms = [x["ms"][6] for x in L if x.get("type") == "kf"]
            row["kf_ms_p50_p95"] = [round(float(np.percentile(ms, 50)), 2), round(float(np.percentile(ms, 95)), 2)] if ms else None
            if which == "loop":
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
                if pz.exists() and row["n_loop"]:
                    poses = [json.loads(x) for x in gzip.open(pz, "rt")]
                    row["edges"] = loop_edges_vs_gt(r, r["rec"], poses, lj)
                    ch = [e for e in row["edges"] if e.get("gt")]
                    row["edges_checked"] = len(ch)
                    row["edges_false"] = sum(e["false"] for e in ch)
                    row["edge_err_cm_max"] = max((e["err_cm"] for e in ch), default=None)
            rows[f"{cond}/{name}"] = row
    json.dump(dict(made=time.strftime("%F %T"), rows=rows), open(OUT / "score.json", "w"), indent=1)
    (RD / "score.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), out=str(OUT), rows=rows), indent=1))
    for k, v in rows.items():
        print(k, {x: v.get(x) for x in v if x not in ("edges", "name", "rec")})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["freeze", "run", "job", "score"])
    ap.add_argument("arg", nargs="?")
    ap.add_argument("--par", type=int, default=4)
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
