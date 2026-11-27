#!/usr/bin/env python3
"""gsmap 2026-11-27 dry run: 121801 decimated to 30 fps -> VIO (c4f60) + motion keyframes (3 cm / 4 deg) + pair PLANNING only
(no FFS, no training, no mesh). Compares with the same chain at 60 fps (same machine, same code) and the official 60 fps plan.

  python dryrun30.py D        (D = scan3d/runs/dryrun30_121801_20261127, after scan.sh made D/scan30 and D/scan60)

Planner inputs per variant: dump_ba/cams.npz written here = keyframes of that scan, keyframe pose = its VIO pose (NO BA: the dry run
does not run kf_ba), no covisibility points (no FFS/MoGe in a new run -> make_plan_all keeps every candidate). To show what these two
simplifications cost, the official 60 fps BA keyframes are planned once more without covisibility points (B60_ba_nocovis).
"""
import gzip, json, subprocess, sys, time
from collections import Counter
from pathlib import Path

import numpy as np

D = Path(sys.argv[1]).resolve()
REPO = Path.home() / "scanner/StereoDepthSeg-gsmap"
PY = str(Path.home() / "scanner/gsmap_env/venv/bin/python")
P2 = REPO / "scan3d/gsmap/gsm_ffs_pairs2.py"; P3 = REPO / "scan3d/gsmap/gsm_ffs_pairs3.py"
OFF_RUN = REPO / "scan3d/runs/gsmap_20261125_121801"; OFF_PLAN = REPO / "scan3d/runs/gsmap_20261125_121801_v2/ffs_pairs2/plan.json"
OFF_VIO = Path.home() / "scanner/StereoDepthSeg-fixes/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan/vio/B_o0.poses.jsonl.gz"
OFF_RED = OFF_VIO.parent.parent / "red/hs_20261112_121801_std60_kf"
LEG = ["--min-perp", "40", "--fb-perp", "30", "--dir-deg", "10", "--n-pairs", "4", "--k", "2"]     # = run_ffs_pairs2.sh


def poses(p):
    t, T = [], []
    for line in gzip.open(p, "rt"):
        d = json.loads(line)
        if d.get("type") == "pose" and d.get("T_world_cam0") and d.get("status") == "tracking":
            t.append(int(d["t_ns"])); T.append(np.array(d["T_world_cam0"], float).reshape(4, 4))
    return np.array(t), np.array(T)


def n_rows(p):
    return sum(1 for line in gzip.open(p, "rt") if '"type": "pose"' in line or '"type":"pose"' in line)


def umeyama(A, B, scale):
    ma, mb = A.mean(0), B.mean(0); U, S, Vt = np.linalg.svd((B - mb).T @ (A - ma))
    Dm = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]); R = U @ Dm @ Vt
    s = float(np.trace(np.diag(S) @ Dm) / ((A - ma) ** 2).sum()) if scale else 1.0
    return s, R, mb - s * R @ ma


def ate(pa, pb, name):
    """pa: (t, T) evaluated, pb: reference. Match equal t_ns, rigid SE(3) (and Sim(3)) fit of positions, RMS in cm."""
    ta, Ta = pa; tb, Tb = pb; ib = {t: i for i, t in enumerate(tb)}
    m = [(i, ib[t]) for i, t in enumerate(ta) if t in ib]
    A = np.array([Ta[i][:3, 3] for i, _ in m]); B = np.array([Tb[j][:3, 3] for _, j in m])
    out = dict(name=name, n_common=len(m), n_eval=len(ta), n_ref=len(tb))
    for sc in (False, True):
        s, R, t = umeyama(A, B, sc); E = (s * A @ R.T + t) - B; e = np.linalg.norm(E, axis=1) * 100
        rot = [np.degrees(np.arccos(np.clip((np.trace((R @ Ta[i][:3, :3]).T @ Tb[j][:3, :3]) - 1) / 2, -1, 1))) for i, j in m]
        out["sim3" if sc else "se3"] = dict(scale=round(s, 5), rms_cm=round(float(np.sqrt((e ** 2).mean())), 2), median_cm=round(float(np.median(e)), 2),
                                            max_cm=round(float(e.max()), 2), rot_median_deg=round(float(np.median(rot)), 3), rot_p95_deg=round(float(np.percentile(rot, 95)), 3))
    return out


def path_len(T):
    p = T[:, :3, 3]; return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def make_cams(name, red, rec, vio):
    man = json.loads((red / "manifest.json").read_text()); kfl = [int(x) for x in man["keyframes"]]
    g = np.load(rec / "lensonly" / "gt.npz"); t_ns = g["t_ns"]; K = g["K"].copy()
    tv, Tv = poses(vio); itv = {t: i for i, t in enumerate(tv)}
    T = []; miss = 0
    for f in kfl:
        i = itv.get(int(t_ns[f]))
        if i is None:                      # nearest within 8 ms (same rule as load_common)
            k = int(np.argmin(np.abs(tv - t_ns[f]))); i = k if abs(tv[k] - t_ns[f]) < 8e6 else None
        if i is None:
            miss += 1; T.append(np.eye(4))
        else:
            T.append(Tv[i])
    run = D / f"plan_{name}"; (run / "dump_ba").mkdir(parents=True, exist_ok=True)
    Kr = K.copy(); Kr[:2] *= 0.5
    np.savez(run / "dump_ba" / "cams.npz", kf=np.array(kfl), T=np.array(T), t_ns=t_ns[kfl], Kr=Kr, K=K, dist=g["dist"], size=np.array([960, 540]),
             made=np.array(time.strftime("%F %T")), poses=np.array(f"VIO {vio} (no BA)"))
    return run, dict(n_kf=len(kfl), kf_without_vio_pose=miss, kf_select=man.get("kf_select"), frames_in_lensonly=int(len(t_ns)))


def plan(run, rec, vio, out, extra, cmd="plan", script=P2):
    out.mkdir(parents=True, exist_ok=True); t0 = time.time()
    a = [PY, "-W", "ignore", str(script), cmd, str(run), "--vio", str(vio), "--rec", str(rec), "--out", str(out)] + extra
    r = subprocess.run(["nice", "-n", "19"] + a, cwd=REPO, capture_output=True, text=True)
    (out / "plan.log").write_text(r.stdout + "\n--- stderr\n" + r.stderr)
    if r.returncode:
        raise SystemExit(f"plan failed {out}: {r.stderr[-2000:]}")
    return time.time() - t0, [l for l in r.stderr.splitlines() if l.startswith("[")]


def summ(pj, n_kf=None):
    d = json.loads(Path(pj).read_text()); c = d["chosen"]; n = c["n_kf"]
    src = Counter(q[1] for v in c["plan"].values() for q in v)
    strict = {k: [q for q in v if q[1] != "fallback"] for k, v in c["plan"].items()}
    return dict(n_kf=n, ge1=c["ge1"], ge2=c["ge2"], zero=n - c["ge1"], ge1_pct=round(100 * c["ge1"] / n, 1), ge2_pct=round(100 * c["ge2"] / n, 1),
                zero_pct=round(100 * (n - c["ge1"]) / n, 1), fallback_kf=len(c["fallback_kf"]), pairs=c["n_pairs_total"], pair_src=dict(src),
                strict_ge2=sum(len(v) >= 2 for v in strict.values()), strict_ge2_pct=round(100 * sum(len(v) >= 2 for v in strict.values()) / n, 1),
                rate=d.get("rate"), covis_pts_src=d["whole_video"]["covis_pts_src"])


def main():
    t00 = time.time(); res = dict(made=time.strftime("%F %T"), script=str(Path(__file__).resolve()), dir=str(D))
    S30, S60 = D / "scan30", D / "scan60"
    R30, R60 = D / "rec_hs_20261112_121801_std30", D / "rec_hs_20261112_121801_std60"
    V30, V60 = S30 / "vio/B_o0.poses.jsonl.gz", S60 / "vio/B_o0.poses.jsonl.gz"
    RED30, RED60 = S30 / f"red/{R30.name}_kf", S60 / f"red/{R60.name}_kf"
    res["inputs"] = dict(rec30=str(R30), rec60=str(R60), vio30=str(V30), vio60_desktop=str(V60), vio60_official=str(OFF_VIO),
                         official_plan=str(OFF_PLAN), official_run=str(OFF_RUN), vio_cfg="build_livepose/cfg_gen/c4f60 (run_scanner default, track_frequency 60)",
                         video_check=json.loads((D / "video_check.json").read_text()))
    # ---- VIO
    p30, p60, poff = poses(V30), poses(V60), poses(OFF_VIO)
    vj = {k: json.loads((p.parent / "B_o0.json").read_text()) for k, p in (("30", V30), ("60_desktop", V60), ("60_official", OFF_VIO))}
    res["vio"] = {k: dict(fed=v.get("frames_fed"), poses=v.get("poses"), tracked=v.get("score", {}).get("tracked"), tracked_frac=v.get("score", {}).get("tracked_frac"),
                          selection=v.get("input_path", {}).get("selection"), wall_s=v.get("wall_s"), crashed=v.get("crashed"), bridge_cmd_profile="rec60" if "rec60" in v.get("bridge_cmd", "") else "?")
                  for k, v in vj.items()}
    for k, p in (("30", p30), ("60_desktop", p60), ("60_official", poff)):
        res["vio"][k]["n_tracking_poses"] = int(len(p[0])); res["vio"][k]["path_m"] = round(path_len(p[1]), 3)
        res["vio"][k]["first_tracking_s"] = round((p[0][0] - poff[0][0]) * 1e-9, 3) if len(p[0]) else None
    res["ate"] = [ate(p30, poff, "30fps vs 60fps official (laptop 11-12)"), ate(p60, poff, "60fps desktop rerun vs 60fps official"),
                  ate(p30, p60, "30fps vs 60fps desktop rerun")]
    # ---- keyframes + plans
    run30, k30 = make_cams("30", RED30, R30, V30); run60, k60 = make_cams("60", RED60, R60, V60)
    kofs = json.loads((OFF_RED / "manifest.json").read_text())
    res["keyframes"] = {"30": k30, "60_desktop": k60, "60_official": dict(n_kf=kofs["n_keyframes"], kf_select=kofs["kf_select"]),
                        "same_kf_60_desktop_vs_official": json.loads((RED60 / "manifest.json").read_text())["keyframes"] == kofs["keyframes"]}
    nb = D / "plan_B60_ba_nocovis"; (nb).mkdir(exist_ok=True)
    if not (nb / "dump_ba").exists():
        (nb / "dump_ba").symlink_to(OFF_RUN / "dump_ba")
    variants = [("A60_vio_legacy", run60, R60, V60, LEG), ("A60_vio_new", run60, R60, V60, LEG + ["--profile", "new"]),
                ("A30_vio_legacy", run30, R30, V30, LEG), ("A30_vio_new", run30, R30, V30, LEG + ["--profile", "new"]),
                ("B60_ba_nocovis_legacy", nb, OFF_RUN / "rec", OFF_VIO, LEG)]
    res["plans"] = {"60_official (BA poses + covisibility, ffs_pairs2/plan.json)": summ(OFF_PLAN)}
    for name, run, rec, vio, ex in variants:
        o = D / "plans" / name; dt, notes = plan(run, rec, vio, o, ex)
        s = summ(o / "plan.json"); s["plan_s"] = round(dt, 1); s["notes"] = notes; res["plans"][name] = s
    # pairs3 low tier (a backdoor) at 60 / 30, legacy
    res["pairs3_low_tier_legacy"] = {}
    for name, run, rec, vio in (("60_vio", run60, R60, V60), ("30_vio", run30, R30, V30)):
        o = D / "plans" / f"P3_{name}_legacy"; plan(run, rec, vio, o, LEG + ["--p2", str(D / "plans" / f"A{name.split('_')[0]}_vio_legacy")], script=P3)
        d3 = json.loads((o / "plan3.json").read_text())
        res["pairs3_low_tier_legacy"][name] = dict(n_no_pair_base=d3["n_no_pair_base"], low_kf=d3["low"]["kf"], low_pairs=d3["low"]["pairs"],
                                                   fwd_total_ge1=d3["fwd_total_ge1"], base_plan_matches_p2=d3["base_plan_matches_p2"])
    res["total_s"] = round(time.time() - t00, 1)
    (D / "dryrun30.json").write_text(json.dumps(res, indent=1, default=float))
    print(json.dumps(res, indent=1, default=float))


if __name__ == "__main__":
    main()
