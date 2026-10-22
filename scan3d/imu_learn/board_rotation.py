#!/usr/bin/env python3
"""Board 90 deg rotation test 2026-10-22 (docs/board_rotation_plan.md): is the slow (>30 s) VIO-minus-GT error tied to
the board axes (GT) or to gravity (VIO)?

  run     : i3-confirm base VIO (z2_isolate job path, strict static init 1.0 s / 15 px, raw IMU, 640x360, 60 fps,
            OpenVINS threads 2, still-hold off, gyro_rebias=False explicitly), offsets 0/1/2 s, 3 recordings
  job     : one run (internal; patches vio_bench_hs.run like z2_isolate.job and sets a.gyro_rebias = False)
  analyse : metric + decision -> runs/board_rotation/analysis.json
Run from scan3d/:  ../venv_ffs/bin/python imu_learn/board_rotation.py run | analyse
"""
import gzip
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from ate_floor import lowpass, match, umeyama  # noqa: E402

PY = "../venv_ffs/bin/python"
OUT = Path("runs/board_rotation")
RECS = {"hs_20261022_133820": "rot90", "hs_20261022_134204": "rot90", "hs_20261022_134550": "control"}
OFFS = [0.0, 1.0, 2.0]
THREADS = "2"
MARGIN = 1.5


def args_for(rec, off):
    return [f"recordings/{rec}", "--init-cfg", "strict", "--init-window", "1.0", "--init-disp", "15", "--clones", "11",
            "--noise", "cfg", "--start-off", str(off), "--imu-lead", "2.0", "--max-secs", "0", "--decode-threads", "2",
            "--imu-raw"]


def name_of(rec, k):
    return f"{rec[-6:]}_base_o{k}"


def job(argv):
    import c5_run
    import vio_bench_hs as vb
    orig = vb.run

    def run(a):
        a.cfg_over = f"num_opencv_threads={THREADS}"
        a.still_hold = None
        a.gyro_rebias = False                     # explicit: comparable with runs before 2026-10-22
        return orig(a)
    vb.run = run
    sys.argv = ["c5_run.py"] + argv
    c5_run.main()
    out = sys.argv[sys.argv.index("--out") + 1]
    d = json.load(open(out + ".json"))
    head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    blob = subprocess.run(["git", "hash-object", "imu_learn/board_rotation.py"], capture_output=True, text=True).stdout.strip()
    d.update(br_runner="imu_learn/board_rotation.py", br_head=head, br_blob=blob, br_gyro_rebias_set=False,
             br_cfg_over=f"num_opencv_threads={THREADS}")
    json.dump(d, open(out + ".json", "w"), indent=1, ensure_ascii=False)


def run_all():
    OUT.mkdir(parents=True, exist_ok=True)
    procs = []
    for rec in RECS:
        for k, off in enumerate(OFFS):
            n = name_of(rec, k)
            if (OUT / f"{n}.json").exists():
                continue
            cmd = [PY, "imu_learn/board_rotation.py", "job", *args_for(rec, off), "--out", str(OUT / n)]
            print(time.strftime("%T"), "start", n, " ".join(cmd), flush=True)
            procs.append((n, subprocess.Popen(["nice", "-n", "5"] + cmd, stdout=subprocess.DEVNULL,
                                              stderr=open(OUT / f"{n}.err", "w"))))
            time.sleep(0.5)
    for n, p in procs:
        print(time.strftime("%T"), "done", n, p.wait(), flush=True)


# ------------------------------------------------------------------------------------------------------- analyse
def gravity_dir(rec):
    """unit gravity-up direction in the GT board frame: mean of R_bc R_ci a_raw over GT-ok frames."""
    from imu_gt import read_imu
    R = Path("recordings") / rec
    al = json.load(open(R / "imu_gt" / "align.json"))
    g = np.load(R / "imu_gt_spl" / "gt.npz", allow_pickle=True)
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    t, Rb = g["t_ns"][ok], g["R_bc"][ok]
    acc, _ = read_imu(R / "imu.csv", raw=True)
    ta = acc[:, 0].astype(np.int64) + int(al["dt_ns"])
    a = np.stack([np.interp(t, ta, acc[:, 1 + j]) for j in range(3)], 1)
    f = np.einsum("nij,jk,nk->ni", Rb, np.asarray(al["R_ci"]), a)
    m = f.mean(0)
    # cross-check (scale_ls stores only |g|): start still only (first 3.5 s of GT-ok frames)
    s = f[(t - t[0]) < 3.5e9].mean(0)
    ang = float(np.degrees(np.arccos(np.clip(m @ s / np.linalg.norm(m) / np.linalg.norm(s), -1, 1))))
    return m / np.linalg.norm(m), float(np.linalg.norm(m)), int(ok.sum()), round(ang, 2)


def fractions(L, u):
    tot = float((L ** 2).sum())
    return float(((L @ u) ** 2).sum() / tot)


def run_metric(rec, jf, ug):
    j = json.load(open(jf))
    if j.get("error") or not j.get("ate_cm"):
        return dict(status="no result", error=j.get("error"))
    collapsed = j["ate_cm"] > 10 or not (0.5 <= j["scale"] <= 2)
    poses = [json.loads(x) for x in gzip.open(jf.parent / (jf.stem + ".poses.jsonl.gz"), "rt") if x.strip()]
    M = match(Path("recordings") / rec, j, poses, "imu_gt_spl")
    P, Pg, t = M["P"], M["Pg"], M["t"]
    Rs, c, ts = umeyama(P, Pg, scale=True)
    es = c * P @ Rs.T + ts - Pg
    R, _, tt = umeyama(P, Pg)
    e3 = P @ R.T + tt - Pg
    ex, ey, ez = np.eye(3)
    o = dict(status="collapsed" if collapsed else "ok", bench_ate=j["ate_cm"], bench_sim3=j["ate_sim3_cm"],
             scale=j["scale"], gyro_rebias=j.get("gyro_rebias"), n=len(P), dur_s=round(float(t[-1] - t[0]), 1),
             ate_sim3_cm=round(float(np.sqrt((es ** 2).sum(1).mean())) * 100, 3))
    for tag, err in (("sim3", es), ("se3", e3)):
        for lab, fc in (("L30", 1 / 30), ("L5", 0.2)):
            L = lowpass(t, err, fc)
            d = dict(rms_cm=round(float(np.sqrt((L ** 2).sum(1).mean())) * 100, 3),
                     f_v=fractions(L, ug), f_x=fractions(L, ex), f_y=fractions(L, ey), f_z=fractions(L, ez))
            # horizontal axes of the gravity frame: board axes projected off u_g (two orthonormal)
            h1 = np.cross(ug, ez) if abs(ug @ ez) < 0.9 else np.cross(ug, ex)
            h1 /= np.linalg.norm(h1)
            h2 = np.cross(ug, h1)
            d.update(f_h1=fractions(L, h1), f_h2=fractions(L, h2))
            d["axis_rms_cm"] = dict(board=[round(float(np.sqrt(np.mean(L[:, k] ** 2))) * 100, 3) for k in range(3)],
                                    vert=round(float(np.sqrt(np.mean((L @ ug) ** 2))) * 100, 3))
            d = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in d.items()}
            d["r_y_over_v"] = round(d["f_y"] / d["f_v"], 3) if d["f_v"] > 0 else None
            o[f"{tag}_{lab}"] = d
            if tag == "sim3" and lab == "L30":
                o["_L30"] = (M["vt"], L)
    # GT motion extent per axis (confound check)
    Pc = Pg - Pg.mean(0)
    o["gt_motion_sd_cm"] = dict(board=[round(float(Pc[:, k].std()) * 100, 2) for k in range(3)],
                                vert=round(float((Pc @ ug).std()) * 100, 2))
    return o


def analyse():
    res = dict(made=time.strftime("%F %T"), plan="docs/board_rotation_plan.md", gt="imu_gt_spl", recs={})
    for rec, cond in RECS.items():
        ug, gnorm, nok, ang_still = gravity_dir(rec)
        ang = float(np.degrees(np.arccos(abs(ug[1]))))       # angle between board y and gravity axis (0..90)
        R = Path("recordings") / rec
        g = np.load(R / "imu_gt_rs" / "gt.npz", allow_pickle=True)
        okr = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
        sc = {}
        sf = OUT / "gt" / f"{rec}_scale_spl.json"
        if sf.exists():
            d = json.load(open(sf))[0]
            sc = {k: d.get(k) for k in ("s", "s_ci95", "s_lever", "s_fixedlever", "s_fixedlever_ci95", "g_norm_fixedlever",
                                        "check", "windows")}
        r = dict(cond=cond, u_g_board=[round(float(v), 4) for v in ug], spec_force_norm=round(gnorm, 4), n_gt_ok=nok,
                 angle_ug_all_vs_start_still_deg=ang_still,
                 angle_boardy_gravity_deg=round(ang, 2), gt_rs_ok_share=round(float(okr.mean()), 4), scale_check=sc,
                 runs={})
        commons = []
        for k in range(len(OFFS)):
            jf = OUT / f"{name_of(rec, k)}.json"
            if not jf.exists():
                r["runs"][f"o{k}"] = dict(status="missing")
                continue
            o = run_metric(rec, jf, ug)
            Lc = o.pop("_L30", None)
            if o["status"] == "ok" and Lc is not None:
                commons.append(Lc)
            r["runs"][f"o{k}"] = o
        good = [o for o in r["runs"].values() if o.get("status") == "ok"]
        r["n_ok"] = len(good)
        for tag in ("sim3_L30", "sim3_L5", "se3_L30", "se3_L5"):
            if good:
                r[tag + "_median"] = {f: round(float(np.median([o[tag][f] for o in good])), 4)
                                      for f in ("rms_cm", "f_v", "f_x", "f_y", "f_z", "f_h1", "f_h2")}
                m = r[tag + "_median"]
                m["r_y_over_v"] = round(m["f_y"] / m["f_v"], 3)
        # common component across offsets (mean of L30 on common VIO times, nearest within 2 ms)
        if len(commons) >= 2:
            t0 = commons[0][0]
            Ls = [commons[0][1]]
            keep = np.ones(len(t0), bool)
            for vt, L in commons[1:]:
                i = np.clip(np.searchsorted(vt, t0), 1, len(vt) - 1)
                i = np.where(np.abs(vt[i - 1] - t0) < np.abs(vt[i] - t0), i - 1, i)
                keep &= np.abs(vt[i] - t0) < 2_000_000
                Ls.append(L[i])
            C = np.mean(Ls, 0)[keep]
            r["common_L30"] = dict(rms_cm=round(float(np.sqrt((C ** 2).sum(1).mean())) * 100, 3),
                                   f_v=round(fractions(C, ug), 4), f_y=round(fractions(C, np.eye(3)[1]), 4),
                                   f_x=round(fractions(C, np.eye(3)[0]), 4), n=int(keep.sum()))
        res["recs"][rec] = r
    res["decision"] = decide(res["recs"])
    OUT.mkdir(parents=True, exist_ok=True)
    json.dump(res, open(OUT / "analysis.json", "w"), indent=1, ensure_ascii=False)
    print(json.dumps(res["decision"], ensure_ascii=False, indent=1))
    for rec, r in res["recs"].items():
        print(rec, r["cond"], "ang", r["angle_boardy_gravity_deg"], "n_ok", r["n_ok"], "L30", r.get("sim3_L30_median"),
              "L5", r.get("sim3_L5_median"), "common", r.get("common_L30"))


def decide(recs, key="sim3_L30_median"):
    why = []
    rot = [r for r in recs.values() if r["cond"] == "rot90"]
    ctl = [r for r in recs.values() if r["cond"] == "control"][0]
    for r in rot:
        if not (70 <= r["angle_boardy_gravity_deg"] <= 90):
            why.append(f"rotated board y-gravity angle {r['angle_boardy_gravity_deg']} not 90+-20")
    if ctl["angle_boardy_gravity_deg"] > 15:
        why.append(f"control angle {ctl['angle_boardy_gravity_deg']} > 15")
    for r in list(recs.values()):
        if r["gt_rs_ok_share"] < 0.5:
            why.append(f"GT ok share {r['gt_rs_ok_share']} < 0.5")
        if r["n_ok"] < 2:
            why.append(f"{r['cond']} only {r['n_ok']} non-collapsed runs")
    cm = ctl.get(key)
    if cm:
        hz = [cm["f_h1"], cm["f_h2"]]
        ctl_ok = cm["f_v"] >= 0.5 and all(cm["f_v"] >= MARGIN * h for h in hz)
        if not ctl_ok:
            why.append(f"control does not reproduce (f_v {cm['f_v']}, horizontal {hz})")
    ratios = [r[key]["r_y_over_v"] for r in rot if r.get(key)]
    out = dict(metric=key, rotated_r_y_over_v=ratios, control=cm, invalid_reasons=why)
    if why:
        out["verdict"] = "시험 무효 (미판정)"
    elif len(ratios) == 2 and all(x > MARGIN for x in ratios):
        out["verdict"] = "GT 쪽"
    elif len(ratios) == 2 and all(x < 1 / MARGIN for x in ratios):
        out["verdict"] = "VIO 쪽"
    else:
        out["verdict"] = "미판정"
    rms = [r[key]["rms_cm"] for r in rot if r.get(key)]
    out["weak_small_slow"] = any(x < 0.1 for x in rms)
    k5 = key.replace("L30", "L5")
    out["L5_ratios"] = [r[k5]["r_y_over_v"] for r in rot if r.get(k5)]
    return out


if __name__ == "__main__":
    c = sys.argv[1]
    if c == "job":
        job(sys.argv[2:])
    elif c == "run":
        run_all()
    elif c == "analyse":
        analyse()
