#!/usr/bin/env python3
"""Re-score stored VIO runs against the old-K GT (imu_gt_spl) and the new-K GT (imu_gt_spl_cal1022), no VIO re-run
(docs/lens_switch_20261023.md section 7). Runs scored:
  lens A/B : runs/lens_switch/<key>_old, <key>_new          (base_nozupt + lever, o0)
  IMU C1   : runs/phone60/<key>_V4 (base) vs runs/imucal/<key>_C1 (6-face accel)
Metrics per run and GT: window ATE + divergence on the camera position (= imucal_run.cam_metrics, zupt_ob_score.run_metrics
with the GT dir swapped) and the bench evaluate() (SE3 ATE, Sim3 ATE, Sim3 scale) re-run on the stored poses.
Check: evaluate() with the old GT must reproduce the stored ate_cm / ate_sim3_cm / scale.
GT quality: spline corner px median/p90 (new GT log vs a re-fit of the old imu_gt_rs with the same spline code), scale_ls
s / s_lever / s_fixedlever for old and new GT (runs/lens_switch/gt/<rec>_scale_{old,new}.json).

  ../venv_ffs/bin/python imu_learn/lens_switch_gtscore.py     # -> runs/lens_switch/gtscore.json
"""
import argparse
import gzip
import json
import os
import re
import shutil
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
os.chdir(SCAN3D)
sys.path.insert(0, str(HERE))
from phone60_run import RAW, OLD, DROP  # noqa: E402
import vio_bench_hs as vb  # noqa: E402
import zupt_ob_score as zs  # noqa: E402

os.chdir(SCAN3D)
OUT = Path("runs/lens_switch")
GTS = {"oldK": "imu_gt_spl", "newK": "imu_gt_spl_cal1022"}
SETS = {"old": "runs/lens_switch/{k}_old", "new": "runs/lens_switch/{k}_new",
        "base": "runs/phone60/{k}_V4", "C1": "runs/imucal/{k}_C1"}


class NpProxy:
    """numpy with load() redirected from <rec>/imu_gt_spl/gt.npz to the chosen GT dir (zupt_ob_score hardcodes it)."""
    def __init__(self, gt):
        self.gt = gt

    def __getattr__(self, k):
        return getattr(np, k)

    def load(self, p, *a, **kw):
        p = str(p)
        if p.endswith("/imu_gt_spl/gt.npz"):
            p = p[:-len("imu_gt_spl/gt.npz")] + self.gt + "/gt.npz"
        return np.load(p, *a, **kw)


def load_poses(pre):
    return [json.loads(x) for x in gzip.open(str(pre) + ".poses.jsonl.gz", "rt") if x.strip()]


def cam_metrics(pre, rec, gt):
    tmp = OUT / "campos_gt"
    tmp.mkdir(exist_ok=True)
    dst = tmp / Path(str(pre)).name
    shutil.copy(str(pre) + ".json", str(dst) + ".json")
    with gzip.open(str(dst) + ".poses.jsonl.gz", "wt") as f:
        for m in load_poses(pre):
            if m.get("T_world_cam0"):
                m["p"] = list(np.asarray(m["T_world_cam0"], float).reshape(4, 4)[:3, 3])
            f.write(json.dumps(m) + "\n")
    zs.np = NpProxy(gt)
    try:
        return zs.run_metrics(dst, rec)
    finally:
        zs.np = np


def bench_eval(pre, rec, gt):
    d = json.load(open(str(pre) + ".json"))
    if d.get("ate_cm") is None:
        return {}
    P = load_poses(pre)
    dt_ns = int(json.load(open(f"recordings/{rec}/imu_gt/align.json"))["dt_ns"])
    a = argparse.Namespace(gt=gt, fps=d["fps"], size=d["size"], npts=d["npts"], clones=d["clones"], noise=d["noise"],
                           imu_calib=d.get("imu_calib"), lever=d.get("lever"), dropout="", imu_raw=d["imu_source"] == "raw_16_35",
                           noise_scale=d.get("noise_scale", 1.0))
    r = vb.evaluate(Path(f"recordings/{rec}"), a, P, d["frames_fed"], [], dt_ns, 0.0, d["noise_used"], None, d["frames"])
    return {k: r.get(k) for k in ("ate_cm", "ate_sim3_cm", "scale", "matched", "gt")}


def gt_quality(rec):
    n = rec
    q = {}
    for tag, log in (("newK", OUT / "gt" / f"{n}_spline.log"), ("oldK_refit", OUT / "gt" / f"{n}_spline_oldK_refit.log")):
        if log.exists():
            m = re.findall(r"corner px median ([0-9.]+)\s+p90 ([0-9.]+)", log.read_text())
            fin = re.findall(r"→ \S+: frames ok (\d+) \(per-frame GT (\d+)\), corner px median ([0-9.]+)", log.read_text())
            q[tag] = dict(corner_px_median=float(m[-1][0]) if m else None, corner_px_p90=float(m[-1][1]) if m else None,
                          frames_ok=int(fin[-1][0]) if fin else None)
    for tag in ("old", "new"):
        f = OUT / "gt" / f"{n}_scale_{tag}.json"
        if f.exists():
            s = json.load(open(f))[0]
            q[f"scale_{tag}"] = {k: s.get(k) for k in ("s", "s_lever", "s_fixedlever", "check", "gt_square_mm",
                                                       "square_mm_raw_fixedlever")}
    # GT size ratio new/old: median |p_bc| over frames ok in both
    try:
        go = np.load(f"recordings/{rec}/imu_gt_spl/gt.npz", allow_pickle=True)
        gn = np.load(f"recordings/{rec}/imu_gt_spl_cal1022/gt.npz", allow_pickle=True)
        if len(go["t_ns"]) == len(gn["t_ns"]) and np.array_equal(go["t_ns"], gn["t_ns"]):
            ok = go["ok"].astype(bool) & gn["ok"].astype(bool)
            r = np.linalg.norm(gn["p_bc"][ok], axis=1) / np.linalg.norm(go["p_bc"][ok], axis=1)
            dz = gn["p_bc"][ok][:, 2] / go["p_bc"][ok][:, 2]
            q["dist_ratio_new_old_median"] = round(float(np.median(r)), 4)
            q["depth_ratio_new_old_median"] = round(float(np.median(dz)), 4)
            q["fx"] = dict(old=round(float(go["K"][0, 0]), 1), new=round(float(gn["K"][0, 0]), 1))
    except FileNotFoundError as e:
        q["error"] = str(e)
    return q


def summarize(rows, a, b, gt):
    use = [x for x in rows.values() if x["rec"] != DROP]
    both = [x for x in use if not x[a][gt]["div"] and not x[b][gt]["div"]]

    def med(c, f):
        v = [f(x[c][gt]) for x in both if x[c][gt].get("scale") is not None]
        return round(float(np.median(v)), 4) if v else None
    d = [x[b][gt]["ate_w"] - x[a][gt]["ate_w"] for x in both]
    return dict(n=len(use), div={a: sum(x[a][gt]["div"] for x in use), b: sum(x[b][gt]["div"] for x in use)},
                n_both_normal=len(both),
                ate_w_med={a: med(a, lambda y: y["ate_w"]), b: med(b, lambda y: y["ate_w"])},
                ate_w_d_med=round(float(np.median(d)), 3) if d else None, n_b_better=int(sum(v < 0 for v in d)),
                sim3_med={a: med(a, lambda y: y["ate_sim3_cm"]), b: med(b, lambda y: y["ate_sim3_cm"])},
                scale_med={a: med(a, lambda y: y["scale"]), b: med(b, lambda y: y["scale"])},
                abs_scale_err_med={a: med(a, lambda y: abs(y["scale"] - 1)), b: med(b, lambda y: abs(y["scale"] - 1))})


def main():
    rows, check, skipped = {}, [], []
    for rec in RAW + OLD:
        key = f"{rec[-6:]}_o0"
        if not Path(f"recordings/{rec}/{GTS['newK']}/gt.npz").exists():
            skipped.append(rec)                  # new GT not built (yet): interim score without it
            continue
        row = dict(rec=rec, key=key, gt_quality=gt_quality(rec))
        for s, pat in SETS.items():
            pre = pat.format(k=key)
            stored = json.load(open(pre + ".json"))
            row[s] = {}
            for g, gd in GTS.items():
                mc = cam_metrics(pre, rec, gd)
                be = bench_eval(pre, rec, gd)
                row[s][g] = dict(div=bool(mc.get("divergent")), ate_w=mc.get("ate_w"), rel5_max=mc.get("rel5_max"), **be)
            o = row[s]["oldK"]
            check.append(all(o.get(k) == stored.get(k) for k in ("ate_cm", "ate_sim3_cm", "scale")))
        rows[key] = row
        print(key, {s: {g: (('*' if v['div'] else '') + str(round(v['ate_w'], 2) if v['ate_w'] is not None else None),
                            v.get('scale')) for g, v in row[s].items()} for s in SETS}, flush=True)
    summ = {f"{a}_vs_{b}": {g: summarize(rows, a, b, g) for g in GTS} for a, b in (("old", "new"), ("base", "C1"))}
    q = [x["gt_quality"] for x in rows.values()]

    def qm(f):
        v = [f(x) for x in q]
        v = [y for y in v if y is not None]
        return round(float(np.median(v)), 4) if v else None
    gq = dict(corner_px_median=dict(oldK_refit=qm(lambda x: x.get("oldK_refit", {}).get("corner_px_median")),
                                    newK=qm(lambda x: x.get("newK", {}).get("corner_px_median"))),
              corner_px_p90=dict(oldK_refit=qm(lambda x: x.get("oldK_refit", {}).get("corner_px_p90")),
                                 newK=qm(lambda x: x.get("newK", {}).get("corner_px_p90"))),
              s_fixedlever=dict(old=qm(lambda x: x.get("scale_old", {}).get("s_fixedlever")),
                                new=qm(lambda x: x.get("scale_new", {}).get("s_fixedlever"))),
              abs_s_fixedlever_minus_1=dict(
                  old=qm(lambda x: abs(x["scale_old"]["s_fixedlever"] - 1) if x.get("scale_old", {}).get("s_fixedlever") else None),
                  new=qm(lambda x: abs(x["scale_new"]["s_fixedlever"] - 1) if x.get("scale_new", {}).get("s_fixedlever") else None)),
              dist_ratio_new_old=qm(lambda x: x.get("dist_ratio_new_old_median")),
              depth_ratio_new_old=qm(lambda x: x.get("depth_ratio_new_old_median")))
    res = dict(made=time.strftime("%F %T"), doc="docs/lens_switch_20261023.md", gts=GTS, sets=SETS,
               excluded_from_summary=DROP, skipped_no_new_gt=skipped, reproduce_stored_with_oldK=f"{sum(check)}/{len(check)}",
               gt_quality_median=gq, summary=summ, rows=rows)
    json.dump(res, open(OUT / "gtscore.json", "w"), indent=1, ensure_ascii=False)
    print(json.dumps(dict(reproduce=res["reproduce_stored_with_oldK"], gt_quality=gq, summary=summ), ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
