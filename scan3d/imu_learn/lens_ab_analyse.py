#!/usr/bin/env python3
"""Analysis of the lens A/B (docs/lens_ab_plan.md, criteria exactly as pre-registered).

  ../venv_ffs/bin/python imu_learn/lens_ab_analyse.py      # -> runs/lens_ab/summary.json (+ table on stdout)
Collapse / wipe-out definitions are the zupt_fixA ones (functions imported from imu_learn/zupt_fixA_analyse.py).
"""
import gzip
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
sys.path.insert(0, str(HERE))
from zupt_fixA_analyse import parse, updates, wipeouts, poses  # noqa: E402

OUT = SCAN3D / "runs" / "lens_ab"
RECS = ["111836", "133820", "134204", "134550"]


def info(name):
    pre = OUT / name
    if not Path(str(pre) + ".json").exists():
        return None
    j = json.load(open(str(pre) + ".json"))
    base = dict(name=name, lens=j.get("lab_lens"), gt=j.get("gt"), head=j.get("lab_head"), dirty=j.get("lab_dirty"))
    if j.get("error") or j.get("ate_cm") is None or not Path(str(pre) + ".stderr.log.gz").exists():
        return dict(base, result=False, error=j.get("error"), ate_cm=j.get("ate_cm"), collapse=True)
    up = updates(parse(str(pre) + ".stderr.log.gz"))
    P, vt, vp = poses(pre)
    wd = []
    for t in wipeouts(up):
        m = (vt >= t) & (vt <= t + 5)
        d = float(np.max(np.linalg.norm(vp[m] - vp[m][0], axis=1))) if m.sum() >= 2 else None
        wd.append(dict(t=round(t, 3), maxdisp5_m=None if d is None else round(d, 3), collapse=bool(d is not None and d > 0.5)))
    return dict(base, result=True, ate_cm=j["ate_cm"], ate_sim3_cm=j.get("ate_sim3_cm"), scale=j.get("scale"),
                t_init=j.get("t_init_clip_s"), wipeouts=wd,
                collapse=bool(any(w["collapse"] for w in wd) or j["ate_cm"] > 50))


def same_poses(a, b):
    pa = [json.loads(x) for x in gzip.open(str(OUT / a) + ".poses.jsonl.gz", "rt") if x.strip()]
    pb = [json.loads(x) for x in gzip.open(str(OUT / b) + ".poses.jsonl.gz", "rt") if x.strip()]
    keys = ("t", "status", "p", "q", "v")
    return len(pa) == len(pb) and all(x.get(k) == y.get(k) for x, y in zip(pa, pb) for k in keys)


def med(x):
    x = [v for v in x if v is not None]
    return round(float(np.median(x)), 3) if x else None


def main():
    rows, missing = [], []
    for r in RECS:
        for k in range(3):
            c = f"L_{r}_o{k}"
            A, B = info(f"{c}_A"), info(f"{c}_B")
            if A is None or B is None:
                missing.append(c)
                continue
            d = (B["ate_cm"] - A["ate_cm"]) if (A["result"] and B["result"]) else None
            rows.append(dict(cell=c, rec=r, A=A, B=B, d_ate_cm=None if d is None else round(d, 3),
                             d_sim3_cm=(round(B["ate_sim3_cm"] - A["ate_sim3_cm"], 3)
                                        if A["result"] and B["result"] and A.get("ate_sim3_cm") is not None
                                        and B.get("ate_sim3_cm") is not None else None),
                             new_collapse=bool(B["collapse"] and not A["collapse"]),
                             lost_collapse=bool(A["collapse"] and not B["collapse"])))
    ds = [x["d_ate_cm"] for x in rows]
    md = med(ds)
    n_new = sum(x["new_collapse"] for x in rows)
    complete = not missing and len(rows) == 12
    c1 = md is not None and md <= 0
    c2 = n_new == 0
    verdict = ("incomplete" if not complete else "B adopted" if (c1 and c2) else "B not adopted (keep A)")
    per_rec = {r: dict(d_median=med([x["d_ate_cm"] for x in rows if x["rec"] == r]),
                       A_ate=[x["A"]["ate_cm"] for x in rows if x["rec"] == r],
                       B_ate=[x["B"]["ate_cm"] for x in rows if x["rec"] == r],
                       A_sim3=[x["A"].get("ate_sim3_cm") for x in rows if x["rec"] == r],
                       B_sim3=[x["B"].get("ate_sim3_cm") for x in rows if x["rec"] == r],
                       A_scale=[x["A"].get("scale") for x in rows if x["rec"] == r],
                       B_scale=[x["B"].get("scale") for x in rows if x["rec"] == r]) for r in RECS}
    # GT sensitivity (134204 with GT rebuilt from lens B)
    gts = []
    for k in range(3):
        c = f"L_134204_o{k}"
        A2, B2 = info(f"{c}_A_gtB"), info(f"{c}_B_gtB")
        if A2 is None or B2 is None:
            continue
        gts.append(dict(cell=c, A_gtB=A2, B_gtB=B2,
                        d_gtB=round(B2["ate_cm"] - A2["ate_cm"], 3) if A2["result"] and B2["result"] else None,
                        poses_same_as_main=dict(A=same_poses(f"{c}_A", f"{c}_A_gtB"), B=same_poses(f"{c}_B", f"{c}_B_gtB"))))
    d_gtA_134204 = med([x["d_ate_cm"] for x in rows if x["rec"] == "134204"])
    d_gtB_134204 = med([x["d_gtB"] for x in gts]) if len(gts) == 3 else None
    gt_flag = None
    if d_gtA_134204 is not None and d_gtB_134204 is not None:
        gt_flag = ("GT 의존" if (d_gtA_134204 <= 0) != (d_gtB_134204 <= 0) else "GT 렌즈에 둔감 (134204 한 녹화 기준)")
    scale_chk = {}
    for L in ("A", "B"):
        f = OUT / f"gt_134204_scale_spl_lens{L}.json"
        if f.exists():
            s = json.load(open(f))[0]
            scale_chk[L] = {k: s.get(k) for k in ("s", "s_lever", "s_fixedlever", "check", "gt_square_mm", "square_mm_raw_fixedlever")}
    ident = json.load(open(OUT / "ident.json")) if (OUT / "ident.json").exists() else None
    res = dict(made=time.strftime("%F %T"), plan="docs/lens_ab_plan.md", n_cells=len(rows), missing=missing,
               identity_gate=ident, criteria=dict(median_d_ate_cm=md, c1_median_le_0=c1, new_collapses=n_new, c2_no_new=c2),
               verdict=verdict, n_d_neg=sum(1 for v in ds if v is not None and v < 0),
               n_d_pos=sum(1 for v in ds if v is not None and v > 0), n_d_zero=sum(1 for v in ds if v == 0),
               median_A_ate=med([x["A"]["ate_cm"] for x in rows]), median_B_ate=med([x["B"]["ate_cm"] for x in rows]),
               median_A_sim3=med([x["A"].get("ate_sim3_cm") for x in rows]),
               median_B_sim3=med([x["B"].get("ate_sim3_cm") for x in rows]),
               median_d_sim3=med([x["d_sim3_cm"] for x in rows]),
               median_A_scale=med([x["A"].get("scale") for x in rows]), median_B_scale=med([x["B"].get("scale") for x in rows]),
               collapse_cells_A=[x["cell"] for x in rows if x["A"]["collapse"]],
               collapse_cells_B=[x["cell"] for x in rows if x["B"]["collapse"]],
               per_rec=per_rec, rows=rows,
               gt_sensitivity=dict(rec="hs_20261022_134204", gt_B="imu_gt_spl_lensB", cells=gts,
                                   d_median_gtA=d_gtA_134204, d_median_gtB=d_gtB_134204, flag=gt_flag, scale_check=scale_chk))
    json.dump(res, open(OUT / "summary.json", "w"), indent=1, ensure_ascii=False)
    print(f"verdict: {verdict}  median d {md} cm  new collapses {n_new}  (d<0 {res['n_d_neg']}, d>0 {res['n_d_pos']})")
    print(f"{'cell':16s} {'A ate':>7s} {'B ate':>7s} {'d':>7s} {'A sim3':>7s} {'B sim3':>7s} {'A sc':>6s} {'B sc':>6s} colA colB")
    for x in rows:
        A, B = x["A"], x["B"]
        print(f"{x['cell']:16s} {A['ate_cm']!s:>7} {B['ate_cm']!s:>7} {x['d_ate_cm']!s:>7} {A.get('ate_sim3_cm')!s:>7} "
              f"{B.get('ate_sim3_cm')!s:>7} {A.get('scale')!s:>6} {B.get('scale')!s:>6} {int(A['collapse'])}    {int(B['collapse'])}")
    print("GT sensitivity:", json.dumps(res["gt_sensitivity"]["d_median_gtA"]), json.dumps(d_gtB_134204), gt_flag)
    for g in gts:
        print(f"  {g['cell']}: A_gtB {g['A_gtB']['ate_cm']} sim3 {g['A_gtB'].get('ate_sim3_cm')} sc {g['A_gtB'].get('scale')} | "
              f"B_gtB {g['B_gtB']['ate_cm']} sim3 {g['B_gtB'].get('ate_sim3_cm')} sc {g['B_gtB'].get('scale')} | d {g['d_gtB']} "
              f"same poses {g['poses_same_as_main']}")
    print("scale check:", json.dumps(scale_chk))


if __name__ == "__main__":
    main()
