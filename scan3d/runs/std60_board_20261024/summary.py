"""2026-10-24: summary over all std60 board recordings (v2 GT, square 60.40 mm): align (R_ci, dt) vs the assumed std60
values (config/phone_ov_main/variants.json = HS chordal mean R_ci, dt -2.17 ms), scale check, GT quality, VIO scores.
  ../venv_ffs/bin/python runs/std60_board_20261024/summary.py <rec names...>  -> runs/std60_board_20261024/summary.json"""
import json
import re
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as Rr

OD = Path("runs/std60_board_20261024")
v = json.load(open("config/phone_ov_main/variants.json"))
A = np.array(v["R_ci"])
DT_A = float(np.median(v["dt_ns"])) / 1e6


def ang(R1, R2):
    return float(np.degrees(np.linalg.norm(Rr.from_matrix(np.asarray(R1) @ np.asarray(R2).T).as_rotvec())))


def chordal_mean(Rs):
    U, _, Vt = np.linalg.svd(np.sum(Rs, 0))
    M = U @ Vt
    if np.linalg.det(M) < 0:
        M = U @ np.diag([1, 1, -1]) @ Vt
    return M


rows, Rs = {}, []
for n in sys.argv[1:]:
    d = OD / n.split("_")[2]
    al = json.load(open(f"recordings/{n}/imu_gt/align.json"))
    sc = json.load(open(d / "scale_spl.json"))[0]
    scr = json.load(open(d / "scale_rs.json"))[0]
    sp = re.findall(r"frames ok (\d+) \(per-frame GT (\d+)\), corner px median ([0-9.]+)", (d / "spline.log").read_text())
    p90 = re.findall(r"corner px median [0-9.]+\s+p90 ([0-9.]+)", (d / "spline.log").read_text())
    ig = re.findall(r"프레임 (\d+), 자세 OK (\d+) .*재투영 중앙 ([0-9.]+) px, 거리 중앙 ([0-9.]+) m", (d / "imu_gt.log").read_text())
    vio = json.load(open(d / "score.json"))["rows"] if (d / "score.json").exists() else {}
    Rs.append(np.array(al["R_ci"]))
    rows[n] = dict(dt_ms=round(al["dt_ns"] / 1e6, 3), d_dt_vs_assumed_ms=round(al["dt_ns"] / 1e6 - DT_A, 3),
                   R_ci_vs_hs_mean_deg=round(ang(al["R_ci"], A), 3), frame_res_deg_med=al.get("frame_res_deg"),
                   frame_res_deg_p90=al.get("frame_res_deg_p90"),
                   frames=int(ig[-1][0]) if ig else None, poses_ok=int(ig[-1][1]) if ig else None,
                   pnp_reproj_px=float(ig[-1][2]) if ig else None, dist_m=float(ig[-1][3]) if ig else None,
                   spline_frames_ok=int(sp[-1][0]) if sp else None, spline_corner_px_med=float(sp[-1][2]) if sp else None,
                   spline_corner_px_p90=float(p90[-1]) if p90 else None,
                   s_fixedlever_spl=sc.get("s_fixedlever"), s_fixedlever_spl_ci=sc.get("s_fixedlever_ci95"),
                   s_fixedlever_rs=scr.get("s_fixedlever"), gt_square_mm=sc.get("gt_square_mm"),
                   vio={k: {q: x.get(q) for q in ("div", "ate_w", "ate_sim3_cm", "scale", "tracked_frac", "t_init_clip_s",
                                                  "n_slam_motion_med", "n_slam_motion_frac0")} for k, x in vio.items()})
M = chordal_mean(np.array(Rs)) if Rs else None
res = dict(made=time.strftime("%F %T"), recs=sys.argv[1:], assumed_dt_ms=DT_A, rows=rows)
if M is not None:
    res.update(std60_R_ci_chordal_mean=M.tolist(), std60_mean_vs_hs_mean_deg=round(ang(M, A), 3),
               std60_spread_deg=[round(ang(R, M), 3) for R in Rs],
               dt_ms_median=float(np.median([r["dt_ms"] for r in rows.values()])),
               dt_ms_all=[r["dt_ms"] for r in rows.values()])
    for c in ("base", "aud", "baseA", "audA"):
        cells = [x for r in rows.values() for k, x in r["vio"].items() if k.endswith("_" + c)]
        ok = [x for x in cells if not x["div"]]
        noinit = [x for x in cells if not x.get("tracked_frac")]
        res[f"vio_{c}"] = dict(n=len(cells), no_init=len(noinit), div=len(cells) - len(ok) - len(noinit),
                               ate_w_med=float(np.median([x["ate_w"] for x in ok])) if ok else None,
                               scale_med=float(np.median([x["scale"] for x in ok])) if ok else None,
                               abs_scale_err_med=float(np.median([abs(x["scale"] - 1) for x in ok])) if ok else None)
json.dump(res, open(OD / "summary.json", "w"), indent=1, ensure_ascii=False)
print(json.dumps({k: v_ for k, v_ in res.items() if k != "rows"}, indent=1))
for n, r in rows.items():
    print(n, {k: v_ for k, v_ in r.items() if k != "vio"})
    for k, x in r["vio"].items():
        print("   ", k, x)
