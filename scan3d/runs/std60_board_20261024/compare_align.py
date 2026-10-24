"""2026-10-24: this recording's batch_sync align vs the ASSUMED std60 values (config/phone_ov_main/variants.json) and vs the
other GT recordings' batch_sync results (frame residual, dt). Writes runs/std60_board_20261024/compare_align.json."""
import glob
import json
import time

import numpy as np
from scipy.spatial.transform import Rotation as Rr

REC = "hs_20261024_165506_std60"
v = json.load(open("config/phone_ov_main/variants.json"))
A = np.array(v["R_ci"])
dtA = v["timeshift_cam_imu_s"] * -1e3 if v["timeshift_cam_imu_s"] > 0 else v["timeshift_cam_imu_s"] * 1e3
dtA = float(np.median(v["dt_ns"])) / 1e6


def ang(R1, R2):
    return float(np.degrees(np.linalg.norm(Rr.from_matrix(np.asarray(R1) @ np.asarray(R2).T).as_rotvec())))


al = json.load(open(f"recordings/{REC}/imu_gt/align.json"))
b2 = json.load(open("runs/std60_board_20261024/batch_sync_it2000.json"))[0]
others = []
for f in sorted(glob.glob("runs/overnight_gt/*_batch_sync.json")):
    d = json.load(open(f))[0]
    others.append(dict(rec=d["rec"].split("/")[-1], dt_ms=round(d["dt_ns"] / 1e6, 3), res_med=round(d["frame_res_deg_med"], 3),
                       res_p90=round(d["frame_res_deg_p90"], 3)))
hs_angles = []
for r in v["recordings"]:
    n = r if isinstance(r, str) else r.get("rec", r.get("name"))
    try:
        hs_angles.append(round(ang(json.load(open(f"recordings/{n}/imu_gt/align.json"))["R_ci"], A), 3))
    except Exception as e:  # noqa: BLE001
        hs_angles.append(str(e)[:60])
res = dict(made=time.strftime("%F %T"), rec=REC, assumed_dt_ms=dtA, assumed_R_ci=A.tolist(),
           align_dt_ms=al["dt_ns"] / 1e6, align_vs_assumed_deg=ang(al["R_ci"], A), d_dt_ms=al["dt_ns"] / 1e6 - dtA,
           it2000_dt_ms=b2["dt_ns"] / 1e6, it2000_res_med=b2["frame_res_deg_med"],
           frame_res_deg_med=al.get("frame_res_deg"), frame_res_deg_p90=al.get("frame_res_deg_p90"),
           inc_res_deg=al.get("inc_res_deg"), hs_R_ci_vs_assumed_deg=hs_angles, hs_dt_ms=[x / 1e6 for x in v["dt_ns"]],
           other_batch_sync=others)
json.dump(res, open("runs/std60_board_20261024/compare_align.json", "w"), indent=1)
print(json.dumps({k: res[k] for k in res if k not in ("other_batch_sync", "assumed_R_ci")}, indent=1))
for o in others:
    print(o)
