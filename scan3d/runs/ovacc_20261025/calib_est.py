"""What does OpenVINS' online calibration converge to? Per cell of a condition: final R_ItoC0 vs the recording's GT
alignment R_ci (fed prior) and vs the live mean R_ci; final dt_cam_imu (absolute, = prior + online correction) and the
implied total camera-IMU offset (align dt + online dt); p_IinC0 vs the lever arm.
  python3 calib_est.py COND [std]
"""
import gzip, json, os, sys, glob
import numpy as np
sys.path.insert(0, "/home/system/scanner/StereoDepthSeg/scan3d/imu_learn")
os.chdir("/home/system/scanner/StereoDepthSeg/scan3d")
cond = sys.argv[1]
vj = json.load(open("/home/system/scanner/StereoDepthSeg-ovacc/scan3d/config/phone_ov_main/variants.json"))
Rl = np.array(vj["R_ci"])
Rs = np.array(json.load(open("runs/std60_board_20261024/summary.json"))["std60_R_ci_chordal_mean"])
lever = np.array(json.load(open("config/imu_cam_lever.json"))["r_cm"])


def ang(A, B):
    return float(np.degrees(np.arccos(np.clip((np.trace(A.T @ B) - 1) / 2, -1, 1))))


out = []
for f in sorted(glob.glob(f"/tmp/ovacc/ate/{cond}/*_{cond}.json")):
    d = json.load(open(f))
    rec = d["rec"]
    al = json.load(open(f"recordings/{rec}/imu_gt/align.json"))
    P = [json.loads(x) for x in gzip.open(f[:-5] + ".poses.jsonl.gz", "rt") if x.strip()]
    trk = [m for m in P if m.get("status") == "tracking" and m.get("R_ItoC0")]
    if not trk:
        out.append((os.path.basename(f)[:9], None))
        continue
    m = trk[-1]
    R_ItoC = np.array(m["R_ItoC0"]).reshape(3, 3)       # = R_ci (IMU -> camera)
    Ra = np.array(al["R_ci"])
    p = np.array(m["p_IinC0"]) * 100
    tot = al["dt_ns"] * 1e-6 - m["dt_cam_imu"] * 1e3 if "dt_ns" in al else None
    out.append((os.path.basename(f)[:9], dict(R_vs_align=round(ang(R_ItoC, Ra), 3), R_vs_live=round(ang(R_ItoC, Rl), 3),
               R_vs_std60mean=round(ang(R_ItoC, Rs), 3), align_vs_live=round(ang(Ra, Rl), 3),
               dt_online_ms=round(m["dt_cam_imu"] * 1e3, 3), align_dt_ms=round(al["dt_ns"] * 1e-6, 3),
               p_IinC_cm=np.round(p, 2).tolist(), ate=d.get("ate_cm"), trk_s=round(m["t"] - trk[0]["t"], 1))))
for k, v in out:
    print(k, v)
ok = [v for _, v in out if v]
if ok:
    for key in ("R_vs_align", "R_vs_live", "R_vs_std60mean", "align_vs_live", "dt_online_ms"):
        a = np.array([v[key] for v in ok])
        print(f"{key}: median {np.median(a):.3f} p10 {np.percentile(a, 10):.3f} p90 {np.percentile(a, 90):.3f}")
    pp = np.array([v["p_IinC_cm"] for v in ok])
    print("p_IinC cm median", np.round(np.median(pp, 0), 2).tolist(), "lever r_cm", lever.tolist())
