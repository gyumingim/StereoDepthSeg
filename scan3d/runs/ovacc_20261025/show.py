"""quick per-cell look: python3 show.py COND[,COND] [cellkey]"""
import json, sys, glob, os
conds = sys.argv[1].split(",")
key = sys.argv[2] if len(sys.argv) > 2 else "*"
for c in conds:
    for f in sorted(glob.glob(f"/tmp/ovacc/ate/{c}/{key}_{c}.json")):
        d = json.load(open(f))
        ap = d.get("ovacc_applied", {})
        print(c, os.path.basename(f)[:9], "ate", d.get("ate_cm"), "trk", d.get("tracked"), "job_s", d.get("ovacc_job_s"),
              "ts_ms", round(ap.get("timeshift", 0) * 1e3, 3), "fx", round(ap.get("fx", 0), 2), "Rdiff", ap.get("live_R_diff_deg"),
              "dtdiff", ap.get("live_dt_minus_rec_ms"), "dt_fin", d.get("dt_cam_imu_final_ms"), "hist", d.get("cfg_over"))
