#!/usr/bin/env python3
"""Self-check for docs/setdown_mitigation.html, independent of setdown_mitig.analyse_one where possible:
  1. the YAML the bridge actually read (ov_config_files in each run JSON) has the condition's ZUPT values;
  2. end-still drift straight from the pose stream (|p(t1) - p(t0)| of the IMU-clock end-still window);
  3. IMU-only ZUPT accepts (a 'failed disparity' line followed by 'accepted' before the next frame) by a plain line scan;
  4. summary cell medians re-derived from the per-run rows.
  ../venv_ffs/bin/python imu_learn/setdown_mitig_check.py > runs/setdown_mitig/check.txt
"""
import gzip
import json
import re

import numpy as np

S = json.load(open("runs/setdown_mitig/summary.json"))
ES = json.load(open("runs/endstill_tmp/endstill_check.json"))
WANT = {"base": {"zupt_chi2_multipler": "0", "zupt_noise_multiplier": "10"},
        "Z1": {"zupt_chi2_multipler": "1", "zupt_noise_multiplier": "10"},
        "Z2": {"zupt_chi2_multipler": "1", "zupt_noise_multiplier": "1"},
        "Z3": {"zupt_chi2_multipler": "5", "zupt_noise_multiplier": "10"},
        "H": {"zupt_chi2_multipler": "0", "zupt_noise_multiplier": "10"}}
bad = 0
for r in S["runs"]:
    p = "runs/setdown_mitig/" + r["name"]
    d = json.load(open(p + ".json"))
    y = d["ov_config_files"]["estimator_config.yaml"]
    got = {k: re.search(rf"^{k}:\s*([^#\s]+)", y, re.M)[1] for k in WANT[r["cond"]]}
    ok_cfg = got == WANT[r["cond"]] and (("still_hold" in d) == (r["cond"] == "H"))
    e = ES["recs"][d["rec"]]
    t0 = int(e["t0_ns"])
    w0, w1 = (t0 + int(x * 1e9) for x in e["used"]["end"])
    P = [json.loads(x) for x in gzip.open(p + ".poses.jsonl.gz", "rt")]
    P = [m for m in P if m.get("status") == "tracking" and m.get("p") is not None and w0 <= m["t_ns"] <= w1]
    drift = float(np.linalg.norm(np.subtract(P[-1]["p"], P[0]["p"])) * 100)
    n_io, dfail = 0, False
    for line in gzip.open(p + ".stderr.log.gz", "rt", errors="replace"):
        if "[DIAGT]" in line:
            dfail = False
        elif "failed disparity" in line:
            dfail = True
        elif "[ZUPT]: accepted" in line and dfail:
            n_io += 1
    ok_d = abs(drift - r["drift_cm"]) < 0.05
    bad += (not ok_cfg) + (not ok_d)
    print(f"{r['name']}: cfg {got} hold {'still_hold' in d} {'OK' if ok_cfg else 'MISMATCH'}; end-still drift {drift:.2f} "
          f"vs {r['drift_cm']:.2f} {'OK' if ok_d else 'MISMATCH'}; IMU-only ZUPT accepts (whole run) {n_io}")
for k, c in S["cells"].items():
    rec, cond = k.split("|")
    rs = [r for r in S["runs"] if r["rec"].endswith(rec) and r["cond"] == cond]
    for m in ("lastwin_med_cm", "drift_cm", "ate100_cm"):
        v = float(np.median([r[m] for r in rs]))
        if abs(v - c[m]) > 1e-9:
            bad += 1
            print(f"cell {k} {m} {v} != {c[m]}")
print(f"cells re-derived: {len(S['cells'])}; runs {len(S['runs'])}; mismatches {bad}")
