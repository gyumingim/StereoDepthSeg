#!/usr/bin/env python3
"""nncrun 2026-10-27: phone NPU output (13 eval frames, 1x130x44x80 f32 each) vs PC fp32 reference of the ORIGINAL model
on the raw input (ref_eval_fp32.bin). Same metrics as acc_nnc.py (acc_lib.xf_compare, heat ch129 = logit -> sigmoid).
  python acc2.py PHONE_OUT.bin OUT.json [LABEL]"""
import json, sys, time
import numpy as np
sys.path.insert(0, "/home/system/scanner/StereoDepthSeg-ovxfeat/scan3d/runs/npu_20261027")
from acc_lib import xf_compare
D = "/home/system/scanner/tmp_npu/nncrun_data"
ph = np.fromfile(sys.argv[1], np.float32).reshape(-1, 130, 44, 80)
ref = np.fromfile(f"{D}/ref_eval_fp32.bin", np.float32).reshape(-1, 130, 44, 80)
assert len(ph) == len(ref)
sig = lambda x: 1 / (1 + np.exp(-x))
snr = lambda r, t: float(10 * np.log10((r.astype(np.float64) ** 2).sum() / max(1e-30, ((r - t).astype(np.float64) ** 2).sum())))
split = lambda o: (o[:64], o[64:129], sig(o[129]))
rows = []
for i in range(len(ph)):
    q = xf_compare(split(ref[i]), split(ph[i]))
    rows.append({"frame": i, "snr_all_db": snr(ref[i], ph[i]), "snr_feats_db": snr(ref[i][:64], ph[i][:64]),
                 "snr_kpts_db": snr(ref[i][64:129], ph[i][64:129]), "overlap": q[0], "cos_med": q[1], "cos_min": q[4]})
summ = {k: float(np.median([r[k] for r in rows])) for k in ("snr_all_db", "snr_feats_db", "snr_kpts_db", "overlap", "cos_med")}
summ["overlap_min"] = float(min(r["overlap"] for r in rows)); summ["overlap_max"] = float(max(r["overlap"] for r in rows))
res = {"made": time.strftime("%F %T"), "label": sys.argv[3] if len(sys.argv) > 3 else "", "phone_out": sys.argv[1],
       "ref": f"{D}/ref_eval_fp32.bin (PC ORT fp32, original model, raw input)", "median": summ, "frames": rows}
print(json.dumps(summ))
json.dump(res, open(sys.argv[2], "w"), indent=1)
