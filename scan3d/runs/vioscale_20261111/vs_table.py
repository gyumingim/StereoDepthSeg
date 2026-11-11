#!/usr/bin/env python3
"""vioscale: print/collect the scores of one batch dir (+ rel.json if present).  python vs_table.py OUT_DIR [rel.json] [--md]"""
import json, sys
from pathlib import Path

out = Path(sys.argv[1])
rel = json.load(open(sys.argv[2]))["conds"] if len(sys.argv) > 2 and sys.argv[2] != "--md" else {}
rows = {}
for l in open(out / "scores.jsonl"):
    d = json.loads(l); rows[d["cond"]] = d
conds = json.load(open(out / "conds.json"))["conds"]
for c, d in rows.items():
    r = d["rulers"]; red = r.get("red", {}); im = r.get("imu", {}).get("dtvio", {}) if "dtvio" in r.get("imu", {}) else {}
    bd = r.get("board", {}); rl = rel.get(c, {})
    print(f"{c:12s} {str(d['status']):6s} init {d['init_s']} path {d['path_m']} loop {d['loop_err_cm']} | "
          f"rel {rl.get('rel')} s_est {rl.get('s_est')} | red {red.get('s')} rms {red.get('rms_px')} "
          f"h {red.get('s_first_half')}/{red.get('s_second_half')} | board fwd {bd.get('k_fwd')} rev {bd.get('k_rev')} ate {bd.get('ate_se3_cm')} | "
          f"imu {im.get('s_fixedlever')} {im.get('s_fixedlever_ci95')} g {im.get('g_norm_fixedlever')} "
          f"{(r.get('imu', {}).get('error') or '')[-80:]}  args {' '.join(conds.get(c, []))}")
