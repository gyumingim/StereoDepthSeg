#!/usr/bin/env python3
"""npunight: collect phone/<tag>/score.json -> summary.json + ABBA decisions (README rules). Prints a table."""
import glob
import json
import os
import re

H = os.path.dirname(os.path.abspath(__file__))
rows = []
for d in sorted(glob.glob(os.path.join(H, "phone", "*"))):
    t = os.path.basename(d)
    sp = os.path.join(d, "score.json")
    st = open(os.path.join(d, "start.txt")).read().strip() if os.path.isfile(os.path.join(d, "start.txt")) else ""
    rm = json.load(open(os.path.join(d, "rep_meta.json"))) if os.path.isfile(os.path.join(d, "rep_meta.json")) else {}
    if not os.path.isfile(sp):
        rows.append(dict(tag=t, error="no score", skipped=os.path.isfile(os.path.join(d, "SKIPPED.txt"))))
        continue
    s = json.load(open(sp))
    n = s.get("npu") or {}
    m = re.search(r"SKIN ([0-9.]+) AP ([0-9.]+)", st)
    rows.append(dict(tag=t, rec=s.get("rec"), npu=rm.get("npu_loop"), extras=rm.get("extras", ""), valid=s.get("valid"),
                     vio_ate=s["vio"].get("ate_cm"), map_ate=s["map"].get("ate_cm"), map_end=s["map"].get("end_cm"),
                     vio_end=s["vio"].get("end_cm"), loops=s["loop"]["n_loop"], correct=s["loop"]["n_loop_correct"], loop_gt=s["loop"]["n_loop_gt"],
                     kf=s["loop"]["n_kf"], ms50=s["vio_ms_p50"], ms95=s["vio_ms_p95"], msmax=s["vio_ms_max"],
                     kf_drop=s.get("kf_pushed_dropped"), late5=(s.get("meta") or {}).get("replay_late_over5ms"),
                     stop=(s.get("meta") or {}).get("stop_reason"), skin0=float(m.group(1)) if m else None, ap0=float(m.group(2)) if m else None,
                     ap_max=n.get("ap_max"), skin_max=n.get("skin_max"),
                     F_prep=n.get("F_prep_ms_p50"), F_npu=n.get("F_npu_ms_p50"), F_post=n.get("F_post_q_ms_p50"), F_tot=n.get("F_tot_ms_p50"),
                     M1_post=n.get("M1_post_ms_p50"), M2_pack=n.get("M2_pack_ms_p50"), M2_npu=n.get("M2_npu_ms_p50"), M2_post=n.get("M2_post_ms_p50"),
                     M1_n=n.get("M1_n_p50"), M2_n=n.get("M2_n_p50"), nF=n.get("n_F"), nM2=n.get("n_M2"), cpus=(s.get("meta") or {}).get("npu_cpus")))
dec = {}
for rec in ("165506", "213341", "214550"):
    R = {r["tag"].split("_")[1]: r for r in rows if r["tag"].startswith(rec + "_") and r.get("map_ate") is not None}
    if not all(k in R for k in ("A1", "B1", "B2", "A2")):
        dec[rec] = "미완료"
        continue
    if not all(R[k]["valid"] for k in ("A1", "B1", "B2", "A2")):
        dec[rec] = "무효 run 있음 → 판정 안 함"
        continue
    a = [R["A1"]["map_ate"], R["A2"]["map_ate"]]
    b = [R["B1"]["map_ate"], R["B2"]["map_ate"]]
    dec[rec] = "NPU 쪽이 낫다" if max(a) < min(b) else "BRIEF 쪽이 낫다" if min(a) > max(b) else "차이 없음"
vals = list(dec.values())
nn, nb = vals.count("NPU 쪽이 낫다"), vals.count("BRIEF 쪽이 낫다")
overall = "NPU 쪽이 낫다 (추정)" if nn >= 2 and nb == 0 else "BRIEF 쪽이 낫다 (추정)" if nb >= 2 and nn == 0 else "차이 보이지 않음"
if any(v in ("미완료", "무효 run 있음 → 판정 안 함") for v in vals):
    overall += " (판정 못 한 녹화 있음)"
json.dump(dict(rows=rows, decision=dec, overall=overall), open(os.path.join(H, "summary.json"), "w"), indent=1, ensure_ascii=False)
for r in rows:
    print(r["tag"], {k: r.get(k) for k in ("valid", "map_ate", "vio_ate", "loops", "correct", "ms50", "ms95", "F_prep", "F_npu", "F_post", "M1_post", "M2_npu", "M2_post", "ap0", "ap_max")})
print(dec, overall)
