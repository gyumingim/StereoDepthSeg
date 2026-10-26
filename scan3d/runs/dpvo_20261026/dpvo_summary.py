#!/usr/bin/env python3
"""dpvo 2026-10-26: comparison table DPVO / DPV-SLAM vs OpenVINS (integ2 0.6.27 profile re-run, GL30/GL60) on the same cells.

Inputs: score.json (dpvo_run.py score) and ov_ref/score_ov_ref.json (ov_ref_run.py score), both next to this file.
HS: 42 cells (14 recordings x offsets 0/1/2; the phone-drop recording is excluded like in ov60eval / integ2).
Per set and fps:
  OpenVINS GL: div count, window ATE (ate_w, SE3, metric scale from the IMU) median over non-divergent cells, Sim3 median.
  DPVO (causal / final): div = divergent under the oracle (full Sim3) scale; ate_w (oracle scale), ate_w_ow (same, cut to
  the OpenVINS tracking window), ate_w_s5 (scale fitted on the first 5 s only, realistic), sim3, |scale drift| 2nd/1st half.
  paired = median of (DPVO - OpenVINS) on the cells where both are non-divergent.
Loops: start-end error / path length (loop_pct, scale free).
"""
import json
import sys
from pathlib import Path

import numpy as np

RD = Path(__file__).resolve().parent
sys.path.insert(0, "/home/system/scanner/StereoDepthSeg/scan3d/imu_learn")
from phone60_run import DROP  # noqa: E402


def med(v):
    v = [x for x in v if x is not None]
    return round(float(np.median(v)), 3) if v else None


def main():
    D = json.load(open(RD / "score.json"))
    O = json.load(open(RD / "ov_ref" / "score_ov_ref.json"))
    ov = {}
    for st in ("hs", "std"):
        for row in O[st]["cells"]:
            for c in ("GL30", "GL60"):
                if c in row:
                    ov[(row["key"], int(c[2:]))] = row[c]
    rows = [r for r in D["rows"] if r["rec"] != DROP]
    out = {"made": D["made"], "ov_made": O["made"], "tables": {}, "loops": {}}
    lines = []
    for which in ("hs", "std"):
        for fps in (30, 60):
            for cfg in ("base", "slam", "p48"):
                rr = [r for r in rows if r["which"] == which and r["fps"] == fps and r["cfg"] == cfg]
                if not rr:
                    continue
                keys = [r["key"] for r in rr]
                o = [ov.get((k, fps)) for k in keys]
                o_ok = [x for x in o if x and not x["div"] and x["ate_w"] is not None]
                T = dict(n=len(rr), ov=dict(n=sum(x is not None for x in o), div=sum(1 for x in o if x and x["div"]),
                                            ate_w_med=med([x["ate_w"] for x in o_ok]), sim3_med=med([x["sim3"] for x in o if x]),
                                            scale_err_med=med([abs(x["scale"] - 1) for x in o if x and x.get("scale")])))
                for kind in ("causal", "final"):
                    d = [r[kind] for r in rr]
                    ok = [x for x in d if not x.get("div") and x.get("ate_w") is not None]
                    pair = [(r[kind]["ate_w"], ov[(r["key"], fps)]["ate_w"]) for r in rr
                            if (r["key"], fps) in ov and not r[kind].get("div") and r[kind].get("ate_w") is not None
                            and not ov[(r["key"], fps)]["div"] and ov[(r["key"], fps)]["ate_w"] is not None]
                    pair_s3 = [(r[kind]["sim3"], ov[(r["key"], fps)]["sim3"]) for r in rr
                               if (r["key"], fps) in ov and r[kind].get("sim3") is not None and ov[(r["key"], fps)].get("sim3") is not None]
                    T[kind] = dict(div=sum(bool(x.get("div")) for x in d), div_cells=[r["key"] for r in rr if r[kind].get("div")],
                                   fail=[r["key"] for r in rr if r.get("error") or not r[kind].get("tracked")],
                                   ate_w_med=med([x["ate_w"] for x in ok]), ate_w_ow_med=med([x.get("ate_w_ow") for x in ok]),
                                   div_ow=sum(bool(x.get("div_ow")) for x in d),
                                   ate_w_s5_med=med([x.get("ate_w_s5") for x in d if x.get("ate_w_s5") is not None and not x.get("div_s5")]),
                                   div_s5=sum(bool(x.get("div_s5")) for x in d),
                                   sim3_med=med([x.get("sim3") for x in d]),
                                   scale_drift_abs_med=med([abs(x["scale_drift"]) for x in d if x.get("scale_drift") is not None]),
                                   s5_over_s_med=med([x.get("s5_over_s") for x in d]),
                                   pair_n=len(pair), pair_d_med=med([a - b for a, b in pair]),
                                   pair_dpvo_better=sum(a < b for a, b in pair),
                                   pair_sim3_n=len(pair_s3), pair_sim3_d_med=med([a - b for a, b in pair_s3]),
                                   pair_sim3_dpvo_better=sum(a < b for a, b in pair_s3))
                T["ms_med_of_cells"] = med([r["ms_med"] for r in rr])
                T["ms_p95_med_of_cells"] = med([r["ms_p95"] for r in rr])
                T["mem_mb_max"] = max(r["mem_mb"] for r in rr)
                T["load1_med"] = med([r["load1"] for r in rr])
                out["tables"][f"{which}{fps}_{cfg}"] = T
                lines.append(f"\n## {which} {fps} fps cfg {cfg} (n {T['n']})  OV GL{fps}: div {T['ov']['div']}, ate_w med {T['ov']['ate_w_med']}, "
                             f"sim3 med {T['ov']['sim3_med']}, |scale-1| med {T['ov']['scale_err_med']}")
                for kind in ("causal", "final"):
                    t = T[kind]
                    lines.append(f"  DPVO {kind}: div {t['div']} {t['div_cells']} fail {t['fail']} | ate_w {t['ate_w_med']} (ow {t['ate_w_ow_med']}, "
                                 f"div_ow {t['div_ow']}) | s5: ate_w {t['ate_w_s5_med']} div {t['div_s5']} | sim3 {t['sim3_med']} | "
                                 f"|drift| {t['scale_drift_abs_med']} | pair ate_w n {t['pair_n']} d_med {t['pair_d_med']} dpvo better {t['pair_dpvo_better']} | "
                                 f"pair sim3 n {t['pair_sim3_n']} d_med {t['pair_sim3_d_med']} better {t['pair_sim3_dpvo_better']}")
                lines.append(f"  time ms med/p95 (median of cells) {T['ms_med_of_cells']}/{T['ms_p95_med_of_cells']}, mem max {T['mem_mb_max']} MB, load1 med {T['load1_med']}")
    lines.append("\n## loops (loop_pct = start-end / path)")
    for r in sorted((r for r in rows if r["which"] == "loop"), key=lambda r: (r["key"], r["fps"], r["cfg"])):
        o = O["loops"].get(r["key"], {}).get(f"GL{r['fps']}", {})
        e = dict(ov=o, causal=r["causal"], final=r["final"], n_loop_ba=r.get("n_loop_ba"))
        out["loops"][f"{r['key']}_f{r['fps']}_{r['cfg']}"] = e
        lines.append(f"  {r['key']} f{r['fps']} {r['cfg']}: OV {o.get('loop_err_cm')} cm / {o.get('path_m')} m = {o.get('loop_pct')} % | "
                     f"DPVO causal {r['causal'].get('loop_pct')} % final {r['final'].get('loop_pct')} % (tracked {r['final'].get('tracked')}/{r['frames']})")
    for r in rows:
        if r["cfg"] == "slam":
            lines.append(f"  slam {r['name']}: loop BA events {r.get('n_loop_ba')}")
    json.dump(out, open(RD / "summary.json", "w"), indent=1)
    txt = "\n".join(lines)
    (RD / "summary.txt").write_text(f"made from score.json {D['made']} / ov {O['made']}\n" + txt + "\n")
    print(txt)


if __name__ == "__main__":
    main()
