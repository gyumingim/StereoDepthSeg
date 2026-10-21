#!/usr/bin/env python3
"""Self-check for docs/init_window_result.html: recompute the main tables from the raw run files WITHOUT the analysis code
path (own log parse for the init frame, bench ate_cm / scale for the trajectory rule, motion onset from summary 'still'
only) and compare with runs/init_window/summary.json. Prints mismatches and exits 1 if any label or count differs.

  ../venv_ffs/bin/python imu_learn/c5_initwin_check.py runs/init_window
"""
import gzip
import json
import re
import sys
from collections import Counter
from pathlib import Path

NAME = re.compile(r"^(L0|L2)_S(\d)_(W05|W10|W15)_(D224|D5|D15)_(\d{6})_(dm025|d0|dp025)$")


def main(d):
    d = Path(d)
    summ = json.load(open(d / "summary.json"))
    onset = {rec[-6:]: s["fine_s"] for rec, s in summ["still"].items()}
    lab_s = {r["name"]: r for r in summ["runs"]}
    mine, bad = {}, []
    for f in sorted(d.glob("*.json")):
        m = NAME.match(f.stem)
        if not m:
            continue
        j = json.load(open(f))
        ti = None
        last = None
        with gzip.open(str(f)[:-5] + ".stderr.log.gz", "rt", errors="replace") as fh:
            for line in fh:
                if "[DIAGT]" in line:
                    last = float(line.rsplit("]", 1)[1])
                elif "successful initialization" in line:
                    ti = last
                    break
        with gzip.open(str(f)[:-5] + ".poses.jsonl.gz", "rt") as fh:
            t0 = json.loads(fh.readline())["t"]
        still = onset[m[5]] - j["start_off"]
        if ti is None:
            lab = "noinit"
        elif ti - t0 > still + 0.1:
            lab = "late"
        else:
            a, s = j.get("ate_cm"), j.get("scale")
            lab = "success" if (a is not None and s is not None and a <= 10 and 0.5 <= s <= 2) else "collapse"
        mine[f.stem] = lab
        if f.stem not in lab_s:
            bad.append(f"{f.stem}: not in summary")
        elif lab_s[f.stem]["label"] != lab:
            r = lab_s[f.stem]
            bad.append(f"{f.stem}: check {lab} vs summary {r['label']} (bench ate {j.get('ate_cm')} scale {j.get('scale')}, "
                       f"summary full ate {r.get('ate_full_cm')} scale {r.get('scale_full')})")
    # coarse table (L0, d0) and per-setting collapse totals
    cnt = Counter()
    for n, lab in mine.items():
        m = NAME.match(n)
        if m[1] == "L0" and m[6] == "d0":
            cnt[(f"{m[3]}|{m[4]}", "n")] += 1
            cnt[(f"{m[3]}|{m[4]}", lab + f"_S{m[2]}")] += 1
        cnt[(f"{m[3]}|{m[4]}", "all_collapse")] += lab == "collapse"
    for k, v in summ["coarse"].items():
        mc = (cnt[(k, "n")], cnt[(k, "collapse_S1")] + cnt[(k, "collapse_S2")], cnt[(k, "success_S2")], cnt[(k, "success_S1")])
        sc = (v["n"], v["collapse"], v["succ_L0_S2"], v["succ_L0_S1"])
        print(f"coarse {k}: check {mc} summary {sc} {'OK' if mc == sc else 'DIFF'}")
        if mc != sc:
            bad.append(f"coarse {k} {mc} != {sc}")
    for k, v in summ["settings"].items():
        if cnt[(k, "all_collapse")] != v["collapse"]:
            bad.append(f"setting {k} collapse {cnt[(k, 'all_collapse')]} != {v['collapse']}")
    print(f"runs checked {len(mine)}, summary runs {len(lab_s)}, label counts {dict(Counter(mine.values()))}")
    print("MISMATCHES:" if bad else "ALL MATCH", *bad, sep="\n  ")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "runs/init_window"))
