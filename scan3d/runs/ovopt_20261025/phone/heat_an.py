# Heat proxies per phone run from phone_run.sh samples (samp.txt, every ~5 s) + time from tl.csv.
#   python3 heat_an.py OUT_DIR [...]
# Per run: SKIN / AP start (run.txt) -> mean / max / end over the samples, battery "current now" mean (dumpsys battery;
# negative = discharging, more negative = more power; unit as reported by the phone, mA assumed), mean scaling_cur_freq
# of cpu9 (X4) and of cpu7 (mid A720 domain 7-8), screen state; proc = remap + feed p50 / p95, % > 33 ms.
import csv
import re
import sys
from pathlib import Path

import numpy as np

print("| run | proc p50 / p95 / >33% | SKIN start / mean / max / end | AP start / mean / max / end | current mean (min) | cpu9 / cpu7 MHz mean | screen |")
print("|---|---|---|---|---|---|---|")
for od in sys.argv[1:]:
    for d in sorted(Path(od).iterdir()):
        if not (d / "samp.txt").exists():
            continue
        S = []
        for line in open(d / "samp.txt"):
            if not line.startswith("t "):
                continue
            m = dict(re.findall(r"(\w+) (-?[\d.]+|\w+)", line.split(" freq ")[0]))
            fr = line.split(" freq ")[1].split() if " freq " in line else []
            S.append(dict(skin=float(m.get("skin", "nan")), ap=float(m.get("ap", "nan")),
                          cur=float(m["cur"]) if re.match(r"-?[\d.]+$", m.get("cur", "x")) else np.nan,
                          wake=m.get("wake", "?"), f9=float(fr[9]) / 1000 if len(fr) > 9 else np.nan,
                          f7=float(fr[7]) / 1000 if len(fr) > 7 else np.nan))
        if not S:
            continue
        kv = dict(re.findall(r"(\w+)=('[^']*'|\S+)", open(d / "run.txt").read())) if (d / "run.txt").exists() else {}
        a = lambda k: np.array([s[k] for s in S], float)
        T = list(csv.DictReader(open(d / "tl.csv"))) if (d / "tl.csv").exists() else []
        if T:
            tr = np.array([float(r["init"]) for r in T]) > 0
            proc = np.array([float(r["remap_ms"]) + float(r["feed_ms"]) for r in T])[tr]
            pt = f"{np.percentile(proc, 50):.2f} / {np.percentile(proc, 95):.2f} / {(proc > 33.3).mean() * 100:.1f}"
        else:
            pt = "-"
        sk, ap, cur = a("skin"), a("ap"), a("cur")
        wakes = sorted(set(s["wake"] for s in S))
        print(f"| {d.name} | {pt} | {kv.get('skin_start', '?')} / {np.nanmean(sk):.1f} / {np.nanmax(sk):.1f} / {sk[-1]:.1f} | "
              f"{kv.get('ap_start', '?')} / {np.nanmean(ap):.1f} / {np.nanmax(ap):.1f} / {ap[-1]:.1f} | "
              f"{np.nanmean(cur):.0f} ({np.nanmin(cur):.0f}) | {np.nanmean(a('f9')):.0f} / {np.nanmean(a('f7')):.0f} | {','.join(wakes)} |")
