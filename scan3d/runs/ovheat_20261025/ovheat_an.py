# ovheat phone-run analysis (2026-10-25). Per run dir (tl.csv, samp.txt, run.txt, po.jsonl, pe.txt from ovheat_run.sh):
#   python3 ovheat_an.py [--min] RUN_DIR [...]
# Whole run: proc = remap_ms + feed_ms of tracking frames (init=1) p50 / p95 / max, % > 33.3 ms, lag (receive vs
#   schedule + processing; backlog) p95 / max, frames fed / late-dropped, n_slam median, SKIN / AP start / mean / max,
#   cpu9 cap min / mean, cpu9 + cpu7 clock mean, battery current mean (dumpsys, coarse), bridge CPU s (utime+stime of all
#   threads at the last sample; jiffies = 10 ms) and cores used (= CPU s / run wall s), spin line from pe.txt.
# --min: one row per minute (by receive time): proc p50 / p95 / max, >33 %, lag p95 / max, n_slam, SKIN / AP mean / max,
#   cpu9 cap min, cpu9 clock mean, current mean.
import csv
import json
import re
import sys
from pathlib import Path

import numpy as np

per_min = "--min" in sys.argv
dirs = [a for a in sys.argv[1:] if a != "--min"]


def pc(a, q):
    return float(np.percentile(a, q)) if len(a) else float("nan")


def samples(d):
    S, cpu = [], {}
    cur_t = None
    for line in open(d / "samp.txt"):
        if line.startswith("t "):
            head = line.split(" freq ")[0]
            m = dict(re.findall(r"(\w+) (-?[\d.]+|\w+)", head))
            fr = line.split(" freq ")[1].split() if " freq " in line else []
            num = lambda k: float(m[k]) if re.match(r"-?[\d.]+$", m.get(k, "x")) else np.nan
            cur_t = num("t")
            S.append(dict(t=cur_t, skin=num("skin"), ap=num("ap"), cur=num("cur"), cap9=num("cap9") / 1000,
                          cap7=num("cap7") / 1000, f9=float(fr[9]) / 1000 if len(fr) > 9 else np.nan,
                          f7=float(fr[7]) / 1000 if len(fr) > 7 else np.nan, wake=m.get("wake", "?")))
        elif line.strip().startswith("tid"):
            p = line.split()
            cpu[p[1]] = (int(p[3]) + int(p[5])) / 100.0  # utime + stime, jiffies -> s (last sample wins)
    return S, sum(cpu.values())


def tlrows(d):
    T = list(csv.DictReader(open(d / "tl.csv")))
    f = lambda k: np.array([float(r[k]) for r in T])
    tn = np.array([int(r["t_ns"]) for r in T], dtype=np.int64)
    rx = np.array([int(r["rx_boot_ns"]) for r in T], dtype=np.int64)
    proc = f("remap_ms") + f("feed_ms")
    lag = ((rx - rx[0]) - (tn - tn[0])) / 1e6
    lag = lag - lag.min() + proc + f("wait_ms")
    return dict(init=f("init") > 0, proc=proc, lag=lag, rx_s=(rx - rx[0]) / 1e9, rx_boot=rx, nslam=f("n_slam"))


for dd in dirs:
    d = Path(dd)
    if not (d / "tl.csv").exists():
        continue
    kv = {m.group(1): m.group(2).strip("'") for m in re.finditer(r"(\w+)=('[^']*'|\S+)", open(d / "run.txt").read())} if (d / "run.txt").exists() else {}
    L = tlrows(d)
    tr = L["init"]
    S, cpu_s = samples(d) if (d / "samp.txt").exists() else ([], float("nan"))
    a = lambda k: np.array([s[k] for s in S], float)
    wall = L["rx_s"][-1] if len(L["rx_s"]) else float("nan")
    dropped = None
    resets = 0
    if (d / "po.jsonl").exists():
        last = None
        for x in open(d / "po.jsonl"):
            if '"reset"' in x:
                resets += 1
            if '"pose"' in x:
                last = x
        if last:
            dropped = json.loads(last).get("late_dropped")
    spin = ""
    if (d / "pe.txt").exists():
        for line in open(d / "pe.txt"):
            if "idle spin end" in line or ("idle spin" in line and "waits" in line):
                spin = line.strip()
    p = L["proc"][tr]
    print(f"## {d.name}  ({kv.get('extra', '')}; spin {kv.get('idle_spin')} thr {kv.get('threads')} loops {kv.get('loops')}; "
          f"start {kv.get('start')} end {kv.get('end')} aborted {kv.get('aborted')} screen {kv.get('screen_start')}/{kv.get('screen_end')})")
    print(f"- proc p50/p95/max {pc(p, 50):.2f} / {pc(p, 95):.2f} / {p.max() if len(p) else float('nan'):.1f} ms, >33 ms {(p > 33.3).mean() * 100:.2f} %, "
          f"lag p95/max {pc(L['lag'][tr], 95):.1f} / {L['lag'].max():.1f} ms, fed {len(L['proc'])} (tracking {int(tr.sum())}), "
          f"late_dropped {dropped}, resets {resets}, n_slam med {np.median(L['nslam'][tr]) if tr.any() else float('nan'):.0f}")
    if S:
        print(f"- SKIN start/mean/max/end {kv.get('skin_start')} / {np.nanmean(a('skin')):.1f} / {np.nanmax(a('skin')):.1f} / {a('skin')[-1]:.1f}, "
              f"AP start/mean/max/end {kv.get('ap_start')} / {np.nanmean(a('ap')):.1f} / {np.nanmax(a('ap')):.1f} / {a('ap')[-1]:.1f}, "
              f"cap9 min/mean {np.nanmin(a('cap9')):.0f} / {np.nanmean(a('cap9')):.0f} MHz, f9/f7 mean {np.nanmean(a('f9')):.0f} / {np.nanmean(a('f7')):.0f} MHz, "
              f"current mean {np.nanmean(a('cur')):.0f}, bridge CPU {cpu_s:.1f} s / {wall:.0f} s = {cpu_s / wall:.2f} cores")
    if spin:
        print(f"- {spin}")
    if per_min and len(L["proc"]):
        print("| min | proc p50 / p95 / max | >33 % | lag p95 / max | n_slam | SKIN mean / max | AP mean / max | cap9 min | f9 mean | cur |")
        print("|---|---|---|---|---|---|---|---|---|---|")
        t0 = S[0]["t"] - 5 if S else 0
        boot0 = L["rx_boot"][0] / 1e9
        st = a("t") - t0 if S else np.array([])
        for k in range(int(wall // 60) + 1):
            m = (L["rx_s"] >= 60 * k) & (L["rx_s"] < 60 * (k + 1))
            mt = m & tr
            pp = L["proc"][mt]
            ms = (st >= 60 * k) & (st < 60 * (k + 1)) if S else np.array([], bool)
            g = lambda key, fn: fn(a(key)[ms]) if S and ms.any() else float("nan")
            print(f"| {k} | {pc(pp, 50):.1f} / {pc(pp, 95):.1f} / {pp.max() if len(pp) else float('nan'):.1f} | {(pp > 33.3).mean() * 100 if len(pp) else float('nan'):.1f} | "
                  f"{pc(L['lag'][mt], 95):.0f} / {L['lag'][m].max() if m.any() else float('nan'):.0f} | {np.median(L['nslam'][mt]) if mt.any() else float('nan'):.0f} | "
                  f"{g('skin', np.nanmean):.1f} / {g('skin', np.nanmax):.1f} | {g('ap', np.nanmean):.1f} / {g('ap', np.nanmax):.1f} | "
                  f"{g('cap9', np.nanmin):.0f} | {g('f9', np.nanmean):.0f} | {g('cur', np.nanmean):.0f} |")
    print()
