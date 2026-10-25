#!/usr/bin/env python3
"""integ2 2026-10-26 (copy of ovupd_20261026/phone/ph_an.py, grouping t_N/t_O): per-run phone replay table.
  python3 ph_an.py OUT.md RUNDIR(out_V1) [group-regex for A/B pairing]
Per run: tl.csv feed_ms (all frames after first n_slam>0... no: all frames with init==1) p50/p95/max, frames>33 ms,
prof.csv SLAM-frame parts p50 (slam_ekf, ekf_ma, ekf_s, ekf_cov, ekf_sym, msckf_ekf), battery plugged/status/temp,
cpu9 cap at start + min/max during run (samp.txt), SKIN start/max, AP max, aborted."""
import csv, os, re, sys, glob, datetime
import numpy as np

out, rd = sys.argv[1], sys.argv[2]

def kv(path):
    s = open(path).read() if os.path.exists(path) else ""
    return dict(re.findall(r"(\w+)=('[^']*'|\S+)", s))

def batt(path):
    if not os.path.exists(path):
        return "-"
    t = open(path).read()
    g = lambda k: (re.search(rf"^\s*{k}: (\S+)", t, re.M) or [None, "?"])[1]
    pw = "AC" if g("AC powered") == "true" else ("USB" if g("USB powered") == "true" else "bat")
    return f"{pw} st{g('status')} lvl{g('level')} T{int(g('temperature'))/10 if g('temperature').isdigit() else '?'}"

def pct(v, q):
    return float(np.percentile(v, q)) if len(v) else float("nan")

rows, cond = [], {}
for d in sorted(glob.glob(os.path.join(rd, "*")), key=os.path.getmtime):
    if not os.path.isdir(d):
        continue
    tag = os.path.basename(d)
    run = kv(os.path.join(d, "run.txt"))
    tl = os.path.join(d, "tl.csv")
    if not os.path.exists(tl):
        rows.append((tag, None)); continue
    T = [r for r in csv.DictReader(open(tl)) if r.get("init") == "1"]
    f = np.array([float(r["feed_ms"]) for r in T])
    rm = np.array([float(r["remap_ms"]) for r in T])
    pr = os.path.join(d, "prof.csv")
    P = [r for r in csv.DictReader(open(pr)) if float(r.get("n_slam", 0) or 0) > 0] if os.path.exists(pr) else []
    parts = {}
    for k in ["slam_ekf", "ekf_ma", "ekf_s", "ekf_cov", "ekf_sym", "msckf_ekf", "delay_init"]:
        v = np.array([float(r[k]) for r in P]) if P and k in P[0] else np.array([])
        parts[k] = (pct(v, 50), v.mean() if len(v) else float("nan"))
    caps, skins, aps, curs, bst = [], [], [], [], set()
    sp = os.path.join(d, "samp.txt")
    if os.path.exists(sp):
        for ln in open(sp):
            if ln.startswith("t "):
                m = dict(zip(ln.split()[0::2], ln.split()[1::2]))
                try:
                    caps.append(int(m["cap9"]) // 1000); skins.append(float(m["skin"])); aps.append(float(m["ap"]))
                    curs.append(int(m["cur"])); bst.add(m["bst"])
                except (KeyError, ValueError):
                    pass
    r = dict(tag=tag, n=len(f), mean=f.mean() if len(f) else float("nan"), p50=pct(f, 50), p95=pct(f, 95), mx=f.max() if len(f) else float("nan"),
             o33=int((f > 33).sum()), rm50=pct(rm, 50), e2e95=pct(f + rm, 95), parts=parts, nslam=len(P),
             cap0=int(run.get("cpu9_cap_start", "0")) // 1000, capmin=min(caps) if caps else 0,
             capmax=max(caps) if caps else 0, capmed=int(np.median(caps)) if caps else 0, skin0=run.get("skin_start"), skinmax=max(skins) if skins else None,
             apmax=max(aps) if aps else None, cur=np.median(curs) if curs else None, bst=",".join(sorted(bst)),
             batt=batt(os.path.join(d, "battery_start.txt")), ab=run.get("aborted"), start=run.get("start"),
             scr=run.get("screen_start"), extra=run.get("extra", ""))
    rows.append((tag, r))

L = [f"# phone replay {os.path.basename(rd)} (made {datetime.datetime.now():%Y-%m-%d %H:%M}, input {rd})", "",
     "feed_ms = VIO processing per frame (tl.csv, frames with init=1); parts = p50 on frames with n_slam>0 (prof.csv).",
     "cpu9 cap = big-core scaling_max_freq (MHz): start (run.txt, one read) / median / min / max during run (samp.txt every 5 s); cpuinfo_max_freq read 3207 MHz at 01:27 (one read). CAP = median below 3207.", "",
     "| run | start | n | feed mean | feed p50 | p95 | max | >33 ms | slam_ekf p50 | ekf_ma | ekf_s | ekf_cov | ekf_sym | msckf_ekf | cpu9 cap start/med/min/max | SKIN start/max | AP max | battery | cur med mA | aborted |",
     "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
for tag, r in rows:
    if r is None:
        L.append(f"| {tag} | no tl.csv |" + " |" * 16); continue
    p = r["parts"]
    flag = " CAP" if r["capmed"] and r["capmed"] < 3207 else ""
    L.append(f"| {tag} | {r['start']} | {r['n']} | {r['mean']:.2f} | {r['p50']:.2f} | {r['p95']:.2f} | {r['mx']:.1f} | {r['o33']} | "
             f"{p['slam_ekf'][0]:.2f} | {p['ekf_ma'][0]:.2f} | {p['ekf_s'][0]:.2f} | {p['ekf_cov'][0]:.2f} | {p['ekf_sym'][0]:.2f} | {p['msckf_ekf'][0]:.2f} | "
             f"{r['cap0']}/{r['capmed']}/{r['capmin']}/{r['capmax']}{flag} | {r['skin0']}/{r['skinmax']} | {r['apmax']} | {r['batt']} bst{r['bst']} | {r['cur']} | {r['ab']} |")
# condition means (A vs B etc.)
grp = {}
for tag, r in rows:
    if r is None or r["ab"] == "1":
        continue
    m = re.match(r"t_([A-Za-z]+)_r\d", tag)  # integ2: t_N_rK / t_O_rK (c4 = max_slam 50)
    if m:
        grp.setdefault(("s50", m[1]), []).append(r)
L += ["", "## per condition (feed mean/p50/p95 and parts = mean of per-run values; max = max over runs; >33 ms = sum over runs; not aborted)", "",
      "| max_slam | cond | runs | feed mean | feed p50 | p95 | max | >33 ms | slam_ekf p50 | ekf_ma | ekf_cov | cpu9 cap median (each run) |", "|---|---|---|---|---|---|---|---|---|---|---|---|"]
for (s, c), rs in sorted(grp.items()):
    mn = lambda k: np.mean([x[k] for x in rs])
    mp = lambda k: np.mean([x["parts"][k][0] for x in rs])
    mm = lambda k: np.mean([x["parts"][k][1] for x in rs])
    L.append(f"| {s} | {c} | {len(rs)} | {mn('mean'):.2f} | {mn('p50'):.2f} | {mn('p95'):.2f} | {max(x['mx'] for x in rs):.1f} | {sum(x['o33'] for x in rs)} | "
             f"{mp('slam_ekf'):.2f} (mean {mm('slam_ekf'):.2f}) | {mp('ekf_ma'):.2f} | {mp('ekf_cov'):.2f} | {', '.join(str(x['capmed']) for x in rs)} |")
open(out, "w").write("\n".join(L) + "\n")
print("\n".join(L))
