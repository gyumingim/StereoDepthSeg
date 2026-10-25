#!/usr/bin/env python3
"""timing0626: awake phone replay A/B (N=new rec20261025 profile, O=old ovopt). python3 tw_an.py OUT.md RUNDIR...
Per run: feed_ms (tl.csv, init==1) p50/p95/mean/max/>33; SLAM update = slam_jac+slam_ekf and slam_ekf alone (prof.csv, frames n_slam>0) p50/p95;
suspend check: rel rx lag = (rx_boot_ns/1e6 - t*1e3) - min over run; 'persist' = max over run of the 5 s rolling MIN of rel lag
(transient backlog after slow frames is removed by the rolling min; a suspend leaves a permanent step) -> VALID if persist < 200 ms,
all samp.txt wake=Awake, foreground only NotificationShade/launcher (no user app; fg sampled every 5 s from W1b on, start/end only before), midrun_not_awake=0, not aborted. Wall = rt_end-rt_start (s). Pairs in list order (ABBA): N_rk vs O_rk."""
import csv, os, re, sys, glob, datetime, math
import numpy as np
out, dirs = sys.argv[1], sys.argv[2:]
def kv(p):
    s = open(p).read() if os.path.exists(p) else ""
    return dict(re.findall(r"(\w+)=('[^']*'|\S+)", s))
def batt(p):
    t = open(p).read() if os.path.exists(p) else ""
    g = lambda k: (re.search(rf"^\s*{k}: (\S+)", t, re.M) or [None, "?"])[1]
    pw = "AC" if g("AC powered") == "true" else ("USB" if g("USB powered") == "true" else "bat")
    return f"{pw} st{g('status')} T{int(g('temperature'))/10 if g('temperature').isdigit() else '?'}"
P = lambda v, q: float(np.percentile(v, q)) if len(v) else float("nan")
R = {}
for rd in dirs:
    for d in sorted(x for x in glob.glob(os.path.join(rd, "w_*")) if os.path.isdir(x)):
        tag = os.path.basename(d); run = kv(os.path.join(d, "run.txt"))
        T = list(csv.DictReader(open(os.path.join(d, "tl.csv"))))
        I = [r for r in T if r["init"] == "1"]
        f = np.array([float(r["feed_ms"]) for r in I])
        pr = [r for r in csv.DictReader(open(os.path.join(d, "prof.csv"))) if float(r["n_slam"] or 0) > 0]
        su = np.array([float(r["slam_jac"]) + float(r["slam_ekf"]) for r in pr]); se = np.array([float(r["slam_ekf"]) for r in pr])
        tt = np.array([float(r["t"]) for r in T if r["rx_boot_ns"] not in ("", "0")])
        lg = np.array([float(r["rx_boot_ns"]) / 1e6 - float(r["t"]) * 1e3 for r in T if r["rx_boot_ns"] not in ("", "0")]); lg -= lg.min()
        rm = [lg[(tt >= x) & (tt < x + 5)].min() for x in np.arange(tt[0], tt[-1] - 5, 1.0)]
        persist = max(rm) if rm else float("nan")
        wk, caps, sk, ap, fgs = set(), [], [], [], {run.get("fg_start"), run.get("fg_end")}
        for ln in open(os.path.join(d, "samp.txt")):
            if ln.startswith("t "):
                a = ln.split(); m = dict(zip(a[0::2], a[1::2]))
                wk.add(m.get("wake")); fgs.add(m.get("fg", run.get("fg_start"))); 
                try: caps.append(int(m["cap9"]) // 1000); sk.append(float(m["skin"])); ap.append(float(m["ap"]))
                except (KeyError, ValueError): pass
        wall = float(run.get("rt_end", "nan")) - float(run.get("rt_start", "nan"))
        valid = persist < 200 and wk == {"Awake"} and run.get("midrun_not_awake") == "0" and run.get("aborted") == "0" and all(x and ("NotificationShade" in x or "launcher" in x) for x in fgs)
        R[tag] = dict(n=len(f), nT=len(T), p50=P(f, 50), p95=P(f, 95), mean=f.mean(), mx=f.max(), o33=int((f > 33).sum()),
                      su50=P(su, 50), su95=P(su, 95), se50=P(se, 50), se95=P(se, 95), nsl=len(pr), persist=persist, lagmax=lg.max(), wall=wall,
                      wake=",".join(sorted(map(str, wk))), cap0=int(run.get("cpu9_cap_start", 0)) // 1000, capmed=int(np.median(caps)) if caps else 0,
                      capmin=min(caps) if caps else 0, capmax=max(caps) if caps else 0, cap1=int(run.get("cpu9_cap_end", 0)) // 1000,
                      skin0=run.get("skin_start"), skinmax=max(sk) if sk else None, apmax=max(ap) if ap else None, batt=batt(os.path.join(d, "battery_start.txt")),
                      start=run.get("start"), valid=valid, nw=run.get("midrun_not_awake"), fg=",".join(sorted(x.split("/")[0] for x in fgs if x)))
order = sorted(R, key=lambda t: R[t]["start"])
L = [f"# timing0626 awake replay A/B (made {datetime.datetime.now():%Y-%m-%d %H:%M}, input {' '.join(dirs)})", "", __doc__.strip(), "",
     "| run | start | frames (init) | feed p50 | p95 | mean | max | >33 | SLAM upd p50 | p95 | slam_ekf p50 | p95 | lag persist ms | lag max ms | wall s | wake | fg (start/samples/end) | cpu9 cap start/med/min/max/end | SKIN start/max | AP max | battery | valid |",
     "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
for t in order:
    r = R[t]
    L.append(f"| {t} | {r['start']} | {r['nT']} ({r['n']}) | {r['p50']:.2f} | {r['p95']:.2f} | {r['mean']:.2f} | {r['mx']:.1f} | {r['o33']} | {r['su50']:.2f} | {r['su95']:.2f} | {r['se50']:.2f} | {r['se95']:.2f} | {r['persist']:.0f} | {r['lagmax']:.0f} | {r['wall']:.1f} | {r['wake']} | {r['fg']} | {r['cap0']}/{r['capmed']}/{r['capmin']}/{r['capmax']}/{r['cap1']} | {r['skin0']}/{r['skinmax']} | {r['apmax']} | {r['batt']} | {'yes' if r['valid'] else 'NO'} |")
pairs = []
for t in order:
    m = re.match(r"w_N_r(\d+)", t)
    if m and f"w_O_r{m[1]}" in R: pairs.append((t, f"w_O_r{m[1]}"))
L += ["", "## paired N - O (pair k = w_N_rk vs w_O_rk, adjacent in ABBA; only pairs where both runs valid)", "",
      "| metric | per-pair N-O (ms) | mean | sd | 95% CI (t, n-1) | N mean / O mean | rel |", "|---|---|---|---|---|---|---|"]
TQ = {1: 12.71, 2: 4.30, 3: 3.18, 4: 2.78, 5: 2.57, 6: 2.45, 7: 2.36}
vp = [(a, b) for a, b in pairs if R[a]["valid"] and R[b]["valid"]]
for k, nm in [("p50", "frame p50"), ("p95", "frame p95"), ("mean", "frame mean"), ("su50", "SLAM upd p50"), ("su95", "SLAM upd p95"), ("se50", "slam_ekf p50"), ("se95", "slam_ekf p95")]:
    d = np.array([R[a][k] - R[b][k] for a, b in vp])
    if not len(d): continue
    sd = d.std(ddof=1) if len(d) > 1 else float("nan"); hw = TQ.get(len(d) - 1, 2.0) * sd / math.sqrt(len(d)) if len(d) > 1 else float("nan")
    nm_, om_ = np.mean([R[a][k] for a, b in vp]), np.mean([R[b][k] for a, b in vp])
    L.append(f"| {nm} | {', '.join(f'{x:+.2f}' for x in d)} | {d.mean():+.2f} | {sd:.2f} | [{d.mean()-hw:+.2f}, {d.mean()+hw:+.2f}] | {nm_:.2f} / {om_:.2f} | {100*(nm_-om_)/om_:+.0f}% |")
L.append(f"\nvalid pairs: {len(vp)} of {len(pairs)}: {vp}")
open(out, "w").write("\n".join(L) + "\n"); print("\n".join(L))
