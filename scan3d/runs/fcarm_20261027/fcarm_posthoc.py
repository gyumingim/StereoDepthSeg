#!/usr/bin/env python3
"""fcarm 2026-10-27 POST HOC (after results.json showed 0 feasible configs; NOT the pre-registered rule).
Re-runs fcarm_replay on the kept event streams to get per-run x per-config rows, then:
  1. which D runs stay armable for every config, and why (VIO history: LOST poses / max |v| after init, end-still |v|)
  2. the PLAN feasibility rule with D' = D minus the "recovered" divergences (D runs armable at the end in EVERY config)
  3. still_ms x gyro table of HA / FR, first-arm latency after put-down (runs with >= 3 s end still only)
usage: fcarm_posthoc.py <replay binary> <sweep dir>"""
import sys, os, json, subprocess, itertools, gzip, math, time
import numpy as np
from concurrent.futures import ThreadPoolExecutor

BIN, OUT = sys.argv[1], sys.argv[2]
R = json.load(open(OUT + "/results.json"))
cfg_lines = open(OUT + "/configs.txt").read().split("\n")
cfg_lines = [l for l in cfg_lines if l.strip()]
cfgs = [tuple(float(x) for x in l.split()) for l in cfg_lines]
per = R["per_run"]


def run(n):
    d = per[n]
    m_last = int(round(d["t_put_down_s"] * 1e3))
    # segment args: H start not stored -> recompute from the event file (same rule as fcarm_eval)
    ev = OUT + f"/ev/{n}.txt"
    g, t = [], []
    vio = []  # (t, flags, speed)
    for l in open(ev):
        p = l.split()
        if p[0] == "I":
            t.append(int(p[1])); g.append(float(p[2]))
        elif p[0] == "P":
            vio.append((int(p[1]), int(p[2]), float(p[5])))
    g = np.array(g); t = np.array(t)
    moving = np.convolve(np.abs(g - np.median(g[:200])) > 0.08, np.ones(20), "same") > 0
    m_first = int(t[np.argmax(moving)]); m_last2 = int(t[len(moving) - 1 - np.argmax(moving[::-1])])
    assert abs(m_last2 - m_last) <= 10, (n, m_last, m_last2)
    r = subprocess.run([BIN, ev, OUT + "/configs.txt", str(m_first), str(m_last2), str(m_last2), str(m_last2 + 3000)],
                       capture_output=True, text=True, check=True)
    rows = np.array([[int(x) for x in l.split()] for l in r.stdout.splitlines()], dtype=np.int64)
    vio = np.array(vio) if vio else np.zeros((0, 3))
    lost = int(((vio[:, 1].astype(int) & 2) == 0).sum()) if len(vio) else 0
    sp = vio[:, 2] if len(vio) else np.zeros(0)
    end = vio[:, 0] > m_last2 if len(vio) else np.zeros(0, bool)
    hist = dict(lost_after_init=lost, vmax_all=round(float(np.nanmax(sp)), 3) if len(sp) else None,
                end_v_med=round(float(np.nanmedian(sp[end])), 3) if end.any() else None,
                end_v_max=round(float(np.nanmax(sp[end])), 3) if end.any() else None,
                end_v_max_after3s=round(float(np.nanmax(sp[vio[:, 0] > m_last2 + 3000])), 3) if (vio[:, 0] > m_last2 + 3000).any() else None)
    return n, rows, hist


names = sorted(per)
with ThreadPoolExecutor(max_workers=8) as ex:
    out = {n: (rows, hist) for n, rows, hist in ex.map(run, names)}
bench = [n for n in names if not n.endswith("V")]
D = [n for n in bench if per[n]["label"] == "D"]
N = [n for n in bench if per[n]["label"] == "N"]
NE = [n for n in N if per[n]["end_still_s"] >= 3.0]  # runs with an end-still segment (hand loops + std60)
idx = {c: i for i, c in enumerate(cfgs)}
STILL = [0, 250, 500, 1000, 1500, 2000, 3000]; GYRO = [0.03, 0.05, 0.08, 0.10, 0.15, 0.20]; AVAR = [0.5, 0.1, 0.02]
SPEED = [0.0, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0]; HOLD = [0, 500]
grid = [(st, g, a, sp, h) for st, g, a, sp, h in itertools.product(STILL, GYRO, AVAR, SPEED, HOLD) if not (sp == 0 and h)]
# 1. D runs armable at the end in every grid config (std as-is)
always = [n for n in D if all(out[n][0][idx[tuple(map(float, c)) + (0.15,)]][4] > 0 for c in grid)]
never_base = [n for n in D if out[n][0][0][4] == 0]
Dp = [n for n in D if n not in always]


def metr(ci, Dset):
    FRn = sum(out[n][0][ci][8] for n in N); HAn = sum(out[n][0][ci][2] for n in bench)
    DAn = sum(out[n][0][ci][4] for n in Dset)
    lat = [out[n][0][ci][9] for n in NE]
    lat = [x if x >= 0 else 10 ** 9 for x in lat]
    return dict(FRn=int(FRn), HAn=int(HAn), DAn=int(DAn), lat_max_ms=int(max(lat)), lat_med_ms=int(np.median(lat)))


tab = {}
for c in grid:
    ci = idx[tuple(map(float, c)) + (0.15,)]
    m = metr(ci, Dp)
    m2 = metr(idx[tuple(map(float, c)) + (1e9,)], Dp)
    m["DA2n"] = m2["DAn"]
    m["feasible"] = m["FRn"] == 0 and m["HAn"] == 0 and m["DAn"] == 0
    tab[c] = m
axes = [STILL, GYRO, AVAR, SPEED, HOLD]


def nbs(c):
    o = set()
    for d, ax in enumerate(axes):
        i = ax.index(c[d])
        for j in (i - 1, i + 1):
            if 0 <= j < len(ax):
                cc = list(c); cc[d] = ax[j]
                if cc[3] == 0.0: cc[4] = 0
                o.add(tuple(cc))
    o.discard(tuple(c))
    return o


rob = [c for c in grid if tab[c]["feasible"] and all(tab[x]["feasible"] for x in nbs(c))]
order = lambda c: (c[0], -c[1], -c[2], -(c[3] if c[3] > 0 else 99), c[4])
rob.sort(key=order)
fea = sorted([c for c in grid if tab[c]["feasible"]], key=order)
rec = rob[0] if rob else (fea[0] if fea else None)
# still x gyro table at acc_var 0.5, speed 0.3 / hold 0 and speed off
tables = {}
for sp in (0.0, 0.3):
    rows = []
    for st in STILL:
        rows.append({str(g): [tab[(st, g, 0.5, sp, 0)]["HAn"], tab[(st, g, 0.5, sp, 0)]["FRn"], tab[(st, g, 0.5, sp, 0)]["lat_max_ms"]] for g in GYRO})
    tables[f"speed{sp}"] = dict(rows=STILL, cols=GYRO, cell="[HA cycles, FR cycles, worst first-arm ms after put-down]", data=rows)
# speed slice at the recommendation
spsl = []
if rec:
    for sp in SPEED:
        for h in HOLD:
            c = (rec[0], rec[1], rec[2], sp, h if sp else 0)
            if sp == 0 and h: continue
            spsl.append(dict(speed=sp, hold=h, **tab[c]))
res = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), note="POST HOC, not the PLAN rule", sweep=OUT + "/results.json",
           D=D, D_always_armable_at_end=always, D_blocked_at_end_by_baseline=never_base, D_prime=Dp,
           n_feasible=len(fea), n_robust=len(rob),
           recommendation=None if rec is None else dict(cfg=dict(zip(["arm_still_ms", "arm_gyro_max_rps", "arm_acc_var_max", "arm_vio_speed_max", "arm_vio_speed_hold_ms"], rec)), **tab[rec]),
           feasible_ranges={nm: sorted({c[d] for c in fea}) for d, nm in enumerate(["still_ms", "gyro", "acc_var", "speed", "hold"])},
           robust_first=[dict(cfg=c, **tab[c]) for c in rob[:15]], tables=tables, speed_slice=spsl,
           baseline=metr(0, Dp), baseline_fullD=metr(0, D),
           history={n: dict(label=per[n]["label"], **out[n][1]) for n in names})
json.dump(res, open(OUT + "/posthoc.json", "w"), indent=1)
print(json.dumps({k: res[k] for k in ["D_always_armable_at_end", "D_blocked_at_end_by_baseline", "n_feasible", "n_robust", "recommendation", "feasible_ranges", "baseline", "speed_slice"]}, indent=1))
for k, v in tables.items():
    print(k, v["cell"])
    for st, row in zip(v["rows"], v["data"]):
        print(f"  still {st:5d}: " + "  ".join(f"g{g}:{row[str(g)]}" for g in GYRO))
for n in names:
    h = res["history"][n]
    if per[n]["label"] == "D" or n in ("l151646_k0_GL60", "l142148_k0_NL60"):
        print(n, h)
