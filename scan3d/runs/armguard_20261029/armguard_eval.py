#!/usr/bin/env python3
"""armguard 2026-10-29: replay the 75 fcarm event streams (69 bench runs + 6 VIOP captures) through fc_safety.c with the NEW
defaults (arm guard ON) and the former values, same segments and scoring as fcarm_eval.py / fcarm_posthoc.py.
NOT a new evidence base (same post hoc data the defaults were chosen on) -> report as unverified.
usage: armguard_eval.py <replay binary> <ev dir> <fcarm dir (results.json, posthoc.json)> <out json>"""
import sys, os, json, subprocess, time
import numpy as np
from concurrent.futures import ThreadPoolExecutor

BIN, EV, FD, OUTF = sys.argv[1:5]
per = json.load(open(FD + "/results.json"))["per_run"]
ph = json.load(open(FD + "/posthoc.json"))
CFG = ["0 0.2 0.5 0 0 0.15",          # 0: former values (baseline for FR)
       "default",                     # 1: untouched fs_config_default() = new defaults
       "1000 0.05 0.1 0.2 0 0.15",    # 2: the same set, explicit (must equal 1)
       "1000 0.05 0.1 0.2 0 1e9"]     # 3: new set, std check off (DA(ii))
CF = os.path.splitext(OUTF)[0] + "_configs.txt"
open(CF, "w").write("\n".join(CFG) + "\n")


def run(n):
    ev = f"{EV}/{n}.txt"
    g, t = [], []
    for l in open(ev):
        p = l.split()
        if p[0] == "I":
            t.append(int(p[1])); g.append(float(p[2]))
    g = np.array(g); t = np.array(t)
    moving = np.convolve(np.abs(g - np.median(g[:200])) > 0.08, np.ones(20), "same") > 0
    m_first = int(t[np.argmax(moving)]); m_last = int(t[len(moving) - 1 - np.argmax(moving[::-1])])
    assert abs(m_last - int(round(per[n]["t_put_down_s"] * 1e3))) <= 10, n
    r = subprocess.run([BIN, ev, CF, str(m_first), str(m_last), str(m_last), str(m_last + 3000)], capture_output=True, text=True, check=True)
    return n, np.array([[int(x) for x in l.split()] for l in r.stdout.splitlines()], dtype=np.int64)


names = sorted(per)
with ThreadPoolExecutor(max_workers=8) as ex:
    R = dict(ex.map(run, names))
bench = [n for n in names if not n.endswith("V")]
D = ph["D"]; Dp = ph["D_prime"]
N = [n for n in bench if per[n]["label"] == "N"]
NE = [n for n in N if per[n]["end_still_s"] >= 3.0]
# columns: idx nH armH nE armE nE3 armE3 base_okE3 base_ok_new_rejE3 first_arm_after_e_ms rejbits_E3


def metr(ci):
    lat = [int(R[n][ci][9]) for n in NE]
    return dict(FRn=int(sum(R[n][ci][8] for n in N)), FRd=int(sum(R[n][ci][7] for n in N)),
                HAn=int(sum(R[n][ci][2] for n in bench)), HAd=int(sum(R[n][ci][1] for n in bench)),
                DAn_D9=int(sum(R[n][ci][4] for n in D)), DAn_D7=int(sum(R[n][ci][4] for n in Dp)),
                DAn_D9_by_run={n: int(R[n][ci][4]) for n in D if R[n][ci][4]},
                lat_n=len(lat), lat_med_ms=int(np.median([x if x >= 0 else 10 ** 9 for x in lat])),
                lat_max_ms=int(max(x if x >= 0 else 10 ** 9 for x in lat)), lat_p90_ms=int(np.percentile(lat, 90)),
                N_never_armable_after_putdown=[n for n in N if R[n][ci][9] < 0],
                N_refused_E3=[n for n in N if R[n][ci][8] > 0])


same12 = all((R[n][1][1:] == R[n][2][1:]).all() for n in names)
viop = {n: dict(label=per[n]["label"], first_ms_former=int(R[n][0][9]), first_ms_new=int(R[n][1][9]), armE_new=int(R[n][1][4]),
                nE=int(R[n][1][3])) for n in names if n.endswith("V")}
lat_runs = {n: dict(former=int(R[n][0][9]), new=int(R[n][1][9]), end_still_s=per[n]["end_still_s"]) for n in NE}
out = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), note="armguard 10-29: new defaults ON vs former; POST HOC data (same as the choice) -> unverified",
           inputs=dict(ev=EV, fcarm=FD, binary=BIN, configs=CFG), n_streams=len(names), n_bench=len(bench), n_N=len(N), n_NE=len(NE),
           n_D=len(D), D_prime=Dp, default_equals_explicit_set=bool(same12),
           former=metr(0), new_default=metr(1), new_std_off=dict(DAn_D9=metr(3)["DAn_D9"], DAn_D7=metr(3)["DAn_D7"]),
           first_arm_by_run=lat_runs, viop=viop)
json.dump(out, open(OUTF, "w"), indent=1)
print(json.dumps({k: out[k] for k in ["n_streams", "n_bench", "n_N", "n_NE", "default_equals_explicit_set", "former", "new_default", "new_std_off", "viop"]}, indent=1))
