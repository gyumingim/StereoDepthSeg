#!/usr/bin/env python3
"""armhist 2026-10-29: replay the same 75 fcarm event streams (69 bench + 6 VIOP) through fc_safety.c with the VIO history
latch, same segments and scoring as armguard_eval.py. Rule + pass criteria pre-registered in README.md section 0 (commit
e4cc5b0, before this script was run).
usage: armhist_eval.py <replay binary> <ev dir> <fcarm dir (results.json, posthoc.json)> <out json>"""
import sys, os, json, subprocess, time
import numpy as np
from concurrent.futures import ThreadPoolExecutor

BIN, EV, FD, OUTF = sys.argv[1:5]
per = json.load(open(FD + "/results.json"))["per_run"]
ph = json.load(open(FD + "/posthoc.json"))
G = "1000 0.05 0.1 0.2 0 0.15"  # armguard defaults (instant checks)
CFG = [G + " 0 0",              # 0: current default (armguard, history off) = baseline for FR / first arm
       "default",               # 1: untouched fs_config_default() = new defaults (history ON)
       G + " 2.0 1",            # 2: the same, explicit (must equal 1)
       G + " 2.0 0",            # 3: speed latch only
       G + " 0 1",              # 4: lost latch only
       "0 0.2 0.5 0 0 0.15 2.0 1",  # 5: former instant arm + history
       "0 0.2 0.5 0 0 0.15 0 0"]    # 6: former values, history off (reference)
SENS = [(v, l) for v in (1.5, 2.5, 3.0, 5.0) for l in (1, 0)]  # secondary, post hoc label
for v, l in SENS:
    CFG.append(f"{G} {v} {l}")
CF = os.path.splitext(OUTF)[0] + "_configs.txt"
open(CF, "w").write("\n".join(CFG) + "\n")
TRAIN_RECS = ("hs_20261019_131326", "hs_20261019_154320", "hs_20261024_142148", "hs_20261024_165506")
TEST_RECS = ("hs_20261024_142610", "hs_20261024_151646")


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
    return n, np.array([[int(x) for x in l.split()] for l in r.stdout.splitlines()], dtype=np.int64), m_last


names = sorted(per)
with ThreadPoolExecutor(max_workers=8) as ex:
    out_ = list(ex.map(run, names))
R = {n: r for n, r, _ in out_}
MLAST = {n: m for n, _, m in out_}
assert all(len(R[n]) == len(CFG) for n in names)
bench = [n for n in names if not n.endswith("V")]
D = ph["D"]; Dp = ph["D_prime"]
N = [n for n in bench if per[n]["label"] == "N"]
NE = [n for n in N if per[n]["end_still_s"] >= 3.0]
split = {n: ("train" if per[n]["rec"].startswith(TRAIN_RECS) else "test" if per[n]["rec"].startswith(TEST_RECS) else "?") for n in names}
assert all(split[n] != "?" for n in N)
# columns: 0 idx 1 nH 2 armH 3 nE 4 armE 5 nE3 6 armE3 7 base_okE3 8 base_ok_new_rejE3 9 first_arm_after_e_ms 10 rejbits_E3
#          11 hist_latched 12 hist_t_ms 13 hist_why 14 nHistRej


def metr(ci):
    lat = [int(R[n][ci][9]) for n in NE]
    big = lambda x: x if x >= 0 else 10 ** 9
    return dict(FRn=int(sum(R[n][ci][8] for n in N)), FRd=int(sum(R[n][ci][7] for n in N)),
                HAn=int(sum(R[n][ci][2] for n in bench)), HAd=int(sum(R[n][ci][1] for n in bench)),
                DAn_D9=int(sum(R[n][ci][4] for n in D)), DAn_D7=int(sum(R[n][ci][4] for n in Dp)),
                DAn_D9_by_run={n: int(R[n][ci][4]) for n in D if R[n][ci][4]},
                D9_latched=[n for n in D if R[n][ci][11]],
                lat_n=len(lat), lat_med_ms=int(np.median([big(x) for x in lat])), lat_max_ms=int(max(big(x) for x in lat)),
                NE_first_arm_differs_from_cfg0=[n for n in NE if R[n][ci][9] != R[n][0][9]],
                N_latched_train=[n for n in N if R[n][ci][11] and split[n] == "train"],
                N_latched_test=[n for n in N if R[n][ci][11] and split[n] == "test"],
                N_never_armable_after_putdown=[n for n in N if R[n][ci][9] < 0],
                N_refused_E3=[n for n in N if R[n][ci][8] > 0])


M = {i: metr(i) for i in range(len(CFG))}
same12 = all((R[n][1][1:] == R[n][2][1:]).all() for n in names)
viop = {n: dict(label=per[n]["label"], first_ms_cfg0=int(R[n][0][9]), first_ms_new=int(R[n][1][9]), armE_new=int(R[n][1][4]),
                nE=int(R[n][1][3]), latched=int(R[n][1][11]), hist_why=int(R[n][1][13])) for n in names if n.endswith("V")}
Dinfo = {n: dict(latched=int(R[n][1][11]), hist_t_ms=int(R[n][1][12]), put_down_ms=MLAST[n], why=int(R[n][1][13]),
                 armE_cfg0=int(R[n][0][4]), armE_new=int(R[n][1][4]), armE_speed_only=int(R[n][3][4]), armE_lost_only=int(R[n][4][4]),
                 nE=int(R[n][1][3])) for n in D}
viopD = [n for n in viop if viop[n]["label"] == "D"]
viopN = [n for n in viop if viop[n]["label"] == "N"]
m1 = M[1]
P = dict(P1=m1["DAn_D9"] == 0,
         P2=m1["FRn"] == 0 and not m1["NE_first_arm_differs_from_cfg0"] and not m1["N_latched_train"] and not m1["N_latched_test"],
         P3=m1["HAn"] == 0,
         P4=all(viop[n]["armE_new"] == 0 for n in viopD) and all(viop[n]["first_ms_new"] == viop[n]["first_ms_cfg0"] for n in viopN),
         P5=bool(same12))
P["PASS"] = all(P.values())
out = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), note="armhist 10-29: VIO history latch; rule pre-registered (README sec 0, e4cc5b0)",
           inputs=dict(ev=EV, fcarm=FD, binary=BIN, configs=CFG), n_streams=len(names), n_bench=len(bench), n_N=len(N), n_NE=len(NE),
           n_N_train=sum(split[n] == "train" for n in N), n_N_test=sum(split[n] == "test" for n in N), n_D=len(D),
           default_equals_explicit=bool(same12), pass_=P,
           cfg={i: dict(line=CFG[i], **M[i]) for i in range(len(CFG))}, D_runs=Dinfo, viop=viop,
           first_arm_by_run={n: dict(cfg0=int(R[n][0][9]), new=int(R[n][1][9])) for n in NE})
json.dump(out, open(OUTF, "w"), indent=1)
short = {i: {k: M[i][k] for k in ("FRn", "HAn", "DAn_D9", "DAn_D9_by_run", "lat_med_ms", "lat_max_ms", "NE_first_arm_differs_from_cfg0",
                                  "N_latched_train", "N_latched_test")} for i in M}
print(json.dumps(dict(pass_=P, same12=same12, cfg=short, D_runs=Dinfo, viop=viop), indent=1))
