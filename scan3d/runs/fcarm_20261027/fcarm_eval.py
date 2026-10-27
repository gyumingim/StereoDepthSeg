#!/usr/bin/env python3
"""fcarm 2026-10-27: arm threshold sweep over the existing gyrocheck runs (PLAN.md). Read-only on the inputs.
Builds the event stream per run (same mapping as gyrocheck arm_replay/arm_prep.py, plus |v| per pose), runs
fcarm_replay (fc_safety.c of this branch) for every grid config, applies the PLAN rules, writes results.json.
usage: fcarm_eval.py <replay binary> <out dir> [--viop]"""
import sys, os, gzip, json, csv, math, itertools, subprocess, struct, time, hashlib
from concurrent.futures import ThreadPoolExecutor
import numpy as np

BIN, OUT = sys.argv[1], sys.argv[2]
ATE = "/home/system/scanner/tmp_gyrocheck/ate"
SCORE = "/home/system/scanner/StereoDepthSeg-gyrocheck/scan3d/runs/gyrocheck_20261027/score.json"
RECD = "/home/system/scanner/StereoDepthSeg/scan3d/recordings"
os.makedirs(OUT + "/ev", exist_ok=True)

# ---------------- grid (PLAN.md) ----------------
STILL = [0, 250, 500, 1000, 1500, 2000, 3000]
GYRO = [0.03, 0.05, 0.08, 0.10, 0.15, 0.20]
AVAR = [0.5, 0.1, 0.02]
SPEED = [0.0, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0]  # 0 = off
HOLD = [0, 500]
STD = [0.15, 1e9]  # as-is / std check off (DA(ii))
grid = []
for st, g, a, sp, h in itertools.product(STILL, GYRO, AVAR, SPEED, HOLD):
    if sp == 0.0 and h != 0:
        continue
    grid.append((st, g, a, sp, h))
cfg_lines = ["0 0.2 0.5 0 0 0.15"]  # line 0 = untouched defaults (baseline)
cfg_index = {}
for sd in STD:
    for c in grid:
        cfg_index[(c, sd)] = len(cfg_lines)
        cfg_lines.append(" ".join(str(x) for x in c) + f" {sd}")
open(OUT + "/configs.txt", "w").write("\n".join(cfg_lines) + "\n")


def load_imu(rec):
    rows = list(csv.DictReader(open(f"{RECD}/{rec}/imu.csv")))
    acc = np.array([[int(r["t_ns"]), float(r["v0"]), float(r["v1"]), float(r["v2"])] for r in rows if r["type"] == "1"])
    gt = "16" if any(r["type"] == "16" for r in rows[:5000]) else "4"
    gyr = np.array([[int(r["t_ns"]), float(r["v0"]), float(r["v1"]), float(r["v2"])] for r in rows if r["type"] == gt])
    return acc, gyr, gt


IMU = {}


def build(run, poses_path, viop=False):
    rec = run["rec"]
    if rec not in IMU:
        IMU[rec] = load_imu(rec)
    acc, gyr, gtype = IMU[rec]
    t0 = min(acc[0, 0], gyr[0, 0]); t_end = min(acc[-1, 0], gyr[-1, 0])
    ref = acc[acc[:, 0] > t_end - 2e9, 1:].mean(0); ref /= np.linalg.norm(ref)
    if viop:
        raw = gzip.open(poses_path, "rb").read(); P = []; i = 0
        while i + 10 <= len(raw):
            nn = int.from_bytes(raw[i:i + 2], "little"); fr = raw[i + 10:i + 10 + nn]; i += 10 + nn
            if len(fr) < 10 or fr[0] != 0xA5 or fr[1] != 0x5A or fr[3] != 0x01:
                continue
            pl = fr[8:8 + fr[4]]
            t_us, = struct.unpack_from("<Q", pl, 0); fl, = struct.unpack_from("<H", pl, 14)
            sd = struct.unpack_from("<3H", pl, 60); vel = struct.unpack_from("<3f", pl, 28)
            P.append(dict(t_ns=t_us * 1000, init=bool(fl & 1), status="tracking" if fl & 2 else "lost", flags=fl & 3, q=pl[79],
                          sd=max(sd) / 1000.0, sp=float(np.linalg.norm(vel))))
        P.sort(key=lambda m: m["t_ns"])
    else:
        P = []
        for x in gzip.open(poses_path, "rt"):
            if not x.strip():
                continue
            m = json.loads(x)
            if m.get("type", "pose") != "pose":
                continue
            init = bool(m.get("init")); lost = m.get("status") != "tracking"
            h = m.get("h") or {}
            sd = h.get("pos_std")
            sd = float("nan") if sd is None else sd / math.sqrt(3.0)  # PLAN: sqrt(trace/3)
            q = 0 if lost or not (sd == sd) else min(100, max(1, round(100 * (1 - sd / 0.20))))
            v = m.get("v")
            sp = float(np.linalg.norm(v)) if v is not None else float("nan")
            P.append(dict(t_ns=m["t_ns"], init=init, status=m.get("status"), flags=1 | (0 if lost else 2), q=q, sd=sd, sp=sp))
    ev = []
    grid_t = np.arange(t0, t_end, 10e6)
    ia = np.searchsorted(acc[:, 0], grid_t).clip(0, len(acc) - 1)
    ig = np.searchsorted(gyr[:, 0], grid_t).clip(0, len(gyr) - 1)
    k = 25
    acs = np.cumsum(np.vstack([np.zeros(3), acc[:, 1:]]), 0)
    gw = []
    for j, t in enumerate(grid_t):
        a = acc[ia[j], 1:]; g = gyr[ig[j], 1:]
        lo, hi = max(0, ia[j] - k), min(len(acc), ia[j] + 1)
        am = (acs[hi] - acs[lo]) / max(1, hi - lo)
        c = float(np.dot(am, ref) / max(1e-9, np.linalg.norm(am)))
        tilt = math.acos(max(-1.0, min(1.0, c)))
        gn = float(np.linalg.norm(g)); gw.append(gn)
        tm = int((t - t0) / 1e6)
        ev.append((tm, 2, f"I {tm} {gn:.5f} {np.linalg.norm(a):.5f} {tilt:.5f}"))
    state_at = []
    vmax_trk = 0.0
    for m in P:
        tm = int((m["t_ns"] - t0) / 1e6)
        if tm < 0:
            continue
        lost = m["status"] != "tracking"
        state_at.append((tm, m["init"], lost))
        if m["init"]:
            ev.append((tm, 0, f"P {tm} {m['flags'] if not viop else m['flags']} {m['q']} {m['sd']:.5f} {m['sp']:.5f}"))
            if not lost and m["sp"] == m["sp"]:
                vmax_trk = max(vmax_trk, m["sp"])
    si = 0; last = (False, False)
    for tm in range(0, int((t_end - t0) / 1e6), 50):
        while si < len(state_at) and state_at[si][0] <= tm:
            last = state_at[si][1:]; si += 1
        st = 0 if not last[0] else (2 if last[1] else 1)
        ev.append((tm, 1, f"H {tm} {st}"))
    ev.sort(key=lambda x: (x[0], x[1]))
    gw = np.array(gw); tms = np.array([e[0] for e in ev if e[1] == 2])
    moving = np.convolve(np.abs(gw - np.median(gw[:200])) > 0.08, np.ones(20), "same") > 0
    m_first = int(tms[np.argmax(moving)]); m_last = int(tms[len(moving) - 1 - np.argmax(moving[::-1])])
    t_init = next((s[0] for s in state_at if s[1]), None)
    return "\n".join(e[2] for e in ev) + "\n", dict(m_first=m_first, m_last=m_last, t_end=int(tms[-1]), t_init=t_init,
                                                    vmax_trk=round(vmax_trk, 3), gyro_type=gtype)


def run_one(name, run, path, viop=False):
    txt, seg = build(run, path, viop)
    evf = f"{OUT}/ev/{name}.txt"
    open(evf, "w").write(txt)
    r = subprocess.run([BIN, evf, OUT + "/configs.txt", str(seg["m_first"]), str(seg["m_last"]), str(seg["m_last"]),
                        str(seg["m_last"] + 3000)], capture_output=True, text=True, check=True)
    rows = np.array([[int(x) for x in l.split()] for l in r.stdout.splitlines()], dtype=np.int64)
    if not os.environ.get("KEEP_EV"):
        os.remove(evf)
    return name, seg, rows


def main():
    S = json.load(open(SCORE))["runs"]
    jobs, seen = [], set()
    for key, run in sorted(S.items()):
        if run["noinit"]:
            continue
        dk = (run["rec"], run["hash"])
        if dk in seen:
            continue
        seen.add(dk)
        jobs.append((key, run, f"{ATE}/{run['cond']}/{key}.poses.jsonl.gz", False))
    if "--viop" in sys.argv:
        for cond in ("NL60V", "GL60V"):
            for f in sorted(os.listdir(f"{ATE}/{cond}")):
                if f.endswith(".viop.bin.gz"):
                    key = f[:-len(".viop.bin.gz")]
                    base = key.replace("V", "", 1) if False else key[:-1]  # l142148_k0_NL60V -> l142148_k0_NL60
                    jobs.append((key, S[base], f"{ATE}/{cond}/{f}", True))
    # load IMU once per recording (sequential), then replay in parallel
    for _, run, _, _ in jobs:
        if run["rec"] not in IMU:
            IMU[run["rec"]] = load_imu(run["rec"])
    t0 = time.time()
    res = {}
    with ThreadPoolExecutor(max_workers=8) as ex:
        for name, seg, rows in ex.map(lambda j: run_one(*j), jobs):
            res[name] = (seg, rows)
    print(f"replayed {len(res)} runs x {len(cfg_lines)} configs in {time.time() - t0:.0f} s", flush=True)
    labels = {}
    for name, (seg, rows) in res.items():
        base = name[:-1] if name.endswith("V") else name
        run = S[base]
        D = bool(run["div"]) or seg["vmax_trk"] > 2.0
        labels[name] = dict(rec=run["rec"], cond=run["cond"], k=run["k"], D=D, div=run["div"], viop=name.endswith("V"), **seg)
    names = sorted(res)
    bench = [n for n in names if not labels[n]["viop"]]
    COL = dict(nH=1, aH=2, nE=3, aE=4, nE3=5, aE3=6, bE3=7, brE3=8, first=9, rej=10)

    def metrics(ci, use):
        FRn = FRd = HAn = HAd = DAn = DAd = 0
        worst_first = 0
        for n in use:
            r = res[n][1][ci]
            HAn += r[COL["aH"]]; HAd += r[COL["nH"]]
            if labels[n]["D"]:
                DAn += r[COL["aE"]]; DAd += r[COL["nE"]]
            else:
                FRn += r[COL["brE3"]]; FRd += r[COL["bE3"]]
                f = r[COL["first"]]
                worst_first = max(worst_first, 10 ** 9 if f < 0 else f)
        return dict(FR=FRn / max(1, FRd), FRn=int(FRn), HA=HAn / max(1, HAd), HAn=int(HAn), DA=DAn / max(1, DAd), DAn=int(DAn),
                    worst_first_ms=int(worst_first))

    table = []
    for c in grid:
        m1 = metrics(cfg_index[(c, 0.15)], bench)
        m2 = metrics(cfg_index[(c, 1e9)], bench)
        feas = m1["FRn"] == 0 and m1["HAn"] == 0 and m1["DAn"] == 0
        table.append(dict(cfg=c, m=m1, DA2=m2["DA"], DA2n=m2["DAn"], feasible=feas))
    feas_set = {tuple(t["cfg"]) for t in table if t["feasible"]}
    axes = [STILL, GYRO, AVAR, SPEED, HOLD]

    def neighbours(c):
        out = []
        for d, ax in enumerate(axes):
            i = ax.index(c[d])
            for j in (i - 1, i + 1):
                if 0 <= j < len(ax):
                    cc = list(c); cc[d] = ax[j]
                    if cc[3] == 0.0:
                        cc[4] = 0
                    if cc[3] != 0.0 or cc[4] == 0:
                        out.append(tuple(cc))
        return [x for x in set(out) if x != tuple(c)]

    for t in table:
        t["robust"] = t["feasible"] and all(nb in feas_set for nb in neighbours(t["cfg"]))
    order = lambda t: (t["cfg"][0], -t["cfg"][1], -t["cfg"][2], -(t["cfg"][3] if t["cfg"][3] > 0 else 99), t["cfg"][4])
    rob = sorted([t for t in table if t["robust"]], key=order)
    fea = sorted([t for t in table if t["feasible"]], key=order)
    if rob:
        rec, kind = rob[0], "robust"
    elif fea:
        rec, kind = fea[0], "feasible"
    else:
        rec = min(table, key=lambda t: (t["m"]["FRn"] + t["m"]["HAn"] + t["m"]["DAn"], order(t))); kind = "none (closest)"
    base_m = metrics(0, bench)
    # per-run detail for the baseline and the recommendation
    det = {}
    ci = cfg_index[(tuple(rec["cfg"]), 0.15)]
    for n in names:
        r0, r1 = res[n][1][0], res[n][1][ci]
        det[n] = dict(label="D" if labels[n]["D"] else "N", rec=labels[n]["rec"], vmax_trk=labels[n]["vmax_trk"],
                      t_put_down_s=labels[n]["m_last"] / 1e3, end_still_s=round((labels[n]["t_end"] - labels[n]["m_last"]) / 1e3, 2),
                      base=dict(H=round(r0[2] / max(1, r0[1]), 4), E=round(r0[4] / max(1, r0[3]), 4), first_ms=int(r0[9])),
                      new=dict(H=round(r1[2] / max(1, r1[1]), 4), E=round(r1[4] / max(1, r1[3]), 4), E3_falserej=int(r1[8]),
                               first_ms=int(r1[9]), rej_bits=int(r1[10])))
    # 1-D slices around the recommendation (each parameter varied, others fixed)
    slices = {}
    for d, nm in enumerate(["still_ms", "gyro", "acc_var", "speed", "hold"]):
        sl = []
        for v in axes[d]:
            cc = list(rec["cfg"]); cc[d] = v
            if cc[3] == 0.0: cc[4] = 0
            t = next((t for t in table if tuple(t["cfg"]) == tuple(cc)), None)
            if t: sl.append(dict(v=v, FRn=t["m"]["FRn"], HAn=t["m"]["HAn"], DAn=t["m"]["DAn"], DA2n=t["DA2n"], worst_first_ms=t["m"]["worst_first_ms"]))
        slices[nm] = sl
    # speed-only view: the old rule + speed check (no still/gyro change): which speed thresholds keep FR 0 and DA(ii) 0
    sp_only = []
    for sp in SPEED:
        for h in HOLD:
            c = (0, 0.2, 0.5, sp, h if sp else 0)
            if (c, 0.15) not in cfg_index or (sp == 0 and h):
                continue
            m1 = metrics(cfg_index[(c, 0.15)], bench); m2 = metrics(cfg_index[(c, 1e9)], bench)
            sp_only.append(dict(speed=sp, hold=h, FRn=m1["FRn"], DAn=m1["DAn"], DA2n=m2["DAn"], DA2=round(m2["DA"], 4), HA=round(m1["HA"], 4)))
    # VIOP captures: baseline + recommendation (cross-check of the sqrt(trace/3) approximation)
    viop = {n: det[n] for n in names if labels[n]["viop"]}
    nN = sum(1 for n in bench if not labels[n]["D"]); nD = sum(1 for n in bench if labels[n]["D"])
    out = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), plan="PLAN.md (commit 86b09f2, before results)", inputs=dict(score=SCORE, ate=ATE, recordings=RECD),
               n_runs_bench=len(bench), n_normal=nN, n_div=nD, n_recordings=len({labels[n]["rec"] for n in bench}),
               div_runs=sorted(n for n in bench if labels[n]["D"]),
               n_configs=len(grid), n_feasible=len(fea), n_robust=len(rob), recommendation=dict(kind=kind, cfg=dict(zip(
                   ["arm_still_ms", "arm_gyro_max_rps", "arm_acc_var_max", "arm_vio_speed_max", "arm_vio_speed_hold_ms"], rec["cfg"])), m=rec["m"],
                   DA_std_off=rec["DA2"], DA_std_off_n=rec["DA2n"]),
               baseline=base_m, baseline_std_off=metrics(cfg_index[((0, 0.2, 0.5, 0.0, 0), 1e9)], bench), slices=slices, speed_only=sp_only,
               feasible_ranges={nm: sorted({t["cfg"][d] for t in fea}) for d, nm in enumerate(["still_ms", "gyro", "acc_var", "speed", "hold"])},
               robust_list=[dict(cfg=t["cfg"], m=t["m"], DA2n=t["DA2n"]) for t in rob[:40]], per_run=det, viop=viop)
    json.dump(out, open(OUT + "/results.json", "w"), indent=1)
    json.dump([dict(cfg=t["cfg"], **t["m"], DA2n=t["DA2n"], feasible=t["feasible"], robust=t["robust"]) for t in table],
              open(OUT + "/grid.json", "w"))
    print(json.dumps({k: out[k] for k in ["n_runs_bench", "n_normal", "n_div", "n_feasible", "n_robust", "recommendation", "baseline", "baseline_std_off", "feasible_ranges"]}, indent=1))


main()
