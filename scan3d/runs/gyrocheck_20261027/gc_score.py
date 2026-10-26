#!/usr/bin/env python3
"""gyrocheck scorer: same rulers as integ2_ate (cam_metrics ate_w/divergent, fi_limits_run.loop_eval) + PLAN.md verdicts.
Writes <RD>/score.json and prints a table."""
import os, sys, json, time
sys.dont_write_bytecode = True
os.environ["OV60_OUT"] = "/home/system/scanner/tmp_gyrocheck/ate"
sys.path.insert(0, "/home/system/scanner/StereoDepthSeg-integ2/scan3d/runs/integ2_20261026")
import numpy as np  # noqa: E402
import integ2_ate as R  # noqa: E402
from pathlib import Path  # noqa: E402
sys.path.insert(0, str(Path(__file__).resolve().parent))
import gc_run as G  # noqa: E402
import lens_switch_gtscore as gs  # noqa: E402
from fi_limits_run import loop_eval  # noqa: E402
OUT = G.OUT
gs.OUT = OUT / "score_tmp"
gs.OUT.mkdir(parents=True, exist_ok=True)
DELTA = {"loop": 1.0, "std": 0.05, "hs": 0.05}
import gzip, hashlib  # noqa: E402


def track_hash(pre):
    """POST HOC (supervisor 23:31 C3): hash of the tracking poses (absolute t_ns + pose); equal = same run."""
    f = Path(str(pre) + ".poses.jsonl.gz")
    if not f.exists():
        return None
    h = hashlib.sha256(); n = 0
    for l in gzip.open(f, "rt"):
        m = json.loads(l)
        if m.get("status") == "tracking" and m.get("T_world_cam0"):
            h.update(json.dumps([m["t_ns"], m["T_world_cam0"], m.get("p")]).encode()); n += 1
    return h.hexdigest()[:16] if n else None
rows = {}
for which, rec in G.rec_list():
    key = G.key_of(which, rec)
    for k in [f"k{i}" for i in G.KS] + G.EXTRA:
        for c in G.CONDS:
            n = f"{key}_{k}_{c}"
            pre = OUT / c / n
            if not Path(str(pre) + ".json").exists():
                continue
            st = json.load(open(str(pre) + ".json"))
            row = dict(which=which, rec=rec, key=key, k=k, cond=c, tracked=st.get("tracked"), t_init=st.get("t_init_clip_s"),
                       frames_fed=st.get("frames_fed"), start_off=st.get("gc_start_off"), job_s=st.get("gc_job_s"),
                       hash=track_hash(pre), noinit=not st.get("tracked"))
            if which == "loop":
                e = loop_eval(Path(str(pre) + ".json"))
                v = e.get("loop_err_cm")
                row.update(val=v, path_m=e.get("path_m"), div=(v is None or v >= 100.0 or (st.get("tracked") or 0) < 30))
            else:
                mc = gs.cam_metrics(pre, rec, R.GT if which == "hs" else R.STD_GT)
                row.update(val=mc.get("ate_w"), div=bool(mc.get("divergent")) or mc.get("ate_w") is None, rel5_max=mc.get("rel5_max"))
            rows[n] = row


def verdict(key, which, dedup=False):
    PH = [f"k{i}" for i in G.KS] + (G.EXTRA if dedup else [])
    R_ = {k: {c: rows.get(f"{key}_{k}_{c}") for c in G.CONDS} for k in PH}
    done = [k for k in PH if all(R_[k][c] for c in G.CONDS)]
    noinit = [k for k in done if any(R_[k][c]["noinit"] for c in G.CONDS)]
    n_distinct = len({(R_[k]["NL60"]["hash"], R_[k]["GL60"]["hash"]) for k in done})
    if dedup:  # POST HOC: phases whose (NL60, GL60) runs are identical count once; init failures listed apart
        seen, keep = set(), []
        for k in done:
            if all(R_[k][c]["noinit"] for c in G.CONDS):  # both failed to init: drop (one side only = that side diverged, kept)
                continue
            hk = (R_[k]["NL60"]["hash"], R_[k]["GL60"]["hash"])
            if hk in seen:
                continue
            seen.add(hk); keep.append(k)
        done = keep
    dn = sum(R_[k]["NL60"]["div"] for k in done)
    dg = sum(R_[k]["GL60"]["div"] for k in done)
    pairs = [k for k in done if not R_[k]["NL60"]["div"] and not R_[k]["GL60"]["div"]]
    d = np.array([R_[k]["GL60"]["val"] - R_[k]["NL60"]["val"] for k in pairs])
    # direction per phase (+1 gyro better, -1 worse, 0 equal); divergence = worst
    dirs = []
    for k in done:
        a, b = R_[k]["NL60"], R_[k]["GL60"]
        if a["div"] and b["div"]:
            dirs.append(0)
        elif a["div"]:
            dirs.append(+1)
        elif b["div"]:
            dirs.append(-1)
        else:
            dd = b["val"] - a["val"]
            dirs.append(+1 if dd < 0 else (-1 if dd > 0 else 0))
    v = "TIE"
    why = ""
    import math
    n_d = len(done)
    div_need = max(2, math.ceil(n_d / 3)) if dedup else 2  # POST HOC 2 (supervisor 00:03 E2): ratio thresholds
    if abs(dg - dn) >= div_need:
        v = "GYRO+" if dg < dn else "GYRO-"
        why = f"div {dn}->{dg}"
    elif len(pairs) >= 3:
        need = (3 if len(pairs) == 3 else 4) if not dedup else max(3, math.ceil(2 * len(pairs) / 3))
        nb, nw = int((d < 0).sum()), int((d > 0).sum())
        med = float(np.median(d))
        if nb >= need and med <= -DELTA[which]:
            v = "GYRO+"
        elif nw >= need and med >= DELTA[which]:
            v = "GYRO-"
        why = f"pairs {len(pairs)} better {nb} worse {nw} med {med:+.3f}"
    else:
        why = f"pairs {len(pairs)} < 3"
    if dedup and len(done) < 3:
        v, why = "판정 불가", f"distinct runs {len(done)} < 3 ({why})"
    sysv = None
    if v in ("GYRO+", "GYRO-"):
        want = +1 if v == "GYRO+" else -1
        sysv = "계통" if sum(x == want for x in dirs) >= (4 if not dedup else math.ceil(2 * len(dirs) / 3)) else "약함"
    lottery = None
    if done and done[0] == "k0" and dirs:
        d0 = dirs[0]
        lottery = (d0 != 0 and sum(x == -d0 for x in dirs[1:]) >= (2 if not dedup else max(1, math.ceil((len(dirs) - 1) / 3))))
    tin = {c: [R_[k][c]["t_init"] for k in done] for c in G.CONDS}
    return dict(key=key, which=which, dedup=dedup, n_phase=len(done), n_distinct_all=n_distinct, noinit_phases=noinit, div_NL60=dn, div_GL60=dg, n_pairs=len(pairs),
                d_med=round(float(np.median(d)), 4) if len(d) else None, n_gyro_better=int((d < 0).sum()),
                verdict=v, why=why, consistency=sysv, dirs=dirs, k0_is_lottery=lottery,
                phases=PH,
                NL60=[None if not R_[k]["NL60"] else (round(R_[k]["NL60"]["val"], 3) if R_[k]["NL60"]["val"] is not None else None) for k in PH],
                GL60=[None if not R_[k]["GL60"] else (round(R_[k]["GL60"]["val"], 3) if R_[k]["GL60"]["val"] is not None else None) for k in PH],
                divs={c: [None if not R_[k][c] else bool(R_[k][c]["div"]) for k in PH] for c in G.CONDS},
                used_phases=done,
                t_init=tin, t_init_spread={c: (round(max(x) - min(x), 3) if x and None not in x else None) for c, x in tin.items()},
                med={c: (round(float(np.median([R_[k][c]["val"] for k in done if not R_[k][c]["div"]])), 3)
                         if any(not R_[k][c]["div"] for k in done) else None) for c in G.CONDS})


def overall(V):
    np_ = sum(x["verdict"] == "GYRO+" for x in V)
    nm = sum(x["verdict"] == "GYRO-" for x in V)
    tdn = sum(x["div_NL60"] for x in V)
    tdg = sum(x["div_GL60"] for x in V)
    if np_ >= 3 and nm <= 1 and tdg <= tdn:
        rec = "유지 (추정)"
    elif (nm >= 3 and np_ <= 1) or tdg >= tdn + 2:
        rec = "끄기 (추정)"
    else:
        rec = "판정 불가 -> 기본값 그대로, 라이브 60 fps GT 로 결정"
    return dict(gyro_plus=np_, gyro_minus=nm, div_total=dict(NL60=tdn, GL60=tdg), recommendation=rec)


VD = [verdict(G.key_of(w, r), w, dedup=True) for w, r in G.rec_list()]
OD = overall(VD)
V = [verdict(G.key_of(w, r), w) for w, r in G.rec_list()]
np_ = sum(x["verdict"] == "GYRO+" for x in V)
nm = sum(x["verdict"] == "GYRO-" for x in V)
tdn = sum(x["div_NL60"] for x in V)
tdg = sum(x["div_GL60"] for x in V)
if np_ >= 3 and nm <= 1 and tdg <= tdn:
    rec = "유지 (추정)"
elif (nm >= 3 and np_ <= 1) or tdg >= tdn + 2:
    rec = "끄기 (추정)"
else:
    rec = "판정 불가 -> 기본값 그대로, 라이브 60 fps GT 로 결정"
res = dict(made=time.strftime("%F %T"), plan="PLAN.md (committed fc42c97 before results)", bridge=json.load(open(OUT / "bin" / "bridge.json")),
           n_runs=len(rows), per_rec_dedup_posthoc=VD, overall_dedup_posthoc=OD, per_rec=V, gyro_plus=np_, gyro_minus=nm, div_total=dict(NL60=tdn, GL60=tdg), recommendation=rec, runs=rows)
json.dump(res, open(G.RD / "score.json", "w"), indent=1, ensure_ascii=False)
for x in V:
    print(f"{x['key']:8s} {x['which']:4s} n{x['n_phase']} NL60 {x['NL60']} GL60 {x['GL60']} div {x['div_NL60']}/{x['div_GL60']} "
          f"-> {x['verdict']} ({x['why']}) {x['consistency']} dirs {x['dirs']} k0_lottery {x['k0_is_lottery']} tinit {x['t_init']}")
print("GYRO+", np_, "GYRO-", nm, "div", tdn, tdg, "->", rec, "  [PLAN rule as written]")
print("--- post hoc dedup (supervisor C3) ---")
for x in VD:
    print(f"{x['key']:8s} distinct {x['n_phase']} (all-phase distinct {x['n_distinct_all']}) noinit {x['noinit_phases']} div {x['div_NL60']}/{x['div_GL60']} "
          f"-> {x['verdict']} ({x['why']}) {x['consistency']} dirs {x['dirs']} med NL {x['med']['NL60']} GL {x['med']['GL60']}")
print(OD)
