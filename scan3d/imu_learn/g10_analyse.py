#!/usr/bin/env python3
"""Analysis for imu_learn/g10.py (docs/g10_plan.md). Called via `g10.py analyse` -> runs/g10/summary.json."""
import gzip
import io
import json
import re
import time
from pathlib import Path

import numpy as np

import c5_overnight_analyse as an
import g10
import i3_takeoff as it
import i3_takeoff_analyse as ta
import z2_regress as zr

OUT = g10.OUT
SAME = 0.05
# The G1.0 bridge prints '[G10] t thr x\n' to stderr (unbuffered) while OpenVINS' own log goes through buffered stdout
# (dup'ed to stderr), so the [G10] text can land inside another log line. Removing the exact fragment restores the lines.
G10RE = re.compile(r"\[G10\] [-0-9.e+]+ thr [0-9.]+\n")


class _GzProxy:
    def __getattr__(self, k):
        return getattr(gzip, k)

    @staticmethod
    def open(path, mode="rb", *a, **kw):
        if str(path).endswith(".stderr.log.gz") and "t" in mode:
            with gzip.open(path, "rt", errors="replace") as f:
                return io.StringIO(G10RE.sub("", f.read()))
        return gzip.open(path, mode, *a, **kw)


ta.gzip = _GzProxy()
zr.gzip = _GzProxy()


def exists(p):
    return p is not None and Path(str(p) + ".json").exists()


_NC = {}


def corners(rec):
    """GT sample times / corner counts with the same filter as c5_overnight_analyse.gt (ok and |p| < 1.2 m)."""
    if rec not in _NC:
        g = np.load(Path("recordings") / rec / "imu_gt_spl" / "gt.npz", allow_pickle=True)
        ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
        _NC[rec] = (g["t_ns"][ok], g["n_corners"][ok])
    return _NC[rec]


def false_pass(prefix, rec):
    """Frames with '[ZUPT]: passed disparity' whose nearest GT sample (<= 20 ms) has GT speed > 2 cm/s and >= 12 corners."""
    r = ta.load_run(prefix)
    z = ta.zupt(prefix, r)
    if not len(z["t"]):
        return dict(n_pass=0, false_pass=0, n_pass_gt=0)
    dt = r["d"]["align_dt_ms"] * 1e6
    gt_t, gv = zr.gt_speed(rec)
    ct, nc = corners(rec)
    assert len(ct) == len(gt_t) and np.array_equal(ct, gt_t)
    tp = z["t"][z["ok"]] + dt
    j = np.clip(np.searchsorted(gt_t, tp), 1, len(gt_t) - 1)
    j = np.where(np.abs(gt_t[j - 1] - tp) < np.abs(gt_t[j] - tp), j - 1, j)
    near = np.abs(gt_t[j] - tp) < zr.GT_TOL * 1e9
    v = np.where(near, gv[j], np.nan)
    fp = near & (v > zr.V_MOVE) & (nc[j] >= 12)
    return dict(n_pass=int(z["ok"].sum()), n_pass_gt=int(np.isfinite(v).sum()), false_pass=int(fp.sum()),
                false_pass_v_cms=[round(float(x) * 100, 2) for x in v[fp]][:30])


def dep_cell(c):
    rec = c["rec"]
    x = it.inp(rec)
    cl = dict(rec=rec, still_imu_ns=c["still_imu_ns"])
    f0 = x["first_frame_imu_ns"]
    ms = float(c["args"][c["args"].index("--max-secs") + 1])
    lim = min(x["last_frame_imu_ns"], f0 + int((c["off"] + ms) * 1e9)) if ms else x["last_frame_imu_ns"]
    t_dep = c["still_imu_ns"][1]
    row = dict(key=c["key"], kind=c["kind"], rec=rec, off=c["off"], window_inside=bool(t_dep + it.WIN_S * 1e9 <= lim))
    for cond in ("base", "i3", "G"):
        p = c["src"].get(cond) if cond in c["src"] else str(OUT / f"{c['key']}_{cond}")
        if not exists(p):
            row[cond] = None
            continue
        b = ta.b_run(p, cl, x)
        b.update(false_pass(p, rec), prefix=str(p))
        d = json.load(open(str(p) + ".json"))
        b["gate_open_frames"] = d.get("g10_gate_open_frames")
        row[cond] = b
    bs = row.get("base")
    if bs is None:
        row["tested"], row["why"] = None, "base 없음 (실행 못 함)"
    elif bs["status"] == "noinit":
        row["tested"], row["why"] = False, "base 초기화 못 함"
    elif not row["window_inside"]:
        row["tested"], row["why"] = False, "출발 뒤 10 s 창이 클립 밖"
    elif bs["status"] == "unscored":
        row["tested"], row["why"] = False, "base 출발 창 채점 안 됨"
    else:
        row["tested"], row["why"] = True, ""
    return row


def r_cell(c, gates):
    row = dict(key=c["key"], kind="R", rec=c["rec"], gate_possible=gates.get(c["key"], [None])[0])
    for cond in ("base", "G"):
        p = str(OUT / f"{c['key']}_{cond}")
        if not exists(p):
            row[cond] = None
            continue
        a = zr.analyse_one(p)
        a.update(false_pass(p, c["rec"]))
        a["gate_open_frames"] = json.load(open(p + ".json")).get("g10_gate_open_frames")
        row[cond] = {k: a.get(k) for k in ("name", "status", "ate_cm", "scale", "t_init_clip_s", "n_pass", "n_pass_gt",
                                            "false_pass", "false_pass_v_cms", "gate_open_frames", "false_acc")}
    if not row["gate_possible"]:
        row["tested"], row["why"] = False, "문 안 열림 (사전 계산) → G = base 구성상"
    elif row["base"] is None:
        row["tested"], row["why"] = None, "base 실행 못 함"
    elif row["base"]["status"] == "noinit":
        row["tested"], row["why"] = False, "base 초기화 못 함"
    elif row["G"] is None:
        row["tested"], row["why"] = None, "G 실행 못 함"
    else:
        row["tested"], row["why"] = True, ""
    return row


def bitsame(p1, p2):
    if not (exists(p1) and exists(p2)):
        return None
    return ta.bitsame(p1, p2)


def signflip_p(d):
    d = np.asarray([x for x in d if x != 0], float)
    if not len(d):
        return None
    n = len(d)
    if n > 16:
        rng = np.random.default_rng(0)
        s = rng.choice([-1, 1], size=(200000, n))
    else:
        s = np.array(np.meshgrid(*[[-1, 1]] * n)).reshape(n, -1).T
    m = (s * np.abs(d)).mean(1)
    return float(np.mean(np.abs(m) >= abs(d.mean()) - 1e-12))


def pairs(rows, a, b):
    out = []
    for r in rows:
        if r.get("tested") and r.get(a) and r.get(b) and r[a].get("ate_cm") is not None and r[b].get("ate_cm") is not None:
            out.append((r["key"], round(r[a]["ate_cm"] - r[b]["ate_cm"], 3)))
    d = [x for _, x in out]
    return dict(n=len(d), med=None if not d else float(np.median(d)), good=sum(x < -SAME for x in d),
                same=sum(abs(x) <= SAME for x in d), bad=sum(x > SAME for x in d), p=signflip_p(d), diffs=out)


def analyse():
    cs = g10.cells()
    gates = json.load(open(OUT / "gates.json"))["gates"]
    chk = dict(gbridge_vs_diag=bitsame(OUT / "CHK_B_215348_s0_d1_gbridge_base", it.OUT / "B_215348_s0_d1_base"),
               G_closed_vs_base=bitsame(OUT / "CHK_T_214138_S1_d0_G", OUT / "CHK_T_214138_S1_d0_base"))
    if exists(OUT / "CHK_T_214138_S1_d0_G"):
        chk["G_closed_gate_open_frames"] = json.load(open(OUT / "CHK_T_214138_S1_d0_G.json")).get("g10_gate_open_frames")
    dep = [dep_cell(c) for c in cs if c["dep"]]
    rr = [r_cell(c, gates) for c in cs if c["kind"] == "R"]
    # ---- criterion 1: safety
    fp_bad, new_coll = [], []
    for r in dep + rr:
        if not r.get("tested") or not r.get("G") or not r.get("base"):
            continue
        if r["G"]["false_pass"] > r["base"]["false_pass"]:
            fp_bad.append([r["key"], r["base"]["false_pass"], r["G"]["false_pass"]])
        if r["base"]["status"] == "ok" and r["G"]["status"] != "ok":
            new_coll.append([r["key"], r["base"]["status"], r["G"]["status"]])
    n_safety = sum(1 for r in dep + rr if r.get("tested") and r.get("G") and r.get("base"))
    c1 = dict(n_cells=n_safety, false_pass_worse=fp_bad, new_collapses=new_coll, ok=(not fp_bad and not new_coll) if n_safety else None)
    # ---- criterion 2: regression on gate-opened R cells
    band_parts = {}
    for d_ in (1, 5):
        for cond in ("base", "i3"):
            vals = []
            mid = next(r for r in dep if r["key"] == f"B_213341_s0_d{d_}")
            for r in [mid] + [r for r in dep if r["key"].startswith(f"N_213341_s0_d{d_}_")]:
                x = r.get(cond)
                if x and x.get("ate_cm") is not None and x["status"] in ("ok", "collapse"):
                    vals.append(x["ate_cm"])
            if len(vals) >= 2:
                band_parts[f"d{d_}_{cond}"] = dict(vals=vals, range=round(max(vals) - min(vals), 3))
    band = max((v["range"] for v in band_parts.values()), default=None)
    opened = [r for r in rr if r.get("tested") and r["G"] and (r["G"].get("gate_open_frames") or 0) > 0
              and r["base"]["status"] in ("ok", "collapse") and r["G"]["status"] in ("ok", "collapse")]
    rd = [(r["key"], round(r["G"]["ate_cm"] - r["base"]["ate_cm"], 3)) for r in opened]
    dvals = [x for _, x in rd]
    med = float(np.median(dvals)) if dvals else None
    worst = max(dvals) if dvals else None
    c2a = None if med is None else bool(med <= 0.1 + 1e-9)
    c2b = None if (worst is None or band is None) else bool(worst <= band + 1e-9)
    c2 = dict(n_opened_scored=len(rd), diffs=rd, median=med, worst=worst, band=band, band_parts=band_parts, a_ok=c2a, b_ok=c2b,
              ok=None if (c2a is None or c2b is None) else (c2a and c2b),
              not_opened_but_possible=[r["key"] for r in rr if r.get("tested") and r["G"] and not r["G"].get("gate_open_frames")],
              not_run=[r["key"] for r in rr if r.get("tested") is None])
    # ---- criterion 3: 214550 gain
    t41 = [r for r in dep if r["rec"] == g10.R41 and r.get("tested") and r.get("G")]
    d41 = [(r["key"], round(r["G"]["ate_cm"] - r["base"]["ate_cm"], 3)) for r in t41
           if r["G"].get("ate_cm") is not None and r["base"].get("ate_cm") is not None]
    prev = [r["key"] for r in t41 if r["base"]["status"] == "collapse" and r["G"]["status"] == "ok"]
    m41 = float(np.median([x for _, x in d41])) if d41 else None
    c3 = dict(n_tested=len(t41), diffs=d41, median=m41, prevented=prev, a_ok=None if m41 is None else bool(m41 < 0),
              b_ok=len(prev) >= 2, ok=None if m41 is None else bool(m41 < 0 and len(prev) >= 2),
              base_collapses=[r["key"] for r in t41 if r["base"]["status"] == "collapse"],
              not_tested={r["key"]: r["why"] for r in dep if r["rec"] == g10.R41 and not r.get("tested")})
    # ---- criterion 4 (report): paired + takeoff rules applied to G vs base
    tested_dep = [r for r in dep if r.get("tested")]
    paired = {f"{a}-{b}": pairs(tested_dep, a, b) for a, b in (("G", "base"), ("i3", "base"), ("G", "i3"))}
    paired_41 = {f"{a}-{b}": pairs([r for r in tested_dep if r["rec"] == g10.R41], a, b) for a, b in (("G", "base"), ("i3", "base"))}
    paired_not41 = {f"{a}-{b}": pairs([r for r in tested_dep if r["rec"] != g10.R41], a, b) for a, b in (("G", "base"), ("i3", "base"))}
    rule_rows = [dict(key=r["key"], rec=r["rec"][-6:], window_inside=r["window_inside"], base=r["base"], i3=r["G"])
                 for r in tested_dep if r["kind"] == "B" and r.get("G")]
    takeoff_rules_G = ta.criteria(rule_rows) if rule_rows else None
    v1, v2, v3 = c1["ok"], c2["ok"], c3["ok"]
    chk_ok = chk["gbridge_vs_diag"] is True and chk["G_closed_vs_base"] is True
    txt = (f"214550 에서 G1.0 효과 {'있음' if v3 else ('확인 안 됨' if v3 is False else '판정 불가')} · "
           f"안전 {'통과' if v1 else ('실패' if v1 is False else '판정 불가')} · "
           f"회귀 {'통과' if v2 else ('실패' if v2 is False else '판정 불가')} (추정, 이득은 녹화 1 개)")
    if not chk_ok:
        txt = "동일성 점검 실패 또는 미완 → G 결과 판정에 안 씀. " + txt
    return dict(made=time.strftime("%F %T"), git=g10.blobs()[0], checks=chk, dep_rows=dep, r_rows=rr,
                verdict=dict(c1_safety=c1, c2_regression=c2, c3_gain_214550=c3, conclusion=txt, checks_ok=chk_ok),
                paired=paired, paired_214550=paired_41, paired_not_214550=paired_not41, takeoff_rules_G_vs_base=takeoff_rules_G,
                counts=dict(dep_tested=len(tested_dep), dep_total=len(dep),
                            r_tested=sum(1 for r in rr if r.get("tested")), r_total=len(rr),
                            g_runs=sum(1 for r in dep + rr if r.get("G"))))
