#!/usr/bin/env python3
"""Analysis + HTML for imu_learn/i3_takeoff.py (docs/i3_takeoff_plan.md). Called via `i3_takeoff.py analyse|html`."""
import base64
import gzip
import html as H
import io
import json
import re
import time
from pathlib import Path

import numpy as np

import c5_overnight_analyse as an
import i3_confirm as ic
import i3_takeoff as it
from imu_gt import read_imu

OUT, SRC = it.OUT, it.SRC
ANSI = re.compile(r"\x1b\[[0-9;]*m")
G = 9.81
_ACC = {}


# ---------------------------------------------------------------- raw readers
def load_run(prefix):
    d = json.load(open(str(prefix) + ".json"))
    P = [json.loads(x) for x in gzip.open(str(prefix) + ".poses.jsonl.gz", "rt")]
    P = [m for m in P if m.get("type", "pose") == "pose"]
    t = np.array([m["t_ns"] for m in P], np.int64)
    tc = np.array([m.get("t", np.nan) for m in P], float)
    st = np.array([m.get("status", "") for m in P])

    def col(k, n):
        return np.array([m[k] if m.get(k) is not None else [np.nan] * n for m in P], float).reshape(len(P), n)
    T = col("T_world_cam0", 16)
    trk = (st == "tracking") & np.isfinite(T[:, 0])
    return dict(d=d, t=t, tc=tc, trk=trk, X=T.reshape(-1, 4, 4)[:, :3, 3], v=col("v", 3), ba=col("ba", 3), bg=col("bg", 3),
                n_track=np.array([m.get("n_track") or 0 for m in P], float))


def zupt(prefix, r):
    """Per-frame ZUPT decision from stderr: [DIAGT] <clip t> then '[ZUPT]: passed|failed disparity (x > 0.5, n features)'.
    Clip time -> IMU-clock ns by linear interpolation on the run's own pose (t, t_ns)."""
    ok = np.isfinite(r["tc"])
    tc, tn = r["tc"][ok], r["t"][ok].astype(float)
    o = np.argsort(tc)
    tc, tn = tc[o], tn[o]
    rows, cur = [], None
    for line in gzip.open(str(prefix) + ".stderr.log.gz", "rt", errors="replace"):
        line = ANSI.sub("", line)
        if "[DIAGT]" in line:
            cur = float(line.split("[DIAGT]")[1])
        elif cur is not None and ("[ZUPT]: passed disparity" in line or "[ZUPT]: failed disparity" in line):
            m = re.search(r"\(([-0-9.]+) [<>] [0-9.]+, (\d+) features\)", line)
            rows.append((cur, "passed" in line, float(m[1]) if m else np.nan, int(m[2]) if m else -1))
    if not rows:
        return dict(t=np.zeros(0), ok=np.zeros(0, bool), disp=np.zeros(0), nf=np.zeros(0))
    a = np.array(rows, float)
    return dict(t=np.interp(a[:, 0], tc, tn), ok=a[:, 1] > 0.5, disp=a[:, 2], nf=a[:, 3])


def acc_raw(rec):
    if rec not in _ACC:
        acc, _ = read_imu(Path("recordings") / rec / "imu.csv", raw=True)
        _ACC[rec] = (acc[:, 0].astype(np.int64), np.linalg.norm(acc[:, 1:4], axis=1))
    return _ACC[rec]


def frac(z, lo, hi):
    m = (z["t"] >= lo) & (z["t"] < hi)
    return (None if not m.any() else round(float(z["ok"][m].mean()), 4)), int(m.sum())


# ---------------------------------------------------------------- B
def window(r, rec, t_dep):
    """10 s window from departure: SE3 ATE + Sim3 scale as c5_overnight_analyse (VIO t + align_dt, GT nearest 2 ms)."""
    gt_t, gt_p, _ = an.gt(rec, "imu_gt_spl")
    dt = int(r["d"]["align_dt_ms"] * 1e6)
    w = r["trk"] & (r["t"] >= t_dep) & (r["t"] <= t_dep + it.WIN_S * 1e9)
    if not r["trk"].any():
        return dict(status="noinit", n_match=0)
    j, mm = an.match(r["t"][w] + dt, gt_t)
    if mm.sum() < it.MIN_MATCH:
        return dict(status="unscored", n_match=int(mm.sum()))
    a, a_s, c = an.ate(r["X"][w][mm], gt_p[j[mm]])
    st = "collapse" if (c < 0.5 or c > 2 or a > 10) else "ok"
    return dict(status=st, n_match=int(mm.sum()), ate_cm=round(a, 3), ate_sim3_cm=round(a_s, 3), scale=round(c, 4))


def b_run(prefix, cl, x):
    r = load_run(prefix)
    rec = cl["rec"]
    st0, st1 = cl["still_imu_ns"]
    t_dep = st1
    f0 = x["first_frame_imu_ns"]
    w = window(r, rec, t_dep)
    ft = int(r["t"][r["trk"]][0]) if r["trk"].any() else None
    z = zupt(prefix, r)
    zf, zn = frac(z, max(ft, st0), t_dep) if ft is not None and ft < t_dep else (None, 0)
    d = r["d"]
    out = dict(name=Path(prefix).name, **w, t_dep_s=round((t_dep - f0) / 1e9, 2),
               first_trk_s=None if ft is None else round((ft - f0) / 1e9, 2),
               first_trk_before_dep=None if ft is None else bool(ft < t_dep),
               zupt_still=zf, zupt_still_n=zn, zupt_still_all=frac(z, st0, t_dep), bench_ate_cm=d.get("ate_cm"), bench_scale=d.get("scale"),
               t_init_clip_s=d.get("t_init_clip_s"), frames_held=d.get("frames_held"),
               holds_s=[[round((a - f0) / 1e9, 2), None if b is None else round((b - f0) / 1e9, 2)]
                        for a, b in (d.get("hold_segments_imu_ns") or [])],
               hold_wait_timeouts=d.get("hold_wait_timeouts"), cfg_over=d.get("cfg_over"), still_hold=d.get("still_hold"),
               start_off=d.get("start_off"), max_secs=d.get("max_secs", None), error=d.get("error"),
               version=json.dumps(dict(bench=d.get("bench_blob"), runner=d.get("runner_blob"), i3t=d.get("i3t_blobs"),
                                       bin=sorted((d.get("binary_sha256") or {}).values())), sort_keys=True))
    # arming: held frames must start >= 3 s after init
    if d.get("hold_segments_imu_ns") and d.get("hold_init_ns"):
        out["arm_s"] = round((min(a for a, _ in d["hold_segments_imu_ns"]) - d["hold_init_ns"]) / 1e9, 3)
    return out


def analyse_B():
    cells = it.cells()
    rows = []
    for cl in cells:
        x = it.inp(cl["rec"])
        f0 = x["first_frame_imu_ns"]
        row = dict(key=cl["key"], rec=cl["rec"], si=cl["si"], d=cl["d"], still_s=cl["still_s"], off=cl.get("off"),
                   left_s=cl.get("left_s"), skip=cl.get("skip"))
        if cl.get("skip"):
            rows.append(row)
            continue
        st0, st1 = cl["still_imu_ns"]
        clip_end = f0 + int((cl["off"] + it.MAX_SECS) * 1e9)
        lim = min(x["last_frame_imu_ns"], clip_end)
        row["window_inside"] = bool(st1 + it.WIN_S * 1e9 <= lim)
        row["window_end_short_s"] = round((st1 + it.WIN_S * 1e9 - lim) / 1e9, 2)
        ta, aa = acc_raw(cl["rec"])
        m = (ta >= st1 - 0.5e9) & (ta <= st1 + 1.0e9)
        row["impact_g"] = round(float(aa[m].max()) / G, 2) if m.any() else None
        for c in it.CONDS:
            p = OUT / f"B_{cl['key']}_{c}"
            row[c] = b_run(p, cl, x) if Path(str(p) + ".json").exists() else None
        rows.append(row)
    tested = [r for r in rows if not r.get("skip") and r.get("window_inside") and r.get("base") and r.get("i3")]
    new = [r["key"] for r in tested if r["base"]["status"] == "ok" and r["i3"]["status"] != "ok"]
    prev = [r["key"] for r in tested if r["base"]["status"] == "collapse" and r["i3"]["status"] == "ok"]
    other = [r["key"] for r in tested if r["base"]["status"] in ("noinit", "unscored") and r["i3"]["status"] == "ok"]
    prev_recs = sorted({k.split("_")[0] for k in prev})
    base_coll = [r["key"] for r in tested if r["base"]["status"] == "collapse"]
    both = [r for r in tested if r["base"].get("ate_cm") is not None and r["i3"].get("ate_cm") is not None]
    bm = float(np.median([r["base"]["ate_cm"] for r in both])) if both else None
    zm = float(np.median([r["i3"]["ate_cm"] for r in both])) if both else None
    lim = None if bm is None else max(bm * 1.10, bm + 0.2)
    c1 = not new
    c2 = len(prev) >= 2 and len(prev_recs) >= 2
    c3 = None if lim is None else bool(zm <= lim + 1e-9)
    recs_tested = sorted({r["rec"] for r in tested})
    if not tested:
        concl = "시험 안 됨 (시험 칸 없음)"
    elif not base_coll:
        concl = "시험 안 됨 (base 무너짐 0 — 이득 아님)"
    elif c1 and c2 and c3:
        concl = f"긴 정지 뒤 출발에서 i3 이득 (추정, 녹화 {len(recs_tested)} 개) — 앱 반영은 사용자 확인 후"
    else:
        concl = "이득 확인 안 됨"
    per_rec = {}
    for rec in it.RECS:
        rr = [r for r in both if r["rec"] == rec]
        if rr:
            per_rec[rec] = dict(n=len(rr), base_med=float(np.median([r["base"]["ate_cm"] for r in rr])),
                                i3_med=float(np.median([r["i3"]["ate_cm"] for r in rr])))
    crit = dict(n_cells=len(rows), n_run_cells=sum(not r.get("skip") for r in rows), n_tested=len(tested),
                recs_tested=recs_tested, not_tested=[r["key"] for r in rows if not r.get("skip") and not r.get("window_inside")],
                skipped={r["key"]: r["skip"] for r in rows if r.get("skip")},
                base_collapses=base_coll, i_new=new, i_ok=c1, ii_prevented=prev, ii_recs=prev_recs, ii_ok=c2,
                ii_other_base_noinit_i3_ok=other, iii_n=len(both), iii_base_med=bm, iii_i3_med=zm, iii_lim=lim, iii_ok=c3,
                per_rec=per_rec, conclusion=concl)
    # still re-check (amendment 1 function on raw gyro == inputs json)
    still_chk = {}
    for rec in it.RECS:
        _, gyr = read_imu(Path("recordings") / rec / "imu.csv", raw=True)
        runs = ic.still_runs(gyr[:, 0].astype(np.int64), gyr[:, 1:4])
        x = it.inp(rec)
        still_chk[rec] = all(w["still_imu_ns"] in runs for w in x["setdown"])
    return dict(rows=rows, criteria=crit, still_recheck_same=still_chk)


# ---------------------------------------------------------------- A
def bitsame(p1, p2):
    a, b = load_run(p1), load_run(p2)
    if a["trk"].sum() != b["trk"].sum():
        return False
    return bool(np.array_equal(a["t"][a["trk"]], b["t"][b["trk"]]) and np.array_equal(a["X"][a["trk"]], b["X"][b["trk"]]))


def analyse_A():
    rec = it.A_REC
    x = it.inp(rec)
    gts = ic.gt_gated_speed(rec)
    out = {}
    allpairs = []
    for c in it.CONDS:
        pre = [SRC / f"{rec[-6:]}_{c}_o1"] + [OUT / f"A_{rec[-6:]}_{c}_o1_r{r}" for r in (1, 2)]
        pre = [p for p in pre if Path(str(p) + ".json").exists()]
        ms = [ic.run_metrics(p, x, gts) for p in pre]
        vals = [[w.get("drift_cm") for w in m["setdown"]] for m in ms]
        pairs = []
        for i in range(len(ms)):
            for k in range(i + 1, len(ms)):
                dw = [None if a is None or b is None else round(abs(a - b), 4) for a, b in zip(vals[i], vals[k])]
                pairs.append(dict(a=ms[i]["name"], b=ms[k]["name"], d_win=dw,
                                  d_ate=None if ms[i]["ate_cm"] is None or ms[k]["ate_cm"] is None
                                  else round(abs(ms[i]["ate_cm"] - ms[k]["ate_cm"]), 3),
                                  bitsame=bitsame(pre[i], pre[k])))
        blob = [json.load(open(str(p) + ".json")).get("bench_blob") for p in pre]
        out[c] = dict(runs=[m["name"] for m in ms], drift=vals, ate=[m["ate_cm"] for m in ms], pairs=pairs, bench_blobs=blob)
        allpairs += pairs
    ds = [v for p in allpairs for v in p["d_win"] if v is not None]
    dmax = max(ds) if ds else None
    new_only = [v for p in allpairs if p["a"].startswith("A_") for v in p["d_win"] if v is not None]
    if dmax is None:
        verdict = "계산 불가"
    elif dmax == 0 and all(p["bitsame"] for p in allpairs):
        verdict = "비트 동일 → d_rep = 0, 215348 의 차이는 '붙잡기 켜고 끔이 만든 차이' 로 기록 (이번 판정 불변)"
    elif dmax >= 0.05:
        verdict = (f"d_rep 최대 {dmax:.3f} cm ≥ 0.05 → '기준 4 같음 띠 0.05 cm 는 실행 잡음보다 좁음'. "
                   f"다음 시험 같음 띠 = max(0.05, 2 × {dmax:.3f}) = {max(0.05, 2 * dmax):.3f} cm")
    else:
        verdict = f"d_rep 최대 {dmax:.3f} cm < 0.05 (비트 동일은 아님) → 같음 띠 0.05 cm 유지"
    return dict(conds=out, d_rep_max=dmax, d_rep_max_new_only=max(new_only) if new_only else None,
                all_bitsame=all(p["bitsame"] for p in allpairs), verdict=verdict,
                next_tie_band=None if dmax is None else max(0.05, 2 * dmax))


# ---------------------------------------------------------------- C
def analyse_C():
    rec = "hs_20261021_214550"
    x = it.inp(rec)
    f0 = x["first_frame_imu_ns"]
    t_dep = x["start_still_imu_ns"][1]
    res, series = {}, {}
    for k in range(3):
        for c in it.CONDS:
            pre = SRC / f"{rec[-6:]}_{c}_o{k}"
            r = load_run(pre)
            z = zupt(pre, r)
            w = r["trk"] & (r["t"] >= t_dep - 0.5e9) & (r["t"] < t_dep)
            ba = np.linalg.norm(r["ba"], axis=1)
            v = np.linalg.norm(r["v"], axis=1)
            bg = np.linalg.norm(r["bg"], axis=1)
            m2 = r["trk"] & (r["t"] >= t_dep) & (r["t"] <= t_dep + 2e9)
            gt_t, gt_p, _ = an.gt(rec, "imu_gt_spl")
            dtn = int(r["d"]["align_dt_ms"] * 1e6)
            jj, okk = an.match(r["t"][m2] + dtn, gt_t)
            vio2 = float(np.linalg.norm(r["X"][m2][-1] - r["X"][m2][0]) * 100) if m2.sum() > 1 else None
            gt2 = float(np.linalg.norm(gt_p[jj[okk]][-1] - gt_p[jj[okk]][0]) * 100) if okk.sum() > 1 else None
            zb = frac(z, t_dep - 2e9, t_dep)
            za = frac(z, t_dep, t_dep + 2e9)
            res[f"{c}_o{k}"] = dict(ba_pre=None if not w.any() else float(np.median(ba[w])),
                                    v_pre=None if not w.any() else float(np.median(v[w])),
                                    bg_pre=None if not w.any() else float(np.median(bg[w])), n_pre=int(w.sum()),
                                    zupt_before_2s=zb, zupt_after_2s=za, move2_vio_cm=vio2, move2_gt_cm=gt2)
            m = r["trk"] & (r["t"] >= t_dep - 2e9) & (r["t"] <= t_dep + 2e9)
            mz = (z["t"] >= t_dep - 2e9) & (z["t"] <= t_dep + 2e9)
            series[f"{c}_o{k}"] = dict(t=((r["t"][m] - t_dep) / 1e9).tolist(), v=v[m].tolist(), ba=ba[m].tolist(),
                                       bg=bg[m].tolist(), n_track=r["n_track"][m].tolist(),
                                       zt=((z["t"][mz] - t_dep) / 1e9).tolist(), zd=z["disp"][mz].tolist(),
                                       zok=z["ok"][mz].astype(int).tolist(), znf=z["nf"][mz].tolist())
    ratios = {}
    for q in ("ba_pre", "v_pre"):
        rr = []
        for k in range(3):
            b, i = res[f"base_o{k}"][q], res[f"i3_o{k}"][q]
            rr.append(None if b is None or i is None or i == 0 else b / i)
        ok = [r for r in rr if r is not None]
        ratios[q] = dict(per_start=rr, median=float(np.median(ok)) if ok else None)
    hit = any((ratios[q]["median"] or 0) >= 2 for q in ratios)
    verdict = ("정지 중 바이어스 오추정 → 출발 순간 튐 (추정, 유력 가설)" if hit else "기작 미상 (추정)")
    return dict(rec=rec, t_dep_s=round((t_dep - f0) / 1e9, 2), runs=res, ratios=ratios, verdict=verdict, series=series)


def analyse():
    head, b, dirty = it.blobs()
    res = dict(made=time.strftime("%F %T"), git=head, blobs=b, dirty=dirty)
    res["A"] = analyse_A()
    res["B"] = analyse_B()
    res["C"] = analyse_C()
    runs = [r[c] for r in res["B"]["rows"] for c in it.CONDS if r.get(c)]
    res["B"]["code_versions"] = len({x["version"] for x in runs})
    res["B"]["n_runs"] = len(runs)
    res["B"]["threads"] = sorted({str(x["cfg_over"]) for x in runs})
    res["B"]["wait_timeouts"] = {x["name"]: x["hold_wait_timeouts"] for x in runs if x.get("hold_wait_timeouts")}
    res["B"]["arm"] = {x["name"]: x["arm_s"] for x in runs if "arm_s" in x}
    res["B"]["errors"] = {x["name"]: x["error"] for x in runs if x.get("error")}
    return res


# ---------------------------------------------------------------- HTML
def f_(x, nd=2):
    if x is None:
        return "–"
    if isinstance(x, float):
        return f"{x:.{nd}f}"
    if isinstance(x, (list, tuple)):
        return ", ".join(f_(y, nd) for y in x)
    return H.escape(str(x))


def png(fig):
    b = io.BytesIO()
    fig.savefig(b, format="png", dpi=110, bbox_inches="tight")
    return "<img alt='plot' style='max-width:100%' src='data:image/png;base64," + base64.b64encode(b.getvalue()).decode() + "'>"


def plot_B(rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6.2, 5.2))
    cols = {"213341": "#1f77b4", "213955": "#9467bd", "214550": "#d62728", "215348": "#2ca02c"}
    for r in rows:
        b, z = r.get("base"), r.get("i3")
        if not b or not z or b.get("ate_cm") is None or z.get("ate_cm") is None:
            continue
        mk = "o" if r.get("window_inside") else "x"
        ax.scatter(b["ate_cm"], z["ate_cm"], c=cols.get(r["rec"][-6:], "k"), marker=mk, s=40,
                   label=r["rec"][-6:] if mk == "o" else None)
    lo, hi = 0.05, 200
    ax.plot([lo, hi], [lo, hi], color="#888", lw=1)
    ax.axvline(10, color="#c66", ls=":", lw=1)
    ax.axhline(10, color="#c66", ls=":", lw=1)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlim(lo, hi)
    ax.set_ylim(lo, hi)
    ax.set_xlabel("base: ATE in 10 s after departure (cm)")
    ax.set_ylabel("i3: ATE in 10 s after departure (cm)")
    h, lab = ax.get_legend_handles_labels()
    u = dict(zip(lab, h))
    ax.legend(u.values(), u.keys(), fontsize=8, title="recording (o tested, x reference)")
    ax.set_title("below diagonal = i3 smaller; dotted 10 cm = collapse limit", fontsize=9)
    s = png(fig)
    plt.close(fig)
    return s


def plot_C(series):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(4, 1, figsize=(7.5, 9), sharex=True)
    cc = ["#1f77b4", "#ff7f0e", "#2ca02c"]
    for k in range(3):
        for c, ls in (("base", "-"), ("i3", "--")):
            s = series[f"{c}_o{k}"]
            lab = f"{c} o{k}"
            axs[0].plot(s["t"], s["v"], ls, color=cc[k], lw=1, label=lab)
            axs[1].plot(s["t"], s["ba"], ls, color=cc[k], lw=1)
            axs[2].plot(s["zt"], s["zd"], ls, color=cc[k], lw=0.8)
            axs[3].plot(s["t"], s["n_track"], ls, color=cc[k], lw=0.8)
    axs[0].set_ylabel("|v| (m/s)")
    axs[1].set_ylabel("|ba| (m/s^2)")
    axs[2].set_ylabel("ZUPT disparity (px)")
    axs[2].axhline(0.5, color="#c33", lw=1, ls=":")
    axs[3].set_ylabel("tracked features")
    axs[3].set_xlabel("time from departure = IMU still end (s)")
    for a in axs:
        a.axvline(0, color="#555", lw=0.8)
    axs[0].legend(fontsize=7, ncol=3)
    axs[0].set_title("214550 start still -> departure +-2 s (solid base, dashed i3, colour = start)", fontsize=9)
    s = png(fig)
    plt.close(fig)
    return s


ST = {"ok": "정상", "collapse": "무너짐", "noinit": "초기화 못 함", "unscored": "점수 없음"}


def write_html(res, path, check=None):
    A, B, C = res["A"], res["B"], res["C"]
    c = B["criteria"]
    o = ["<!doctype html><html lang='ko'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>",
         "<title>i3 출발 시험</title><style>body{font-family:system-ui,'Malgun Gothic',sans-serif;max-width:1000px;margin:0 auto;"
         "padding:16px;line-height:1.55;color:#1d232b;background:#fff}h1{font-size:1.45em}h2{margin-top:1.6em;font-size:1.15em}"
         "table{border-collapse:collapse;width:100%;font-size:.86em;margin:.5em 0}td,th{border:1px solid #ccd;padding:4px 6px;"
         "text-align:left;vertical-align:top}th{background:#f1f3f7}.big{font-size:1.2em;font-weight:700;padding:12px;"
         "border-radius:8px;background:#eef2f8}.mut{color:#5b6573;font-size:.9em}.ok{color:#17803d;font-weight:600}"
         ".bad{color:#b42318;font-weight:600}.wrap{overflow-x:auto}</style></head><body>"]
    o.append("<h1>i3 (가만히 있을 때 화면 고정) — 오래 둔 뒤 출발 시험</h1>")
    o.append(f"<div class='big'>결론 (실험 B): {H.escape(c['conclusion'])}</div>")
    o.append("<ul>")
    o.append(f"<li>시험 칸 {c['n_tested']} 개 / 실행한 칸 {c['n_run_cells']} 개 (출발 뒤 10 s 가 녹화 안에 있는 칸만 시험). "
             f"시험 안 됨: {H.escape(', '.join(c['not_tested']) or '없음')}</li>")
    o.append(f"<li>base 가 무너진 칸: {len(c['base_collapses'])} 개 {H.escape(str(c['base_collapses']))}</li>")
    o.append(f"<li>(i) i3 새 무너짐 = 0 이어야 함: <b class='{'ok' if c['i_ok'] else 'bad'}'>{len(c['i_new'])} 개</b> {H.escape(str(c['i_new']))}</li>")
    o.append(f"<li>(ii) 막은 무너짐 ≥ 2 그리고 녹화 ≥ 2 개: <b class='{'ok' if c['ii_ok'] else 'bad'}'>{len(c['ii_prevented'])} 개, 녹화 "
             f"{len(c['ii_recs'])} 개</b> {H.escape(str(c['ii_prevented']))}"
             f"{' (base 초기화 못 함/점수 없음 + i3 정상, 세지 않음: ' + H.escape(str(c['ii_other_base_noinit_i3_ok'])) + ')' if c['ii_other_base_noinit_i3_ok'] else ''}</li>")
    o.append(f"<li>(iii) 출발 뒤 10 s ATE 중앙값 (칸 {c['iii_n']} 개 모음): base {f_(c['iii_base_med'])} → i3 {f_(c['iii_i3_med'])} cm, "
             f"한계 {f_(c['iii_lim'])}: <b class='{'ok' if c['iii_ok'] else 'bad'}'>{'통과' if c['iii_ok'] else ('계산 불가' if c['iii_ok'] is None else '실패')}</b></li>")
    o.append("</ul>")
    o.append(f"<p><b>실험 A (실행 간 잡음):</b> {H.escape(A['verdict'])}.</p>")
    o.append(f"<p><b>실험 C (출발 순간 기작, 214550):</b> {H.escape(C['verdict'])} — 출발 직전 0.5 s 의 base/i3 비율 중앙값: "
             f"|ba| {f_(C['ratios']['ba_pre']['median'])}, |v| {f_(C['ratios']['v_pre']['median'])} (≥ 2 이면 유력 가설).</p>")
    o.append("<p class='mut'>기준·지표는 실행 전에 docs/i3_takeoff_plan.md 에 커밋 (4718321, 00:25:53) — 판정자 문구 그대로. "
             "녹화 4 개뿐이고 같은 녹화의 시작 3 개는 독립 반복이 아니다. 모든 결론은 추정.</p>")
    if check is not None:
        mm = check.get("mismatch") or []
        o.append(f"<p class='mut'>검산 (imu_learn/i3_takeoff_check.py, 분석 코드를 import 하지 않고 원 파일에서 다시 계산): "
                 f"<b>{'일치' if not mm else '불일치 ' + H.escape(str(mm)[:600])}</b> — 비교 {check.get('n_checked')} 항목.</p>")
    o.append("<h2>실험 B — 칸별 (녹화 × 내려놓기 정지 × 시작)</h2>" + plot_B(B["rows"]))
    o.append("<div class='wrap'><table><tr><th>칸</th><th>정지 (s)</th><th>클립 시작 (s)</th><th>출발 (s)</th><th>시험?</th>"
             "<th>base 출발 뒤 10 s: 상태 / 축척 / ATE (cm)</th><th>i3 같은 값</th><th>base ZUPT 통과율: 첫 추적 뒤 정지 (등록) / 정지 전체 (참고)</th>"
             "<th>출발 충격 (g)</th><th>첫 추적 (s) base / i3</th></tr>")
    for r in B["rows"]:
        if r.get("skip"):
            o.append(f"<tr><td>{r['key']}</td><td colspan='9'>실행 안 함: {H.escape(r['skip'])}</td></tr>")
            continue
        b, z = r.get("base") or {}, r.get("i3") or {}

        def cell(x):
            if not x:
                return "–"
            s = ST.get(x.get("status"), x.get("status"))
            cls = "bad" if x.get("status") != "ok" else ""
            return f"<span class='{cls}'>{s}</span> / {f_(x.get('scale'))} / {f_(x.get('ate_cm'))}"
        o.append(f"<tr><td>{r['key']}</td><td>{f_(r['still_s'])}</td><td>{f_(r['off'])}</td><td>{f_(b.get('t_dep_s'))}</td>"
                 f"<td>{'예' if r.get('window_inside') else '아니오 (출발 뒤 10 s 가 ' + f_(r.get('window_end_short_s')) + ' s 모자람)'}</td>"
                 f"<td>{cell(b)}</td><td>{cell(z)}</td><td>{f_(b.get('zupt_still'))} ({b.get('zupt_still_n', 0)} 판정) / 전체 {f_((b.get('zupt_still_all') or [None])[0])} "
                 f"({(b.get('zupt_still_all') or [0, 0])[1]})</td>"
                 f"<td>{f_(r.get('impact_g'))}</td><td>{f_(b.get('first_trk_s'))} / {f_(z.get('first_trk_s'))}</td></tr>")
    o.append("</table></div>")
    pr = c.get("per_rec") or {}
    if pr:
        o.append("<p class='mut'>녹화별 출발 뒤 10 s ATE 중앙값 (참고, 판정은 모은 값): " +
                 "; ".join(f"{k[-6:]} base {v['base_med']:.2f} → i3 {v['i3_med']:.2f} cm (칸 {v['n']})" for k, v in pr.items()) + "</p>")
    o.append("<p class='mut'>참고 (시험 안 된 칸 포함): 클립 전체 ATE base → i3 (cm): " +
             "; ".join(f"{r['key']} {f_((r.get('base') or {}).get('bench_ate_cm'))} → {f_((r.get('i3') or {}).get('bench_ate_cm'))}"
                       for r in B["rows"] if not r.get("skip")) + "</p>")
    o.append("<h2>실험 A — 같은 설정 반복 (215348 o1)</h2><div class='wrap'><table><tr><th>조건</th><th>실행</th>"
             "<th>기준 4 창 값 (cm)</th><th>전체 ATE (cm)</th></tr>")
    for cn, x in A["conds"].items():
        for n, dv, a in zip(x["runs"], x["drift"], x["ate"]):
            o.append(f"<tr><td>{cn}</td><td>{H.escape(n)}</td><td>{f_(dv, 3)}</td><td>{f_(a)}</td></tr>")
    o.append("</table></div><div class='wrap'><table><tr><th>짝</th><th>창 값 차이 d_rep (cm)</th><th>ATE 차이 (cm)</th><th>비트 동일?</th></tr>")
    for cn, x in A["conds"].items():
        for p in x["pairs"]:
            o.append(f"<tr><td>{H.escape(p['a'])} ↔ {H.escape(p['b'])}</td><td>{f_(p['d_win'], 4)}</td><td>{f_(p['d_ate'], 3)}</td>"
                     f"<td>{'예' if p['bitsame'] else '아니오'}</td></tr>")
    o.append(f"</table></div><p>d_rep 최대 {f_(A['d_rep_max'], 4)} cm (새 실행끼리만 {f_(A['d_rep_max_new_only'], 4)} cm). "
             f"다음 시험 같음 띠 {f_(A['next_tie_band'], 3)} cm. 원 실행과 새 실행의 벤치 코드 blob: "
             f"{H.escape(str(sorted({b for x in A['conds'].values() for b in x['bench_blobs']})))}.</p>")
    o.append("<h2>실험 C — 214550 출발 순간 (기존 실행, 새 VIO 없음)</h2>" + plot_C(C["series"]))
    o.append("<div class='wrap'><table><tr><th>실행</th><th>출발 직전 0.5 s |ba| (m/s²)</th><th>|v| (m/s)</th><th>|bg| (rad/s)</th>"
             "<th>ZUPT 통과율 출발 전 2 s</th><th>출발 뒤 2 s</th><th>출발 뒤 2 s 이동 VIO / 정답 (cm, 참고)</th></tr>")
    for k, x in C["runs"].items():
        o.append(f"<tr><td>{k}</td><td>{f_(x['ba_pre'], 4)}</td><td>{f_(x['v_pre'], 4)}</td><td>{f_(x['bg_pre'], 5)}</td>"
                 f"<td>{f_(x['zupt_before_2s'])}</td><td>{f_(x['zupt_after_2s'])}</td>"
                 f"<td>{f_(x.get('move2_vio_cm'), 1)} / {f_(x.get('move2_gt_cm'), 1)}</td></tr>")
    o.append("</table></div>")
    o.append(f"<p>base/i3 비율 (시작 o0, o1, o2): |ba| {f_(C['ratios']['ba_pre']['per_start'])}, |v| {f_(C['ratios']['v_pre']['per_start'])}.</p>")
    o.append("<h2>실행 점검</h2><ul>")
    o.append(f"<li>B 실행 {B.get('n_runs')} 개, 코드 버전 {B.get('code_versions')} 개 (1 이어야 함), 스레드 {f_(B.get('threads'))}, "
             f"i3 대기 시간 초과 {f_(B.get('wait_timeouts') or '없음')}, 오류 {f_(B.get('errors') or '없음')}</li>")
    o.append(f"<li>내려놓기 정지 재계산 (i3_confirm.still_runs, 원본 자이로) = inputs json: {f_(B.get('still_recheck_same'))}</li>")
    o.append(f"<li>i3 첫 고정 − 초기화 (s, 3 이상): {H.escape(', '.join(f'{k}: {v}' for k, v in (B.get('arm') or {}).items()) or '–')}</li>")
    o.append(f"<li>git {H.escape(str(res.get('git')))}, 분석 {H.escape(res.get('made', ''))}</li></ul>")
    o.append("<p class='mut'>용어: 출발 = IMU(자이로) 정지가 끝난 시각. 출발 뒤 10 s 창 = 그 시각부터 10 초의 추적 위치를 정답에 맞춤. "
             "무너짐 = 그 창 축척 0.5–2 밖 또는 오차(ATE) 10 cm 넘음. ZUPT = 영상이 거의 안 움직이면 '정지' 로 보고 속도 0 을 넣는 원래 기능 "
             "(시차 0.5 px 미만일 때만 통과). 코드 imu_learn/i3_takeoff.py, i3_takeoff_analyse.py, 검산 i3_takeoff_check.py, 결과 runs/i3_takeoff/.</p>")
    o.append("</body></html>")
    Path(path).write_text("\n".join(o), encoding="utf-8")
