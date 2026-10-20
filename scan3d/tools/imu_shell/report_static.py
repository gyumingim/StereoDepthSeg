"""HTML report for analyze_static.py (docs/gyro_bias_stability.html)."""
import html
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from analyze_static import png

C = {"x": "#2a6fdb", "y": "#e07b00", "z": "#1a9e5a"}


def f(v, n=2):
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return "–"
    return f"{v:.{n}f}"


def table(head, rows):
    h = "<tr>" + "".join(f"<th>{html.escape(str(x))}</th>" for x in head) + "</tr>"
    b = "".join("<tr>" + "".join(f"<td>{x}</td>" for x in r) + "</tr>" for r in rows)
    return f"<div class=tw><table>{h}{b}</table></div>"


def best_temp(res):
    th = res.get("thermal")
    if th and th.get("best_source"):
        return th["best_source"], th["per_source"][th["best_source"]]
    tc = res["temp_correlation"]
    if not tc:
        return None, None
    # strongest mean |r| over gyro axes, among sources with a real range
    best = None
    for k, v in tc.items():
        rs = [abs(v[f"gyro_raw_mdeg_s_{a}"]["r"]) for a in "xyz" if math.isfinite(v[f"gyro_raw_mdeg_s_{a}"]["r"])]
        sc = np.mean(rs) if rs else -1
        if best is None or sc > best[1]:
            best = (k, sc)
    return best[0], tc[best[0]]


def write(res, arr, path):
    A = res["allan"]
    g, a = A["gyro_raw"], A["accel_raw"]
    curves = arr["curves"]
    # ---------------- plots
    fig, axs = plt.subplots(1, 2, figsize=(11, 4))
    for ax_, nm, ylab, sc in ((axs[0], "gyro_raw", "Allan dev (°/s)", 1), (axs[1], "accel_raw", "Allan dev (mg)", 1e3 / 9.81)):
        for k in "xyz":
            t, s = curves[(nm, k)]
            ax_.loglog(t, s * sc, color=C[k], label=k)
        ax_.set_xlabel("τ (s)"); ax_.set_ylabel(ylab); ax_.grid(True, which="both", alpha=.3); ax_.legend()
    axs[0].set_title("자이로 (raw = uncalibrated)"); axs[1].set_title("가속도 (raw)")
    p_allan = png(fig)

    b = res["block10s"]
    bt = np.array(b["t"]) / 60
    bm = np.array(b["gyro_raw_mdeg_s"])
    fig, ax1 = plt.subplots(figsize=(11, 4))
    for j, k in enumerate("xyz"):
        if len(bm):
            ax1.plot(bt, bm[:, j] - bm[:, j].mean(), "o-", ms=3, color=C[k], label=f"자이로 {k} (평균 {bm[:, j].mean():.1f} 빼고)")
    ax1.set_xlabel("시간 (분)"); ax1.set_ylabel("10 s 평균 자이로 − 전체 평균 (mdeg/s)"); ax1.grid(alpha=.3)
    T = res["temperature"]
    if T:
        ax2 = ax1.twinx()
        tt = np.array(T["t"]) / 60
        for k, c in (("battery", "#d33"), ("AP", "#555"), ("SKIN", "#999")):
            if k in T:
                ax2.plot(tt, T[k], "--", color=c, label=f"온도 {k}")
        ax2.set_ylabel("°C"); ax2.legend(loc="upper right", fontsize=8)
    ax1.legend(loc="upper left", fontsize=8)
    p_blk = png(fig)

    hs = res["hs_recordings"]
    fig, ax_ = plt.subplots(figsize=(11, 4))
    for r in hs:
        if len(r["segments"]) < 2:
            continue
        tt = [(s["t0"] + s["t1"]) / 2 for s in r["segments"]]
        m = np.array([s["gyro_mean_mdeg_s"] for s in r["segments"]])
        nrm = m - m[0]
        ax_.plot(tt, np.linalg.norm(nrm, axis=1), "o-", ms=3, label=r["name"][3:])
    ax_.set_xlabel("녹화 시작부터 초"); ax_.set_ylabel("|정지 구간 평균 − 첫 정지 구간| (mdeg/s)")
    ax_.grid(alpha=.3); ax_.legend(fontsize=7, ncol=3)
    p_hs = png(fig)

    th = res.get("thermal")
    p_th = p_ts = None
    if th:
        src = th.get("best_source") or "battery"
        Tb = np.array(th["T_blocks"][src]); dT = np.array(th["dTdt_blocks"][src])
        cam = np.array(th["cam"]); bg = np.array(th["gyro_mdeg_s_earth_corr"]); ba = np.array(th["accel_mg"])
        fig, axs = plt.subplots(2, 3, figsize=(12, 6.5), sharex=True)
        for j, k in enumerate("xyz"):
            for row, Y, lab in ((0, bg, "자이로 mdeg/s"), (1, ba, "가속도 mg")):
                ax_ = axs[row, j]
                for m, c, nm in ((~cam & (dT > 0.002), "#d33", "데워지는 중"), (~cam & (dT < -0.002), "#2a6fdb", "식는 중"),
                                 (~cam & (abs(dT) <= 0.002), "#888", "거의 일정"), (cam, "#e0b000", "카메라 켬")):
                    ax_.plot(Tb[m], Y[m, j], "o", ms=3, color=c, label=nm)
                ax_.set_title(f"{lab} {k}"); ax_.grid(alpha=.3)
                if row == 1:
                    ax_.set_xlabel(f"{src} 온도 °C")
        axs[0, 0].legend(fontsize=7)
        p_th = png(fig)
        tt = np.array(th["t"]) / 60
        fig, axs = plt.subplots(3, 1, figsize=(11, 7), sharex=True)
        for j, k in enumerate("xyz"):
            axs[0].plot(tt, bg[:, j] - np.nanmean(bg[~cam, j]), ".-", ms=3, color=C[k], label=f"자이로 {k}")
            axs[1].plot(tt, ba[:, j] - np.nanmean(ba[~cam, j]), ".-", ms=3, color=C[k], label=f"가속도 {k}")
        TT = res["temperature"]
        for k2, c in (("battery", "#d33"), ("AP", "#555"), ("SKIN", "#1a9e5a"), ("BAT", "#e07b00")):
            if TT and k2 in TT:
                v = np.array([np.nan if x is None else x for x in TT[k2]], float)
                axs[2].plot(np.array(TT["t"]) / 60, v, color=c, label=k2)
        for a_, b_ in res.get("camera_intervals_s", []):
            for ax_ in axs:
                ax_.axvspan(a_ / 60, b_ / 60, color="#e0b000", alpha=.2)
        axs[0].set_ylabel("자이로 − 평균 (mdeg/s)"); axs[1].set_ylabel("가속도 − 평균 (mg)"); axs[2].set_ylabel("°C"); axs[2].set_xlabel("분")
        for ax_ in axs:
            ax_.grid(alpha=.3); ax_.legend(fontsize=7, ncol=4)
        p_ts = png(fig)

    an = res.get("android_cal") or {}
    p_an = None
    if an:
        fig, axs = plt.subplots(2, 1, figsize=(11, 5.5), sharex=True)
        for ax_, nm in ((axs[0], "gyro"), (axs[1], "accel")):
            if nm not in an:
                continue
            tt = np.array(an[nm]["series_t"]) / 60; v = np.array(an[nm]["series"])
            for j, k in enumerate("xyz"):
                ax_.step(tt, v[:, j], where="post", color=C[k], label=k)
            ax_.set_ylabel(("Android 자이로 바이어스 추정 " if nm == "gyro" else "Android 가속도 바이어스 추정 ") + an[nm]["unit"])
            ax_.grid(alpha=.3); ax_.legend(fontsize=7)
            for a_, b_ in res.get("camera_intervals_s", []):
                ax_.axvspan(a_ / 60, b_ / 60, color="#e0b000", alpha=.2)
        axs[1].set_xlabel("분")
        p_an = png(fig)

    # ---------------- numbers
    N = [g[k]["ARW_deg_per_sqrt_h"] for k in "xyz"]
    BI = [g[k]["BI_deg_s"] * 1e3 for k in "xyz"]
    d2, d5 = res["start_estimate_error"]["2s"], res["start_estimate_error"]["5s"]

    def all_rms(d, L):
        s = d["all_starts"].get(str(L))
        return None if not s else float(np.linalg.norm(s["rms_per_axis_mdeg_s"]))

    def p95(d, L):
        s = d["all_starts"].get(str(L))
        return None if not s else s["p95_norm_mdeg_s"]
    floor2 = float(np.linalg.norm(res["mean_noise_floor_mdeg_s"]["2s"]))
    floor5 = float(np.linalg.norm(res["mean_noise_floor_mdeg_s"]["5s"]))
    tname, tc = best_temp(res)
    o = res["orientation"]
    earth = o.get("earth_rate_device_deg_s")
    mr = res["mean_gyro_raw_mdeg_s"]
    mc = res["mean_gyro_cal_mdeg_s"]

    temp_concl = "온도 기록 없음."
    if tc:
        gs = [tc.get(f"gyro_raw_mdeg_s_{x}", {}) for x in "xyz"]
        as_ = [tc.get(f"accel_raw_mg_{x}", {}) for x in "xyz"]
        temp_concl = (f"온도 폭 {f(tc['range_C'], 1)} °C ({html.escape(str(tname))} 기준, 가장 잘 따라가는 온도원). 자이로 기울기 "
                      + ", ".join(f(g_.get('slope_per_C'), 1) for g_ in gs) + " mdeg/s/°C (r " + ", ".join(f(g_.get('r')) for g_ in gs) + "), 가속도 "
                      + ", ".join(f(a_.get('slope_per_C'), 2) for a_ in as_) + " mg/°C (r " + ", ".join(f(a_.get('r')) for a_ in as_) + ")."
                      + (" 폭이 2 °C 미만이라 기울기는 믿기 어렵습니다." if tc['range_C'] < 2 else "")
                      + " 즉 비행 중 폰이 몇 °C 데워지면 그만큼 바이어스가 움직이므로, 긴 비행에서는 온도 보상이나 비행 중 재추정이 필요할 수 있습니다(추정).")
    concl = f"""<ul>
<li><b>시작 때 잡은 자이로 바이어스는 몇 분 동안 거의 그대로입니다.</b> 처음 2 s 평균으로 잡으면 1 / 3 / 5 분 뒤 차이(세 축 합친 크기, 모든 시작점 RMS)는
 {f(all_rms(d2, 60), 1)} / {f(all_rms(d2, 180), 1)} / {f(all_rms(d2, 300), 1)} mdeg/s, 5 s 평균이면 {f(all_rms(d5, 60), 1)} / {f(all_rms(d5, 180), 1)} / {f(all_rms(d5, 300), 1)} mdeg/s.
 이 차이의 대부분은 시작 평균 자체의 잡음(2 s 평균 ≈ {f(floor2, 1)}, 5 s ≈ {f(floor5, 1)} mdeg/s, ARW 로 계산)입니다.</li>
<li>잡음(ARW) {f(min(N))}–{f(max(N))} °/√h, 바이어스 불안정성 {f(min(BI), 1)}–{f(max(BI), 1)} mdeg/s (τ ≈ {f(min(g[k]['tau_B'] for k in 'xyz'), 0)}–{f(max(g[k]['tau_B'] for k in 'xyz'), 0)} s).</li>
<li>바이어스 자체는 큽니다: raw(uncalibrated) 평균 {", ".join(f(x, 0) for x in mr)} mdeg/s (x, y, z). 그래서 <b>매 비행 시작 때 정지 상태에서 다시 잡는 것은 꼭 필요</b>하고, 한 번 잡은 값은 수 분 동안 쓸 만합니다(정지·실온 조건, 추정).</li>
<li>온도: {temp_concl}</li>
</ul>"""

    gyro_rows = []
    for k in "xyz":
        p = g[k]
        gyro_rows.append([k, f(p["ARW_deg_per_sqrt_h"], 3), f(p["ARW_deg_s_per_sqrtHz"] * 1e3, 2), f(p["BI_deg_s"] * 1e3, 2),
                          f(p["BI_deg_h"], 1), f(p["tau_B"], 0), "–" if p["K"] is None else f(p["K"] * 1e3, 3)])
    acc_rows = []
    for k in "xyz":
        p = a[k]
        acc_rows.append([k, f(p["VRW_m_s_per_sqrt_h"], 4), f(p["VRW_ug_per_sqrtHz"], 0), f(p["BI_mg"], 3), f(p["BI_m_s2"] * 1e3, 2), f(p["tau_B"], 0)])

    def lit(d, L):
        v = d["literal_mdeg_s"].get(str(L))
        return "–" if v is None else ", ".join(f(x, 1) for x in v) + f" (|{f(float(np.linalg.norm(v)), 1)}|)"
    drift_rows = []
    for L, lab in ((60, "1 분"), (180, "3 분"), (300, "5 분")):
        drift_rows.append([lab, lit(d2, L), f(all_rms(d2, L), 1), f(p95(d2, L), 1), lit(d5, L), f(all_rms(d5, L), 1), f(p95(d5, L), 1)])

    temp_html = "<p>온도 기록 없음.</p>"
    if th:
        rr = []
        for k, v in th["per_source"].items():
            def cell(key):
                it = v.get(key) or {}
                sl = it.get("slope_per_C"); r = it.get("r")
                return f"{f(sl, 2)} (r {f(r)})"
            rr.append([k + (" ★" if k == th.get("best_source") else ""), f(v["min"], 1), f(v["max"], 1), f(v["range_C"], 1)]
                      + [cell(f"gyro_raw_mdeg_s_{x}") for x in "xyz"] + [cell(f"accel_raw_mg_{x}") for x in "xyz"])
        temp_html = table(["온도원", "최저 °C", "최고 °C", "폭 °C", "자이로 x mdeg/s/°C", "y", "z", "가속도 x mg/°C", "y", "z"], rr)
        bs = th["per_source"].get(th.get("best_source")) or {}
        hy = []
        for lab, unit in (("gyro_raw_mdeg_s", "mdeg/s"), ("accel_raw_mg", "mg")):
            for x in "xyz":
                it = bs.get(f"{lab}_{x}") or {}
                hy.append([("자이로 " if lab.startswith("gyro") else "가속도 ") + x,
                           f((it.get("heating") or {}).get("slope_per_C"), 2), f((it.get("cooling") or {}).get("slope_per_C"), 2),
                           f(it.get("hysteresis_cooling_minus_heating"), 2) + " " + unit, f(it.get("resid_std"), 2)])
        temp_html += f"<h3>데워질 때 / 식을 때 (온도원 {html.escape(str(th.get('best_source')))})</h3>" + table(
            ["축", "데워질 때 기울기 /°C", "식을 때 기울기 /°C", "히스테리시스 (식을 때 − 데워질 때, 같은 온도)", "직선 맞춘 뒤 남는 흔들림 std"], hy)
        nz = th.get("noise_cam_on_vs_off_mdeg_s") or {}
        temp_html += ("<p>카메라(240 fps) 켰을 때 자이로 1 s 흔들림(std, x/y/z): " + (", ".join(f(x, 0) for x in nz["on"]) if nz.get("on") else "–")
                      + " mdeg/s, 껐을 때: " + ", ".join(f(x, 0) for x in nz.get("off", [])) + " mdeg/s. 폰에 팬은 없으므로 차이는 OIS·카메라 모듈 진동이나 전력·전기적 영향으로 추정."
                      + " 카메라 켠 구간은 온도 기울기 계산에서 뺐습니다.</p>")
        if p_ts:
            temp_html = f'<img src="{p_ts}" alt="time series"><img src="{p_th}" alt="bias vs temperature">' + temp_html
    elif res["temp_correlation"]:
        rr = []
        for k, v in res["temp_correlation"].items():
            rr.append([k, f(v["min"], 1), f(v["max"], 1), f(v["range_C"], 1)]
                      + [f"{f(v[f'gyro_raw_mdeg_s_{x}']['slope_per_C'], 1)} (r {f(v[f'gyro_raw_mdeg_s_{x}']['r'])})" for x in "xyz"]
                      + [f"{f(v[f'accel_raw_mg_{x}']['slope_per_C'], 2)} (r {f(v[f'accel_raw_mg_{x}']['r'])})" for x in "xyz"])
        temp_html = table(["온도원", "최저 °C", "최고 °C", "폭 °C", "자이로 x mdeg/s/°C", "y", "z", "가속도 x mg/°C", "y", "z"], rr)

    earth_html = ""
    if earth:
        earth_html = table(["축", "raw 평균 mdeg/s", "지구 자전 성분 mdeg/s", "raw − 지구 자전", "Android 보정 후(cal) 평균"],
                           [[k, f(mr[j], 1), f(earth[j] * 1e3, 2), f(mr[j] - earth[j] * 1e3, 1), f(mc[j], 1)] for j, k in enumerate("xyz")])

    android_html = ""
    if an:
        ag = an.get("gyro") or {}; aa = an.get("accel") or {}
        steps = sorted(ag.get("steps", []), key=lambda x: -x["jump_norm"])[:12]
        srows = [[f(st["t"], 1), f(st["t"] / 60, 1), ", ".join(f(x, 1) for x in st["before"]), ", ".join(f(x, 1) for x in st["after"]),
                  f(st["jump_norm"], 1), "예" if st["during_camera"] else ""] for st in sorted(steps, key=lambda x: x["t"])]
        tcal = res.get("thermal_cal") or {}
        bsrc = (res.get("thermal") or {}).get("best_source")
        cal_rows = []
        if tcal and bsrc in tcal.get("per_source", {}):
            v = tcal["per_source"][bsrc]; vr = res["thermal"]["per_source"][bsrc]
            for lab, nm in (("gyro_raw_mdeg_s", "자이로 mdeg/s/°C"), ("accel_raw_mg", "가속도 mg/°C")):
                for x in "xyz":
                    r_ = vr.get(f"{lab}_{x}") or {}; c_ = v.get(f"{lab}_{x}") or {}
                    cal_rows.append([f"{nm} {x}", f"{f(r_.get('slope_per_C'), 2)} (r {f(r_.get('r'))})", f"{f(c_.get('slope_per_C'), 2)} (r {f(c_.get('r'))})"])
        gc = A["gyro_cal"]
        allan_cmp = [[k, f(g[k]["ARW_deg_per_sqrt_h"], 3), f(gc[k]["ARW_deg_per_sqrt_h"], 3), f(g[k]["BI_deg_s"] * 1e3, 2), f(gc[k]["BI_deg_s"] * 1e3, 2)] for k in "xyz"]
        rng = ag.get("range") or [0, 0, 0]
        android_html = f"""<h2>5. Android 자동 보정(calibrated) vs 센서 그대로(uncalibrated)</h2>
<p>Android 의 자이로(type 4)·가속도(type 1)는 센서 값(type 16·35)에서 Android 가 스스로 추정한 바이어스를 뺀 값입니다. 그 추정값(type 16·35 의 v3..v5 = raw − calibrated)이 시간에 따라 어떻게 바뀌는지:</p>
<img src="{p_an}" alt="android bias estimate">
<p>자이로 추정값: 처음 ({", ".join(f(x, 0) for x in ag.get("first", []))}) → 끝 ({", ".join(f(x, 0) for x in ag.get("last", []))}) mdeg/s, 바뀐 횟수 {ag.get("n_changes", "–")}, 범위 {", ".join(f(x, 0) for x in rng)} mdeg/s.
가속도 추정값: 처음 ({", ".join(f(x, 1) for x in aa.get("first", []))}) → 끝 ({", ".join(f(x, 1) for x in aa.get("last", []))}) mg, 바뀐 횟수 {aa.get("n_changes", "–")}.
(type 4 − type 16 이 −추정값과 같은지 검사: 최대 차이 {f(an.get("check_type4_minus_type16_vs_minus_estimate_max_mdeg_s"), 3)} mdeg/s.)</p>
<h3>자이로 추정값의 큰 변화 (크기순 최대 12개, 시간순)</h3>
{table(["t s", "분", "전 (x, y, z) mdeg/s", "후", "변화 크기", "카메라 켠 중"], srows) if srows else "<p>변화 없음.</p>"}
<h3>Allan: 센서 그대로 vs Android 보정 후 (가열 전 정지 구간)</h3>
{table(["축", "ARW raw °/√h", "ARW cal", "BI raw mdeg/s", "BI cal"], allan_cmp)}
<h3>온도 기울기: 센서 그대로 vs Android 보정 후 ({html.escape(str(bsrc))})</h3>
{table(["", "raw (uncalibrated)", "calibrated"], cal_rows) if cal_rows else "<p>온도 자료 부족.</p>"}
<p><b>해석:</b> Android 추정값은 계단식으로 바뀝니다(바꾸는 순간 calibrated 자이로에 그 크기만큼의 계단이 생김). 그 크기가 수십~100 mdeg/s 이면,
녹화마다 calibrated 자이로 바이어스가 ~0.1 °/s 씩 다르게 보이는 것과 녹화 중 갑자기 바뀌는 것의 상당 부분은 센서가 아니라 Android 보정 때문일 수 있습니다(추정, 위 표의 실제 크기로 판단).
VIO·자체 필터에는 <b>uncalibrated(type 16)</b> 를 쓰고 바이어스는 직접 추정하는 편이 일관됩니다(제안).</p>"""

    hs_rows = []
    for r in hs:
        segs = r["segments"]
        first = segs[0]["gyro_mean_mdeg_s"] if segs else None
        last = segs[-1]["gyro_mean_mdeg_s"] if segs else None
        q = r.get("quietest_0p5s_window") or {}
        hs_rows.append([r["name"], f(r["duration_s"], 0), r.get("battery_c_end") or "–", len(segs),
                        f(q.get("gyro_std_p1_dps", float("nan")) * 1e3, 0), f(q.get("gyro_mean_norm_p1_dps", float("nan")) * 1e3, 0),
                        "–" if not segs else f"{segs[0]['t0']:.0f} s: " + ", ".join(f(x, 0) for x in first),
                        "–" if len(segs) < 2 else f"{segs[-1]['t0']:.0f} s: " + ", ".join(f(x, 0) for x in last),
                        "–" if "first_vs_last_mdeg_s" not in r else ", ".join(f(x, 0) for x in r["first_vs_last_mdeg_s"]),
                        "–" if "trend_mdeg_s_per_min" not in r else ", ".join(f(x, 1) for x in r["trend_mdeg_s_per_min"])])

    cov = res["coverage"].get("4") or res["coverage"].get(4)
    cuts = res["cuts"]
    body = f"""
<h1>폰 자이로 바이어스, 시작 때 잡으면 그대로인가</h1>
<p class=mut>Galaxy S24 (LSM6DSV), 책상 위 정지, 카메라 끔, IMU 만 기록 · {html.escape(res['recording'])} · {res['duration_s'] / 60:.1f} 분 ·
자르기: {html.escape(res['cuts_source'])} {html.escape(str(cuts))} {'<b>(잠정 결과 — 자를 구간 확인 전)</b>' if 'auto' in res['cuts_source'] else ''}</p>
<h2>결론</h2>{concl}

<h2>1. 시작 때 잡은 바이어스가 얼마나 틀어지나 (데이터에서 직접)</h2>
<p>“처음 2 s(또는 5 s) 평균 = 바이어스”로 잡은 뒤, 1·3·5 분 뒤의 10 s 평균과 비교. 단위 mdeg/s (= 0.001 °/s). “녹화 시작”은 맨 처음 한 번,
“모든 시작점”은 5 s 마다 시작점을 옮겨 가며 잰 것(세 축 합친 크기의 RMS / 95 %).</p>
{table(["경과", "2 s 추정: 녹화 시작 (x, y, z)", "2 s: 모든 시작점 RMS", "2 s: 95 %", "5 s 추정: 녹화 시작", "5 s: RMS", "5 s: 95 %"], drift_rows)}
<p class=mut>참고: 시작 평균 자체의 잡음(ARW/√T, 세 축 합) 2 s ≈ {f(floor2, 1)}, 5 s ≈ {f(floor5, 1)} mdeg/s.
5 mdeg/s 바이어스 오차를 5 분 적분하면 각도 오차 ≈ 1.5° (추정, 다른 보정 없을 때).</p>

<h2>2. Allan 분산</h2>
<img src="{p_allan}" alt="Allan deviation">
<h3>자이로 (uncalibrated = Android 바이어스 보정 전)</h3>
{table(["축", "ARW °/√h", "ARW mdeg/s/√Hz", "바이어스 불안정성 mdeg/s", "°/h", "그때 τ s", "RRW mdeg/s/√s"], gyro_rows)}
<h3>가속도</h3>
{table(["축", "VRW m/s/√h", "µg/√Hz", "바이어스 불안정성 mg", "mm/s²", "τ s"], acc_rows)}
<p class=mut>바이어스 불안정성 = Allan 최솟값 / 0.664. RRW 는 τ 가 큰 쪽에서 +½ 기울기가 보일 때만(“–” = 이 길이에서 안 보임).
녹화가 {res['duration_s'] / 60:.0f} 분이라 τ 가 수백 s 인 점은 표본이 적어 흔들립니다.</p>

<h2>3. 온도와 바이어스 (자이로·가속도)</h2>
{temp_html}
<h3>가열 전 정지 구간의 10 s 평균</h3><img src="{p_blk}" alt="block means">
<p class=mut>기울기 = 카메라 끈 30 s 블록 평균(지구 자전 뺀 raw 자이로, raw 가속도)을 온도에 직선으로 맞춘 값. 온도원: battery = dumpsys battery, AP/BAT/CP/PA/SKIN/USB = thermal HAL (dumpsys thermalservice). IMU 칩 자체 온도 센서는 Android 에 안 보임
(센서 목록에 temperature 형 없음). 폭이 2 °C 미만이면 기울기는 잡음과 구분이 어렵습니다. 기울기가 가장 뚜렷한 온도원: {html.escape(str(tname))}.
가열은 폰을 건드리지 않고 240 fps 녹화를 몇 분씩 켰다 껐다 하며 천천히 했고(배터리 42 °C 에서 멈추는 조건), 끝난 뒤 식는 동안도 계속 기록했습니다. 가열용 영상은 폰에서 지웠습니다.
/sys 열 존은 권한이 없어 못 읽었습니다.</p>

<h2>4. 폰 자세와 지구 자전</h2>
<p>사용자 말대로 폰은 <b>화면이 책상 쪽(아래)</b>으로 엎어져 있었고, 가속도 평균 z = {f(o.get('accel_mean_m_s2', [0, 0, 0])[2], 2)} m/s² (음수 → 폰의 +z 축(화면 밖 방향)이 아래, 정지 가속도계는 위쪽을 +g 로 읽음) 로 일치합니다.
가속도 평균으로 기울기, 자력계(+ rotation vector)로 방위. 화면 {o.get('screen_faces', '?')}, 수평에서 {f(o.get('tilt_deg_from_flat'), 1)}° 기울어짐,
위쪽 모서리 방위 {f(o.get('heading_top_edge_deg_from_N'), 0)}° (북=0, 추정). |B| = {f(o.get('mag_norm_uT'), 0)} µT (한국 지구자기 ≈ 50 µT → 실내 자기 교란 있음,
방위 오차 수~수십 ° 가능). 위도 {o.get('lat_assumed_deg')}° 가정, 지구 자전 15.04 °/h = 4.18 mdeg/s.</p>
{earth_html}
<p class=mut>지구 자전은 일정한 값이라 “바이어스가 그대로인가”에는 영향이 없고, 바이어스 절댓값만 몇 mdeg/s 바꿉니다. 기울기는 가속도 바이어스 ~0.02 m/s² → ~0.1° 오차라 무시 가능.
방위 10° 틀리면 지구 자전 성분 오차 ≈ {f((o.get('earth_rate_err_10deg_heading_deg_s') or 0) * 1e3, 2)} mdeg/s — 측정 바이어스(수십~백 mdeg/s)에 비해 무시 가능.</p>

{android_html}

<h2>6. 기존 녹화(hs_*, 240 fps 촬영 중)의 정지 구간</h2>
<p>각 녹화에서 1 s 이상 정지(자이로 흔들림 &lt; {f(res['hs_still_threshold_gyro_std_dps'] * 1e3, 0)} mdeg/s std, 가속도 std &lt; 0.06 m/s²)한 구간의 자이로 평균(calibrated, mdeg/s).
이 녹화들은 Android 보정 후(calibrated) 자이로만 있어서, Android 가 중간에 바이어스 추정을 바꾸면 그 변화도 섞입니다. 녹화 중 온도 기록은 없고 끝 배터리 온도만 있습니다.</p>
<img src="{p_hs}" alt="hs segments">
<p><b>결과: 이 녹화들은 손에 들고(또는 보드를 움직이며) 찍은 것이라 바이어스를 잴 만큼 가만히 있던 구간이 거의 없습니다.</b>
가장 조용한 0.5 s 창(하위 1 %)조차 자이로 흔들림이 수백~수천 mdeg/s, 평균 회전이 100 mdeg/s 이상이라 수 mdeg/s 크기의 바이어스 변화는 이 자료로는 볼 수 없습니다.
녹화 안에서 바이어스 변화를 보려면 녹화 시작·끝에 폰을 5–10 s 책상에 내려놓는 습관이 필요합니다(제안).</p>
{table(["녹화", "길이 s", "끝 배터리 °C", "정지 구간 수", "가장 조용한 창 자이로 std mdeg/s", "그 창 평균 |ω| mdeg/s", "첫 구간 (x, y, z)", "마지막 구간", "마지막 − 첫", "추세 mdeg/s/분"], hs_rows)}

<h2>7. 주의</h2>
<ul class=mut>
<li>처음 10 분은 화면이 꺼진(도즈) 상태라 자이로 표본이 주기적으로 끊겼고, 그 뒤에는 화면 꺼짐 시간을 늘리고 깨워 두었습니다(끝나고 원래 값으로 되돌림). 끊김(약 1 s 단위, 전체에서 끊긴 시간 {f((cov or {}).get('time_in_gaps_frac', 0) * 100, 0)} %). 가속도는 끊김 없음.
Allan 은 끊긴 칸을 빼고 평균하는 방식으로 계산 → 짧은 τ(&lt;1 s) 값은 덜 정확, 바이어스 비교에는 영향 작음 (추정).</li>
<li>기록 방법: 앱 설치 없이 adb shell 사용자로 <code>app_process</code> 에서 SensorManager 를 직접 등록(tools/imu_shell). 카메라 안 씀.</li>
<li>정지·실온 한 번의 측정입니다. 비행 진동·온도 상승이 있는 상황은 다를 수 있습니다(추정).</li>
</ul>"""
    page = f"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>자이로 바이어스 안정성</title><style>
:root{{--bg:#fff;--fg:#1a1a1a;--mut:#666;--line:#ddd;--th:#f3f3f3}}
@media (prefers-color-scheme:dark){{:root:not([data-theme=light]){{--bg:#15171a;--fg:#e8e8e8;--mut:#9a9a9a;--line:#333;--th:#22252a}}}}
body{{background:var(--bg);color:var(--fg);font:15px/1.6 system-ui,sans-serif;margin:0 auto;max-width:1000px;padding:12px 16px}}
.tw{{overflow-x:auto}} table{{border-collapse:collapse;margin:8px 0;font-size:14px}} td,th{{border:1px solid var(--line);padding:3px 8px;text-align:right}}
th{{background:var(--th)}} img{{max-width:100%;background:#fff;border-radius:4px}} .mut{{color:var(--mut)}} code{{font-size:13px}}
</style></head><body>{body}</body></html>"""
    path.write_text(page, encoding="utf-8")
