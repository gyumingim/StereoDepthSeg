"""HTML report for temp_history_bias.py (reads runs/temp_history_bias.json + the 1 s cache) -> docs/temp_history_bias.html"""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams["svg.fonttype"] = "none"

sys.path.insert(0, str(Path(__file__).resolve().parent))
import temp_history_bias as TH  # noqa: E402
import analyze_static as AS     # noqa: E402

ROOT = TH.ROOT
R = json.loads((ROOT / "runs/temp_history_bias.json").read_text())
D = TH.load(ROOT / "recordings" / R["recording"])
B = TH.Blocks(D, 10)
raw, F, dT = TH.build_features(D, B)
ok = B.ok
S10 = R["blk10"]["sensors"]
MODELS = ["M0", "M1", "M2", "M3", "M4", "M5", "M5f"]
MNAME = {"M0": "상수(온도 무시)", "M1": "현재 온도 1차", "M2": "저역통과/시간이동 온도", "M3": "온도 + 온도변화율",
         "M4": "온도 2차식", "M5": "두 센서 합", "M5f": "두 센서 각각 저역통과"}

# ---------------------------------------------------------------- extra checks (camera on/off confound, lead at finer grid)
extra = {}
for j, (s, ax) in enumerate(TH.SENS):
    y = B.Y[:, j]
    best_src = S10[f"{s}_{ax}"]["chrono"]["M1"]["p"]["src"]
    x = F[best_src][(0, 0)]
    X1 = np.c_[np.ones(ok.sum()), x[ok]]
    X2 = np.c_[X1, B.cam[ok].astype(float)]
    b1 = TH.ols(X1, y[ok]); b2 = TH.ols(X2, y[ok])
    extra[f"{s}_{ax}"] = {"src": best_src, "rms_M1_all": TH.rms(y[ok] - X1 @ b1), "rms_M1_plus_cam_all": TH.rms(y[ok] - X2 @ b2),
                          "cam_step": float(b2[2])}
R["extra_camera_flag_check"] = extra
(ROOT / "runs/temp_history_bias.json").write_text(json.dumps(R, indent=1, default=str))

COL = {"room": "#9aa0a6", "heat": "#d9480f", "cool": "#1c7ed6"}


def grp(mask_extra=None):
    g = {"room": ok & (B.phase == "room"), "heat": ok & B.heating, "cool": ok & B.cooling}
    return g


def svg(fig):
    import io
    b = io.StringIO()
    fig.savefig(b, format="svg", bbox_inches="tight")
    plt.close(fig)
    s = b.getvalue()
    return s[s.find("<svg"):]


def fig_timeseries():
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    t1 = np.arange(B.n1) + 0.5
    for s in TH.SRCS:
        axs[0].plot(t1, raw[s], lw=1, label=s)
    axs[0].set_ylabel("온도 °C"); axs[0].legend(ncol=7, fontsize=7)
    for j, nm in ((1, "gyro_y"), (2, "gyro_z")):
        jj = [f"{a}_{b}" for a, b in TH.SENS].index(nm)
        y = B.Y[:, jj]
        a = axs[j]
        a.plot(B.t[ok], y[ok], ".", ms=3, color="k", label="측정(10 s 평균)")
        for mdl, c in (("M1", "#e8590c"), ("M2", "#2b8a3e")):
            p = S10[nm]["chrono"][mdl]["p"]; bb = np.array(S10[nm]["chrono"][mdl]["beta"])
            X = TH.design(mdl, F, dT, p)
            lab = f"{mdl} {p['src']}" + (f" τ={p['tau']} 이동={p['lag']:+d}s" if mdl == "M2" else "")
            a.plot(B.t, X @ bb, color=c, lw=1.2, label=lab)
        a.axvline(B.t_mid, color="gray", ls="--", lw=0.8)
        a.set_ylabel(nm + " mdeg/s"); a.legend(fontsize=7)
    for a in axs:
        for c0, c1 in D["cams"]:
            a.axvspan(c0, c1, color="#ffd8a8", alpha=0.5, lw=0)
    axs[-1].set_xlabel("시간 s (주황 = 240fps 카메라 켬, 점선 = 학습/검증 경계)")
    return svg(fig)


def fig_hyst(nm, src, tau, lag, unit):
    jj = [f"{a}_{b}" for a, b in TH.SENS].index(nm)
    y = B.Y[:, jj]
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.6), sharey=True)
    for a, (tt, LL, title) in zip(axs, ((0, 0, f"현재 온도 {src}"), (tau, lag, f"{src} τ={tau}s 이동 {lag:+d}s"))):
        x = F[src][(tt, LL)]
        for k, m in grp().items():
            a.plot(x[m], y[m], "o", ms=3, color=COL[k], label={"room": "실온", "heat": "가열", "cool": "냉각"}[k], alpha=0.8)
        hb = TH.hysteresis(y, x, B, ok)
        xx = np.linspace(np.nanmin(x[ok]), np.nanmax(x[ok]), 10)
        bb = TH.ols(np.c_[np.ones(ok.sum()), x[ok]], y[ok])
        a.plot(xx, bb[0] + bb[1] * xx, "k-", lw=0.8)
        a.set_title(f"{title}\n잔차 RMS {hb['rms_all']:.2f}, 냉각−가열 {hb['cool_minus_heat']:+.2f} {unit}", fontsize=9)
        a.set_xlabel("°C")
    axs[0].set_ylabel(f"{nm} {unit}"); axs[0].legend(fontsize=8)
    return svg(fig)


def fig_tau(nm, unit):
    a_ = S10[nm]["allfit_M2"]
    fig, ax = plt.subplots(figsize=(5, 3.2))
    for s in TH.SRCS:
        if s == "USB":
            continue
        ax.plot(TH.TAUS, a_[s]["tau_profile_lag0"], "-o", ms=2.5, lw=1, label=s)
    ax.set_xlabel("저역통과 시간상수 τ (s), 시간이동 0"); ax.set_ylabel(f"전체 잔차 RMS {unit}")
    ax.set_title(nm, fontsize=9); ax.legend(fontsize=7, ncol=2)
    return svg(fig)


def f2(x):
    return f"{x:.2f}"


def table_models():
    rows = []
    for nm, s10 in S10.items():
        unit = "mdeg/s" if nm.startswith("gyro") else "mg"
        lo = s10["loso"]; ch = s10["chrono"]
        bestL = min((m for m in MODELS if m != "M0"), key=lambda m: lo[m]["pooled_test_rms"])
        cells = []
        for m in MODELS:
            v = lo[m]["pooled_test_rms"]
            st = ' style="font-weight:700;background:var(--hi)"' if m == bestL else ""
            cells.append(f"<td{st}>{f2(v)}<br><span class=mut>{f2(ch[m]['test_rms'])}</span></td>")
        rows.append(f"<tr><th>{nm}<br><span class=mut>{unit}</span></th>{''.join(cells)}<td>{R['blk10']['noise_block_white_est'][nm]:.2f}</td></tr>")
    head = "".join(f"<th>{m}<br><span class=mut>{MNAME[m]}</span></th>" for m in MODELS)
    return f"<table><tr><th>축</th>{head}<th>잡음 바닥(추정)</th></tr>{''.join(rows)}</table>"


def table_src():
    rows = []
    for nm, s10 in S10.items():
        unit = "mdeg/s" if nm.startswith("gyro") else "mg"
        for s in TH.SRCS:
            a = s10["allfit_M2"][s]
            rows.append(f"<tr><td>{nm}</td><td>{s}</td><td>{a['slope_per_C']:.2f}</td><td>{f2(a['rms_M1'])}</td><td>{a['tau']}</td><td>{a['lag']:+d}</td>"
                        f"<td>{f2(a['rms'])}</td><td>{a['hyst_M1']['cool_minus_heat']:+.2f}</td><td>{a['hyst_M2']['cool_minus_heat']:+.2f}</td></tr>")
    return ("<table><tr><th>축</th><th>온도원</th><th>기울기 /°C</th><th>M1 RMS</th><th>최적 τ s</th><th>최적 이동 s</th><th>M2 RMS</th>"
            "<th>냉각−가열 (M1)</th><th>냉각−가열 (M2)</th></tr>" + "".join(rows) + "</table>")


def table_extra():
    rows = "".join(f"<tr><td>{k}</td><td>{v['src']}</td><td>{f2(v['rms_M1_all'])}</td><td>{f2(v['rms_M1_plus_cam_all'])}</td><td>{v['cam_step']:+.2f}</td></tr>"
                   for k, v in extra.items())
    return "<table><tr><th>축</th><th>온도원</th><th>M1 RMS</th><th>M1+카메라 켬 표시</th><th>카메라 켬 계단</th></tr>" + rows + "</table>"


plots = {
    "ts": fig_timeseries(),
    "hy_gy": fig_hyst("gyro_y", "PA", 10, 10, "mdeg/s"),
    "hy_gz": fig_hyst("gyro_z", "CP", 0, 10, "mdeg/s"),
    "hy_gx_bat": fig_hyst("gyro_x", "battery", 0, 40, "mdeg/s"),
    "hy_ax": fig_hyst("accel_x", "AP", 480, 40, "mg"),
    "hy_ay": fig_hyst("accel_y", "BAT", 5, 0, "mg"),
    "tau_gy": fig_tau("gyro_y", "mdeg/s"),
    "tau_ax": fig_tau("accel_x", "mg"),
}

css = """
:root{--bg:#fff;--fg:#1f2328;--mut:#6a737d;--line:#d0d7de;--hi:#fff3bf;--card:#f6f8fa}
@media (prefers-color-scheme:dark){:root:not([data-theme=light]){--bg:#0d1117;--fg:#e6edf3;--mut:#8b949e;--line:#30363d;--hi:#3b2f00;--card:#161b22}
 svg{background:#fff;border-radius:6px}}
body{background:var(--bg);color:var(--fg);font:15px/1.6 system-ui,'Malgun Gothic',sans-serif;max-width:1050px;margin:0 auto;padding:16px}
table{border-collapse:collapse;font-size:13px;margin:8px 0;display:block;overflow-x:auto}
td,th{border:1px solid var(--line);padding:3px 7px;text-align:center}
.mut{color:var(--mut);font-size:12px;font-weight:400}
.box{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:10px 16px}
svg{max-width:100%;height:auto}
"""

html = f"""<!doctype html><html lang=ko><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>IMU 바이어스 온도 이력</title><style>{css}</style></head><body>
<h1>IMU 바이어스는 온도 '이력'을 따르나?</h1>
<p class=mut>녹화 {R['recording']} · 만든 시각 {time.strftime('%F %T')} · 도구 tools/imu_shell/temp_history_bias.py, temp_history_report.py · 원자료 runs/temp_history_bias.json</p>

<div class=box style="border:2px solid #e5484d">
<b>정정 (2026-10-20 15:20, 검증 반영 — docs/input_audit_verify.md 2절)</b>
<ul>
<li><b>"온도보다 약 10 s 먼저 움직인다"</b> → 근거 약함. 앞당김 격자가 10 s 간격이라 10 s 는 0 다음 칸이고, 2 s 격자로 다시 맞추면 0~8 s, 개선은 0.03~0.1 mdeg/s 뿐. thermal HAL 값이 약 10 s 마다만 갱신(평균 ~5 s 늦은 값)되므로 <b>HAL 갱신 지연으로 설명되며, 물리적 앞섬이라는 근거는 없다</b>.</li>
<li><b>카메라 켬 효과 "x +5.8, z −5.2"</b> → <b>불확실</b>. 축마다 다른 온도원(x CP, z AP)으로 맞춘 값이고, PA 로 맞추면 x +10.6, z +12.6 으로 z 부호가 바뀐다. 이 녹화로는 식별 불가.</li>
<li><b>LOSO 2.5~3.1 mdeg/s</b> → <b>낙관적</b>. 축마다 7개 모델 중 시험 점수가 가장 좋은 것을 고른 값. 미리 정한 단일 모델 M1(CP/PA 현재 온도 1차식)은 <b>3.4 / 3.6 / 6.1</b> mdeg/s. 독립 가열 사건이 1번뿐이라 일반화 근거 없음.</li>
<li><b>온도 기록 간격 "1 s"</b> → 사용자 결정은 <b>10 s</b> (HAL 이 ~10 s 마다만 갱신, root 없이 기록 — docs/todo_experiments.md). 아래 권장 1번의 "1 s" 는 이 결정으로 대체.</li>
<li>히스테리시스: 이 페이지의 −4~−11 mdeg/s 와 gyro_temp_table.html 의 0~1.7 은 분석 방법이 달라 맞지 않음 → <b>미결</b> (docs/critic_debate_20261020.html C2).</li>
</ul></div>

<div class=box>
<h2>결론 먼저</h2>
<ul>
<li><b>자이로: 긴 이력은 필요 없다.</b> 바이어스는 SoC 쪽 빠른 온도(<b>CP 또는 PA</b>)의 <b>현재 값</b>을 거의 그대로 따른다. 저역통과 τ를 0–600 s 훑어도 최적 τ는 0–20 s(=온도 샘플 간격 ~14 s 이하라 사실상 0). 대신 온도 값보다 <b>약 10 s 먼저</b> 움직인다(추정: 온도 HAL 값 갱신 지연 / 14 s 샘플 보간 탓).</li>
<li>느린 온도(배터리, BAT, SKIN, USB)를 쓰면 가열·냉각 고리(히스테리시스)가 크게 생긴다(자이로 냉각−가열 차 −15 ~ −54 mdeg/s). 이건 "칩이 과거 온도를 기억"해서가 아니라 <b>배터리가 칩보다 늦게 데워지고 늦게 식기</b> 때문이다. 그래서 배터리 온도에 저역통과를 걸면 오히려 나빠지고, 최적값은 '앞당기기' 40–60 s로 나온다.</li>
<li>현재 온도 1차식(M1, CP/PA) 대비 개선: 자이로 한 구간 빼기 검증(LOSO) 잔차 x 3.41→2.81~3.20, y 3.63→3.13~3.25, z 6.06→2.55(온도변화율 항 M3) mdeg/s. 온도 무시하면 27–42 mdeg/s. 10 s 평균의 잡음 바닥이 1.7–2.1 mdeg/s라 <b>남은 개선 여지는 작다</b>.</li>
<li><b>가속도는 반대로 '느린' 온도를 따른다.</b> x·y는 배터리/BAT 현재 온도 1차식이 가장 좋고(LOSO 0.14–0.25 mg), SoC 온도(AP/CP/PA)를 쓰려면 τ ≈ 1.5–8 분 저역통과가 필요하다 = 여기서는 진짜 '이력' 효과(추정: 보드/패키지 기계 응력이 천천히 따라옴). z는 AP + 온도변화율(M3)이 최선(LOSO 0.93→0.60 mg). 크기 자체가 1–2 mg 수준이라 실용 영향은 작다.</li>
<li>히스테리시스: 자이로는 CP/PA 현재 온도 + 10 s 앞당김으로 냉각−가열 차가 −4 ~ −11 → −1.6 ~ −5.5 mdeg/s로 줄지만 0은 아니다(z축 PA는 여전히 −5.5). 가속도 x는 AP τ480 s로 0.63 → 0.03 mg로 거의 사라진다.</li>
<li><b>카메라 켬 자체 효과(혼입 가능)</b>: M1에 '카메라 켬' 0/1 계단을 더하면 자이로 x +5.8, z −5.2 mdeg/s, 가속도 z +3.0 mg 계단이 잡힌다(5절). 가열 = 카메라 켬이라 '10 s 앞당김'과 이 계단이 같은 현상(전류/전원 변화가 온도계보다 먼저 보임)일 수 있다. 이 녹화로는 분리 불가(추정).</li>
<li><b>주의(과적합)</b>: 독립 가열 사건은 사실상 1번(카메라 가열 2회가 붙어 있음 + 냉각 1번). 녹화 1개, 온도 샘플 ~14 s. τ·이동 값은 이 녹화 한정 <b>추정</b>이다. 두 센서 조합(M5f)은 학습 구간에서 좋아도 LOSO에서 나빠지는 경우가 있다(자이로 z 1.86→7.73).</li>
</ul>
<h3>권장 (IMU 보정에 넣을 것)</h3>
<ol>
<li>앱이 <b>1 s마다 thermal HAL CP, PA, AP + 배터리 온도</b>를 기록(1 s면 '10 s 앞당김'이 실제인지 샘플 보간 탓인지도 가려진다).</li>
<li>자이로: 축마다 <b>b = a + k·T_CP(현재)</b> (y축은 PA도 동등). 필터 없음. 선택: z축에 온도변화율 항 c·dT/dt 추가(LOSO에서 가장 크게 개선, 단 근거 1 사이클).</li>
<li>가속도: x·y는 <b>b = a + k·T_battery(현재)</b>, z는 AP + dT/dt. 또는 SoC 온도 하나만 쓸 거면 τ≈300 s 저역통과.</li>
<li>2차식·두 센서 조합은 지금 데이터로는 근거 부족 → 넣지 않음. 남은 2–3 mdeg/s는 VIO 온라인 바이어스 추정에 맡긴다.</li>
<li>검증용 다음 녹화: 가열 속도가 다른 사건(예: CPU 부하로 천천히 가열, 충전 가열)을 하나 더 → 같은 계수로 맞는지 확인.</li>
</ol>
</div>

<h2>1. 시간에 따른 온도와 자이로</h2>
<p>위: 7개 온도원(14 s 샘플을 1 s 선형 보간). 아래: 자이로 y, z 10 s 평균(검은 점)과 학습 구간(점선 왼쪽)으로 맞춘 M1, M2 예측. 점선 오른쪽이 시간 분할 검증 구간.</p>
{plots['ts']}

<h2>2. 모델 비교 (검증 잔차 RMS)</h2>
<p>굵은 숫자 = 한 구간 빼기 검증(LOSO: 실온/가열1/냉각1/가열2/냉각2 앞/냉각2 뒤 6구간 중 하나를 빼고 맞춘 뒤 뺀 구간 예측, 합쳐 RMS). 작은 회색 = 시간 분할 검증(가열 + 냉각 앞 절반 학습 → 냉각 뒤 절반 예측; 이 구간은 온도 변화가 작아 차이가 작게 나온다). 온도원·τ·이동은 학습 구간만 보고 골랐다. 노란 칸 = LOSO 최선.</p>
{table_models()}
<p class=mut>M2 격자: τ = {TH.TAUS} s, 이동 −120…+120 s(10 s 간격, + = 센서가 온도원보다 먼저 반응). M5f τ 격자 0,15,30,60,120,240,480 s. 10 s 블록, 움직임 구간 {R['cuts']} (±2 s) 제외. 5 s 블록 결과도 JSON(blk5)에 있고 순위는 같다.</p>

<h2>3. 온도원별 최적 τ와 시간이동 (전체 자료로 맞춤, 참고용)</h2>
{table_src()}
<p class=mut>'냉각−가열' = 한 직선으로 맞춘 뒤 냉각 블록 평균 잔차 − 가열 블록 평균 잔차. 0에 가까울수록 히스테리시스 없음. USB는 이동 범위 끝(+120 s)에 걸려 의미 없음.</p>
<div style="display:flex;flex-wrap:wrap;gap:8px">{plots['tau_gy']}{plots['tau_ax']}</div>
<p>자이로 y: SoC 온도(CP/PA)는 τ=0 근처가 최소, τ를 키울수록 나빠짐 → 이력 없음. 가속도 x: SoC 온도는 τ 수백 초에서 최소 → 이력 있음, 배터리/BAT는 τ와 무관하게 이미 낮음(배터리 자체가 '느린 온도계').</p>

<h2>4. 히스테리시스: 바이어스 대 온도 (회색 실온, 빨강 가열, 파랑 냉각)</h2>
<h3>자이로 y — PA 현재 vs PA τ10 s +10 s</h3>{plots['hy_gy']}
<h3>자이로 z — CP 현재 vs CP +10 s</h3>{plots['hy_gz']}
<h3>자이로 x — 배터리 현재 vs 배터리 +40 s (느린 온도원은 고리가 큼)</h3>{plots['hy_gx_bat']}
<h3>가속도 x — AP 현재 vs AP τ480 s +40 s (가속도는 진짜 이력)</h3>{plots['hy_ax']}
<h3>가속도 y — BAT 현재</h3>{plots['hy_ay']}

<h2>5. 카메라 켬 자체 효과 확인</h2>
<p>가열 = 카메라 켬이라 "온도"와 "카메라 전류" 효과가 섞일 수 있다. M1에 카메라 켬 표시(0/1)를 더해 봤다(전체 자료).</p>
{table_extra()}
<p class=mut>카메라 계단이 크고 RMS가 많이 줄면 온도 말고 전기적 효과가 섞인 것. 작으면 온도로 설명됨.</p>

<h2>6. 한계</h2>
<ul>
<li>온도 샘플 ~{R['temp_sample_dt_s']['median']:.0f} s 간격(최대 {R['temp_sample_dt_s']['max']:.0f} s), 0.1 °C 단위. 선형 보간 후 1 s 격자에서 필터 → 10 s 블록 평균. 10 s 이하의 τ·이동은 해상도 밖(추정).</li>
<li>가열 사건 1회(카메라 2회), 녹화 1개 → 계수의 일반화는 미검증. LOSO의 '가열1 빼기'가 가장 어렵다(온도 범위 바깥 외삽).</li>
<li>자이로 uncal(type 16) v0..v2, 가속도 uncal(type 35) v0..v2. 지구 자전은 상수라 절편에 흡수. 화면 아래로 정지.</li>
</ul>
</body></html>"""
(ROOT / "docs/temp_history_bias.html").write_text(html)
TH.log("html written docs/temp_history_bias.html; camera-flag check: " + ", ".join(f"{k} {v['rms_M1_all']:.2f}->{v['rms_M1_plus_cam_all']:.2f} step {v['cam_step']:+.2f}" for k, v in extra.items()))
