"""runs/endstill_tmp/endstill_check.json -> docs/endstill_check.html (endstill_check.py 결과를 표로)."""
import json, os
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
D = json.load(open(os.path.join(ROOT, "runs/endstill_tmp/endstill_check.json")))
SB = np.array(D["settings"]["slopes_battery"]["gyro"])
SC = np.array(D["settings"]["slopes_cp"]["gyro"])
RECS = ["hs_20261020_223815", "hs_20261020_224323"]
f1 = lambda v: "—" if v is None or (isinstance(v, float) and np.isnan(v)) else f"{v:+.1f}" if abs(v) < 1e4 else f"{v:.0f}"
f0 = lambda v: f"{v:.1f}"


def row(cells, th=None, cls=""):
    h = f"<th>{th}</th>" if th is not None else ""
    return f"<tr{(' class=' + chr(34) + cls + chr(34)) if cls else ''}>{h}" + "".join(f"<td>{c}</td>" for c in cells) + "</tr>"


def table(head, rows):
    return ('<div class="tw"><table><thead><tr>' + "".join(f"<th>{h}</th>" for h in head) + "</tr></thead><tbody>"
            + "".join(rows) + "</tbody></table></div>")


css = """<style>
:root{--bg:#fff;--fg:#1c1d1f;--mut:#6b6f76;--panel:#f4f5f7;--line:#dfe1e5;--hl:#fff6d6;--bad:#b3261e;--ok:#1e6b34}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#151618;--fg:#e8e9eb;--mut:#9a9ea6;--panel:#1f2124;--line:#33363b;--hl:#3a3420;--bad:#f2867e;--ok:#7fd49a}}
:root[data-theme="dark"]{--bg:#151618;--fg:#e8e9eb;--mut:#9a9ea6;--panel:#1f2124;--line:#33363b;--hl:#3a3420;--bad:#f2867e;--ok:#7fd49a}
body{background:var(--bg);color:var(--fg);font:15px/1.6 system-ui,sans-serif;margin:0;padding:16px;max-width:980px;margin:auto}
h1{font-size:22px} h2{font-size:17px;margin-top:28px} .mut{color:var(--mut)} .bad{color:var(--bad)} .ok{color:var(--ok)}
.box{background:var(--panel);border-radius:8px;padding:12px 14px}
.tw{overflow-x:auto} table{border-collapse:collapse;font-size:13px;min-width:560px;margin:6px 0}
th,td{border-bottom:1px solid var(--line);padding:5px 8px;text-align:right} th{text-align:left}
thead th{text-align:center} tr.hl td,tr.hl th{background:var(--hl)}
</style>"""

H = ['<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">',
     "<title>끝 정지 바이어스 점검</title>", css, "</head><body>", "<h1>끝 정지 바이어스 점검 (시작 정지 vs 끝 정지)</h1>",
     '<p class="mut">2026-10-21. 녹화 hs_20261020_223815, hs_20261020_224323 (둘 다 처음·끝에 정지). 새 VIO 실행 없음 — 녹화 IMU 와 밤샘 VIO 결과 파일만 다시 읽음. '
     "스크립트 imu_learn/endstill_check.py → runs/endstill_tmp/endstill_check.json, 이 페이지는 imu_learn/endstill_html.py.</p>"]

# ---------- 결론 ----------
c1, c2 = D["recs"][RECS[0]]["compare"], D["recs"][RECS[1]]["compare"]
H.append('<div class="box"><b>결론</b><br>'
         f"① 원본 자이로(type 16) 바이어스는 끝 정지에서 <b>x +{c1['gyro_raw16_mdeg_s']['delta'][0]:.0f} / +{c2['gyro_raw16_mdeg_s']['delta'][0]:.0f}, "
         f"z +{c1['gyro_raw16_mdeg_s']['delta'][2]:.0f} / +{c2['gyro_raw16_mdeg_s']['delta'][2]:.0f} mdeg/s</b> 올랐다 (223815 / 224323). "
         f"배터리 온도 상승(+{c1['dT_battery_c']:.1f} / +{c2['dT_battery_c']:.1f} °C) × 배터리 기울기로 예측한 값(x +{c1['pred_gyro_batt'][0]:.0f}/+{c2['pred_gyro_batt'][0]:.0f}, z +{c1['pred_gyro_batt'][2]:.0f}/+{c2['pred_gyro_batt'][2]:.0f})보다 <b>약 1.3~1.4배</b> 크다. "
         "배터리가 칩보다 늦게 데워지니 방향과 크기는 맞는 편 — 실측 변화를 기울기로 나누면 칩 쪽이 배터리보다 약 2 °C 더 올랐다는 뜻 (추정).<br>"
         "② y 축은 판정 불가: 끝 정지 구간에 y 만 튀는 값(최대 0.7~1.6 °/s)이 섞여 std 가 2배, 1초 묶음 평균이 ±70 mdeg/s 흔들림. "
         f"부호도 두 녹화가 반대(+{c1['gyro_raw16_mdeg_s']['delta'][1]:.0f} / {c2['gyro_raw16_mdeg_s']['delta'][1]:.0f}).<br>"
         f"③ 가속도는 비교 불가: 두 정지에서 폰 놓인 방향이 {c1['gravity_angle_deg']:.1f}° / {c2['gravity_angle_deg']:.1f}° 달라 (기준 1°) 바이어스와 중력 방향이 섞인다. "
         "|g| 만 보면 +0.2 / +0.4 mg 차이 — 예측(축당 ±1~2 mg)과 어긋나지 않음 (약한 검사).<br>"
         "④ <b>밤샘 VIO 는 끝 정지를 덮지 않는다</b>: 모든 실행이 녹화 앞 60 s 만 잘라 돌렸다 (VIO 마지막 시각 ≈ 62 s, 끝 정지는 133~141 s / 135~138 s). "
         "그래서 끝 정지의 VIO 속도·위치 표류는 <b>잴 수 없음</b>. 대신 VIO 마지막(≈62 s) 바이어스를 IMU 실측(시작·끝 정지를 배터리 온도로 이은 값)과 비교했다.<br>"
         "⑤ VIO 자이로 바이어스: 시작 추정은 실측과 x·z 4~9 mdeg/s 이내로 맞음. 62 s 에서 x·z 는 실측 보간과 대략 맞지만(설정·시작점에 따라 ±30), "
         "<b>y 는 원본 IMU 실행에서 +90~+130, 보정 IMU(type 4) 223815 실행에서 +430</b> 로 실측 예측(+34)보다 훨씬 크다 → VIO 가 y 바이어스로 다른 오차를 흡수하는 듯 (추정, 녹화 2개·시작점 3개뿐).<br>"
         "⑥ 정지 바이어스 미리 빼기(C/C2)는 VIO 결과를 바꾸지 않음 — 정지 초기화가 같은 구간에서 바이어스를 다시 뽑아서, 결과가 B·A 와 숫자까지 같다.</div>")

# ---------- 1. 자이로 ----------
H.append("<h2>1. 자이로 바이어스 — 시작 정지 vs 끝 정지</h2>")
H.append('<p class="mut">정지 판정: 원본 자이로 0.5 s 창(0.1 s 간격) 축별 std &lt; 0.01 rad/s, |평균| &lt; 0.03 rad/s. 경계에서 0.5 s, 끝(정지 버튼 누름)은 1 s 잘라 씀. '
         "± 는 두 가지: <b>std/√N</b> (샘플 독립 가정, 너무 낙관적) / <b>1초 묶음</b> (1초 평균들의 흩어짐/√묶음수, 느린 흔들림 포함, 더 정직). "
         "자이로 한 칸(LSB) = 70 mdeg/s 로 거칠지만, 잡음 std 가 55~80 mdeg/s (≈1칸)라 평균하면 칸보다 작은 값도 대략 나온다 (중앙값은 70 의 배수로만 나옴). "
         "지구 자전(≈4 mdeg/s) 은 안 뺐음 — 두 정지의 방향 차이 5~6° 로 생기는 차이는 0.5 mdeg/s 이하.</p>")
for rec in RECS:
    r = D["recs"][rec]
    S, E, C = r["start"], r["end"], r["compare"]
    head = ["", "x", "y", "z"]
    rows = []
    for nm, key in [("원본 type 16", "gyro_raw16_mdeg_s"), ("안드로이드 보정 type 4", "gyro_cal4_mdeg_s")]:
        for lab, s in [("시작", S), ("끝", E)]:
            st = s[key]
            rows.append(row([f"{m:.1f} <span class=mut>±{a:.1f} / ±{('—' if np.isnan(b) else f'{b:.1f}')}</span>"
                             for m, a, b in zip(st["mean"], st["sem"], st["block_sem_1s"])], f"{nm} {lab}"))
        cc = C[key]
        rows.append(row([f"<b>{d:+.1f}</b> <span class=mut>±{a:.1f} / ±{('—' if np.isnan(b) else f'{b:.1f}')}</span>"
                         for d, a, b in zip(cc["delta"], cc["u_sem"], cc["u_block"])], f"{nm} 변화 (끝−시작)", "hl"))
    rows.append(row([f"{v:+.1f}" for v in C["pred_gyro_batt"]], f"예측: 배터리 기울기 × ΔT배터리 {C['dT_battery_c']:+.1f} °C"))
    rows.append(row([f"{v:+.1f}" for v in C["pred_gyro_cp_slope_x_dTbatt"]], "참고: CP 기울기 × ΔT배터리 (CP 온도는 이 녹화에 없음)"))
    rows.append(row([f"{v:+.1f}" for v in C["implied_dT_from_batt_slope"]], "실측 변화 ÷ 배터리 기울기 = 실효 ΔT (°C, 추정)"))
    rows.append(row([f"{v:.0f} → {w:.0f}" for v, w in zip(S["gyro_raw16_mdeg_s"]["android_bias_est"], E["gyro_raw16_mdeg_s"]["android_bias_est"])],
                    "안드로이드 자체 바이어스 추정 (type 16 의 4~6열) 시작 → 끝"))
    H.append(f"<h3>{rec}</h3><p class=mut>사용 구간: 시작 {r['used']['start'][0]:.1f}~{r['used']['start'][1]:.1f} s, 끝 {r['used']['end'][0]:.1f}~{r['used']['end'][1]:.1f} s "
             f"(녹화 길이 {r['t_end_s']:.1f} s). 배터리 {S['temp']['battery_c_mean']:.1f} → {E['temp']['battery_c_mean']:.1f} °C, "
             f"열 상태 {S['temp']['thermal_status_max']} → {E['temp']['thermal_status_max']}. 샘플 수 시작 {S['gyro_raw16_mdeg_s']['n']}, 끝 {E['gyro_raw16_mdeg_s']['n']} (≈500 Hz). 단위 mdeg/s.</p>")
    H.append(table(head, rows))
H.append('<p class="mut">읽는 법: 보정 type 4 = 원본 − 안드로이드 추정값. 안드로이드가 녹화 중 자기 추정을 70 mdeg/s 칸 단위로 바꾸므로 (예: 223815 z 70→138), '
         "type 4 의 변화는 온도 영향이 아니라 '안드로이드가 따라간 정도'다. 끝 정지에서도 type 4 에 20~60 mdeg/s 가 남아 있다 → 안드로이드 보정만으로는 바이어스가 0 이 아님. "
         "224323 끝 정지는 2.1 s 뿐이라 1초 묶음 ± 는 못 냄.</p>")
H.append('<p class="mut">y 축 주의: 끝 정지에서 y 만 중앙값에서 0.5 °/s 넘게 튀는 샘플이 152개(223815)·23개(224323), 시작 정지는 25·0개. '
         "1초 묶음 y 평균 223815 끝: 248, 210, 158, 279, 309, 311 — 움직임 판정 기준(0.57 °/s std)은 통과하지만 미세 진동/손 닿음이 섞인 듯 (추정). y 의 변화값은 쓰지 말 것.</p>")

# ---------- 가속도 ----------
H.append("<h2>2. 가속도 (type 35) — 방향이 달라 비교 불가</h2>")
rows = []
for rec in RECS:
    r = D["recs"][rec]; C = r["compare"]
    rows.append(row([", ".join(f"{v:.1f}" for v in r["start"]["acc_raw35_mg"]["mean"]),
                     ", ".join(f"{v:.1f}" for v in r["end"]["acc_raw35_mg"]["mean"]),
                     f"{C['gravity_angle_deg']:.2f}°", f"{C['gravity_norm_mg'][0]:.1f} → {C['gravity_norm_mg'][1]:.1f}",
                     ", ".join(f"{v:+.1f}" for v in C["pred_acc_batt_mg"])], rec))
H.append(table(["녹화", "시작 평균 (mg, x,y,z)", "끝 평균", "중력 방향 차", "|g| (mg)", "온도 예측 바이어스 변화 (mg)"], rows))
H.append('<p class="mut">방향 차 5~6° 면 중력이 축 사이로 ~90 mg 옮겨가서, 예측되는 바이어스 변화(1~2 mg)를 덮는다. 기준(1°) 넘어서 바이어스 비교 안 함. '
         "|g| 차이(+0.2, +0.4 mg)는 방향과 무관하지만 바이어스 3축이 섞인 값이라 약한 확인일 뿐. type 1 은 type 35 와 같은 값(이 폰은 가속도 바이어스 보정을 안 함, 4~6열 = 0).</p>")

# ---------- 3. VIO ----------
H.append("<h2>3. 밤샘 VIO — 끝 정지 표류와 바이어스</h2>")
H.append('<div class="box"><b>끝 정지는 VIO 범위 밖.</b> runs/overnight_vio 의 223815·224323 실행 48개 모두 clip 60 s (start_off 0/1/2 s) — VIO 포즈는 ≈10.5~62 s 에만 있다. '
         "끝 정지(133~141 s, 135~138 s)에서의 VIO 평균 |v|·위치 변화는 <b>잴 수 없음</b>. 시작 정지도 VIO 초기화 직전(≈10.5~12.6 s)이라 정지 중 포즈가 없다. "
         "c5_retest (지금 돌고 있는 것) 도 clip 60 s 라 마찬가지. 끝 정지 표류를 보려면 녹화 전체(≈140 s)를 돌리는 실행이 따로 필요.</div>")
H.append('<p class="mut">아래는 대신 볼 수 있는 것: VIO 의 자이로 바이어스(bg) 를 첫 추적 포즈(= 정지 초기화 추정)와 마지막 포즈(≈62 s)에서 꺼내, 원래 원본 축 기준으로 맞춘 값 '
         "(보정 실행은 bg − 보정값). 비교 기준 '실측 보간' = 시작·끝 정지 원본 평균을 배터리 온도로 선형 보간 (y 는 끝 정지가 흔들려 참고만). "
         "보정 IMU(type 4) 실행은 안드로이드 추정이 빠진 축이라 원본 실측과 직접 비교 안 되고, 변화량(Δ)만 본다. 단위 mdeg/s. 녹화 2개 × 시작점 3개 — 작은 표본.</p>")
for rec in RECS:
    r = D["recs"][rec]; S, E = r["start"], r["end"]
    tS, tE = S["temp"]["battery_c_mean"], E["temp"]["battery_c_mean"]
    mS, mE = np.array(S["gyro_raw16_mdeg_s"]["mean"]), np.array(E["gyro_raw16_mdeg_s"]["mean"])
    rows = []
    v0 = r["vio"][0]
    fr = (v0["battery_c_last"] - tS) / (tE - tS)
    interp = mS + fr * (mE - mS)
    rows.append(row(["—", ", ".join(f"{v:.0f}" for v in mS), ", ".join(f"{v:.0f}" for v in interp),
                     ", ".join(f"{v:+.0f}" for v in interp - mS), ", ".join(f"{v:+.0f}" for v in SB * (v0['battery_c_last'] - tS)), "—", "—"],
                    f"IMU 실측 (배터리 {tS:.1f} → {v0['battery_c_last']:.1f} °C at 62 s)", "hl"))
    for v in r["vio"]:
        if "t_last" not in v:
            rows.append(row(["초기화 실패"] + [""] * 6, v["cfg"]))
            continue
        raw = v["imu_source"] == "raw_16_35"
        rows.append(row([f"{v['t_init']:.1f}–{v['t_last']:.1f}",
                         ", ".join(f"{x:.0f}" for x in v["bg_init_mdeg_s"]) if raw else "<span class=mut>(type 4) </span>" + ", ".join(f"{x:.0f}" for x in v["bg_init_mdeg_s"]),
                         ", ".join(f"{x:.0f}" for x in v["bg_last_mdeg_s"]),
                         ", ".join(f"{x:+.0f}" for x in v["bg_delta_mdeg_s"]), "",
                         ", ".join(f"{x:+.1f}" for x in np.subtract(v["ba_last_mg"], v["ba_init_mg"])),
                         f"{v['ate_cm']}"], f"{v['cfg']} ({'원본' if raw else '보정'})"))
    H.append(f"<h3>{rec}</h3>")
    H.append(table(["실행", "VIO 시간 (s)", "bg 시작 (x,y,z)", "bg 62 s", "Δbg", "배터리 예측 Δ", "Δba (mg)", "ATE cm"], rows))
H.append('<p class="mut">읽는 법: <b>시작 bg</b> 는 원본 실행에서 IMU 정지 평균과 x·z 4~9 mdeg/s 이내 (y 는 223815 에서 25~34 낮음) — 정지 초기화는 잘 됨. '
         "<b>62 s 의 x·z</b> 는 실측 보간 ±30 안 (시작점마다 0~46 흔들림). <b>y 는 원본 실행 +90~+130, 보정 실행 223815 +430</b> — 실측(끝 정지 원본 y 는 시작보다 +50/−40, 둘 다 흔들림 섞임)과 "
         "온도 예측(+34)보다 훨씬 커서 VIO 가 다른 오차(시간 어긋남·회전 외부보정 등)를 y 바이어스로 흡수하는 것으로 보임 (추정). "
         "ba 의 y 도 모든 실행에서 −3~−15 mg (대부분 −9~−15) 로 가는데 온도 예측은 −1 mg 정도 — 같은 흡수일 가능성 (추정, 정지 끝에서 확인 불가). "
         "C2(원본+정지 바이어스 빼기)는 B(원본) 와, C_stillbias 는 A_strict_cal 과 bg 합계·ATE 가 같다: 필터가 바이어스 이동에 대해 거의 선형이고 정지 초기화가 같은 구간에서 다시 뽑으므로 "
         "미리 빼는 것은 효과가 없다. A_loose = A_strict 도 같은 값 (두 설정이 이 녹화에서 같은 정적 초기화를 탐).</p>")

H.append("<h2>4. 정리 · 다음</h2><ul>"
         "<li>원본 자이로 x·z: 2분 녹화 동안 +37~+47 mdeg/s. 배터리 기준 예측의 1.3~1.4배 — 칩 온도 지연을 생각하면 일관됨. 비행 중 바이어스를 고정값으로 두면 2분에 이만큼 틀린다.</li>"
         "<li>y 축과 가속도는 이번 끝 정지로는 판정 못함 (흔들림 / 놓인 방향 다름). 다음 녹화는 끝 정지를 10 s 이상, 처음과 같은 자리·같은 방향으로 내려놓기.</li>"
         "<li>VIO 끝 정지 표류는 녹화 전체(≈140 s)를 도는 실행이 있어야 잴 수 있음 — 이번 작업 범위(새 VIO 금지) 밖.</li>"
         "<li>VIO 의 y 자이로 바이어스·y 가속도 바이어스 표류는 온도로 설명되는 양보다 3배 이상 큼 → 진짜 바이어스가 아닐 가능성 (추정, 녹화 2개).</li></ul>")
H.append("</body></html>")
open(os.path.join(ROOT, "docs/endstill_check.html"), "w").write("\n".join(H))
print("wrote docs/endstill_check.html")
