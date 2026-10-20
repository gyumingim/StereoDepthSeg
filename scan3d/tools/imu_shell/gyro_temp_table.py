#!/usr/bin/env python3
"""runs/gyro_bias_stability.json → docs/gyro_temp_table.html (gyro bias vs temperature: tables + scatter/time plots)."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
d = json.load(open(ROOT / "runs/gyro_bias_stability.json"))
th = d["thermal"]
t = th["t"]
G = th["gyro_mdeg_s_earth_corr"]
cam = th["cam"]
heat_end = max(b for a, b in d["camera_intervals_s"])
static_end = d["static_end_s"]
phase = ["room" if ti < static_end else ("heat" if ti <= heat_end else "cool") for ti in t]
COL = {"room": "#8a8f98", "heat": "#e5484d", "cool": "#3b82f6"}
NAME = {"battery": "배터리", "BAT": "배터리(HAL)", "AP": "AP (프로세서)", "CP": "CP (통신 칩)", "PA": "PA (전파 증폭기)",
        "SKIN": "표면", "USB": "USB 단자"}
ORDER = ["CP", "PA", "AP", "SKIN", "battery", "BAT", "USB"]


def rows():
    out = []
    for s in ORDER:
        v = th["per_source"][s]
        cells = []
        for a in "xyz":
            g = v[f"gyro_raw_mdeg_s_{a}"]
            cells.append(g)
        out.append((s, v["range_C"], cells))
    return out


def fmt(x, n=2):
    return f"{x:.{n}f}"


def scatter(src, axis):
    T = th["T_blocks"][src]
    ys = [g[axis] for g in G]
    pts = [(T[i], ys[i], phase[i], cam[i]) for i in range(len(t)) if T[i] is not None]
    W, H, P = 300, 220, 36
    xs = [p[0] for p in pts]; yy = [p[1] for p in pts]
    x0, x1 = min(xs) - 0.3, max(xs) + 0.3
    y0, y1 = min(yy) - 3, max(yy) + 3
    X = lambda v: P + (v - x0) / (x1 - x0) * (W - P - 8)
    Y = lambda v: H - P + 8 - (v - y0) / (y1 - y0) * (H - P - 8)
    g = th["per_source"][src][f"gyro_raw_mdeg_s_{'xyz'[axis]}"]
    k = g["slope_per_C"]
    # fitted line through the means
    mx = sum(xs) / len(xs); my = sum(yy) / len(yy)
    line = f'<line x1="{X(x0):.1f}" y1="{Y(my + k * (x0 - mx)):.1f}" x2="{X(x1):.1f}" y2="{Y(my + k * (x1 - mx)):.1f}" stroke="var(--fg)" stroke-dasharray="4 3" stroke-width="1"/>'
    dots = "".join(
        f'<circle cx="{X(a):.1f}" cy="{Y(b):.1f}" r="{3.2 if c else 2.6}" fill="{COL[ph]}" fill-opacity="0.8"'
        f'{" stroke=\"var(--fg)\" stroke-width=\"0.8\"" if c else ""}/>' for a, b, ph, c in pts)
    ticks = "".join(f'<text x="{X(v):.1f}" y="{H - P + 22}" font-size="10" text-anchor="middle" fill="var(--mut)">{v:.0f}</text>'
                    for v in range(int(x0) + 1, int(x1) + 1, 2))
    yt = "".join(f'<text x="{P - 4}" y="{Y(v) + 3:.1f}" font-size="10" text-anchor="end" fill="var(--mut)">{v:.0f}</text>'
                 for v in range(int(y0 // 10 * 10 + 10), int(y1), 10))
    return (f'<figure><svg viewBox="0 0 {W} {H}" role="img"><rect x="{P}" y="0" width="{W - P - 8}" height="{H - P + 8}" fill="var(--panel)"/>'
            f'{line}{dots}{ticks}{yt}<text x="{(W + P) / 2}" y="{H - 2}" font-size="10" text-anchor="middle" fill="var(--mut)">{NAME[src]} 온도 °C</text></svg>'
            f'<figcaption>{"xyz"[axis]}축 — 기울기 {k:.2f} mdeg/s/°C, r {g["r"]:.3f}</figcaption></figure>')


def timeplot(src):
    W, H, P = 640, 200, 40
    T = th["T_blocks"][src]
    tmax = max(t)
    X = lambda v: P + v / tmax * (W - P - 40)
    Tmin, Tmax = min(T) - 0.5, max(T) + 0.5
    YT = lambda v: H - 24 - (v - Tmin) / (Tmax - Tmin) * (H - 34)
    paths = []
    colors = ["#e5484d", "#30a46c", "#3b82f6"]
    for a in range(3):
        ys = [g[a] for g in G]; m0 = sum(ys[:10]) / 10
        dev = [y - m0 for y in ys]
        lo, hi = -10, max(80, max(dev) + 5)
        Y = lambda v: H - 24 - (v - lo) / (hi - lo) * (H - 34)
        paths.append(f'<polyline fill="none" stroke="{colors[a]}" stroke-width="1.5" points="' +
                     " ".join(f"{X(t[i]):.1f},{Y(dev[i]):.1f}" for i in range(len(t))) + '"/>')
    tp = '<polyline fill="none" stroke="var(--fg)" stroke-width="2" stroke-dasharray="5 3" points="' + \
         " ".join(f"{X(t[i]):.1f},{YT(T[i]):.1f}" for i in range(len(t))) + '"/>'
    shade = "".join(f'<rect x="{X(a):.1f}" y="4" width="{X(b) - X(a):.1f}" height="{H - 28}" fill="#e5484d" fill-opacity="0.12"/>'
                    for a, b in d["camera_intervals_s"])
    xt = "".join(f'<text x="{X(v):.1f}" y="{H - 8}" font-size="10" text-anchor="middle" fill="var(--mut)">{v // 60:.0f}분</text>'
                 for v in range(0, int(tmax), 300))
    return (f'<figure class="wide"><svg viewBox="0 0 {W} {H}">{shade}{"".join(paths)}{tp}{xt}</svg>'
            f'<figcaption>실선 = 자이로 바이어스 변화 (처음 5분 평균 대비, x <b style="color:#e5484d">빨강</b> · y <b style="color:#30a46c">초록</b> · z <b style="color:#3b82f6">파랑</b>), '
            f'점선 = {NAME[src]} 온도 ({Tmin + 0.5:.0f}~{Tmax - 0.5:.0f} °C), 붉은 칸 = 240 fps 카메라 켬 (데우기)</figcaption></figure>')


tbl = []
for s, rng, cells in rows():
    best = ' class="best"' if s in ("CP", "PA") else ""
    tds = "".join(f"<td>{fmt(c['slope_per_C'])}</td><td>{fmt(c['r'], 3)}</td><td>{fmt(c['hysteresis_cooling_minus_heating'], 1)}</td>" for c in cells)
    tbl.append(f"<tr{best}><th>{NAME[s]}</th><td>{rng:.1f}</td>{tds}</tr>")

cp = th["per_source"]["CP"]
eff = "".join(
    f"<tr><th>{dT} °C</th>" + "".join(f"<td>{cp[f'gyro_raw_mdeg_s_{a}']['slope_per_C'] * dT:.0f}</td>" for a in "xyz") + "</tr>"
    for dT in (1, 5, 10, 15))

html = f"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>자이로 바이어스와 온도</title>
<style>
:root{{--bg:#fff;--fg:#1c1d1f;--mut:#6b6f76;--panel:#f4f5f7;--line:#dfe1e5;--hl:#fff6d6}}
@media (prefers-color-scheme:dark){{:root:not([data-theme="light"]){{--bg:#151618;--fg:#e8e9eb;--mut:#9a9ea6;--panel:#1f2124;--line:#33363b;--hl:#3a3420}}}}
:root[data-theme="dark"]{{--bg:#151618;--fg:#e8e9eb;--mut:#9a9ea6;--panel:#1f2124;--line:#33363b;--hl:#3a3420}}
body{{background:var(--bg);color:var(--fg);font:15px/1.6 system-ui,sans-serif;margin:0;padding:16px;max-width:980px;margin:auto}}
h1{{font-size:22px}} h2{{font-size:17px;margin-top:28px}} .mut{{color:var(--mut)}}
.box{{background:var(--panel);border-radius:8px;padding:12px 14px}}
.tw{{overflow-x:auto}} table{{border-collapse:collapse;font-size:13px;min-width:560px}}
th,td{{border-bottom:1px solid var(--line);padding:5px 8px;text-align:right}} th{{text-align:left}}
thead th{{text-align:center}} tr.best td,tr.best th{{background:var(--hl)}}
.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:10px}}
figure{{margin:0}} figure.wide{{grid-column:1/-1}} svg{{width:100%;height:auto}} figcaption{{font-size:12px;color:var(--mut)}}
.lg span{{display:inline-block;width:10px;height:10px;border-radius:50%;margin:0 4px 0 10px}}
</style></head><body>
<h1>자이로 바이어스와 온도</h1>
<p class="mut">S24 (자이로 칩 STM LSM6DSV), 원본 자이로 (type 16), 화면 아래로 책상 위 정지. {d['wall_start']} 부터 {d['duration_s'] / 60:.0f}분:
실온 정지 {static_end / 60:.0f}분 → 240 fps 카메라로 데우기 → 식히기. 30초 묶음 평균, 지구 자전 뺀 값. 움직인 구간 {len(d['cuts'])}곳 뺌.</p>
<div class="box"><b>결론</b><br>
① 자이로 바이어스는 온도를 거의 직선으로 따라간다 — <b>칩 쪽 온도(CP·PA) 1 °C 당 x 3.9 · y 7.0 · z 4.4 mdeg/s</b>, 상관 r 0.99~1.00.<br>
② 데울 때·식을 때 차이(히스테리시스)가 CP·PA 기준 0~1.7 mdeg/s 로 거의 없다 → 지금 온도만 알면 보정된다. 과거 온도 평균은 필요 없음 (가장 잘 맞는 평균 길이 0~20 s).<br>
③ 배터리·표면 온도는 칩보다 늦게 데워지고 식어서 히스테리시스가 2~5 mdeg/s 로 커진다 → 보정에는 CP 를 쓴다.<br>
④ 240 fps 녹화 3분에 약 8~10 °C 오르므로 녹화 한 번 안에서 바이어스가 40~70 mdeg/s 움직인다.<br>
<span class="mut">주의(추정): 데우기 = 카메라 켬이라 카메라 전원 효과(x +5.8, z −5.2 mdeg/s 정도)가 섞여 있고, 가열은 사실상 1번 → 카메라 없이 데우는 실험으로 확인 예정 (docs/todo_experiments.md).</span></div>

<h2>1. 온도 센서별 상관 (mdeg/s per °C)</h2>
<div class="tw"><table><thead><tr><th rowspan="2">온도 센서</th><th rowspan="2">온도 폭 °C</th><th colspan="3">x 축</th><th colspan="3">y 축</th><th colspan="3">z 축</th></tr>
<tr><th>기울기</th><th>r</th><th>히스테리시스</th><th>기울기</th><th>r</th><th>히스테리시스</th><th>기울기</th><th>r</th><th>히스테리시스</th></tr></thead>
<tbody>{''.join(tbl)}</tbody></table></div>
<p class="mut">기울기 = 온도 1 °C 오를 때 바이어스 변화 (mdeg/s = 0.001 °/s). r = 상관 (1 이면 완전한 직선). 히스테리시스 = 같은 온도에서 식을 때 − 데울 때 (mdeg/s, 0 에 가까울수록 좋음).
노란 줄 = 보정에 쓸 센서. USB 는 온도 폭이 작고 늦게 따라가 맞지 않음.</p>

<h2>2. 실제로 얼마나 움직이나 (CP 기준, mdeg/s)</h2>
<div class="tw"><table><thead><tr><th>온도 상승</th><th>x</th><th>y</th><th>z</th></tr></thead><tbody>{eff}</tbody></table></div>
<p class="mut">비교: 이 폰 자이로 바이어스 자체가 약 (116, 179, 71) mdeg/s, 정지 중 2초 평균 잡음 2~3 mdeg/s, 칩 최소 단위 70 mdeg/s.</p>

<h2>3. 보정 모델 비교 (빼 둔 구간으로 채점, 잔차 mdeg/s)</h2>
<div class="tw"><table><thead><tr><th>모델</th><th>x</th><th>y</th><th>z</th></tr></thead><tbody>
<tr><th>온도 무시 (상수 바이어스)</th><td colspan="3" style="text-align:center">27 ~ 42</td></tr>
<tr><th>지금 온도 1차식</th><td>3.41</td><td>3.63</td><td>6.06</td></tr>
<tr class="best"><th>가장 좋은 모델</th><td>2.81 <span class="mut">(AP+BAT)</span></td><td>3.13 <span class="mut">(PA)</span></td><td>2.55 <span class="mut">(CP + 온도 변화 속도)</span></td></tr>
<tr><th>잡음 바닥 (10초 평균)</th><td colspan="3" style="text-align:center">1.7 ~ 2.1</td></tr>
</tbody></table></div>
<p class="mut">출처: docs/temp_history_bias.html. 두 센서 섞기는 다른 구간에서 나빠지기도 해서(z 1.86→7.73) 권장은 CP 1차식 하나.</p>

<h2>4. 그림</h2>
<p class="lg mut"><span style="background:{COL['room']}"></span>실온 정지<span style="background:{COL['heat']}"></span>데우는 중<span style="background:{COL['cool']}"></span>식는 중 (테두리 = 카메라 켬)</p>
<div class="grid">{timeplot('CP')}{scatter('CP', 0)}{scatter('CP', 1)}{scatter('CP', 2)}{scatter('battery', 1)}</div>
<p class="mut">마지막 그림(배터리, y축): 같은 바이어스인데 데울 때(빨강)와 식을 때(파랑)가 갈라짐 → 배터리가 늦게 따라가는 흔적.</p>
<p class="mut">데이터: recordings/static_imu_20261020_095113 · 수치 runs/gyro_bias_stability.json · 만든 스크립트 tools/imu_shell/gyro_temp_table.py</p>
</body></html>"""
(ROOT / "docs/gyro_temp_table.html").write_text(html)
print("wrote docs/gyro_temp_table.html")
