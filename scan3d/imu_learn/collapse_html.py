#!/usr/bin/env python3
"""docs/vio_collapse_detect.html from collapse_report.py + collapse_signals.py output (plots = inline SVG).
  ../venv_ffs/bin/python imu_learn/collapse_html.py runs/collapse/report.json runs/collapse/signals.json docs/vio_collapse_detect.html
Narrative numbers come from the JSON; the text blocks (NOTES) are written by hand after reading the results."""
import html
import json
import math
import sys

import numpy as np

rep = json.load(open(sys.argv[1]))
SIG = {r["name"]: r for r in json.load(open(sys.argv[2])) if "series" in r}
OUT = sys.argv[3]
NOTES = json.load(open(sys.argv[4])) if len(sys.argv) > 4 else {}
E = html.escape


def frac(e):
    return f"{e['tp']}/{e['n_c']}", f"{e['fa']}/{e['n_h']}"


def fmt(x, d=1):
    return "-" if x is None else f"{x:.{d}f}"


def plot(key, thr, tmin, title, unit, logy=False, ymax=None):
    W, H, L, B, T, Rr = 460, 220, 46, 30, 22, 10
    runs = list(SIG.values())
    vals = []
    for r in runs:
        s = r["series"]
        if key in s:
            t = np.asarray(s["t"]); x = np.asarray(s[key], float)
            vals += list(x[(t <= 10) & np.isfinite(x)])
    if not vals:
        return ""
    hi = ymax or max(np.nanpercentile(vals, 99), thr * 1.6)
    lo = max(min(v for v in vals if v > 0) if logy else 0, hi / 1e3) if logy else 0.0

    def ys(v):
        v = min(max(v, lo), hi)
        if logy:
            return T + (H - T - B) * (1 - (math.log10(v) - math.log10(lo)) / (math.log10(hi) - math.log10(lo)))
        return T + (H - T - B) * (1 - (v - lo) / (hi - lo))

    def xs(t):
        return L + (W - L - Rr) * t / 10.0
    out = [f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="{E(title)}">']
    for tt in (0, 2, 4, 6, 8, 10):
        out.append(f'<line class="grid" x1="{xs(tt):.1f}" x2="{xs(tt):.1f}" y1="{T}" y2="{H - B}"/>'
                   f'<text class="ax" x="{xs(tt):.1f}" y="{H - B + 14}" text-anchor="middle">{tt}</text>')
    ticks = [10 ** k for k in range(math.floor(math.log10(lo)), math.ceil(math.log10(hi)) + 1)] if logy else \
        list(np.linspace(lo, hi, 5))
    for v in ticks:
        if lo <= v <= hi:
            out.append(f'<line class="grid" x1="{L}" x2="{W - Rr}" y1="{ys(v):.1f}" y2="{ys(v):.1f}"/>'
                       f'<text class="ax" x="{L - 4}" y="{ys(v) + 4:.1f}" text-anchor="end">{v:.3g}</text>')
    for r in sorted(runs, key=lambda r: r["collapsed"]):
        s = r["series"]
        if key not in s:
            continue
        t = np.asarray(s["t"]); x = np.asarray(s[key], float)
        m = (t <= 10) & np.isfinite(x)
        if m.sum() < 2:
            continue
        pts = " ".join(f"{xs(a):.1f},{ys(b):.1f}" for a, b in zip(t[m][::2], x[m][::2]))
        cls = "c" if r["collapsed"] else "h"
        lab = f"{r['name']} ({'붕괴' if r['collapsed'] else '정상'}, 창 스케일 {fmt(r.get('scale10'), 2)})"
        out.append(f'<polyline class="ln {cls}" points="{pts}"><title>{E(lab)}</title></polyline>')
    out.append(f'<line class="thr" x1="{xs(tmin):.1f}" x2="{W - Rr}" y1="{ys(thr):.1f}" y2="{ys(thr):.1f}"/>'
               f'<text class="ax" x="{W - Rr}" y="{ys(thr) - 4:.1f}" text-anchor="end">문턱 {thr:g} {E(unit)}</text>')
    out.append(f'<text class="ax" x="{L}" y="{T - 8}">{E(title)}</text>'
               f'<text class="ax" x="{W - Rr}" y="{H - 2}" text-anchor="end">초기화 뒤 시간 (s)</text></svg>')
    return "".join(out)


S = {s["name"]: s for s in rep["signals"]}
C = rep["combos"]
css = """
:root{--bg:#fcfcfb;--fg:#0b0b0b;--mut:#52514e;--panel:#f2f1ed;--line:#dedcd5;--hl:#fff3d1;--h:#2a78d6;--c:#eb6834}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#1a1a19;--fg:#f0efec;--mut:#c3c2b7;--panel:#252523;--line:#3a3936;--hl:#3a3420;--h:#3987e5;--c:#d95926}}
:root[data-theme="dark"]{--bg:#1a1a19;--fg:#f0efec;--mut:#c3c2b7;--panel:#252523;--line:#3a3936;--hl:#3a3420;--h:#3987e5;--c:#d95926}
body{background:var(--bg);color:var(--fg);font:15px/1.65 system-ui,sans-serif;margin:0 auto;padding:16px;max-width:980px}
h1{font-size:22px}h2{font-size:17px;margin-top:30px}.mut{color:var(--mut)}.box{background:var(--panel);border-radius:8px;padding:12px 14px}
.tw{overflow-x:auto}table{border-collapse:collapse;font-size:13px}th,td{border-bottom:1px solid var(--line);padding:5px 8px;text-align:right;white-space:nowrap}
th:first-child,td:first-child{text-align:left}tr.hl td{background:var(--hl)}
.grid4{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:12px}svg{width:100%;height:auto}
svg .grid{stroke:var(--line);stroke-width:1}svg .ax{fill:var(--mut);font-size:11px}
svg .ln{fill:none;stroke-width:1.6;opacity:.75}svg .ln.h{stroke:var(--h)}svg .ln.c{stroke:var(--c)}
svg .ln:hover{stroke-width:3;opacity:1}svg .thr{stroke:var(--fg);stroke-dasharray:5 4;stroke-width:1.2}
.lg span{display:inline-block;width:14px;height:3px;vertical-align:middle;margin:0 5px 0 12px}
code{font-size:13px}
"""
p = []
p.append(f"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>VIO 초기화 붕괴 감지</title><style>{css}</style></head><body>
<h1>VIO 초기화 붕괴, 정답 없이 몇 초 안에 알아챌 수 있나</h1>
<p class="mut">2026-10-20 · OpenVINS (PC, ov_bridge) · 240 fps 판 녹화 11 개 · 고정 벤치 설정 (meas×10, 60 fps, 640×360) ·
실행 {rep['n']} 번 (붕괴 {rep['n_collapsed']}) · 정답(GT)은 <b>붕괴/정상 이름표에만</b> 썼다</p>
""")
p.append('<div class="box">' + NOTES.get("conclusion", "") + "</div>")
p.append(NOTES.get("mechanism", ""))
# combos
p.append("<h2>감지기 성능</h2>")
p.append(NOTES.get("protocol", ""))
p.append('<div class="tw"><table><tr><th>감지기 (OR = 하나라도 넘으면 경보)</th><th>전체: 잡음</th><th>전체: 헛경보 (10 s)</th>'
         '<th>헛경보 (전 구간)</th><th>감지 시각 중앙 / 90%</th><th>시험 (지연 0.5 s): 잡음 / 헛</th>'
         '<th>시험 중 새 초기화만: 잡음 / 헛</th><th>놓친 실행 (전체)</th></tr>')
names = {"all5": "5 신호 모두", "no_chi2": "chi2 뺀 4 신호 (추천)", "cov_only": "갱신 끊김 + 속도 표준편차만"}
for k, nm in names.items():
    c = C[k]
    p.append(f'<tr class="{"hl" if k == NOTES.get("best_combo") else ""}"><td>{nm}</td><td>{frac(c["all"])[0]}</td>'
             f'<td>{frac(c["all"])[1]}</td><td>{c["all"]["fa_full"]}/{c["all"]["n_h"]}</td>'
             f'<td>{fmt(c["all"]["det_med"])} / {fmt(c["all"]["det_p90"])} s</td>'
             f'<td>{frac(c["test"])[0]} / {frac(c["test"])[1]}</td><td>{frac(c["test_new"])[0]} / {frac(c["test_new"])[1]}</td>'
             f'<td>{E(", ".join(c["all"]["missed"]) or "-")}</td></tr>')
p.append("</table></div><p class='mut'>잡음 = 붕괴 실행 중 초기화 뒤 10 s 안에 경보가 난 수. 헛경보 (10 s) = 정상 실행 중 초기화 뒤 "
         "10 s 안에 경보가 난 수, (전 구간) = 실행 끝 (초기화 뒤 약 20~29 s) 까지. '새 초기화만' = 지연 0 실행과 초기화 시각이 0.1 s 넘게 "
         "다른 것 (같은 시각이면 사실상 같은 실행의 복사).</p>")
# per signal
p.append("<h2>신호별 분리 (하나씩)</h2>")
p.append('<div class="tw"><table><tr><th>신호</th><th>뜻</th><th>문턱</th><th>시작</th><th>시험 잡음</th><th>시험 헛경보</th>'
         '<th>전체 잡음</th><th>전체 헛경보</th><th>감지 중앙</th><th>이 자료 최적 문턱 (낙관)</th></tr>')
for s in rep["signals"]:
    b = s["best_all"]
    p.append(f"<tr><td>{s['name']}</td><td style='text-align:left;white-space:normal'>{E(s['desc'])}</td><td>{s['thr']:g} {E(s['unit'])}</td>"
             f"<td>{s['tmin']:g} s</td><td>{frac(s['test'])[0]}</td><td>{frac(s['test'])[1]}</td><td>{frac(s['all'])[0]}</td>"
             f"<td>{frac(s['all'])[1]}</td><td>{fmt(s['all']['det_med'])} s</td>"
             f"<td>{b['thr']:.3g}: {b['tp']}/{b['n_c']}, 헛 {b['fa']}/{b['n_h']}</td></tr>")
p.append("</table></div>")
p.append(NOTES.get("signals_note", ""))
# plots
p.append("<h2>그림: 초기화 뒤 10 s</h2><p class='lg'><span style='background:var(--h)'></span>정상"
         "<span style='background:var(--c)'></span>붕괴 (GT 창 스케일 &lt; 0.5 또는 &gt; 2, 또는 창 ATE &gt; 10 cm) · 점선 = 문턱 · 선에 마우스 = 실행 이름</p>"
         "<div class='grid4'>")
p.append(plot("noupd_s", 3.0, 0, "카메라 갱신 없이 지난 시간 (s)", "s"))
p.append(plot("sig_v", 0.03, 3, "속도 표준편차 (m/s, 로그)", "m/s", logy=True))
p.append(plot("dv_acc1", 0.3, 0.5, "IMU 와 안 맞는 가속도 (m/s², 1 s)", "m/s²", ymax=1.5))
p.append(plot("zupt_v2", 0.3, 0.5, "'멈춤' 때 필터 속도 (m/s)", "m/s", ymax=1.5))
p.append("</div>")
p.append(NOTES.get("init_section", ""))
# run table
p.append("<h2>실행 목록</h2><div class='tw'><table><tr><th>실행</th><th>초기화</th><th>시각 s</th><th>시차 옛/새 px</th>"
         "<th>초기 bg °/s</th><th>동적 cond</th><th>창 스케일 10 s</th><th>창 ATE cm</th><th>판정</th></tr>")
for r in sorted(rep["runs"], key=lambda r: (r["rec"], r["parts"], r["start_off"] or 0)):
    ii = r["init_info"] or {}
    p.append(f"<tr><td>{E(r['name'])}</td><td>{r['init_method']}</td><td>{fmt(r['t_init_s'], 2)}</td>"
             f"<td>{fmt(ii.get('disp_old'))}/{fmt(ii.get('disp_new'))}</td><td>{fmt(r['bg_init_dps'], 2)}</td>"
             f"<td>{'%.1e' % ii['dyn_cond'] if ii.get('dyn_cond') else '-'}</td><td>{fmt(r.get('scale10'), 3)}</td>"
             f"<td>{fmt(r.get('ate10_cm'))}</td><td>{'붕괴' if r['collapsed'] else '정상'}</td></tr>")
p.append("</table></div>")
p.append(NOTES.get("method", ""))
p.append("</body></html>")
open(OUT, "w", encoding="utf-8", newline="\n").write("\n".join(p))
print("wrote", OUT)
