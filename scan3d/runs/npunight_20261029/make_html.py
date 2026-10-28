#!/usr/bin/env python3
"""npunight: summary.json -> Korean morning page (conclusion first, tables, 미검증 labels). Text blocks come from notes.json
(written by hand after reading the numbers): {"conclusion": [..html li..], "speed": [..], "next": [..], "limits": [..]}.
  make_html.py OUT.html"""
import html
import json
import os
import sys
import time

H = os.path.dirname(os.path.abspath(__file__))
S = json.load(open(os.path.join(H, "summary.json")))
N = json.load(open(os.path.join(H, "notes.json")))
rows = S["rows"]


def f(v, d=2):
    if v is None:
        return "–"
    if isinstance(v, bool):
        return "예" if v else "아니오"
    if isinstance(v, (int,)):
        return str(v)
    try:
        return f"{float(v):.{d}f}"
    except Exception:
        return html.escape(str(v))


def tab(head, body):
    h = "".join(f"<th>{x}</th>" for x in head)
    b = "".join("<tr>" + "".join(f"<td>{c}</td>" for c in r) + "</tr>" for r in body)
    return f'<div class="tw"><table><thead><tr>{h}</tr></thead><tbody>{b}</tbody></table></div>'


ab = []
for rec in ("165506", "213341", "214550", "221925", "223815"):
    for k in ("A1", "B1", "B2", "A2"):
        r = next((x for x in rows if x["tag"] == f"{rec}_{k}"), None)
        if not r:
            ab.append([rec, k, "–", "없음", "–", "–", "–", "–", "–", "–"])
            continue
        ab.append([rec, k, "NPU" if r.get("npu") == 1 else "BRIEF", f(r.get("valid")), f(r.get("map_ate")), f(r.get("vio_ate")),
                   f(r.get("map_end")), f"{r.get('correct', '–')}/{r.get('loop_gt', '–')}/{r.get('loops', '–')}", f"{f(r.get('ms50'),1)} / {f(r.get('ms95'),1)}",
                   f"{f(r.get('ap0'),1)} → {f(r.get('ap_max_pc'),1)}"])
dec = [[k, html.escape(v)] for k, v in S["decision"].items()] + [[k + " (추가, 탐색)", html.escape(v)] for k, v in S.get("decision_extra", {}).items()]
sp = []
for t in ("165506_A1", "165506_A2", "S1a", "S1b", "S2a", "S2b", "S3a", "S3b", "S4a", "S4b", "S5a", "S5b", "D1a", "D1b", "D0"):
    r = next((x for x in rows if x["tag"] == t), None)
    if not r:
        continue
    ex = r.get("extras", "")
    o = ex.split("npu_opts ")[1].split(" ")[0] if "npu_opts" in ex else "기본"
    sp.append([t, html.escape(o), html.escape(str(r.get("cpus"))), f(r.get("F_prep"), 1), f(r.get("F_npu"), 1), f(r.get("F_post"), 1),
               f(r.get("M1_post"), 1), f(r.get("M2_npu"), 1), f(r.get("M2_post"), 1), f"{f(r.get('ms50'),1)} / {f(r.get('ms95'),1)}",
               f"{r.get('correct')}/{r.get('loop_gt')}/{r.get('loops')}", f(r.get("map_ate")), f(r.get("valid"))])
li = lambda L: "".join(f"<li>{x}</li>" for x in L)
page = f"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>NPU 밤 재생 시험</title><style>
:root{{--bg:#f7f6f2;--card:#fff;--ink:#1d1d1b;--sub:#5d5b55;--line:#dedbd2;--accent:#2f6fdb;--warn:#b7791f;--soft:#eef2fb;--softw:#fbf3e4}}
@media (prefers-color-scheme: dark){{:root:not([data-theme="light"]){{--bg:#16171a;--card:#1f2125;--ink:#ecebe7;--sub:#b0ada6;--line:#34363c;--accent:#6c9cf0;--warn:#e0a84a;--soft:#1f2a3d;--softw:#3a2f1a}}}}
:root[data-theme="dark"]{{--bg:#16171a;--card:#1f2125;--ink:#ecebe7;--sub:#b0ada6;--line:#34363c;--accent:#6c9cf0;--warn:#e0a84a;--soft:#1f2a3d;--softw:#3a2f1a}}
*{{box-sizing:border-box}}body{{margin:0;background:var(--bg);color:var(--ink);font-family:-apple-system,"Apple SD Gothic Neo","Malgun Gothic","Noto Sans KR",system-ui,sans-serif;line-height:1.6}}
.wrap{{max-width:900px;margin:0 auto;padding:20px 16px 80px}}section{{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:16px;margin:16px 0}}
h1{{font-size:1.5rem;margin:.2em 0}}h2{{font-size:1.15rem;margin:0 0 8px}}.sub{{color:var(--sub);font-size:.9rem}}
.warn{{background:var(--softw);border-radius:10px;padding:8px 12px;font-size:.92rem}}.tag{{background:var(--softw);color:var(--warn);font-size:.78rem;padding:1px 8px;border-radius:999px;font-weight:600}}
.tw{{overflow-x:auto}}table{{border-collapse:collapse;font-size:.85rem;width:100%;font-variant-numeric:tabular-nums}}th,td{{border-bottom:1px solid var(--line);padding:4px 6px;text-align:left;white-space:nowrap}}th{{color:var(--sub);font-weight:600}}
li{{margin:4px 0}}code{{font-size:.85em}}</style></head><body><div class="wrap">
<p class="sub">scan3d · NPU 밤 (10-28 21:09 → 10-29 아침) · 앱 0.6.35d (A/B·S) / 0.6.36 (D) · 만든 시각 {time.strftime('%F %H:%M')}</p>
<h1>카메라 없이 폰에서 녹화를 다시 넣어 본 NPU 루프 시험 <span class="tag">전부 미검증</span></h1>
<p class="warn">모든 숫자는 혼자 잰 값이고 독립 검토 전입니다 (미검증). 각 조건 2 회뿐이라 결론은 <b>추정</b>입니다.</p>
<section><h2>결론</h2><ul>{li(N['conclusion'])}</ul></section>
<section><h2>ABBA 판정 (사전 등록 규칙)</h2>{tab(['녹화','판정'], dec)}<p><b>전체: {html.escape(S['overall'])}</b></p>
<p class="sub">규칙: 녹화마다 두 NPU run 의 지도(루프 보정) ATE 가 두 BRIEF run 보다 모두 작으면 "NPU 쪽이 낫다". 녹화 3 개 중 2 개 이상 같은 쪽이고 반대 0 개면 전체 판정.</p></section>
<section><h2>run 별 결과 (ABBA; 221925 · 223815 는 추가)</h2>
{tab(['녹화','순서','루프','유효','지도 ATE cm','VIO ATE cm','지도 끝점 cm','맞는/GT있는/채택 루프','VIO ms p50/p95','AP 시작→최대'], ab)}
<p class="sub">ATE = 카메라 위치, 보드 GT, 강체 정렬(회전+이동), 추적 중 포즈. 같은 자로 PC: 165506 지도 1.86 / VIO 2.00, 213341 지도 0.34 / VIO 0.37 cm.
"맞는 루프" = 두 키프레임 시각의 GT 카메라 거리 &lt; 0.5 m 이고 광축 차 &lt; 30°.</p></section>
<section><h2>앱 속 느린 부분 (165506, NPU 켬, ms 중앙값)</h2>
{tab(['run','npu_opts','NPU 서버 코어','XFeat 준비','XFeat NPU','XFeat 후처리','상호 NN','LG NPU','LG 후처리','VIO ms p50/p95','루프','지도 ATE','유효'], sp)}
<ul>{li(N['speed'])}</ul></section>
<section><h2>다음 할 일 · 결정할 것</h2><ul>{li(N['next'])}</ul></section>
<section><h2>한계 · 확인 안 된 것</h2><ul>{li(N['limits'])}</ul></section>
<p class="sub">자료: WSL <code>~/scanner/StereoDepthSeg-appnpu/scan3d/runs/npunight_20261029/</code> (README.md, summary.json, phone/&lt;run&gt;/score.json), 브랜치 npunight-20261029.</p>
</div></body></html>"""
open(sys.argv[1], "w", encoding="utf-8").write(page)
print("wrote", sys.argv[1])
