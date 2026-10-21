#!/usr/bin/env python3
"""docs/g10_result.html from runs/g10/summary.json + check.json + runs/g10/lead.html (hand-written conclusion)."""
import html as H
import json
from pathlib import Path

S = Path(__file__).resolve().parent.parent
G = S / "runs" / "g10"


def f(x, nd=2):
    if x is None:
        return "–"
    if isinstance(x, bool):
        return "예" if x else "아니오"
    if isinstance(x, float):
        return f"{x:.{nd}f}"
    return H.escape(str(x))


def ok(x):
    return '<b class="ok">통과</b>' if x else ('<b class="bad">실패</b>' if x is False else '<b class="na">판정 불가</b>')


def cell(x):
    if not x:
        return "<td>–</td><td>–</td><td>–</td>"
    st = x.get("status")
    cls = "bad" if st == "collapse" else ("na" if st in ("noinit", "unscored") else "")
    return f'<td class="{cls}">{f(st)}</td><td>{f(x.get("ate_cm"))}</td><td>{f(x.get("false_pass"))}</td>'


def main():
    s = json.load(open(G / "summary.json"))
    ck = json.load(open(G / "check.json")) if (G / "check.json").exists() else None
    lead = (G / "lead.html").read_text(encoding="utf-8") if (G / "lead.html").exists() else ""
    v = s["verdict"]
    c1, c2, c3 = v["c1_safety"], v["c2_regression"], v["c3_gain_214550"]
    h = ["""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>G1.0 시험 결과</title><style>
:root{--bg:#fff;--fg:#1d1d1f;--mut:#666;--ok:#0a7d32;--bad:#b3261e;--na:#8a6d00;--line:#ddd;--hd:#f4f4f6}
@media (prefers-color-scheme:dark){:root{--bg:#18181a;--fg:#eee;--mut:#aaa;--ok:#5fd38a;--bad:#ff8a80;--na:#e6c35c;--line:#333;--hd:#242428}}
body{background:var(--bg);color:var(--fg);font:15px/1.6 system-ui,sans-serif;max-width:1000px;margin:auto;padding:16px}
table{border-collapse:collapse;margin:8px 0;font-size:13px;display:block;overflow-x:auto}th,td{border:1px solid var(--line);padding:3px 7px;text-align:right}
th{background:var(--hd)}td.l,th.l{text-align:left}.ok{color:var(--ok)}.bad{color:var(--bad)}.na{color:var(--na)}.mut{color:var(--mut);font-size:13px}
h1{font-size:22px}h2{font-size:18px;margin-top:28px}</style></head><body>""",
         "<h1>G1.0 시험 결과 (자이로가 멈췄을 때만 ZUPT 문턱 1.0 px)</h1>", lead]
    h.append("<h2>미리 정한 기준 판정</h2><table><tr><th class=l>기준</th><th class=l>내용</th><th>결과</th><th class=l>숫자</th></tr>")
    h.append(f"<tr><td class=l>1 안전</td><td class=l>모든 칸 G 헛 통과 ≤ base, 새 무너짐 0</td><td>{ok(c1['ok'])}</td>"
             f"<td class=l>칸 {c1['n_cells']}, 헛 통과 늘어난 칸 {len(c1['false_pass_worse'])} {f(c1['false_pass_worse'])}, 새 무너짐 {f(c1['new_collapses'])}</td></tr>")
    h.append(f"<tr><td class=l>2 회귀</td><td class=l>문 열린 R 칸 ATE 짝 차이 중앙 ≤ +0.1 cm, 최대 악화 ≤ 시작 민감도 띠</td><td>{ok(c2['ok'])}</td>"
             f"<td class=l>칸 {c2['n_opened_scored']}, 중앙 {f(c2['median'],3)} cm, 최대 {f(c2['worst'],3)} cm, 띠 {f(c2['band'],3)} cm</td></tr>")
    h.append(f"<tr><td class=l>3 이득 (214550)</td><td class=l>출발 10 s ATE 짝 차이 중앙 &lt; 0 그리고 base 무너짐→G 정상 ≥ 2</td><td>{ok(c3['ok'])}</td>"
             f"<td class=l>칸 {c3['n_tested']}, 중앙 {f(c3['median'],3)} cm, 막은 무너짐 {len(c3['prevented'])} {f(c3['prevented'])}</td></tr></table>")
    h.append(f"<p>결론 문장 (규칙): <b>{f(v['conclusion'])}</b></p>")
    h.append(f"<p class=mut>동일성 점검: G 브리지(플래그 없음) = 진단 브리지 {f(s['checks']['gbridge_vs_diag'])}, 문이 안 열린 G = base {f(s['checks']['G_closed_vs_base'])} (추적 포즈 비트 동일).</p>")
    if ck:
        h.append(f"<p class=mut>독립 검산 (imu_learn/g10_check.py): 실행 {ck['n_compared_runs']} 개 비교, 불일치 {len(ck['mismatches'])} 개."
                 + ("".join(f"<br>{H.escape(m)}" for m in ck["mismatches"][:20])) + "</p>")
    h.append("<h2>짝 판정 (칸별 차이, 출발 칸)</h2><table><tr><th class=l>묶음</th><th class=l>비교</th><th>칸</th><th>중앙 (cm)</th><th>좋음/같음/나쁨</th><th>부호 뒤집기 p</th></tr>")
    for lab, grp in (("전체 출발 칸", s["paired"]), ("214550", s["paired_214550"]), ("214550 밖", s["paired_not_214550"])):
        for k, p in grp.items():
            h.append(f"<tr><td class=l>{lab}</td><td class=l>{k}</td><td>{p['n']}</td><td>{f(p['med'],3)}</td>"
                     f"<td>{p['good']}/{p['same']}/{p['bad']}</td><td>{f(p['p'],3)}</td></tr>")
    h.append("</table>")
    tr = s.get("takeoff_rules_G_vs_base")
    if tr:
        h.append(f"<p class=mut>참고: i3 출발 시험의 (i)(ii)(iii) 규칙을 B 칸에 G 대 base 로 적용 — (i) 새 무너짐 {f(tr['i_new'])}, "
                 f"(ii) 막은 무너짐 {f(tr['ii_prevented'])} (녹화 {f(tr['ii_recs'])}), (iii) 중앙 base {f(tr['iii_base_med'])} / G {f(tr['iii_i3_med'])} 한계 {f(tr['iii_lim'])} → {f(tr['conclusion'])}</p>")
    h.append("<h2>출발 칸 (출발 뒤 10 s 창)</h2><p class=mut>상태 · ATE (cm) · 헛 통과 수 (ZUPT 통과 & 정답 > 2 cm/s & 코너 ≥ 12). G 문 열림 = G 실행에서 문이 열린 프레임 수.</p>")
    h.append("<table><tr><th class=l>칸</th><th class=l>시험?</th><th>base</th><th>ATE</th><th>헛</th><th>i3</th><th>ATE</th><th>헛</th><th>G1.0</th><th>ATE</th><th>헛</th><th>G 문 열림</th><th>G−base</th></tr>")
    for r in s["dep_rows"]:
        g, b = r.get("G"), r.get("base")
        dd = (g["ate_cm"] - b["ate_cm"]) if (g and b and g.get("ate_cm") is not None and b.get("ate_cm") is not None) else None
        h.append(f"<tr><td class=l>{f(r['key'])}</td><td class=l>{'예' if r.get('tested') else f(r.get('why'))}</td>"
                 f"{cell(b)}{cell(r.get('i3'))}{cell(g)}<td>{f((g or {}).get('gate_open_frames'))}</td><td>{f(dd)}</td></tr>")
    h.append("</table><h2>회귀 칸 (z2_regress 묶음, 전체 실행)</h2><table><tr><th class=l>칸</th><th class=l>시험?</th><th>base</th><th>ATE</th><th>헛</th><th>G1.0</th><th>ATE</th><th>헛</th><th>G 문 열림</th><th>G−base</th></tr>")
    for r in s["r_rows"]:
        g, b = r.get("G"), r.get("base")
        dd = (g["ate_cm"] - b["ate_cm"]) if (g and b and g.get("ate_cm") is not None and b.get("ate_cm") is not None) else None
        if not r.get("gate_possible"):
            h.append(f"<tr><td class=l>{f(r['key'])}</td><td class=l colspan=9>{f(r['why'])}</td></tr>")
            continue
        h.append(f"<tr><td class=l>{f(r['key'])}</td><td class=l>{'예' if r.get('tested') else f(r.get('why'))}</td>{cell(b)}{cell(g)}"
                 f"<td>{f((g or {}).get('gate_open_frames'))}</td><td>{f(dd, 3)}</td></tr>")
    h.append("</table>")
    h.append(f"<p class=mut>만든 시각 {f(s['made'])} · git {f(s['git'])} · 계획 docs/g10_plan.md · 실행기 imu_learn/g10.py · 분석 imu_learn/g10_analyse.py · "
             "검산 imu_learn/g10_check.py · 결과 runs/g10/ · 진행 runs/g10.log</p></body></html>")
    (S / "docs" / "g10_result.html").write_text("\n".join(h), encoding="utf-8")
    print("wrote docs/g10_result.html")


if __name__ == "__main__":
    main()
