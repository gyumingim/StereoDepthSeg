#!/usr/bin/env python3
"""docs/c5_test_result.html from runs/c5/summary.json (c5_analyse.py) + a notes JSON (conclusion text written after
reading the numbers: keys conclusion, interpretation, limits — HTML strings).

  ../venv_ffs/bin/python imu_learn/c5_html.py runs/c5/summary.json runs/c5/notes.json docs/c5_test_result.html
"""
import html
import json
import sys

NAMES = {"cfg_loose": "(c) 느슨 (옛 벤치)", "cfg_strict": "(a) 엄격·정지만 (앱)", "cfg_strictdyn": "(d) 엄격 정지 + 동적",
         "cfg_strictdyn30": "(e) (d) + clone 30"}
ORDER = ["cfg_loose", "cfg_strictdyn", "cfg_strictdyn30", "cfg_strict"]
PREV_COL = ["115620", "132803", "145816", "151358", "152631", "161035", "163345"]
PREV_OK = ["114000", "125714", "131326", "154320"]
VERD = {"gone": "사라짐", "remains": "남음", "untested": "시험 안 됨"}


def f(v, n=2):
    if v is None:
        return "–"
    if isinstance(v, float):
        return f"{v:.{n}f}"
    return html.escape(str(v))


def main():
    s = json.load(open(sys.argv[1]))
    notes = json.load(open(sys.argv[2]))
    st = s["settings"]
    runs = {r["name"]: r for r in s["runs"]}
    css = """:root{--bg:#fcfcfb;--fg:#111;--mut:#5a5955;--panel:#f2f1ed;--line:#dedcd5;--bad:#fbe3e0;--good:#e3f3e5;--mid:#fff3d1;--acc:#2a6fd6}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#1a1a19;--fg:#f0efec;--mut:#bdbcb2;--panel:#252523;--line:#3a3936;--bad:#40231f;--good:#1f3524;--mid:#3a3420;--acc:#6ea4f0}}
:root[data-theme="dark"]{--bg:#1a1a19;--fg:#f0efec;--mut:#bdbcb2;--panel:#252523;--line:#3a3936;--bad:#40231f;--good:#1f3524;--mid:#3a3420;--acc:#6ea4f0}
*{box-sizing:border-box}body{background:var(--bg);color:var(--fg);font:16px/1.7 system-ui,-apple-system,"Malgun Gothic",sans-serif;margin:0 auto;padding:16px;max-width:980px}
h1{font-size:22px;margin:4px 0 6px}h2{font-size:18px;margin:28px 0 8px;border-bottom:1px solid var(--line);padding-bottom:4px}
.mut{color:var(--mut);font-size:14px}.box{background:var(--panel);border-radius:10px;padding:12px 14px;border-left:5px solid var(--acc)}
.wrap{overflow-x:auto}table{border-collapse:collapse;width:100%;font-size:14px}th,td{border-bottom:1px solid var(--line);padding:5px 6px;text-align:left;vertical-align:top;white-space:nowrap}
.bad{background:var(--bad)}.good{background:var(--good)}.mid{background:var(--mid)}code{font-size:13px;background:var(--panel);padding:0 4px;border-radius:4px;overflow-wrap:anywhere}"""
    o = [f"<!doctype html><html lang='ko'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
         f"<title>C5 시험 결과</title><style>{css}</style></head><body>",
         "<h1>C5 시험 결과: 붕괴의 주원인이 느슨한 초기화인가</h1>",
         "<p class='mut'>2026-10-20 · 계획 docs/c5_test_plan.md (실행 전 커밋, 변경 기록 포함) · 코드 imu_learn/c5_run.py, c5_analyse.py, "
         "vio_bench_hs.py · 결과 runs/c5/</p>",
         f"<div class='box'>{notes['conclusion']}</div>"]
    # setting summary
    o.append("<h2>설정별 요약</h2><div class='wrap'><table><tr><th>설정</th><th>실행</th><th>초기화</th><th>붕괴 / 이름표</th>"
             "<th>이전 붕괴 7 개 중 사라짐</th><th>시험 안 됨</th><th>정상 4 개 중 망가짐</th><th>초기화까지 (중앙)</th>"
             "<th>덮은 비율 (중앙)</th><th>초기 |bg| &gt; 0.5 °/s</th></tr>")
    for k in ORDER:
        t = st.get(k)
        if not t:
            continue
        o.append(f"<tr><td>{NAMES.get(k, k)}</td><td>{t['runs_total']}</td><td>{t['runs_init']}</td>"
                 f"<td>{t['runs_collapsed']} / {t['runs_labelled']}</td><td>{t['n_gone']} {f(', '.join(t['prev_collapsed_gone']))}</td>"
                 f"<td>{t['n_untested']}</td><td>{f(', '.join(t['healthy_broken'])) or '없음'}</td><td>{f(t['t_init_med_s'])} s</td>"
                 f"<td>{f(t['covered_med'])}</td><td>{t['bg_over_0p5']}</td></tr>")
    o.append("</table></div>")
    o.append(f"<div>{notes.get('interpretation', '')}</div>")
    # per recording
    o.append("<h2>녹화별 결과 (시작 0 s / 1 s)</h2><p class='mut'>칸 = 초기화 방식·시각(s)·초기 |bg|(°/s) · 10 s 창 스케일 / ATE(cm) "
             "· 전체 ATE(cm). 붉은 칸 = 붕괴 (스케일 &lt;0.5·&gt;2 또는 10 s ATE &gt;10 cm), 초록 = 정상, 노랑 = 이름표 없음.</p>")
    o.append("<div class='wrap'><table><tr><th>녹화</th><th>이전</th>" + "".join(f"<th>{NAMES[k]}</th>" for k in ORDER[:3]) + "</tr>")
    for rec in PREV_COL + PREV_OK:
        row = [f"<td>{rec}</td><td>{'붕괴' if rec in PREV_COL else '정상'}</td>"]
        for k in ORDER[:3]:
            cells = []
            for off in (0, 1):
                r = runs.get(f"{rec}_{k}_o{off}")
                if not r:
                    cells.append("<div class='mid'>없음</div>")
                    continue
                if not r.get("initialized"):
                    cells.append("<div class='mid'>초기화 안 됨</div>")
                    continue
                c = r.get("collapsed")
                cls = "bad" if c else ("good" if c is False else "mid")
                cells.append(f"<div class='{cls}'>{f(r.get('init_method'))} {f(r.get('t_init_clip_s'), 1)}s "
                             f"bg {f(r.get('bg_init_dps'))} · {f(r.get('scale10'))} / {f(r.get('ate10_cm'), 1)} · {f(r.get('ate_cm'), 1)}</div>")
            v = st.get(k, {}).get("recs", {}).get(rec, {}).get("verdict")
            row.append(f"<td>{''.join(cells)}<span class='mut'>{VERD.get(v, '') if rec in PREV_COL else ''}</span></td>")
        o.append("<tr>" + "".join(row) + "</tr>")
    o.append("</table></div>")
    # (a)
    a = [r for n, r in runs.items() if "_cfg_strict_o" in n]
    if a:
        o.append("<h2>(a) 엄격·정지만 (앱 설정 그대로) 확인 실행</h2><ul>")
        for r in a:
            o.append(f"<li>{r['name']}: {'초기화 ' + f(r.get('t_init_clip_s')) + ' s' if r.get('initialized') else '60 s 동안 초기화 안 됨'}</li>")
        o.append("</ul>")
    # determinism
    if s.get("determinism"):
        o.append("<h2>같은 실행 반복 (부하 다름)</h2><div class='wrap'><table><tr><th>실행</th><th>loadavg (처음 / 반복)</th><th>초기화 시각</th>"
                 "<th>방식</th><th>10 s 스케일</th><th>전체 ATE cm</th><th>붕괴</th></tr>")
        for d in s["determinism"]:
            o.append(f"<tr><td>{d['name']}</td><td>{f(d['load'][0][0] if d['load'][0] else None)} / {f(d['load'][1][0] if d['load'][1] else None)}</td>"
                     f"<td>{f(d['t_init'][0])} / {f(d['t_init'][1])}</td><td>{f(d['method'][0])} / {f(d['method'][1])}</td>"
                     f"<td>{f(d['scale10'][0], 3)} / {f(d['scale10'][1], 3)}</td><td>{f(d['ate_cm'][0])} / {f(d['ate_cm'][1])}</td>"
                     f"<td>{f(d['collapsed'][0])} / {f(d['collapsed'][1])}</td></tr>")
        o.append("</table></div>")
    # versions / failures
    fails = [x for x in s.get("failed", []) if x["why"] != "no result json: not run yet"]
    o.append("<h2>버전·실패</h2><ul>")
    o.append(f"<li>코드·설정·실행 파일 버전 종류: {len(s.get('versions', []))} (섞임: {'예' if s.get('mixed_versions') else '아니오'}). "
             f"벤치 blob {f(', '.join(s.get('bench_blobs', [])))}</li>")
    if s.get("versions"):
        o.append(f"<li class='mut'><code>{html.escape(s['versions'][0])}</code></li>")
    o.append(f"<li>실패한 실행: {len(fails)} 개" + ("" if not fails else ": " + ", ".join(f"{x['name']} ({html.escape(x['why'][:80])})" for x in fails)) + "</li></ul>")
    o.append(f"<h2>한계</h2>{notes.get('limits', '')}")
    o.append("</body></html>")
    open(sys.argv[3], "w").write("\n".join(o) + "\n")


if __name__ == "__main__":
    main()
