"""HTML page for the init window test (docs/init_window_result.html), from c5_initwin_analyse.summarize()."""
import html
from pathlib import Path

LAB_KO = {"success": "성공", "collapse": "무너짐", "late": "늦음", "noinit": "안 됨"}
CLS = {"success": "z", "collapse": "f", "late": "w", "noinit": "w"}
W_KO = {"W05": "0.5 s", "W10": "1.0 s", "W15": "1.5 s"}
D_KO = {"D224": "2.24 px", "D5": "5 px", "D15": "15 px"}


def fmt(v, n=2):
    if v is None:
        return "–"
    if isinstance(v, float):
        return f"{v:.{n}f}"
    return html.escape(str(v))


def sk(k):
    w, d = k.split("|")
    return f"창 {W_KO[w]} · 시차 {D_KO[d]}"


def write(res, path, concl):
    css = (":root{--bg:#fff;--fg:#1d1d1f;--mut:#666;--line:#ddd;--box:#f5f6f8;--good:#e8f6ec;--bad:#fbe9e9;--warn:#fdf5e0}"
           "@media (prefers-color-scheme:dark){:root{--bg:#16171a;--fg:#e8e8ea;--mut:#9a9aa0;--line:#33343a;--box:#202226;"
           "--good:#1d3a26;--bad:#3a1f1f;--warn:#3a321c}}"
           "body{background:var(--bg);color:var(--fg);font:15px/1.6 system-ui,sans-serif;max-width:1100px;margin:0 auto;padding:16px}"
           "table{border-collapse:collapse;margin:8px 0 16px;font-size:13px;display:block;overflow-x:auto}"
           "th,td{border:1px solid var(--line);padding:4px 6px;text-align:center}small{color:var(--mut)}"
           ".box{background:var(--box);border-radius:8px;padding:10px 14px;margin:10px 0}.concl{background:var(--good)}"
           "td.z{background:var(--good)}td.f{background:var(--bad)}td.w{background:var(--warn)}code{font-size:12px}")
    h = ["<!doctype html><html lang='ko'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
         f"<title>초기화 창 시험</title><style>{css}</style></head><body>"
         "<h1>짧은 정지에서 정적 초기화 창·시차 문턱 시험 (2026-10-21)</h1>"]
    h.append(f"<div class='box concl'>{concl}</div>" if concl else "<div class='box'>결론 작성 전 (자동 표만)</div>")
    h.append("<div class='box'><b>읽는 법</b>: <b>안 됨</b>·<b>늦음</b> = 이륙 전에 \"아직 준비 안 됨\"을 알 수 있는 안전한 실패 "
             "(늦음 = 정지가 끝난 뒤 초기화). <b>무너짐</b> = 이륙 전에 초기화됐는데 궤적이 틀림 (축척 0.5–2 밖 또는 ATE &gt; 10 cm) — 비행 중 실패. "
             "녹화 5 개·한 장소라 모든 결론은 <b>추정</b>. 정답 칸 42.72 mm (사양 기반 추정). IMU 는 원본(유형 16/35).</div>")
    h.append(f"<h2>1. 무엇을 돌렸나</h2><p>실행 {res['n_runs']} 개, 결과 없는 실행 {len(res['failed'])} 개, 코드 버전 "
             f"{'<b>섞임 (판정 무효)</b>' if res['mixed_versions'] else '하나'}, IMU {html.escape(', '.join(res['imu_sources']))}. "
             "정적 전용 초기화 (동적 끔, IMU 문턱 0.3 m/s² 그대로)에서 창 길이 W 와 시차 문턱 D 만 바꿈. 정지를 S = 1·2 s 만 남기고 자름. "
             "계획 docs/init_window_plan.md (10:40 사후 변경: 거친 단계 δ = 0 → 상위 후보만 δ ±0.25 반복 → 1 등만 IMU 2 s 앞당김).</p>")
    h.append("<p><b>초기화 시각</b>은 DEBUG 로그의 'successful initialization' 프레임 시각이다. 벤치의 <code>t_init_clip_s</code> 는 첫 'tracking' "
             "포즈 = 첫 영상 갱신 시각이라 정지 중 초기화 뒤 ZUPT 가 붙잡는 동안은 나오지 않는다 (움직임 시작 무렵). "
             "c5 재시험의 '늦은 초기화' 집계는 이 뜻이었다.</p>")
    h.append("<table><tr><th>녹화</th><th>움직임 시작 (0.1 s 창, 클립 기준 s)</th><th>정지 중 원본 자이로 평균 (mdeg/s, x y z)</th><th>평균 구간 (s)</th></tr>")
    for rec, s in sorted(res["still"].items()):
        h.append(f"<tr><td>{rec[-6:]}</td><td>{fmt(s['fine_s'])}</td><td>{' '.join(f'{v:.0f}' for v in s['bg_mdps'])}</td>"
                 f"<td>{s['win'][0]}–{s['win'][1]}</td></tr>")
    h.append("</table>")
    # coarse
    h.append("<h2>2. 거친 단계 (IMU 앞당김 0, δ = 0, 녹화 5 × S 2 = 설정마다 10 실행)</h2><table><tr><th>설정</th><th>실행</th>"
             "<th>무너짐</th><th>성공 S = 2</th><th>성공 S = 1</th><th>바이어스 오차 중앙값 (mdeg/s)</th><th>순위</th></tr>")
    for k, v in res["coarse"].items():
        rk = res["coarse_rank"].index(k) + 1 if k in res["coarse_rank"] else "–"
        h.append(f"<tr><td>{sk(k)}</td><td>{v['n']}</td><td class='{'z' if v['collapse'] == 0 else 'f'}'>{v['collapse']}</td>"
                 f"<td>{v['succ_L0_S2']}/5</td><td>{v['succ_L0_S1']}/5</td><td>{fmt(v['bg_err_med_pre'], 0)}</td><td>{rk}</td></tr>")
    h.append("</table><small>순위 = 무너짐 0 인 설정 중 ① S2 성공 ② S1 성공 ③ 바이어스 오차 작음 ④ D 작음 ⑤ W 김 (사전 등록).</small>")
    # cells
    h.append("<h2>3. 칸별 결과 (모든 단계, δ 반복 포함)</h2><table><tr><th>앞당김</th><th>S</th><th>설정</th><th>실행</th>"
             "<th>안 됨</th><th>늦음</th><th>무너짐</th><th>성공</th><th>초기화 − 정지 끝 (s, 중앙값)</th>"
             "<th>바이어스 오차 중앙값 / 최대 (mdeg/s)</th><th>성공 ATE 중앙값 (cm)</th><th>무너진 녹화</th></tr>")
    for k in sorted(res["cells"], key=lambda k: (k.split("|")[0], k.split("|")[1], k.split("|")[2], k.split("|")[3])):
        c = res["cells"][k]
        lead, S, w, d = k.split("|")
        h.append(f"<tr><td>{'0 s' if lead == 'L0' else '2 s'}</td><td>{S[1:]}</td><td>{sk(w + '|' + d)}</td><td>{c['n']}</td>"
                 f"<td>{c['noinit']}</td><td>{c['late']}</td><td class='{'z' if c['collapse'] == 0 else 'f'}'>{c['collapse']}</td>"
                 f"<td>{c['success']}</td><td>{fmt(c['t_init_minus_still_med'])}</td>"
                 f"<td>{fmt(c['bg_err_med_pre'], 0)} / {fmt(c['bg_err_max_pre'], 0)}</td><td>{fmt(c['ate_med_success'])}</td>"
                 f"<td>{html.escape(' '.join(c['collapse_recs']))}</td></tr>")
    h.append("</table>")
    h.append("<h2>4. 설정별 합계 (돌린 실행 전부)</h2><table><tr><th>설정</th><th>실행</th><th>무너짐</th><th>성공 L0 S2</th>"
             "<th>성공 L0 S1</th><th>성공 L2 S2</th><th>성공 L2 S1</th><th>바이어스 오차 중앙값</th></tr>")
    for k, v in res["settings"].items():
        h.append(f"<tr><td>{sk(k)}</td><td>{v['n']}</td><td class='{'z' if v['collapse'] == 0 else 'f'}'>{v['collapse']}</td>"
                 f"<td>{v['succ_L0_S2']}</td><td>{v['succ_L0_S1']}</td><td>{v['succ_L2_S2']}</td><td>{v['succ_L2_S1']}</td>"
                 f"<td>{fmt(v['bg_err_med_pre'], 0)}</td></tr>")
    h.append(f"</table><p>반복까지 돈 후보: {html.escape(', '.join(sk(k) for k in res['repeated']) or '없음')}. "
             f"최종 순위 (무너짐 0 후보): {html.escape(' > '.join(sk(k) for k in res['rank']) or '없음')}.</p>")
    h.append("<h2>5. 실행 하나하나</h2><table><tr><th>실행</th><th>남은 정지 (s)</th><th>초기화 (s)</th><th>판정</th>"
             "<th>초기화 시차 (옛·새 px)</th><th>바이어스 오차 (mdeg/s)</th><th>전체 축척</th><th>전체 ATE (cm)</th><th>늦은 실행 궤적</th></tr>")
    for r in sorted(res["runs"], key=lambda r: r["name"]):
        h.append(f"<tr><td>{html.escape(r['name'])}</td><td>{fmt(r['still_left_s'])}</td><td>{fmt(r['t_init_true_s'])}</td>"
                 f"<td class='{CLS[r['label']]}'>{LAB_KO[r['label']]}</td><td>{fmt(r['init_disp'][0])}, {fmt(r['init_disp'][1])}</td>"
                 f"<td>{fmt(r['bg_err_mdps'], 0)}</td><td>{fmt(r.get('scale_full'), 3)}</td><td>{fmt(r.get('ate_full_cm'))}</td>"
                 f"<td>{fmt(r.get('late_traj'))}</td></tr>")
    h.append("</table>")
    if res["failed"]:
        h.append("<h2>결과 없는 실행</h2><ul>" + "".join(f"<li>{html.escape(f['name'])}: {html.escape(f['why'])}</li>"
                                                     for f in res["failed"]) + "</ul>")
    h.append("<p><small>코드: imu_learn/c5_initwin.py (일정), c5_run.py, vio_bench_hs.py (--init-window/--init-disp), "
             "c5_initwin_analyse.py, c5_initwin_html.py, 검산 c5_initwin_check.py. 결과 runs/init_window/, summary.json, 진행 runs/next_vio.log.</small></p>"
             "</body></html>")
    Path(path).write_text("".join(h))
