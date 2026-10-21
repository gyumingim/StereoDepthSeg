#!/usr/bin/env python3
"""HTML for the set-down mitigation test (docs/setdown_mitigation.html) from runs/setdown_mitig/summary.json
(imu_learn/setdown_mitig.py analyse) + a hand-written conclusion block (HTML fragment).

  ../venv_ffs/bin/python imu_learn/setdown_mitig_html.py --concl runs/setdown_mitig/concl.html --out docs/setdown_mitigation.html
"""
import argparse
import html
import json
from pathlib import Path

COND_KO = {"base": "기준 (시차만 ZUPT)", "Z1": "Z1: ZUPT IMU 판정 켬 (chi² ×1)",
           "Z2": "Z2: Z1 + ZUPT 잡음 배수 1", "Z3": "Z3: ZUPT IMU 판정 느슨 (chi² ×5)",
           "H": "H: IMU 정지 → 프레임 붙잡기"}


def f(v, n=1):
    if v is None:
        return "–"
    if isinstance(v, bool):
        return "예" if v else "아니오"
    if isinstance(v, float):
        return f"{v:.{n}f}"
    return html.escape(str(v))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--summary", default="runs/setdown_mitig/summary.json")
    ap.add_argument("--concl", required=True)
    ap.add_argument("--out", required=True)
    o = ap.parse_args()
    s = json.load(open(o.summary))
    css = (":root{--bg:#fff;--fg:#1d1d1f;--mut:#666;--line:#ddd;--box:#f5f6f8;--good:#e8f6ec;--bad:#fbe9e9}"
           "@media (prefers-color-scheme:dark){:root:not([data-theme='light']){--bg:#16171a;--fg:#e8e8ea;--mut:#9a9aa0;"
           "--line:#33343a;--box:#202226;--good:#1d3a26;--bad:#3a1f1f}}"
           "body{background:var(--bg);color:var(--fg);font:15px/1.6 system-ui,sans-serif;max-width:1100px;margin:0 auto;padding:16px}"
           "table{border-collapse:collapse;margin:8px 0 16px;font-size:13px;display:block;overflow-x:auto}"
           "th,td{border:1px solid var(--line);padding:4px 6px;text-align:center}small{color:var(--mut)}"
           ".box{background:var(--box);border-radius:8px;padding:10px 14px;margin:10px 0}.concl{background:var(--good)}"
           "td.g{background:var(--good)}td.b{background:var(--bad)}code{font-size:12px}")
    h = [f"<!doctype html><html lang='ko'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
         f"<title>내려놓기 대책 시험</title><style>{css}</style></head><body>"
         "<h1>내려놓기 실패 대책 시험 (2026-10-21)</h1>", Path(o.concl).read_text()]
    h.append("<h2>1. 녹화·조건별 결과 (시작 위치 3 개의 중앙값)</h2><table><tr><th>녹화</th><th>조건</th><th>초기화</th>"
             "<th>마지막 정답 창 오차 중앙값 / 최대 (cm)</th><th>끝 정지 VIO 이동 (cm)</th><th>끝 정지 속도 평균 / 최대 (cm/s)</th>"
             "<th>전체 ATE (cm)</th><th>첫 100 s ATE (cm)</th><th>첫 100 s 축척</th><th>100 s 뒤 ZUPT 통과율</th></tr>")
    for k, c in s["cells"].items():
        rec, cond = k.split("|")
        h.append(f"<tr><td>{rec}</td><td>{COND_KO[cond]}</td><td>{c['n_init']}/{c['n']}</td>"
                 f"<td>{f(c['lastwin_med_cm'])} / {f(c['lastwin_max_cm'])}</td><td>{f(c['drift_cm'])}</td>"
                 f"<td>{f(c['v_mean_cms'])} / {f(c['v_max_cms'])}</td><td>{f(c['bench_ate_cm'], 2)}</td>"
                 f"<td>{f(c['ate100_cm'], 2)}</td><td>{f(c['scale100'], 3)}</td><td>{f(c['zupt_frac_after_100'], 2)}</td></tr>")
    h.append("</table><small>마지막 정답 창 = 정답이 있는 마지막 3 s. 오차는 t ≤ 100 s 구간으로만 SE3 맞춘 뒤의 위치 오차 (맞춤에 끝부분이 안 섞임). "
             "끝 정지 창 = runs/endstill_tmp/endstill_check.json (224323 135.7–137.8 s, 223815 133.7–140.5 s).</small>")
    h.append("<h2>2. 사전 판정 (docs/setdown_mitigation_plan.md)</h2><table><tr><th>조건</th><th>오차 ≤ 기준 50 %</th>"
             "<th>끝 정지 이동 ≤ 기준 50 %</th><th>도움</th><th>회귀 없음</th></tr>")
    for cond, v in s["verdict"].items():
        h.append(f"<tr><td>{COND_KO[cond]}</td><td>{f(v['helps_err'])}</td><td>{f(v['helps_drift'])}</td>"
                 f"<td class='{'g' if v['helps'] else 'b'}'>{f(v['helps'])}</td>"
                 f"<td class='{'g' if v['no_regression'] else 'b'}'>{f(v['no_regression'])}</td></tr>")
    h.append(f"</table><p>기준이 실패를 재현했나 (224323 마지막 창 오차 중앙값 ≥ 10 cm): <b>{f(s['reproduced'])}</b>. "
             f"사전 규칙의 권고: <b>{f(s['recommend'])}</b>.</p>")
    h.append("<h2>3. 감시 지표 (막지 않고 알리기만)</h2><table><tr><th>실행</th><th>F5 첫 경보 (100 s 뒤, s)</th>"
             "<th>그때 오차 (cm)</th><th>F5 헛경보 (≤ 100 s)</th><th>F30 첫 경보 (s)</th><th>F30 헛경보 (≤ 100 s)</th></tr>")
    for r in s["runs"]:
        g5, g30 = r.get("gate5") or {}, r.get("gate30") or {}
        h.append(f"<tr><td>{html.escape(r['name'])}</td><td>{f(g5.get('first_after_100'), 2)}</td><td>{f(g5.get('err_at_cm'))}</td>"
                 f"<td>{g5.get('n_le_100', '–')}</td><td>{f(g30.get('first_after_100'), 2)}</td><td>{g30.get('n_le_100', '–')}</td></tr>")
    h.append("</table><small>F_N = 초기화 뒤 (ZUPT 통과 안 됨) 그리고 (MSCKF+SLAM 갱신 특징 0) 이 N 프레임 연속 (60 fps: N 5 ≈ 0.08 s, 30 = 0.5 s).</small>")
    h.append("<h3>충격 표시 (|a|−g &gt; 15 m/s² 뒤 3 s 안에 IMU 정지)</h3><table><tr><th>녹화</th><th>표시 (충격 s → 정지 시작 s, 봉우리 m/s²)</th><th>IMU 정지 구간 (s)</th></tr>")
    for rec, v in s["impact"].items():
        fl = ", ".join(f"{x['t_impact']} → {x['t_still']} ({x['peak']})" for x in v["flags"]) or "없음"
        segs = ", ".join(f"{a}–{b}" for a, b in v["still_segments"])
        h.append(f"<tr><td>{rec}</td><td>{fl}</td><td>{segs}</td></tr>")
    h.append("</table>")
    h.append("<h2>4. 실행 하나하나</h2><table><tr><th>실행</th><th>초기화 (s)</th><th>마지막 창 오차 중앙값 / 최대</th><th>끝 정지 이동</th>"
             "<th>끝 정지 속도 최대</th><th>전체 ATE</th><th>100 s ATE / 축척</th><th>붙잡은 프레임</th><th>붙잡은 구간 (s)</th></tr>")
    for r in s["runs"]:
        hs = ", ".join(f"{a}–{b}" for a, b in (r.get("hold_segments_s") or [])) or "–"
        h.append(f"<tr><td>{html.escape(r['name'])}</td><td>{f(r.get('t_init_s'), 2)}</td>"
                 f"<td>{f(r.get('lastwin_med_cm'))} / {f(r.get('lastwin_max_cm'))}</td><td>{f(r.get('drift_cm'))}</td>"
                 f"<td>{f(r.get('v_max_cms'))}</td><td>{f(r.get('bench_ate_cm'), 2)}</td>"
                 f"<td>{f(r.get('ate100_cm'), 2)} / {f(r.get('scale100'), 3)}</td><td>{f(r.get('frames_held'))}</td><td>{hs}</td></tr>")
    h.append("</table>")
    h.append(f"<p><small>실행 {s['n_runs']} 개, 결과 없음 {len(s['failed'])} 개, 코드 버전 {s['code_versions']} 개 (git {s['git']}). "
             "코드: imu_learn/setdown_mitig.py (실행·분석), vio_bench_hs.py (--cfg-over, --still-hold), c5_run.py (진단 브리지 + DEBUG 로그), "
             "setdown_mitig_html.py. 결과 runs/setdown_mitig/ (실행마다 JSON·포즈·로그, summary.json). 계획 docs/setdown_mitigation_plan.md.</small></p>"
             "</body></html>")
    Path(o.out).write_text("".join(h))


if __name__ == "__main__":
    main()
