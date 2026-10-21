#!/usr/bin/env python3
"""docs/z2_regression.html from runs/z2_regress/summary.json (imu_learn/z2_regress.py analyse) + a self-check that
recomputes the main table straight from the per-run JSON files (bench ate_cm/scale, poses) without analyse_one.

  ../venv_ffs/bin/python imu_learn/z2_regress_html.py [--notes NOTES.html]
"""
import argparse
import gzip
import html
import json
import os
from pathlib import Path

import sys

import numpy as np

HERE = Path(__file__).resolve().parent
os.chdir(HERE.parent)
sys.path.insert(0, str(HERE))
OUT = Path("runs/z2_regress")
DOC = Path("docs/z2_regression.html")
CSS = ("<style>:root{--bg:#fff;--fg:#1d1d1f;--mut:#666;--line:#ddd;--box:#f5f6f8;--good:#e8f6ec;--bad:#fbe9e9;--warn:#fff4d6}"
       "@media (prefers-color-scheme:dark){:root:not([data-theme='light']){--bg:#16171a;--fg:#e8e8ea;--mut:#9a9aa0;--line:#33343a;"
       "--box:#202226;--good:#1d3a26;--bad:#3a1f1f;--warn:#3a321a}}"
       ":root[data-theme='dark']{--bg:#16171a;--fg:#e8e8ea;--mut:#9a9aa0;--line:#33343a;--box:#202226;--good:#1d3a26;--bad:#3a1f1f;--warn:#3a321a}"
       "body{background:var(--bg);color:var(--fg);font:15px/1.6 system-ui,sans-serif;max-width:1100px;margin:0 auto;padding:16px}"
       "table{border-collapse:collapse;margin:8px 0 16px;font-size:13px;display:block;overflow-x:auto}"
       "th,td{border:1px solid var(--line);padding:4px 6px;text-align:center}small{color:var(--mut)}"
       ".box{background:var(--box);border-radius:8px;padding:10px 14px;margin:10px 0}.concl{background:var(--good)}"
       ".warnbox{background:var(--warn)}td.g{background:var(--good)}td.b{background:var(--bad)}td.w{background:var(--warn)}"
       "code{font-size:12px}</style>")
ST = {"ok": "성공", "collapse": "무너짐", "noinit": "초기화 안 됨", "unscored": "미판정"}


def f(x, d=2):
    return "–" if x is None else f"{x:.{d}f}"


def selfcheck(S):
    """Recompute status and median ATE per group/cond from the raw run JSON + poses (independent of analyse_one)."""
    import z2_regress as z
    bad, n = [], 0
    for g, v in S["groups"].items():
        for c in ("base", "Z2"):
            names = sorted(j["name"] for j in z.jobs() if j["group"] == g and j["cond"] == c)
            st, at = [], []
            for nm in names:
                p = OUT / nm
                if not Path(str(p) + ".json").exists():
                    continue
                d = json.load(open(str(p) + ".json"))
                trk = any(json.loads(x).get("status") == "tracking" for x in gzip.open(str(p) + ".poses.jsonl.gz", "rt"))
                if not trk:
                    s = "noinit"
                elif d.get("ate_cm") is None or d.get("scale") is None:
                    s = "unscored"
                else:
                    s = "ok" if (0.5 <= d["scale"] <= 2 and d["ate_cm"] <= 10) else "collapse"
                    at.append(d["ate_cm"])
                st.append(s)
                zc = d.get("config_used", {})
                want = ("1", "1") if c == "Z2" else ("0", "10")
                if (zc.get("zupt_chi2_multipler"), zc.get("zupt_noise_multiplier")) != want:
                    bad.append(f"{nm}: zupt cfg {zc.get('zupt_chi2_multipler')}/{zc.get('zupt_noise_multiplier')}")
            med = float(np.median(at)) if at else None
            cell = v["cells"][c]
            n += 1
            if st != cell["status"]:
                bad.append(f"{g} {c}: status {st} vs summary {cell['status']}")
            if (med is None) != (cell["ate_med"] is None) or (med is not None and abs(med - cell["ate_med"]) > 1e-9):
                bad.append(f"{g} {c}: ATE median {med} vs summary {cell['ate_med']}")
    return n, bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--notes", default=None, help="HTML fragment with the hand-written conclusion")
    o = ap.parse_args()
    S = json.load(open(OUT / "summary.json"))
    nchk, bad = selfcheck(S)
    G = S["groups"]
    rows = []
    for g, v in G.items():
        b, z = v["cells"]["base"], v["cells"]["Z2"]
        kind = {"F": "전체", "T": "짧은 정지 60 s", "O": "옛 녹화 (Android IMU)"}[g[0]]

        def sts(c):
            return " / ".join(ST[s] for s in c["status"])
        ok_ate = v["ate_ok"]
        rows.append(
            f"<tr><td>{g[2:]}</td><td>{kind}</td><td>{sts(b)}</td><td>{f(b['ate_med'])}</td><td>{sts(z)}</td>"
            f"<td>{f(z['ate_med'])}</td><td>{f(v['ate_limit'])}</td>"
            f"<td class='{'g' if ok_ate else ('b' if ok_ate is False else 'w')}'>{'예' if ok_ate else ('아니오' if ok_ate is False else '비교 불가')}</td>"
            f"<td class='{'b' if v['new_collapses'] else 'g'}'>{len(v['new_collapses'])}</td>"
            f"<td class='{'b' if z['false_acc'] else 'g'}'>{' / '.join(str(x) for x in z['false_acc_runs'])}</td>"
            f"<td>{z['imu_only']} / {z['imu_only_unknown']} / {z['unknown_gyro_moving']}</td>"
            f"<td>{b['zupt_moving_any']} / {z['zupt_moving_any']}</td>"
            f"<td class='{'g' if v['pass_'] else 'b'}'><b>{'통과' if v['pass_'] else '실패'}</b></td></tr>")
    runs = []
    for r in sorted(S["runs"], key=lambda r: (r["group"], r["pair"], r["cond"])):
        runs.append(f"<tr><td>{r['name']}</td><td>{f(r['start_off'], 3)}</td><td>{f(r['t_init_clip_s'])}</td>"
                    f"<td>{ST[r['status']]}</td><td>{f(r['ate_cm'])}</td><td>{f(r['scale'], 3)}</td>"
                    f"<td>{r['zupt_acc']}</td><td>{r['zupt_imu_only']}</td><td>{r['false_acc']}"
                    f"{(' (' + ', '.join(map(str, r['false_acc_v_cms'][:5])) + ' cm/s)') if r['false_acc'] else ''}</td>"
                    f"<td>{r['imu_only_gt_unknown']}</td><td>{'복사' if r['reused'] else ''}</td></tr>")
    fails = [(g, v) for g, v in G.items() if not v["pass_"]]
    auto = (f"<p>자동 판정: 묶음 {len(G)} 개 중 <b>통과 {len(G) - len(fails)}</b>, 실패 {len(fails)}. 실행 {S['n_runs']} 개, 빠진 실행 "
            f"{len(S['missing'])}, 코드 버전 {S['code_versions']} 개.</p>")
    if fails:
        auto += "<ul>" + "".join(
            f"<li>{g}: " + "; ".join(x for x in (
                None if v["ate_ok"] is not False else f"ATE 중앙값 {f(v['cells']['Z2']['ate_med'])} > 한계 {f(v['ate_limit'])} cm",
                ("새 무너짐 " + ", ".join(v["new_collapses"])) if v["new_collapses"] else None,
                (f"헛 통과 {v['cells']['Z2']['false_acc_runs']}") if v["cells"]["Z2"]["false_acc"] else None) if x)
            + "</li>" for g, v in fails) + "</ul>"
    notes = Path(o.notes).read_text() if o.notes else ""
    chk = (f"<p>원시 JSON·포즈 파일에서 상태와 ATE 중앙값을 다시 계산해 요약과 대조: 칸 {nchk} 개, 불일치 {len(bad)} 개."
           f"{' ZUPT 설정값(base 0/10, Z2 1/1)도 실행마다 확인.' if not bad else ''}</p>"
           + ("<ul>" + "".join(f"<li>{html.escape(b)}</li>" for b in bad) + "</ul>" if bad else ""))
    page = (f"<!doctype html><html lang='ko'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
            f"<title>Z2 회귀 점검</title>{CSS}</head><body><h1>Z2 회귀 점검 (2026-10-21)</h1>"
            f"<div class='box concl'><h2 style='margin-top:0'>결론 먼저 (모두 추정)</h2>{notes}{auto}</div>"
            "<h2>무엇을 비교했나</h2><div class='box'><p><b>base</b> = 지금 설정 (ZUPT 가 영상 시차만 봄, chi² 배수 0, 잡음 배수 10). "
            "<b>Z2</b> = <code>zupt_chi2_multipler: 1</code> + <code>zupt_noise_multiplier: 1</code> (시차가 실패해도 IMU 판정이 통과하면 정지 갱신). "
            "나머지는 같음: 정적 전용 초기화 (창 1.0 s, 시차 15 px, 동적 끔), clones 11, 640x360 @60, 잡음 cfg. "
            "사전 등록 docs/z2_regression_plan.md (실행 전 커밋).</p>"
            "<p>판정 (요청 그대로): ① Z2 ATE 중앙값 ≤ max(base×1.10, base+0.2 cm) ② 새 무너짐 0 ③ Z2 실행마다 <b>헛 통과</b> 0 — "
            "헛 통과 = 시차 검사는 실패했는데 IMU 판정만으로 ZUPT 가 통과한 프레임 중 정답 속도 > 2 cm/s (정답 위치 ±0.1 s 차분).</p></div>"
            "<h2>묶음별 결과</h2><table><tr><th>묶음</th><th>종류</th><th>base 상태 (3 회)</th><th>base ATE 중앙 cm</th>"
            "<th>Z2 상태</th><th>Z2 ATE 중앙 cm</th><th>한계 cm</th><th>① ATE</th><th>② 새 무너짐</th><th>③ 헛 통과 (실행별)</th>"
            "<th>Z2 IMU 통과 / 정답 없음 / 그중 자이로 움직임</th><th>정답 >2 cm/s 인 ZUPT 전체 base / Z2</th><th>판정</th></tr>"
            + "".join(rows) + "</table>"
            "<p><small>F = 새 녹화 전체 길이 (시작 위치 0/1/2 s). T = C5 짧은 정지 (정지 S 초만 남기고 자름, δ −0.25/0/+0.25 s, 60 s, IMU 앞당김 0). "
            "O = 옛 모니터 녹화 (원본 IMU 가 없어 Android 보정판). 마지막에서 두 번째 칸은 정답 잡음 바닥을 보이려고 넣음: base 는 IMU 판정이 꺼져 있어 "
            "그 수는 시차로 통과한 프레임 = 실제로 거의 정지한 프레임인데도 정답 속도가 2 cm/s 를 넘은 수.</small></p>"
            f"<h2>스스로 점검</h2><div class='box'>{chk}</div>"
            "<details><summary>실행별 표</summary><table><tr><th>실행</th><th>시작 s</th><th>초기화 s</th><th>상태</th><th>ATE cm</th>"
            "<th>축척</th><th>ZUPT 통과</th><th>IMU 만으로</th><th>헛 통과 (정답 속도)</th><th>정답 없음</th><th>비고</th></tr>"
            + "".join(runs) + "</table></details>"
            f"<p><small>만든 시각 {S['made']}, git {S['git']}, 코드 버전 {S['code_versions']} 개. 결과 runs/z2_regress/summary.json. "
            "'복사' = runs/setdown_mitig 의 같은 인자 실행을 복사 (재실행이 포즈까지 같음을 확인).</small></p></body></html>")
    DOC.write_text(page)
    print(f"wrote {DOC}; selfcheck cells {nchk} mismatches {len(bad)}")
    for b in bad:
        print(" ", b)


if __name__ == "__main__":
    main()
