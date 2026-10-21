#!/usr/bin/env python3
"""docs/z2_isolate.html from runs/z2_isolate/summary.json (imu_learn/z2_isolate.py analyse) + a self-check that
recomputes status / median ATE per group x condition and the ZUPT config of every run straight from the raw run JSON
and pose files (no analyse_one).

  ../venv_ffs/bin/python imu_learn/z2_isolate_html.py [--notes NOTES.html]
"""
import argparse
import gzip
import html
import json
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
os.chdir(HERE.parent)
sys.path.insert(0, str(HERE))
import z2_regress_html as zh  # noqa: E402  (same CSS / formatting)

OUT = Path("runs/z2_isolate")
DOC = Path("docs/z2_isolate.html")
ST = zh.ST
f = zh.f
DESC = {"base": "지금 설정 (chi² 0 = 시차만, 잡음 ×10)", "Z2": "chi² 1 + 잡음 1", "Z1": "chi² 1 만",
        "N1": "잡음 1 만", "H": "바깥 IMU 정지 붙잡기", "Z2c": "chi² 0.5 + 잡음 1", "Z2v": "Z2 + 속도 문턱 0.05"}
WANT = {"base": ("0", "10", "0.1"), "Z2": ("1", "1", "0.1"), "Z1": ("1", "10", "0.1"), "N1": ("0", "1", "0.1"),
        "H": ("0", "10", "0.1"), "Z2c": ("0.5", "1", "0.1"), "Z2v": ("1", "1", "0.05")}


def selfcheck(S):
    bad, n = [], 0
    for g, v in S["groups"].items():
        for c, cell in v["cells"].items():
            st, at = [], []
            for pr in cell["pairs"]:
                nm = pr.replace(f"{g}_", f"{g}_{c}_") if pr.startswith(g + "_") else None
                p = OUT / nm
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
                got = (zc.get("zupt_chi2_multipler"), zc.get("zupt_noise_multiplier"), zc.get("zupt_max_velocity"))
                if got != WANT[c]:
                    bad.append(f"{nm}: zupt cfg {got} != {WANT[c]}")
                hold = d.get("still_hold")
                if (c == "H") != bool(hold):
                    bad.append(f"{nm}: still_hold {hold}")
            med = float(np.median(at)) if at else None
            n += 1
            if st != cell["status"]:
                bad.append(f"{g} {c}: status {st} vs summary {cell['status']}")
            if (med is None) != (cell["ate_med"] is None) or (med is not None and abs(med - cell["ate_med"]) > 1e-9):
                bad.append(f"{g} {c}: ATE median {med} vs summary {cell['ate_med']}")
    return n, bad


def sts(c):
    return " / ".join(ST[s] for s in c["status"])


def lst(xs, d=1):
    return " / ".join(f(x, d) for x in xs)


def group_rows(S, groups, conds):
    rows = []
    for g in groups:
        v = S["groups"][g]
        for c in conds:
            if c not in v["cells"]:
                continue
            x = v["cells"][c]
            jd = v["judged"].get(c)
            if jd is None:
                tail = "<td colspan='5'>기준</td>"
            else:
                a = jd["ate_ok"]
                wp = jd.get("worst_pair")
                tail = (f"<td class='{'g' if a else ('b' if a is False else 'w')}'>{'예' if a else ('아니오' if a is False else '비교 불가')}</td>"
                        f"<td class='{'b' if jd['new_collapses'] else 'g'}'>{len(jd['new_collapses'])}</td>"
                        f"<td class='{'b' if jd['false_acc'] else 'g'}'>{' / '.join(map(str, x['false_acc_runs']))}</td>"
                        f"<td>{'' if not wp else f'{wp[1]:+.2f} ({wp[0][-2:]})'}"
                        f"{' · 한도 넘은 짝 ' + ', '.join(p[-2:] for p in jd['pair_worse_than_limit']) if jd['pair_worse_than_limit'] else ''}</td>"
                        f"<td class='{'g' if jd['pass_'] else 'b'}'><b>{'통과' if jd['pass_'] else '실패'}</b></td>")
            rows.append(f"<tr><td>{g}</td><td>{c}</td><td>{sts(x)}</td><td>{lst(x['ates'], 2)}</td><td>{f(x['ate_med'])}</td>"
                        f"<td>{x['zupt_moving_any']}</td>{tail}</tr>")
    return ("<table><tr><th>묶음</th><th>조건</th><th>상태 (o0/o1/o2 또는 δ)</th><th>ATE cm</th><th>ATE 중앙</th>"
            "<th>정답 >2 cm/s 인 ZUPT 전체 (보조)</th><th>① ATE</th><th>② 새 무너짐</th><th>③ 헛 통과 (실행별)</th>"
            "<th>짝별 최악 (조건−base, cm)</th><th>판정</th></tr>" + "".join(rows) + "</table>")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--notes", default=None)
    o = ap.parse_args()
    S = json.load(open(OUT / "summary.json"))
    nchk, bad = selfcheck(S)
    import z2_isolate as zi
    conds = list(zi.CONDS)
    P1 = [g for g in zi.P1_GROUPS if g in S["groups"]]
    P2 = [g for g in S["groups"] if g not in zi.P1_GROUPS]
    # cause table: the 2 collapse pairs, every condition, with timing
    R = {r["name"]: r for r in S["runs"]}
    cause = []
    for g, pr in (("F_221052", "o1"), ("O_131326", "o0"), ("F_221052", "o0"), ("O_131326", "o1")):
        for c in conds:
            r = R.get(f"{g}_{c}_{pr}")
            if not r:
                continue
            cause.append(f"<tr><td>{g} {pr}</td><td>{c}</td><td>{DESC[c]}</td>"
                         f"<td class='{'g' if r['status'] == 'ok' else 'b'}'>{ST[r['status']]}</td><td>{f(r['ate_cm'])}</td>"
                         f"<td>{f(r['scale'], 3)}</td><td>{f(r.get('t_init_log_s'))}</td><td>{f(r.get('t_first_vis_update_s'))}</td>"
                         f"<td>{f(r.get('t_init_clip_s'))}</td><td>{r.get('zupt_init_to_vis')}</td><td>{r['zupt_imu_only']}</td>"
                         f"<td>{r['zupt_acc']}</td></tr>")
    sd = []
    for g in ("F_224323", "F_223815"):
        if g not in S["groups"]:
            continue
        for c, x in S["groups"][g]["cells"].items():
            hp = S["helps"].get(c) if g == "F_224323" else None
            sd.append(f"<tr><td>{g}</td><td>{c}</td><td>{f(x['lastwin_med'])}</td><td>{f(x['drift_med'])}</td>"
                      f"<td>{f(x['ate_med'])}</td><td>{'' if hp is None else ('예' if hp else '아니오')}</td></tr>")
    rank = "".join(f"<tr><td>{c}</td><td>{DESC[c]}</td><td>{'예' if p['helps'] else '아니오'}</td><td>{p['n_fail']}</td>"
                   f"<td>{', '.join(p['fails'])}</td><td>{p['n_new']}</td><td>{p['n_fa']}</td><td>{f(p['lastwin_224323'])}</td></tr>"
                   for c, p in sorted(S["phase1"].items(), key=lambda kv: (not kv[1]["helps"], kv[1]["n_fail"], kv[1]["n_new"],
                                                                         kv[1]["n_fa"])))
    det = S.get("det")
    dtxt = "아직 없음"
    if det:
        dtxt = (f"기준 {html.escape(det['ref']['name'])} ATE {det['ref']['ate']} cm. " + "; ".join(
            f"{k}: ATE {v['ate']} cm, 축척 {v['scale']}, 공통 포즈 {v['n_common']}/{v['n_a']}, 최대 위치 차 {v['max_pos_diff_m']} m"
            for k, v in det.items() if k != "ref"))
    full = S.get("full")
    ftxt = "2 단계 아직 없음."
    if full:
        ftxt = (f"<p>조건 <b>{full['cond']}</b>, 묶음 {full['n_groups']} 개: 실패 {len(full['fails'])} "
                f"({', '.join(full['fails']) or '없음'}). ① 어김 {full['crit1'] or '없음'}, ② 어김 {full['crit2'] or '없음'}, "
                f"③ 어김 {full['crit3'] or '없음'}.</p>")
    notes = Path(o.notes).read_text() if o.notes else ""
    chk = (f"<p>원시 JSON·포즈 파일에서 상태·ATE 중앙값을 다시 계산하고, 실행마다 실제 yaml 의 ZUPT 값(chi²·잡음·속도 문턱)과 "
           f"still-hold 여부를 조건과 대조: 칸 {nchk} 개, 불일치 {len(bad)} 개. 코드 버전 {S['code_versions']} 개 "
           f"(실행 수 {S['version_counts']}), 빠진 실행 {len(S['missing'])}.</p>"
           + ("<ul>" + "".join(f"<li>{html.escape(b)}</li>" for b in bad) + "</ul>" if bad else ""))
    runs = "".join(
        f"<tr><td>{r['name']}</td><td>{ST[r['status']]}</td><td>{f(r['ate_cm'])}</td><td>{f(r['scale'], 3)}</td>"
        f"<td>{f(r.get('t_init_log_s'))}</td><td>{f(r.get('t_first_vis_update_s'))}</td><td>{f(r.get('t_init_clip_s'))}</td>"
        f"<td>{r['zupt_acc']}</td><td>{r['zupt_imu_only']}</td><td>{r['false_acc']}</td><td>{r.get('frames_held') or ''}</td>"
        f"<td>{r['reused_from']}</td></tr>" for r in sorted(S["runs"], key=lambda r: (r["group"], r["pair"], r["cond"])))
    page = (f"<!doctype html><html lang='ko'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
            f"<title>Z2 나누어 보기</title>{zh.CSS}</head><body><h1>Z2 나누어 보기 (2026-10-21)</h1>"
            f"<div class='box concl'><h2 style='margin-top:0'>결론 먼저 (모두 추정)</h2>{notes}</div>"
            "<h2>무엇을 했나</h2><div class='box'><p>Z2 = <code>zupt_chi2_multipler 1</code> + <code>zupt_noise_multiplier 1</code> 두 가지를 "
            "한꺼번에 바꾼 것. 하나씩 떼어 보았다: " + ", ".join(f"<b>{c}</b> = {DESC[c]}" for c in conds) + ". "
            "벤치·인자·초기화·코드는 docs/z2_regression_plan.md 와 같음 (사전 등록 docs/z2_isolate_plan.md, 실행 전 커밋). "
            "판정 ①–③ 도 같음 (① ATE 중앙값 ≤ max(base×1.10, base+0.2 cm) ② 새 무너짐 0 ③ 헛 통과 0).</p>"
            "<p>참고: chi² 0 인 조건 (base·N1·H) 은 IMU 만으로 ZUPT 통과가 구조상 없다 → ③ 은 자동 0. "
            "'정답 >2 cm/s 인 ZUPT 전체' 칸은 시차 통과까지 센 보조 값 (정답 잡음 바닥 포함).</p></div>"
            "<h2>1. 무너짐 원인: 무너진 두 짝을 조건마다</h2>"
            "<table><tr><th>짝</th><th>조건</th><th>뜻</th><th>상태</th><th>ATE cm</th><th>축척</th><th>초기화 로그 s</th>"
            "<th>첫 시각 갱신 s</th><th>첫 tracking 포즈 s</th><th>초기화~첫 시각 갱신 ZUPT 수</th><th>IMU 만 ZUPT</th><th>ZUPT 전체</th></tr>"
            + "".join(cause) + "</table><p><small>시각 = 클립 첫 프레임부터 초. o0 (221052)·o1 (131326) 은 비교용 (Z2 에서도 성공한 짝). "
            "'추적 시작을 같게 강제' 하는 변형은 벤치에 장치가 없어 만들지 않았다 (코드 한 버전 규칙).</small></p>"
            "<h2>2. 내려놓기 (224323) 를 고치나</h2><table><tr><th>녹화</th><th>조건</th><th>마지막 3 s 창 오차 중앙 cm</th>"
            "<th>끝 정지 이동 중앙 cm</th><th>ATE 중앙 cm</th><th>고침 (≤ base 50 % 둘 다)</th></tr>" + "".join(sd) + "</table>"
            "<h2>3. 1 단계 순위 (최선 조건 고르기, 규칙은 실행 전 고정)</h2><table><tr><th>조건</th><th>뜻</th><th>내려놓기 고침</th>"
            "<th>실패 묶음 수</th><th>실패 묶음</th><th>새 무너짐</th><th>헛 통과</th><th>224323 마지막 창 cm</th></tr>" + rank + "</table>"
            f"<p>최선 조건: <b>{S.get('best')}</b></p>"
            "<h2>4. 1 단계 묶음별 전체 표</h2>" + group_rows(S, P1, conds)
            + "<h2>5. 2 단계: 최선 조건을 나머지 회귀 묶음 전체에서</h2>" + ftxt
            + (group_rows(S, P2, ["base", "Z2", S.get("best")]) if P2 else "")
            + f"<h2>결정성</h2><div class='box'><p>{dtxt}</p></div>"
            f"<h2>스스로 점검</h2><div class='box'>{chk}</div>"
            "<details><summary>실행별 표</summary><table><tr><th>실행</th><th>상태</th><th>ATE</th><th>축척</th><th>초기화 로그 s</th>"
            "<th>첫 시각 갱신 s</th><th>첫 tracking s</th><th>ZUPT</th><th>IMU 만</th><th>헛 통과</th><th>붙잡은 프레임</th><th>출처</th></tr>"
            + runs + "</table></details>"
            f"<p><small>만든 시각 {S['made']}, git {S['git']}. 결과 runs/z2_isolate/summary.json. 출처 'z2_regress'·'setdown_mitig' = "
            "그 폴더의 같은 인자 실행을 복사 (원본 그대로).</small></p></body></html>")
    DOC.write_text(page)
    print(f"wrote {DOC}; selfcheck cells {nchk} mismatches {len(bad)}")
    for b in bad:
        print(" ", b)


if __name__ == "__main__":
    main()
