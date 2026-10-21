#!/usr/bin/env python3
"""docs/h_arm_init.html from runs/h_arm_init/summary.json (imu_learn/h_arm_init.py analyse) + a self-check that
recomputes from the raw run JSON / pose files (not analyse_one): every cell's status and group median ATE, the
still_hold string and num_opencv_threads of executed runs, the copy-rule source, the held-frame prediction, and the
held counts per group.

  ../venv_ffs/bin/python imu_learn/h_arm_init_html.py --notes NOTES.html
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
import h_arm_init as ha  # noqa: E402
from h_tune_html import CSS, f  # noqa: E402

DOC = Path("docs/h_arm_init.html")
V3 = {"pass": '<span class="ok">통과</span>', "fail": '<span class="bad">실패</span>', "not tested": "시험 안 됨",
      "n/a (0 held)": "해당 없음 (붙잡기 0)"}


def yes(b):
    return '<span class="ok">통과</span>' if b else '<span class="bad">실패</span>'


def status_raw(p):
    d = json.load(open(str(p) + ".json"))
    trk = any(json.loads(x).get("status") == "tracking" for x in gzip.open(str(p) + ".poses.jsonl.gz", "rt"))
    if not trk:
        return "noinit", None, d
    if d.get("ate_cm") is None or d.get("scale") is None:
        return "unscored", None, d
    return ("ok" if (0.5 <= d["scale"] <= 2 and d["ate_cm"] <= 10) else "collapse"), d["ate_cm"], d


def selfcheck(S):
    bad, n = [], 0
    cp = json.load(open(ha.OUT / "copyplan.json"))
    mk = json.load(open(ha.OUT / "mask.json"))["runs"]
    for v, F in S["full"].items():
        for g, x in F["groups"].items():
            for c, key_a, key_m in (("base", "base_ates", "base_med"), (v, "ates", "med")):
                st, at, held = [], [], 0
                for pr in x["pairs"]:
                    own = ha.OUT / ha.ht.name(pr, c)
                    if Path(str(own) + ".json").exists():
                        p, src = own, "own"
                    elif c != "base" and cp.get(c, {}).get(pr):
                        p, src = ha.OUT / ha.ht.name(pr, cp[c][pr]), cp[c][pr]
                    else:
                        bad.append(f"{pr}/{c}: no source")
                        continue
                    s, a, d = status_raw(p)
                    st.append(s)
                    if s in ("ok", "collapse"):
                        at.append(a)
                    n += 1
                    if d.get("cfg_over") != f"num_opencv_threads={ha.THREADS}" or d.get("bench_blob") != S["blobs"][
                            "imu_learn/vio_bench_hs.py"]:
                        bad.append(f"{pr}/{c}: cfg_over {d.get('cfg_over')} bench {d.get('bench_blob')}")
                    if c != "base":
                        want = ha.VARIANTS[c] if src == "own" else ha.VARIANTS.get(src)
                        if src != "base" and d.get("still_hold") != want:
                            bad.append(f"{pr}/{c}: still_hold {d.get('still_hold')} != {want}")
                        if src == "base" and (mk[pr][c]["held"] != 0 or d.get("still_hold")):
                            bad.append(f"{pr}/{c}: base copy but predicted held {mk[pr][c]['held']}")
                        if src not in ("own", "base") and mk[pr][c]["segs"] != mk[pr][src]["segs"]:
                            bad.append(f"{pr}/{c}: copy of {src} with different predicted segments")
                        hd = d.get("frames_held") or 0
                        if mk[pr][c]["held"] != hd:
                            bad.append(f"{pr}/{c}: predicted held {mk[pr][c]['held']} != {hd} ({src})")
                        held += hd
                    elif d.get("still_hold"):
                        bad.append(f"{pr}/base: still_hold set")
                med = float(np.median(at)) if at else None
                want_st = x["base_status"] if c == "base" else x["status"]
                if st != want_st:
                    bad.append(f"{v}/{g}/{c}: status {st} != {want_st}")
                if (med is None) != (x[key_m] is None) or (med is not None and abs(med - x[key_m]) > 1e-9):
                    bad.append(f"{v}/{g}/{c}: median {med} != {x[key_m]}")
                if c != "base" and held != sum(h or 0 for h in x["held"]):
                    bad.append(f"{v}/{g}: held {held} != {x['held']}")
    return n, bad


def full_table(v, F):
    h = [f"<h2>18 묶음 × 3 시작 — {html.escape(v)} (<code>--still-hold {html.escape(ha.VARIANTS[v])}</code>) 대 base</h2>",
         "<table><tr><th class=l>묶음</th><th>base ATE (cm)</th><th>변형 ATE (cm)</th><th>중앙값 base→변형</th><th>한도</th>"
         "<th>1 ATE</th><th>2 새 무너짐</th><th>3 헛 통과</th><th>붙잡은 프레임</th><th>&gt;2 cm/s 표본 (비율)</th>"
         "<th>최대 GT 속도 (cm/s)</th><th>원래 3'</th><th>고친 3'</th><th>원래 기준 판정</th><th class=l>출처</th></tr>"]
    for g, x in F["groups"].items():
        share = "–" if x["held_share"] is None else f"{x['held_gt_fast']}/{x['held_gt_n']} ({100 * x['held_share']:.2f} %)"
        h.append(f"<tr><td class=l>{g}</td><td>{' / '.join(f(a) for a in x['base_ates'])}</td>"
                 f"<td>{' / '.join(f(a) for a in x['ates'])}</td><td>{f(x['base_med'])} → {f(x['med'])}</td><td>{f(x['lim'])}</td>"
                 f"<td>{yes(x['ate_ok'] is not False)}</td><td>{yes(not x['new_bad'])}</td><td>{x['false_acc']}</td>"
                 f"<td>{' / '.join(str(a) for a in x['held'])}</td><td>{share}</td>"
                 f"<td>{f(x['held_vmax_cms']) if x['held_gt_n'] else '–'}</td><td>{V3[x['c3h']]}</td><td>{V3[x['c3h_amended']]}</td>"
                 f"<td>{V3[x['verdict_orig']]}</td><td class=l>{html.escape(', '.join(sorted(set(x['src']))))}</td></tr>")
    h.append("</table>")
    sd = F["setdown"]
    if sd:
        h.append(f"<p>내려놓기 (224323, 시작 3 개 중앙값): 마지막 창 오차 base {f(sd['lastwin'][0])} → {f(sd['lastwin'][1])} cm, "
                 f"끝 정지 이동 {f(sd['drift'][0])} → {f(sd['drift'][1])} cm → 고침 {yes(sd['ok'])}</p>")
    return h


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--notes", default=None)
    o = ap.parse_args()
    S = json.load(open(ha.OUT / "summary.json"))
    n_chk, bad = selfcheck(S)
    notes = Path(o.notes).read_text() if o.notes else ""
    h = ["<!doctype html><html lang=ko><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
         f"<title>초기화 뒤 붙잡기</title><style>{CSS}</style></head><body><h1>정지 붙잡기를 OpenVINS 초기화 뒤에만 켜기</h1>",
         f"<p class=small>만든 시각 {S['made']} · git {S['git']} · 계획 docs/h_arm_init_plan.md (사전 등록 31915ae) · 결과 runs/h_arm_init/</p>",
         notes]
    # summary of verdicts
    h.append("<h2>한눈에 (18 묶음 × 3 시작)</h2><table><tr><th class=l>변형</th><th class=l>--still-hold</th>"
             "<th>원래 기준 실패 묶음</th><th>시험 안 됨</th><th>고친 3' 기준 실패 묶음</th><th>짝 한도 넘은 짝</th><th>새 무너짐</th>"
             "<th>내려놓기 고침</th></tr>")
    for v, F in S["full"].items():
        h.append(f"<tr><td class=l>{v}</td><td class=l><code>{ha.VARIANTS[v]}</code></td>"
                 f"<td class=l>{', '.join(F['orig_fail']) or '<span class=ok>없음</span>'}</td>"
                 f"<td class=l>{', '.join(F['orig_untested']) or '없음'}</td>"
                 f"<td class=l>{', '.join(F['amended_fail']) or '<span class=ok>없음</span>'}</td><td>{F['pair_over']}</td>"
                 f"<td>{F['new_bad']}</td><td>{yes(F['setdown'] and F['setdown']['ok'])}</td></tr>")
    h.append("</table>")
    # coarse
    h.append("<h2>1 차 (시작 1 개: F·O o0, T d0)</h2><table><tr><th class=l>변형</th><th>원래 기준 실패 묶음</th><th>짝 한도 넘은 짝</th>"
             "<th>새 무너짐</th><th>원래 3' 실패</th><th>224323_o0 내려놓기 고침</th></tr>")
    for v, C in S["coarse"].items():
        sd = C["setdown"]
        lw = "" if not sd else f"(마지막 창 {f(sd['lastwin'][0])} → {f(sd['lastwin'][1])} cm)"
        h.append(f"<tr><td class=l>{v}</td><td class=l>{', '.join(C['orig_fail']) or '없음'}</td><td>{C['pair_over']}</td>"
                 f"<td>{C['new_bad']}</td><td class=l>{', '.join(C['c3h_fail']) or '없음'}</td>"
                 f"<td>{yes(sd and sd['ok'])} {lw}</td></tr>")
    h.append("</table>")
    for v in ["Hw15i0", "Hw15i3", "Hw15i10", "Hw15m05"]:
        if v in S["full"]:
            h += full_table(v, S["full"][v])
    h.append("<h2>점검</h2><ul>")
    h.append(f"<li>코드 버전 수 {S['code_versions']}, OpenVINS 스레드 {S['threads']} (base 포함 전부 같은 설정)</li>")
    h.append(f"<li>붙잡을 프레임 예측 = 실제 frames_held: 실행 {S['pred_checked']} 개, 불일치 {len(S['pred_mismatch'])} "
             f"{html.escape(str(S['pred_mismatch']))}</li>")
    h.append(f"<li>새 base 의 초기화 시각 = 예측에 쓴 저장된 base 의 초기화 시각: {sum(S['init_same'].values())}/{len(S['init_same'])}</li>")
    for k, d in S["conf"].items():
        h.append(f"<li>복사 규칙 점검 {html.escape(k)}: 포즈 {d['n_common']}/{d['n_a']} 공통, 최대 위치 차 {d['max_pos_diff_m']} m, "
                 f"붙잡기 {d['held']}, 브리지 초기화 시각 {d['hold_init_ns']} (base {d['base_init_ns']}), 기다림 초과 {d['wait_timeouts']}</li>")
    h.append(f"<li>브리지 pose 기다림 초과(5 s) 가 있었던 실행: {html.escape(str(S['wait_timeouts'])) or '없음'}</li>")
    h.append(f"<li>스스로 점검 (원본 JSON·포즈에서 상태·중앙값·설정·출처·예측·붙잡기 수 다시 계산): {n_chk} 칸, 불일치 {len(bad)}"
             + "".join(f"<br><code>{html.escape(b)}</code>" for b in bad[:20]) + "</li></ul>")
    h.append("</body></html>")
    DOC.write_text("\n".join(h))
    print(f"wrote {DOC}; selfcheck {n_chk} cells, {len(bad)} mismatches", bad[:10])


if __name__ == "__main__":
    main()
