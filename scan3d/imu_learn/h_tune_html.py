#!/usr/bin/env python3
"""docs/h_tune.html from runs/h_tune/summary_<coarse>.json and summary_<final>.json (imu_learn/h_tune.py analyse) +
a self-check that recomputes, straight from the raw run JSON / pose files (no analyse_one): every cell's status and the
group median ATE of the final variant, its still_hold string and num_opencv_threads, the copy-rule sources, and the
held-frame prediction vs frames_held.

  ../venv_ffs/bin/python imu_learn/h_tune_html.py --coarse Hm05,Hw15m05,Hsm05,Hw2m05 --final Hm05 --notes NOTES.html
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
import h_tune as ht  # noqa: E402

DOC = Path("docs/h_tune.html")
CSS = """body{font-family:system-ui,'Malgun Gothic',sans-serif;max-width:1100px;margin:24px auto;padding:0 16px;
line-height:1.55;color:#1d232a;background:#fff}h1{font-size:1.5em}h2{font-size:1.2em;margin-top:1.6em;border-bottom:1px solid #ddd}
table{border-collapse:collapse;font-size:.88em;margin:.6em 0;display:block;overflow-x:auto}td,th{border:1px solid #ccc;
padding:3px 7px;text-align:right;white-space:nowrap}th{background:#f2f4f6}td.l,th.l{text-align:left}.ok{color:#11702f;font-weight:600}
.bad{color:#b3261e;font-weight:600}.box{background:#f5f8fb;border-left:4px solid #3b6ea8;padding:.6em 1em}.small{color:#555;font-size:.9em}
@media (prefers-color-scheme:dark){body{background:#16191d;color:#e3e6ea}th{background:#262b31}td,th{border-color:#3a4048}
.box{background:#1f252c}.small{color:#aab}.ok{color:#6fd18f}.bad{color:#ff8a80}}"""


def f(x, d=2):
    return "–" if x is None else (f"{x:.{d}f}" if isinstance(x, float) else str(x))


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


def selfcheck(S, v):
    """Independent recomputation for the final variant v."""
    bad, n = [], 0
    cp = json.load(open(ht.OUT / "copyplan.json")).get(v, {})
    mk = json.load(open(ht.OUT / "mask.json"))["runs"]
    for g, x in S["full"][v]["groups"].items():
        for c in ("base", v):
            cell = x["cells"][c]
            st, at = [], []
            for pr in cell["pairs"]:
                own = ht.OUT / ht.name(pr, c)
                if c == "base":
                    p, want_src = ht.zr.OUT / ht.name(pr, "base"), "z2_regress"
                elif Path(str(own) + ".json").exists():
                    p, want_src = own, "this run"
                elif cp.get(pr) == "base":
                    p, want_src = ht.zr.OUT / ht.name(pr, "base"), "base copy (0 held)"
                elif cp.get(pr) == "H":
                    p, want_src = ht.zi.OUT / ht.name(pr, "H"), "H copy (same held frames)"
                else:
                    bad.append(f"{pr}/{c}: no source")
                    continue
                s, a, d = status_raw(p)
                st.append(s)
                if s in ("ok", "collapse"):
                    at.append(a)
                n += 1
                if c == v and want_src == "this run":
                    if d.get("still_hold") != ht.VARIANTS[v]:
                        bad.append(f"{pr}: still_hold {d.get('still_hold')}")
                    if (d.get("config_used") or {}).get("num_opencv_threads") not in (None, ht.THREADS):
                        bad.append(f"{pr}: threads {(d.get('config_used') or {}).get('num_opencv_threads')}")
                    if mk[pr][v]["held"] != d.get("frames_held"):
                        bad.append(f"{pr}: predicted held {mk[pr][v]['held']} != {d.get('frames_held')}")
                if c == v and want_src.startswith("base copy") and mk[pr][v]["held"] != 0:
                    bad.append(f"{pr}: copied from base but predicted held {mk[pr][v]['held']}")
            med = float(np.median(at)) if at else None
            if st != cell["status"]:
                bad.append(f"{g}/{c}: status {st} != {cell['status']}")
            if (med is None) != (cell["ate_med"] is None) or (med is not None and abs(med - cell["ate_med"]) > 1e-9):
                bad.append(f"{g}/{c}: median {med} != {cell['ate_med']}")
    return n, bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--coarse", required=True)
    ap.add_argument("--final", required=True)
    ap.add_argument("--notes", default=None)
    o = ap.parse_args()
    C = json.load(open(ht.OUT / f"summary_{o.coarse.replace(',', '_')}.json"))
    S = json.load(open(ht.OUT / f"summary_{o.final}.json"))
    v = o.final
    F = S["full"][v]
    n_chk, bad = selfcheck(S, v)
    notes = Path(o.notes).read_text() if o.notes else ""
    h = [f"<!doctype html><html lang=ko><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
         f"<title>H 다듬기</title><style>{CSS}</style></head><body><h1>H 다듬기 — 시작 정지는 건너뛰는 붙잡기</h1>",
         f"<p class=small>만든 시각 {S['made']} · git {S['git']} · 계획 docs/h_tune_plan.md · 결과 runs/h_tune/</p>", notes]
    # final table
    h.append(f"<h2>18 묶음 판정 — {html.escape(v)} (<code>--still-hold {html.escape(ht.VARIANTS[v])}</code>) 대 base</h2>")
    h.append("<table><tr><th class=l>묶음</th><th>base ATE (cm)</th><th>변형 ATE (cm)</th><th>중앙값 base→변형</th><th>한도</th>"
             "<th>1 ATE</th><th>2 새 무너짐</th><th>3 헛 통과</th><th>3' 붙잡은 동안 최대 GT 속도 (cm/s)</th>"
             "<th>&gt;2 cm/s 표본</th><th>짝별 최악 (cm)</th><th>붙잡은 프레임</th><th class=l>변형 결과 출처</th><th>판정</th></tr>")
    for g, x in F["groups"].items():
        b, z = x["cells"]["base"], x["cells"][v]
        h.append(f"<tr><td class=l>{g}</td><td>{' / '.join(f(a) for a in b['ates'])}</td><td>{' / '.join(f(a) for a in z['ates'])}</td>"
                 f"<td>{f(b['ate_med'])} → {f(z['ate_med'])}</td><td>{f(x['ate_limit'])}</td><td>{yes(x['ate_ok'] is not False)}</td>"
                 f"<td>{yes(not x['new_collapses'])}</td><td>{z['false_acc']}</td><td>{f(z['held_gt_vmax_cms'])}</td>"
                 f"<td>{z['held_gt_fast']}/{z['held_gt_n']}</td><td>{html.escape(str(x['worst_pair']))}</td>"
                 f"<td>{' / '.join(str(a) for a in z['held'])}</td><td class=l>{', '.join(sorted(set(z['src'])))}</td>"
                 f"<td>{yes(x['pass_'])}</td></tr>")
    h.append("</table>")
    sd = F["groups"]["F_224323"]["cells"]
    h.append(f"<p>내려놓기 (224323, 시작 3 개 중앙값): 마지막 3 s 창 오차 base {f(sd['base']['lastwin_med'])} → {f(sd[v]['lastwin_med'])} cm, "
             f"끝 정지 이동 {f(sd['base']['drift_med'])} → {f(sd[v]['drift_med'])} cm → 고침 {yes(F['helps_setdown'])}</p>")
    # coarse table
    h.append("<h2>1 차 (시작 1 개) — 변형 4 개</h2><table><tr><th class=l>변형</th><th class=l>짝</th><th>base ATE</th><th>변형 ATE</th>"
             "<th>짝 한도 넘음</th><th>상태</th><th>붙잡은 프레임</th><th>붙잡은 동안 최대 GT 속도</th><th>마지막 창 (base)</th>"
             "<th>끝 정지 이동 (base)</th><th class=l>출처</th></tr>")
    for cv, rows in C["coarse"].items():
        srcs = {r["key"]: r["src"] for r in C["runs"]}
        for r in rows:
            h.append(f"<tr><td class=l>{cv} <code>{html.escape(ht.VARIANTS[cv])}</code></td><td class=l>{r['pair']}</td>"
                     f"<td>{f(r['base'])}</td><td>{f(r['ate'])}</td><td>{'<span class=bad>예</span>' if r['over'] else '아니오'}</td>"
                     f"<td>{r['base_status']}→{r['status']}</td><td>{r['held']}</td><td>{f(r['held_vmax'])}</td>"
                     f"<td>{f(r['lastwin'])} ({f(r['base_lastwin'])})</td><td>{f(r['drift'])} ({f(r['base_drift'])})</td>"
                     f"<td class=l>{srcs.get(r['pair'] + '/' + cv, '')}</td></tr>")
    h.append("</table>")
    # checks
    det = S.get("det") or C.get("det") or {}
    h.append("<h2>점검</h2><ul>")
    for k, d in det.items():
        h.append(f"<li>det {k}: 새 벤치 + OpenVINS 스레드 2 대 저장 결과 — 포즈 {d['n_common']}/{d['n_a']} 개 공통, 최대 위치 차 "
                 f"{d['max_pos_diff_m']} m (ATE {d['ate']} vs {d['ref_ate']})</li>")
    h.append(f"<li>붙잡을 프레임 예측 = 실제 frames_held: 실행 {S['pred_checked']} 개 확인, 불일치 {len(S['pred_mismatch'])}</li>")
    h.append(f"<li>코드 버전 수(분석이 셈): {S['code_versions']} ({S['version_counts']}) — base·H 는 옛 벤치 blob, 새 실행은 새 blob. "
             f"둘이 같은 결과를 내는 것은 det 로 확인.</li>")
    h.append(f"<li>스스로 점검 (원본 JSON·포즈에서 상태·중앙값·설정·출처 다시 계산): {n_chk} 칸, 불일치 {len(bad)}"
             + ("".join(f"<br><code>{html.escape(b)}</code>" for b in bad[:20])) + "</li></ul>")
    h.append("</body></html>")
    DOC.write_text("\n".join(h))
    print(f"wrote {DOC}; selfcheck {n_chk} cells, {len(bad)} mismatches", bad[:10])


if __name__ == "__main__":
    main()
