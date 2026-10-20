#!/usr/bin/env python3
"""C5 retest 2026-10-21 analysis (docs/c5_retest_plan.md). Per run labels = c5_overnight_analyse.analyse_run (10 s window
rule and full-run rule, GT as on disk); failure = not initialized + collapsed + unlabelled. Per lead x S x config counts,
per recording failures, pair comparisons (recording-level sign test), drone recommendation rule, fine motion onset.

  ../venv_ffs/bin/python imu_learn/c5_retest_analyse.py runs/c5_retest [--html docs/c5_retest_result.html --concl FILE]
"""
import argparse
import html
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import c5_overnight_analyse as an  # noqa: E402
from imu_gt import read_imu  # noqa: E402

SCAN3D = HERE.parent
STILL = SCAN3D / "runs" / "overnight_vio" / "inputs"
CFGS = ("loose", "strict", "strictdyn")
CFG_KO = {"loose": "loose (옛 템플릿)", "strict": "strict (앱 정적)", "strictdyn": "strict-dyn (앱 동적 허용)"}
PAIRS = (("loose", "strict"), ("loose", "strictdyn"), ("strict", "strictdyn"))
NAME = re.compile(r"^(L2_)?(\d{6})_S(\d)_(loose|strict|strictdyn)_(dm025|d0|dp025)$")
LATE_S = 3.0


def fine_onset(rec, coarse_s):
    """Motion onset with 0.1 s windows (step 0.02 s) from the coarse onset (0.5 s window start) on, same thresholds."""
    s = json.load(open(STILL / f"{rec}_still.json"))
    t0 = s["clip_start_imu_ns"]
    acc, gyr = read_imu(SCAN3D / "recordings" / rec / "imu.csv", raw=True)
    t = gyr[:, 0]
    g = np.degrees(gyr[:, 1:4])
    an_ = np.linalg.norm(np.stack([np.interp(t, acc[:, 0], acc[:, 1 + j]) for j in range(3)], 1), axis=1)
    x = t0 + coarse_s * 1e9
    while x < t0 + (coarse_s + 2) * 1e9:
        m = (t >= x) & (t < x + 0.1e9)
        if m.sum() > 5 and (g[m].std(0).max() >= 1.0 or an_[m].std() >= 0.1):
            return round((x - t0) / 1e9, 2)
        x += 0.02e9
    return None


def sign_p(a, b):
    return an.sign_p(a, b)


def summarize(d):
    runs, failed = [], []
    for f in sorted(Path(d).glob("*.json")):
        m = NAME.match(f.stem)
        if not m:
            continue
        try:
            r = an.analyse_run(str(f)[:-5], k=1.0)
        except Exception as e:  # noqa: BLE001
            failed.append(dict(name=f.stem, why=repr(e)))
            continue
        lead = "L2" if m[1] else "L0"
        r.update(lead=lead, S=int(m[3]), cfg=m[4], dn=m[5])
        still = json.load(open(STILL / f"{r['rec']}_still.json"))
        ms = still["motion_start_s"]
        r["still_left_s"] = round(ms - r["start_off"], 3)
        r["t_init_after_motion_s"] = (None if r.get("t_init_clip_s") is None
                                      else round(r["t_init_clip_s"] - r["still_left_s"], 3))
        r["late_init"] = bool(r["t_init_after_motion_s"] is not None and r["t_init_after_motion_s"] > LATE_S)
        for lab, key in (("win", "collapsed"), ("full", "collapsed_full")):
            if not r.get("initialized"):
                r[f"out_{lab}"] = "noinit"
            elif r.get(key) is None:
                r[f"out_{lab}"] = "unlabelled"
            else:
                r[f"out_{lab}"] = "collapsed" if r[key] else "ok"
            r[f"fail_{lab}"] = r[f"out_{lab}"] != "ok"
        runs.append(r)
    for e in Path(d).glob("*.err"):
        if NAME.match(e.stem) and not Path(str(e)[:-4] + ".json").exists():
            lines = e.read_text(errors="replace").strip().splitlines()
            failed.append(dict(name=e.stem, why="no json: " + (lines[-1][-200:] if lines else "(running or empty)")))
    by = defaultdict(list)
    for r in runs:
        by[(r["lead"], r["S"], r["cfg"], r["rec"])].append(r)
    recs = sorted({r["rec"] for r in runs})
    cells = {}
    for lead in ("L0", "L2"):
        for S in (1, 2, 3):
            for c in CFGS:
                rs = [r for r in runs if r["lead"] == lead and r["S"] == S and r["cfg"] == c]
                if not rs:
                    continue
                ok_full = [r for r in rs if r["out_full"] == "ok"]
                cells[f"{lead}|S{S}|{c}"] = dict(
                    n=len(rs), noinit=sum(r["out_win"] == "noinit" for r in rs),
                    **{f"{k}_{lab}": sum(r[f"out_{lab}"] == k for r in rs) for lab in ("win", "full")
                       for k in ("collapsed", "unlabelled")},
                    fail_win=sum(r["fail_win"] for r in rs), fail_full=sum(r["fail_full"] for r in rs),
                    late_init=sum(r["late_init"] for r in rs),
                    static=sum(r.get("init_method") == "static" for r in rs),
                    dynamic=sum(r.get("init_method") == "dynamic" for r in rs),
                    t_init_after_motion_med=an.med([r["t_init_after_motion_s"] for r in rs]),
                    ate_full_med_ok=an.med([r.get("ate_full_cm") for r in ok_full]),
                    per_rec={rec[-6:]: dict(fail_win=sum(x["fail_win"] for x in by[(lead, S, c, rec)]),
                                            fail_full=sum(x["fail_full"] for x in by[(lead, S, c, rec)]),
                                            n=len(by[(lead, S, c, rec)])) for rec in recs})
    comps = {}
    for lead in ("L0", "L2"):
        for S in (1, 2, 3):
            for x, y in PAIRS:
                for lab in ("win", "full"):
                    cx, cy = cells.get(f"{lead}|S{S}|{x}"), cells.get(f"{lead}|S{S}|{y}")
                    if not cx or not cy:
                        continue
                    yb = xb = same = 0
                    rows = []
                    for rec in recs:
                        fx, fy = cx["per_rec"][rec[-6:]][f"fail_{lab}"], cy["per_rec"][rec[-6:]][f"fail_{lab}"]
                        rows.append((rec[-6:], fx, fy))
                        if fy < fx:
                            yb += 1
                        elif fx < fy:
                            xb += 1
                        else:
                            same += 1
                    p = sign_p(yb, xb)
                    n_rec = len(recs)
                    if yb == n_rec:
                        v = f"{y} 가 나음 (녹화 {n_rec}개 모두, p = {p:.4f}, 추정)"
                    elif xb == n_rec:
                        v = f"{x} 가 나음 (녹화 {n_rec}개 모두, p = {p:.4f}, 추정)"
                    elif yb == xb == 0:
                        v = "차이 없음 — 근거 약함"
                    elif yb > xb:
                        v = f"{y} 쪽으로 기움 — 근거 약함"
                    elif xb > yb:
                        v = f"{x} 쪽으로 기움 — 근거 약함"
                    else:
                        v = "엇갈림 — 근거 약함"
                    comps[f"{lead}|S{S}|{x}_vs_{y}|{lab}"] = dict(x=x, y=y, x_better=xb, y_better=yb, same=same,
                                                                 sign_p=p, verdict=v, rows=rows,
                                                                 fail_x=cx[f"fail_{lab}"], fail_y=cy[f"fail_{lab}"])
    rec_rule = {}
    for c in CFGS:
        zero = [S for S in (1, 2, 3) if cells.get(f"L0|S{S}|{c}", {}).get("fail_full") == 0
                and cells[f"L0|S{S}|{c}"]["n"] == 15]
        rec_rule[c] = dict(min_S_zero_fail=min(zero) if zero else None,
                           fail_sum=sum(cells.get(f"L0|S{S}|{c}", {}).get("fail_full", 0) for S in (1, 2, 3)))
    cand = [c for c in CFGS if rec_rule[c]["min_S_zero_fail"] is not None]
    if cand:
        best = sorted(cand, key=lambda c: (rec_rule[c]["min_S_zero_fail"], rec_rule[c]["fail_sum"], c != "strict"))[0]
        rec_txt = f"{best}, 정지 {rec_rule[best]['min_S_zero_fail']} s 이상 (전 구간 기준 0/15 실패, 녹화 5개, 추정)"
    else:
        best = None
        rec_txt = "3 s 로는 부족 — 어떤 설정도 S = 3 에서 0/15 가 아님 (밤샘 10 s 이상에선 실패 0)"
    onset = {}
    for rec in recs:
        ms = json.load(open(STILL / f"{rec}_still.json"))["motion_start_s"]
        fo = fine_onset(rec, ms)
        onset[rec[-6:]] = dict(coarse_s=ms, fine_s=fo, extra_still_s=None if fo is None else round(fo - ms, 2))
    versions = sorted({json.dumps(r["version"], sort_keys=True) for r in runs})
    return dict(n_runs=len(runs), failed=failed, recs=recs, mixed_versions=len(versions) > 1, versions=versions,
                cells=cells, comps=comps, rec_rule=rec_rule, recommend=best, recommend_txt=rec_txt, onset=onset,
                runs=[{k: r.get(k) for k in ("name", "rec", "lead", "S", "cfg", "dn", "start_off", "still_left_s",
                                             "initialized", "init_method", "t_init_clip_s", "t_init_after_motion_s",
                                             "late_init", "scale10", "ate10_cm", "scale5", "ate5_cm", "scale_full",
                                             "ate_full_cm", "out_win", "out_full", "bg_init_dps", "bridge_crash",
                                             "error", "wall_s")} for r in runs])


def fmt(v, n=2):
    if v is None:
        return "–"
    if isinstance(v, float):
        return f"{v:.{n}f}"
    return html.escape(str(v))


def write_html(res, path, concl):
    css = (":root{--bg:#fff;--fg:#1d1d1f;--mut:#666;--line:#ddd;--box:#f5f6f8;--good:#e8f6ec;--bad:#fbe9e9}"
           "@media (prefers-color-scheme:dark){:root{--bg:#16171a;--fg:#e8e8ea;--mut:#9a9aa0;--line:#33343a;--box:#202226;"
           "--good:#1d3a26;--bad:#3a1f1f}}"
           "body{background:var(--bg);color:var(--fg);font:15px/1.6 system-ui,sans-serif;max-width:1100px;margin:0 auto;padding:16px}"
           "table{border-collapse:collapse;margin:8px 0 16px;font-size:13px;display:block;overflow-x:auto}"
           "th,td{border:1px solid var(--line);padding:4px 6px;text-align:center}small{color:var(--mut)}"
           ".box{background:var(--box);border-radius:8px;padding:10px 14px;margin:10px 0}.concl{background:var(--good)}"
           "td.z{background:var(--good)}td.f{background:var(--bad)}code{font-size:12px}")
    h = [f"<!doctype html><html lang='ko'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
         f"<title>C5 재시험 결과</title><style>{css}</style></head><body><h1>C5 재시험: 정지가 짧으면 초기화 설정이 중요한가 (2026-10-21)</h1>"]
    h.append(f"<div class='box concl'>{concl}</div>" if concl else "<div class='box'>결론 작성 전 (자동 표만)</div>")
    h.append("<div class='box'><b>주의</b>: 정답 칸 크기는 아직 40.70 mm 추정값이다 (자로 안 잼). 실제가 44.49 mm 면 ATE 크기는 약 ×1.09 — "
             "실패 비율은 거의 안 바뀐다 (10 cm·축척 문턱 근처 실행만 바뀔 수 있음). 녹화 5개라 모든 결론은 <b>추정</b>이다.</div>")
    h.append(f"<h2>1. 무엇을 돌렸나</h2><p>실행 {res['n_runs']} 개, 실패(결과 없음) {len(res['failed'])} 개, 코드 버전 "
             f"{'<b>섞임 (판정 무효)</b>' if res['mixed_versions'] else '하나'}. 녹화 5개 × 정지 S = 1·2·3 s × 설정 3 × 자르는 위치 3 (±0.25 s). "
             "IMU 도 첫 프레임부터 넣음(앞당김 0 s). 계획: docs/c5_retest_plan.md.</p>")
    h.append("<table><tr><th>녹화</th><th>움직임 시작 (0.5 s 창, 자르기 기준)</th><th>0.1 s 창으로 다시 잰 시작</th><th>실제 정지가 더 김 (s)</th></tr>")
    for rec, o in res["onset"].items():
        h.append(f"<tr><td>{rec}</td><td>{fmt(o['coarse_s'])}</td><td>{fmt(o['fine_s'])}</td><td>{fmt(o['extra_still_s'])}</td></tr>")
    h.append("</table>")
    for lead, title in (("L0", "2. 주 결과 (IMU 앞당김 0 s)"), ("L2", "3. 보조: IMU 를 2 s 먼저 넣으면 (S = 1 만)")):
        if not any(k.startswith(lead) for k in res["cells"]):
            continue
        h.append(f"<h2>{title}</h2><table><tr><th>정지 S</th><th>설정</th><th>실행</th><th>초기화 안 됨</th>"
                 "<th>무너짐 (10 s 창)</th><th>실패 합 (10 s 창)</th><th>무너짐 (전 구간)</th><th>실패 합 (전 구간)</th>"
                 "<th>늦은 초기화 (&gt;3 s)</th><th>정적/동적</th><th>초기화 시각 − 움직임 (s, 중앙값)</th><th>성공 ATE 중앙값 (cm)</th>"
                 "<th>녹화별 실패 (전 구간)</th></tr>")
        for S in (1, 2, 3):
            for c in CFGS:
                x = res["cells"].get(f"{lead}|S{S}|{c}")
                if not x:
                    continue
                pr = " ".join(f"{k}:{v['fail_full']}" for k, v in x["per_rec"].items())
                cls = "z" if x["fail_full"] == 0 else ("f" if x["fail_full"] >= x["n"] / 2 else "")
                h.append(f"<tr><td>{S}</td><td>{CFG_KO[c]}</td><td>{x['n']}</td><td>{x['noinit']}</td>"
                         f"<td>{x['collapsed_win'] + x['unlabelled_win']}</td><td>{x['fail_win']}</td>"
                         f"<td>{x['collapsed_full'] + x['unlabelled_full']}</td><td class='{cls}'>{x['fail_full']}</td>"
                         f"<td>{x['late_init']}</td><td>{x['static']}/{x['dynamic']}</td><td>{fmt(x['t_init_after_motion_med'])}</td>"
                         f"<td>{fmt(x['ate_full_med_ok'])}</td><td><small>{pr}</small></td></tr>")
        h.append("</table><small>실패 = 초기화 안 됨 + 무너짐 + 미판정. 10 s 창 = 초기화 뒤 첫 10 s 축척 0.5~2·ATE &lt; 10 cm (밤샘 사전 기준). "
                 "전 구간 = 초기화 뒤 전체 축척 0.5~2·ATE ≤ 10 cm. 늦은 초기화는 판정에 안 씀(드론이면 이미 이륙한 뒤).</small>")
        h.append("<h3>짝 비교 (녹화 단위)</h3><table><tr><th>S</th><th>비교 (x vs y)</th><th>기준</th><th>실패 x / y</th>"
                 "<th>x 나음 / y 나음 / 같음 (녹화)</th><th>p</th><th>판정</th></tr>")
        for S in (1, 2, 3):
            for x_, y_ in PAIRS:
                for lab, lk in (("full", "전 구간"), ("win", "10 s 창")):
                    c = res["comps"].get(f"{lead}|S{S}|{x_}_vs_{y_}|{lab}")
                    if not c:
                        continue
                    h.append(f"<tr><td>{S}</td><td>{x_} vs {y_}</td><td>{lk}</td><td>{c['fail_x']} / {c['fail_y']}</td>"
                             f"<td>{c['x_better']} / {c['y_better']} / {c['same']}</td><td>{fmt(c['sign_p'], 3)}</td>"
                             f"<td>{html.escape(c['verdict'])}</td></tr>")
        h.append("</table>")
    h.append(f"<h2>4. 드론 권고 규칙 (사전 등록)</h2><p>{html.escape(res['recommend_txt'])}</p><table><tr><th>설정</th>"
             "<th>실패 0/15 인 가장 짧은 S</th><th>S 1~3 실패 합 (전 구간)</th></tr>")
    for c, v in res["rec_rule"].items():
        h.append(f"<tr><td>{CFG_KO[c]}</td><td>{fmt(v['min_S_zero_fail'])}</td><td>{v['fail_sum']}</td></tr>")
    h.append("</table>")
    h.append("<h2>5. 실행 하나하나</h2><table><tr><th>실행</th><th>남은 정지 (s)</th><th>초기화</th><th>방식</th><th>초기화 − 움직임 (s)</th>"
             "<th>10 s 창 축척</th><th>10 s 창 ATE</th><th>전체 축척</th><th>전체 ATE (cm)</th><th>10 s 창</th><th>전 구간</th></tr>")
    for r in sorted(res["runs"], key=lambda r: (r["lead"], r["S"], r["cfg"], r["rec"], r["dn"])):
        h.append(f"<tr><td>{html.escape(r['name'])}</td><td>{fmt(r['still_left_s'])}</td><td>{'예' if r['initialized'] else '아니오'}</td>"
                 f"<td>{fmt(r.get('init_method'))}</td><td>{fmt(r['t_init_after_motion_s'])}</td><td>{fmt(r.get('scale10'), 3)}</td>"
                 f"<td>{fmt(r.get('ate10_cm'))}</td><td>{fmt(r.get('scale_full'), 3)}</td><td>{fmt(r.get('ate_full_cm'))}</td>"
                 f"<td>{r['out_win']}</td><td>{r['out_full']}</td></tr>")
    h.append("</table>")
    if res["failed"]:
        h.append("<h2>결과 없는 실행</h2><ul>" + "".join(f"<li>{html.escape(f['name'])}: {html.escape(f['why'])}</li>"
                                                     for f in res["failed"]) + "</ul>")
    h.append("<p><small>코드: imu_learn/c5_retest.py, c5_run.py, vio_bench_hs.py (--imu-lead), c5_retest_analyse.py, 검산 c5_retest_check.py. "
             "결과: runs/c5_retest/ (실행마다 JSON·포즈·로그), summary.json, 진행 runs/c5_retest.log.</small></p></body></html>")
    Path(path).write_text("".join(h))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--html", default=None)
    ap.add_argument("--concl", default=None)
    o = ap.parse_args()
    res = summarize(o.dir)
    json.dump(res, open(Path(o.dir) / "summary.json", "w"), indent=1, ensure_ascii=False, default=float)
    if o.html:
        concl = Path(o.concl).read_text() if o.concl and Path(o.concl).exists() else ""
        write_html(res, o.html, concl)
    print(json.dumps(dict(n_runs=res["n_runs"], failed=len(res["failed"]), mixed=res["mixed_versions"],
                          cells={k: (v["n"], v["noinit"], v["fail_win"], v["fail_full"], v["late_init"])
                                 for k, v in res["cells"].items()},
                          recommend=res["recommend_txt"], onset=res["onset"]), ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
