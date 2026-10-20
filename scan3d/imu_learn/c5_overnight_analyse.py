#!/usr/bin/env python3
"""Overnight VIO 2026-10-20 analysis (docs/overnight_vio_prereg.md): per run labels from the saved pose stream and the
ground truth AS IT IS NOW on disk (so after imu_learn/rescale_gt.py just run this again), times an optional what-if GT
scale k (positions of the GT multiplied by k; k = s_true / s_est, s_est = 40.70 mm square).

Per run: initialized, init method / time / |bg|, first post-init 10 s window (else 5 s) Sim3 scale + SE3 ATE ->
success (plan: scale 0.5..2 and ATE < 10 cm), full post-init SE3/Sim3 ATE and scale, median of non-overlapping 10 s
window ATEs, tracked fraction. Per recording and comparison: prereg rules (sign tests at recording level).

  ../venv_ffs/bin/python imu_learn/c5_overnight_analyse.py runs/overnight_vio [--k 1.0] [--out runs/overnight_vio/summary.json]
      [--html docs/overnight_vio_result.html --concl runs/overnight_vio/conclusion.html]
"""
import argparse
import html
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import collapse_signals as cs  # noqa: E402
import vio_bench_hs as vb  # noqa: E402

K_ALT = {"13.3인치 (40.5 mm)": 40.5 / 40.70, "14인치 (42.7 mm)": 42.7 / 40.70}
_GT = {}


def gt(rec, name):
    key = (rec, name)
    if key not in _GT:
        g = np.load(vb.SCAN3D / "recordings" / rec / name / "gt.npz", allow_pickle=True)
        ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
        _GT[key] = (g["t_ns"][ok], g["p_bc"][ok], str(g["square_mm"]) if "square_mm" in g.files else None)
    return _GT[key]


def match(vt, gt_t):
    j = np.clip(np.searchsorted(gt_t, vt), 1, len(gt_t) - 1)
    j = np.where(np.abs(gt_t[j - 1] - vt) < np.abs(gt_t[j] - vt), j - 1, j)
    return j, np.abs(gt_t[j] - vt) < 2e6


def ate(P, Pg):
    R, _, t = vb.umeyama(P, Pg)
    Rs, c, ts = vb.umeyama(P, Pg, scale=True)
    e = np.linalg.norm(P @ R.T + t - Pg, axis=1)
    es = np.linalg.norm(c * P @ Rs.T + ts - Pg, axis=1)
    return float(np.sqrt((e ** 2).mean()) * 100), float(np.sqrt((es ** 2).mean()) * 100), float(c)


def analyse_run(prefix, k=1.0):
    d = json.load(open(prefix + ".json"))
    out = dict(name=Path(prefix).name, rec=d["rec"], init_cfg=d.get("init_cfg"), imu=d.get("imu_source"),
               calib=d.get("calib_label", "none"), noise=d.get("noise"), start_off=d.get("start_off"),
               t_init_clip_s=d.get("t_init_clip_s"), covered_frac=d.get("covered_frac"),
               tracked_frac=d.get("tracked_frac"), bench_ate_cm=d.get("ate_cm"), bench_scale=d.get("scale"),
               gt=d.get("gt"), wall_s=d.get("wall_s"), loadavg_start=d.get("loadavg_start"), error=d.get("error"),
               bridge_crash=d.get("bridge_crash"),
               version=dict(bench_blob=d.get("bench_blob"), runner_blob=d.get("runner_blob"),
                            make_ov_config_blob=d.get("make_ov_config_blob"), template_blobs=d.get("template_blobs"),
                            binary_sha256=d.get("binary_sha256"), bench_dirty=d.get("bench_dirty"),
                            lens_blob=d.get("lens_blob")),
               calib_sha256=d.get("calib_sha256"), dt_cam_imu_final_ms=d.get("dt_cam_imu_final_ms"), k=k)
    run = cs.load(prefix)
    ini = np.nonzero((run["status"] == "tracking") & np.isfinite(run["T"][:, 0]))[0]
    if not len(ini):
        out.update(initialized=False, success=False, collapsed=None)
        return out
    i0 = int(ini[0])
    info = run["init_info"]
    out.update(initialized=True, init_method=info.get("method", "?"),
               bg_init_dps=float(np.degrees(np.linalg.norm(info["bg_init"]))) if "bg_init" in info else None)
    gt_t, gt_p, sq = gt(d["rec"], d.get("gt") or "imu_gt_spl")
    gt_p = gt_p * k
    out["gt_square_mm"] = sq
    m = (run["status"] == "tracking") & np.isfinite(run["T"][:, 0])
    vt = run["t_ns"] + int(d["align_dt_ms"] * 1e6)
    t0 = run["t_ns"][i0]
    for W in (5, 10):
        w = m & (run["t_ns"] >= t0) & (run["t_ns"] <= t0 + W * 1e9)
        j, mm = match(vt[w], gt_t)
        if mm.sum() < 30:
            continue
        P = run["T"][w].reshape(-1, 4, 4)[:, :3, 3][mm]
        a, a_s, c = ate(P, gt_p[j[mm]])
        out[f"scale{W}"], out[f"ate{W}_cm"], out[f"ate{W}_sim3_cm"] = c, a, a_s
    c = out.get("scale10", out.get("scale5", np.nan))
    a = out.get("ate10_cm", out.get("ate5_cm", np.nan))
    out["label_window_s"] = 10 if "scale10" in out else (5 if "scale5" in out else None)
    out["collapsed"] = None if not np.isfinite(c) else bool(c < 0.5 or c > 2 or a > 10)
    out["success"] = out["collapsed"] is False
    j, mm = match(vt[m], gt_t)
    if mm.sum() >= 50:
        P = run["T"][m].reshape(-1, 4, 4)[:, :3, 3][mm]
        Pg = gt_p[j[mm]]
        out["ate_full_cm"], out["ate_full_sim3_cm"], out["scale_full"] = ate(P, Pg)
        tt = vt[m][mm]
        wins = []
        s = tt[0]
        while s + 10e9 <= tt[-1]:
            w = (tt >= s) & (tt < s + 10e9)
            if w.sum() >= 100:
                wins.append(ate(P[w], Pg[w])[0])
            s += 10e9
        out["ate_win10_med_cm"] = float(np.median(wins)) if wins else None
        out["n_win10"] = len(wins)
        out["post_init_s"] = float((tt[-1] - tt[0]) / 1e9)
    return out


def sign_p(n_plus, n_minus):
    n = n_plus + n_minus
    if n == 0:
        return None
    kk = min(n_plus, n_minus)
    return min(1.0, 2 * sum(math.comb(n, i) for i in range(kk + 1)) / 2 ** n)


def med(x):
    x = [v for v in x if v is not None and not (isinstance(v, float) and math.isnan(v))]
    return float(np.median(x)) if x else None


def group_stats(runs):
    succ = [r for r in runs if r.get("success")]
    a = [r["ate_full_cm"] for r in succ if r.get("ate_full_cm") is not None]
    return dict(n=len(runs), n_init=sum(bool(r.get("initialized")) for r in runs),
                n_labelled=sum(r.get("collapsed") is not None for r in runs),
                n_collapsed=sum(bool(r.get("collapsed")) for r in runs), n_success=len(succ),
                ate_med=med(a), ate_range=(max(a) - min(a)) if len(a) >= 2 else None, ates=a,
                ate_sim3_med=med([r.get("ate_full_sim3_cm") for r in succ]),
                scale_med=med([r.get("scale_full") for r in succ]), ate_win10_med=med([r.get("ate_win10_med_cm") for r in succ]),
                t_init_med=med([r.get("t_init_clip_s") for r in runs]), tracked_med=med([r.get("tracked_frac") for r in runs]))


def compare(by, x, y, recs):
    """Prereg rule: per recording median post-init ATE of successful runs, y vs x; 'same' if |diff| <= repeat spread
    (max of the two within-recording ranges); fewer than 2 successes in either -> untested. Sign test better vs worse."""
    rows, better, worse, same, unt = [], 0, 0, 0, 0
    for rec in recs:
        gx, gy = by.get((x, rec)), by.get((y, rec))
        if not gx or not gy:
            continue
        sx, sy = group_stats(gx), group_stats(gy)
        if sx["n_success"] < 2 or sy["n_success"] < 2:
            v, unt = "untested", unt + 1
            diff = spread = None
        else:
            diff = sy["ate_med"] - sx["ate_med"]
            spread = max(sx["ate_range"] or 0, sy["ate_range"] or 0)
            if abs(diff) <= spread:
                v, same = "same", same + 1
            elif diff < 0:
                v, better = "better", better + 1
            else:
                v, worse = "worse", worse + 1
        rows.append(dict(rec=rec, x=sx, y=sy, diff_cm=diff, spread_cm=spread, verdict=v,
                         success_x=sx["n_success"], success_y=sy["n_success"]))
    n_rec = better + worse + same
    s_plus = sum(r["success_y"] > r["success_x"] for r in rows)
    s_minus = sum(r["success_y"] < r["success_x"] for r in rows)
    if n_rec == 0:
        overall = "판정 불가 (비교 가능한 녹화 없음)"
    elif better == worse == 0:
        overall = "차이 없음"
    elif better > worse:
        overall = "y 가 나음 (방향)"
    elif worse > better:
        overall = "y 가 나쁨 (방향)"
    else:
        overall = "엇갈림"
    p = sign_p(better, worse)
    weak = n_rec < 6 or p is None or p >= 0.05
    return dict(x=x, y=y, rows=rows, better=better, worse=worse, same=same, untested=unt, sign_p=p,
                overall=overall + (" — 근거 약함" if weak else ""), success_more=s_plus, success_less=s_minus,
                success_sign_p=sign_p(s_plus, s_minus))


def c5_rule(by, recs_still):
    rows, tot_a, tot_c, lab_a, lab_c, plus, minus, unt = [], 0, 0, 0, 0, 0, 0, 0
    for rec in recs_still:
        ga, gc = by.get(("A_strict_cal", rec)), by.get(("A_loose_cal", rec))
        if not ga or not gc:
            continue
        sa, sc = group_stats(ga), group_stats(gc)
        tested = sa["n_labelled"] >= 2 and sc["n_labelled"] >= 2
        unt += not tested
        if tested:
            tot_a += sa["n_collapsed"]
            tot_c += sc["n_collapsed"]
            lab_a += sa["n_labelled"]
            lab_c += sc["n_labelled"]
            fa, fc = sa["n_collapsed"] / sa["n_labelled"], sc["n_collapsed"] / sc["n_labelled"]
            plus += fa < fc
            minus += fa > fc
        rows.append(dict(rec=rec, strict=sa, loose=sc, tested=tested))
    if lab_a == 0:
        v = "판정 불가 (strict 가 초기화 안 됨 / 시험된 녹화 없음)"
    elif tot_c == 0:
        v = "판정 불가 (loose 도 무너지지 않음 — 줄일 무너짐이 없음)"
    elif tot_a / lab_a <= 0.5 * tot_c / lab_c:
        v = "지지"
    else:
        v = "지지 안 함"
    p = sign_p(plus, minus)
    n_t = len([r for r in rows if r["tested"]])
    if n_t < 6 or p is None or p >= 0.05:
        v += " — 근거 약함"
    return dict(rows=rows, collapsed_strict=tot_a, labelled_strict=lab_a, collapsed_loose=tot_c, labelled_loose=lab_c,
                recs_strict_better=plus, recs_strict_worse=minus, sign_p=p, untested_recs=unt, verdict=v)


def summarize(d, k):
    runs, failed = [], []
    for f in sorted(Path(d).glob("*.json")):
        if f.name in ("summary.json", "best_config.json") or f.parent.name == "inputs":
            continue
        p = str(f)[:-5]
        if not Path(p + ".poses.jsonl.gz").exists():
            failed.append(dict(name=f.stem, why="no poses"))
            continue
        try:
            runs.append(analyse_run(p, k))
        except Exception as e:  # noqa: BLE001
            failed.append(dict(name=f.stem, why=repr(e)))
    for e in Path(d).glob("*.err"):
        if not Path(str(e)[:-4] + ".json").exists():
            lines = e.read_text(errors="replace").strip().splitlines()
            failed.append(dict(name=e.stem, why="no json: " + (lines[-1][-200:] if lines else "(running or empty)")))
    still = {}
    for sj in (Path(d) / "inputs").glob("*_still.json"):
        s = json.load(open(sj))
        still[s["rec"]] = s
    by = defaultdict(list)
    for r in runs:
        if r["name"].startswith("rep_"):
            continue
        rec6, setting = r["name"].split("_", 1)
        setting = setting.rsplit("_o", 1)[0]
        by[(setting, r["rec"])].append(r)
    recs = sorted({r["rec"] for r in runs})
    recs_still = [x for x in recs if still.get(x, {}).get("has_still")]
    best = json.load(open(Path(d) / "best_config.json"))["best"] if (Path(d) / "best_config.json").exists() else None
    versions = sorted({json.dumps(r["version"], sort_keys=True) for r in runs})
    res = dict(k=k, n_runs=len(runs), failed=failed, mixed_versions=len(versions) > 1, versions=versions, best=best,
               recs=recs, recs_still=recs_still, still=still,
               settings={f"{s}|{rec}": group_stats(v) for (s, rec), v in sorted(by.items())})
    res["C5"] = c5_rule(by, recs_still)
    res["A_strictdyn_vs_loose"] = compare(by, "A_loose_cal", "A_strictdyn_cal", recs)
    res["A_strict_vs_loose_ate"] = compare(by, "A_loose_cal", "A_strict_cal", recs_still)
    if best:
        res["B_raw_vs_cal"] = compare(by, f"A_{best}_cal", f"B_{best}_raw", recs)
        res["C_oldphys"] = compare(by, f"A_{best}_cal", f"C_{best}_oldphys", recs)
        res["C_stillbias"] = compare(by, f"A_{best}_cal", f"C_{best}_stillbias", recs)
        res["N_allan"] = compare(by, f"A_{best}_cal", f"N_{best}_allan", recs)
        res["C2_raw_stillbias"] = compare(by, f"B_{best}_raw", f"C2_{best}_rawstillbias", recs)
    det = []
    for r in runs:
        if r["name"].startswith("rep_"):
            o = next((x for x in runs if x["name"] == r["name"][4:]), None)
            if o:
                det.append(dict(name=o["name"], load=[o.get("loadavg_start"), r.get("loadavg_start")],
                                ate_full=[o.get("ate_full_cm"), r.get("ate_full_cm")],
                                t_init=[o.get("t_init_clip_s"), r.get("t_init_clip_s")],
                                success=[o.get("success"), r.get("success")]))
    res["determinism"] = det
    res["runs"] = runs
    return res


def fmt(v, n=2):
    if v is None:
        return "–"
    if isinstance(v, float):
        return f"{v:.{n}f}"
    return html.escape(str(v))


def cmp_table(c, xl, yl):
    h = [f"<table><tr><th>녹화</th><th>{xl} 성공</th><th>{yl} 성공</th><th>{xl} ATE 중앙값</th><th>{yl} ATE 중앙값</th>"
         "<th>차이 (y−x)</th><th>반복 흩어짐</th><th>판정</th></tr>"]
    for r in c["rows"]:
        h.append(f"<tr><td>{r['rec'][-6:]}</td><td>{r['success_x']}/{r['x']['n']}</td><td>{r['success_y']}/{r['y']['n']}</td>"
                 f"<td>{fmt(r['x']['ate_med'])}</td><td>{fmt(r['y']['ate_med'])}</td><td>{fmt(r['diff_cm'])}</td>"
                 f"<td>{fmt(r['spread_cm'])}</td><td>{ {'better': 'y 나음', 'worse': 'y 나쁨', 'same': '같음', 'untested': '시험 안 됨'}[r['verdict']]}</td></tr>")
    h.append(f"</table><p>녹화 단위: y 나음 {c['better']}, y 나쁨 {c['worse']}, 같음 {c['same']}, 시험 안 됨 {c['untested']}, "
             f"부호 검정 p = {fmt(c['sign_p'], 3)} → <b>{html.escape(c['overall'])}</b>. 성공 수가 y 쪽이 많은 녹화 {c['success_more']}, "
             f"적은 녹화 {c['success_less']}.</p>")
    return "".join(h)


def write_html(res, alt, path, concl):
    css = (":root{--bg:#fff;--fg:#1d1d1f;--mut:#666;--line:#ddd;--box:#f5f6f8;--good:#e8f6ec}"
           "@media (prefers-color-scheme:dark){:root{--bg:#16171a;--fg:#e8e8ea;--mut:#9a9aa0;--line:#33343a;--box:#202226;--good:#1d3a26}}"
           "body{background:var(--bg);color:var(--fg);font:15px/1.6 system-ui,sans-serif;max-width:1100px;margin:0 auto;padding:16px}"
           "table{border-collapse:collapse;margin:8px 0 16px;font-size:13px;display:block;overflow-x:auto}"
           "th,td{border:1px solid var(--line);padding:4px 6px;text-align:center}small{color:var(--mut)}"
           ".box{background:var(--box);border-radius:8px;padding:10px 14px;margin:10px 0}.concl{background:var(--good)}code{font-size:12px}")
    h = [f"<!doctype html><html lang='ko'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
         f"<title>밤샘 VIO 결과</title><style>{css}</style></head><body><h1>밤샘 VIO 실험 결과 (2026-10-20 → 21)</h1>"]
    h.append(f"<div class='box concl'>{concl}</div>" if concl else "<div class='box'>결론 작성 전 (자동 표만)</div>")
    h.append("<div class='box'><b>주의 — 정답 축척은 추정</b>: 보드가 노트북 화면이라 칸 크기 40.70 mm 는 IMU 로 추정한 값이다 (자로 안 잼). "
             "SE3 ATE·10 cm 문턱·Sim3 축척 판정은 이 값에 기대어 있다. 아침에 화면 폭을 재서 <code>imu_learn/rescale_gt.py</code> 로 정답을 고친 뒤 "
             "<code>../venv_ffs/bin/python imu_learn/c5_overnight_analyse.py runs/overnight_vio --html docs/overnight_vio_result.html --concl runs/overnight_vio/conclusion.html</code> "
             "를 다시 돌리면 모든 숫자가 새 축척으로 다시 계산된다 (VIO 를 다시 돌릴 필요 없음 — 포즈를 저장해 둠). "
             "<code>--k s_true/s_est</code> 로 미리 볼 수도 있다.</div>")
    h.append(f"<h2>1. 무엇을 돌렸나</h2><p>실행 {res['n_runs']} 개, 실패 {len(res['failed'])} 개, 코드·설정·바이너리 버전 "
             f"{'섞임 (판정 무효!)' if res['mixed_versions'] else '하나'}. 가장 좋은 초기화 설정 (사전 규칙으로 고름): <b>{fmt(res['best'])}</b>. "
             f"녹화: {', '.join(r[-6:] for r in res['recs'])}; 처음 정지 있는 녹화: {', '.join(r[-6:] for r in res['recs_still'])}.</p>")
    h.append("<table><tr><th>녹화</th><th>처음 정지 (s)</th><th>움직임 시작 (s, 클립 기준)</th><th>영상 길이 (s)</th><th>정지 자이로 평균 (보정판, °/s)</th><th>배터리 °C</th></tr>")
    for rec, s in sorted(res["still"].items()):
        g = s.get("gyro_mean_cal_rad_s")
        h.append(f"<tr><td>{rec[-6:]}{' (바이어스는 ' + s['bias_from'][-6:] + ')' if s.get('bias_from') else ''}</td><td>{s['still_s']}</td>"
                 f"<td>{fmt(s.get('motion_start_s'))}</td><td>{s.get('video_end_after_clip_s')}</td>"
                 f"<td>{fmt(np.round(np.degrees(g), 3).tolist() if g else None)}</td><td>{fmt((s.get('battery') or {}).get('first_c'))}→{fmt((s.get('battery') or {}).get('last_c'))}</td></tr>")
    h.append("</table>")
    h.append("<h2>2. 설정별 녹화별 표</h2><table><tr><th>설정</th><th>녹화</th><th>실행</th><th>초기화</th><th>성공</th><th>무너짐</th>"
             "<th>ATE 중앙값 (성공, cm)</th><th>Sim3 ATE</th><th>Sim3 축척</th><th>10 s 창 ATE</th><th>초기화 시각 (s)</th><th>추적 비율</th></tr>")
    for key, s in res["settings"].items():
        st, rec = key.split("|")
        h.append(f"<tr><td>{st}</td><td>{rec[-6:]}</td><td>{s['n']}</td><td>{s['n_init']}</td><td>{s['n_success']}</td><td>{s['n_collapsed']}</td>"
                 f"<td>{fmt(s['ate_med'])}</td><td>{fmt(s['ate_sim3_med'])}</td><td>{fmt(s['scale_med'], 3)}</td><td>{fmt(s['ate_win10_med'])}</td>"
                 f"<td>{fmt(s['t_init_med'])}</td><td>{fmt(s['tracked_med'])}</td></tr>")
    h.append("</table><small>성공 = 초기화 뒤 첫 10 s 창 Sim3 축척 0.5~2 그리고 SE3 ATE < 10 cm (계획 정의). ATE = 초기화 뒤 전체, SE3 정렬. "
             "초기화 시각 = 클립 시작부터.</small>")
    c5 = res["C5"]
    h.append(f"<h2>3. C5 (엄격한 정지 초기화가 무너짐을 절반 이하로 줄이나)</h2><p>strict 무너짐 {c5['collapsed_strict']}/{c5['labelled_strict']}, "
             f"loose 무너짐 {c5['collapsed_loose']}/{c5['labelled_loose']}; 녹화 단위 strict 나음 {c5['recs_strict_better']}, 나쁨 {c5['recs_strict_worse']}, "
             f"p = {fmt(c5['sign_p'], 3)}, 시험 안 된 녹화 {c5['untested_recs']} → <b>{html.escape(c5['verdict'])}</b></p>")
    h.append("<h3>보조: strict ATE vs loose (정지 있는 녹화)</h3>" + cmp_table(res["A_strict_vs_loose_ate"], "loose", "strict"))
    h.append("<h3>보조: strict-dyn vs loose</h3>" + cmp_table(res["A_strictdyn_vs_loose"], "loose", "strict-dyn"))
    for key, title, xl, yl in (("B_raw_vs_cal", "4. 원본 IMU (16/35) vs 보정판 (1/4)", "보정판", "원본"),
                               ("C_oldphys", "5. 물리 보정 (옛 모니터 녹화로 학습) vs 보정 없음", "없음", "옛 물리 보정"),
                               ("C_stillbias", "5b. 처음 정지 구간 자이로 바이어스만 vs 보정 없음", "없음", "정지 바이어스"),
                               ("N_allan", "6. 잡음: Allan (보조) vs 앱 기본", "앱 기본", "Allan"),
                               ("C2_raw_stillbias", "6b. 원본 IMU + 정지 바이어스 vs 원본 IMU (보조)", "원본", "원본+정지 바이어스")):
        if key in res:
            h.append(f"<h2>{title}</h2>" + cmp_table(res[key], xl, yl))
    h.append("<h2>7. 결정성 (같은 실행 두 번, 부하 다름)</h2><table><tr><th>실행</th><th>부하 (1분)</th><th>ATE</th><th>초기화 시각</th><th>성공</th></tr>")
    for x in res["determinism"]:
        h.append(f"<tr><td>{x['name']}</td><td>{fmt([(l or ['?'])[0] for l in x['load']])}</td><td>{fmt(x['ate_full'])}</td>"
                 f"<td>{fmt(x['t_init'])}</td><td>{fmt(x['success'])}</td></tr>")
    h.append("</table>")
    h.append("<h2>8. 축척 민감도 (정답 칸 크기가 다르면 판정이 바뀌나)</h2><table><tr><th>가정</th><th>k</th><th>C5</th><th>원본 vs 보정판</th><th>옛 물리 보정</th><th>정지 바이어스</th></tr>")
    for lab, (kk, r) in alt.items():
        h.append(f"<tr><td>{lab}</td><td>{kk:.3f}</td><td>{html.escape(r['C5']['verdict'])}</td>"
                 + "".join(f"<td>{html.escape(r[x]['overall']) if x in r else '–'}</td>" for x in ("B_raw_vs_cal", "C_oldphys", "C_stillbias"))
                 + "</tr>")
    h.append("</table>")
    if res["failed"]:
        h.append("<h2>실패·빠진 실행</h2><ul>" + "".join(f"<li>{html.escape(f['name'])}: {html.escape(f['why'])}</li>" for f in res["failed"]) + "</ul>")
    h.append("<p><small>사전 등록: docs/overnight_20261020_plan.md + docs/overnight_vio_prereg.md. 코드: imu_learn/c5_overnight.py, c5_run.py, "
             "c5_still.py, c5_overnight_analyse.py, vio_bench_hs.py. 결과: runs/overnight_vio/ (실행마다 JSON + 포즈 + 로그), summary.json.</small></p></body></html>")
    Path(path).write_text("".join(h))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--k", type=float, default=1.0, help="what-if GT scale (s_true / s_est)")
    ap.add_argument("--out", default=None)
    ap.add_argument("--html", default=None)
    ap.add_argument("--concl", default=None)
    o = ap.parse_args()
    res = summarize(o.dir, o.k)
    alt = {}
    if o.html:
        for lab, kk in K_ALT.items():
            _r = summarize(o.dir, o.k * kk)
            alt[lab] = (o.k * kk, {x: _r[x] for x in ("C5", "B_raw_vs_cal", "C_oldphys", "C_stillbias") if x in _r})
    res["k_sensitivity"] = {lab: dict(k=v[0], **{x: (v[1][x]["verdict"] if x == "C5" else v[1][x]["overall"]) for x in v[1]})
                            for lab, v in alt.items()}
    out = o.out or str(Path(o.dir) / "summary.json")
    json.dump(res, open(out, "w"), indent=1, ensure_ascii=False, default=float)
    if o.html:
        concl = Path(o.concl).read_text() if o.concl and Path(o.concl).exists() else ""
        write_html(res, alt, o.html, concl)
    print(json.dumps(dict(n_runs=res["n_runs"], failed=len(res["failed"]), mixed=res["mixed_versions"], best=res["best"],
                          C5=res["C5"]["verdict"], **{x: res[x]["overall"] for x in ("B_raw_vs_cal", "C_oldphys", "C_stillbias",
                                                                                    "N_allan") if x in res},
                          k_sens=res["k_sensitivity"]), ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
