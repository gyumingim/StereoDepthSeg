#!/usr/bin/env python3
"""Separability of GT-free collapse signals (collapse_signals.py output) -> summary JSON (+ SVG plots for the HTML).

Online detector model: alarm at the first frame t >= t_min (s after init) where signal >= threshold.
  TP  = collapsed run alarmed within 10 s after init      (detection time = alarm t)
  FA  = healthy run alarmed anywhere in its post-init stream (up to ~30 s after init)
Thresholds: an a-priori one (physics / known sensor numbers, chosen before looking) and the best on this data
(max TP - 2 FA, then earliest median detection) -> the data one is optimistic (same runs pick and score it).

  ../venv_ffs/bin/python imu_learn/collapse_report.py runs/collapse/signals.json --out runs/collapse/report.json
"""
import argparse
import json
from pathlib import Path

import numpy as np

# name, series key, t_min (s after init), a-priori threshold, unit, description
SIGNALS = [
    ("noupd", "noupd_s", 0.0, 3.0, "s", "카메라 갱신(MSCKF/SLAM) 없이 지난 시간"),
    ("sig_v", "sig_v", 3.0, 0.03, "m/s", "필터가 믿는 속도 표준편차 (공분산)"),
    ("sig_p", "sig_p", 3.0, 0.15, "m", "필터가 믿는 위치 표준편차 (공분산)"),
    ("dv_acc", "dv_acc1", 0.5, 0.3, "m/s²", "1 s 동안 카메라가 고친 속도 ÷ 1 s (IMU 와 안 맞는 가속도)"),
    ("dv_nis", "dv_nis1", 0.5, 1.0, "-", "속도 수정량 ÷ 속도 표준편차 (1 s 평균)"),
    ("bg", "bg_dps", 0.0, 1.0, "°/s", "|자이로 바이어스 추정|"),
    ("v", "v", 0.0, 1.0, "m/s", "|속도 추정| (손에 든 폰)"),
    ("chi2_rej", "chi2_rej1", 0.5, 0.3, "-", "chi2 검사에서 버린 특징 비율 (1 s)"),
    ("zupt_v", "zupt_v2", 0.5, 0.3, "m/s", "영상은 '멈춤'인데 필터 속도가 큰 정도 (ZUPT 때 |v|, 2 s)"),
]
# thresholds frozen 2026-10-20 12:49 / 12:55 (runs/collapse_progress.log) on the start-offset-0 runs, before the
# offset runs (test set) finished
FROZEN = [("noupd_s", 0.0, 3.0), ("sig_v", 3.0, 0.03), ("dv_acc1", 0.5, 0.3), ("chi2_rej1", 0.5, 0.3),
          ("zupt_v2", 0.5, 0.3)]
NO_CHI2 = [r for r in FROZEN if r[0] != "chi2_rej1"]   # chi2 rule dropped after the SLAM-chunk fix (3 false alarms)


def alarm_time(r, key, tmin, thr, tmax=None):
    s = r["series"]
    if key not in s:
        return None, False
    t = np.asarray(s["t"], float)
    x = np.asarray(s[key], float)
    m = (t >= tmin) & np.isfinite(x)
    if tmax is not None:
        m &= t <= tmax
    hit = np.nonzero(m & (x >= thr))[0]
    return (float(t[hit[0]]) if len(hit) else None), True


def score(R, rules):
    """OR of (key, tmin, thr) rules. Alarm window = first 10 s after init for BOTH classes (the label covers 10 s);
    fa_full = healthy runs alarmed anywhere in their post-init stream (~20-29 s). Runs lacking every rule's series skip."""
    tp, fa, fa_full, det, n_c, n_h, fa_names, miss = 0, 0, 0, [], 0, 0, [], []
    for r in R:
        ts, ts_full, any_ok = [], [], False
        for key, tmin, thr in rules:
            at, ok = alarm_time(r, key, tmin, thr, 10.0)
            any_ok |= ok
            if at is not None:
                ts.append(at)
            af, _ = alarm_time(r, key, tmin, thr, None)
            if af is not None:
                ts_full.append(af)
        if not any_ok:
            continue
        at = min(ts) if ts else None
        if r["collapsed"]:
            n_c += 1
            if at is not None:
                tp += 1
                det.append(at)
            else:
                miss.append(r["name"])
        else:
            n_h += 1
            if at is not None:
                fa += 1
                fa_names.append(r["name"])
            if ts_full:
                fa_full += 1
    return dict(rules=rules, tp=tp, n_c=n_c, fa=fa, n_h=n_h, fa_full=fa_full,
                det_med=float(np.median(det)) if det else None, det_p90=float(np.percentile(det, 90)) if det else None,
                det_max=float(np.max(det)) if det else None, fa_runs=fa_names, missed=miss)


def evaluate(R, key, tmin, thr):
    e = score(R, [(key, tmin, thr)])
    e["thr"] = thr
    return e


def best(R, key, tmin):
    vals = []
    for r in R:
        s = r["series"]
        if key in s:
            t = np.asarray(s["t"], float)
            x = np.asarray(s[key], float)
            vals += list(x[(t >= tmin) & (t <= 10) & np.isfinite(x)])
    if not vals:
        return None
    cands = np.unique(np.quantile(vals, np.linspace(0.5, 0.999, 200)))
    out = None
    for c in cands:
        e = evaluate(R, key, tmin, float(c))
        score = (e["tp"] - 2 * e["fa"], -(e["det_med"] or 99))
        if out is None or score > out[0]:
            out = (score, e)
    return out[1]


def combo(R, rules):
    return score(R, rules)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("signals")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    R = [r for r in json.load(open(a.signals)) if "series" in r and r.get("collapsed") is not None]
    train = [r for r in R if not r.get("start_off")]
    # a start offset often gives the SAME init (static init fires at motion onset): a test run whose absolute init time
    # is within 0.1 s of a train run of the same recording/parts is a near-copy -> 'test_new' keeps only new inits
    for r in R:
        r["t_init_abs"] = (r.get("t_init_s") or 0) + (r.get("start_off") or 0)
    test = [r for r in R if r.get("start_off")]
    for r in test:
        r["dup"] = any(q["rec"] == r["rec"] and q["parts"] == r["parts"] and abs(q["t_init_abs"] - r["t_init_abs"]) < 0.1
                       for q in train)
    test_new = [r for r in test if not r["dup"]]
    rep = dict(n=len(R), n_collapsed=sum(r["collapsed"] for r in R), n_train=len(train), n_test=len(test),
               n_test_collapsed=sum(r["collapsed"] for r in test), n_test_new=len(test_new),
               n_test_new_collapsed=sum(r["collapsed"] for r in test_new), signals=[], combos={})
    for name, key, tmin, thr, unit, desc in SIGNALS:
        row = dict(name=name, key=key, tmin=tmin, thr=thr, unit=unit, desc=desc)
        for nm, S in (("all", R), ("train", train), ("test", test), ("test_new", test_new)):
            row[nm] = evaluate(S, key, tmin, thr)
        row["best_all"] = best(R, key, tmin)
        rep["signals"].append(row)
        e, b = row["test"], row["best_all"]
        print(f"{name:9s} thr {thr:g} TEST: TP {e['tp']}/{e['n_c']} FA {e['fa']}/{e['n_h']} det med {e['det_med']} "
              f"| ALL: TP {row['all']['tp']}/{row['all']['n_c']} FA {row['all']['fa']}/{row['all']['n_h']} "
              f"| best-on-all thr {b['thr']:.4g}: TP {b['tp']} FA {b['fa']}")
    for cname, rules in (("all5", FROZEN), ("no_chi2", NO_CHI2),
                         ("cov_only", [("noupd_s", 0.0, 3.0), ("sig_v", 3.0, 0.03)])):
        rep["combos"][cname] = {nm: combo(S, rules) for nm, S in (("all", R), ("train", train), ("test", test),
                                                                   ("test_new", test_new))}
        c = rep["combos"][cname]
        print(f"OR {cname:9s} TEST TP {c['test']['tp']}/{c['test']['n_c']} FA {c['test']['fa']}/{c['test']['n_h']} "
              f"det med {c['test']['det_med']} p90 {c['test']['det_p90']} missed {c['test']['missed']} FA {c['test']['fa_runs']}"
              f" | NEW TP {c['test_new']['tp']}/{c['test_new']['n_c']} FA {c['test_new']['fa']}/{c['test_new']['n_h']}"
              f" | ALL TP {c['all']['tp']}/{c['all']['n_c']} FA {c['all']['fa']}/{c['all']['n_h']} FA_full {c['all']['fa_full']}")
    rep["runs"] = [{k: r.get(k) for k in ("name", "rec", "parts", "start_off", "init_method", "t_init_s", "bg_init_dps",
                                          "pre_gyro_mean_dps", "scale10", "ate10_cm", "path_ratio10", "collapsed",
                                          "has_chi2_log", "init_info", "dup", "t_init_abs")} for r in R]
    json.dump(rep, open(a.out, "w"), indent=1, ensure_ascii=False)


if __name__ == "__main__":
    main()
