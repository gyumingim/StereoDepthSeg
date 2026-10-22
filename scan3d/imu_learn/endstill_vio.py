#!/usr/bin/env python3
"""Full-length VIO on recordings with an END still (2026-10-21, docs/endstill_check.html): does VIO stay put during the
end still, and does its estimated IMU bias match the bias measured in that still?

Runs (imu_learn/c5_run.py, whole clip = --max-secs 0, init loose = best in docs/c5_retest_result.html):
  recordings 223815 / 224323 x IMU {cal = Android types 1/4, raw = types 16/35} x start offset {0, 1, 2} s (inside the
  start still, motion starts at 10.5 / 10.0 s) = 12 runs, at most 3 OpenVINS at once, decode 2 threads, nice 5.
Analysis per run (end-still window from runs/endstill_tmp/endstill_check.json "used.end", IMU clock):
  mean / max |v|, position drift |p_end - p_start| and path length in the window, VIO bg / ba (mean over the window)
  vs measured mean gyro (types 4 or 16, same as fed) and accel mean minus gravity rotated by the VIO attitude
  (ba_meas = a_mean - R_GtoI g, OpenVINS JPL convention; angle between a_mean and R_GtoI g printed as a check).
  Full-clip ATE / Sim3 scale via c5_overnight_analyse.analyse_run (GT on disk, 42.72 mm square since 2026-10-21).

  (setsid nohup ../venv_ffs/bin/python imu_learn/endstill_vio.py run > runs/endstill_vio/sched.out 2>&1 &)
  ../venv_ffs/bin/python imu_learn/endstill_vio.py analyse [--html docs/endstill_vio.html]
"""
import argparse
import gzip
import html
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
os.chdir(SCAN3D)
sys.path.insert(0, str(HERE))
PY = "../venv_ffs/bin/python"
OUT = Path("runs/endstill_vio")
LOG = OUT / "sched.log"
RECS = ["hs_20261020_223815", "hs_20261020_224323"]
IMUS = ("cal", "raw")
OFFS = (0, 1, 2)
PAR = int(os.environ.get("PAR", 3))
ENDSTILL = Path("runs/endstill_tmp/endstill_check.json")
G_MAG = 9.80                                   # make_ov_config gravity_mag (checked per run from the saved yaml)


def log(msg):
    line = f"{time.strftime('%F %T')} esv: {msg}"
    with open(LOG, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


def jobs():
    out = []
    for rec in RECS:
        for imu in IMUS:
            for o in OFFS:
                name = f"{rec[-6:]}_loose_{imu}_o{o}"
                args = [PY, "imu_learn/c5_run.py", f"recordings/{rec}", "--init-cfg", "loose", "--clones", "11",
                        "--noise", "cfg", "--start-off", str(o), "--max-secs", "0", "--decode-threads", "2",
                        "--no-gyro-rebias", "--out", str(OUT / name)] + (["--imu-raw"] if imu == "raw" else [])
                out.append(dict(name=name, args=args))
    return out


def run_all():
    OUT.mkdir(parents=True, exist_ok=True)
    files = ("imu_learn/vio_bench_hs.py", "imu_learn/c5_run.py", "imu_learn/imu_gt.py", "imu_learn/endstill_vio.py",
             "make_ov_config.py")
    head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    blobs = {f: subprocess.run(["git", "hash-object", f], capture_output=True, text=True).stdout.strip() for f in files}
    dirty = subprocess.run(["git", "status", "--porcelain", *files], capture_output=True, text=True).stdout.strip()
    js = [j for j in jobs() if not (OUT / f"{j['name']}.json").exists()]
    log(f"start git {head} dirty={bool(dirty)} PAR {PAR} jobs {len(js)} blobs {blobs}")
    running = {}
    while js or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is None:
                continue
            del running[n]
            try:
                d = json.load(open(OUT / f"{n}.json"))
                tail = f"init {d.get('t_init_clip_s')} ate {d.get('ate_cm')} scale {d.get('scale')} err {d.get('error')}"
            except Exception:  # noqa: BLE001
                lines = (OUT / f"{n}.err").read_text(errors="replace").strip().splitlines()
                tail = "FAILED: " + (lines[-1][-300:] if lines else "")
            log(f"done {n} ({time.time() - t0:.0f} s) {tail}")
        while js and len(running) < PAR:
            j = js.pop(0)
            err = open(OUT / f"{j['name']}.err", "w")
            running[j["name"]] = (subprocess.Popen(["nice", "-n", "5"] + j["args"], stdout=subprocess.DEVNULL, stderr=err),
                                  time.time())
            log(f"start {j['name']} ({len(js)} left)")
        time.sleep(10)
    log("end (all done)")


def R_GtoI(q):
    """OpenVINS JPL quaternion [x, y, z, w] -> rotation G->I (ov_core quat_2_Rot)."""
    q = np.asarray(q, float)
    v, w = q[:3], q[3]
    sk = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return (2 * w * w - 1) * np.eye(3) - 2 * w * sk + 2 * np.outer(v, v)


def load_poses(prefix):
    out = []
    with gzip.open(prefix + ".poses.jsonl.gz", "rt") as f:
        for line in f:
            m = json.loads(line)
            if m.get("status") == "tracking" and m.get("p") is not None:
                out.append(m)
    return out


def analyse_one(prefix, es):
    import c5_overnight_analyse as an
    d = json.load(open(prefix + ".json"))
    r = an.analyse_run(prefix)
    rec = d["rec"]
    imu = "raw" if d.get("imu_source", "").startswith("raw") or "_raw_" in Path(prefix).name else "cal"
    e = es["recs"][rec]
    t0 = int(e["t0_ns"])
    w0, w1 = (t0 + int(x * 1e9) for x in e["used"]["end"])
    ps = load_poses(prefix)
    t = np.array([m["t_ns"] for m in ps], np.int64)
    sel = (t >= w0) & (t <= w1)
    res = dict(name=Path(prefix).name, rec=rec, imu=imu, start_off=d.get("start_off"),
               initialized=r.get("initialized"), init_method=r.get("init_method"), t_init_clip_s=d.get("t_init_clip_s"),
               ate_full_cm=r.get("ate_full_cm"), ate_full_sim3_cm=r.get("ate_full_sim3_cm"), scale_full=r.get("scale_full"),
               collapsed_full=r.get("collapsed_full"), success_full=r.get("success_full"), tracked_frac=d.get("tracked_frac"),
               gt_square_mm=r.get("gt_square_mm"), endstill_window_s=[round((w0 - t0) / 1e9, 2), round((w1 - t0) / 1e9, 2)],
               n_poses_endstill=int(sel.sum()), last_pose_s=round((t[-1] - t0) / 1e9, 2) if len(t) else None,
               version=r.get("version"))
    gyro_key = "gyro_raw16_mdeg_s" if imu == "raw" else "gyro_cal4_mdeg_s"
    acc_key = "acc_raw35_mg" if imu == "raw" else "acc_cal1_mg"
    g_meas = np.asarray(e["end"][gyro_key]["mean"])                                  # mdeg/s
    a_meas = np.asarray(e["end"][acc_key]["mean"]) * 1e-3 * 9.80665                   # m/s^2
    res["bg_meas_mdps"] = g_meas.tolist()
    try:                                   # GT camera positions in the same window (camera clock = IMU + align dt)
        gt_t, gt_p, _ = an.gt(rec, d.get("gt") or "imu_gt_spl")
        sh = int(d["align_dt_ms"] * 1e6)
        gw = (gt_t >= w0 + sh) & (gt_t <= w1 + sh)
        res["gt_n_endstill"] = int(gw.sum())
        if gw.sum() >= 5:
            res["gt_drift_cm"] = float(np.linalg.norm(gt_p[gw][-1] - gt_p[gw][0]) * 100)
            res["gt_spread_cm"] = float(np.linalg.norm(gt_p[gw].std(0)) * 100)
    except Exception as e:  # noqa: BLE001
        res["gt_err"] = repr(e)
    if sel.sum() < 10:
        res["note"] = "VIO not tracking during the end still"
        return res
    P = np.array([ps[i]["p"] for i in np.nonzero(sel)[0]])
    V = np.array([ps[i]["v"] for i in np.nonzero(sel)[0]])
    BG = np.array([ps[i]["bg"] for i in np.nonzero(sel)[0]])
    BA = np.array([ps[i]["ba"] for i in np.nonzero(sel)[0]])
    Q = [ps[i]["q_GtoI_jpl"] for i in np.nonzero(sel)[0]]
    dur = (t[sel][-1] - t[sel][0]) / 1e9
    sp = np.linalg.norm(V, axis=1)
    res.update(endstill_dur_s=round(dur, 2), v_mean_cms=float(sp.mean() * 100), v_max_cms=float(sp.max() * 100),
               drift_cm=float(np.linalg.norm(P[-1] - P[0]) * 100),
               path_cm=float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum() * 100))
    res["drift_cm_per_s"] = res["drift_cm"] / dur if dur > 0 else None
    bg = BG.mean(0) * 180 / np.pi * 1000
    res.update(bg_vio_mdps=bg.tolist(), bg_err_mdps=(bg - g_meas).tolist(), bg_err_norm_mdps=float(np.linalg.norm(bg - g_meas)),
               bg_meas_norm_mdps=float(np.linalg.norm(g_meas)))
    gI = np.mean([R_GtoI(q) @ np.array([0, 0, G_MAG]) for q in Q], axis=0)
    ang = np.degrees(np.arccos(np.clip(gI @ a_meas / np.linalg.norm(gI) / np.linalg.norm(a_meas), -1, 1)))
    ba_meas = a_meas - gI
    ba = BA.mean(0)
    res.update(ba_vio_mg=(ba / 9.80665 * 1000).tolist(), ba_meas_mg=(ba_meas / 9.80665 * 1000).tolist(),
               ba_err_mg=((ba - ba_meas) / 9.80665 * 1000).tolist(),
               ba_err_norm_mg=float(np.linalg.norm(ba - ba_meas) / 9.80665 * 1000),
               gravity_angle_deg=float(ang), acc_norm_mg=float(np.linalg.norm(a_meas) / 9.80665 * 1000))
    # first tracking pose bias (at init, start of motion) for comparison
    res["bg_init_mdps"] = (np.asarray(ps[0]["bg"]) * 180 / np.pi * 1000).tolist() if ps else None
    res["bg_start_meas_mdps"] = e["start"][gyro_key]["mean"]
    return res


def fmt(v, n=2):
    if v is None:
        return "–"
    if isinstance(v, (list, tuple)):
        return "[" + ", ".join(fmt(x, n) for x in v) + "]"
    if isinstance(v, float):
        return f"{v:.{n}f}"
    return html.escape(str(v))


def med(xs):
    xs = [x for x in xs if x is not None]
    return float(np.median(xs)) if xs else None


def write_html(res, path, concl):
    css = (":root{--bg:#fff;--fg:#1d1d1f;--mut:#666;--line:#ddd;--box:#f5f6f8;--good:#e8f6ec}"
           "@media (prefers-color-scheme:dark){:root{--bg:#16171a;--fg:#e8e8ea;--mut:#9a9aa0;--line:#33343a;--box:#202226;--good:#1d3a26}}"
           "body{background:var(--bg);color:var(--fg);font:15px/1.6 system-ui,sans-serif;max-width:1100px;margin:0 auto;padding:16px}"
           "table{border-collapse:collapse;margin:8px 0 16px;font-size:13px;display:block;overflow-x:auto}"
           "th,td{border:1px solid var(--line);padding:4px 6px;text-align:center}small{color:var(--mut)}"
           ".box{background:var(--box);border-radius:8px;padding:10px 14px;margin:10px 0}.concl{background:var(--good)}code{font-size:12px}")
    h = [f"<!doctype html><html lang='ko'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
         f"<title>끝 정지 VIO</title><style>{css}</style></head><body><h1>끝 정지 구간에서 VIO 가 가만히 있나 (전체 길이, 2026-10-21)</h1>"]
    h.append(f"<div class='box concl'>{concl}</div>" if concl else "")
    h.append("<h2>1. 무엇을 돌렸나</h2><p>녹화 223815·224323 (끝에 폰을 내려놓은 정지가 있음, docs/endstill_check.html), "
             "<b>클립 전체</b> (60 s 자르기 없음), 초기화 loose (C5 재시험에서 가장 나음), IMU 두 가지 (Android 보정판 1/4, 원본 16/35), "
             f"시작 위치 0·1·2 s (모두 처음 정지 안) → {res['n_runs']} 실행. 코드 버전 {'<b>섞임</b>' if res['mixed'] else '하나'}. "
             f"정답 칸 {fmt(res.get('gt_square_mm'))} mm (노트북 사양 계산, 추정).</p>")
    h.append("<h2>2. 끝 정지 동안 움직임 (VIO 단위 ≈ m, 축척 ≈ 1)</h2><table><tr><th>실행</th><th>끝 정지 창 (s, IMU 시작 기준)</th>"
             "<th>길이 (s)</th><th>평균 |v| (cm/s)</th><th>최대 |v| (cm/s)</th><th>위치 이동 (cm)</th><th>이동/초 (cm/s)</th><th>경로 길이 (cm)</th>"
             "<th>정답 이동 (cm, 정답 포즈 수)</th><th>전체 ATE (cm)</th><th>전체 Sim3 축척</th><th>초기화</th></tr>")
    for r in res["runs"]:
        h.append(f"<tr><td>{r['name']}</td><td>{fmt(r['endstill_window_s'], 1)}</td><td>{fmt(r.get('endstill_dur_s'))}</td>"
                 f"<td>{fmt(r.get('v_mean_cms'))}</td><td>{fmt(r.get('v_max_cms'))}</td><td>{fmt(r.get('drift_cm'))}</td>"
                 f"<td>{fmt(r.get('drift_cm_per_s'))}</td><td>{fmt(r.get('path_cm'))}</td>"
                 f"<td>{fmt(r.get('gt_drift_cm'))} ({r.get('gt_n_endstill', 0)})</td><td>{fmt(r.get('ate_full_cm'))}</td>"
                 f"<td>{fmt(r.get('scale_full'), 3)}</td><td>{fmt(r.get('init_method'))} {fmt(r.get('t_init_clip_s'))} s</td></tr>")
    h.append("</table>")
    h.append("<h2>3. VIO 가 추정한 바이어스 vs 끝 정지에서 잰 바이어스</h2><table><tr><th>실행</th><th>자이로 VIO (mdeg/s, x/y/z)</th>"
             "<th>자이로 잰 값</th><th>차이 크기 (mdeg/s)</th><th>가속도 VIO (mg)</th><th>가속도 잰 값 (mg, 추정)</th><th>차이 크기 (mg)</th>"
             "<th>중력 방향 각도 차 (°)</th></tr>")
    for r in res["runs"]:
        h.append(f"<tr><td>{r['name']}</td><td>{fmt(r.get('bg_vio_mdps'), 0)}</td><td>{fmt(r.get('bg_meas_mdps'), 0)}</td>"
                 f"<td>{fmt(r.get('bg_err_norm_mdps'), 0)}</td><td>{fmt(r.get('ba_vio_mg'), 1)}</td><td>{fmt(r.get('ba_meas_mg'), 1)}</td>"
                 f"<td>{fmt(r.get('ba_err_norm_mg'), 1)}</td><td>{fmt(r.get('gravity_angle_deg'), 2)}</td></tr>")
    h.append("</table><small>자이로 잰 값 = 끝 정지 창 평균 (IMU 를 VIO 에 넣은 것과 같은 종류: 보정판은 type 4, 원본은 type 16). "
             "자이로 1 LSB = 70 mdeg/s 라 평균도 수십 mdeg/s 흔들림 (docs/endstill_check.html). "
             "가속도 잰 값 = 끝 정지 평균 − (VIO 자세로 돌린 중력 9.80) → VIO 자세에 기대므로 <b>추정</b> (기울기 1° ≈ 17 mg).</small>")
    h.append("<h2>4. 녹화·IMU 별 중앙값 (반복 3)</h2><table><tr><th>녹화</th><th>IMU</th><th>평균 |v|</th><th>위치 이동 (cm)</th>"
             "<th>자이로 차이 (mdeg/s)</th><th>가속도 차이 (mg)</th><th>전체 ATE (cm)</th><th>축척</th></tr>")
    for k, g in res["groups"].items():
        h.append(f"<tr><td>{k.split('|')[0][-6:]}</td><td>{k.split('|')[1]}</td><td>{fmt(g['v_mean_cms'])}</td><td>{fmt(g['drift_cm'])}</td>"
                 f"<td>{fmt(g['bg_err_norm_mdps'], 0)}</td><td>{fmt(g['ba_err_norm_mg'], 1)}</td><td>{fmt(g['ate_full_cm'])}</td>"
                 f"<td>{fmt(g['scale_full'], 3)}</td></tr>")
    h.append("</table>")
    if res["failed"]:
        h.append("<h2>실패</h2><ul>" + "".join(f"<li>{html.escape(x)}</li>" for x in res["failed"]) + "</ul>")
    h.append("<p><small>코드: imu_learn/endstill_vio.py (실행·분석), c5_run.py, vio_bench_hs.py, c5_overnight_analyse.py (ATE). "
             "결과: runs/endstill_vio/ (실행마다 JSON·포즈·로그, summary.json). 끝 정지 창: runs/endstill_tmp/endstill_check.json.</small></p></body></html>")
    Path(path).write_text("".join(h))


def analyse(html_path, concl):
    es = json.load(open(ENDSTILL))
    runs, failed = [], []
    for j in jobs():
        p = str(OUT / j["name"])
        if not Path(p + ".json").exists():
            failed.append(f"{j['name']}: no json")
            continue
        try:
            runs.append(analyse_one(p, es))
        except Exception as e:  # noqa: BLE001
            failed.append(f"{j['name']}: {e!r}")
    groups = {}
    for rec in RECS:
        for imu in IMUS:
            rs = [r for r in runs if r["rec"] == rec and r["imu"] == imu]
            groups[f"{rec}|{imu}"] = {k: med([r.get(k) for r in rs]) for k in
                                      ("v_mean_cms", "drift_cm", "bg_err_norm_mdps", "ba_err_norm_mg", "ate_full_cm", "scale_full")}
    vers = sorted({json.dumps(r["version"], sort_keys=True) for r in runs})
    res = dict(n_runs=len(runs), failed=failed, mixed=len(vers) > 1, versions=vers, groups=groups, runs=runs,
               gt_square_mm=next((r["gt_square_mm"] for r in runs if r.get("gt_square_mm")), None),
               made=time.strftime("%F %T"))
    json.dump(res, open(OUT / "summary.json", "w"), indent=1, ensure_ascii=False, default=float)
    if html_path:
        write_html(res, html_path, Path(concl).read_text() if concl and Path(concl).exists() else "")
    for r in runs:
        print(r["name"], fmt(r.get("v_mean_cms")), fmt(r.get("drift_cm")), "bg", fmt(r.get("bg_vio_mdps"), 0), fmt(r.get("bg_meas_mdps"), 0),
              "ba", fmt(r.get("ba_vio_mg"), 1), fmt(r.get("ba_meas_mg"), 1), "ang", fmt(r.get("gravity_angle_deg")),
              "ate", fmt(r.get("ate_full_cm")), "s", fmt(r.get("scale_full"), 3), r.get("note", ""))
    print(json.dumps(dict(groups=groups, failed=failed, mixed=res["mixed"]), indent=1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["run", "analyse"])
    ap.add_argument("--html", default=None)
    ap.add_argument("--concl", default=str(OUT / "conclusion.html"))
    o = ap.parse_args()
    run_all() if o.cmd == "run" else analyse(o.html, o.concl)


if __name__ == "__main__":
    main()
