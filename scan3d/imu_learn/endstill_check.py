"""시작 정지 vs 끝 정지 IMU 바이어스 비교 + 밤샘 VIO 결과의 바이어스 추정 비교 (분석 전용, VIO 실행 안 함).

python imu_learn/endstill_check.py  ->  runs/endstill_tmp/endstill_check.json
정지 판정: 자이로 raw(type 16) 0.5 s 창(0.1 s 간격)에서 축별 std < 0.01 rad/s 이고 |mean| < 0.03 rad/s.
시작 정지 = 녹화 처음부터 첫 '움직임' 창 전까지, 끝 정지 = 마지막 '움직임' 창 뒤부터 끝까지.
경계에서 0.5 s 씩 잘라내고 (움직임 꼬리, 정지 버튼 누름), 끝은 추가로 0.5 s 더 자른다.
"""
import csv, glob, gzip, json, os, sys
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RECS = ["hs_20261020_223815", "hs_20261020_224323", "hs_20261020_222747"]
R2MD = np.degrees(1.0) * 1000.0          # rad/s -> mdeg/s
G = 9.80665
LSB_MDEG = 70.0                          # 자이로 양자화 간격 (0.0012217 rad/s ≈ 70 mdeg/s)
WIN, STEP, STD_T, MEAN_T = 0.5, 0.1, 0.01, 0.03
TRIM, END_EXTRA = 0.5, 0.5
SLOPE_BATT = {"gyro": [4.888, 8.914, 5.533], "acc_mg": [0.146, -0.286, 0.185]}   # runs/gyro_bias_stability.json
SLOPE_CP = {"gyro": [3.869, 7.043, 4.426], "acc_mg": [0.114, -0.223, 0.155]}


def read_all(path):
    d = {1: [], 4: [], 16: [], 35: []}
    with open(path) as f:
        for r in csv.reader(f):
            if not r or r[0] == "type":
                continue
            try:
                t = int(r[0])
                if t in d and len(r) >= 5:
                    d[t].append([int(r[1])] + [float(x) for x in r[2:8 if t in (16, 35) else 5]])
            except (ValueError, IndexError):
                pass
    return {k: np.asarray([x for x in v if len(x) == (7 if k in (16, 35) else 4)]) for k, v in d.items()}


def still_windows(g):
    t = (g[:, 0] - g[0, 0]) * 1e-9
    starts = np.arange(0, t[-1] - WIN, STEP)
    ok = []
    for s in starts:
        m = (t >= s) & (t < s + WIN)
        if m.sum() < 20:
            ok.append(None)          # 샘플 빈칸 (type 16 은 띄엄띄엄 끊김) -> 판정 보류
            continue
        w = g[m, 1:4]
        ok.append(bool(np.all(w.std(0) < STD_T) and np.linalg.norm(w.mean(0)) < MEAN_T))
    return starts, ok


def find_stills(g, t_end):
    starts, ok = still_windows(g)
    # 시작 정지: 첫 False 창 시작까지
    i = next((k for k, v in enumerate(ok) if v is False), len(ok))
    start = (0.0, starts[i]) if i > 0 else None
    j = next((k for k in range(len(ok) - 1, -1, -1) if ok[k] is False), -1)
    end = (starts[j] + WIN, t_end) if j < len(ok) - 1 else None
    return start, end


def seg_stats(arr, t0, a, b, scale):
    t = (arr[:, 0] - t0) * 1e-9
    m = (t >= a) & (t < b)
    x = arr[m, 1:4] * scale
    n = len(x)
    mean, std = x.mean(0), x.std(0, ddof=1)
    # 1 s 블록 평균의 흩어짐 (상관된 잡음 / 느린 표류 포함, 더 정직한 불확도)
    blk = [x[(t[m] >= s) & (t[m] < s + 1.0)].mean(0) for s in np.arange(a, b - 0.999, 1.0)
           if ((t[m] >= s) & (t[m] < s + 1.0)).sum() > 50]
    blk = np.asarray(blk)
    bsem = blk.std(0, ddof=1) / np.sqrt(len(blk)) if len(blk) >= 3 else np.full(3, np.nan)
    out = {"n": int(n), "mean": mean.tolist(), "std": std.tolist(), "sem": (std / np.sqrt(n)).tolist(),
           "block_sem_1s": bsem.tolist(), "n_blocks": int(len(blk)),
           "duty": float(n / max(1, (b - a)) )}
    if arr.shape[1] >= 7:
        out["android_bias_est"] = (arr[m, 4:7] * scale).mean(0).tolist()
    return out


def temps_at(rec, t0_ns, ts):
    rows = list(csv.DictReader(open(os.path.join(ROOT, "recordings", rec, "temps.csv"))))
    e = np.array([int(r["elapsed_ns"]) for r in rows])
    bc = np.array([int(r["battery_dC"]) / 10 for r in rows])
    th = np.array([int(r["thermal_status"]) for r in rows])
    res = []
    for a, b in ts:
        m = (e >= t0_ns + a * 1e9) & (e <= t0_ns + b * 1e9)
        k_last = np.searchsorted(e, t0_ns + b * 1e9, side="right") - 1
        k_first = np.searchsorted(e, t0_ns + a * 1e9, side="right") - 1
        res.append({"battery_c_first": float(bc[max(k_first, 0)]), "battery_c_last": float(bc[max(k_last, 0)]),
                    "battery_c_mean": float(bc[m].mean()) if m.any() else float(bc[max(k_last, 0)]),
                    "thermal_status_max": int(th[max(k_first, 0):k_last + 1].max())})
    return res, {"t_s": ((e - t0_ns) * 1e-9).round(2).tolist(), "battery_c": bc.tolist(), "thermal": th.tolist()}


def angle_deg(u, v):
    c = np.dot(u, v) / np.linalg.norm(u) / np.linalg.norm(v)
    return float(np.degrees(np.arccos(np.clip(c, -1, 1))))


def imu_part(rec):
    d = read_all(os.path.join(ROOT, "recordings", rec, "imu.csv"))
    t0 = int(d[16][0, 0])                     # t 기준 = 첫 IMU 샘플 (VIO 포즈의 t 와 같음)
    t_end = (max(d[1][-1, 0], d[16][-1, 0]) - t0) * 1e-9
    st, en = find_stills(d[16], t_end)
    out = {"rec": rec, "t0_ns": t0, "t_end_s": t_end, "detected": {"start": st, "end": en}}
    segs = {}
    if st:
        segs["start"] = (st[0] + 0.2, st[1] - TRIM)
    if en:
        segs["end"] = (en[0] + TRIM, en[1] - TRIM - END_EXTRA)
    out["used"] = segs
    temps, tser = temps_at(rec, t0, list(segs.values()))
    out["temps_series"] = tser
    for (name, (a, b)), tm in zip(segs.items(), temps):
        s = {"t": [a, b], "temp": tm}
        s["gyro_raw16_mdeg_s"] = seg_stats(d[16], t0, a, b, R2MD)
        s["gyro_cal4_mdeg_s"] = seg_stats(d[4], t0, a, b, R2MD)
        s["acc_raw35_mg"] = seg_stats(d[35], t0, a, b, 1000 / G)
        s["acc_cal1_mg"] = seg_stats(d[1], t0, a, b, 1000 / G)
        out[name] = s
    if "start" in out and "end" in out:
        S, E = out["start"], out["end"]
        dT = E["temp"]["battery_c_mean"] - S["temp"]["battery_c_mean"]
        cmp = {"dT_battery_c": dT}
        for key in ("gyro_raw16_mdeg_s", "gyro_cal4_mdeg_s", "acc_raw35_mg", "acc_cal1_mg"):
            dm = np.subtract(E[key]["mean"], S[key]["mean"])
            u_sem = np.hypot(E[key]["sem"], S[key]["sem"])
            u_blk = np.hypot(E[key]["block_sem_1s"], S[key]["block_sem_1s"])
            cmp[key] = {"delta": dm.tolist(), "u_sem": u_sem.tolist(), "u_block": u_blk.tolist()}
        g0, g1 = np.array(S["acc_raw35_mg"]["mean"]), np.array(E["acc_raw35_mg"]["mean"])
        cmp["gravity_angle_deg"] = angle_deg(g0, g1)
        cmp["gravity_norm_mg"] = [float(np.linalg.norm(g0)), float(np.linalg.norm(g1))]
        cmp["accel_comparable"] = cmp["gravity_angle_deg"] < 1.0
        cmp["pred_gyro_batt"] = (np.array(SLOPE_BATT["gyro"]) * dT).tolist()
        cmp["pred_gyro_cp_slope_x_dTbatt"] = (np.array(SLOPE_CP["gyro"]) * dT).tolist()
        cmp["pred_acc_batt_mg"] = (np.array(SLOPE_BATT["acc_mg"]) * dT).tolist()
        cmp["android_bias_est_delta"] = np.subtract(E["gyro_raw16_mdeg_s"]["android_bias_est"],
                                                    S["gyro_raw16_mdeg_s"]["android_bias_est"]).tolist()
        # 실측 변화를 배터리 기울기로 나눈 '실효 온도 상승' (칩 온도가 배터리보다 얼마나 더 올랐나, 추정)
        cmp["implied_dT_from_batt_slope"] = (np.array(cmp["gyro_raw16_mdeg_s"]["delta"]) / SLOPE_BATT["gyro"]).tolist()
        cmp["implied_dT_from_cp_slope"] = (np.array(cmp["gyro_raw16_mdeg_s"]["delta"]) / SLOPE_CP["gyro"]).tolist()
        out["compare"] = cmp
    return out, tser


def vio_part(rec, imu):
    short = rec.split("_")[-1]
    rows = []
    for f in sorted(glob.glob(os.path.join(ROOT, "runs/overnight_vio", f"{short}_*_o[0-9].json"))):
        j = json.load(open(f))
        cfg = os.path.basename(f)[len(short) + 1:-5]
        pf = f[:-5] + ".poses.jsonl.gz"
        first = last = None
        n = 0
        with gzip.open(pf, "rt") as g:
            for l in g:
                p = json.loads(l)
                if p.get("init") and p.get("bg") is not None:
                    if first is None:
                        first = p
                    last = p
                    n += 1
        calib_bg = (j.get("calib_used") or {}).get("bg") or [0, 0, 0]
        # c5_run 보정은 w' = w + calib_bg (calib_bg = -정지평균) -> VIO 의 bg 는 남은 바이어스. 원래 축 기준 합 = bg - calib_bg
        tot = lambda p: (np.array(p["bg"]) - np.array(calib_bg)) * R2MD
        r = {"cfg": cfg, "imu_source": j.get("imu_source"), "calib_label": j.get("calib_label"),
             "ate_cm": j.get("ate_cm"), "clip_s": j.get("clip_s"), "start_off": j.get("start_off")}
        if first is None:
            r["note"] = "no init"
        else:
            vlast = [np.linalg.norm(p) for p in [last["v"]]]
            r.update({"t_init": first["t"], "t_last": last["t"],
                      "bg_init_mdeg_s": tot(first).tolist(), "bg_last_mdeg_s": tot(last).tolist(),
                      "bg_delta_mdeg_s": (tot(last) - tot(first)).tolist(),
                      "ba_init_mg": (np.array(first["ba"]) * 1000 / G).tolist(),
                      "ba_last_mg": (np.array(last["ba"]) * 1000 / G).tolist(),
                      "covers_end_still": bool(imu.get("used", {}).get("end") and last["t"] >= imu["used"]["end"][0])})
        rows.append(r)
    return rows


def batt_at(tser, t):
    ts, bc = np.array(tser["t_s"]), np.array(tser["battery_c"])
    k = np.searchsorted(ts, t, side="right") - 1
    return float(bc[max(k, 0)])


def main():
    os.makedirs(os.path.join(ROOT, "runs/endstill_tmp"), exist_ok=True)
    res = {"settings": {"win_s": WIN, "step_s": STEP, "std_thr": STD_T, "mean_thr": MEAN_T, "trim_s": TRIM,
                        "end_extra_trim_s": END_EXTRA, "lsb_mdeg_s": LSB_MDEG,
                        "slopes_battery": SLOPE_BATT, "slopes_cp": SLOPE_CP}, "recs": {}}
    for rec in RECS:
        imu, tser = imu_part(rec)
        if rec != "hs_20261020_222747":
            imu["vio"] = vio_part(rec, imu)
            for r in imu["vio"]:
                if "t_last" in r:
                    r["battery_c_init"] = batt_at(tser, r["t_init"])
                    r["battery_c_last"] = batt_at(tser, r["t_last"])
        res["recs"][rec] = imu
        print(rec, "start", imu["used"].get("start"), "end", imu["used"].get("end"), file=sys.stderr)
    json.dump(res, open(os.path.join(ROOT, "runs/endstill_tmp/endstill_check.json"), "w"), indent=1, default=float)


if __name__ == "__main__":
    main()
