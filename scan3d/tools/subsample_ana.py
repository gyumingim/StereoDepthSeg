#!/usr/bin/env python3
"""subsample_test.ps1 결과 폴더 요약 (앱 1.8-subsample 실측): 저장 fps, 짝 시각차, 카메라 결과 /s, 노출, CPU, 온도, 용량.

  python3 scan3d/tools/subsample_ana.py scan3d/runs/subsample_<이름> [...]
"""
import csv, json, sys, glob, os, statistics as st

def summary(d):
    out = {}


    def rows(p):
        with open(p, encoding="utf-8-sig") as f:
            return list(csv.DictReader(f))


    r2 = rows(os.path.join(d, "frames_cam2.csv"))
    r5 = rows(os.path.join(d, "frames_cam5.csv"))
    t2 = [int(r["t_mid_ns"]) for r in r2]
    t5 = [int(r["t_mid_ns"]) for r in r5]
    dur = (max(t2[-1], t5[-1]) - min(t2[0], t5[0])) / 1e9
    out["saved_frames"] = [len(t2), len(t5)]
    out["saved_fps"] = round(len(t2) / dur, 2)
    # nearest-pair |dt| (what the PC pairer sees)
    dts = []
    j = 0
    for t in t2:
        while j + 1 < len(t5) and abs(t5[j + 1] - t) <= abs(t5[j] - t):
            j += 1
        dts.append(abs(t5[j] - t) / 1e6)
    dts.sort()
    q = lambda a, x: a[min(len(a) - 1, int(round(x * (len(a) - 1))))]
    out["pair_dt_ms"] = dict(median=round(q(dts, .5), 2), p95=round(q(dts, .95), 2), max=round(dts[-1], 2),
                             over_36ms=sum(1 for x in dts if x > 36.67))
    gaps = [(b - a) / 1e6 for a, b in zip(t2, t2[1:])]
    out["ref_interval_ms"] = dict(median=round(st.median(gaps), 2), max=round(max(gaps), 2))
    for name, rr in (("2", r2), ("5", r5)):
        e = [int(r["exposure_ns"]) / 1e6 for r in rr if int(r["exposure_ns"]) > 0]
        out[f"exposure_ms_{name}"] = dict(median=round(st.median(e), 2), min=round(min(e), 2), max=round(max(e), 2)) if e else None
    # capture rate from frame_timing rows
    imu = os.path.join(d, "imu.csv")
    if os.path.exists(imu):
        ft = []
        with open(imu) as f:
            for line in f:
                if line.startswith("100,"):
                    p = line.strip().split(",")
                    ft.append((int(p[1]), [float(x) for x in p[2:]]))
        if ft:
            lt = [a for a, _ in ft]
            iv = [(b - a) / 1e6 for a, b in zip(lt, lt[1:])]
            out["results_per_s"] = round((len(lt) - 1) / ((lt[-1] - lt[0]) / 1e9), 2)
            out["result_interval_ms"] = dict(median=round(st.median(iv), 2), max=round(max(iv), 2),
                                             gt_1_5_period=sum(1 for x in iv if x > 1.5 * st.median(iv)))
            durs = [v[3] for _, v in ft if len(v) > 3]
            out["frame_duration_ms_median"] = round(st.median(durs), 2)


    ms = sorted(glob.glob(os.path.join(d, "metrics_*.json")))
    ser = []
    for p in ms:
        try:
            m = json.loads(open(p, encoding="utf-8-sig").read())
        except ValueError:
            continue
        ph = m.get("phone", {})
        L = m.get("lenses", {})
        ser.append(dict(cpu=ph.get("proc_cpu_pct_one_core"), temp=ph.get("battery_temp_c"),
                        head=ph.get("thermal_headroom_10s"), therm=ph.get("thermal_status"),
                        cam_fps=[L.get(k, {}).get("cam_fps") for k in ("2", "5")],
                        rec_fps=[L.get(k, {}).get("rec_fps") for k in ("2", "5")],
                        kbps=[L.get(k, {}).get("h264_kbps") for k in ("2", "5")],
                        copy=[L.get(k, {}).get("copy_ms_avg") for k in ("2", "5")],
                        gap=[L.get(k, {}).get("cam_gap_drops") for k in ("2", "5")]))
    if ser:
        cpus = [s["cpu"] for s in ser[1:] if s["cpu"] is not None]
        out["cpu_pct_one_core_median"] = round(st.median(cpus), 1) if cpus else None
        out["temp_first_last"] = [ser[0]["temp"], ser[-1]["temp"]]
        out["headroom_first_last"] = [ser[0]["head"], ser[-1]["head"]]
        out["thermal_status_max"] = max(s["therm"] or 0 for s in ser)
        out["cam_fps_median"] = [st.median([s["cam_fps"][i] for s in ser[1:] if s["cam_fps"][i]]) for i in (0, 1)]
        out["rec_fps_median"] = [st.median([s["rec_fps"][i] for s in ser[1:] if s["rec_fps"][i]]) for i in (0, 1)]
        out["h264_kbps_median"] = [st.median([s["kbps"][i] for s in ser[1:] if s["kbps"][i]]) for i in (0, 1)]
        out["copy_ms_median"] = [st.median([s["copy"][i] for s in ser[1:] if s["copy"][i] is not None]) for i in (0, 1)]
        out["cam_gap_drops_last"] = ser[-1]["gap"]
    try:
        meta = json.loads(open(os.path.join(d, "meta.json"), encoding="utf-8-sig").read())
        out["meta"] = {k: meta.get(k) for k in ("fps", "capture_fps", "save_fps", "rec_kbps", "subsample", "frames", "dropped")}
    except (OSError, ValueError):
        pass
    lsf = os.path.join(d, "ls.txt")
    if os.path.exists(lsf):
        tot = 0
        for line in open(lsf, encoding="utf-8-sig"):
            p = line.split()
            if len(p) > 4 and p[-1].endswith(".mp4"):
                tot += int(p[4])
        out["mp4_MB"] = round(tot / 1e6, 1)
        out["mp4_MB_per_min"] = round(tot / 1e6 / dur * 60, 1)
    out["duration_s"] = round(dur, 1)
    return out

if __name__ == "__main__":
    for d in sys.argv[1:]:
        print(d)
        print(json.dumps(summary(d), indent=1, ensure_ascii=False))
