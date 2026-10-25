#!/usr/bin/env python3
"""Table for one ispprobe run dir (phone_stereo/ispprobe/run.sh pulls it to scan3d/runs/isp_probe/<run>/).

lat   = onImageAvailable elapsedRealtimeNanos - SENSOR_TIMESTAMP (start of exposure, first row; REALTIME clock assumed
        only when caps.json timestamp_source == REALTIME)
isp   = lat - exposure - rolling skew (time after the last row was read out; HAL/ISP + delivery)
shut  = onCaptureStarted callback - sensor ts, res = onCaptureCompleted callback - sensor ts
Frames in the first WARM s (default 1.0) of each case are skipped (start-up); drops = sensor-ts gaps > 1.5 periods.
"""
import csv, json, os, sys
import numpy as np

WARM = float(os.environ.get("WARM", "1.0"))


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float("nan")


def load(d, name):
    rows = []
    with open(os.path.join(d, name + ".csv")) as f:
        for r in csv.DictReader(f):
            rows.append({k: int(v) for k, v in r.items()})
    return rows


def main(d):
    plan = json.load(open(os.path.join(d, "plan.json")))
    caps = json.load(open(os.path.join(d, "caps.json")))
    print(f"run {os.path.basename(d.rstrip('/'))}  device {plan.get('device')} sdk {plan.get('sdk')}  "
          f"iso {plan['iso']} exp_us {plan['exp_us']} secs {plan['secs']} warm-up skipped {WARM} s")
    print(f"caps: timestamp_source {caps.get('timestamp_source')}  pipeline_max_depth {caps.get('request_pipeline_max_depth')}  "
          f"partial_results {caps.get('request_partial_result_count')}  readout_ts {caps.get('sensor_readout_timestamp')}  "
          f"hw_level {caps.get('hardware_level')}  RAW capability {caps.get('capability_raw')}")
    print(f"      RAW streams (logical 0): " + "; ".join(f"{k}: {len(v)} sizes" + (f" e.g. {v[0]}" if v else "")
                                                       for k, v in caps.get("raw", {}).items()))
    print(f"      NR {caps.get('nr_modes')} EDGE {caps.get('edge_modes')} HOTPIX {caps.get('hot_pixel_modes')} "
          f"TONEMAP {caps.get('tonemap_modes')} vendor keys {len(caps.get('vendor_request_keys', []))}")
    if caps.get("timestamp_source") != "REALTIME":
        print("WARNING: timestamp source is not REALTIME -> lat values are not comparable to elapsedRealtimeNanos")
    if plan.get("dry"):
        print("\nDRY run (camera never opened). Planned cases:")
        for c in plan["cases"]:
            print(f"  {c['name']:18s} tmpl {c['template']} {c['w']}x{c['h']} fmt {c['fmt']} {c['fps']} fps  isp {c['isp']}"
                  f"  {'SKIP: ' + c['skip'] if c['skip'] else ''} {c['note']}")
        return
    hdr = (f"{'case':18s} {'status':8s} {'fps':>5s} {'n':>4s} {'drop':>4s} {'lat p50':>7s} {'p95':>6s} {'max':>6s} "
           f"{'isp p50':>7s} {'d base':>6s} {'shut':>5s} {'res':>5s} {'depth':>7s} {'skew':>5s} {'fdur':>5s} {'exp':>5s}  NR/EDGE(res)")
    print("\n(ms)\n" + hdr + "\n" + "-" * len(hdr))
    base = None
    for c in plan["cases"]:
        name = c["name"]
        if c["skip"]:
            print(f"{name:18s} SKIPPED  {c['skip']}")
            continue
        if not os.path.exists(os.path.join(d, name + ".csv")):
            print(f"{name:18s} no csv")
            continue
        m = json.load(open(os.path.join(d, name + ".json")))
        rows = load(d, name)
        img = [r for r in rows if r["t_image_cb_ns"] > 0]
        if not img:
            print(f"{name:18s} {m.get('status', '?')[:40]}  no images (results {m.get('n_results')})")
            continue
        t0 = img[0]["t_sensor_ns"]
        use = [r for r in img if r["t_sensor_ns"] - t0 >= WARM * 1e9]
        ts = np.array([r["t_sensor_ns"] for r in img], dtype=np.int64)
        lat = np.array([(r["t_image_cb_ns"] - r["t_sensor_ns"]) / 1e6 for r in use])
        isp = np.array([(r["t_image_cb_ns"] - r["t_sensor_ns"] - r["exp_ns"] - r["skew_ns"]) / 1e6
                        for r in use if r["exp_ns"] > 0 and r["skew_ns"] > 0])
        shut = np.array([(r["t_shutter_cb_ns"] - r["t_sensor_ns"]) / 1e6 for r in use if r["t_shutter_cb_ns"] > 0])
        res = np.array([(r["t_result_cb_ns"] - r["t_sensor_ns"]) / 1e6 for r in use if r["t_result_cb_ns"] > 0])
        dep = np.array([r["pipeline_depth"] for r in use if r["pipeline_depth"] >= 0])
        skew = np.array([r["skew_ns"] / 1e6 for r in use if r["skew_ns"] > 0])
        fd = np.array([r["frame_dur_ns"] / 1e6 for r in use if r["frame_dur_ns"] > 0])
        ex = np.array([r["exp_ns"] / 1e6 for r in use if r["exp_ns"] > 0])
        per = 1e9 / c["fps"]
        gaps = np.diff(ts)
        drops = int(sum(round(g / per) - 1 for g in gaps if g > 1.5 * per))
        fps = (len(ts) - 1) / ((ts[-1] - ts[0]) / 1e9) if len(ts) > 1 else float("nan")
        p50 = pct(lat, 50)
        if name == "base":
            base = p50
        dbase = f"{p50 - base:+6.1f}" if base is not None and name != "base" else "     -"
        fr = m.get("first_result", {})
        depth = f"{int(np.median(dep))}({dep.min()}-{dep.max()})" if len(dep) else "-"
        st = m.get("status", "?")
        st = "ok" if st == "ok" else st[:8]
        extra = f" fail {m['n_failed']} lost {m['n_buffer_lost']}" if m.get("n_failed") or m.get("n_buffer_lost") else ""
        print(f"{name:18s} {st:8s} {fps:5.1f} {len(img):4d} {drops:4d} {p50:7.1f} {pct(lat, 95):6.1f} {lat.max() if len(lat) else float('nan'):6.1f} "
              f"{pct(isp, 50):7.1f} {dbase} {pct(shut, 50):5.1f} {pct(res, 50):5.1f} {depth:>7s} {pct(skew, 50):5.2f} "
              f"{pct(fd, 50):5.1f} {pct(ex, 50):5.2f}  {fr.get('nr')}/{fr.get('edge')}{extra}")
    print("\nNR/EDGE: 0 OFF 1 FAST 2 HIGH_QUALITY 3 MINIMAL/ZSL 4 ZSL(edge). d base = lat p50 - base lat p50. "
          "Compare base vs base_end for drift before trusting small differences (one run per case).")


if __name__ == "__main__":
    main(sys.argv[1])
