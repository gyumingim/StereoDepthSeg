# read-only: ovpipe P_1 B60_r2 late drops - mechanism check (permanent offset shift vs transient backlog)
import csv, re, sys
D = "/home/system/scanner/StereoDepthSeg-ovpipe/scan3d/runs/ovpipe_20261026/phone/out_P1"
for r in sys.argv[1:]:
    tl = list(csv.DictReader(open(f"{D}/{r}/tl.csv")))
    fl = [(float(x["t"]), (int(x["rx_hdr_boot_ns"]) - int(x["t_ns"])) / 1e6, float(x["feed_ms"])) for x in tl]
    m = min(v for _, v, _ in fl)
    pe = open(f"{D}/{r}/pe.txt").read()
    drops = [(float(a), float(b), int(c)) for a, b, c in re.findall(r"drop-late: frame t=([\d.]+) ([\d.]+) ms late .*?#(\d+)\)", pe)]
    # gaps in fed frames = where frames were dropped
    gaps = []
    for (t0, _, _), (t1, _, _) in zip(fl, fl[1:]):
        if t1 - t0 > 0.03: gaps.append((t0, t1, round((t1 - t0) / (1 / 60.0)) - 1))
    print(f"== {r}: fed {len(fl)} t {fl[0][0]:.2f}..{fl[-1][0]:.2f}; drop log lines {len(drops)}; gaps(>1 frame) {len(gaps)} total missing ~{sum(g[2] for g in gaps)}")
    runs = []
    for g in gaps:
        if runs and g[0] - runs[-1][1] < 0.5: runs[-1] = (runs[-1][0], g[1], runs[-1][2] + g[2])
        else: runs.append(g)
    for a, b, n in runs[:30]: print(f"   drop cluster t {a:.2f}..{b:.2f} ({b-a:.2f} s) ~{n} frames")
    print("   rx lag - min (ms) at t:", " ".join(f"{tq}:{min((abs(t-tq),v) for t,v,_ in fl)[1]-m:.0f}" for tq in (2,5,10,15,16,17,18,20,25,30,35,38)))
    big = [(t, f) for t, _, f in fl if f > 100]
    print(f"   feed_ms > 100: {len(big)} e.g. {[(round(t,2), round(f)) for t, f in big[:12]]}")
    # recording is 30 fps: missing frames per gap with dt 1/30; associate with the preceding stall
    miss = []
    for (t0, _, f0), (t1, _, _) in zip(fl, fl[1:]):
        k = round((t1 - t0) * 30) - 1
        if k > 0: miss.append((t0, k, f0))
    print(f"   30fps gaps: {len(miss)} gaps, {sum(k for _,k,_ in miss)} missing frames; longest gap {max((k for _,k,_ in miss), default=0)} frames")
    print("   gaps (t, missing, feed_ms of frame before):", [(round(t,2), k, round(f)) for t, k, f in miss if k >= 4])
