#!/usr/bin/env python3
"""ovpipe phone replay summary (2026-10-26).
  python3 phone_an.py OUTDIR [REF_TAG_PREFIX...]
For every run folder OUTDIR/<tag>/ (runner ovpipe_run.sh): tl.csv (--timing-log), po.jsonl (pose + pred lines), pe.txt.
  e2e  = CLOCK_BOOTTIME when the frame's "frame" prediction line was formed (pred thread, right after the pose line =
         the end of that frame's processing) - CLOCK_BOOTTIME when the bridge had read the frame record (tl rx_boot_ns).
         = the time a frame spends inside the bridge (queue + remap + track + update), independent of where the work runs.
  feed = feed_measurement_camera ms on the main thread (with --pipe: commit/wait + update only; tracking ran earlier).
  over = frames with e2e > 16.7 ms (60 fps period) / > 33.3 ms.
  drops = bridge cam_dropped (queue overflow) at the end + drop-late count (pe.txt).
  same = pose lines identical to the first run of the same group (A/B by tag prefix A60/B60/A30/B30 -> group 60/30),
         fields ms/remap_ms/wait_ms/h/late_dropped excluded.
Writes OUTDIR/summary.json and prints a table.
"""
import gzip
import json
import os
import re
import sys

out = sys.argv[1]


def pct(v, f):
    if not v:
        return float('nan')
    v = sorted(v)
    return v[min(len(v) - 1, int(f * len(v)))]


EX = {"ms", "remap_ms", "wait_ms", "h", "late_dropped"}
rows, poses_by = [], {}
for tag in sorted(os.listdir(out)):
    d = os.path.join(out, tag)
    if not os.path.isdir(d) or not os.path.exists(os.path.join(d, 'tl.csv')):
        continue
    tl = {}
    feed, cdrop = [], 0
    with open(os.path.join(d, 'tl.csv')) as f:
        hdr = f.readline().strip().split(',')
        ix = {k: i for i, k in enumerate(hdr)}
        for line in f:
            p = line.strip().split(',')
            if len(p) < len(hdr):
                continue
            t_ns = int(p[ix['t_ns']])
            tl[t_ns] = int(p[ix['rx_boot_ns']])
            if p[ix['init']] == '1':
                feed.append(float(p[ix['feed_ms']]))
            cdrop = int(p[ix['cam_dropped']])
    e2e, poses = [], []
    pp = os.path.join(d, 'po.jsonl')
    with (open(pp) if os.path.exists(pp) else gzip.open(pp + '.gz', 'rt')) as f:
        for line in f:
            if '"pred"' in line and '"frame"' in line:
                j = json.loads(line)
                if j.get('src') == 'frame' and 'tb' in j and j.get('t_cam_ns') in tl:
                    e2e.append((j['tb'] - tl[j['t_cam_ns']]) / 1e6)
            elif '"pose"' in line:
                j = json.loads(line)
                for k in EX:
                    j.pop(k, None)
                poses.append(json.dumps(j, sort_keys=True))
    pe = open(os.path.join(d, 'pe.txt'), errors='replace').read()
    m = re.search(r'drop-late: limit [\d.]+ ms \| dropped (\d+)', pe)
    late = int(m.group(1)) if m else 0
    m = re.search(r'pipe: submitted (\d+) \| fed frames: precomputed (\d+), synchronous (\d+)', pe)
    pipe = f"{m.group(2)}/{int(m.group(2)) + int(m.group(3))}" if m else '-'
    run = {}
    if os.path.exists(os.path.join(d, 'run.txt')):
        for kv in re.findall(r"(\w+)=('[^']*'|\S+)", open(os.path.join(d, 'run.txt')).read()):
            run[kv[0]] = kv[1].strip("'")
    bat = open(os.path.join(d, 'battery_start.txt')).read() if os.path.exists(os.path.join(d, 'battery_start.txt')) else ''
    chg = 'AC' if 'AC powered: true' in bat else ('USB' if 'USB powered: true' in bat else 'no')
    grp = tag[1:3] if tag[:1] in 'AB' else tag
    ref = poses_by.setdefault(grp, poses)
    same = sum(1 for a, b in zip(ref, poses) if a == b)
    r = dict(tag=tag, frames=len(feed), poses=len(poses), same=f"{same}/{len(ref)}", e2e_n=len(e2e),
             e2e_p50=pct(e2e, .5), e2e_p95=pct(e2e, .95), e2e_p99=pct(e2e, .99), e2e_max=max(e2e) if e2e else float('nan'),
             over16=sum(x > 16.7 for x in e2e), over33=sum(x > 33.3 for x in e2e),
             feed_p50=pct(feed, .5), feed_p95=pct(feed, .95), feed_max=max(feed) if feed else float('nan'),
             cam_dropped=cdrop, late_dropped=late, pipe=pipe, charging=chg, skin_start=run.get('skin_start'),
             skin_max=run.get('skin_max'), start=run.get('start'), aborted=run.get('aborted'))
    rows.append(r)
print(f"{'tag':10s} {'fed':>5s} {'same':>11s} {'e2e p50/p95/p99/max ms':>26s} {'>16.7':>6s} {'>33':>4s} "
      f"{'feed p50/p95/max':>18s} {'drop':>5s} {'late':>4s} {'pipe':>10s} chg skin")
for r in rows:
    print(f"{r['tag']:10s} {r['frames']:5d} {r['same']:>11s} {r['e2e_p50']:6.2f}/{r['e2e_p95']:5.2f}/{r['e2e_p99']:5.2f}/{r['e2e_max']:6.1f} "
          f"{r['over16']:6d} {r['over33']:4d} {r['feed_p50']:5.2f}/{r['feed_p95']:5.2f}/{r['feed_max']:5.1f} "
          f"{r['cam_dropped']:5d} {r['late_dropped']:4d} {r['pipe']:>10s} {r['charging']} {r['skin_start']}-{r['skin_max']}")
json.dump(rows, open(os.path.join(out, 'summary.json'), 'w'), indent=1)
