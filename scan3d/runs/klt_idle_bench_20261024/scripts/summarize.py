# summarize.py RUNDIR...  -> per-config table (repeats pooled as list) from klt_bench_idle outputs + freq logs
import sys, re, os, glob, collections
rows = collections.OrderedDict()
pol = lambda c: 0 if c < 4 else 1 if c < 7 else 2 if c < 9 else 3
for d in sys.argv[1:]:
    for f in sorted(glob.glob(os.path.join(d, '*.txt'))):
        b = os.path.basename(f)
        if b == 'log.txt' or b.endswith('_freq.txt') or b.endswith('_frames.txt'): continue
        tag = b[:-4]; s = open(f).read()
        m = re.search(r'idle=(\S+) klt_p50 ([\d.]+) p95 ([\d.]+) mean ([\d.]+) \| freq MHz mean pol0 (\d+) pol4 (\d+) pol7 (\d+) pol9 (\d+)', s)
        if not m: print('# no result', f); continue
        h = re.search(r'([0-9a-f]{16})  \(get', s).group(1)
        t = re.search(r'SKIN_before (\S+) SKIN_after (\S+)(?: AP_before (\S+) AP_after (\S+))?', s)
        fq = os.path.join(d, tag + '_freq.txt'); mf = []; c9 = 0; n = 0
        if os.path.exists(fq):
            for ln in open(fq):
                if ln.startswith('#'): continue
                v = ln.split(); c = int(v[7])
                if c < 0: continue
                fr = [int(x) for x in v[3:7]]; mf.append(fr[pol(c)] / 1000); n += 1; c9 += c == 9
        cfg = re.sub(r'_r\d+$', '', tag)
        rows.setdefault(cfg, []).append(dict(d=os.path.basename(d), p50=float(m.group(2)), p95=float(m.group(3)), pol9=int(m.group(8)),
            maincore=sum(mf) / len(mf) if mf else 0, on9=100.0 * c9 / n if n else 0, hash=h,
            temp='stale' if os.path.basename(d) == 'p1' else '%s->%s / AP %s->%s' % t.groups()))
print('%-22s %3s  %-14s %-14s %-12s %-10s %-9s %s' % ('config', 'n', 'p50 ms', 'p95 ms', 'maincoreMHz', 'mainOn9%', 'pol9MHz', 'SKIN / AP (HAL, per run)'))
hs = set()
for k, v in rows.items():
    j = lambda key, fmt: '/'.join(fmt % r[key] for r in v)
    print('%-22s %3d  %-14s %-14s %-12s %-10s %-9s %s' % (k, len(v), j('p50', '%.1f'), j('p95', '%.1f'), j('maincore', '%.0f'), j('on9', '%.0f'),
          j('pol9', '%d'), ' ; '.join(r['temp'] for r in v)))
    for r in v: hs.add((k.startswith('n400'), r['hash']))
print('hashes (npts400?, hash):', sorted(hs))
