# ovdrone phone replay summary: per run proc = remap_ms + feed_ms (initialised frames), prof "rs" slot, stderr summaries
import csv, sys, os, re, statistics as st
D = sys.argv[1]
def q(x, f): x = sorted(x); return x[min(len(x)-1, int(f*len(x)))] if x else float('nan')
print("| run | frames | proc p50 / p95 ms | > 33 ms | rs slot mean / p95 ms | battery | SKIN start/max | bridge summary |")
print("|---|---|---|---|---|---|---|---|")
for run in sorted(os.listdir(D)):
    p = os.path.join(D, run)
    if not os.path.isdir(p): continue
    rows = list(csv.DictReader(open(os.path.join(p, 'tl.csv'))))
    pr = [float(r['remap_ms']) + float(r['feed_ms']) for r in rows if r['init'] == '1']
    rs = []
    if os.path.exists(os.path.join(p, 'prof.csv')):
        pf = list(csv.DictReader(open(os.path.join(p, 'prof.csv'))))
        rs = [float(r['rs']) for r in pf if r.get('rs') not in (None, '')]
    bat = open(os.path.join(p, 'battery_start.txt')).read()
    m = re.search(r'^  status: (\d)', bat, re.M); ac = re.search(r'AC powered: (\w+)', bat)
    run_txt = open(os.path.join(p, 'run.txt')).read()
    s0 = re.search(r'skin_start=([\d.]+)', run_txt).group(1); sm = re.search(r'skin_max=([\d.]+)', run_txt).group(1)
    pe = open(os.path.join(p, 'pe.txt'), errors='replace').read()
    summ = '; '.join(l.split('] ', 1)[1][:150] for l in pe.splitlines() if re.match(r'\[ov_bridge\] (rs_derot: readout .*\||predict:|viop: udp .*sent|mav: udp .*sent)', l))
    print(f"| {run} | {len(pr)} | {q(pr,.5):.1f} / {q(pr,.95):.1f} | {100*sum(x>33 for x in pr)/max(1,len(pr)):.1f} % | "
          f"{(st.mean(rs) if rs else float('nan')):.3f} / {q(rs,.95):.3f} | status {m.group(1) if m else '?'} ac {ac.group(1) if ac else '?'} | {s0}/{sm} | {summ} |")
