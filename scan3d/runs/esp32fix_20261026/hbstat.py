# heartbeat gaps from viop_rx.py hb.csv (PC receive clock): count, rate, p50/p95/max, gaps > 60 / 75 ms, top 3, flags seen
import csv, sys
r = list(csv.DictReader(open(sys.argv[1])))
t = [int(x["pc_ns"]) for x in r]
g = sorted((t[i + 1] - t[i]) / 1e6 for i in range(len(t) - 1))
def p(f): return round(g[min(len(g) - 1, int(f * len(g)))], 2)
fl = {}
for x in r: fl[x["flags"]] = fl.get(x["flags"], 0) + 1
print(sys.argv[2] if len(sys.argv) > 2 else "", "hb n", len(r), "rate %.2f Hz" % ((len(t) - 1) / ((t[-1] - t[0]) / 1e9)), "gap p50/p95/max", p(.5), p(.95), round(g[-1], 2),
      "| >60ms", sum(1 for x in g if x > 60), ">75ms", sum(1 for x in g if x > 75), "| top3", [round(x, 1) for x in g[-3:]], "| flags", fl)
