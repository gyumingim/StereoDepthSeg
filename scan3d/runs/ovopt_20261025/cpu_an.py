# CPU time per variant from a prof_pc.sh log (run 3 on: "== <cfg>_<var>_r<k> cpu_user_sys_wall_s U S W ...").
# Total CPU (user + sys) of the whole bridge process over the 80 s stream = work done on all cores (the parallel
# options trade a little extra CPU for wall time; on the phone CPU time ~ energy at a given clock, a proxy only).
#   python3 cpu_an.py prof_pc_3.log
import re
import sys
from collections import defaultdict

import numpy as np

d = defaultdict(list)
for line in open(sys.argv[1]):
    m = re.match(r"== (.+)_([A-Za-z0-9]+)_r(\d+) cpu_user_sys_wall_s ([\d.]+) ([\d.]+) ([\d.]+)", line)
    if m:
        d[(m.group(1), m.group(2))].append((float(m.group(4)), float(m.group(5)), float(m.group(6))))
print("| config | variant | runs | CPU user+sys s (median) | wall s (median) | CPU / wall |")
print("|---|---|---|---|---|---|")
for (cfg, v), rows in sorted(d.items()):
    a = np.array(rows)
    cpu = np.median(a[:, 0] + a[:, 1])
    wall = np.median(a[:, 2])
    print(f"| {cfg} | {v} | {len(rows)} | {cpu:.2f} | {wall:.2f} | {cpu / wall:.2f} |")
