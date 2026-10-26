cd ~/scanner/StereoDepthSeg-integ2/scan3d/runs/integ2_20261026/loop60
python3 - <<'PY'
p='loop60_run.py'; s=open(p,encoding='utf-8').read()
a='    "N60": dict(fps=60.0, extra="--loop 0"),'
if '"A60"' not in s: s=s.replace(a,a+'\n    "A60": dict(fps=60.0, extra="--loop-sync 0"),   # live-like async loop thread'); open(p,'w',encoding='utf-8').write(s)
PY
export OVLOOP_OUT=$HOME/scanner/tmp_integ2l/ate_id LOOP60_PP="--pose-precision 17"
P=~/scanner/StereoDepthSeg/venv_ffs/bin/python
$P loop60_run.py run "std:A60" --par 3 >> $OVLOOP_OUT/sched.log 2>&1; tail -1 $OVLOOP_OUT/sched.log
cat > cmp_id.py <<'PY'
"""integ2 0.6.28 identity on std60 (vio_bench feed, --pose-precision 17): O60 = 0.6.27 build --profile rec60,
N60 = 0.6.28 --profile rec60 --loop 0, R60 = 0.6.28 --profile rec60 (loop on, --loop-sync 1), A60 = same, --loop-sync 0 (live).
Compares every pose line, all keys except timing keys and "map"."""
import gzip, json, os, sys
B = os.environ.get("OVLOOP_OUT", os.path.expanduser("~/scanner/tmp_integ2l/ate_id"))
TIMING = {"ms", "remap_ms", "t_out", "age_ms", "lat_ms", "feed_ms", "map"}
def load(c, cell):
    f = f"{B}/{c}/{cell}_{c}.poses.jsonl.gz"
    return [json.loads(x) for x in gzip.open(f, "rt")] if os.path.exists(f) else None
for rec in ("s165506", "s173432"):
    for o in (0, 1, 2):
        cell = f"{rec}_o{o}"
        ref = load("O60", cell)
        for c in ("N60", "R60", "A60"):
            x = load(c, cell)
            if ref is None or x is None: print(cell, c, "missing"); continue
            nd = sum(1 for a, b in zip(ref, x) if {k: v for k, v in a.items() if k not in TIMING} != {k: v for k, v in b.items() if k not in TIMING})
            print(f"{cell} O60 vs {c}: lines {len(ref)}/{len(x)} differing {nd} map lines {sum('map' in b for b in x)} keys_extra {sorted(set().union(*[set(b) for b in x]) - set().union(*[set(a) for a in ref]))}")
PY
python3 cmp_id.py | tee cmp_id.txt
