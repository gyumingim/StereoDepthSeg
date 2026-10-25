import gzip, json, glob, os
n = same = 0
for f in sorted(glob.glob('/tmp/ovdrone/ate/B0/*.poses.jsonl.gz')):
    g = f.replace('/tmp/ovdrone/', '/tmp/ovopt/')
    if not os.path.exists(f.replace('.poses.jsonl.gz', '.json')) or not os.path.exists(g):
        continue
    strip = lambda p: [json.dumps({k: v for k, v in json.loads(l).items() if k not in ('ms', 'remap_ms', 'wait_ms')}, sort_keys=True) for l in gzip.open(p, 'rt') if l.strip()]
    a, b = strip(f), strip(g)
    n += 1; same += a == b
    if a != b: print('DIFF', os.path.basename(f), len(a), len(b))
print(f'B0 ovdrone vs ovopt: {same}/{n} cells identical (pose lines without timing fields)')
