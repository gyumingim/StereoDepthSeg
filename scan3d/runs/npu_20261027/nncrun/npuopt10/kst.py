import re, os, glob
NAMES = ['x960fpmc', 'x960f1mc']
H = '/home/exynos-lab'
print([f for f in os.listdir(H) if f.endswith('.done')])
for n in NAMES:
    s = []
    for k in ('conv', 'comp'):
        p = f'{H}/{n}.{k}.log'
        if not os.path.exists(p): s.append(k + ':-'); continue
        t = re.sub(r'\x1b\[[0-9;]*m', '', open(p, errors='replace').read())
        e = re.findall(r'(?:Error:|Error msg:|RuntimeError:|ValueError:|IndexError:|AssertionError:|Report ERROR\]|is not supported!?)([^\n]{0,110})', t)
        ok = 'Conversion is done' in t if k == 'conv' else bool(re.search(r'NNC size = (\d+)', t))
        sz = re.findall(r'NNC size = (\d+)', t)
        s.append(f"{k}:{'OK' if ok else 'ERR'}{(' ' + sz[-1]) if sz else ''}{(' | ' + e[0].strip()) if (e and not ok) else ''} ({len(t)}B)")
    print(n, ' ; '.join(s), [os.path.basename(x) for x in glob.glob(f'{H}/{n}/Compiler_result/*.nnc')])
