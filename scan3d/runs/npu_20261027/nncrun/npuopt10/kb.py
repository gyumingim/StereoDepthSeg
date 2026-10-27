import subprocess, shutil, os, glob, re
from collections import Counter
H = '/home/exynos-lab'
for n in ['x960fpmc', 'x960f1mc']:
    d = f'{H}/{n}'; z = f'{H}/b_{n}.zip'
    nnc = glob.glob(d + '/Compiler_result/*.nnc')
    o = f'{H}/bo/{n}'; shutil.rmtree(o, ignore_errors=True); os.makedirs(o)
    shutil.copy(nnc[0], o); [shutil.copy(f, o) for f in glob.glob(f'{H}/{n}.*.log')]
    for f in glob.glob(d + '/Converter_result/**/testvector/inout/*.bin', recursive=True): shutil.copy(f, o)
    t = re.sub(r'\x1b\[[0-9;]*m', '', open(f'{H}/{n}.comp.log', errors='replace').read())
    c = Counter(re.findall(r'\[TARGET_HW: (\w+)\][^(\n]*\(([A-Za-z-]+)', t))
    open(o + '/hw_ops.txt', 'w').write('\n'.join(f'{v} {k[0]} {k[1]}' for k, v in c.most_common()))
    shutil.make_archive(z[:-4], 'zip', o)
    print(n, os.path.getsize(z), sorted(os.listdir(o)), {f'{k[0]}:{k[1]}': v for k, v in c.items() if k[0] != 'NPU'})
