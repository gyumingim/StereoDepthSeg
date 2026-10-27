import subprocess, os, re, shutil, hashlib
H = '/home/exynos-lab'
for name, src in [('x960fpmc', 'xf960_3ch_noadd_op13_cat1out.onnx'), ('x960f1mc', 'xf960_1ch_noadd_op13_cat1out.onnx')]:
    print(src, os.path.getsize(f'{H}/{src}'), hashlib.md5(open(f'{H}/{src}','rb').read()).hexdigest()[:8])
    d = f'{H}/{name}'; shutil.rmtree(d, ignore_errors=True); os.makedirs(d)
    shutil.copy(f'{H}/{src}', f'{d}/{name}.onnx')
    subprocess.run(f'cd {d} && eais init --w {d}', shell=True, capture_output=True)
    y = open(f'{d}/{name}.yaml').read().replace('device: Gen-8', 'device: Gen-6').replace('mode: default', 'mode: multicore', 1)
    open(f'{d}/{name}.yaml', 'w').write(y)
    L = open(f'{d}/Config/legacy_config.yaml').read(); L = re.sub(r'(\nquantize_type:\s*)\S+', r'\g<1>fp16', L, count=1)
    open(f'{d}/Config/legacy_config.yaml', 'w').write(L)
    print(name, [l.strip() for l in y.splitlines() if 'mode:' in l or 'device:' in l], re.findall(r'\n(quantize_type:.*|use_randomdb:.*)', L))
open(f'{H}/run20.sh', 'w').write('for n in x960fpmc x960f1mc; do cd /home/exynos-lab/$n && eais conversion > ../$n.conv.log 2>&1; eais compile > ../$n.comp.log 2>&1; done; echo DONE > /home/exynos-lab/run20.done\n')
subprocess.Popen(f'nohup bash {H}/run20.sh > {H}/run20.log 2>&1 &', shell=True); print('started')
