import sys, torch, collections
sys.path.insert(0, '/home/system/scanner/dpvo_ext/DPVO')
sd = torch.load('/home/system/scanner/dpvo_ext/DPVO/dpvo.pth', map_location='cpu')
g = collections.OrderedDict()
for k, v in sd.items():
    k = k.replace('module.', '')
    top = '.'.join(k.split('.')[:2]) if k.startswith('update') else k.split('.')[0] + '.' + k.split('.')[1]
    g[top] = g.get(top, 0) + v.numel()
tot = sum(v.numel() for v in sd.values())
print('total params', tot)
for k, v in g.items(): print(f'{k:30s} {v:9d}')
