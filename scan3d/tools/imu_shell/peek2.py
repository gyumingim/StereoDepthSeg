import sys, numpy as np
sys.path.insert(0, 'tools/imu_shell')
from analyze_static import read_csv
d = read_csv('runs/static_imu_preview/imu.csv')
t0 = min(d[k][0,0] for k in d)
for k in (4, 1):
    t = (d[k][:,0]-t0)*1e-9; dt = np.diff(t)
    print(k, 'dt pct', np.percentile(dt,[1,50,99,99.9]).round(5), 'n gaps>20ms', (dt>0.02).sum())
    big = np.flatnonzero(dt>0.05)
    print('  big gaps (t,len):', [(round(t[i],1), round(dt[i],3)) for i in big[:30]])
g = d[16]; print('uncal bias est first/last', g[0,4:7]*57.3e3, g[-1,4:7]*57.3e3)
print('cal mean mdeg/s', d[4][:,1:4].mean(0)*57.3e3, 'uncal mean', g[:,1:4].mean(0)*57.3e3)
