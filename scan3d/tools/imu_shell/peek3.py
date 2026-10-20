import sys, glob, json, numpy as np
sys.path.insert(0, 'tools/imu_shell')
from analyze_static import read_csv, to_grid, fill_nan, R2D
print('thr', json.load(open('runs/gyro_bias_stability.json'))['hs_still_threshold_gyro_std_dps'])
for rd in sorted(glob.glob('recordings/hs_2026101*_*')):
    d = read_csv(rd + '/imu.csv')
    t0 = d[4][0, 0]; T = (d[4][-1, 0] - t0) * 1e-9
    G = fill_nan(to_grid((d[4][:, 0] - t0) * 1e-9, d[4][:, 1:4] * R2D, 0.01, 0, T))
    A = fill_nan(to_grid((d[1][:, 0] - t0) * 1e-9, d[1][:, 1:4], 0.01, 0, T))
    nw = len(G) // 50
    gs = G[:nw * 50].reshape(nw, 50, 3).std(1).max(1)
    gm = np.linalg.norm(G[:nw * 50].reshape(nw, 50, 3).mean(1), axis=1)
    as_ = A[:nw * 50].reshape(nw, 50, 3).std(1).max(1)
    print(rd[-18:], 'gstd pct', np.percentile(gs, [1, 5, 20]).round(3), 'gmean pct', np.percentile(gm, [1, 5]).round(3), 'astd pct', np.percentile(as_, [1, 5, 20]).round(3))
