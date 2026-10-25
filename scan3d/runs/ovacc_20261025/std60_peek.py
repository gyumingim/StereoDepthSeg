import json, os, glob
import numpy as np
os.chdir('/home/system/scanner/StereoDepthSeg/scan3d/recordings')
vj = json.load(open('/home/system/scanner/StereoDepthSeg-ovacc/scan3d/config/phone_ov_main/variants.json'))
Rl = np.array(vj['R_ci'])
def ang(A, B):
    return float(np.degrees(np.arccos(np.clip((np.trace(A.T @ B) - 1) / 2, -1, 1))))
for r in ['hs_20261024_165506_std60', 'hs_20261024_173432_std60', 'hs_20261024_141623_std60', 'hs_20261024_151646_std60']:
    print('==', r, sorted(os.listdir(r))[:30])
    if os.path.isdir(r + '/imu_gt'):
        print('  imu_gt:', os.listdir(r + '/imu_gt'))
    m = json.load(open(r + '/meta.json'))
    print('  meta:', {k: m[k] for k in m if any(s in k for s in ('fps', 'width', 'expo', 'focus', 'iso', 'lens', 'dur', 'ois', 'stab'))})
    al = r + '/imu_gt/align.json'
    if os.path.exists(al):
        a = json.load(open(al))
        if 'R_ci' in a:
            print('  dt_ms', a['dt_ns'] * 1e-6, 'angle vs live R_ci deg', round(ang(np.array(a['R_ci']), Rl), 3), 'corr_mse', a.get('corr_mse'), 'omega_rms', a.get('omega_rms'), 'n', a.get('n'))
# HS recs angle spread
for r in sorted(glob.glob('hs_2026102[0-2]_*/imu_gt/align.json'))[:20]:
    a = json.load(open(r))
    print(r.split('/')[0], 'dt', round(a['dt_ns'] * 1e-6, 3), 'ang', round(ang(np.array(a['R_ci']), Rl), 3), 'mse', round(a.get('corr_mse', 0), 3))
