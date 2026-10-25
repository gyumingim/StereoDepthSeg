import json, gzip, glob, os, sys
import numpy as np
os.chdir('/home/system/scanner/StereoDepthSeg/scan3d')
for r in ['hs_20261024_165506_std60', 'hs_20261024_173432_std60', 'hs_20261021_213341']:
    a = json.load(open(f'recordings/{r}/imu_gt/align.json'))
    print(r, a['dt_ns'], a['method'], a.get('corr_mse'), a.get('omega_rms'), np.round(a['R_ci'], 5).tolist())
L = [json.loads(x) for x in gzip.open('/tmp/ovopt/ate/B0/213341_o0_B0.poses.jsonl.gz', 'rt') if x.strip()]
print(len(L), L[0].keys() if L else None); print(L[len(L)//2] if L else None)
rows = []
for f in sorted(glob.glob('/tmp/ovopt/ate/B0/*_B0.json')):
    d = json.load(open(f))
    rows.append((os.path.basename(f)[:9], d['align_dt_ms'], d['dt_cam_imu_init_ms'], d['dt_cam_imu_final_ms'], d['dt_cam_imu_med_ms'], d['timeshift_cam_imu'], d.get('ate_cm')))
for r in rows: print(r)
