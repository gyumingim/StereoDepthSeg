import json
r = json.load(open('runs/gyro_bias_stability.json'))
an = r['android_cal']
for k in ('gyro', 'accel'):
    a = an.get(k, {})
    print(k, 'n', a.get('n_changes'), 'first', a.get('first'), 'last', a.get('last'), 'range', a.get('range'))
    for s in a.get('steps', [])[:10]:
        print('  ', s['t'], s['jump'])
print('check', an.get('check_type4_minus_type16_vs_minus_estimate_max_mdeg_s'))
for L in ('60', '180', '300'):
    for w in ('2s', '5s'):
        e = r['start_estimate_error'][w]
        print(w, L, 'lit', e['literal_mdeg_s'].get(L), 'all', e['all_starts'].get(L))
print('floor', r['mean_noise_floor_mdeg_s'])
th = r.get('thermal') or {}
print('best', th.get('best_source'))
for src, v in (th.get('per_source') or {}).items():
    print(src, round(v['range_C'], 2), [round(v[f'gyro_raw_mdeg_s_{x}'].get('slope_per_C', 0), 2) for x in 'xyz'], [round(v[f'gyro_raw_mdeg_s_{x}'].get('r', 0), 2) for x in 'xyz'])
for k in 'xyz':
    g = r['allan']['gyro_raw'][k]; a = r['allan']['accel_raw'][k]
    print(k, 'ARW', round(g['ARW_deg_per_sqrt_h'], 3), round(g['N'] * 1e3, 2), 'BI', round(g['BI_deg_s'] * 1e3, 2), g['tau_B'], 'K', g['K'], '| VRW', round(a['VRW_m_s_per_sqrt_h'], 4), 'BI mg', round(a['BI_mg'], 3), a['tau_B'])
print(r['orientation'])
print(r['mean_gyro_raw_mdeg_s'], r['mean_gyro_raw_minus_earth_mdeg_s'], r['mean_gyro_cal_mdeg_s'])
print(r['coverage'])
