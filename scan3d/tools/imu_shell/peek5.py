import json
r = json.load(open('runs/gyro_bias_stability.json'))
print('cams', r['camera_intervals_s'], 'static_end', r['static_end_s'], 'cuts', r['cuts'], r['auto_cut_reasons_t'])
for nm in ('thermal', 'thermal_cal'):
    th = r[nm]
    print('==', nm, 'best', th.get('best_source'), 'noise', th.get('noise_cam_on_vs_off_mdeg_s'))
    for src, v in th['per_source'].items():
        g = [v[f'gyro_raw_mdeg_s_{x}'] for x in 'xyz']; a = [v[f'accel_raw_mg_{x}'] for x in 'xyz']
        print(f"{src:7s} {v['min']:.1f}-{v['max']:.1f} rng {v['range_C']:.1f} | g slope", [round(i.get('slope_per_C', 0), 2) for i in g], 'r', [round(i.get('r', 0), 2) for i in g],
              '| a slope', [round(i.get('slope_per_C', 0), 3) for i in a], 'r', [round(i.get('r', 0), 2) for i in a])
        if src == th.get('best_source') or src in ('battery', 'AP'):
            for lab, items in (('g', g), ('a', a)):
                print('    ', lab, 'heat', [round((i.get('heating') or {}).get('slope_per_C', float('nan')), 2) for i in items],
                      'cool', [round((i.get('cooling') or {}).get('slope_per_C', float('nan')), 2) for i in items],
                      'hyst', [round(i.get('hysteresis_cooling_minus_heating', float('nan')), 2) for i in items],
                      'resid', [round(i.get('resid_std', float('nan')), 2) for i in items])
an = r['android_cal']['gyro']
cams = r['camera_intervals_s']
print('android gyro n', an['n_changes'], 'range', an['range'], 'first', an['first'], 'last', an['last'])
import collections
c = collections.Counter(tuple(s['jump']) for s in an['steps'])
print('jump kinds', c.most_common(8))
for s in an['steps']:
    if s['t'] > r['static_end_s'] - 60:
        print('  late step', s['t'], s['jump'], s['after'], s['during_camera'])
        break
big = [s for s in an['steps'] if s['jump_norm'] > 75]
print('big steps', [(s['t'], s['jump']) for s in big][:20])
print('accel android', r['android_cal'].get('accel', {}).get('n_changes'))
for w in ('2s', '5s'):
    e = r['start_estimate_error'][w]
    print(w, {L: (e['literal_mdeg_s'].get(L), e['all_starts'].get(L, {}).get('rms_per_axis_mdeg_s'), e['all_starts'].get(L, {}).get('p95_norm_mdeg_s')) for L in ('60', '180', '300')})
for k in 'xyz':
    g = r['allan']['gyro_raw'][k]; gc = r['allan']['gyro_cal'][k]; a = r['allan']['accel_raw'][k]
    print(k, 'ARW', round(g['ARW_deg_per_sqrt_h'], 3), round(g['N'] * 1e3, 2), 'BI', round(g['BI_deg_s'] * 1e3, 2), g['tau_B'], 'K', g['K'], '| cal ARW', round(gc['ARW_deg_per_sqrt_h'], 3), 'BI', round(gc['BI_deg_s'] * 1e3, 2), gc['tau_B'], '| VRW', round(a['VRW_m_s_per_sqrt_h'], 4), round(a['VRW_ug_per_sqrtHz']), 'BI mg', round(a['BI_mg'], 3), a['tau_B'])
print(r['mean_gyro_raw_mdeg_s'], r['mean_gyro_raw_minus_earth_mdeg_s'], r['mean_gyro_cal_mdeg_s'], r['coverage']['4'])
print('floor', r['mean_noise_floor_mdeg_s'])
