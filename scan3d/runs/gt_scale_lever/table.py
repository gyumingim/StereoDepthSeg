"""Table of s / s_lever / s_fixedlever per recording (spl GT, rs when no spl) + old-s identity check vs before/."""
import json, glob, os, time
import numpy as np
OD = 'runs/overnight_gt'; B = 'runs/gt_scale_lever/before'
def load(p):
    try: return json.load(open(p))
    except Exception: return None
rows, diff = [], 0
recs = {}
for gd in ('rs', 'spl'):
    for x in load(f'{OD}/scale_ref_monitor_imu_gt_{gd}.json'): recs.setdefault(x['rec'], {})[gd] = x
    for x in load(f'{B}/scale_ref_monitor_imu_gt_{gd}.json'): recs[x['rec']].setdefault('old_' + gd, x)
for p in sorted(glob.glob(f'{OD}/hs_*_scale_*.json')):
    gd = p.rsplit('_', 1)[1][:-5]; x = load(p)[0]
    recs.setdefault(x['rec'], {})[gd] = x
    o = load(f'{B}/' + os.path.basename(p))
    if o: recs[x['rec']]['old_' + gd] = o[0]
for k, v in recs.items():
    for gd in ('rs', 'spl'):
        if gd in v and 'old_' + gd in v and v[gd].get('s') != v['old_' + gd].get('s'):
            diff += 1; print('OLD S CHANGED', k, gd, v[gd].get('s'), v['old_' + gd].get('s'))
def board(rec):
    b = load(f'recordings/{rec}/imu_gt/board.json')
    return (b['board_name'].replace('board_', '').replace('.json', ''), b['square_mm']) if b else ('monitor_6x6 (10-19, no board.json)', 63.403)
out = []
print('| 녹화 | 보드 (칸 mm) | GT | 창 | s (옛, 지렛대 없음) | s_lever [95 %] | 맞춘 r cm | s_fixedlever [95 %] | 판정 |')
print('|---|---|---|---|---|---|---|---|---|')
for k in sorted(recs):
    v = recs[k]; gd = 'spl' if 'spl' in v and 's' in v['spl'] else 'rs'
    x = v.get(gd, {})
    bn, sq = board(k)
    if 's' not in x:
        print(f'| {k[3:]} | {bn} ({sq}) | {gd} | {x.get("windows")} | — | — | — | — | 창 부족 |'); continue
    out.append(dict(rec=k, board=bn, square_mm=sq, gt=gd, **{q: x[q] for q in ('windows', 's', 's_ci95', 's_lever', 's_lever_ci95', 'lever_fit_cm', 's_fixedlever', 's_fixedlever_ci95', 's_fixedlever_boot_std', 'check')}))
    print(f'| {k[3:]} | {bn} ({sq}) | {gd} | {x["windows"]} | {x["s"]:.3f} | {x["s_lever"]:.3f} [{x["s_lever_ci95"][0]:.3f}, {x["s_lever_ci95"][1]:.3f}] | '
          + ', '.join(f'{c:.2f}' for c in x['lever_fit_cm']) + f' | {x["s_fixedlever"]:.3f} [{x["s_fixedlever_ci95"][0]:.3f}, {x["s_fixedlever_ci95"][1]:.3f}] | {x["check"]} |')
print()
print('| 보드 | n | s 평균 ± sd | s_lever 평균 ± sd | s_fixedlever 평균 ± sd |'); print('|---|---|---|---|---|')
groups = {}
for r in out: groups.setdefault((r['board'], r['square_mm'], r['rec'][3:11]), []).append(r)
for g, rs in sorted(groups.items()):
    f = lambda q: f'{np.mean([r[q] for r in rs]):.3f} ± {np.std([r[q] for r in rs], ddof=1) if len(rs) > 1 else 0:.3f}'
    print(f'| {g[0]} {g[1]} mm ({g[2]}) | {len(rs)} | {f("s")} | {f("s_lever")} | {f("s_fixedlever")} |')
a = lambda q, sel: np.std([r[q] for r in out if sel(r)], ddof=1)
s17 = lambda r: r['rec'][3:11] in ('20261019', '20261021', '20261022')
print('sd over ate_floor 17:', *(f'{q} {a(q, s17):.4f}' for q in ('s', 's_lever', 's_fixedlever')), ' n', sum(map(s17, out)))
print('sd over all:', *(f'{q} {a(q, lambda r: True):.4f}' for q in ('s', 's_lever', 's_fixedlever')), ' n', len(out))
print('old s changed:', diff)
json.dump(dict(made=time.strftime('%F %T'), inputs='runs/overnight_gt/*_scale_*.json (scale_ls.py, lever config config/imu_cam_lever.json)', rows=out), open('runs/gt_scale_lever/table.json', 'w'), indent=1)
