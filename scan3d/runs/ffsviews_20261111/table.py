"""ffsviews 11-11: table of the queue runs (133820). Writes table.json next to this file."""
import json, sys, time, hashlib
from pathlib import Path
H = Path(__file__).resolve().parent
sys.path.insert(0, str(H.parent / 'kfonly_20261111'))
from mesh_layers import layers
L = H.parent / 'laptop' / '20261022_133820'
base = json.loads((L / 'fv_n3' / 'scan' / 'scan_summary.json').read_text())['step_s']
up = base['kfba']; pre = base['vio'] + base['reduce']
rows = []
for t in ['n3', 'ref', 'n3b', 'n3c', 'n2', 'n4', 'n5', 'n3med', 'a2', 'a3', 'a4', 'a5', 'g2', 'g3', 'g4', 'n3d', 'd2', 'd3', 'd4', 'd5', 'n3e', 'c2', 'c3', 'c4', 'c5']:
    d = L / f'fv_{t}' / ('mesh' if t == 'ref' else 'scan/mesh')
    if not (d / 'summary.json').exists():
        rows.append(dict(tag=t, missing=True)); continue
    m = json.loads((d / 'summary.json').read_text())
    kf = json.loads((d / 'keyframes.json').read_text())
    if t == 'ref':
        mesh_s = m['runtime_s']
    else:
        mesh_s = json.loads((d.parent / 'scan_summary.json').read_text())['step_s']['mesh']
    ffs = m.get('ffs_s_total', round(sum(sum(r.get('ffs_ms', [])) for r in kf) / 1000, 1))
    s = m['poses']['sim3_scale_would_be']
    tot = pre + up + mesh_s; pc = up + mesh_s
    ly = layers(str(d))
    rows.append(dict(tag=t, ffs_views=m.get('ffs_views', 3), partners=m.get('ffs_partners', 'near'), fuse=m.get('ffs_fuse', 'mean'),
                     n_kf=m['n_kf'], kf_with_pair=m['n_kf_depth'], n_pairs=sum(r.get('pairs', 0) for r in kf),
                     ffs_s=ffs, mesh_s=mesh_s, total_s=round(tot, 1), pc_s=round(pc, 1),
                     absdZ_p90_pooled=m['pooled_absdZ_cm']['p90'], absdZ_p90_kfmed=m.get('kf_board_absdZ_p90_cm_median'),
                     n_kf_board_eval=m.get('n_kf_board_eval'), k_pct=(1 / s - 1) * 100, double_layer=ly['ghost_frac'],
                     board_cover=m.get('mesh_board_cover_frac'), mesh_v=m['mesh_vertices'], rejected=m.get('n_pairs_rejected'), dy_rej=m.get('pair_check_rejected'), dy_n=m.get('pair_check_n'), dy_s=m.get('pair_check_s'),
                     ply_md5=hashlib.md5((d / 'mesh.ply').read_bytes()).hexdigest()[:10]))
import numpy as np
ev = {}
for r in rows:
    if 'missing' in r: continue
    d = L / f"fv_{r['tag']}" / ('mesh' if r['tag'] == 'ref' else 'scan/mesh')
    ev[r['tag']] = {x['i']: x['board_absdZ_p90_cm'] for x in json.loads((d / 'keyframes.json').read_text()) if 'board_absdZ_p90_cm' in x}
common = set.intersection(*[set(v) for v in ev.values()])
for r in rows:
    if 'missing' not in r:
        r['n_common'] = len(common); r['absdZ_p90_kfmed_common'] = float(np.median([ev[r['tag']][i] for i in common]))
c = next(r for r in rows if r['tag'] == 'n3')
for r in rows:
    if 'missing' not in r:
        r['total_pct'] = round((r['total_s'] / c['total_s'] - 1) * 100, 1); r['pc_pct'] = round((r['pc_s'] / c['pc_s'] - 1) * 100, 1)
out = dict(made=time.strftime('%F %T'), base_steps=base, rows=rows)
(H / 'table.json').write_text(json.dumps(out, indent=1))
for r in rows:
    print(r)
