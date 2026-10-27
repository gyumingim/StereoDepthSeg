#!/usr/bin/env python3
"""lgopt 2026-10-27 step 1: why FLG gets 0 loops on the white-wall hand loops 142148 / 142610 (xfopt test logs, read only).
Per candidate (KF, old KF): XFeat path xn (all LG/NN matches of the pair), nm (matches with 3D), ni (PnP inliers), why, and the
FLG gates (inl 60, ratio 0.6, rp 2) / FNN gates (ratio 0.3, cells 10, rp 2, yaw 3) the pose would face. Output diag_hand.json + text."""
import json, math, sys
from pathlib import Path
T = Path('/home/system/scanner/tmp_xfopt/test')
RD = Path(__file__).resolve().parent
G = dict(FLG=dict(inl=60, ratio=0.6, cells=0, rp=2, yaw=10), FNN=dict(inl=0, ratio=0.3, cells=10, rp=2, yaw=3))
def first_fail(e, g):
    if not e.get('q'):
        return e['why']
    if e['rp'] > g['rp']: return 'rollpitch'
    if abs(e['dyaw'])*180/math.pi > g['yaw']: return 'yaw'
    if e['ni'] < g['inl']: return 'xinl'
    if e['ni'] < g['ratio']*e['nm']: return 'xratio'
    if e.get('cells',0) < g['cells']: return 'xcells'
    return 'pass'
out = {}
for rec in ('l142148_o0', 'l142610_o0'):
    pair = {}
    for c in ('FLG', 'FNN'):
        L = [json.loads(x) for x in open(T/c/f'{rec}_{c}.loop.jsonl')]
        kfs = [x for x in L if x.get('type') == 'kf']
        loops = [(x['cur'], x['old']) for x in L if x.get('type') == 'loop']
        xf = [json.loads(x) for x in open(T/c/f'{rec}_{c}.xf.jsonl')]
        seqM = [m['n'] for m in xf if m['op'] == 'M']
        for k in kfs:
            for e in k.get('cx', []):
                if e['p'] != 'x': continue
                d = pair.setdefault((k['idx'], e['o']), {})
                d[c] = dict(xn=e.get('xn'), nm=e['nm'], ni=e['ni'], why=e['why'], rp=e.get('rp'), yaw=(abs(e['dyaw'])*180/math.pi if 'dyaw' in e else None),
                            cells=e.get('cells'), ratio=(round(e['ni']/e['nm'],2) if e['nm'] else None), ntri=e.get('ntri'),
                            gate=first_fail(e, G[c]), loop=(k['idx'], e['o']) in loops, n3d=k['n3d'], xfk=k.get('xf'))
        out.setdefault(rec, {})[c] = dict(n_kf=len(kfs), loops=loops, n_cand=sum(len(k['cand']) for k in kfs),
                                          seq_match_med=sorted(seqM)[len(seqM)//2] if seqM else None, n_tri_med=sorted(k['xf'][2] for k in kfs)[len(kfs)//2])
    rows = []
    print('==', rec, {c: {k: v for k, v in out[rec][c].items()} for c in ('FLG', 'FNN')})
    print(f"{'cur':>3} {'old':>3} | {'FLG xn nm ni why gate':<44} | FNN xn nm ni why gate")
    for (cur, old), d in sorted(pair.items()):
        s = []
        for c in ('FLG', 'FNN'):
            e = d.get(c)
            if not e: s.append('-'); continue
            extra = f" rp{e['rp']:.1f} y{e['yaw']:.1f} c{e['cells']} r{e['ratio']}" if e['rp'] is not None else ''
            s.append(f"{e['xn']:4} {e['nm']:3} {e['ni']:3} {e['why']:<9} {e['gate']:<9}{'*' if e['loop'] else ''}{extra}")
        print(f'{cur:3} {old:3} | {s[0]:<44} | {s[1]}')
        rows.append(dict(cur=cur, old=old, **d))
    out[rec]['pairs'] = rows
json.dump(out, open(RD/'diag_hand.json', 'w'), indent=1, default=str)
