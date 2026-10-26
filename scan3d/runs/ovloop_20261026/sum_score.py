import json,sys,statistics as st
S=json.load(open('/home/system/scanner/tmp_ovloop/ate/score.json'))['rows']
for cond in sorted({v['cond'] for v in S.values()}):
    R=[v for v in S.values() if v['cond']==cond]
    for which in ('loop','std','hs'):
        X=[v for v in R if v['set']==which]
        if not X: continue
        if which=='loop':
            print(cond,which,[(v['name'][:7],v.get('vio_loop_cm'),v.get('map_loop_cm'),v['n_loop']) for v in X]); continue
        ok=[v for v in X if v.get('vio_ate_cm') is not None and v.get('map_ate_cm') is not None]
        d=[v['map_ate_cm']-v['vio_ate_cm'] for v in ok]
        better=sum(x< -0.005 for x in d); worse=sum(x>0.005 for x in d)
        ed=sum(v.get('edges_checked',0) for v in X); fa=sum(v.get('edges_false',0) for v in X); nl=sum(v['n_loop'] for v in X)
        mx=max((v.get('edge_err_cm_max') or 0 for v in X),default=0)
        print(f"{cond} {which}: cells {len(X)} scored {len(ok)} ATE vio med {st.median(v['vio_ate_cm'] for v in ok):.3f} map med {st.median(v['map_ate_cm'] for v in ok):.3f} | diff med {st.median(d):+.3f} mean {st.mean(d):+.3f} better {better} worse {worse} | loops {nl} GT-checked {ed} false {fa} max edge err {mx} cm | cells with loops {sum(1 for v in X if v['n_loop'])}")
        bad=[(v['name'],e) for v in X for e in v.get('edges',[]) if e.get('false')]
        for b in bad[:5]: print('   FALSE',b)
