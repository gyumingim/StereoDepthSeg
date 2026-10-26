import gzip,json,sys,glob,os
B=os.path.expanduser('~/scanner/tmp_integ2l/ate')
SKIP={'map','ms','t_out','remap_ms','age_ms','lat_ms'}
for o in (0,1,2):
    a=[json.loads(x) for x in gzip.open(f'{B}/R60/s165506_o{o}_R60.poses.jsonl.gz','rt')]
    b=[json.loads(x) for x in gzip.open(f'{B}/N60/s165506_o{o}_N60.poses.jsonl.gz','rt')]
    keys=set()
    for x in a[:5]: keys|=set(x)
    diff=0
    for x,y in zip(a,b):
        for k in ('p','q','v','T_world_cam0','t_ns','status','n_slam'):
            if x.get(k)!=y.get(k): diff+=1; break
    print(o,len(a),len(b),'lines differing in p/q/v/T/t/status/n_slam:',diff,'map in R60:',sum('map' in x for x in a),'keys:',sorted(keys)[:30])
a=[json.loads(x) for x in gzip.open(f'{B}/R60/s165506_o0_R60.poses.jsonl.gz','rt')]
print('lines with p:',sum('p' in x for x in a),'with T:',sum(x.get('T_world_cam0') is not None for x in a), [x for x in a if 'map' in x][0].get('map'))
