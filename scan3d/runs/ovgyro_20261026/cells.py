import json
for f,cs in [('/tmp/ovgyro/ate/score_PRK_PG3_PG1.json',['PRK','PG1','PG3']),('/tmp/ovgyro/ate/score_B0K_BG3.json',['B0K','BG3'])]:
    J=json.load(open(f))
    for r in J['cells']:
        if any(r.get(c,{}).get('div') for c in cs):
            print(r['key'], ' | '.join(f"{c} div={r[c]['div']} ate_w={r[c]['ate_w'] and round(r[c]['ate_w'],2)} sim3={r[c]['sim3']} kept={r[c]['klt']['kept_pct']} lostmax={r[c]['klt']['lost_max_pct']}" for c in cs))
