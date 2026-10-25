import json
d = json.load(open('/tmp/ovdrone/ate/score_B0_R294_R294F.json'))
rows = d['rows'] if 'rows' in d else d.get('cells', d)
for r in rows:
    if not isinstance(r, dict) or 'key' not in r: continue
    if any(r.get(c, {}).get('div') for c in ('B0', 'R294', 'R294F')):
        print(r['key'], ' | '.join(f"{c}: div={r[c]['div']} ate_w={r[c]['ate_w']} why={r[c]['why']} trk={r[c]['tracked']}" for c in ('B0','R294','R294F') if c in r))
print([k for k in d.keys()])
