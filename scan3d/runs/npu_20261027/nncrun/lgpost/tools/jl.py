"""Jupyter helper over CDP page: python jl.py ls PATH | put LOCAL REMOTE | get REMOTE LOCAL | sh 'cmd' [timeout_s]"""
import sys, json, base64, time
from playwright.sync_api import sync_playwright
p = sync_playwright().start(); b = p.chromium.connect_over_cdp("http://127.0.0.1:9333")
pg = [x for x in b.contexts[0].pages if 'ai-studio-farm' in x.url and '/lab' in x.url][0]
BASE = "/user/0933431f-a161-4283-b633-085309f887b5"
JS = """async ([base, method, path, body]) => {
  const x = document.cookie.split('; ').find(c => c.startsWith('_xsrf='));
  const h = {'Content-Type': 'application/json'}; if (x) h['X-XSRFToken'] = decodeURIComponent(x.slice(6));
  const r = await fetch(base + path, {method, headers: h, body: body ? body : undefined, credentials: 'same-origin'});
  return [r.status, await r.text()]; }"""
def api(method, path, body=None):
    return pg.evaluate(JS, [BASE, method, path, json.dumps(body) if body is not None else None])
c = sys.argv[1]
if c == 'ls':
    s, t = api('GET', '/api/contents/' + sys.argv[2].strip('/') + '?content=1'); print(s) if s != 200 else None; j = json.loads(t) if s == 200 else {}
    print(s, [(e['name'], e.get('size')) for e in j.get('content', [])] if isinstance(j.get('content'), list) else t[:300])
elif c == 'put':
    d = base64.b64encode(open(sys.argv[2], 'rb').read()).decode()
    s, t = api('PUT', '/api/contents/' + sys.argv[3], dict(type='file', format='base64', content=d)); print(s, t[:200])
elif c == 'get':
    s, t = api('GET', '/api/contents/' + sys.argv[2] + '?content=1&format=base64&type=file'); print(s) if s != 200 else None; j = json.loads(t) if s == 200 else {}
    open(sys.argv[3], 'wb').write(base64.b64decode(j['content'])); print(s, j['size'])
elif c == 'sh':
    # run via a temp script file + terminal-less: use kernel-free approach: write sh, run with POST to a terminal websocket
    cmd = sys.argv[2]; to = float(sys.argv[3]) if len(sys.argv) > 3 else 120
    tag = 'J%d' % int(time.time() * 1000)
    out = pg.evaluate(open(__file__.replace('jl.py','term.js')).read(), [BASE, cmd, tag, to])
    t = out[1].replace('\r', '')
    k = t.find(tag + '_""END')
    t = t[k + len(tag) + 6:] if k >= 0 else t
    t = t.split(tag + '_END')[0]
    print(out[0]); print(t[-8000:])
