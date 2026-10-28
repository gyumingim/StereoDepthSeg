async ([base, cmd, tag, to]) => {
  const x = document.cookie.split('; ').find(c => c.startsWith('_xsrf='));
  const h = {'Content-Type': 'application/json'}; if (x) h['X-XSRFToken'] = decodeURIComponent(x.slice(6));
  let r = await fetch(base + '/api/terminals', {method: 'POST', headers: h, credentials: 'same-origin'});
  const name = (await r.json()).name;
  const ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + base + '/terminals/websocket/' + name);
  const mark = tag + '_' + 'END';
  let buf = '';
  const done = await new Promise(res => {
    const t = setTimeout(() => res('TIMEOUT'), to * 1000);
    ws.onmessage = e => { const m = JSON.parse(e.data); if (m[0] === 'stdout') { buf += m[1]; if (buf.split(mark).length > 1) { clearTimeout(t); res('OK'); } } };
    ws.onopen = () => ws.send(JSON.stringify(['stdin', '(' + cmd + ') 2>&1; echo ' + tag + '_""END\r']));
  });
  ws.close();
  await fetch(base + '/api/terminals/' + name, {method: 'DELETE', headers: h, credentials: 'same-origin'});
  return [done, buf];
}
