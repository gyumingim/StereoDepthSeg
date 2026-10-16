// scan3d 웹 뷰어 — three.js r128, 월드 z-up (OpenVINS). 복셀 = InstancedMesh 정육면체, 증분 폴링.
'use strict';
const $ = id => document.getElementById(id);
const PALETTE = ['042AFF','0BDBEB','F3F3F3','00DFB7','111F68','FF6FDD','FF444F','CCED00','00F344','BD00FF',
  '00B4FF','DD00BA','00FFFF','26C000','01FFB3','7D24FF','7B0068','FF1B6C','FC6D2F','A2FF0B'].map(h => [0, 2, 4].map(i => parseInt(h.substr(i, 2), 16)));
const clsRGB = c => c >= 0 ? PALETTE[c % PALETTE.length] : [120, 120, 120];
const instRGB = i => { if (i < 0) return [90, 90, 90]; const h = (i * 0.61803398875) % 1; const c = new THREE.Color().setHSL(h, 0.75, 0.55); return [c.r * 255, c.g * 255, c.b * 255]; };

// ---------- three.js 기본 ----------
const canvas = $('view');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(2, devicePixelRatio));
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f1318);
const cam = new THREE.PerspectiveCamera(60, 1, 0.01, 200);
cam.up.set(0, 0, 1);
scene.add(new THREE.HemisphereLight(0xffffff, 0x404050, 0.85));
const sun = new THREE.DirectionalLight(0xffffff, 0.45); sun.position.set(3, -2, 5); scene.add(sun);
const grid = new THREE.GridHelper(10, 20, 0x334455, 0x222b36); grid.rotation.x = Math.PI / 2; scene.add(grid);
scene.add(new THREE.AxesHelper(0.3));

// ---------- 궤도 조작 (z-up) ----------
const orbit = { target: new THREE.Vector3(0, 0, 0), r: 3, yaw: -2.2, pitch: 0.6 };
function applyOrbit() {
  const p = orbit.pitch, y = orbit.yaw;
  cam.position.set(orbit.target.x + orbit.r * Math.cos(p) * Math.cos(y), orbit.target.y + orbit.r * Math.cos(p) * Math.sin(y), orbit.target.z + orbit.r * Math.sin(p));
  cam.lookAt(orbit.target);
}
let drag = null;
canvas.addEventListener('contextmenu', e => e.preventDefault());
canvas.addEventListener('pointerdown', e => { drag = { x: e.clientX, y: e.clientY, pan: e.button !== 0 || e.shiftKey }; canvas.setPointerCapture(e.pointerId); });
canvas.addEventListener('pointerup', () => drag = null);
canvas.addEventListener('pointermove', e => {
  if (!drag) return;
  const dx = e.clientX - drag.x, dy = e.clientY - drag.y; drag.x = e.clientX; drag.y = e.clientY;
  if (drag.pan) {
    const s = orbit.r * 0.0015, right = new THREE.Vector3(), up = new THREE.Vector3();
    cam.matrix.extractBasis(right, up, new THREE.Vector3());
    orbit.target.addScaledVector(right, -dx * s).addScaledVector(up, dy * s);
  } else {
    orbit.yaw -= dx * 0.006; orbit.pitch = Math.max(-1.5, Math.min(1.5, orbit.pitch + dy * 0.006));
  }
});
canvas.addEventListener('wheel', e => { e.preventDefault(); orbit.r = Math.max(0.1, Math.min(80, orbit.r * Math.exp(e.deltaY * 0.001))); }, { passive: false });

// ---------- 복셀 (InstancedMesh) ----------
const V = { epoch: -1, version: 0, vm: 0.03, n: 0, cap: 0, mesh: null, index: new Map(), rgb: null, cls: null, inst: null, keys: null, mode: 'rgb' };
const boxGeo = new THREE.BoxGeometry(1, 1, 1);
const boxMat = new THREE.MeshLambertMaterial({ color: 0xffffff });
function allocVoxels(cap) {
  const old = V.mesh;
  const mesh = new THREE.InstancedMesh(boxGeo, boxMat, cap);
  mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
  mesh.setColorAt(0, new THREE.Color(1, 1, 1));
  mesh.frustumCulled = false;
  const rgb = new Uint8Array(cap * 3), cls = new Int16Array(cap), inst = new Int32Array(cap), keys = new Int32Array(cap * 3);
  if (old) {
    for (let i = 0; i < V.n; i++) { const m = new THREE.Matrix4(); old.getMatrixAt(i, m); mesh.setMatrixAt(i, m); }
    rgb.set(V.rgb.subarray(0, V.n * 3)); cls.set(V.cls.subarray(0, V.n)); inst.set(V.inst.subarray(0, V.n)); keys.set(V.keys.subarray(0, V.n * 3));
    scene.remove(old); old.dispose();
  }
  Object.assign(V, { mesh, cap, rgb, cls, inst, keys });
  mesh.count = V.n;
  scene.add(mesh);
  for (let i = 0; i < V.n; i++) colorVoxel(i);
}
function clearVoxels() { V.n = 0; V.index.clear(); if (V.mesh) V.mesh.count = 0; }
const tmpM = new THREE.Matrix4(), tmpC = new THREE.Color();
function colorVoxel(i) {
  const c = V.mode === 'class' ? (V.cls[i] >= 0 ? clsRGB(V.cls[i]) : [V.rgb[i * 3] * 0.35, V.rgb[i * 3 + 1] * 0.35, V.rgb[i * 3 + 2] * 0.35])
    : V.mode === 'inst' ? (V.inst[i] >= 0 && V.cls[i] >= 0 ? instRGB(V.inst[i]) : [V.rgb[i * 3] * 0.35, V.rgb[i * 3 + 1] * 0.35, V.rgb[i * 3 + 2] * 0.35])
    : [V.rgb[i * 3], V.rgb[i * 3 + 1], V.rgb[i * 3 + 2]];
  tmpC.setRGB(c[0] / 255, c[1] / 255, c[2] / 255);
  V.mesh.setColorAt(i, tmpC);
}
function recolorAll() { for (let i = 0; i < V.n; i++) colorVoxel(i); if (V.mesh && V.mesh.instanceColor) V.mesh.instanceColor.needsUpdate = true; }

const REC = 17;
async function pollVoxels() {
  try {
    const r = await fetch(`/api/voxels?since=${V.version}&epoch=${V.epoch}`, { cache: 'no-store' });
    const buf = await r.arrayBuffer();
    const dv = new DataView(buf);
    const epoch = dv.getUint32(0, true), version = dv.getUint32(4, true), M = dv.getUint32(8, true), vm = dv.getFloat32(12, true);
    if (epoch !== V.epoch || Math.abs(vm - V.vm) > 1e-7) { clearVoxels(); V.epoch = epoch; V.vm = vm; }
    if (!V.mesh || V.n + M > V.cap) allocVoxels(Math.max(65536, 1 << Math.ceil(Math.log2(V.n + M + 1))));
    const s = vm * 0.92;
    for (let k = 0, o = 16; k < M; k++, o += REC) {
      const ix = dv.getInt16(o, true), iy = dv.getInt16(o + 2, true), iz = dv.getInt16(o + 4, true);
      const key = ((ix + 32768) * 65536 + (iy + 32768)) * 65536 + (iz + 32768);
      let i = V.index.get(key);
      if (i === undefined) {
        i = V.n++; V.index.set(key, i);
        tmpM.makeScale(s, s, s); tmpM.setPosition((ix + 0.5) * vm, (iy + 0.5) * vm, (iz + 0.5) * vm);
        V.mesh.setMatrixAt(i, tmpM);
        V.keys[i * 3] = ix; V.keys[i * 3 + 1] = iy; V.keys[i * 3 + 2] = iz;
      }
      V.rgb[i * 3] = dv.getUint8(o + 6); V.rgb[i * 3 + 1] = dv.getUint8(o + 7); V.rgb[i * 3 + 2] = dv.getUint8(o + 8);
      V.cls[i] = dv.getInt16(o + 9, true); V.inst[i] = dv.getInt32(o + 11, true);
      colorVoxel(i);
    }
    V.version = version;
    V.mesh.count = V.n;
    if (M) { V.mesh.instanceMatrix.needsUpdate = true; if (V.mesh.instanceColor) V.mesh.instanceColor.needsUpdate = true; }
  } catch (e) { /* 서버 재시작 중 */ }
  setTimeout(pollVoxels, 250);
}

// ---------- 물체 박스·라벨 ----------
const objGroup = new THREE.Group(); scene.add(objGroup);
let objects = [], classes = {};
const edgeGeo = new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1));
function rebuildObjects() {
  while (objGroup.children.length) { const c = objGroup.children.pop(); c.material.dispose(); }
  for (const o of objects) {
    const col = new THREE.Color(o.color[0] / 255, o.color[1] / 255, o.color[2] / 255);
    const seg = new THREE.LineSegments(edgeGeo, new THREE.LineBasicMaterial({ color: col }));
    seg.scale.set(...o.box_size); seg.position.set(...o.box_center); seg.rotation.set(0, 0, o.yaw);
    objGroup.add(seg);
  }
  objGroup.visible = $('cBoxes').checked;
  const list = $('objs'); list.innerHTML = '';
  $('nObj').textContent = `(${objects.length})`;
  for (const o of objects) {
    const d = document.createElement('div');
    const sz = o.box_size.map(v => (v * 100).toFixed(0)).join('×');
    d.innerHTML = `<span><span class="dot" style="background:rgb(${o.color})"></span>${o.label} #${o.id}</span><span class="sz">${sz} cm · ${o.n_voxels}</span>`;
    d.title = `부피 ${(o.volume_m3 * 1000).toFixed(2)} L, 신뢰 ${o.conf.toFixed(2)}`;
    d.onclick = () => { orbit.target.set(...o.box_center); orbit.r = Math.max(0.5, 2.5 * Math.max(...o.box_size)); };
    list.appendChild(d);
  }
}
const labelsEl = $('labels');
function drawLabels() {
  const W = canvas.clientWidth, H = canvas.clientHeight;
  let html = '';
  if ($('cBoxes').checked) {
    const v = new THREE.Vector3();
    for (const o of objects) {
      v.set(o.box_center[0], o.box_center[1], o.box_center[2] + o.box_size[2] / 2).project(cam);
      if (v.z > 1 || v.x < -1 || v.x > 1 || v.y < -1 || v.y > 1) continue;
      html += `<span style="left:${(v.x + 1) / 2 * W}px;top:${(1 - v.y) / 2 * H}px;color:rgb(${o.color})">${o.label} #${o.id}</span>`;
    }
  }
  labelsEl.innerHTML = html;
}

// ---------- 궤적·현재 자세 ----------
const trajLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xffcf57 }));
trajLine.frustumCulled = false; scene.add(trajLine);
const frustum = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x44ddff }));
frustum.frustumCulled = false; scene.add(frustum);
let curT = null;
async function pollTraj() {
  try {
    const j = await (await fetch('/api/trajectory', { cache: 'no-store' })).json();
    const pos = new Float32Array(j.points.length * 3);
    j.points.forEach((p, i) => { pos[i * 3] = p[1]; pos[i * 3 + 1] = p[2]; pos[i * 3 + 2] = p[3]; });
    trajLine.geometry.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    trajLine.geometry.computeBoundingSphere();
    trajLine.visible = $('cTraj').checked;
    curT = j.T_world_rect0;
    if (curT) {
      // rect0 카메라 (x 오른쪽, y 아래, z 앞) 의 0.12 m 프러스텀 → 월드
      const c = j.cam, d = 0.12, T = curT;
      const P = (x, y, z) => [T[0] * x + T[1] * y + T[2] * z + T[3], T[4] * x + T[5] * y + T[6] * z + T[7], T[8] * x + T[9] * y + T[10] * z + T[11]];
      const o = P(0, 0, 0);
      const cs = [[0, 0], [c.w, 0], [c.w, c.h], [0, c.h]].map(([u, v]) => P((u - c.cx) / c.f * d, (v - c.cy) / c.f * d, d));
      const segs = [];
      for (let i = 0; i < 4; i++) segs.push(...o, ...cs[i], ...cs[i], ...cs[(i + 1) % 4]);
      const up = P(0, -d * 0.8, d); segs.push(...cs[0], ...up, ...up, ...cs[1]);   // 영상 위쪽 표시
      frustum.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(segs), 3));
      frustum.visible = true;
      if ($('cFollow').checked) orbit.target.set(o[0], o[1], o[2]);
    } else frustum.visible = false;
  } catch (e) { }
  setTimeout(pollTraj, 500);
}

// ---------- 상태 ----------
let paused = false, lastObjSig = '';
const f = (v, n = 1) => v === null || v === undefined ? '-' : (typeof v === 'number' ? v.toFixed(n) : v);
async function pollState() {
  try {
    const s = await (await fetch('/api/state', { cache: 'no-store' })).json();
    paused = s.paused; $('bPause').textContent = paused ? '재개' : '일시정지'; $('bPause').classList.toggle('on', paused);
    classes = s.classes || {};
    const vio = s.vio || {}, ph = s.phone || {}, dp = s.depth || {}, mp = s.map || {};
    renderSession(s.session || {});
    const vioTxt = vio.error ? `<span class="bad">${esc(vio.error)}</span>` : vio.status === 'idle' ? '<span class="warn">세션 없음</span>' : vio.status === 'lost' ? '<span class="bad">추적 실패 — 브리지 --auto-reset 이면 자동 재시작, 아니면 "VIO 재시작"</span>' : (vio.init ? '<span class="ok">추적 중</span>' : '<span class="warn">초기화 대기 (폰 2초 정지)</span>');
    const rates = Object.entries(ph.imu_rates || {}).map(([k, v]) => `${k.replace('_uncalibrated', '_u')} ${v}`).join(', ');
    const rows = [
      ['OpenVINS', `${vioTxt} · ${f(vio.ms)} ms · SLAM ${f(vio.n_slam, 0)} / MSCKF ${f(vio.n_msckf, 0)}${vio.auto_resets ? ` · 자동 재시작 ${vio.auto_resets}` : ''}`],
      ['VIO 입력', `스테레오 ${vio.stereo_frames || 0} / mono ${vio.mono_frames || 0} · 버림 ${vio.cams_dropped || 0} · IMU ${vio.imu_sent || 0}`],
      ['폰', ph.connected ? `${(ph.fps || []).map(v => v.toFixed(1)).join(' / ')} fps · 쌍 ${ph.pairs} · skew ${f(ph.skew_ms, 2)} ms` : '<span class="warn">접속 대기</span>'],
      ['IMU Hz', rates || '-'],
      ['IMU 동기', `${ph.imu_sync ? ph.imu_sync.out : 0} 출력 / ${ph.imu_sync ? ph.imu_sync.dropped : 0} 버림 · 시계차 ${f(ph.clock_gap_ms)} ms`],
      ['FFS', `${f(dp.ffs_ms)} ms · 반복 ${f(dp.iters, 0)}`], ['YOLO', `${f(dp.yolo_ms)} ms · 검출 ${f(dp.n_dets, 0)}`],
      ['처리', `${f(dp.fps, 2)} 키프레임/s · 총 ${f(dp.keyframes, 0)} · ${f(dp.total_ms, 0)} ms · 점 ${f(dp.points, 0)}`],
      ['복셀', `${mp.shown || 0} 표시 / ${mp.voxels || 0} · ${((mp.voxel_m || 0) * 100).toFixed(1)} cm`],
    ];
    if (ph.error) rows.push(['폰 오류', `<span class="bad">${esc(ph.error)}</span>`]);
    if (dp.error) rows.push(['깊이 오류', `<span class="bad">${esc(dp.error)}</span>`]);
    if (!dp.ready && !dp.error) rows.push(['깊이', '<span class="warn">모델 로드 중</span>']);
    if (s.epi && s.epi.status === 'ok') rows.push(['정류 검증', `dy ${f(s.epi.dy_med, 2)} px · inlier ${f(s.epi.inlier_frac, 2)}`]);
    $('stat').innerHTML = rows.map(r => `<tr><td>${r[0]}</td><td>${r[1]}</td></tr>`).join('');
    $('log').textContent = (s.log || []).join('\n');
    if (document.activeElement !== $('iHits')) $('iHits').value = mp.min_hits;
    const sig = JSON.stringify((s.objects || []).map(o => [o.id, o.n_voxels, o.box_center, o.box_size, o.yaw]));
    if (sig !== lastObjSig) { lastObjSig = sig; objects = s.objects || []; rebuildObjects(); }
  } catch (e) { $('stat').innerHTML = '<tr><td>서버</td><td class="bad">연결 끊김</td></tr>'; }
  setTimeout(pollState, 500);
}

// ---------- 모드·녹화·재생 ----------
const esc = v => String(v === null || v === undefined ? '' : v).replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const MODE_KO = { idle: '대기', live: '실시간', record: '녹화 (폰)', both: '녹화 + 실시간', replay: '녹화 재생 (복원)' };
let lastPullState = '', sessMode = 'idle', adbOk = false;
function renderSession(se) {
  sessMode = se.mode || 'idle';
  adbOk = !!(se.adb && se.adb.available);
  const rows = [['모드', `<b>${MODE_KO[sessMode] || esc(sessMode)}</b>`],
    ['연결', `${esc(se.link)} · ${esc(se.phone_spec)} · adb ${adbOk ? esc(se.adb.kind) : '<span class="warn">없음</span>'}`]];
  if (se.app) rows.push(['앱', `${esc(se.app.state || '')} ${se.app.ok === false ? '<span class="bad">시작 확인 실패</span>' : ''} <small>${esc(se.app.raw || '')}</small>`]);
  if (se.rec && (sessMode === 'record' || sessMode === 'both')) {
    const r = se.rec, t = r.t !== undefined ? r.t : (r.time !== undefined ? r.time : (r.elapsed !== undefined ? r.elapsed : r.duration));
    const fr = r.frames !== undefined ? r.frames : (r.frame !== undefined ? r.frame : r.n);
    const tTxt = t !== undefined ? esc(t) + ' s' : (r.elapsed_s !== undefined ? '앱 시작 후 ' + r.elapsed_s + ' s' : '-');
    rows.push(['녹화', `${esc(r.state || '-')} · ${tTxt} · 프레임 ${fr !== undefined ? esc(fr) : '-'}`]);
    rows.push(['status', `<small>${esc(r.raw || '')} (${esc(r.polled || '')})</small>`]);
  }
  if (sessMode === 'record') rows.push(['PC', '<span class="warn">계산 안 함 (상태만)</span>']);
  if (se.last_result) rows.push(['복원 결과', `<small>${esc(se.last_result)}</small>`]);
  $('sess').innerHTML = rows.map(r => `<tr><td>${r[0]}</td><td>${r[1]}</td></tr>`).join('');
  $('cStartApp').disabled = !adbOk;
  $('bRecList').disabled = !adbOk;
  const rp = se.replay;
  $('progReplay').hidden = !rp;
  if (rp) {
    const pct = rp.total_s > 0 ? Math.min(100, 100 * rp.pos_s / rp.total_s) : 0;
    $('barReplay').style.width = pct.toFixed(1) + '%';
    $('txtReplay').innerHTML = `재생 ${esc(rp.name)}: <b>${esc(rp.state)}</b> · ${f(rp.pos_s)} / ${f(rp.total_s)} s (${pct.toFixed(0)}%) · 쌍 ${rp.pairs}/${rp.frames_total} · 키프레임 ${rp.keyframes}`
      + `${rp.queue ? ' · 대기 ' + rp.queue : ''} · ${rp.speed ? rp.speed.toFixed(2) + '× 실시간' : '-'} · ${f(rp.elapsed_s, 0)} s`
      + `${rp.pose_timeouts ? ' · 포즈 시간초과 ' + rp.pose_timeouts : ''}${rp.every_frame ? ' · 모든 프레임' : ''} · 반복 ${rp.iters}`
      + `${rp.error ? `<br><span class="bad">${esc(rp.error)}</span>` : ''}${rp.out && (rp.state === '완료' || rp.state === '정지됨') ? `<br>결과: ${esc(rp.out)}` : ''}`;
  }
  const pl = se.pull;
  $('progPull').hidden = !pl;
  if (pl) {
    const pct = pl.total > 0 ? Math.min(100, 100 * pl.done / pl.total) : 0;
    $('barPull').style.width = pct.toFixed(1) + '%';
    $('txtPull').innerHTML = `가져오기 ${esc(pl.name)}: <b>${esc(pl.state)}</b> · ${(pl.done / 1e6).toFixed(1)} / ${(pl.total / 1e6).toFixed(1)} MB${pl.error ? ` <span class="bad">${esc(pl.error)}</span>` : ''}`;
    if (pl.state !== lastPullState) { lastPullState = pl.state; if (pl.state !== '가져오는 중') loadRecs(false); }
  }
}
function renderRecs(j) {
  let h = '<h3>폰</h3>';
  if (j.phone_error) h += `<div class="warn">${esc(j.phone_error)}</div>`;
  else if (!j.phone || !j.phone.length) h += '<div class="warn">녹화 없음 (또는 아직 새로고침 안 함)</div>';
  for (const r of j.phone || []) {
    h += `<div class="rec"><span title="${esc(r.base)}">${esc(r.name)} <small>${(r.size_kb / 1024).toFixed(1)} MB${r.local ? ' · 가져옴' : ''}</small></span>`
      + `<span><button data-pull="${esc(r.name)}" data-replay="1">가져와서 복원</button> <button data-pull="${esc(r.name)}" data-replay="0">가져오기</button></span></div>`;
  }
  h += `<h3>PC (${esc(j.rec_root)})</h3>`;
  if (!j.local || !j.local.length) h += '<div class="warn">없음</div>';
  for (const r of j.local || []) {
    h += `<div class="rec"><span title="${esc(r.path)}">${esc(r.name)} <small>${r.size_mb} MB${r.recon.length ? ' · 복원 ' + r.recon.length + '회' : ''}${r.has_video ? '' : ' · 영상 없음'}</small></span>`
      + `<span><button data-replay-local="${esc(r.name)}">복원</button></span></div>`;
  }
  $('recs').innerHTML = h;
  const opts = () => ({ every_frame: $('cEvery').checked, iters: parseInt($('iIters').value) || 8 });
  $('recs').querySelectorAll('[data-pull]').forEach(b => b.onclick = async () => {
    const rep = b.dataset.replay === '1';
    if (rep && sessMode !== 'idle' && !confirm('가져온 뒤 지금 세션을 멈추고 복원합니다. 계속할까요?')) return;
    const r = await cmd(Object.assign({ cmd: 'pull', name: b.dataset.pull, replay: rep }, opts()));
    if (!r.ok) alert(r.error || '실패');
  });
  $('recs').querySelectorAll('[data-replay-local]').forEach(b => b.onclick = async () => {
    if (sessMode !== 'idle' && !confirm('지금 세션을 멈추고 복원할까요? (맵이 새로 시작됩니다)')) return;
    const r = await cmd(Object.assign({ cmd: 'replay', name: b.dataset.replayLocal }, opts()));
    if (!r.ok) alert(r.error || '실패');
  });
}
async function loadRecs(refresh) {
  try { renderRecs(await (await fetch(`/api/recordings${refresh ? '?refresh=1' : ''}`, { cache: 'no-store' })).json()); }
  catch (e) { $('recs').innerHTML = '<div class="bad">목록 실패</div>'; }
}
$('bRecList').onclick = () => { $('recs').innerHTML = '<div class="warn">읽는 중…</div>'; loadRecs(true); };
$('bStart').onclick = async () => {
  const m = $('sMode').value;
  if (sessMode !== 'idle' && !confirm(`지금 세션(${MODE_KO[sessMode] || sessMode})을 멈추고 ${MODE_KO[m]} 을(를) 시작할까요?`)) return;
  const r = await cmd({ cmd: 'mode', mode: m, start_app: $('cStartApp').checked && adbOk });
  if (r.ok === false || r.error) alert(r.error || '실패');
};
$('bStop').onclick = async () => { await cmd({ cmd: 'stop', stop_app: $('cStartApp').checked && adbOk }); loadRecs(false); };

// ---------- 2D 패널 ----------
let kind2d = 'overlay';
function poll2d() {
  if ($('panel2d').style.display !== 'none') {
    const img = new Image();
    img.onload = () => { $('img2d').src = img.src; };
    img.src = `/api/frame.jpg?kind=${kind2d}&_=${Date.now()}`;
  }
  setTimeout(poll2d, 800);
}
document.querySelectorAll('.pk').forEach(b => b.onclick = () => { kind2d = b.dataset.k; document.querySelectorAll('.pk').forEach(x => x.classList.toggle('on', x === b)); });
$('bHide2d').onclick = () => { $('img2d').style.display = $('img2d').style.display === 'none' ? '' : 'none'; };

// ---------- 버튼 ----------
const cmd = async body => (await fetch('/api/cmd', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) })).json();
$('bPause').onclick = () => cmd({ cmd: paused ? 'resume' : 'pause' });
$('bResetMap').onclick = () => { if (confirm('맵을 지울까요?')) cmd({ cmd: 'reset_map' }); };
$('bResetVio').onclick = () => { if (confirm('VIO 와 맵을 다시 시작할까요? 폰을 2초 정지해 두세요.')) cmd({ cmd: 'reset_vio' }); };
$('bSave').onclick = async () => { $('saveMsg').textContent = '저장 중…'; const r = await cmd({ cmd: 'save' }); $('saveMsg').textContent = r.ok ? `저장: ${r.dir}` : `실패: ${r.error}`; };
document.querySelectorAll('.cm').forEach(b => b.onclick = () => { V.mode = b.dataset.m; document.querySelectorAll('.cm').forEach(x => x.classList.toggle('on', x === b)); recolorAll(); });
$('sVoxel').onchange = e => cmd({ cmd: 'set', voxel_m: parseFloat(e.target.value) });
$('iHits').onchange = e => cmd({ cmd: 'set', min_hits: parseInt(e.target.value) });
$('cBoxes').onchange = e => { objGroup.visible = e.target.checked; };
$('cTraj').onchange = e => { trajLine.visible = e.target.checked; };

// ---------- 렌더 루프 ----------
function resize() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== Math.floor(w * renderer.getPixelRatio()) || canvas.height !== Math.floor(h * renderer.getPixelRatio())) {
    renderer.setSize(w, h, false); cam.aspect = w / Math.max(1, h); cam.updateProjectionMatrix();
  }
}
function loop() { resize(); applyOrbit(); renderer.render(scene, cam); drawLabels(); requestAnimationFrame(loop); }
allocVoxels(65536);
loop(); pollVoxels(); pollState(); pollTraj(); poll2d(); loadRecs(false);
