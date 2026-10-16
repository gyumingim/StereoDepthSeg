// 3D 스캐너 웹 화면 — three.js r128, 월드 z-up (OpenVINS). 복셀 = InstancedMesh 정육면체, 증분 폴링.
// 위쪽 "단계 표시줄" 은 /api/state 의 vio·phone·depth·session·ui(USB·온도·중계) 로 지금 할 일을 한 줄로 계산한다 (stepFrom).
// 오른쪽 아래 카메라 창 2개 (/api/cam.jpg?lens=0|1) — 스캔·복원 중이고 화면이 보일 때만 약 3 fps 로 갱신.
// 깊이 렌즈 '3개 모두 녹화 (비교용)' ([폰에 녹화만] 에서만) → 녹화 목록에서 쌍별 [2+5 보기] [2+6 보기], 리플레이 위쪽에서 쌍 바꾸기 + 비교 숫자.
// 녹화본 목록(오른쪽 칸, /api/recordings items)에서 [▶ 보기] → 리플레이 (최종 맵 · 시간 따라 보기, /api/replay/*). 카메라 창은 그 시각의 녹화 프레임.
// state.preview 가 참이면 (scan3d/preview_server.py) 위쪽에 "미리보기 — 가짜 데이터" 띠를 띄운다.
'use strict';
const $ = id => document.getElementById(id);
const PALETTE = ['042AFF','0BDBEB','F3F3F3','00DFB7','111F68','FF6FDD','FF444F','CCED00','00F344','BD00FF',
  '00B4FF','DD00BA','00FFFF','26C000','01FFB3','7D24FF','7B0068','FF1B6C','FC6D2F','A2FF0B'].map(h => [0, 2, 4].map(i => parseInt(h.substr(i, 2), 16)));
const clsRGB = c => c >= 0 ? PALETTE[c % PALETTE.length] : [120, 120, 120];
const instRGB = i => { if (i < 0) return [90, 90, 90]; const h = (i * 0.61803398875) % 1; const c = new THREE.Color().setHSL(h, 0.75, 0.55); return [c.r * 255, c.g * 255, c.b * 255]; };
const esc = v => String(v === null || v === undefined ? '' : v).replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const f = (v, n = 1) => v === null || v === undefined ? '-' : (typeof v === 'number' ? v.toFixed(n) : v);
const cmd = async body => (await fetch('/api/cmd', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) })).json();

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
  const dim = [V.rgb[i * 3] * 0.35, V.rgb[i * 3 + 1] * 0.35, V.rgb[i * 3 + 2] * 0.35];
  const c = V.mode === 'class' ? (V.cls[i] >= 0 ? clsRGB(V.cls[i]) : dim)
    : V.mode === 'inst' ? (V.inst[i] >= 0 && V.cls[i] >= 0 ? instRGB(V.inst[i]) : dim)
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

// ---------- 물체 상자·라벨 ----------
const objGroup = new THREE.Group(); scene.add(objGroup);
let objects = [];
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
  $('nObj').textContent = objects.length ? `(${objects.length})` : '';
  if (!objects.length) { list.innerHTML = '<div class="empty">아직 없음</div>'; return; }
  for (const o of objects) {
    const d = document.createElement('div');
    const sz = o.box_size.map(v => (v * 100).toFixed(0)).join('×');
    d.innerHTML = `<span><span class="dot" style="background:rgb(${o.color})"></span>${esc(o.label)} #${o.id}</span><span class="sz">${sz} cm</span>`;
    d.title = `부피 ${(o.volume_m3 * 1000).toFixed(2)} L · 복셀 ${o.n_voxels} · 신뢰 ${o.conf.toFixed(2)} — 누르면 그 물체로 이동`;
    d.onclick = () => { orbit.target.set(...o.box_center); orbit.r = Math.max(0.5, 2.5 * Math.max(...o.box_size)); };
    list.appendChild(d);
  }
}
const labelsEl = $('labels');
function drawLabels() {
  const W = canvas.clientWidth, H = canvas.clientHeight;
  let html = '';
  if ($('cBoxes').checked && !RV.on) {
    const v = new THREE.Vector3();
    for (const o of objects) {
      v.set(o.box_center[0], o.box_center[1], o.box_center[2] + o.box_size[2] / 2).project(cam);
      if (v.z > 1 || v.x < -1 || v.x > 1 || v.y < -1 || v.y > 1) continue;
      html += `<span style="left:${(v.x + 1) / 2 * W}px;top:${(1 - v.y) / 2 * H}px;color:rgb(${o.color})">${esc(o.label)} #${o.id}</span>`;
    }
  }
  labelsEl.innerHTML = html;
}

// ---------- 궤적·현재 자세 ----------
const trajLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xffcf57 }));
trajLine.frustumCulled = false; scene.add(trajLine);
const frustum = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x44ddff }));
frustum.frustumCulled = false; scene.add(frustum);
async function pollTraj() {
  if (RV.on) { setTimeout(pollTraj, 500); return; }      // 리플레이 중에는 리플레이 궤적·절두체만
  try {
    const j = await (await fetch('/api/trajectory', { cache: 'no-store' })).json();
    const pos = new Float32Array(j.points.length * 3);
    j.points.forEach((p, i) => { pos[i * 3] = p[1]; pos[i * 3 + 1] = p[2]; pos[i * 3 + 2] = p[3]; });
    trajLine.geometry.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    trajLine.geometry.computeBoundingSphere();
    trajLine.visible = $('cTraj').checked;
    const T = j.T_world_rect0;
    if (T) {
      // rect0 카메라 (x 오른쪽, y 아래, z 앞) 의 0.12 m 프러스텀 → 월드
      const c = j.cam, d = 0.12;
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

// ---------- 단계 표시줄: 지금 할 일 계산 ----------
// 반환: { step: 1..4 (지금 단계, 0 = 없음), done: 끝난 단계 수, level: ok|warn|bad|'', main, sub }
const MODE_KO = { idle: '대기', live: '실시간', record: '폰에 녹화', both: '녹화 + 실시간', replay: '녹화 복원' };
let sessT0 = 0, lastMode = 'idle', camSeenT = 0;
// 최근 10초 짝 없어 버린 비율 % (REVIEW5 N1). 쌍 묶기 기준이 반 프레임 주기(0.45·T)라 위상과 상관없이 거의 다 짝이 됨 →
// 버림은 한쪽 렌즈 프레임이 빠졌다는 뜻 (USB·발열). 그래서 예전(50/80)보다 낮게 잡는다.
const PAIR_DROP_WARN = 20, PAIR_DROP_BAD = 50;
// 렌즈 위상 맞추기 (앱 1.5-sync, SPEC 15): ui.sync = metrics.json sync, se.sync = status 줄 꼬리 (녹화 모드)
const SYNC_KO = { off: '끔', measuring: '재는 중', aligning: '맞추는 중', locked: '맞음', failed: '못 맞춤 (잠깐 쉼)',
  drift: '어긋남 (녹화 중이라 다시 안 맞춤)' };
const METHOD_KO = { C: 'C (카메라 다시 열기)', A: 'A (한 렌즈 찍는 주기 조절)', B: 'B (촬영 다시 시작)', none: '처음부터 맞음' };
function syncOf(s) { const ui = s.ui || {}, se = s.session || {}; return ui.sync || se.sync || null; }
function syncTry(sy) { const n = sy.method === 'C' && sy.c_try !== undefined && sy.c_try !== null ? sy.c_try : sy.tries; return n || 0; }
// 맞추는 중 안내 (1.6-sync 방법 C: 카메라를 닫았다 다시 열 때마다 찍는 순간이 새로 정해짐, SPEC 15.3)
function syncMain(sy) {
  const ph = sy.phase_ms !== null && sy.phase_ms !== undefined ? `지금 위상 ${f(Math.abs(sy.phase_ms), 1)} ms` : '위상 재는 중';
  if (sy.state === 'failed') return `두 렌즈 시각을 아직 못 맞췄습니다 — ${sy.retry_in_s != null ? sy.retry_in_s + '초 뒤' : '잠시 뒤'} 다시 시도합니다 (${ph})`;
  if (sy.state === 'measuring') return `두 렌즈 시각 재는 중… (${ph})`;
  if (sy.method === 'C') return `두 렌즈 시각 맞추는 중 — 카메라 다시 여는 중 (${syncTry(sy)}번째) · ${ph}`;
  if (sy.method === 'B') return `두 렌즈 시각 맞추는 중 — 촬영 다시 시작 (${syncTry(sy)}번째) · ${ph}`;
  return `두 렌즈 시각 맞추는 중… (${ph})`;
}
function syncSub(sy, extra) {
  const parts = ['폰을 그대로 두세요.'];
  if (sy.method === 'C' || sy.state === 'failed') {
    parts.push('카메라를 다시 열 때마다 두 렌즈가 찍는 순간이 새로 정해지고, 2.5 ms 안에 들어오면 멈춥니다.');
    const e = sy.c_expected_tries, t = sy.c_try_ms_avg;
    if (e) parts.push(`보통 ${Math.round(e)}번쯤${t ? ` (한 번에 약 ${(t / 1000).toFixed(1)}초)` : ''} 걸립니다.`);
    if (sy.state === 'failed') parts.push(`${sy.c_max_tries || 25}번 해도 안 맞으면 30초 쉬었다가 다시 합니다.`);
  }
  parts.push(extra || '그동안 영상이 잠깐씩 끊기고 짝이 0 이어도 정상입니다.');
  return parts.join(' ');
}
function phaseTxt(sy) { return sy && sy.phase_ms !== null && sy.phase_ms !== undefined ? `${f(Math.abs(sy.phase_ms), 1)} ms` : '재는 중'; }
function loadSec(s, dp) {                        // 깊이 모델 로드 경과 초 (서버 시각 기준, REVIEW5 N2)
  if (!dp || !dp.load_t0) return '';
  const el = Math.max(0, (s.t || Date.now() / 1000) - dp.load_t0);
  return ` ${el.toFixed(0)}초`;
}
function sizeMismatch(err, s) {                  // REVIEW6 R10: "Invalid JPEG/H.264 packet: stream=0 WxH ..." → 앱 크기 ≠ 고른 크기
  const m = /Invalid (?:JPEG|H\.264) packet: stream=\d+ (\d+x\d+)/.exec(err || '');
  const st = (s.ui && s.ui.stream) || {};
  const want = st.size || (Array.isArray((s.stream || {}).size) ? s.stream.size.join('x') : '');
  return m && want && m[1] !== want ? { got: m[1], want } : null;
}
function stepFrom(s) {
  const ui = s.ui || {}, se = s.session || {}, vio = s.vio || {}, ph = s.phone || {}, dp = s.depth || {};
  const mode = se.mode || 'idle', active = mode === 'live' || mode === 'both' || mode === 'record';
  const now = Date.now() / 1000, since = sessT0 ? now - sessT0 : 0;
  const relay = ui.relay || {}, app = se.app || {};
  const fps = (ph.fps || []).filter(v => v > 0);
  const camOn = !!ph.connected && fps.length > 0;
  if (camOn) camSeenT = now;
  const R = (level, step, done, main, sub) => ({ level, step, done, main, sub: sub || '' });

  // 복원은 PC 파일만 쓰므로 USB·온도보다 먼저 보여 준다 (REVIEW4 B4)
  if (mode === 'replay') {
    const rp = se.replay || {};
    const pct = rp.total_s > 0 ? Math.min(100, 100 * rp.pos_s / rp.total_s) : 0;
    if (rp.error) return R('bad', 0, 0, '복원 중 오류가 났습니다', esc(rp.error));
    if (!dp.ready && !dp.error) return R('warn', 0, 0, `깊이 모델을 불러오는 중…${loadSec(s, dp)} (처음 한 번 30초쯤)`,
      (dp.load_stage ? esc(dp.load_stage) + ' · ' : '') + '불러오면 복원이 바로 시작됩니다.');
    return R('ok', 0, 0, `녹화를 3D 로 복원하는 중… ${pct.toFixed(0)}%`, '끝나면 결과가 녹화 폴더에 저장됩니다. 기다려 주세요.');
  }

  // 폰 USB
  const usb = ui.usb;
  if (usb === 'noadb') return R('bad', 1, 0, 'adb 를 찾지 못했습니다', 'Android platform-tools 가 필요합니다 (README 설치 4 참고). 그동안 녹화 복원만 쓸 수 있습니다.');
  if (usb === 'unauthorized') return R('warn', 1, 0, '폰 화면에서 "USB 디버깅 허용" 을 눌러 주세요', '"이 컴퓨터에서 항상 허용" 에 체크하면 다음부터 안 물어봅니다.');
  if (usb === 'none' || usb === 'offline') {
    if (active) return R('bad', 1, 0, '폰 연결이 끊겼습니다 — 케이블을 다시 꽂으세요',
      usb === 'offline' ? '폰이 응답하지 않습니다. 케이블을 뺐다 꽂고, 폰 화면의 허용 창을 확인하세요.' : '다시 꽂으면 자동으로 이어집니다. 잘 안 되면 [정지] 후 [스캔 시작].');
    return R('warn', 1, 0, '폰을 USB 로 연결하고 "USB 디버깅 허용" 을 눌러 주세요', '연결되면 이 줄이 바뀝니다.');
  }
  // 온도 (가장 급한 경고)
  if (ui.temp_level === 'hot') return R('bad', active ? 4 : 1, active ? 3 : 1, `폰이 뜨겁습니다 (${f(ui.temp_c)} °C) — 잠깐 쉬세요`, '[정지] 를 누르고 폰을 식힌 뒤 다시 시작하세요. 케이스를 벗기면 빨리 식습니다.');

  // 가져오기 / 대기
  if (!active) {
    const pl = se.pull;
    if (pl && pl.state === '가져오는 중') return R('ok', 0, 1, '폰에서 녹화를 가져오는 중…', '');
    return R('ok', 1, 1, '준비됐습니다 — [▶ 스캔 시작] 을 누르세요', se.last_result ? `지난 결과: ${esc(se.last_result)}` : '폰을 물체 쪽으로 들고 시작하면 됩니다.');
  }
  // REVIEW7 H2: 폰 실시간 압축기(H.264 인코더)가 멈춤 → "영상 없음" 대신 진짜 원인
  if ((mode === 'live' || mode === 'both') && ui.h264_error) {
    const e = String(ui.h264_error);
    return R('bad', 2, 1, `폰 영상 압축기 오류: ${e.length > 60 ? e.slice(0, 60) + '…' : e} — 고급에서 JPEG 로 바꿔 보세요`,
      '고급 설정 → 영상 형식 을 JPEG 로 고른 뒤 [■ 정지] → [▶ 스캔 시작]. 자세한 오류: ' + esc(e));
  }
  // 앱·중계 경고는 영상(또는 녹화)이 아직 안 들어올 때만 (REVIEW4 B3: 앱이 늦게 떠도 빨간 줄이 남지 않게)
  const rec = se.rec || {};
  const recOn = mode === 'record' && /RECORDING/.test(rec.state || rec.raw || '');
  if (!camOn && !recOn) {
    if (app.ok === false) return R('bad', 2, 1, '폰 카메라를 켜지 못했습니다 — 폰 화면을 켜고 다시 [▶ 스캔 시작]', `앱 상태: ${esc(app.state || app.raw || '-')}`);
    if (relay.state === '오류' && mode !== 'record') return R('bad', 2, 1, 'PC 와 폰 사이 연결을 만들지 못했습니다',
      esc(relay.error || '') + (relay.fails > 1 ? ` — 자동으로 다시 시도 중 (${relay.fails}번째)` : ' — 자동으로 다시 시도합니다. 계속되면 [정지] 후 다시 [▶ 스캔 시작]'));
  }

  const sy = syncOf(s);
  const recWaitTxt = sy ? `맞추면 바로 녹화가 시작됩니다 (최대 ${sy.rec_wait_sync_s || 40}초 기다림).` : '';
  if (mode === 'record') {
    const r = rec;
    if (!recOn) return R('warn', 2, 1, '폰 녹화를 켜는 중…', '폰 화면을 켜 두세요.');
    // 녹화기가 아직 위상 맞춤을 기다릴 때만 (녹화가 시작된 뒤에는 다시 맞추지 않음, REVIEW8 L3)
    if (sy && sy.rec_wait) return R('warn', 3, 2, syncMain(sy), syncSub(sy, recWaitTxt));
    const t = r.elapsed_s !== undefined ? ` · ${r.elapsed_s} s` : '';
    if (sy && sy.state === 'drift') return R('warn', 4, 3, `폰에 녹화 중${t} — 두 렌즈 시각이 조금 어긋났습니다 (${phaseTxt(sy)})`,
      '녹화가 끊기지 않게 녹화 중에는 다시 맞추지 않습니다. 복원 때 짝이 조금 덜 맞을 수 있습니다.');
    if (since < 3) return R('warn', 3, 2, '폰을 2초 가만히 두세요', '');
    return R('ok', 4, 3, `천천히 움직이세요 — 폰에 녹화 중${t}`, '끝나면 [■ 정지]. 나중에 고급 → 녹화본 에서 3D 로 복원합니다.');
  }
  // 실시간: 앱이 카메라를 다시 열며 위상을 맞추는 동안은 영상이 끊기고 짝이 0 이어도 정상 → 빨간 경고 대신 안내 (SPEC 15.6)
  if (sy && (sy.state === 'aligning' || (sy.state === 'failed' && sy.method === 'C') || sy.rec_wait))
    return R('warn', 3, 2, syncMain(sy), syncSub(sy, mode === 'both' && sy.rec_wait
      ? recWaitTxt + ' 실시간 영상도 그때부터 들어옵니다.' : ''));
  // 실시간 (live / both)
  // B9: 브리지 시작 자체가 실패했으면 "카메라 켜는 중" 대신 진짜 원인을 먼저
  if (!camOn && vio.error && /시작 실패|다시 띄우기 실패/.test(vio.error))
    return R('bad', 3, 1, '위치 추적 프로그램을 켜지 못했습니다 — 자동으로 다시 시도합니다', esc(vio.error));
  const sizeBad = sizeMismatch(ph.error, s);
  if (sizeBad) return R('bad', 2, 1, `폰 앱이 다른 화질(${sizeBad.got})로 보내고 있습니다 — 고른 화질은 ${sizeBad.want}`,
    '앱을 PC 가 켜지 않았거나(손으로 켬, 자동 앱 시작 꺼짐) 다른 프로그램이 켰습니다. [■ 정지] 후 다시 [▶ 스캔 시작] 하세요.');
  if (!camOn) {
    if (camSeenT && now - camSeenT < 600 && since > 5) return R('bad', 2, 1, '카메라 영상이 끊겼습니다 — 다시 연결하는 중…', '계속 안 되면 케이블을 다시 꽂거나 [↺ 처음부터 다시].');
    return R('warn', 2, 1, '카메라 켜는 중… 폰 화면을 켜 두세요', since > 25 ? '오래 걸리면 [■ 정지] 후 다시 [▶ 스캔 시작].' : '10초쯤 걸립니다.');
  }
  if (ph.error) return R('bad', 2, 1, '폰 영상 수신 오류', esc(ph.error));
  const imuOut = ph.imu_sync ? ph.imu_sync.out : 0;
  if (!imuOut && (ph.pairs || 0) > 150) return R('bad', 3, 2, '폰 움직임 센서 값이 안 들어옵니다', '폰 앱이 최신이 아닐 수 있습니다 (README 알려진 한계). 위치 추적을 할 수 없습니다.');
  // REVIEW5 N1: 짝을 많이 버리면 원인을 보여 준다. 쌍 묶기 기준은 반 프레임 주기(0.45·T, 15 fps 면 30 ms)라
  // 두 렌즈가 어긋나 찍어도(10~28 ms) 가장 가까운 프레임끼리 짝이 된다 → 버림이 많으면 한쪽 렌즈 프레임이 빠지는 것
  const pd = ph.pair_drop_pct, pairLow = pd !== null && pd !== undefined && pd >= PAIR_DROP_WARN;
  const pairMsg = pairLow ? `한쪽 렌즈 영상이 자주 빠져 짝이 안 맞음 — 기준 ${f(ph.pair_skew_ms, 1)} ms, 버린 비율 ${f(pd, 0)} %` : '';
  const pairHelp = '두 렌즈는 서로 맞춰 찍지 않지만, 가장 가까운 프레임끼리 짝을 짓고 그 사이 폰 움직임은 깊이 계산에서 보정합니다. '
    + '그런데도 짝을 많이 버리면 한쪽 렌즈 영상이 끊기는 것입니다 — 케이블을 확인하고, 폰이 뜨거우면 쉬었다가, '
    + '계속되면 [■ 정지] → [▶ 스캔 시작]. 자세히: 고급 → 렌즈 시각차.';
  if (pairLow && (!vio.init || pd >= PAIR_DROP_BAD)) return R('bad', 3, 2, pairMsg, pairHelp);
  if (vio.error) return R('warn', 3, 2, '위치 추적을 다시 켜는 중… 폰을 2초 가만히 두세요',
    `자동으로 다시 켭니다${vio.bridge_restarts ? ` (${vio.bridge_restarts}번째)` : ''}. 1분 넘게 이 줄이 그대로면 [↺ 처음부터 다시]. (${esc(vio.error)})`);
  if (vio.status === 'lost') return R('bad', 4, 3, '위치 추적을 잃었습니다 — 천천히 움직이거나 [↺ 처음부터 다시]', '빠르게 돌리거나 무늬 없는 벽만 비추면 잃기 쉽습니다.');
  // 한쪽 렌즈(mono) 위치 추적은 처음에 움직임이 있어야 시작된다 (OpenVINS static init: 잠깐 멈췄다가 움직이는 순간)
  if (!vio.init) return vio.vio_mode === 'mono'
    ? R('warn', 3, 2, '폰을 1초 멈췄다가, 처음 1~2초 천천히 움직이세요',
      '한쪽 렌즈로 위치를 잡으려면 처음에 조금 움직여야 합니다. 좌우로 10 cm쯤 천천히 옮기면 됩니다. 시작되면 이 줄이 바뀝니다.'
      + (!dp.ready && dp.loading ? ` · 깊이 모델 불러오는 중…${loadSec(s, dp)}` : ''))
    : R('warn', 3, 2, '폰을 2초 가만히 두세요', '위치 추적을 준비하는 중입니다. 책상 위에 내려놓아도 됩니다.'
      + (!dp.ready && dp.loading ? ` · 깊이 모델 불러오는 중…${loadSec(s, dp)}` : ''));
  let sub = '물체를 0.3~2.5 m 거리에서 여러 방향으로 비추세요.';
  if (dp.error) return R('bad', 4, 3, '깊이 계산에 오류가 났습니다', esc(dp.error));
  // REVIEW5 N2: 첫 세션 모델 로드 동안에는 키프레임이 쌓이지 않으므로 기다리게 하고 진행(경과 초·단계)을 보여 준다
  if (!dp.ready && dp.loading) return R('warn', 4, 3, `깊이 모델을 불러오는 중…${loadSec(s, dp)} — 준비가 끝나면 움직이세요`,
    (dp.load_stage ? esc(dp.load_stage) + ' · ' : '') + '처음 한 번만 30초쯤 걸립니다. 그동안 비춘 곳은 3D 맵에 안 들어갑니다 (위치만 추적).');
  if (pairLow) sub = `<span class="warn">${esc(pairMsg)}</span> · ` + sub;
  if (!dp.ready) sub = dp.loading ? '깊이 모델을 불러오는 중… (처음 한 번 30초쯤) — 그동안 위치만 추적합니다.'
    : '깊이 모델을 아직 안 불러왔습니다 — 곧 불러옵니다.';
  return R('ok', 4, 3, '천천히 움직이세요 — 스캔 중', sub);
}

// index.html 두 배치 모두 지원: #stepbar/#stepMain/#stepSub 또는 #guide
function renderStep(st, ui) {
  let sub = st.sub;
  if (ui && ui.temp_level === 'warm') sub += `${sub ? ' · ' : ''}<span class="warm warn">폰이 따뜻합니다 (${f(ui.temp_c)} °C)</span>`;
  if ($('stepMain')) {
    $('stepbar').className = st.level || '';
    $('stepMain').textContent = st.main;
    $('stepSub').innerHTML = sub;
  } else if ($('guide')) {
    $('guide').className = st.level || '';
    $('guide').innerHTML = esc(st.main) + (sub ? `<small>${sub}</small>` : '');
  }
  document.querySelectorAll('#steps [data-s]').forEach(e => {
    const k = +e.dataset.s, base = e.classList.contains('step') ? 'step ' : '';
    e.className = base + (k === st.step ? (st.level === 'bad' && base ? 'err' : 'now') : (k <= st.done ? 'done' : ''));
  });
}
let toastTimer = 0;
function toast(msg, ms = 4000) {
  const t = $('toast') || $('saveMsg');
  if (!t) return;
  t.textContent = msg; t.style.display = t.id === 'toast' ? 'block' : '';
  clearTimeout(toastTimer); toastTimer = setTimeout(() => { if (t.id === 'toast') t.style.display = 'none'; else t.textContent = ''; }, ms);
}

// ---------- 상태 폴링 ----------
let paused = false, lastObjSig = '', sessMode = 'idle', adbOk = false, failCount = 0;
async function pollState() {
  try {
    const s = await (await fetch('/api/state', { cache: 'no-store' })).json();
    failCount = 0;
    const se = s.session || {}, ui = s.ui || {}, vio = s.vio || {}, ph = s.phone || {}, dp = s.depth || {}, mp = s.map || {};
    sessMode = se.mode || 'idle';
    adbOk = !!(se.adb && se.adb.available);
    if (sessMode !== lastMode) { sessT0 = sessMode === 'idle' ? 0 : Date.now() / 1000; camSeenT = 0; lastMode = sessMode; }
    paused = s.paused; $('bPause').textContent = paused ? '재개' : '일시정지'; $('bPause').classList.toggle('on', paused);
    renderStep(RV.on ? rvStep() : stepFrom(s), ui);
    rvOnState();
    renderPreview(s);
    camLenses(se.cams);
    Q.state = s; renderLoad();

    // 버튼
    const active = sessMode !== 'idle';
    $('bStop').disabled = !active;
    $('bStart').textContent = active ? '▶ 다시 시작' : '▶ 스캔 시작';
    $('bRestart').disabled = !(sessMode === 'live' || sessMode === 'both');
    $('cStartApp').disabled = !adbOk;
    $('bRecList').disabled = !adbOk;

    // 칩: 카메라 fps · 복셀 · 폰 온도
    const fps = (ph.fps || []).filter(v => v > 0);
    let chips = `<span>카메라 ${ph.connected && fps.length ? Math.min(...fps).toFixed(0) + ' fps' : '-'}</span>`
      + `<span>복셀 ${(mp.shown || 0).toLocaleString()}</span>`;
    const fake = s.preview ? ' (가짜)' : '';
    if (ui.temp_c !== null && ui.temp_c !== undefined)
      chips += `<span class="${ui.temp_level === 'hot' ? 'hot' : ui.temp_level === 'warm' ? 'warm' : ''}">폰 ${f(ui.temp_c)} °C${fake}</span>`;
    if ($('cFps')) {
      $('cFps').textContent = ph.connected && fps.length ? `${Math.min(...fps).toFixed(0)} fps` : '-';
      $('cVox').textContent = (mp.shown || 0).toLocaleString();
      const tc = $('chipTemp');
      tc.hidden = ui.temp_c === null || ui.temp_c === undefined;
      if (!tc.hidden) { $('cTemp').textContent = `${f(ui.temp_c)} °C`; tc.className = 'chip ' + (ui.temp_level === 'hot' ? 'hot' : ui.temp_level === 'warm' ? 'warm' : ''); }
    } else if ($('chips')) $('chips').innerHTML = chips;

    renderProgress(se);
    if ($('adv').open) renderDetails(s, se, ui, vio, ph, dp, mp);
    if (document.activeElement !== $('iHits') && mp.min_hits) $('iHits').value = mp.min_hits;
    const sig = JSON.stringify((s.objects || []).map(o => [o.id, o.n_voxels, o.box_center, o.box_size, o.yaw]));
    if (sig !== lastObjSig) { lastObjSig = sig; objects = s.objects || []; rebuildObjects(); }
  } catch (e) {
    if (++failCount >= 3) {
      renderStep({ level: 'bad', step: 0, done: 0, main: '스캐너 서버에 연결할 수 없습니다',
        sub: 'PC 를 켠 뒤 잠깐 기다리거나, scan3d\\start_scanner.bat 을 실행하세요. (서버가 막 켜지는 중이면 곧 이어집니다)' }, null);
    }
  }
  setTimeout(pollState, 500);
}

function renderProgress(se) {
  const rp = se.replay;
  $('progReplay').hidden = !rp;
  if (rp) {
    const pct = rp.total_s > 0 ? Math.min(100, 100 * rp.pos_s / rp.total_s) : 0;
    $('barReplay').style.width = pct.toFixed(1) + '%';
    $('txtReplay').innerHTML = `복원 ${esc(rp.name)}: <b>${esc(rp.state)}</b> · ${f(rp.pos_s)} / ${f(rp.total_s)} s`
      + `${rp.out && (rp.state === '완료' || rp.state === '정지됨') ? ` · 결과: ${esc(rp.out)}` : ''}`;
  }
  const pl = se.pull;
  $('progPull').hidden = !pl || pl.state === '완료';
  if (pl) {
    const pct = pl.total > 0 ? Math.min(100, 100 * pl.done / pl.total) : 0;
    $('barPull').style.width = pct.toFixed(1) + '%';
    $('txtPull').innerHTML = `가져오기 ${esc(pl.name)}: <b>${esc(pl.state)}</b> · ${(pl.done / 1e6).toFixed(1)} / ${(pl.total / 1e6).toFixed(1)} MB${pl.error ? ` <span class="bad">${esc(pl.error)}</span>` : ''}`;
    if (pl.state !== lastPullState) { lastPullState = pl.state; if (pl.state !== '가져오는 중') loadRecs(false); }
  }
}

// 고급 → 렌즈별 H.264 상태 (REVIEW7 H5·H6): 키프레임 대기 · 깨진 프레임 · seq 건너뜀 · 풀기 ms · kbps · B 프레임 경고
function h264Rows(h) {
  if (!h || !Array.isArray(h.waiting_key)) return [];
  const at = (k, i) => (Array.isArray(h[k]) ? h[k][i] : null);
  const rows = h.waiting_key.map((wk, i) => {
    const bad = (at('corrupt', i) || 0) + (at('decode_errors', i) || 0);
    const parts = [
      wk ? '<span class="warn">키프레임 기다리는 중</span>' : '<span class="ok">정상</span>',
      `깨진 프레임 <span class="${bad ? 'warn' : ''}">${bad}</span>`,
      `빠진 번호 ${at('seq_gaps', i) || 0}${at('seq_gap_resync', i) ? ` (다시 맞춤 ${at('seq_gap_resync', i)})` : ''}`,
      `풀기 ${f(at('decode_ms', i))} ms (최대 ${f(at('decode_ms_max', i))})`,
      `${at('kbps', i) != null ? (at('kbps', i) / 1000).toFixed(1) : '-'} Mbps`,
      `키 요청 ${at('keyreq_sent', i) || 0}`];
    if (at('b_frames_seen', i)) parts.push(`<span class="bad">B 프레임 ${at('b_frames_seen', i)}장 — 폰 압축기가 B 프레임을 씀 (지연·시각 어긋남 가능, JPEG 로 바꿔 보세요)</span>`);
    return [`H.264 렌즈 ${i}`, parts.join(' · ')];
  });
  return rows;
}

// 고급 → 렌즈 위상 줄 (SPEC 15.6)
function syncRow(sy) {
  if (!sy) return '정보 없음 (폰 앱이 1.5-sync 보다 옛 버전이거나 아직 못 읽음)';
  const st = SYNC_KO[sy.state] || esc(sy.state || '-');
  const cls = sy.state === 'locked' ? 'ok' : sy.state === 'failed' ? 'bad' : sy.state === 'off' ? '' : 'warn';
  const parts = [`<span class="${cls}">${st}</span>`];
  if (sy.phase_ms !== null && sy.phase_ms !== undefined) parts.push(`위상 ${f(sy.phase_ms, 2)} ms`);
  if (sy.slip_ms_s !== null && sy.slip_ms_s !== undefined) parts.push(`미끄러짐 ${f(sy.slip_ms_s, 3)} ms/s`);
  if (sy.method) parts.push(`방법 ${esc(METHOD_KO[sy.method] || sy.method)}`);
  if (sy.reopens) parts.push(`카메라 다시 열기 ${sy.reopens}번` + (sy.c_try_ms_avg ? ` (한 번 평균 ${f(sy.c_try_ms_avg / 1000, 1)}초)` : ''));
  if (sy.c_last_align_s !== null && sy.c_last_align_s !== undefined) parts.push(`지난번 맞추는 데 ${f(sy.c_last_align_s, 1)}초`);
  if (sy.c_rounds_failed) parts.push(`<span class="warn">${sy.c_max_tries || 25}번 해도 못 맞춤 ${sy.c_rounds_failed}회</span>`);
  if (sy.adjustments) parts.push(`주기 조절 ${sy.adjustments}번`);
  if (sy.restarts) parts.push(`촬영 다시 시작 ${sy.restarts}번`);
  if (sy.hold_reason) parts.push(`<small>${esc(sy.hold_reason)}</small>`);
  if (sy.lens_ref && sy.lens_target) parts.push(`렌즈 ${esc(sy.lens_target)} − ${esc(sy.lens_ref)}`);
  if (sy.method_a_possible === false && sy.mode === 'a') parts.push(`<small>A 못 씀: ${esc(sy.method_a_reason || '')}</small>`);
  if (sy.method_a_failed) parts.push(`<small class="warn">A 안 먹힘: ${esc(sy.method_a_fail_reason || '')}</small>`);
  return parts.join(' · ');
}

// 고급 → 움직임 보정 줄: 두 렌즈 촬영 순간 사이 폰 움직임을 키프레임마다 보정해 정류 (scan3d motion_rect.py)
function motionRow(m) {
  if (!m) return '-';
  if (!m.on) return '꺼짐 (보정값 고정 정류)';
  const parts = [];
  if (m.dt_med_ms !== null && m.dt_med_ms !== undefined) parts.push(`dt(렌즈5−렌즈2) ${f(m.dt_med_ms, 1)} ms (${f(m.dt_min_ms, 1)}~${f(m.dt_max_ms, 1)})`);
  parts.push(`보정 ${m.motion || 0} · 고정 ${m.fixed || 0} · <span class="${m.skipped ? 'warn' : ''}">건너뜀 ${m.skipped || 0}</span>`);
  const l = m.last;
  if (l) {
    const bits = [`dt ${f(l.dt_ms, 1)} ms`];
    if (l.rot_deg !== undefined) bits.push(`회전 ${f(l.rot_deg, 3)}°`);
    if (l.trans_mm !== undefined) bits.push(`이동 ${f(l.trans_mm, 2)} mm`);
    if (l.rect_ms !== undefined) bits.push(`정류맵 ${f(l.rect_ms, 1)} ms`);
    if (l.why) bits.push(esc(l.why));
    parts.push(`<small>마지막 키프레임: ${bits.join(', ')}</small>`);
  }
  return parts.join(' · ');
}

// 고급 → 세부 상태 (열려 있을 때만 그림)
function renderDetails(s, se, ui, vio, ph, dp, mp) {
  const relay = ui.relay || {};
  const srows = [['모드', `<b>${MODE_KO[sessMode] || esc(sessMode)}</b>`],
    ['USB', `${esc(ui.usb)} · adb ${adbOk ? esc(se.adb.kind) : '<span class="warn">없음</span>'}`],
    ['중계', `${esc(relay.state)}${relay.pid ? ' pid ' + relay.pid : ''}${relay.starts ? ' · 시작 ' + relay.starts + '회' : ''}${relay.fails ? ' · 연속 실패 ' + relay.fails : ''}${relay.error ? ` <span class="bad">${esc(relay.error)}</span>` : ''}`]];
  if (relay.log_tail && relay.log_tail.length) srows.push(['중계 로그', `<small>${relay.log_tail.map(esc).join('<br>')}</small>`]);
  if (ui.stream) srows.push(['영상', `${esc(ui.stream.size)} · ${ui.stream.fps} fps · ${esc(FMT_KO[ui.stream.stream_fmt] || ui.stream.stream_fmt || '-')}`
    + (ui.stream.live_kbps ? ` · ${(ui.stream.live_kbps / 1000).toFixed(1)} Mbps × 2` : '')]);
  if (s.stream && s.stream.pair) srows.push(['깊이 렌즈', `${esc(PAIR_KO[s.stream.pair] || s.stream.pair)} (렌즈 ${esc((s.stream.physical || []).join('·'))}) · 두 렌즈 사이 ${f(s.stream.baseline_mm, 1)} mm · 정류 초점 ${f(s.stream.rect && s.stream.rect.f, 0)} px`
    + ` · <small>${esc(s.stream.calib_file || '')} ${esc(s.stream.calib_src || '')}</small>`
    + (s.stream.want && s.stream.want.pair && s.stream.want.pair !== s.stream.pair ? ` · 다음 시작: ${esc(PAIR_KO[s.stream.want.pair] || s.stream.want.pair)}` : '')
    + (s.stream.rec3 ? ` · <b>녹화: 렌즈 3개 모두 (앱 ${esc(s.stream.app_physical || '')})</b>` : '')]);
  if (se.app) srows.push(['앱',`${esc(se.app.state || '')} <small>${esc(se.app.raw || '')}</small>`]);
  if (se.rec) srows.push(['녹화', `<small>${esc(se.rec.raw || se.rec.state || '')}</small>`]);
  if (ui.temp_c !== null && ui.temp_c !== undefined) srows.push(['폰 온도', `${f(ui.temp_c)} °C (SKIN)${s.preview ? ' <span class="warn">가짜</span>' : ''}`]);
  $('sess').innerHTML = srows.map(r => `<tr><td>${r[0]}</td><td>${r[1]}</td></tr>`).join('');
  const rates = Object.entries(ph.imu_rates || {}).map(([k, v]) => `${k.replace('_uncalibrated', '_u')} ${v}`).join(', ');
  const rows = [
    ['위치 추적', `${vio.error ? `<span class="bad">${esc(vio.error)}</span>` : esc(vio.status || '-')} · init ${vio.init ? '예' : '아니오'} · ${f(vio.ms)} ms · SLAM ${f(vio.n_slam, 0)} / MSCKF ${f(vio.n_msckf, 0)}${vio.auto_resets ? ` · 자동 재시작 ${vio.auto_resets}` : ''}${vio.bridge_restarts ? ` · 브리지 다시 띄움 ${vio.bridge_restarts}` : ''}${vio.bridge_exceptions ? ` · 브리지 내부 오류 ${vio.bridge_exceptions}` : ''}`],
    ['추적 입력', `${vio.vio_mode === 'mono' ? '한쪽 렌즈 + IMU (mono, 기본)' : '두 렌즈 (stereo)'} · 스테레오 ${vio.stereo_frames || 0} / mono ${vio.mono_frames || 0} · 버림 ${vio.cams_dropped || 0} · IMU ${vio.imu_sent || 0}`],
    ['렌즈 시각차', (vio.skew_n ? `중앙 ${f(vio.skew_med_ms, 2)} ms · p95 ${f(vio.skew_p95_ms, 2)} ms · ` : '')
      + `기준: 쌍 묶기 ${f(vio.pair_skew_ms ?? ph.pair_skew_ms, 1)} ms (반 프레임 미만)`
      + (vio.vio_mode === 'stereo' ? ` · 추적(VIO) ${f(vio.skew_limit_ms, 1)} ms` : '')
      + ` · 짝 없어 버림 ${Array.isArray(ph.dropped) ? ph.dropped.reduce((a, b) => a + b, 0) : (ph.dropped || 0)}`
      + (ph.pair_drop_pct !== null && ph.pair_drop_pct !== undefined
        ? ` (최근 ${f(ph.pair_drop_window_s, 0)}초 <span class="${ph.pair_drop_pct >= PAIR_DROP_BAD ? 'bad' : ph.pair_drop_pct >= PAIR_DROP_WARN ? 'warn' : 'ok'}">${f(ph.pair_drop_pct, 0)} %</span>)` : '')
      + (vio.vio_mode === 'stereo' ? ` · 추적 기준 넘어 버림 ${vio.skew_dropped || 0}` : '')],
    ['움직임 보정', motionRow(dp.motion)],
    ['렌즈 위상', syncRow(syncOf(s))],
    ['폰 영상', ph.connected ? `${(ph.fps || []).map(v => v.toFixed(1)).join(' / ')} fps · 쌍 ${ph.pairs} · 렌즈 시각차 ${f(ph.skew_ms, 2)} ms` : '접속 대기'],
    ...h264Rows(ph.h264),
    ['IMU Hz', rates || '-'],
    ['IMU 동기', `${ph.imu_sync ? ph.imu_sync.out : 0} 출력 / ${ph.imu_sync ? ph.imu_sync.dropped : 0} 버림 · 시계차 ${f(ph.clock_gap_ms)} ms`],
    ['깊이', dp.error ? `<span class="bad">${esc(dp.error)}</span>` : (dp.ready ? `FFS ${f(dp.ffs_ms)} ms · YOLO ${f(dp.yolo_ms)} ms · ${f(dp.fps, 2)} 키프레임/s · 총 ${f(dp.keyframes, 0)}`
      : dp.loading ? '<span class="warn">모델 불러오는 중…</span>' : '아직 안 불러옴 (대기 중에는 GPU 를 안 씀 — 스캔·복원을 시작하면 불러옴)')],
    ['복셀', `${mp.shown || 0} 표시 / ${mp.voxels || 0} · ${((mp.voxel_m || 0) * 100).toFixed(1)} cm`],
  ];
  if (s.epi && s.epi.status === 'ok') {
    // 기준은 크기 배율만큼 (640x480 에서 1 px, 1440x1080 이면 2.25 px) — scan3d 가 epi.gate_px 로 줌
    const gate = s.epi.gate_px || 1.0;
    const good = s.epi.dy_med < gate && s.epi.inlier_frac >= 0.7;
    const d640 = (s.epi.dy_med_640 != null && s.epi.size && s.epi.size[0] !== 640) ? ` (640 기준 ${f(s.epi.dy_med_640, 2)} px)` : '';
    rows.push(['정류 검증', `<span class="${good ? 'ok' : 'warn'}">세로 어긋남 ${f(s.epi.dy_med, 2)} px${d640} · 기준 ${f(gate, 2)} px · inlier ${f(s.epi.inlier_frac, 2)}</span>${good ? '' : ' (정보: 보정이 조금 어긋남 — 다른 크기면 폰이 화각을 잘랐는지 확인)'}`]);
  }
  $('stat').innerHTML = rows.map(r => `<tr><td>${r[0]}</td><td>${r[1]}</td></tr>`).join('');
  $('log').textContent = (s.log || []).join('\n');
}

// ---------- 큰 버튼 ----------
// 시작 명령에 화질(size)·fps 를 함께 보낸다. 서버(load_model)가 불가능 조합이면 ok:false + reasons 로 거절.
async function startScan(askMsg) {
  const m = $('sMode').value;
  if (sessMode !== 'idle' && !confirm(askMsg || `지금 스캔(${MODE_KO[sessMode] || sessMode})을 멈추고 새로 시작할까요?`)) return;
  $('bStart').disabled = true;
  try {
    const all = Q.pair === 'all' && m === 'record';            // 3개 모두 녹화 (비교용) — 폰에 녹화만
    const pair = all ? 'all' : (Q.pair === 'all' ? '2+6' : Q.pair);
    const r = await cmd({ cmd: 'mode', mode: m, start_app: $('cStartApp').checked && adbOk, size: Q.size, fps: Q.fps, stream_fmt: Q.fmt, pair, live_pair: '2+6' });
    if (r.ok === false || r.error) alert(r.error || '시작 실패');
    else toast(r.warn ? `시작합니다 — ${r.warn}` : `시작합니다 (${Q.size} · ${Q.fps} fps${Q.fmt !== 'h264' ? ' · ' + (FMT_KO[Q.fmt] || Q.fmt) : ''}${pair !== '2+5' ? ' · ' + PAIR_KO[pair] : ''}) — 폰 화면을 켜 두세요`
      + (all ? '. 끝나면 두 쌍(초광각+망원, 초광각+광각)으로 따로 복원합니다' : ''), r.warn || all ? 8000 : 4000);
  } catch (e) { alert('서버에 연결할 수 없습니다'); }
  $('bStart').disabled = false;
}
$('bStart').onclick = () => startScan();
$('bStop').onclick = async () => {
  try {
    const r = await cmd({ cmd: 'stop', stop_app: $('cStartApp').checked && adbOk });
    // 녹화를 멈췄으면: 폰이 영상 마무리 → PC 로 가져오기 → (켜져 있으면) 자동 복원 → 목록에 [보기] (rec_sync.py)
    if (r && r.pull) toast(r.pull.auto_pull
      ? `녹화를 마무리하고 PC 로 가져옵니다${r.pull.auto_recon ? ' → 끝나면 자동으로 3D 복원 → 오른쪽 목록에서 [보기]' : ''}`
      : '녹화를 멈췄습니다 — 자동 가져오기가 꺼져 있어 오른쪽 목록에서 [가져오기] 를 누르세요', 8000);
  } catch (e) { }
  loadRecs(false);
};
$('bRestart').onclick = async () => {
  if (!confirm('위치 추적을 다시 시작하고 3D 맵을 지웁니다. 계속할까요?\n(누른 뒤 폰을 2초 가만히 두세요)')) return;
  try { const r = await cmd({ cmd: 'restart' }); if (r.ok === false) alert(r.error || '실패'); } catch (e) { }
};
$('bSave').onclick = async () => {
  toast('저장 중…');
  try { const r = await cmd({ cmd: 'save' }); toast(r.ok ? `저장됨: ${r.dir}` : `저장 실패: ${r.error}`, 8000); }
  catch (e) { toast('저장 실패 (서버 연결 없음)'); }
};

// ---------- 고급: 녹화본 ----------
let lastPullState = '';
function renderRecs(j) {
  let h = '<h4>폰</h4>';
  if (j.phone_error) h += `<div class="warn">${esc(j.phone_error)}</div>`;
  else if (!j.phone || !j.phone.length) h += '<div class="warn">녹화 없음 (또는 아직 새로고침 안 함)</div>';
  for (const r of j.phone || []) {
    h += `<div class="rec"><span title="${esc(r.base)}">${esc(r.name)} <small>${(r.size_kb / 1024).toFixed(1)} MB${r.local ? ' · 가져옴' : ''}</small></span>`
      + `<span><button data-pull="${esc(r.name)}" data-replay="1">가져와서 복원</button> <button data-pull="${esc(r.name)}" data-replay="0">가져오기</button></span></div>`;
  }
  h += `<h4>PC (${esc(j.rec_root)})</h4>`;
  if (!j.local || !j.local.length) h += '<div class="warn">없음</div>';
  for (const r of j.local || []) {
    h += `<div class="rec"><span title="${esc(r.path)}">${esc(r.name)} <small>${r.size_mb} MB${r.recon.length ? ' · 복원 ' + r.recon.length + '회' : ''}${r.has_video ? '' : ' · 영상 없음'}</small></span>`
      + `<span><button data-replay-local="${esc(r.name)}">복원</button></span></div>`;
  }
  $('recs').innerHTML = h;
  const opts = () => ({ every_frame: $('cEvery').checked, iters: parseInt($('iIters').value) || 8 });
  $('recs').querySelectorAll('[data-pull]').forEach(b => b.onclick = async () => {
    const rep = b.dataset.replay === '1';
    if (rep && sessMode !== 'idle' && !confirm('가져온 뒤 지금 스캔을 멈추고 복원합니다. 계속할까요?')) return;
    const r = await cmd(Object.assign({ cmd: 'pull', name: b.dataset.pull, replay: rep }, opts()));
    if (!r.ok) alert(r.error || '실패');
  });
  $('recs').querySelectorAll('[data-replay-local]').forEach(b => b.onclick = async () => {
    if (sessMode !== 'idle' && !confirm('지금 스캔을 멈추고 복원할까요? (맵이 새로 시작됩니다)')) return;
    const r = await cmd(Object.assign({ cmd: 'replay', name: b.dataset.replayLocal }, opts()));
    if (!r.ok) alert(r.error || '실패');
  });
}
let lastRecsSig = '';
async function loadRecs(refresh) {
  try {
    const j = await (await fetch(`/api/recordings${refresh ? '?refresh=1' : ''}`, { cache: 'no-store' })).json();
    const sig = JSON.stringify([j, sessMode]);
    if (sig === lastRecsSig && !refresh) return;          // 바뀐 게 없으면 다시 안 그림 (버튼이 깜박이지 않게)
    lastRecsSig = sig;
    renderRecs(j); renderItems(j);
  }
  catch (e) { $('recs').innerHTML = '<div class="bad">목록을 못 읽었습니다</div>'; }
}
$('bRecList').onclick = () => { $('recs').innerHTML = '<div class="warn">읽는 중…</div>'; loadRecs(true); };

// ---------- 고급: 보기·맵 ----------
$('bPause').onclick = () => cmd({ cmd: paused ? 'resume' : 'pause' });
$('bResetMap').onclick = () => { if (confirm('3D 맵을 지울까요?')) cmd({ cmd: 'reset_map' }); };
$('bResetVio').onclick = () => { if (confirm('위치 추적과 맵을 다시 시작할까요? 폰을 2초 가만히 두세요.')) cmd({ cmd: 'reset_vio' }); };
document.querySelectorAll('.cm').forEach(b => b.onclick = () => { V.mode = b.dataset.m; document.querySelectorAll('.cm').forEach(x => x.classList.toggle('on', x === b)); recolorAll(); rvRecolor(); });
$('sVoxel').onchange = e => cmd({ cmd: 'set', voxel_m: parseFloat(e.target.value) });
$('iHits').onchange = e => cmd({ cmd: 'set', min_hits: parseInt(e.target.value) });
$('cBoxes').onchange = e => { objGroup.visible = e.target.checked; };
$('cTraj').onchange = e => { trajLine.visible = e.target.checked; };
$('adv').addEventListener('toggle', () => { try { localStorage.setItem('scan3d.adv', $('adv').open ? '1' : '0'); } catch (e) { } });
try { if (localStorage.getItem('scan3d.adv') === '1') $('adv').open = true; } catch (e) { }
try { const m = localStorage.getItem('scan3d.mode'); if (m) $('sMode').value = m; } catch (e) { }
$('sMode').onchange = () => { try { localStorage.setItem('scan3d.mode', $('sMode').value); } catch (e) { } renderPairSel(true); loadCombos(); };

// ---------- 깊이 화면 (2D, 최근 키프레임) — 물체·원본 영상은 오른쪽 아래 카메라 창이 맡는다 ----------
function poll2d() {
  const hidden = $('img2d').style.display === 'none' || document.hidden;
  if (!hidden && !RV.on && (sessMode === 'live' || sessMode === 'both' || sessMode === 'replay')) {   // 녹화 중엔 깊이 그림 없음 (REVIEW5 N3)
    const img = new Image();
    img.onload = () => { $('img2d').src = img.src; };
    img.src = `/api/frame.jpg?kind=depth&_=${Date.now()}`;
  }
  setTimeout(poll2d, 800);
}
$('bHide2d').onclick = () => {
  const img = $('img2d'), hide = img.style.display !== 'none';
  img.style.display = hide ? 'none' : '';
  $('bHide2d').textContent = hide ? '보기' : '숨김';
};

// ---------- 오른쪽 아래 카메라 창 2개 ----------
// lens 0 = 초광각 (렌즈 2, 정류 왼쪽), 1 = 광각 (렌즈 5). 센서 방향 그대로 오므로 [90° 돌리기] 로 세워 볼 수 있다.
const CAM = { busy: [false, false], urls: [null, null], fps: 3, overlay: false, rot: false, hidden: false };
try { CAM.overlay = localStorage.getItem('scan3d.camOverlay') === '1'; CAM.rot = localStorage.getItem('scan3d.camRot') !== '0';
  CAM.hidden = localStorage.getItem('scan3d.camHidden') === '1'; } catch (e) { }
function camLenses(cams) {
  if (!Array.isArray(cams) || RV.on) return;            // 리플레이 중에는 녹화 렌즈 이름 (rvOpen)
  for (const c of cams) { const el = $('camName' + c.lens); if (el) el.textContent = `${c.name} (렌즈 ${c.physical})`; }
}
function camApplyUi() {
  $('camDock').classList.toggle('rot', CAM.rot);
  $('camDock').classList.toggle('hide', CAM.hidden);
  $('cCamOverlay').checked = CAM.overlay; $('cCamRot').checked = CAM.rot;
  $('bCamHide').textContent = CAM.hidden ? '카메라 보기' : '숨김';
}
function camActive() { return !CAM.hidden && !document.hidden && (sessMode === 'live' || sessMode === 'both' || sessMode === 'replay'); }
function camFetch(lens) {
  if (CAM.busy[lens]) return;                       // 앞 요청이 안 끝났으면 건너뜀 (느린 연결에서 쌓이지 않게)
  CAM.busy[lens] = true;
  const ov = CAM.overlay && lens === 0 ? 1 : 0;
  fetch(`/api/cam.jpg?lens=${lens}&overlay=${ov}&_=${Date.now()}`, { cache: 'no-store' })
    .then(r => r.status === 200 ? r.blob() : null)
    .then(b => {
      const box = $('cam' + lens);
      if (!b) { box.classList.add('empty'); return; }
      const u = URL.createObjectURL(b);
      box.querySelector('img').src = u;
      box.classList.remove('empty');
      if (CAM.urls[lens]) URL.revokeObjectURL(CAM.urls[lens]);
      CAM.urls[lens] = u;
    })
    .catch(() => { })
    .finally(() => { CAM.busy[lens] = false; });
}
function camLoop() {
  if (RV.on) { /* 리플레이: rvCamLoop 가 그 시각의 녹화 프레임을 넣음 */ }
  else if (camActive()) { camFetch(0); camFetch(1); }
  else if (!CAM.hidden) for (const l of [0, 1]) $('cam' + l).classList.add('empty');
  setTimeout(camLoop, 1000 / CAM.fps);
}
for (const l of [0, 1]) $('cam' + l).onclick = () => {
  const big = !$('cam' + l).classList.contains('big');
  for (const k of [0, 1]) $('cam' + k).classList.toggle('big', big && k === l);
};
$('cCamOverlay').onchange = e => { CAM.overlay = e.target.checked; try { localStorage.setItem('scan3d.camOverlay', CAM.overlay ? '1' : '0'); } catch (x) { } };
$('cCamRot').onchange = e => { CAM.rot = e.target.checked; camApplyUi(); try { localStorage.setItem('scan3d.camRot', CAM.rot ? '1' : '0'); } catch (x) { } };
$('bCamHide').onclick = () => { CAM.hidden = !CAM.hidden; camApplyUi(); try { localStorage.setItem('scan3d.camHidden', CAM.hidden ? '1' : '0'); } catch (x) { } };
camApplyUi();

// ---------- 미리보기 띠 (preview_server.py 의 가짜 데이터) ----------
function renderPreview(s) {
  const el = $('previewBand');
  if (el) el.hidden = !s.preview;
  document.title = s.preview ? '3D 스캐너 (미리보기 — 가짜 데이터)' : '3D 스캐너';
}

// ---------- 화질·fps 선택 + 예상 부하 (/api/combos, state.ui.stream · state.ui.load) ----------
// 화질 = 4:3 크기만. 회색(disabled) 항목은 이유를 글자로 붙인다 (option 툴팁은 브라우저마다 달라서).
// 막대: 한계 = 막대의 2/3 지점 눈금, 흰 줄 = 스캔 중 실측. 과부하가 6초 넘게 이어지면 [한 단계 낮추기] 제안.
// 영상 형식 (고급): H.264 = 기본·권장 (폰 하드웨어 압축), JPEG·NV21 은 예전 방식.
const FMT_KO = { h264: 'H.264', jpeg: 'JPEG', nv21: 'NV21 (압축 없음)' };
const Q = { tab: null, size: '1440x1080', fps: 15, fmt: 'h264', pair: '2+5', state: null, sigS: '', sigF: '', kf: null };
// 깊이 렌즈 쌍: 2+5 = 초광각+광각 (기본), 2+6 = 초광각+망원 (기선 2배, 거의 동시 촬영, 화면 가운데만)
const PAIR_KO = { '2+5': '초광각+광각', '2+6': '초광각+망원', all: '렌즈 3개 모두 녹화 (비교용)' };
// '3개 모두 녹화 (비교용)' 은 [폰에 녹화만] 에서만 고를 수 있다 (실시간 깊이는 한 쌍만). 다른 방식으로 바꾸면 초광각+망원 으로.
function renderPairSel(changed) {
  const rec = $('sMode').value === 'record', o = $('sPair').querySelector('option[value="all"]');
  if (o) { o.disabled = !rec; o.textContent = rec ? '3개 모두 녹화 (비교용)' : '3개 모두 녹화 (비교용) — 폰에 녹화만에서'; }
  if (!rec && Q.pair === 'all') {
    Q.pair = '2+5'; $('sPair').value = Q.pair;
    if (changed) toast('3개 모두 녹화는 [폰에 녹화만] 에서만 됩니다 — 깊이 렌즈를 초광각+광각 으로 바꿨습니다', 6000);
  }
}
try {
  const pr = localStorage.getItem('scan3d.pair.v2'); if (pr && PAIR_KO[pr]) Q.pair = pr;
  $('sPair').value = Q.pair; renderPairSel(false);
  Q.size = localStorage.getItem('scan3d.size.v2') || Q.size;
  Q.fps = parseInt(localStorage.getItem('scan3d.fps')) || Q.fps;
  const fm = localStorage.getItem('scan3d.fmt'); if (fm && FMT_KO[fm]) Q.fmt = fm;
  $('sFmt').value = Q.fmt;
  const k = parseFloat(localStorage.getItem('scan3d.kf')); if (k > 0) { Q.kf = k; $('iKf').value = k; }
} catch (e) { }
const comboOf = (size, fps) => (Q.tab && Q.tab.combos) ? Q.tab.combos[`${size}@${fps}`] : null;
function shortWhy(c) {
  if (!c || c.ok) return '';
  const r = (c.reasons || [])[0] || '';
  if (/^폰 (JPEG|영상 복사)/.test(r)) return '폰이 힘듦';
  if (/인코더/.test(r)) return /^녹화/.test(r) ? '녹화가 안 됨' : '폰 영상 압축기가 못 함';
  if (/^USB/.test(r)) return 'USB 가 모자람';
  if (/^PC 깊이/.test(r)) return 'PC 깊이 계산이 느림';
  if (/^PC 영상/.test(r)) return 'PC 가 힘듦';
  if (/^카메라/.test(r)) return '카메라가 못 냄';
  if (/^폰이 두 렌즈/.test(r)) return '두 렌즈 동시에 못 엶';
  if (/^위치 추적 설정/.test(r)) return '추적 설정이 다른 크기로 고정';
  if (/^녹화/.test(r)) return '녹화가 안 됨';
  return r;
}
function saveQ() { try { localStorage.setItem('scan3d.size.v2', Q.size); localStorage.setItem('scan3d.fps', String(Q.fps)); } catch (e) { } }
async function loadCombos() {
  try {
    const kf = Q.kf ? `&kf=${Q.kf}` : '';
    const l3 = Q.pair === 'all' && $('sMode').value === 'record' ? '&lenses=3' : '';
    const j = await (await fetch(`/api/combos?mode=${encodeURIComponent($('sMode').value)}${kf}&fmt=${encodeURIComponent(Q.fmt)}${l3}`, { cache: 'no-store' })).json();
    if (j && j.combos) { Q.tab = j; renderQSel(); renderLoad(); renderCoeffs(); }
  } catch (e) { }
}
function fillSelect(sel, items, value) {
  // items: [{value, label, disabled, title}] — 내용이 같으면 다시 안 그림 (열려 있는 목록이 닫히지 않게)
  const sig = JSON.stringify([items, value]);
  if (sel.dataset.sig === sig) return;
  sel.dataset.sig = sig;
  sel.innerHTML = items.map(it => `<option value="${esc(it.value)}"${it.disabled ? ' disabled' : ''} title="${esc(it.title || '')}">${esc(it.label)}</option>`).join('');
  sel.value = String(value);
}
function renderQSel() {
  const t = Q.tab; if (!t) return;
  // usable = 가능 또는 "예상 부하만 넘음"(load_only, 시작은 허용 — REVIEW6 R2). 자동 고르기는 완전히 되는 것 먼저
  const usable = c => !!c && (c.ok || c.load_only);
  const okSize = s => t.fps.some(f => usable(comboOf(s, f)));
  const fullOk = s => t.fps.some(f => (comboOf(s, f) || {}).ok);
  if (!t.sizes.some(s => s.size === Q.size) || !okSize(Q.size)) {
    Q.size = fullOk(t.default.size) ? t.default.size : ((t.sizes.find(s => fullOk(s.size)) || t.sizes.find(s => okSize(s.size)) || t.sizes[0] || {}).size || '640x480');
  }
  // REVIEW6 R6: 회색 크기의 이유 = 카메라가 받는 fps 중 가장 낮은 조합의 이유 (없으면 첫 fps)
  const whyOf = size => { const cs = t.fps.map(f => comboOf(size, f)).filter(Boolean);
    return cs.find(c => !(c.reasons || []).some(r => /^카메라에 고정|^카메라 최대/.test(r))) || cs[0]; };
  const okF = t.fps.filter(f => usable(comboOf(Q.size, f)));
  if (!okF.includes(Q.fps)) {
    const lower = okF.filter(f => f <= Q.fps);
    Q.fps = lower.length ? lower[lower.length - 1] : (okF[0] || t.default.fps);
  }
  saveQ();
  fillSelect($('sSize'), t.sizes.map(s => {
    const ok = okSize(s.size), c0 = whyOf(s.size);
    return { value: s.size, disabled: !ok,
      label: s.size + (s.calib ? ' (보정 파일 크기)' : '') + (ok ? '' : ` — 안 됨: ${shortWhy(c0)}`),
      title: ok ? (s.calib ? '보정 파일 값 그대로 (보드 촬영은 960x720 에서 하고 줄인 값)' : `보정은 ${t.calib} 값을 비례 변환해서 씁니다`) : (c0 ? c0.reasons.join(' · ') : '') };
  }), Q.size);
  fillSelect($('sFps'), t.fps.map(f => {
    const c = comboOf(Q.size, f) || {};
    return { value: f, disabled: !usable(c),
      label: `${f} fps` + (c.ok ? '' : c.load_only ? ` — 부하 높음 (${shortWhy(c)})` : ` — 안 됨: ${shortWhy(c)}`),
      title: c.ok ? '' : (c.reasons || []).join(' · ') };
  }), Q.fps);
}
$('sSize').onchange = e => { Q.size = e.target.value; renderQSel(); renderLoad(); };
$('sFps').onchange = e => { Q.fps = parseInt(e.target.value) || Q.fps; saveQ(); renderLoad(); };
$('sPair').onchange = e => {
  Q.pair = PAIR_KO[e.target.value] ? e.target.value : '2+5';
  renderPairSel(true);
  try { localStorage.setItem('scan3d.pair.v2', Q.pair); } catch (x) { }
  if (Q.pair === '2+6') toast('초광각+망원: 깊이는 화면 가운데(망원이 보는 곳)만 나옵니다. 다음 [▶ 스캔 시작] 부터 적용', 6000);
  if (Q.pair === 'all') toast('렌즈 3개를 한꺼번에 녹화합니다. 끝나면 초광각+망원, 초광각+광각 두 쌍으로 따로 복원해 리플레이에서 비교할 수 있습니다', 8000);
  loadCombos();                                    // 녹화 압축기 3개로 다시 계산
};
$('sFmt').onchange = e => {
  Q.fmt = FMT_KO[e.target.value] ? e.target.value : 'h264';
  try { localStorage.setItem('scan3d.fmt', Q.fmt); } catch (x) { }
  if (Q.fmt !== 'h264') toast(`${FMT_KO[Q.fmt]} 는 폰·USB 부담이 큽니다. 보통은 H.264 (권장) 를 쓰세요`, 6000);
  loadCombos();                                    // 형식마다 가능한 조합이 다르다 (서버가 다시 계산)
};

function setBar(id, frac, measFrac, text, show = true) {
  const el = $(id); el.hidden = !show; if (!show) return;
  const v = el.querySelector('i.v'), m = el.querySelector('i.m');
  if (v) {
    const w = Math.max(0, Math.min(1, frac / 1.5));                 // 한계(1.0) = 막대 2/3
    v.style.width = (100 * w).toFixed(1) + '%';
    v.className = 'v' + (frac > 1 ? ' bad' : frac > 0.75 ? ' warn' : '');
  }
  if (m) {
    if (measFrac === null || measFrac === undefined) m.style.display = 'none';
    else { m.style.display = 'block'; m.style.left = `calc(${(100 * Math.max(0, Math.min(1, measFrac / 1.5))).toFixed(1)}% - 1px)`; }
  }
  el.querySelector('b').innerHTML = text;
}
function renderLoad() {
  const t = Q.tab, s = Q.state || {}, ui = s.ui || {}, ld = ui.load || null, st = ui.stream || null;
  const box = $('loadBox'); if (!t) { box.hidden = true; return; }
  box.hidden = false;
  const mode = $('sMode').value, live = mode !== 'record', rec = mode !== 'live';
  const L = t.limits || {};
  const running = ld && ld.active;
  // 스캔 중이고 고른 값이 지금 도는 값과 같으면 실측을 같이 보여 준다
  const fmt = Q.fmt;                                 // (표는 이 형식으로 다시 읽는 중일 수 있음 — 곧 맞춰짐)
  const runFmt = running ? (ld.stream_fmt || (st && st.stream_fmt) || 'h264') : null;
  const same = running && ld.size === Q.size && +ld.fps === +Q.fps && runFmt === fmt;
  const p = (same && ld.predicted) || comboOf(Q.size, Q.fps) || {};
  const me = same ? (ld.measured || {}) : {};
  const kfMin = L.kf_min || 3, kfBudget = 1000 / kfMin;
  const meas = (v, u, n = 0) => v === undefined || v === null ? '' : ` <small>실제 ${typeof v === 'number' ? v.toFixed(n) : v}${u}</small>`;
  // 폰 막대: JPEG = 압축 시간, H.264·NV21 = 영상 복사 시간 (H.264 압축은 폰의 영상 전용 칩이 하므로 CPU 부담이 거의 없음)
  const phPct = p.phone_cpu_pct !== undefined ? p.phone_cpu_pct : p.phone_jpeg_pct;
  const mePh = me.phone_cpu_pct !== undefined ? me.phone_cpu_pct : me.phone_jpeg_pct;
  $('lbJpegLt').textContent = fmt === 'jpeg' ? '폰 압축' : '폰 부하';
  setBar('lbJpeg', (phPct || 0) / (L.phone_jpeg_pct || 60), mePh !== undefined ? mePh / (L.phone_jpeg_pct || 60) : null,
    `${f(phPct, 0)}%${meas(mePh, '%')}`, live);
  setBar('lbUsb', (p.usb_mbs || 0) / (L.usb_mbs || 8), me.usb_mbs !== undefined ? me.usb_mbs / (L.usb_mbs || 8) : null,
    `${f(p.usb_mbs, 1)} MB/s${meas(me.usb_mbs, ' MB/s', 1)}`, live);
  setBar('lbPc', (p.pc_kf_ms || 0) / kfBudget, me.pc_kf_ms !== undefined ? me.pc_kf_ms / kfBudget : null,
    `${f(p.pc_ffs_fps, 1)} 장/초${meas(me.pc_ffs_fps, ' 장/초', 1)}`, live);
  setBar('lbLive', 0, null, p.live_kbps ? `${(p.live_kbps / 1000).toFixed(1)} Mbps × 2${me.live_kbps ? ` <small>실제 ${(me.live_kbps / 1000).toFixed(1)}</small>` : ''}` : '-',
    live && fmt === 'h264');
  setBar('lbRec', 0, null, p.rec_kbps ? `${(p.rec_kbps / 1000).toFixed(1)} Mbps × ${p.n_rec || 2}` : '-', rec);
  $('lbJpeg').title = fmt === 'jpeg'
    ? `폰 JPEG 압축: 두 렌즈 합, 한 코어 시간 대비 (한계 ${L.phone_jpeg_pct || 60}%). 한 장 ${f(p.phone_jpeg_ms)} ms${me.phone_jpeg_ms_max ? ` · 실제 최대 ${me.phone_jpeg_ms_max} ms` : ''}`
    : `폰 CPU: 영상을 ${fmt === 'h264' ? '압축 칩에' : 'USB 로'} 넘기는 복사 시간, 두 렌즈 합 (한계 ${L.phone_jpeg_pct || 60}%)${p.h264_surface ? ' — 카메라가 압축 칩에 바로 연결돼 거의 0' : ''}`;
  $('lbLive').title = '폰 → PC 실시간 영상 비트레이트 (렌즈마다, H.264). 클수록 깨끗하지만 USB 를 더 씁니다';
  $('lbUsb').title = `폰 → PC 영상 (한계 ${L.usb_mbs || 8} MB/s)${fmt === 'h264' ? ' — H.264 는 비트레이트 합 + 여유' : ''}`;
  renderFmtInfo(p, me, fmt, running, runFmt);
  $('lbPc').title = `PC 깊이(FFS)+물체(YOLO) 한 장 ${f(p.pc_kf_ms, 0)} ms → 1초에 ${f(p.pc_ffs_fps, 1)} 장 (목표 ${kfMin} 장 이상)`;
  $('lbRec').title = '폰 녹화 비트레이트 (렌즈마다)';
  // 보정 안내 + 정류 검증 (비례 변환한 크기일 때 계속 보여 준다)
  const notes = [];
  const curSize = running ? ld.size : null;
  if (Q.size !== t.calib) {
    let n = `보정은 ${t.calib} 값을 비례 변환`;
    if (s.epi && s.epi.status === 'ok') n += ` · 정류 검증: 세로 어긋남 ${f(s.epi.dy_med, 2)} px (기준 ${f(s.epi.gate_px || 1, 2)} px)`;
    notes.push(n);
  }
  if (running && (curSize !== Q.size || +ld.fps !== +Q.fps || runFmt !== fmt))
    notes.push(`지금 ${ld.size} · ${ld.fps} fps${runFmt !== fmt ? ' · ' + (FMT_KO[runFmt] || runFmt) : ''} 로 도는 중 — [▶ 다시 시작] 하면 바뀝니다`);
  if (!running && st && sessMode !== 'idle' && (st.size !== Q.size || +st.fps !== +Q.fps)) notes.push(`지금 ${st.size} · ${st.fps} fps`);
  if (p.ok === false && p.load_only) notes.push(`<span class="warn">예상 부하가 높음 (시작은 됨): ${esc((p.reasons || []).join(' · '))}</span>`);
  else if (p.ok === false) notes.push(`<span class="bad">이 조합은 안 됨: ${esc((p.reasons || []).join(' · '))}</span>`);
  if ((p.notes || []).some(x => /추정/.test(x))) notes.push('폰 능력표가 아직 없어 일부는 추정');
  $('loadNote').innerHTML = notes.map(x => x.startsWith('<') ? x : esc(x)).join(' · ');
  // 과부하 → 한 단계 낮추기
  const flags = running ? (ld.flags || []) : [];
  const w = $('loadWarn');
  w.hidden = !(running && (ld.overload || flags.some(x => x !== '폰이 따뜻함')));
  if (!w.hidden) {
    $('loadFlags').textContent = '⚠ ' + (flags.join(' · ') || '부하가 큽니다');
    const sg = ld.suggest;
    $('bStepDown').hidden = !sg;
    if (sg) { $('bStepDown').textContent = `한 단계 낮추기 → ${sg.size} · ${sg.fps} fps`; $('bStepDown').dataset.size = sg.size; $('bStepDown').dataset.fps = sg.fps; }
  }
}
// 고급 → 영상 형식 아래 한 줄: 예상 비트레이트·인코더 수, 스캔 중이면 실제 kbps·폰 압축 지연·PC 풀기 시간
// (실측 이름 measured.live_kbps / enc_ms / enc_ms_max / pc_dec_ms / pc_dec_ms_max — 통합 때 서버와 맞춤)
function renderFmtInfo(p, me, fmt, running, runFmt) {
  const el = $('fmtInfo'); if (!el) return;
  const parts = [];
  if (fmt === 'h264') {
    parts.push(p.live_kbps ? `비트레이트 ${(p.live_kbps / 1000).toFixed(1)} Mbps × 2 (렌즈마다)` : '비트레이트 -');
    if (p.enc_n) parts.push(`폰 압축기 ${p.enc_n}개 동시${p.enc_shared ? ' (녹화와 같이 씀)' : ''}`);
    parts.push('키프레임 1초마다 · 지연 적은 설정');
  } else if (fmt === 'jpeg') parts.push(`JPEG: 폰 CPU 가 한 장씩 압축 (한 장 약 ${f(p.phone_jpeg_ms)} ms)`);
  else parts.push('NV21: 압축 없이 보냄 — USB 가 금방 모자랍니다');
  if (p.pc_dec_ms) parts.push(`PC 풀기 한 장 약 ${f(p.pc_dec_ms)} ms`);
  let h = parts.map(esc).join(' · ');
  if (running) {
    const m = [];
    if (me.live_kbps) m.push(`실제 ${(me.live_kbps / 1000).toFixed(1)} Mbps`);
    if (me.enc_ms !== undefined) m.push(`폰 압축 지연 ${f(me.enc_ms)} ms${me.enc_ms_max ? ` (최대 ${f(me.enc_ms_max)})` : ''}`);
    if (me.pc_dec_ms !== undefined) m.push(`PC 풀기 ${f(me.pc_dec_ms)} ms${me.pc_dec_ms_max ? ` (최대 ${f(me.pc_dec_ms_max)})` : ''}`);
    if (me.phone_copy_ms !== undefined) m.push(`폰 복사 ${f(me.phone_copy_ms, 2)} ms`);
    if (m.length) h += `<br><b>스캔 중</b>: ${m.map(esc).join(' · ')}`;
    else if (runFmt && runFmt !== fmt) h += `<br>지금은 ${esc(FMT_KO[runFmt] || runFmt)} 로 도는 중`;
  }
  el.innerHTML = h;
}
$('bStepDown').onclick = async () => {
  const b = $('bStepDown');
  Q.size = b.dataset.size; Q.fps = parseInt(b.dataset.fps) || Q.fps;
  saveQ(); renderQSel(); renderLoad();
  await startScan(`${Q.size} · ${Q.fps} fps 로 낮춰서 다시 시작할까요? (3D 맵은 새로 시작됩니다)`);
};

// 고급 → 화질·부하 계산: 계수 보기·초기화, 능력표 다시 읽기, 깊이 목표
function renderCoeffs() {
  const t = Q.tab; if (!t || !t.coeffs || !$('adv').open) return;
  const c = t.coeffs;
  $('capsSrc').textContent = `능력표: ${t.caps_src} · 계수 갱신: ${c.updated || '아직 없음 (처음 추정값)'}`;
  $('coeffs').innerHTML = '<tr><td>항목</td><td>지금 값 (처음 값 · 실측 수)</td></tr>' + Object.keys(c.values).map(k =>
    `<tr><td>${esc((c.info || {})[k] || k)}</td><td><b>${f(c.values[k], 3)}</b> <small>(${f(c.defaults[k], 3)}${c.ewma.includes(k) ? ` · ${c.n_obs[k] || 0}회` : ' · 고정'})</small></td></tr>`).join('');
}
$('adv').addEventListener('toggle', renderCoeffs);
$('bCoeffReset').onclick = async () => {
  if (!confirm('스캔하며 배운 부하 계수를 지우고 처음 추정값으로 돌릴까요?')) return;
  try { const r = await cmd({ cmd: 'load_reset' }); toast(r.ok ? '계수를 초기화했습니다' : (r.error || '실패')); } catch (e) { }
  loadCombos();
};
$('bCapsRefresh').onclick = async () => {
  toast('폰 능력표 읽는 중…');
  try { const r = await cmd({ cmd: 'caps_refresh' }); toast(r.ok ? `능력표: ${r.caps_src}` : (r.error || '실패')); } catch (e) { }
  loadCombos();
};
$('iKf').onchange = e => {
  const v = parseFloat(e.target.value);
  Q.kf = v > 0 ? v : null;
  try { localStorage.setItem('scan3d.kf', Q.kf ? String(Q.kf) : ''); } catch (x) { }
  loadCombos();
};

// ---------- 녹화본 목록 (오른쪽 칸): 녹화마다 상태 한 줄 (rec_sync.py items) ----------
const ST_KO = { pulling: '가져오는 중', recon: '3D 복원 중', ready: '리플레이 가능', recon_wait: '복원 대기', held: '복원 멈춤',
  failed: '복원 실패', bad: '녹화 파일이 덜 끝남', pc_only: 'PC 에 있음', recording: '녹화 중', incomplete: '덜 끝난 녹화 (안 가져옴)',
  pull_wait: '가져오기 대기', pull_failed: '가져오기 실패', phone_only: '폰에만 있음', mismatch: 'PC 사본이 폰과 다름' };
const ST_CLS = { ready: 'ok', failed: 'bad', pull_failed: 'bad', bad: 'warn', mismatch: 'warn', held: 'warn', incomplete: 'warn' };
const recPrev = {};
function recLabel(n) {
  const m = /^rec_(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})$/.exec(n);
  return m ? `${+m[2]}월 ${+m[3]}일 ${m[4]}:${m[5]}:${m[6]} 녹화` : n;
}
function renderItems(j) {
  const items = j.items || [], au = j.auto || {};
  if (document.activeElement !== $('cAutoPull')) $('cAutoPull').checked = au.pull !== false;
  if (document.activeElement !== $('cAutoRecon')) $('cAutoRecon').checked = au.recon !== false;
  const live = sessMode === 'live' || sessMode === 'both' || sessMode === 'record';
  const notes = [];
  if (!au.phone) notes.push('폰이 연결돼 있지 않음 — PC 에 있는 녹화만 보입니다');
  if (au.paused) notes.push('스캔 중이라 가져오기·복원은 쉬는 중 (끝나면 이어서)');
  $('recNote').textContent = notes.join(' · ');
  if (!items.length) { $('recItems').innerHTML = '<div class="empty">녹화가 없습니다 — [폰에 녹화만] 또는 [녹화 + 실시간] 으로 찍으면 여기에 생깁니다</div>'; return; }
  let h = '';
  for (const it of items) {
    const st = it.state || '', pct = it.pct !== undefined && it.pct !== null ? it.pct : null;
    let stTxt = ST_KO[st] || st;
    if (pct !== null && (st === 'pulling' || st === 'recon')) stTxt += ` ${pct.toFixed(0)}%`;
    if (st === 'recon' && it.step && it.step !== '재생 중') stTxt += ` (${it.step})`;
    if (st === 'recon' && it.recon_pair) stTxt += ` · ${it.recon_pair}`;
    if (it.pair_recons && st === 'ready') stTxt = Object.values(it.pair_recons).every(Boolean) ? '두 쌍 비교 가능' : '리플레이 가능 (한 쌍)';
    const where = [it.pair_recons ? '렌즈 3개' : '', it.on_phone ? `폰 ${it.phone_mb} MB` : '', it.on_pc ? `PC ${it.pc_mb} MB` : ''].filter(Boolean).join(' · ');
    let btn = '';
    const dis = live ? ' disabled title="실시간 스캔 중에는 볼 수 없습니다 — 먼저 [■ 정지]"' : '';
    if (it.pair_recons) {
      // 3렌즈 녹화 (비교용): 쌍마다 [2+5 보기] [2+6 보기] — 하나를 열면 리플레이 위쪽에서 다른 쌍으로 바로 바꿔 볼 수 있음
      const parts = [];
      let anyFail = false;
      for (const p of ['2+5', '2+6']) {
        if (!(p in it.pair_recons)) continue;
        const rn = it.pair_recons[p], fe = (it.pair_failed || {})[p];
        if (rn) parts.push(`<button class="go" data-view="${esc(it.name)}" data-recon="${esc(rn)}"${dis || ` title="${esc(PAIR_KO[p])} 로 복원한 결과 보기"`}>${p} 보기</button>`);
        else if (st === 'recon' && it.recon_pair === p) parts.push(`<small>${p} 복원 중</small>`);
        else if (fe) { anyFail = true; parts.push(`<small class="bad" title="${esc(fe)}">${p} 실패</small>`); }
        else if ((it.pair_held || []).includes(p)) { anyFail = true; parts.push(`<small class="warn">${p} 멈춤</small>`); }
        else parts.push(`<small>${p} 대기</small>`);
      }
      if (anyFail) parts.push(`<button data-retry="${esc(it.name)}" title="실패·멈춘 쌍을 다시 복원합니다">다시 복원</button>`);
      btn = `<span class="pb">${parts.join(' ')}</span>`;
    }
    else if (st === 'ready') btn = `<button class="go" data-view="${esc(it.name)}" data-recon="${esc(it.recon)}"${live ? ' disabled title="실시간 스캔 중에는 볼 수 없습니다 — 먼저 [■ 정지]"' : ' title="리플레이 보기"'}>▶ 보기</button>`;
    else if (st === 'failed' || st === 'held' || st === 'pc_only' || st === 'bad') btn = `<button data-retry="${esc(it.name)}" title="이 녹화를 3D 로 복원합니다 (스캔 중이 아닐 때)">복원</button>`;
    else if (st === 'phone_only' || st === 'pull_failed') btn = `<button data-get="${esc(it.name)}">가져오기</button>`;
    let sub = esc(where);
    if (it.error) sub += ` · <span class="bad" title="${esc(it.error)}">${esc(String(it.error).slice(0, 60))}</span>`;
    if (it.paused && (st === 'pull_wait' || st === 'recon_wait')) sub += ' · 스캔이 끝나면 이어서';
    if (it.recons && it.recons.length && st !== 'ready' && st !== 'recon') sub += ` · 옛 복원 ${it.recons.length}개 (시간 정보 없음)`;
    const del = it.can_delete_phone ? `<button class="del" data-del="${esc(it.name)}" title="PC 에 똑같은 사본이 있을 때만 폰에서 지웁니다">폰에서 지우기</button>` : '';
    h += `<div class="ri ${esc(st)}"><div class="l1"><span class="nm" title="${esc(it.name)}">${esc(recLabel(it.name))}</span>`
      + `<span class="st ${ST_CLS[st] || ''}">${esc(stTxt)}</span></div>`
      + `<div class="l2"><span>${sub}</span><span>${del} ${btn}</span></div>`
      + (pct !== null && (st === 'pulling' || st === 'recon') ? `<div class="bar"><i style="width:${Math.min(100, pct).toFixed(1)}%"></i></div>` : '')
      + '</div>';
    if (recPrev[it.name] && recPrev[it.name] !== 'ready' && st === 'ready') toast(`${recLabel(it.name)} — 리플레이 준비됨. 오른쪽 목록에서 [▶ 보기]`, 8000);
    recPrev[it.name] = st;
  }
  $('recItems').innerHTML = h;
  $('recItems').querySelectorAll('[data-view]').forEach(b => b.onclick = () => rvOpen(b.dataset.view, b.dataset.recon));
  $('recItems').querySelectorAll('[data-retry]').forEach(b => b.onclick = async () => {
    const r = await cmd({ cmd: 'rec_retry', name: b.dataset.retry });
    toast(r.ok ? (sessMode === 'idle' ? '곧 복원을 시작합니다' : '스캔이 끝나면 복원합니다') : (r.error || '실패'));
    lastRecsSig = ''; loadRecs(false);
  });
  $('recItems').querySelectorAll('[data-get]').forEach(b => b.onclick = async () => {
    const r = await cmd({ cmd: 'pull', name: b.dataset.get, replay: false });
    toast(r.ok ? (r.note || '가져오기를 시작합니다') : (r.error || '실패'));
    lastRecsSig = ''; loadRecs(false);
  });
  $('recItems').querySelectorAll('[data-del]').forEach(b => b.onclick = async () => {
    if (!confirm(`${recLabel(b.dataset.del)} 을(를) 폰에서 지울까요?\nPC 에 똑같은 사본이 있는지 한 번 더 확인한 뒤 지웁니다. 되돌릴 수 없습니다.`)) return;
    const r = await cmd({ cmd: 'rec_delete_phone', name: b.dataset.del });
    toast(r.ok ? '폰에서 지웠습니다 (PC 사본은 그대로)' : (r.error || '실패'), 6000);
    lastRecsSig = ''; loadRecs(true);
  });
}
$('cAutoPull').onchange = e => cmd({ cmd: 'rec_auto', pull: e.target.checked }).then(() => { lastRecsSig = ''; loadRecs(false); });
$('cAutoRecon').onchange = e => cmd({ cmd: 'rec_auto', recon: e.target.checked }).then(() => { lastRecsSig = ''; loadRecs(false); });

// ---------- 복원 리플레이: [최종 맵] · [시간 따라 보기] (replay_view.py 의 timeline.json + voxels_t.bin) ----------
// 복셀은 처음 보인 시각 순으로 정렬돼 오므로 시각 t 까지 보이는 수 = 이분 탐색 → InstancedMesh.count 만 바꾼다 (다시 안 만듦).
// 비교용 3렌즈 녹화: 쌍마다 복원 결과 (RV.sets[쌍]) 를 불러 두고 위쪽 [초광각+광각 2+5] [초광각+망원 2+6] 로 바꿔 본다
// (같은 시간 막대·보기 방식 유지, 카메라 창은 같은 녹화의 렌즈 2 + 그 쌍의 다른 렌즈). 비교 숫자는 #rpCmp.
const RV = { on: false, rec: null, recon: null, pair: null, tl: null, info: null, sets: {}, mode: 'final', t: 0, dur: 0, playing: false,
  speed: 1, mesh: null, M: 0, times: null, rgb: null, cls: null, inst: null, kfT: null, trajT: null, trajRows: null, trajPos: null,
  depthKf: -1, cmpK: null, last: 0, drag: false,
  cam: { want: 0, got: [null, null], busy: [false, false], urls: [null, null], retry: [0, 0] } };
const rvTraj = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xffcf57 }));
rvTraj.frustumCulled = false; rvTraj.visible = false; scene.add(rvTraj);
const rvFrustum = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x44ddff }));
rvFrustum.frustumCulled = false; rvFrustum.visible = false; scene.add(rvFrustum);
function upperBound(a, x) { let lo = 0, hi = a.length; while (lo < hi) { const m = (lo + hi) >> 1; if (a[m] <= x) lo = m + 1; else hi = m; } return lo; }
function rvColorDs(ds, i) {
  const r = ds.rgb[i * 3], g = ds.rgb[i * 3 + 1], b = ds.rgb[i * 3 + 2], cl = ds.cls[i], ins = ds.inst[i];
  const dim = [r * 0.35, g * 0.35, b * 0.35];
  const c = V.mode === 'class' ? (cl >= 0 ? clsRGB(cl) : dim) : V.mode === 'inst' ? (ins >= 0 && cl >= 0 ? instRGB(ins) : dim) : [r, g, b];
  tmpC.setRGB(c[0] / 255, c[1] / 255, c[2] / 255);
  ds.mesh.setColorAt(i, tmpC);
}
function rvRecolor() {
  for (const ds of Object.values(RV.sets)) {
    for (let i = 0; i < ds.M; i++) rvColorDs(ds, i);
    if (ds.mesh.instanceColor) ds.mesh.instanceColor.needsUpdate = true;
  }
}
function rvFrustumAt(row) {
  // row = [t, r00 r01 r02 x, r10 r11 r12 y, r20 r21 r22 z] — rect0 카메라 (x 오른쪽, y 아래, z 앞) 의 0.12 m 절두체
  const c = RV.tl.cam, d = 0.12;
  const P = (x, y, z) => [row[1] * x + row[2] * y + row[3] * z + row[4], row[5] * x + row[6] * y + row[7] * z + row[8], row[9] * x + row[10] * y + row[11] * z + row[12]];
  const o = P(0, 0, 0);
  const cs = [[0, 0], [c.w, 0], [c.w, c.h], [0, c.h]].map(([u, v]) => P((u - c.cx) / c.f * d, (v - c.cy) / c.f * d, d));
  const segs = [];
  for (let i = 0; i < 4; i++) segs.push(...o, ...cs[i], ...cs[i], ...cs[(i + 1) % 4]);
  const up = P(0, -d * 0.8, d); segs.push(...cs[0], ...up, ...up, ...cs[1]);
  rvFrustum.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(segs), 3));
  rvFrustum.visible = true;
  if ($('cFollow').checked) orbit.target.set(o[0], o[1], o[2]);
}
// 복원 결과 하나 (timeline + voxels_t.bin) → 그릴 거리 (장면에는 아직 안 넣음)
async function rvFetch(rec, recon) {
  const q = `rec=${encodeURIComponent(rec)}${recon ? '&recon=' + encodeURIComponent(recon) : ''}`;
  const tl = await (await fetch('/api/replay/info?' + q, { cache: 'no-store' })).json();
  if (tl.error) throw new Error(tl.error);
  const buf = await (await fetch(`/api/replay/voxels?rec=${encodeURIComponent(rec)}&recon=${encodeURIComponent(tl.recon)}`, { cache: 'no-store' })).arrayBuffer();
  const dv = new DataView(buf);
  const magic = String.fromCharCode(dv.getUint8(0), dv.getUint8(1), dv.getUint8(2), dv.getUint8(3));
  if (magic !== 'S3VT') throw new Error('리플레이 파일 형식이 다릅니다');
  const M = dv.getUint32(8, true), vm = dv.getFloat32(12, true);
  const tOff = 24 + 17 * M + ((4 - ((24 + 17 * M) % 4)) % 4);
  const mesh = new THREE.InstancedMesh(boxGeo, boxMat, Math.max(1, M));
  mesh.frustumCulled = false;
  mesh.setColorAt(0, new THREE.Color(1, 1, 1));
  const ds = { rec, recon: tl.recon, pair: tl.pair || null, tl, mesh, M, times: new Float32Array(buf, tOff, M),
    rgb: new Uint8Array(M * 3), cls: new Int16Array(M), inst: new Int32Array(M), lo: [1e9, 1e9, 1e9], hi: [-1e9, -1e9, -1e9] };
  const s = vm * 0.92, lo = ds.lo, hi = ds.hi;
  for (let k = 0, o = 24; k < M; k++, o += 17) {
    const x = (dv.getInt16(o, true) + 0.5) * vm, y = (dv.getInt16(o + 2, true) + 0.5) * vm, z = (dv.getInt16(o + 4, true) + 0.5) * vm;
    tmpM.makeScale(s, s, s); tmpM.setPosition(x, y, z); mesh.setMatrixAt(k, tmpM);
    ds.rgb[k * 3] = dv.getUint8(o + 6); ds.rgb[k * 3 + 1] = dv.getUint8(o + 7); ds.rgb[k * 3 + 2] = dv.getUint8(o + 8);
    ds.cls[k] = dv.getInt16(o + 9, true); ds.inst[k] = dv.getInt32(o + 11, true);
    rvColorDs(ds, k);
    if (x < lo[0]) lo[0] = x; if (y < lo[1]) lo[1] = y; if (z < lo[2]) lo[2] = z;
    if (x > hi[0]) hi[0] = x; if (y > hi[1]) hi[1] = y; if (z > hi[2]) hi[2] = z;
  }
  mesh.instanceMatrix.needsUpdate = true; if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
  const rows = tl.trajectory || [];
  ds.trajPos = new Float32Array(rows.length * 3);
  rows.forEach((r, i) => { ds.trajPos[i * 3] = r[4]; ds.trajPos[i * 3 + 1] = r[8]; ds.trajPos[i * 3 + 2] = r[12]; });
  ds.trajRows = rows; ds.trajT = rows.map(r => r[0]);
  ds.kfT = (tl.keyframes || []).map(k => k.t);
  ds.dur = Math.max(tl.duration_s || 0, tl.t_max || 0, ds.trajT.length ? ds.trajT[ds.trajT.length - 1] : 0, 0.1);
  return ds;
}
const dsKey = ds => ds.pair || 'one';
async function rvOpen(rec, recon) {
  if (sessMode === 'live' || sessMode === 'both' || sessMode === 'record') { toast('실시간 스캔 중에는 리플레이를 볼 수 없습니다 — 먼저 [■ 정지]', 6000); return; }
  toast('리플레이 불러오는 중…', 20000);
  let ds;
  try { ds = await rvFetch(rec, recon); }
  catch (e) { toast(''); alert(e && e.message ? e.message : '리플레이를 불러오지 못했습니다 (서버 연결)'); return; }
  rvClose(true);
  RV.rec = rec; RV.info = ds.tl; RV.sets = {}; RV.sets[dsKey(ds)] = ds;
  RV.on = true; RV.mesh = null;
  // 실시간 화면 숨김 (복셀·궤적·절두체·물체 상자)
  if (V.mesh) V.mesh.visible = false;
  trajLine.visible = false; frustum.visible = false; objGroup.visible = false;
  rvUse(ds, true);
  $('rpBar').hidden = false;
  rvSetMode('final');
  rvPairBtns();
  const two = ds.tl.pair_recons && Object.values(ds.tl.pair_recons).filter(Boolean).length > 1;
  toast((ds.tl.thumbs ? '리플레이를 열었습니다' : '리플레이를 열었습니다 — 카메라 화면은 준비 중 (잠시 뒤 보입니다)')
    + (two ? ' · 위쪽 렌즈 쌍 버튼으로 다른 쌍 결과와 바꿔 볼 수 있습니다' : ''), 6000);
  rvPreload();
}
// 보고 있는 결과를 ds 로 (시각·보기 방식은 그대로)
function rvUse(ds, fit) {
  if (RV.mesh && RV.mesh !== ds.mesh) scene.remove(RV.mesh);
  if (RV.mesh !== ds.mesh) scene.add(ds.mesh);
  Object.assign(RV, { recon: ds.recon, pair: ds.pair, tl: ds.tl, mesh: ds.mesh, M: ds.M, times: ds.times, rgb: ds.rgb, cls: ds.cls,
    inst: ds.inst, trajRows: ds.trajRows, trajT: ds.trajT, kfT: ds.kfT, trajPos: ds.trajPos });
  RV.dur = Math.max(...Object.values(RV.sets).map(x => x.dur), ds.dur);
  rvTraj.geometry.setAttribute('position', new THREE.BufferAttribute(ds.trajPos, 3));
  rvTraj.geometry.computeBoundingSphere();
  RV.depthKf = -1; RV.cmpK = null; RV.cam.got = [null, null]; RV.cam.retry = [0, 0];
  if (fit && ds.M) {
    const lo = ds.lo, hi = ds.hi;
    orbit.target.set((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2);
    orbit.r = Math.max(0.6, 0.8 * Math.hypot(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]));
  }
  for (const L of ds.tl.lenses || []) { const el = $('camName' + L.lens); if (el) el.textContent = `${L.name} (렌즈 ${L.physical}) · 녹화`; }
  const multi = !!(RV.info && RV.info.pair_recons);
  $('rpTitle').textContent = `리플레이: ${recLabel(RV.rec)}${multi && ds.pair ? ` · ${PAIR_KO[ds.pair] || ds.pair} (${ds.pair})` : ''}`;
  $('rpInfo').textContent = `복셀 ${ds.M.toLocaleString()} · 키프레임 ${ds.kfT.length} · ${RV.dur.toFixed(1)} s`;
}
async function rvSwitch(pair) {
  if (!RV.on || pair === RV.pair) return;
  let ds = RV.sets[pair];
  if (!ds) {
    const rn = ((RV.info || {}).pair_recons || {})[pair];
    if (!rn) return;
    toast('불러오는 중…', 10000);
    try { ds = await rvFetch(RV.rec, rn); } catch (e) { toast(''); alert(e && e.message ? e.message : '불러오지 못했습니다'); return; }
    if (!RV.on || RV.rec !== ds.rec) { ds.mesh.dispose(); return; }
    RV.sets[pair] = ds;
  }
  rvUse(ds, false);
  if (RV.mode === 'final') rvSetMode('final'); else rvSetT(RV.t);
  rvPairBtns(); rvCmp(true);
  toast(`${PAIR_KO[pair] || pair} (${pair}) 결과로 바꿨습니다`, 2500);
}
async function rvPreload() {
  // 다른 쌍 결과도 미리 불러 둔다 (바꿀 때 바로, 이 시각 키프레임 비교 숫자도)
  const pr = RV.info && RV.info.pair_recons, rec = RV.rec;
  if (!pr) return;
  for (const [p, rn] of Object.entries(pr)) {
    if (!rn || RV.sets[p]) continue;
    try {
      const ds = await rvFetch(rec, rn);
      if (!RV.on || RV.rec !== rec || RV.sets[p]) { ds.mesh.dispose(); return; }
      RV.sets[p] = ds; RV.dur = Math.max(RV.dur, ds.dur);
      rvPairBtns(); rvCmp(true);
    } catch (e) { }
  }
}
function rvPairBtns() {
  const pr = RV.info && RV.info.pair_recons, el = $('rpPairs');
  if (!pr) { el.innerHTML = ''; return; }
  el.innerHTML = ['2+5', '2+6'].filter(p => p in pr).map(p => `<button data-rp="${p}" class="${p === RV.pair ? 'on' : ''}"`
    + `${pr[p] ? ` title="${esc(PAIR_KO[p])} 로 복원한 결과"` : ' disabled title="이 쌍은 아직 복원 결과가 없습니다"'}>${esc(PAIR_KO[p])} ${p}</button>`).join('');
  el.querySelectorAll('[data-rp]').forEach(b => b.onclick = () => rvSwitch(b.dataset.rp));
}
// ---- 비교 숫자 (replay_view.kf_quality / quality_summary / pair_compare) ----
const BIN_KO = { '0-1': '0–1 m', '1-2': '1–2 m', '2+': '2 m 넘게' };
const pct = v => v === null || v === undefined ? '-' : `${(100 * v).toFixed(0)}%`;
function cmpTxt(c) {
  if (!c) return '결과 없음';
  const parts = [`키프레임 ${c.keyframes}`];
  if (c.voxels_shown !== null && c.voxels_shown !== undefined)
    parts.push(`복셀 ${c.voxels_shown.toLocaleString()}${c.voxels_total ? ` (전체 ${c.voxels_total.toLocaleString()})` : ''}`);
  parts.push(`깊이 유효 ${pct(c.valid_med)}`);
  if (c.plane_res_mm !== null && c.plane_res_mm !== undefined) {
    const b = Object.entries(c.plane_bins || {}).map(([k, v]) => `${BIN_KO[k] || k} ${f(v.res_mm, 1)}`).join(' · ');
    parts.push(`평면 오차 <b>${f(c.plane_res_mm, 1)} mm</b>${b ? ` (${b})` : ''}`);
  } else parts.push('평면 못 찾음');
  return parts.join(' · ');
}
function kfQTxt(q) {
  if (!q) return '-';
  return `유효 ${pct(q.valid)} · 평면 ${q.plane ? `<b>${f(q.plane.res_mm, 1)} mm</b>` : '못 찾음'}`;
}
function rvCmp(force) {
  const el = $('rpCmp');
  if (!RV.on) { el.hidden = true; return; }
  const t = RV.mode === 'final' ? null : RV.t;
  const sets = Object.entries(RV.sets).sort();
  const key = JSON.stringify([RV.pair, t === null ? -1 : sets.map(([, ds]) => upperBound(ds.kfT, t))]);
  if (!force && key === RV.cmpK) return;
  RV.cmpK = key;
  const pc = RV.info && RV.info.pair_compare, lines = [];
  if (pc && pc.pairs) {
    for (const p of ['2+5', '2+6']) {
      if (!(p in pc.pairs)) continue;
      const e = pc.pairs[p], cur = p === RV.pair;
      lines.push(`<span class="${cur ? 'cur' : ''}">${cur ? '▶ ' : ''}${esc(PAIR_KO[p])} ${p}: ${e ? cmpTxt(e.compare) : '아직 결과 없음'}</span>`);
    }
    const cm = pc.common;
    if (cm && cm.n) lines.push(`같은 키프레임 ${cm.n}개끼리: ` + ['2+5', '2+6'].filter(p => cm.by_pair[p]).map(p => {
      const c = cm.by_pair[p];
      return `${p} 평면 <b>${f(c.plane_res_mm, 1)} mm</b> · 유효 ${pct(c.valid_med)}`;
    }).join(' / '));
    const vr = Object.values(pc.pairs).find(e => e && e.vio_reused);
    if (vr) lines.push(`위치 추적: 두 쌍이 같은 궤적 (${esc(vr.vio_reused)} 의 포즈를 다시 씀) — 깊이만 다릅니다`);
    else if (Object.values(pc.pairs).every(Boolean)) lines.push('<span class="warn">위치 추적을 쌍마다 따로 돌렸습니다 — 궤적이 조금 다를 수 있음</span>');
  } else if (RV.tl && RV.tl.compare) lines.push(cmpTxt(RV.tl.compare));
  if (t !== null) {
    const cur = [];
    for (const [p, ds] of sets) {
      const k = upperBound(ds.kfT, t) - 1, kf = k >= 0 ? ds.tl.keyframes[k] : null;
      cur.push(`${p === 'one' ? '' : p + ' '}${kf ? `${kfQTxt(kf.q)} <small>(${kf.t.toFixed(1)} s)</small>` : '-'}`);
    }
    if (cur.length) lines.push('이 시각 키프레임: ' + cur.join(' / '));
  }
  if (lines.length) lines.push('<small>평면 오차 = 키프레임마다 가장 큰 평면(벽·바닥)에서 점들이 떨어진 거리의 중앙값, 작을수록 평평하게 나옴</small>');
  el.innerHTML = lines.map(x => `<div>${x}</div>`).join('');
  el.hidden = !lines.length;
}
function rvClose(silent) {
  for (const ds of Object.values(RV.sets)) { scene.remove(ds.mesh); ds.mesh.dispose(); }
  if (RV.mesh) { scene.remove(RV.mesh); RV.mesh = null; }
  RV.sets = {}; RV.info = null; RV.pair = null;
  const was = RV.on;
  RV.on = false; RV.playing = false; RV.M = 0;
  rvTraj.visible = false; rvFrustum.visible = false;
  if (V.mesh) V.mesh.visible = true;
  trajLine.visible = $('cTraj').checked; objGroup.visible = $('cBoxes').checked;
  $('rpBar').hidden = true; $('rpCmp').hidden = true; $('rpPairs').innerHTML = '';
  if (was && !silent) {
    $('img2d').removeAttribute('src');
    for (const l of [0, 1]) $('cam' + l).classList.add('empty');
  }
}
function rvSetMode(m) {
  RV.mode = m;
  $('rpFinal').classList.toggle('on', m === 'final');
  $('rpTime').classList.toggle('on', m === 'time');
  $('rpCtl').hidden = m !== 'time';
  if (m === 'final') {
    RV.playing = false;
    RV.mesh.count = RV.M;
    rvTraj.geometry.setDrawRange(0, RV.trajRows.length); rvTraj.visible = $('cTraj').checked;
    rvFrustum.visible = false;
    RV.cam.want = RV.dur;
    rvDepthAt(RV.dur);
    rvCmp();
  } else {
    if (RV.t >= RV.dur - 1e-3) RV.t = 0;
    rvSetT(RV.t);
  }
  rvPlayBtn();
}
function rvPlayBtn() { $('rpPlay').textContent = RV.playing ? '❚❚' : '▶'; }
function rvDepthAt(t) {
  const k = upperBound(RV.kfT, t) - 1;
  if (k === RV.depthKf) return;
  RV.depthKf = k;
  const kf = k >= 0 ? RV.tl.keyframes[k] : null;
  if (kf && kf.depth) $('img2d').src = `/api/replay/depth.jpg?rec=${encodeURIComponent(RV.rec)}&recon=${encodeURIComponent(RV.recon)}&kf=${kf.i}`;
  else $('img2d').removeAttribute('src');
}
function rvSetT(t) {
  RV.t = Math.max(0, Math.min(RV.dur, t));
  RV.mesh.count = upperBound(RV.times, RV.t);
  const k = upperBound(RV.trajT, RV.t);
  rvTraj.geometry.setDrawRange(0, k); rvTraj.visible = $('cTraj').checked && k > 1;
  if (k > 0) rvFrustumAt(RV.trajRows[k - 1]); else rvFrustum.visible = false;
  if (!RV.drag) $('rpSlider').value = String(Math.round(1000 * RV.t / RV.dur));
  $('rpClock').textContent = `${RV.t.toFixed(1)} / ${RV.dur.toFixed(1)} s · 복셀 ${RV.mesh.count.toLocaleString()}`;
  RV.cam.want = RV.t;
  rvDepthAt(RV.t);
  rvCmp();
}
function rvTick() {
  const now = performance.now();
  const dt = Math.min(0.1, (now - (RV.last || now)) / 1000);
  RV.last = now;
  if (!RV.on || RV.mode !== 'time' || !RV.playing || RV.drag) return;
  let t = RV.t + dt * RV.speed;
  if (t >= RV.dur) { t = RV.dur; RV.playing = false; rvPlayBtn(); }
  rvSetT(t);
}
// 카메라 창 2개: 그 시각의 녹화 프레임 (서버 thumbs/). 렌즈마다 요청 하나씩만, 100 ms 마다 (재생·끌기 중 최대 약 10 장/초)
function rvCamLoop() {
  if (RV.on && !CAM.hidden && !document.hidden) {
    const tq = Math.round(RV.cam.want * 50) / 50;          // 0.02 s 단위 (브라우저 캐시가 잘 맞게)
    for (const l of [0, 1]) {
      if (RV.cam.busy[l] || RV.cam.got[l] === tq || performance.now() < RV.cam.retry[l]) continue;
      RV.cam.busy[l] = true;
      const rec = RV.rec, pair = RV.pair;
      // 3렌즈 녹화: 카메라 창 = 보고 있는 쌍의 렌즈 (렌즈 2 + 5 또는 6)
      const L = ((RV.tl || {}).lenses || [])[l], phys = L ? `&phys=${encodeURIComponent(L.physical)}` : '';
      fetch(`/api/replay/frame.jpg?rec=${encodeURIComponent(rec)}&lens=${l}&t=${tq.toFixed(2)}${phys}`)
        .then(r => r.status === 200 ? r.blob() : null)
        .then(b => {
          if (!RV.on || RV.rec !== rec || RV.pair !== pair) return;
          const box = $('cam' + l);
          if (!b) { box.classList.add('empty'); RV.cam.retry[l] = performance.now() + 2000; return; }
          const u = URL.createObjectURL(b);
          box.querySelector('img').src = u; box.classList.remove('empty');
          if (RV.cam.urls[l]) URL.revokeObjectURL(RV.cam.urls[l]);
          RV.cam.urls[l] = u; RV.cam.got[l] = tq;
        })
        .catch(() => { RV.cam.retry[l] = performance.now() + 2000; })
        .finally(() => { RV.cam.busy[l] = false; });
    }
  }
  setTimeout(rvCamLoop, 100);
}
function rvStep() {
  const nm = recLabel(RV.rec || '');
  if (RV.mode === 'final') return { level: 'ok', step: 0, done: 0, main: `리플레이 — ${nm} · 최종 맵`,
    sub: '복원이 끝난 전체 맵과 지나온 길입니다. [시간 따라 보기] 를 누르면 맵이 채워지는 과정을 녹화 시간대로 다시 봅니다.' };
  return { level: 'ok', step: 0, done: 0, main: `리플레이 — ${nm} · 시간 따라 보기`,
    sub: '▶ 로 재생, 막대를 끌면 그 시각으로 갑니다. 하늘색 사각뿔 = 그때 폰 위치·방향, 오른쪽 아래 = 그 시각의 녹화 화면.' };
}
function rvOnState() {
  // 실시간 스캔이 시작되면 리플레이를 닫는다 (두 화면이 섞이지 않게)
  if (RV.on && (sessMode === 'live' || sessMode === 'both' || sessMode === 'record')) {
    rvClose();
    toast('스캔이 시작되어 리플레이를 닫았습니다', 6000);
  }
}
$('rpFinal').onclick = () => rvSetMode('final');
$('rpTime').onclick = () => rvSetMode('time');
$('rpClose').onclick = () => rvClose();
$('rpPlay').onclick = () => {
  if (RV.mode !== 'time') rvSetMode('time');
  if (!RV.playing && RV.t >= RV.dur - 1e-3) RV.t = 0;
  RV.playing = !RV.playing; RV.last = performance.now(); rvPlayBtn();
};
$('rpSpeed').onchange = e => { RV.speed = parseFloat(e.target.value) || 1; };
$('rpSlider').addEventListener('input', e => { RV.drag = true; rvSetT(RV.dur * (+e.target.value) / 1000); });
$('rpSlider').addEventListener('change', () => { RV.drag = false; });
$('rpSlider').addEventListener('pointerup', () => { RV.drag = false; });
document.addEventListener('keydown', e => {
  if (!RV.on || RV.mode !== 'time' || /INPUT|SELECT|TEXTAREA/.test((document.activeElement || {}).tagName || '')) return;
  if (e.code === 'Space') { e.preventDefault(); $('rpPlay').click(); }
  else if (e.code === 'ArrowRight') rvSetT(RV.t + 1);
  else if (e.code === 'ArrowLeft') rvSetT(RV.t - 1);
});
$('cTraj').addEventListener('change', () => { if (RV.on) { trajLine.visible = false; rvTraj.visible = $('cTraj').checked; } });
$('cBoxes').addEventListener('change', () => { if (RV.on) objGroup.visible = false; });

// ---------- 렌더 루프 ----------
function resize() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== Math.floor(w * renderer.getPixelRatio()) || canvas.height !== Math.floor(h * renderer.getPixelRatio())) {
    renderer.setSize(w, h, false); cam.aspect = w / Math.max(1, h); cam.updateProjectionMatrix();
  }
}
function loop() { resize(); rvTick(); applyOrbit(); renderer.render(scene, cam); drawLabels(); requestAnimationFrame(loop); }
allocVoxels(65536);
loop(); pollVoxels(); pollState(); pollTraj(); poll2d(); camLoop(); rvCamLoop(); loadRecs(false); loadCombos();
setInterval(() => { if (!document.hidden) loadRecs(false); }, 2000);   // 녹화본 상태 (가져오는 중 · 복원 중 · 리플레이 가능)
setInterval(() => { if (!document.hidden) loadCombos(); }, 10000);   // 계수가 실측으로 바뀌면 예상도 바뀐다
