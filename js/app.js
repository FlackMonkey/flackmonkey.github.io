import { crsfRcFrame, mavRcOverride, mavHeartbeat, CrsfParser, decodeTelemetry, bridgeSetFrame,
         bridgeGetFrame, crsfPingFrame } from './protocol.js';
import { connectPort, reconnectKnown, transportSupport } from './transport.js';

// =====================================================================
// Config model
// =====================================================================
const NCH = 16;
const STICK_SRC = ['roll', 'pitch', 'throttle', 'yaw'];
const SRC_LABEL = { roll: 'Stick: Roll (A)', pitch: 'Stick: Pitch (E)', throttle: 'Stick: Throttle (T)', yaw: 'Stick: Yaw (R)',
  sw2: 'Switch 2-pos', sw3: 'Switch 3-pos', slider: 'Slider', center: 'Fixed 1500', low: 'Fixed min' };
const MAPS = { AETR: ['roll', 'pitch', 'throttle', 'yaw'], TAER: ['throttle', 'roll', 'pitch', 'yaw'],
  AERT: ['roll', 'pitch', 'yaw', 'throttle'], RETA: ['yaw', 'pitch', 'throttle', 'roll'] };
const AXIS_NAME = { roll: 'Ail', pitch: 'Ele', throttle: 'Thr', yaw: 'Rud' };
const IMU_AXES = { tiltLR: 'Tilt left/right (°)', tiltFB: 'Tilt fwd/back (°)', rateX: 'Gyro X rate (°/s)',
  rateY: 'Gyro Y rate (°/s)', rateZ: 'Gyro Z rate / twist (°/s)' };

function defaultChannels(map = 'AETR') {
  const ch = [];
  for (let i = 0; i < NCH; i++) ch.push({ name: `AUX${i - 3}`, src: i < 8 ? 'sw2' : 'slider', rev: false, min: 1000, max: 2000, sub: 0, expo: 0 });
  MAPS[map].forEach((s, i) => Object.assign(ch[i], { name: AXIS_NAME[s], src: s, expo: s === 'throttle' ? 0 : 20 }));
  ch[4].name = 'ARM'; ch[4].src = 'sw2';
  ch[5].name = 'MODE'; ch[5].src = 'sw3';
  ch[6].name = 'BEEP'; ch[6].src = 'sw2';
  ch[7].name = 'AUX5'; ch[7].src = 'sw3';
  return ch;
}

const DEFAULT_CFG = () => ({
  ver: 1, stickMode: 2, map: 'AETR', channels: defaultChannels('AETR'),
  imu: { enabled: false, engage: 'toggle', zeroOnEngage: true, smoothing: 0.6,
    pairs: [
      { on: true, chX: 0, axX: 'tiltLR', rangeX: 30, invX: false, chY: 1, axY: 'tiltFB', rangeY: 30, invY: false, dead: 2 },
      { on: false, chX: 3, axX: 'rateZ', rangeX: 200, invX: false, chY: 2, axY: 'tiltFB', rangeY: 40, invY: false, dead: 2 },
      { on: false, chX: 8, axX: 'tiltLR', rangeX: 45, invX: false, chY: 9, axY: 'tiltFB', rangeY: 45, invY: false, dead: 0 },
      { on: false, chX: 10, axX: 'rateX', rangeX: 200, invX: false, chY: 11, axY: 'rateY', rangeY: 200, invY: false, dead: 0 }] },
  link: { prefer: 'auto', proto: 'crsf', baud: 460800, rateHz: 100, sysId: 255, compId: 190, targetSys: 1, targetComp: 1 },
  board: { outBaud: 400000, inverted: true, fixedPeriodUs: 0, failsafeMs: 500 },
  safety: { armCh: 4, armThrLow: true },
});

let cfg = loadCfg();
function loadCfg() {
  try { const s = JSON.parse(localStorage.getItem('elrsPhoneTx.cfg')); if (s && s.ver === 1) return deepMerge(DEFAULT_CFG(), s); } catch {}
  return DEFAULT_CFG();
}
function deepMerge(a, b) {
  for (const k in b) {
    if (b[k] && typeof b[k] === 'object' && !Array.isArray(b[k]) && a[k]) a[k] = deepMerge(a[k], b[k]);
    else a[k] = b[k];
  }
  return a;
}
let saveT;
function saveCfg() { clearTimeout(saveT); saveT = setTimeout(() => { try { localStorage.setItem('elrsPhoneTx.cfg', JSON.stringify(cfg)); } catch {} }, 300); }

// =====================================================================
// Live state
// =====================================================================
const input = { roll: 0, pitch: 0, throttle: -1, yaw: 0 };
const sw = new Array(NCH).fill(0);        // 0/1 for sw2, 0/1/2 for sw3
const slider = new Array(NCH).fill(-1);   // -1..1
const out = new Array(NCH).fill(1500);
const imu = { lr: 0, fb: 0, rx: 0, ry: 0, rz: 0, zLR: 0, zFB: 0, active: false, holding: false, seen: false };
const telem = { link: null, battery: null, fmode: '', bridge: null, devinfo: null, lastRx: 0 };
let port = null, parser = null, sendTimer = null, seq = 0, txCount = 0, txBusy = false, lastHb = 0;

const $ = s => document.querySelector(s);
const $$ = s => [...document.querySelectorAll(s)];
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const el = (tag, attrs = {}, ...kids) => {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k.startsWith('on')) e.addEventListener(k.slice(2), v);
    else if (k === 'class') e.className = v; else if (k === 'checked' || k === 'value') e[k] = v; else e.setAttribute(k, v);
  }
  for (const k of kids) if (k != null) e.append(k);
  return e;
};
function toast(msg, bad = false) {
  const t = $('#toast'); t.textContent = msg; t.className = bad ? 'show bad' : 'show';
  clearTimeout(toast.t); toast.t = setTimeout(() => t.className = '', 2500);
}
function log(m) {
  const l = $('#log'); const line = `[${new Date().toLocaleTimeString()}] ${m}\n`;
  l.textContent = (line + l.textContent).slice(0, 6000);
}

// =====================================================================
// Channel mixer
// =====================================================================
function imuChannelValues() {
  const res = {};
  if (!cfg.imu.enabled || !imu.active) return res;
  const src = { tiltLR: imu.lr - imu.zLR, tiltFB: wrap180(imu.fb - imu.zFB), rateX: imu.rx, rateY: imu.ry, rateZ: imu.rz };
  for (const p of cfg.imu.pairs) {
    if (!p.on) continue;
    const norm = (a, range, inv) => {
      const d = p.dead || 0; let v = Math.abs(a) <= d ? 0 : (a - Math.sign(a) * d) / Math.max(1, range - d);
      v = clamp(v, -1, 1); return inv ? -v : v;
    };
    if (p.chX >= 0) res[p.chX] = norm(src[p.axX], p.rangeX, p.invX);
    if (p.chY >= 0) res[p.chY] = norm(src[p.axY], p.rangeY, p.invY);
  }
  return res;
}
const wrap180 = a => ((a + 540) % 360) - 180;

function computeChannels() {
  const imuV = imuChannelValues();
  for (let i = 0; i < NCH; i++) {
    const c = cfg.channels[i]; let v;
    if (i in imuV) v = imuV[i];
    else switch (c.src) {
      case 'roll': case 'pitch': case 'throttle': case 'yaw': v = input[c.src]; break;
      case 'sw2': v = sw[i] ? 1 : -1; break;
      case 'sw3': v = sw[i] - 1; break;
      case 'slider': v = slider[i]; break;
      case 'low': v = -1; break;
      default: v = 0;
    }
    if (STICK_SRC.includes(c.src) && c.expo) { const e = c.expo / 100; v = (1 - e) * v + e * v * v * v; }
    if (c.rev) v = -v;
    const ctr = 1500 + (+c.sub || 0);
    out[i] = clamp(Math.round(v >= 0 ? ctr + v * (c.max - ctr) : ctr + v * (ctr - c.min)), 885, 2115);
  }
  return out;
}
const throttleCh = () => cfg.channels.findIndex(c => c.src === 'throttle');
const isArmed = () => { const a = cfg.safety.armCh; return a >= 0 && cfg.channels[a].src === 'sw2' && sw[a] === 1; };

// =====================================================================
// Sticks (multi-touch)
// =====================================================================
function setupStick(id) {
  const pad = $(id), knob = pad.querySelector('.knob');
  let pid = null;
  const axes = () => {
    const left = id === '#stickL';
    if (cfg.stickMode === 2) return left ? ['yaw', 'throttle'] : ['roll', 'pitch'];
    if (cfg.stickMode === 1) return left ? ['yaw', 'pitch'] : ['roll', 'throttle'];
    if (cfg.stickMode === 3) return left ? ['roll', 'pitch'] : ['yaw', 'throttle'];
    return left ? ['roll', 'throttle'] : ['yaw', 'pitch'];   // mode 4
  };
  const move = e => {
    const r = pad.getBoundingClientRect();
    const x = clamp(((e.clientX - r.left) / r.width) * 2 - 1, -1, 1);
    const y = clamp(1 - ((e.clientY - r.top) / r.height) * 2, -1, 1);
    const [ax, ay] = axes(); input[ax] = x; input[ay] = y; draw();
  };
  const release = () => {
    pid = null; const [ax, ay] = axes();
    input[ax] = 0; if (ay !== 'throttle') input[ay] = 0;   // throttle does not self-center
    draw();
  };
  const draw = () => {
    const [ax, ay] = axes();
    knob.style.left = `${(input[ax] + 1) * 50}%`; knob.style.top = `${(1 - input[ay]) * 50}%`;
    pad.classList.toggle('thr', ay === 'throttle');
  };
  pad.addEventListener('pointerdown', e => { if (pid !== null) return; pid = e.pointerId; pad.setPointerCapture(pid); move(e); e.preventDefault(); });
  pad.addEventListener('pointermove', e => { if (e.pointerId === pid) move(e); });
  pad.addEventListener('pointerup', e => { if (e.pointerId === pid) release(); });
  pad.addEventListener('pointercancel', e => { if (e.pointerId === pid) release(); });
  pad.redraw = draw; draw();
  return pad;
}

// =====================================================================
// Fly screen: aux switches & sliders
// =====================================================================
function buildAux() {
  const box = $('#aux'); box.innerHTML = '';
  cfg.channels.forEach((c, i) => {
    if (c.src === 'sw2' || c.src === 'sw3') {
      const b = el('button', { class: 'sw', onclick: () => {
        const n = c.src === 'sw2' ? 2 : 3; const next = (sw[i] + 1) % n;
        if (i === cfg.safety.armCh && next === 1 && c.src === 'sw2') {
          const t = throttleCh();
          if (cfg.safety.armThrLow && t >= 0 && input.throttle > -0.9) { toast('Throttle must be low to arm', true); return; }
          if (!port) { toast('Not connected', true); return; }
        }
        sw[i] = next; refreshAux();
      } });
      b.dataset.ch = i; box.append(b);
    } else if (c.src === 'slider') {
      const s = el('input', { type: 'range', min: -100, max: 100, value: Math.round(slider[i] * 100), class: 'sl',
        oninput: e => { slider[i] = e.target.value / 100; } });
      const w = el('label', { class: 'slw' }, el('span', {}, `${i + 1} ${c.name}`), s); box.append(w);
    }
  });
  refreshAux();
}
function refreshAux() {
  $$('#aux .sw').forEach(b => {
    const i = +b.dataset.ch, c = cfg.channels[i];
    const pos = c.src === 'sw2' ? ['OFF', 'ON'][sw[i]] : ['LOW', 'MID', 'HIGH'][sw[i]];
    b.textContent = `${i + 1} ${c.name}\n${pos}`;
    b.classList.toggle('on', sw[i] > 0);
    b.classList.toggle('arm', i === cfg.safety.armCh && sw[i] === 1);
  });
  $('#armState').textContent = isArmed() ? 'ARMED' : 'SAFE';
  $('#armState').className = isArmed() ? 'pill bad' : 'pill';
}
function disarm(reason) {
  const a = cfg.safety.armCh; if (a >= 0 && sw[a]) { sw[a] = 0; refreshAux(); if (reason) toast(`Disarmed: ${reason}`, true); }
}

// =====================================================================
// Channel monitor
// =====================================================================
function buildMonitor() {
  const m = $('#monitor'); m.innerHTML = '';
  for (let i = 0; i < NCH; i++) m.append(el('div', { class: 'bar' }, el('i'), el('b', {}, `${i + 1}`)));
}
function drawMonitor() {
  $$('#monitor .bar').forEach((b, i) => {
    const pct = clamp((out[i] - 1000) / 10, 0, 100);
    b.firstChild.style.height = pct + '%'; b.title = `CH${i + 1} ${out[i]}`;
    b.classList.toggle('imu', imu.active && cfg.imu.enabled && cfg.imu.pairs.some(p => p.on && (p.chX === i || p.chY === i)));
  });
  $$('.chval').forEach(e => e.textContent = out[+e.dataset.ch]);
}

// =====================================================================
// IMU
// =====================================================================
let motionOn = false;
function screenAngle() { return (screen.orientation && screen.orientation.angle) || window.orientation || 0; }
function onMotion(e) {
  const g = e.accelerationIncludingGravity; if (!g || g.x == null) return;
  imu.seen = true;
  const a = screenAngle() * Math.PI / 180, c = Math.cos(a), s = Math.sin(a);
  // remap device axes -> screen axes (device frame is fixed to the hardware)
  const sx = g.x * c - g.y * s, sy = g.x * s + g.y * c, sz = g.z;
  const lr = Math.atan2(sx, Math.hypot(sy, sz)) * 57.2958;
  const fb = Math.atan2(sy, sz) * 57.2958;
  const r = e.rotationRate || {};
  const rx = (r.beta || 0) * c - (r.gamma || 0) * s, ry = (r.beta || 0) * s + (r.gamma || 0) * c, rz = r.alpha || 0;
  const k = 1 - clamp(cfg.imu.smoothing, 0, 0.95);
  imu.lr += k * (lr - imu.lr);
  imu.fb += k * wrap180(fb - imu.fb); imu.fb = wrap180(imu.fb);
  imu.rx += k * (rx - imu.rx); imu.ry += k * (ry - imu.ry); imu.rz += k * (rz - imu.rz);
}
async function enableMotion() {
  if (motionOn) return true;
  try {
    if (typeof DeviceMotionEvent !== 'undefined' && DeviceMotionEvent.requestPermission) {
      if (await DeviceMotionEvent.requestPermission() !== 'granted') { toast('Motion permission denied', true); return false; }
    }
    window.addEventListener('devicemotion', onMotion); motionOn = true; return true;
  } catch (e) { toast('Motion sensors unavailable: ' + e.message, true); return false; }
}
function zeroImu() { imu.zLR = imu.lr; imu.zFB = imu.fb; }
function setImuActive(on) {
  if (on && !cfg.imu.enabled) return;
  if (on && !imu.active && cfg.imu.zeroOnEngage) zeroImu();
  imu.active = on;
  $('#imuBtn').classList.toggle('on', on);
  $('#imuBtn').textContent = cfg.imu.engage === 'hold' ? (on ? 'IMU ●' : 'HOLD IMU') : (on ? 'IMU ON' : 'IMU OFF');
}
function setupImuButton() {
  const b = $('#imuBtn');
  b.addEventListener('pointerdown', async e => {
    e.preventDefault(); if (!cfg.imu.enabled) { toast('Enable IMU control in the IMU tab', true); return; }
    await enableMotion();
    if (cfg.imu.engage === 'hold') setImuActive(true); else setImuActive(!imu.active);
  });
  const up = () => { if (cfg.imu.engage === 'hold') setImuActive(false); };
  b.addEventListener('pointerup', up); b.addEventListener('pointercancel', up); b.addEventListener('pointerleave', up);
}

// =====================================================================
// Link: connect / send / receive
// =====================================================================
async function connect() {
  try {
    port = await connectPort(cfg.link.baud, cfg.link.prefer);
    attachPort();
  } catch (e) { log('Connect failed: ' + e.message); toast(e.message, true); port = null; }
}
async function tryAutoReconnect() {
  try { const p = await reconnectKnown(cfg.link.baud, cfg.link.prefer); if (p) { port = p; attachPort(); } } catch (e) { log('Auto-reconnect: ' + e.message); }
}
function attachPort() {
  parser = new CrsfParser(onFrame);
  port.onData(d => parser.push(d));
  port.onDisconnect = () => { log('Device disconnected'); dropPort(); };
  log(`Connected via ${port.kind} — ${port.name || ''} @ ${cfg.link.baud}`);
  toast('Connected');
  disarm();
  startSending();
  setTimeout(() => sendCtl(bridgeGetFrame()), 300);
  keepAwake();
  updateConn();
}
function dropPort() { stopSending(); try { port?.close(); } catch {} port = null; disarm(); updateConn(); }
async function disconnect() { dropPort(); log('Disconnected'); }

function sendRaw(bytes) {
  if (!port || txBusy) return false;
  txBusy = true;
  Promise.resolve(port.write(bytes)).catch(e => { log('Write error: ' + e.message); }).finally(() => { txBusy = false; });
  return true;
}
const ctlQueue = [];
function sendCtl(bytes) { ctlQueue.push(bytes); if (!sendTimer) sendRaw(ctlQueue.shift()); }
function startSending() {
  stopSending();
  sendTimer = setInterval(tick, Math.round(1000 / cfg.link.rateHz));
}
function stopSending() { clearInterval(sendTimer); sendTimer = null; }
function tick() {
  if (!port || document.hidden) return;
  computeChannels();
  if (ctlQueue.length) { if (sendRaw(ctlQueue[0])) ctlQueue.shift(); return; }
  let frame;
  if (cfg.link.proto === 'mavlink') {
    const now = performance.now();
    if (now - lastHb > 1000) { lastHb = now; sendRaw(mavHeartbeat(seq++, cfg.link)); return; }
    const ch18 = out.slice(0, 16).concat([0, 0]);   // ch17-18: 0 = "ignore"
    frame = mavRcOverride(ch18, seq++, cfg.link);
  } else frame = crsfRcFrame(out);
  if (sendRaw(frame)) txCount++;
}

function onFrame(type, payload) {
  telem.lastRx = performance.now();
  const t = decodeTelemetry(type, payload); if (!t) return;
  if (t.kind === 'link') telem.link = t;
  else if (t.kind === 'battery') telem.battery = t;
  else if (t.kind === 'fmode') telem.fmode = t.text;
  else if (t.kind === 'bridge') { telem.bridge = t; renderBridge(); }
  else if (t.kind === 'devinfo') { telem.devinfo = t; $('#devinfo').textContent = `${t.name}  fw ${t.sw}`; log(`Module: ${t.name} ${t.sw}`); }
}

function updateConn() {
  $('#connBtn').textContent = port ? 'Disconnect' : 'Connect';
  $('#connState').textContent = port ? `● ${cfg.link.proto.toUpperCase()}` : '○ offline';
  $('#connState').className = port ? 'pill ok' : 'pill';
}

async function applyBoard() {
  if (!port) { toast('Not connected', true); return; }
  sendCtl(bridgeSetFrame({ ...cfg.board, linkBaud: cfg.link.baud }));
  log(`Board config sent: module ${cfg.board.outBaud} baud, ${cfg.board.inverted ? 'inverted' : 'normal'}, ` +
      `period ${cfg.board.fixedPeriodUs || 'auto'}, failsafe ${cfg.board.failsafeMs} ms, link ${cfg.link.baud}`);
  setTimeout(() => sendCtl(bridgeGetFrame()), 400);
}
async function changeLinkBaud(newBaud) {
  const old = cfg.link.baud; cfg.link.baud = newBaud; saveCfg();
  if (!port) return;
  // tell the board first (at the old rate), then re-open the port at the new rate
  sendCtl(bridgeSetFrame({ ...cfg.board, linkBaud: newBaud }));
  setTimeout(async () => {
    try { stopSending(); await port.setBaud(newBaud); startSending(); log(`Link baud ${old} → ${newBaud}`); }
    catch (e) { log('Baud change failed: ' + e.message); }
  }, 250);
}

function renderBridge() {
  const b = telem.bridge; if (!b) return;
  $('#bridgeInfo').textContent =
    `${b.platform} fw${b.fw} · link ${b.linkProto}${b.linkOk ? '' : ' (FAILSAFE)'} @${b.linkBaud}\n` +
    `module ${b.outBaud} baud ${b.inverted ? 'inv' : 'norm'} · ${b.synced ? 'SYNCED' : 'free-run'} · period ${b.periodUs} µs (${(1e6 / b.periodUs).toFixed(0)} Hz)\n` +
    `in ${b.rxFps}/s · to module ${b.txFps}/s · telemetry ${b.telemFps}/s · failsafe ${b.failsafeMs} ms`;
}

// =====================================================================
// Status bar refresh
// =====================================================================
let lastTx = 0, lastTxT = performance.now();
function statusLoop() {
  computeChannels(); drawMonitor();
  const now = performance.now();
  if (now - lastTxT > 1000) { $('#txRate').textContent = `${Math.round((txCount - lastTx) * 1000 / (now - lastTxT))} fps`; lastTx = txCount; lastTxT = now; }
  const L = telem.link, fresh = now - telem.lastRx < 1500;
  $('#lq').textContent = L && fresh ? `LQ ${L.lq}%  ${L.ant ? L.rssi2 : L.rssi1} dBm  SNR ${L.snr}` : 'LQ —';
  $('#lq').className = 'pill ' + (L && fresh ? (L.lq > 70 ? 'ok' : L.lq > 30 ? 'warn' : 'bad') : '');
  $('#bat').textContent = telem.battery && fresh ? `${telem.battery.v.toFixed(1)} V` : '— V';
  $('#fmode').textContent = telem.fmode || '';
  if ($('#imuLive')) $('#imuLive').textContent = imu.seen
    ? `L/R ${(imu.lr - imu.zLR).toFixed(1)}°  F/B ${wrap180(imu.fb - imu.zFB).toFixed(1)}°  gyro ${imu.rx.toFixed(0)}/${imu.ry.toFixed(0)}/${imu.rz.toFixed(0)} °/s`
    : 'no motion data yet (tap "Start sensors")';
  requestAnimationFrame(statusLoop);
}

// =====================================================================
// Settings UI
// =====================================================================
const opt = (v, t, sel) => { const o = el('option', { value: v }, t); if (sel) o.selected = true; return o; };
const chOptions = (sel, allowNone = true) => {
  const a = allowNone ? [opt(-1, '— none —', sel === -1)] : [];
  return a.concat(cfg.channels.map((c, i) => opt(i, `CH${i + 1} ${c.name}`, i === sel)));
};
const num = (v, onchange, attrs = {}) => el('input', { type: 'number', value: v, inputmode: 'numeric', onchange: e => onchange(+e.target.value), ...attrs });

function buildChannels() {
  const t = $('#chTable'); t.innerHTML = '';
  t.append(el('tr', {}, ...['CH', 'Name', 'Source', 'Rev', 'Min', 'Sub', 'Max', 'Expo', 'µs'].map(h => el('th', {}, h))));
  cfg.channels.forEach((c, i) => {
    const changed = () => { saveCfg(); buildAux(); };
    t.append(el('tr', {},
      el('td', {}, `${i + 1}`),
      el('td', {}, el('input', { value: c.name, maxlength: 6, class: 'nm', onchange: e => { c.name = e.target.value; changed(); buildImu(); } })),
      el('td', {}, el('select', { onchange: e => { c.src = e.target.value; sw[i] = 0; changed(); } },
        ...Object.entries(SRC_LABEL).map(([k, v]) => opt(k, v, k === c.src)))),
      el('td', {}, el('input', { type: 'checkbox', checked: c.rev, onchange: e => { c.rev = e.target.checked; saveCfg(); } })),
      el('td', {}, num(c.min, v => { c.min = clamp(v, 885, 1500); saveCfg(); })),
      el('td', {}, num(c.sub, v => { c.sub = clamp(v, -200, 200); saveCfg(); })),
      el('td', {}, num(c.max, v => { c.max = clamp(v, 1500, 2115); saveCfg(); })),
      el('td', {}, num(c.expo, v => { c.expo = clamp(v, 0, 100); saveCfg(); })),
      el('td', { class: 'chval', 'data-ch': i }, '')));
  });
  $('#mapSel').value = cfg.map; $('#modeSel').value = cfg.stickMode;
  const a = $('#armSel'); a.innerHTML = ''; a.append(...chOptions(cfg.safety.armCh));
  $('#armThrLow').checked = cfg.safety.armThrLow;
}

function applyMap(m) {
  cfg.map = m;
  if (MAPS[m]) {
    // move stick sources to their new channel slots, keep everything else
    MAPS[m].forEach((s, i) => Object.assign(cfg.channels[i], { name: AXIS_NAME[s], src: s, expo: s === 'throttle' ? 0 : cfg.channels[i].expo || 20 }));
  }
  saveCfg(); buildChannels(); buildAux(); buildImu();
}

function buildImu() {
  const I = cfg.imu;
  $('#imuEn').checked = I.enabled; $('#imuEngage').value = I.engage; $('#imuZero').checked = I.zeroOnEngage;
  $('#imuSmooth').value = I.smoothing;
  const box = $('#pairs'); box.innerHTML = '';
  I.pairs.forEach((p, k) => {
    const s = () => saveCfg();
    const axisSel = (key) => el('select', { onchange: e => { p[key] = e.target.value; s(); } },
      ...Object.entries(IMU_AXES).map(([a, t]) => opt(a, t, a === p[key])));
    const side = (L, chK, axK, rgK, invK) => el('div', { class: 'pairside' },
      el('b', {}, L),
      el('label', {}, 'Channel ', el('select', { onchange: e => { p[chK] = +e.target.value; s(); } }, ...chOptions(p[chK]))),
      el('label', {}, 'Axis ', axisSel(axK)),
      el('label', {}, 'Full scale ', num(p[rgK], v => { p[rgK] = clamp(v, 1, 1000); s(); }, { class: 'sm' })),
      el('label', {}, el('input', { type: 'checkbox', checked: p[invK], onchange: e => { p[invK] = e.target.checked; s(); } }), ' invert'));
    box.append(el('fieldset', { class: 'pair' },
      el('legend', {}, el('label', {}, el('input', { type: 'checkbox', checked: p.on, onchange: e => { p.on = e.target.checked; s(); } }), ` Pair ${k + 1}`)),
      side('X', 'chX', 'axX', 'rangeX', 'invX'), side('Y', 'chY', 'axY', 'rangeY', 'invY'),
      el('label', {}, 'Deadband ', num(p.dead, v => { p.dead = clamp(v, 0, 45); s(); }, { class: 'sm' }), ' (° or °/s)')));
  });
}

function buildLink() {
  const L = cfg.link, B = cfg.board;
  $('#prefer').value = L.prefer; $('#proto').value = L.proto; $('#linkBaud').value = L.baud; $('#rate').value = L.rateHz;
  $('#tSys').value = L.targetSys; $('#tComp').value = L.targetComp;
  $('#outBaud').value = B.outBaud; $('#inv').checked = B.inverted; $('#period').value = B.fixedPeriodUs; $('#fsMs').value = B.failsafeMs;
  const s = transportSupport();
  $('#support').textContent = `Web Serial: ${s.serial ? 'yes' : 'no'} · WebUSB: ${s.usb ? 'yes' : 'no'} · ${isSecureContext ? 'secure context' : 'NOT SECURE (needs https)'}`;
}

function wireSettings() {
  $('#mapSel').onchange = e => applyMap(e.target.value);
  $('#modeSel').onchange = e => { cfg.stickMode = +e.target.value; saveCfg(); $('#stickL').redraw(); $('#stickR').redraw(); };
  $('#armSel').onchange = e => { disarm(); cfg.safety.armCh = +e.target.value; saveCfg(); refreshAux(); };
  $('#armThrLow').onchange = e => { cfg.safety.armThrLow = e.target.checked; saveCfg(); };
  $('#resetCh').onclick = () => { if (confirm('Reset all channels to defaults?')) { cfg.channels = defaultChannels(MAPS[cfg.map] ? cfg.map : 'AETR'); saveCfg(); buildChannels(); buildAux(); } };

  $('#imuEn').onchange = async e => { cfg.imu.enabled = e.target.checked; if (!cfg.imu.enabled) setImuActive(false); else await enableMotion(); saveCfg(); $('#imuBtn').hidden = !cfg.imu.enabled; setImuActive(false); };
  $('#imuEngage').onchange = e => { cfg.imu.engage = e.target.value; setImuActive(false); saveCfg(); };
  $('#imuZero').onchange = e => { cfg.imu.zeroOnEngage = e.target.checked; saveCfg(); };
  $('#imuSmooth').oninput = e => { cfg.imu.smoothing = +e.target.value; saveCfg(); };
  $('#imuCal').onclick = () => { zeroImu(); toast('IMU neutral set'); };
  $('#imuStart').onclick = () => enableMotion().then(ok => ok && toast('Sensors running'));

  $('#prefer').onchange = e => { cfg.link.prefer = e.target.value; saveCfg(); };
  $('#proto').onchange = e => { cfg.link.proto = e.target.value; saveCfg(); updateConn(); log(`Link protocol: ${cfg.link.proto}`); };
  $('#linkBaud').onchange = e => changeLinkBaud(+e.target.value);
  $('#rate').onchange = e => { cfg.link.rateHz = +e.target.value; saveCfg(); if (port) startSending(); };
  $('#tSys').onchange = e => { cfg.link.targetSys = +e.target.value; saveCfg(); };
  $('#tComp').onchange = e => { cfg.link.targetComp = +e.target.value; saveCfg(); };
  $('#outBaud').onchange = e => { cfg.board.outBaud = +e.target.value; saveCfg(); };
  $('#inv').onchange = e => { cfg.board.inverted = e.target.checked; saveCfg(); };
  $('#period').onchange = e => { cfg.board.fixedPeriodUs = +e.target.value; saveCfg(); };
  $('#fsMs').onchange = e => { cfg.board.failsafeMs = clamp(+e.target.value, 100, 5000); saveCfg(); };
  $('#applyBoard').onclick = applyBoard;
  $('#readBoard').onclick = () => port ? sendCtl(bridgeGetFrame()) : toast('Not connected', true);
  $('#ping').onclick = () => port ? (sendCtl(crsfPingFrame()), log('Ping sent to module')) : toast('Not connected', true);

  $('#export').onclick = () => {
    const a = el('a', { href: URL.createObjectURL(new Blob([JSON.stringify(cfg, null, 2)], { type: 'application/json' })), download: 'elrs-phone-tx-config.json' });
    a.click();
  };
  $('#import').onchange = async e => {
    try { const c = JSON.parse(await e.target.files[0].text()); cfg = deepMerge(DEFAULT_CFG(), c); saveCfg(); rebuildAll(); toast('Config imported'); }
    catch (err) { toast('Bad config file', true); }
  };
  $('#factory').onclick = () => { if (confirm('Reset ALL settings?')) { cfg = DEFAULT_CFG(); saveCfg(); rebuildAll(); } };
}

function rebuildAll() { buildChannels(); buildAux(); buildImu(); buildLink(); $('#imuBtn').hidden = !cfg.imu.enabled; setImuActive(false); }

// =====================================================================
// Misc: tabs, wake lock, fullscreen, lifecycle
// =====================================================================
let wakeLock = null;
async function keepAwake() { try { wakeLock = await navigator.wakeLock?.request('screen'); } catch {} }
document.addEventListener('visibilitychange', () => {
  if (document.hidden) { disarm('app hidden'); setImuActive(false); }
  else if (port) keepAwake();
});

function wireTabs() {
  $$('nav button').forEach(b => b.onclick = () => {
    $$('nav button').forEach(x => x.classList.toggle('sel', x === b));
    $$('main > section').forEach(s => s.hidden = s.id !== b.dataset.tab);
  });
}

async function fullscreen() {
  try { await document.documentElement.requestFullscreen(); await screen.orientation.lock?.('landscape'); } catch {}
}

function init() {
  wireTabs(); wireSettings();
  setupStick('#stickL'); setupStick('#stickR');
  setupImuButton(); buildMonitor(); rebuildAll();
  $('#connBtn').onclick = () => port ? disconnect() : connect();
  $('#connBtn2').onclick = () => port ? disconnect() : connect();
  $('#fsBtn').onclick = fullscreen;
  $('#killBtn').onclick = () => { disarm('kill switch'); input.throttle = -1; $('#stickL').redraw(); $('#stickR').redraw(); };
  navigator.usb?.addEventListener?.('connect', () => { if (!port) tryAutoReconnect(); });
  navigator.serial?.addEventListener?.('connect', () => { if (!port) tryAutoReconnect(); });
  tryAutoReconnect();
  if ('serviceWorker' in navigator) navigator.serviceWorker.register('./sw.js').catch(() => {});
  requestAnimationFrame(statusLoop);
}
init();
