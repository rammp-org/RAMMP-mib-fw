// panel.js: the RAMMP MIB control panel UI. Transport and protocol are in mib_link.js.
'use strict';

const $ = id => document.getElementById(id);
const sleep = ms => new Promise(r => setTimeout(r, ms));
const css = v => getComputedStyle(document.documentElement).getPropertyValue('--ax-' + v).trim();
const SERIES = () => [1, 2, 3, 4, 5, 6].map(i => css('s' + i));
const fmt = (v, d = 2) => (v === null || v === undefined || !Number.isFinite(v)) ? '–' : v.toFixed(d);
const mean = a => a.length ? a.reduce((s, x) => s + x, 0) / a.length : NaN;
const linfit = (x, y) => { const n = x.length, mx = mean(x), my = mean(y); let sxy = 0, sxx = 0; for (let i = 0; i < n; i++) { sxy += (x[i] - mx) * (y[i] - my); sxx += (x[i] - mx) ** 2; } const m = sxy / sxx; return { m, b: my - m * mx }; };

let link = null;          // MibLink while connected
let status = null;        // last MIB status JSON
let info = null;          // system info JSON
let pollTimer = null;

// ---------------------------------------------------------------- toasts and log
function toast(text, cls = '') {
  const d = document.createElement('div'); d.className = cls; d.textContent = text; $('toast').appendChild(d);
  setTimeout(() => d.remove(), cls === 'err' ? 6000 : 3500);
}
const logState = { lines: [], partial: '', dirty: false };
function logText(text, cls) {
  if (cls) { logState.lines.push({ text: text.replace(/\n$/, ''), cls }); }
  else {
    const parts = (logState.partial + text.replace(/\r/g, '')).split('\n'); logState.partial = parts.pop();
    for (const l of parts) logState.lines.push({ text: l, cls: /^E \(|\bERROR\b|^E /.test(l) ? 'err' : /^W \(|^W /.test(l) ? 'warn' : '' });
  }
  if (logState.lines.length > 4000) logState.lines.splice(0, logState.lines.length - 4000);
  logState.dirty = true;
}
function sys(text) { logText(`[panel ${new Date().toTimeString().slice(0, 8)}] ${text}`, 'sys'); }
setInterval(() => {
  if (!logState.dirty || $('logPause').checked) return; logState.dirty = false;
  const f = $('logFilter').value.trim().toLowerCase(), o = $('logOut');
  const atBottom = o.scrollTop + o.clientHeight >= o.scrollHeight - 30;
  const lines = (f ? logState.lines.filter(l => l.text.toLowerCase().includes(f)) : logState.lines).slice(-1500);
  o.innerHTML = ''; const frag = document.createDocumentFragment();
  for (const l of lines) { const s = document.createElement('span'); if (l.cls) s.className = l.cls; s.textContent = l.text + '\n'; frag.appendChild(s); }
  o.appendChild(frag); if (atBottom) o.scrollTop = o.scrollHeight;
}, 200);
$('logFilter').oninput = () => { logState.dirty = true; };
$('logClear').onclick = () => { logState.lines = []; logState.dirty = true; };
$('logSave').onclick = () => download(`mib-log-${stamp()}.txt`, logState.lines.map(l => l.text).join('\n'));
const stamp = () => new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-');
function download(name, text) { const a = document.createElement('a'); a.href = URL.createObjectURL(new Blob([text])); a.download = name; a.click(); setTimeout(() => URL.revokeObjectURL(a.href), 1000); }

// ---------------------------------------------------------------- charts (uPlot, alex-style)
const charts = [];
function axisStyle(label, size) {
  return { label, labelFont: '500 11px Roboto, sans-serif', labelSize: 18, font: "11px 'Roboto Mono', monospace", stroke: css('muted'), size,
    grid: { stroke: css('grid'), dash: [2, 4], width: 1 }, ticks: { show: false } };
}
function makePlot(el, series, yLabel, { xLabel = 'Time before now, s', height = 200, xDigits = 2 } = {}) {
  el.innerHTML = '';
  const opts = {
    width: Math.max(el.clientWidth, 280), height, legend: { live: true }, cursor: { drag: { x: false, y: false }, points: { size: 6 } },
    scales: { x: { time: false } },
    axes: [axisStyle(xLabel), axisStyle(yLabel, 58)],
    series: [{ label: xLabel.split(',')[0], value: (u, v) => v == null ? '–' : v.toFixed(xDigits) },
      ...series.map(s => ({ label: s.label, stroke: s.color, width: 2.5, dash: s.dash, spanGaps: s.spanGaps ?? false, points: { show: false }, value: (u, v) => v == null ? '–' : v.toFixed(s.digits ?? 2) }))],
  };
  const u = new uPlot(opts, [[], ...series.map(() => [])], el);
  const lg = u.root.querySelector('.u-legend'); if (lg) u.root.insertBefore(lg, u.root.firstChild); // legend above the plot
  new ResizeObserver(() => { if (el.clientWidth > 0 && Math.abs(el.clientWidth - u.width) > 2) u.setSize({ width: el.clientWidth, height }); }).observe(el);
  return u;
}
// A live chart over telemetry channels. `series` items: { ch, label, color, dash }; ch may be a function of the axis.
class LiveChart {
  constructor(el, series, yLabel, seconds = 10) { this.el = el; this.series = series; this.yLabel = yLabel; this.seconds = seconds; this.u = null; charts.push(this); }
  rebuild(series) { this.series = series || this.series; this.u && this.u.destroy(); this.u = null; }
  update(store) {
    if (!this.el.offsetParent) return;
    if (!this.u) this.u = makePlot(this.el, this.series, this.yLabel);
    if (!this.series.length) return;
    const chs = this.series.map(s => typeof s.ch === 'function' ? s.ch() : s.ch);
    this.u.setData(store.window(chs, this.seconds));
  }
}
function drawStatic(el, traces, yLabel, xLabel, existing) {
  // traces: [{ x:[], y:[], label, color, dash }]; resampled onto one x grid for uPlot
  if (existing) existing.destroy();
  if (!traces.length) { el.innerHTML = ''; return null; }
  const xs = [...new Set(traces.flatMap(t => t.x.map(v => Math.round(v * 100) / 100)))].sort((a, b) => a - b);
  const data = [xs, ...traces.map(t => { const m = new Map(t.x.map((x, i) => [Math.round(x * 100) / 100, t.y[i]])); return xs.map(x => m.has(x) ? m.get(x) : null); })];
  const u = makePlot(el, traces.map(t => ({ ...t, spanGaps: true })), yLabel, { xLabel, height: 220 });
  u.setData(data); return u;
}
function chartLoop() {
  if (link && link.store.count) for (const c of charts) c.update(link.store);
  setTimeout(() => requestAnimationFrame(chartLoop), 66);
}

// ---------------------------------------------------------------- tabs
$('tabs').onclick = e => {
  const b = e.target.closest('button'); if (!b) return;
  for (const x of $('tabs').children) x.classList.toggle('on', x === b);
  for (const s of document.querySelectorAll('.tab')) s.classList.toggle('on', s.id === 'tab-' + b.dataset.tab);
};

// ---------------------------------------------------------------- connection
function setConnected(on, label) {
  $('mLink').className = 'ax-status ' + (on ? 'live' : (label === 'Link lost' ? 'critical' : ''));
  $('mLink').textContent = label || (on ? 'Live' : 'Disconnected');
  $('connectBtn').textContent = on ? 'Disconnect' : 'Connect';
  for (const id of ['resetBtn', 'estopBtn', 'clearBtn', 'tuApply', 'tuLoad', 'tdHold', 'tdRev', 'stRun', 'atStart', 'drEnable', 'drDisable', 'csvBtn']) $(id).disabled = !on;
  if (typeof fwRefresh === 'function') fwRefresh();
  $('transport').disabled = on; $('ethTarget').disabled = on; $('ethBridge').disabled = on;
  document.querySelectorAll('#seatGrid button').forEach(b => { b.disabled = !on; });
}
$('connectBtn').onclick = async () => {
  if (link) return disconnect();
  const kind = $('transport').value;
  localStorage.setItem('mib.transport', kind); localStorage.setItem('mib.ethTarget', $('ethTarget').value.trim()); localStorage.setItem('mib.ethBridge', $('ethBridge').value.trim());
  axBusy($('connectBtn'), true, 'Connecting');
  try { await openLink(makeTransport(kind)); }
  catch (e) { if (e.name !== 'NotFoundError') toast('Connect failed: ' + e.message, 'err'); sys('connect failed: ' + e.message); }
  finally { axBusy($('connectBtn'), false); $('connectBtn').textContent = link ? 'Disconnect' : 'Connect'; }
};
// `previous`: the transport of a link that just dropped (a reboot), to reach the same MIB again.
function makeTransport(kind, previous) {
  if (kind === 'eth') return new WsTransport($('ethBridge').value.trim(), $('ethTarget').value.trim());
  if (kind === 'serial') return new SerialTransport(previous ? previous.lastPort : undefined);
  if (kind === 'usb') return new UsbTransport(previous ? previous.lastDevice : undefined);
  return new SimTransport();
}
async function openLink(t) {
  try {
    const l = new MibLink(t, {
      onLog: (text, cls) => logText(text, cls),
      onSchema: names => { sys(`telemetry: ${names.length} channels`); buildChannelList(names); },
      onError: (type, why) => onMibError(type, why),
      onClose: lost => { if (link === l) { link = null; stopAllOverrides(false); clearInterval(pollTimer); setConnected(false, lost ? 'Link lost' : 'Disconnected'); if (lost && !fw.rebooting) toast('Link to the MIB lost', 'err'); } },
    });
    await l.connect(); link = l;
    setConnected(true); sys(`connected: ${t.name()}`); toast('Connected: ' + t.name(), 'ok');
    try { info = await l.info(); $('mFw').textContent = `fw ${info.version} (${info.built})`; sys(`MIB ${info.project} ${info.version}, reset reason ${info.reset_reason}, up ${info.uptime_s} s`); } catch (e) { sys('no info reply: ' + e.message); }
    startPolling(t.kind === 'serial' && !l.fast ? 500 : 250);
    renderFwInfo();
  } catch (e) {
    try { await t.close(); } catch (_) {}
    link = null; setConnected(false); throw e;
  }
}
async function disconnect() { stopAllOverrides(true); await sleep(60); clearInterval(pollTimer); const l = link; link = null; if (l) await l.close(); setConnected(false); }
$('resetBtn').onclick = async () => {
  if (!link || !confirm('Reboot the MIB? Every motor disarms while it restarts.')) return;
  // Over serial the auto-reset circuit does it; the link drops back to 115200 and renegotiates once the MIB is up.
  try { if (link.t.kind === 'serial') { await link.t.hardReset(); } else { await link.reset(); } sys('reset sent'); } catch (e) { toast('Reset: ' + e.message, 'err'); }
};
// Chrome lists an already-paired MIB without a chooser; reconnect when it is plugged back in.
if (navigator.usb) navigator.usb.addEventListener('connect', e => { if (!link && $('transport').value === 'usb') sys(`USB device plugged in: ${e.device.productName}. Press Connect.`); });

function startPolling(ms) {
  clearInterval(pollTimer);
  let inflight = false;
  pollTimer = setInterval(async () => {
    if (!link || inflight || fw.busy) return; inflight = true;
    try { status = await link.status(); renderStatus(); } catch (e) { /* a missed poll is shown by the age of the data */ }
    inflight = false;
  }, ms);
}
let lastErrToast = 0;
function onMibError(type, why) {
  if (type === MIBP.OVERRIDE || type === MIBP.STICK) {
    if (overrides.size || padState.down) { stopAllOverrides(false); padRelease(); }
    if (performance.now() - lastErrToast > 1500) { toast('MIB refused: ' + why, 'err'); lastErrToast = performance.now(); }
    if (autotune.running) autotune.abortReason = 'MIB refused the command: ' + why;
    if (stepTest.running) stepTest.abortReason = 'MIB refused the command: ' + why;
  } else toast('MIB refused: ' + why, 'err');
  sys(`MIB refused request 0x${type.toString(16)}: ${why}`);
}

// ---------------------------------------------------------------- status rendering
const stateCls = s => ({ IDLE: 'good', DRIVE_ENABLED: 'live', AUTONOMOUS: 'live', SHARED: 'live', INIT: 'warning', CALIBRATE: 'warning', ESTOP: 'critical', ERROR: 'critical' }[s] || '');
const stateLabel = { DRIVE_ENABLED: 'Driving', IDLE: 'Ready', AUTONOMOUS: 'Jetson driving', SHARED: 'Shared control', CALIBRATE: 'Calibrating', ESTOP: 'E-stop', ERROR: 'Error' };
function renderStatus() {
  const s = status; if (!s) return;
  $('tState').textContent = s.state.replace('_ENABLED', '');
  $('tStateSub').innerHTML = `<span class="ax-status ${stateCls(s.state)}">${stateLabel[s.state] || 'Starting'}</span>${s.panel_stick ? ' <span class="ax-muted">panel stick</span>' : ''}`;
  $('tEstop').textContent = s.estop.latched ? 'LATCHED' : 'CLEAR';
  $('tEstop').style.color = s.estop.latched ? css('critical') : '';
  $('tEstopSub').innerHTML = s.estop.latched ? `<span class="ax-status critical">Cause: ${s.estop.cause}</span>${s.estop.clearing ? ' <span class="ax-spin"></span> clearing' : ''}` : `<span class="ax-status good">Clear</span> <span class="ax-muted">${s.estop.releases} releases</span>`;
  $('tProfile').textContent = PROFILES[s.profile] || '–'; $('tProfileSub').textContent = 'active drive response';
  $('tNet').textContent = s.ip; $('mIp').textContent = 'ip ' + s.ip;
  $('tNetSub').innerHTML = s.ethernet ? (s.rtps ? '<span class="ax-status good">Ethernet + RTPS up</span>' : '<span class="ax-status warning">Link, RTPS starting</span>') : '<span class="ax-status critical">No Ethernet link</span>';
  const online = s.axes.filter(a => a.online);
  $('tMotors').textContent = `${online.length}/${s.axes.length}`;
  const faults = s.axes.filter(a => a.online && a.state === 'FAULT');
  $('tMotorsSub').innerHTML = faults.length ? `<span class="ax-status critical">${faults.map(a => a.label + ' ' + a.fault).join(', ')}</span>` : online.length ? `<span class="ax-status good">${online.map(a => a.label).join(', ')}</span>` : '<span class="ax-status warning">None reporting</span>';
  renderAxesTable(s); renderTuneInForce(s); renderSeatReported(s);
  $('drEnable').disabled = !link || !(s.state === 'IDLE' || s.state === 'DRIVE_ENABLED');
  $('drDisable').disabled = !link || !['DRIVE_ENABLED', 'AUTONOMOUS', 'SHARED'].includes(s.state);
  $('padHint').textContent = s.state === 'DRIVE_ENABLED' ? 'Hold and drag on the pad. Up is forward, right is a right turn. Let go to stop.' : s.state === 'AUTONOMOUS' ? 'The Jetson is driving. Touch the pad to take control back (drive then needs re-enabling).' : 'Enable drive, then hold and drag on the pad.';
}
function renderAxesTable(s) {
  const tb = $('axesTable').querySelector('tbody');
  if (tb.children.length !== s.axes.length || !tb.querySelector('[data-ax]')) {
    tb.innerHTML = s.axes.map(a => `<tr data-ax="${a.axis}"><td>${a.label} <span class="ax-muted">${a.axis}</span></td>${'<td></td>'.repeat(12)}<td><button class="ax-btn" data-clear="${a.axis}">Clear fault</button></td></tr>`).join('');
    tb.querySelectorAll('[data-clear]').forEach(b => b.onclick = async () => { try { await link.clearFault(+b.dataset.clear); toast('Clear-fault sent', 'ok'); } catch (e) { toast(e.message, 'err'); } });
  }
  for (const a of s.axes) {
    const tr = tb.querySelector(`[data-ax="${a.axis}"]`), td = tr.children, st = link ? link.store : null, seg = a.segment;
    const g = n => st ? st.get(`${seg}.${n}`) : NaN;
    td[1].innerHTML = a.online ? `<span class="ax-status good">${a.age_ms} ms</span>` : `<span class="ax-status ${a.age_ms < 0 ? '' : 'warning'}">${a.age_ms < 0 ? 'never seen' : 'stale'}</span>`;
    td[2].innerHTML = a.online ? `<span class="ax-status ${a.state === 'FAULT' ? 'critical' : a.state === 'ARMED' ? 'live' : ''}">${a.state}</span>${a.override ? ' <span class="ax-muted">panel</span>' : ''}` : '<span class="pill-off">–</span>';
    td[3].textContent = a.online ? a.fault : '–';
    const cells = [[g('vel'), 2], [a.cmd.vel, 2], [g('iq'), 2], [a.online ? a.vbus : NaN, 1], [a.online ? a.temp_max : NaN, 1], [a.online ? a.tuning.kp : NaN, 4], [a.online ? a.tuning.ki : NaN, 4], [a.online ? a.tuning.pll : NaN, 1]];
    cells.forEach(([v, d], i) => { td[4 + i].className = 'num'; td[4 + i].textContent = fmt(v, d); });
    td[12].textContent = a.online ? `${a.fw} · API ${a.api}` : '–'; td[12].className = 'ax-muted';
  }
}

// ---------------------------------------------------------------- e-stop
$('estopBtn').onclick = async () => { try { await link.estop(true); toast('E-stop engaged', 'err'); } catch (e) { toast('E-stop: ' + e.message, 'err'); } stopAllOverrides(false); };
document.addEventListener('keydown', e => { if (e.key === 'Escape' && link) $('estopBtn').click(); });
holdButton($('clearBtn'), 2000, async () => { try { await link.estop(false); toast('Clear attempt started', 'ok'); } catch (e) { toast(e.message, 'err'); } });
function holdButton(btn, ms, fn) {
  let timer = null; const label = btn.textContent;
  const down = e => { if (btn.disabled) return; e.preventDefault(); btn.classList.add('active'); const t0 = performance.now();
    timer = setInterval(() => { const left = ms - (performance.now() - t0); if (left <= 0) { up(); fn(); } else btn.textContent = `Hold ${(left / 1000).toFixed(1)} s`; }, 50); };
  const up = () => { clearInterval(timer); timer = null; btn.classList.remove('active'); btn.textContent = label; };
  btn.addEventListener('pointerdown', down); ['pointerup', 'pointerleave', 'pointercancel'].forEach(ev => btn.addEventListener(ev, up));
}

// ---------------------------------------------------------------- motor overrides (deadman streams)
const overrides = new Map(); let ovTimer = null;
function setOverride(axis, cmd) {
  overrides.set(axis, cmd);
  if (!ovTimer) ovTimer = setInterval(sendOverrides, 50);
  sendOverrides();
}
function sendOverrides() { if (!link) return; for (const [axis, cmd] of overrides) link.override(axis, cmd).catch(() => {}); }
function clearOverride(axis) {
  overrides.delete(axis); if (!overrides.size) { clearInterval(ovTimer); ovTimer = null; }
  if (link) link.release(axis).catch(() => {});
}
function stopAllOverrides(release = true) {
  overrides.clear(); clearInterval(ovTimer); ovTimer = null;
  if (release && link) link.release(MIBP.ALL).catch(() => {});
}
// Anything that takes the page away from the operator stops the motors.
document.addEventListener('visibilitychange', () => { if (document.hidden) { stopAllOverrides(true); padRelease(); if (autotune.running) autotune.abortReason = 'page hidden'; if (stepTest.running) stepTest.abortReason = 'page hidden'; } });
window.addEventListener('blur', () => { if (padState.down) padRelease(); });

// ---------------------------------------------------------------- motor tuning tab
for (const sel of [$('tuAxis')]) sel.innerHTML = AXES.map(a => `<option value="${a.id}" ${a.id === 2 ? 'selected' : ''}>${a.label}</option>`).join('');
const tuAxis = () => +$('tuAxis').value, tuSeg = () => AXES[tuAxis()].segment;
function renderTuneInForce(s) {
  const a = s.axes[tuAxis()]; if (!a) return;
  const pending = a.tuning.req && a.tuning.ack !== a.tuning.req;
  $('tuInForce').innerHTML = a.online ? `In force on ${a.label}: k<sub>p</sub> <b>${fmt(a.tuning.kp, 4)}</b>, k<sub>i</sub> <b>${fmt(a.tuning.ki, 4)}</b>, PLL <b>${fmt(a.tuning.pll, 1)}</b> Hz · ${pending ? '<span class="ax-status warning">waiting for the board to echo req ' + a.tuning.req + '</span>' : '<span class="ax-status good">ack ' + a.tuning.ack + '</span>'}${a.api < 3 ? ' · <span class="ax-status warning">API ' + a.api + ': needs v3 for zero gains</span>' : ''}`
    : `${a.label} is not reporting.`;
  if (!$('tuKp').value && a.online) loadInForce();
}
function loadInForce() { const a = status && status.axes[tuAxis()]; if (!a || !a.online) return; $('tuKp').value = a.tuning.kp.toFixed(4); $('tuKi').value = a.tuning.ki.toFixed(4); $('tuPll').value = a.tuning.pll.toFixed(1); }
$('tuLoad').onclick = loadInForce;
$('tuAxis').onchange = () => { $('tuKp').value = ''; loadInForce(); tuneCharts.forEach(c => c.rebuild()); };
async function applyGains(axis, kp, ki, pll, persist) {
  const req = await link.tune(axis, kp, ki, pll, persist);
  const ok = await waitFor(s => s.axes[axis].tuning.ack === req, 3000);
  if (!ok) throw new Error(`board did not echo tuning req ${req}`);
  return req;
}
$('tuApply').onclick = async () => {
  const kp = parseFloat($('tuKp').value) || 0, ki = parseFloat($('tuKi').value) || 0, pll = parseFloat($('tuPll').value) || 0;
  axBusy($('tuApply'), true, 'Applying');
  try { await applyGains(tuAxis(), kp, ki, pll, $('tuPersist').checked); toast(`Gains applied to ${AXES[tuAxis()].label}${$('tuPersist').checked ? ' and saved' : ''}`, 'ok'); sys(`tuning ${tuSeg()}: kp ${kp} ki ${ki} pll ${pll}${$('tuPersist').checked ? ' (persist)' : ''}`); }
  catch (e) { toast('Tuning: ' + e.message, 'err'); } finally { axBusy($('tuApply'), false); }
};
async function waitFor(pred, ms) { const t0 = performance.now(); while (performance.now() - t0 < ms) { if (status && pred(status)) return true; await sleep(100); } return false; }

const tdUnits = { 2: 'rad/s', 1: 'N·m', 3: 'rad', 4: '–' };
$('tdMode').onchange = () => { $('tdUnit').textContent = tdUnits[$('tdMode').value]; };
function testCommand(sign = 1) {
  const mode = +$('tdMode').value, sp = (parseFloat($('tdSet').value) || 0) * sign;
  const startPos = link ? link.store.get(`${tuSeg()}.pos`) : 0;
  return { state: REQ_STATE.ARMED, mode, velocity: mode === MODE.VELOCITY ? sp : 0, torque: mode === MODE.TORQUE ? sp : 0,
    position: mode === MODE.POSITION ? (Number.isFinite(startPos) ? startPos : 0) + sp : 0,
    torqueLimit: parseFloat($('tdTl').value) || 0, velLimit: parseFloat($('tdVl').value) || 0, accelLimit: parseFloat($('tdAl').value) || 0 };
}
for (const [id, sign] of [['tdHold', 1], ['tdRev', -1]]) {
  const b = $(id);
  b.addEventListener('pointerdown', e => { if (b.disabled) return; e.preventDefault(); b.setPointerCapture(e.pointerId); b.classList.add('active'); setOverride(tuAxis(), testCommand(sign)); });
  const up = () => { if (!b.classList.contains('active')) return; b.classList.remove('active'); clearOverride(tuAxis()); };
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(ev => b.addEventListener(ev, up));
}

// Step test: 0 for 0.5 s, the setpoint for the chosen time, then 0 for 1.5 s.
const stepTest = { running: false, runs: [], plot: null, abortReason: null };
function stepMetrics(rows, onset, target, dur) {
  const t = [], n = [];
  for (const r of rows) { if (r[0] < onset - 0.3 || r[0] > onset + dur) continue; if (!Number.isFinite(r[1])) continue; t.push(r[0] - onset); n.push(r[1] / target); }
  const after = t.map((x, i) => [x, n[i]]).filter(p => p[0] >= 0);
  const first = th => { const p = after.find(q => q[1] >= th); return p ? p[0] : NaN; };
  const peak = Math.max(...after.map(p => p[1]));
  let settle = 0; for (const p of after) if (Math.abs(p[1] - 1) > 0.05) settle = p[0];
  const tail = after.filter(p => p[0] >= dur * 0.75).map(p => p[1]);
  let iae = 0; for (let i = 1; i < after.length; i++) iae += Math.abs(1 - after[i][1]) * (after[i][0] - after[i - 1][0]);
  return { t, y: n.map(v => v * target), rise: first(0.9) - first(0.1), overshoot: Math.max(0, (peak - 1) * 100), settle, ssErr: (1 - mean(tail)) * 100, iae };
}
// Runs one speed step on `axis` through the override stream and returns the metrics.
async function runStep(axis, target, dur, guard, { accel = 0, vl = 0, tl = 0 } = {}) {
  const seg = AXES[axis].segment, cmd = v => ({ state: REQ_STATE.ARMED, mode: MODE.VELOCITY, velocity: v, torqueLimit: tl, velLimit: vl, accelLimit: v === 0 ? 6 : accel });
  setOverride(axis, cmd(0)); await guard(500);
  const tMark = link.store.now();
  setOverride(axis, cmd(target)); await guard(dur * 1000);
  setOverride(axis, cmd(0)); await guard(1200);
  const rows = link.store.since(tMark - 0.5, [`${seg}.vel`, `${seg}.cmd_vel`]);
  const on = rows.find(r => r[0] >= tMark && Math.abs(r[2] - target) < 1e-3);
  if (!on) throw new Error('the command never reached the telemetry');
  return stepMetrics(rows.map(r => [r[0], r[1]]), on[0], target, dur);
}
function guardFor(state) {
  return async ms => { const t0 = performance.now(); while (performance.now() - t0 < ms) { if (state.abortReason) throw new Error(state.abortReason); if (!link) throw new Error('link lost'); await sleep(20); } };
}
$('stRun').onclick = async () => {
  if (stepTest.running) return;
  if (+$('tdMode').value !== MODE.VELOCITY) { toast('Step tests run in velocity mode; set Mode to Velocity', 'err'); return; }
  const axis = tuAxis(), target = parseFloat($('tdSet').value) || 0, dur = parseFloat($('stDur').value) || 2;
  if (!target) return;
  stepTest.running = true; stepTest.abortReason = null; axBusy($('stRun'), true, 'Stepping');
  try {
    const a = status.axes[axis];
    const m = await runStep(axis, target, dur, guardFor(stepTest), { vl: parseFloat($('tdVl').value) || 0, tl: parseFloat($('tdTl').value) || 0 });
    stepTest.runs.push({ ...m, label: `${a.label} kp ${fmt(a.tuning.kp, 3)} ki ${fmt(a.tuning.ki, 3)}`, kp: a.tuning.kp, ki: a.tuning.ki });
    if (stepTest.runs.length > 6) stepTest.runs.shift();
    renderStepRuns(target);
  } catch (e) { toast('Step test: ' + e.message, 'err'); }
  finally { clearOverride(axis); stepTest.running = false; axBusy($('stRun'), false); }
};
function renderStepRuns(target) {
  const cols = SERIES();
  stepTest.plot = drawStatic($('cSteps'), [...stepTest.runs.map((r, i) => ({ x: r.t, y: r.y, label: `Run ${i + 1}`, color: cols[i] })),
    { x: [-0.3, -0.01, 0, stepTest.runs[0] ? Math.max(...stepTest.runs[0].t) : 2], y: [0, 0, target, target], label: 'Command', color: css('muted'), dash: [6, 4] }], 'Speed, rad/s', 'Time from step, s', stepTest.plot);
  $('stTable').querySelector('tbody').innerHTML = stepTest.runs.map((r, i) => `<tr><td><span style="color:${cols[i]}">■</span> Run ${i + 1}</td><td class="num">${fmt(r.kp, 4)}</td><td class="num">${fmt(r.ki, 4)}</td><td class="num">${fmt(r.rise, 2)}</td><td class="num">${fmt(r.overshoot, 1)}</td><td class="num">${fmt(r.settle, 2)}</td><td class="num">${fmt(r.ssErr, 1)}</td><td class="num">${fmt(r.iae, 3)}</td></tr>`).join('');
}

// ---------------------------------------------------------------- auto-tune
const autotune = { running: false, abortReason: null, previous: {}, results: {}, idPlot: null, stepPlot: null, traces: [] };
$('atStep').oninput = () => { $('atRoll').textContent = `${((parseFloat($('atStep').value) || 0) * 0.127 * 2.5).toFixed(1)} m`; }; $('atStep').oninput();
function atSteps(list) { $('atSteps').innerHTML = list.map(s => `<li class="${s.cls || ''}"><span>${s.name}</span>${s.detail ? `<span class="d">${s.detail}</span>` : ''}</li>`).join(''); }
$('atAbort').onclick = () => { autotune.abortReason = 'aborted by the operator'; };
$('atStart').onclick = async () => {
  if (autotune.running) return;
  if (!$('atSafe').checked) { toast('Confirm the wheels are clear and someone is on the e-stop', 'err'); return; }
  if (!status || status.state !== 'IDLE') { toast(`Auto-tune needs the MIB IDLE (it is ${status ? status.state : 'unknown'}). Disable drive and clear the e-stop.`, 'err'); return; }
  const axes = $('atAxis').value === 'both' ? [2, 3] : [+$('atAxis').value];
  for (const ax of axes) { const a = status.axes[ax]; if (!a.online) { toast(`${a.label} is not reporting`, 'err'); return; } if (a.api < 2) { toast(`${a.label} runs motor API ${a.api}; tuning needs v2+`, 'err'); return; } }
  const opts = { A: Math.abs(parseFloat($('atStep').value) || 2), tau: Math.abs(parseFloat($('atTorque').value) || 3), wns: $('atWn').value.split(/[ ,]+/).map(Number).filter(x => x > 0).sort((a, b) => a - b), ovs: parseFloat($('atOvs').value) || 15, ground: $('atSetup').value === 'ground' };
  autotune.running = true; autotune.abortReason = null; autotune.traces = []; autotune.results = {};
  $('atStart').disabled = true; $('atAbort').disabled = false; $('atResultRow').style.display = 'none'; axBusy($('atStart'), true, 'Tuning');
  $('atTable').querySelector('tbody').innerHTML = '';
  try {
    for (const ax of axes) await autotuneAxis(ax, opts);
    $('atResultRow').style.display = '';
    toast('Auto-tune finished. Gains are live but not saved.', 'ok');
  } catch (e) {
    toast('Auto-tune stopped: ' + e.message, 'err'); sys('auto-tune stopped: ' + e.message);
    for (const [ax, g] of Object.entries(autotune.previous)) { try { await link.tune(+ax, g.kp, g.ki, 0, false); } catch (_) {} }
    if (Object.keys(autotune.previous).length) sys('previous gains restored');
  } finally {
    stopAllOverrides(true); autotune.running = false; axBusy($('atStart'), false); $('atStart').disabled = !link; $('atAbort').disabled = true;
  }
};
async function autotuneAxis(axis, o) {
  const a0 = status.axes[axis], seg = a0.segment, guard = guardFor(autotune), cols = SERIES();
  autotune.previous[axis] = { kp: a0.tuning.kp, ki: a0.tuning.ki, pll: a0.tuning.pll };
  const vl = o.A * 1.6 + 1, tl = Math.max(o.tau * 1.5, 6);
  const steps = [
    { name: `${a0.label}: drag`, detail: '' }, { name: `${a0.label}: plant gain`, detail: '' },
    ...o.wns.map(w => ({ name: `${a0.label}: verify ω<sub>n</sub> ${w} rad/s`, detail: '' })), { name: `${a0.label}: apply`, detail: '' }];
  const mark = (i, cls, detail) => { steps[i].cls = cls; if (detail !== undefined) steps[i].detail = detail; atSteps(steps); };
  const vel = (v, accel = 4) => ({ state: REQ_STATE.ARMED, mode: MODE.VELOCITY, velocity: v, torqueLimit: tl, velLimit: vl, accelLimit: accel });
  let dir = 1; const flip = () => { if (o.ground) dir = -dir; return dir; };

  // 1. Drag: hold half the step speed on the present gains; the current it takes is drag.
  mark(0, 'now', 'holding half the step speed on the present gains');
  const v0 = 0.5 * o.A * flip();
  setOverride(axis, vel(v0)); await guard(2600);
  const dr = link.store.since(link.store.now() - 1.0, [`${seg}.vel`, `${seg}.iq`]);
  const vMean = mean(dr.map(r => r[1])), iDrag = Math.abs(mean(dr.map(r => r[2])));
  if (!Number.isFinite(iDrag)) throw new Error('no MotorState from ' + seg);
  if (Math.abs(vMean) < 0.3 * Math.abs(v0)) throw new Error(`the wheel did not turn (${fmt(vMean)} rad/s at ${fmt(v0)}): check the brake, the gains, or raise the torque limit`);
  setOverride(axis, vel(0, 6)); await guard(1500);
  mark(0, 'ok', `i<sub>drag</sub> = ${fmt(iDrag, 2)} A at ${fmt(vMean, 2)} rad/s`);

  // 2. Plant gain: a torque step from rest; the speed slope over the current above drag.
  // MotorState arrives at 50 Hz (20 Hz before PACE RACER ab44267), so the spin-up must last
  // long enough to sample: a probe step first, then the torque rescaled so the wheel takes
  // about 0.6 s to reach 1.2x the step.
  const torqueStep = async (s, tau) => {
    const tMark = link.store.now();
    setOverride(axis, { state: REQ_STATE.ARMED, mode: MODE.TORQUE, torque: s * tau, torqueLimit: tl, velLimit: vl, accelLimit: 0 });
    const tStart = performance.now();
    while (performance.now() - tStart < 1500) { await guard(40); if (Math.abs(link.store.get(`${seg}.vel`)) > o.A * 1.2) break; }
    setOverride(axis, vel(0, 6)); await guard(1600);
    const rows = link.store.since(tMark, [`${seg}.vel`, `${seg}.iq`, `${seg}.cmd_torque`, `${seg}.torque`]);
    const torq = rows.filter(r => Math.abs(r[3] - s * tau) < 1e-3);
    const onset = torq.find(r => Math.abs(r[1]) > 0.15);
    // MotorState repeats between updates at the 50 Hz telemetry rate: fit distinct samples.
    const win = onset ? torq.filter((r, i, all) => r[0] >= onset[0] && (i === 0 || r[1] !== all[i - 1][1])) : [];
    const fit = win.length >= 2 ? linfit(win.map(r => r[0]), win.map(r => s * r[1])) : { m: NaN, b: NaN };
    const iNet = Math.abs(mean(win.map(r => r[2]))) - iDrag;
    const kt = mean(torq.map(r => Math.abs(r[4]))) / mean(torq.map(r => Math.abs(r[2])));
    return { rows, win, fit, iNet, kt, tau, s };
  };
  let tau = o.tau, res = null;
  for (let tries = 0; tries < 5; tries++) {
    mark(1, 'now', `torque step ${fmt(tau, 2)} N·m${tries ? ` (try ${tries + 1})` : ''}`);
    res = await torqueStep(flip(), tau);
    if (!res.win.length) { tau = Math.min(tl, tau * 1.6); continue; }            // no motion: push harder
    if (res.win.length >= 6) break;                                               // enough samples
    const Kr = res.fit.m / res.iNet;
    tau = (res.win.length >= 2 && Kr > 0 && Number.isFinite(res.kt)) ? res.kt * (iDrag + 2 * o.A / Kr) : tau * 0.4;
    tau = Math.max(0.2, Math.min(tl, tau));
  }
  if (!res.win.length) throw new Error(`no motion up to a ${fmt(tau, 1)} N·m step`);
  if (res.win.length < 4) throw new Error(`the wheel spun up too fast to sample (${res.win.length} distinct MotorStates) even at ${fmt(res.tau, 2)} N·m`);
  const { rows, win, fit, iNet, s } = res;
  if (!(fit.m > 0) || !(iNet > 0.05)) throw new Error(`plant fit failed (α ${fmt(fit.m)} rad/s², net current ${fmt(iNet)} A)`);
  const K = fit.m / iNet;
  const t0 = rows[0][0];
  autotune.idPlot = drawStatic($('cAtId'), [
    { x: rows.map(r => r[0] - t0), y: rows.map(r => s * r[1]), label: `${a0.label} speed`, color: cols[axis === 2 ? 0 : 1] },
    { x: win.map(r => r[0] - t0), y: win.map(r => fit.m * r[0] + fit.b), label: 'fit', color: css('muted'), dash: [6, 4] }], 'Speed, rad/s', 'Time, s', autotune.idPlot);
  $('atIdCap').innerHTML = `${a0.label}: α = ${fmt(fit.m, 1)} rad/s² at ${fmt(iNet, 2)} A above drag → <b>K = ${fmt(K, 2)} rad/s² per A</b>${o.ground ? ' (chair on the ground: includes the chair\'s inertia)' : ''}.`;
  mark(1, 'ok', `${fmt(res.tau, 2)} N·m: α ${fmt(fit.m, 1)} rad/s² at ${fmt(iNet, 2)} A net, ${win.length} samples → K = ${fmt(K, 2)} (rad/s²)/A`);

  // 3. Candidates: ζ = 1 at ωn, so kp = 2ωn/K, ki = ωn²/K; verify each with a speed step.
  const runs = [];
  for (let i = 0; i < o.wns.length; i++) {
    const wn = o.wns[i], kp = 2 * wn / K, ki = wn * wn / K;
    mark(2 + i, 'now', `k<sub>p</sub> ${fmt(kp, 4)}, k<sub>i</sub> ${fmt(ki, 4)}`);
    await applyGains(axis, kp, ki, 0, false);
    const target = o.A * flip();
    const m = await runStep(axis, target, 2.0, guard, { vl, tl });
    const ok = m.overshoot <= o.ovs && Math.abs(m.ssErr) < 10 && Number.isFinite(m.rise);
    runs.push({ wn, kp, ki, ...m, ok });
    autotune.traces.push({ x: m.t, y: m.y.map(v => v / target * o.A), label: `${a0.label} ωn ${wn}`, color: cols[autotune.traces.length % 6] });
    autotune.stepPlot = drawStatic($('cAtSteps'), [...autotune.traces, { x: [-0.3, -0.01, 0, 2], y: [0, 0, o.A, o.A], label: 'Command', color: css('muted'), dash: [6, 4] }], 'Speed, rad/s', 'Time from step, s', autotune.stepPlot);
    $('atTable').querySelector('tbody').insertAdjacentHTML('beforeend', `<tr><td>${a0.label}</td><td class="num">${wn}</td><td class="num">${fmt(kp, 4)}</td><td class="num">${fmt(ki, 4)}</td><td class="num">${fmt(m.rise, 2)}</td><td class="num">${fmt(m.overshoot, 1)}</td><td class="num">${fmt(m.ssErr, 1)}</td><td><span class="ax-status ${ok ? 'good' : 'warning'}">${ok ? 'Passes' : 'Too lively'}</span></td></tr>`);
    mark(2 + i, ok ? 'ok' : 'bad', `k<sub>p</sub> ${fmt(kp, 4)}, k<sub>i</sub> ${fmt(ki, 4)} → rise ${fmt(m.rise, 2)} s, overshoot ${fmt(m.overshoot, 0)} %, ss error ${fmt(m.ssErr, 1)} %`);
    if (m.overshoot > 3 * o.ovs) { for (let j = i + 1; j < o.wns.length; j++) mark(2 + j, 'skip', 'skipped: the previous try was already far too lively'); break; }
  }
  // 4. Keep the fastest that passes; with none, the slowest try.
  const pick = [...runs].reverse().find(r => r.ok) || runs[0];
  await applyGains(axis, pick.kp, pick.ki, 0, false);
  autotune.results[axis] = pick;
  mark(steps.length - 1, 'ok', `${pick.ok ? '' : 'no try passed; '}applied ω<sub>n</sub> ${pick.wn}: <b>k<sub>p</sub> ${fmt(pick.kp, 4)}, k<sub>i</sub> ${fmt(pick.ki, 4)}</b> (live, not saved)`);
  sys(`auto-tune ${seg}: K ${fmt(K, 3)} (rad/s^2)/A, drag ${fmt(iDrag, 2)} A; applied wn ${pick.wn}: kp ${fmt(pick.kp, 4)} ki ${fmt(pick.ki, 4)}`);
  clearOverride(axis);
}
$('atSave').onclick = async () => {
  try { for (const [ax, r] of Object.entries(autotune.results)) await applyGains(+ax, r.kp, r.ki, 0, true); toast('Gains saved to the board NVS', 'ok'); } catch (e) { toast('Save: ' + e.message, 'err'); }
};
$('atRevert').onclick = async () => {
  try { for (const [ax, g] of Object.entries(autotune.previous)) await applyGains(+ax, g.kp, g.ki, 0, false); toast('Previous gains restored', 'ok'); $('atResultRow').style.display = 'none'; } catch (e) { toast('Revert: ' + e.message, 'err'); }
};

// ---------------------------------------------------------------- drive tab
$('drEnable').onclick = async () => { try { await link.drive(true, +$('drProfile').value); toast('Drive enabled', 'ok'); } catch (e) { toast('Enable: ' + e.message, 'err'); } };
$('drDisable').onclick = async () => { padRelease(); try { await link.drive(false, +$('drProfile').value); toast('Drive disabled', 'ok'); } catch (e) { toast('Disable: ' + e.message, 'err'); } };
$('drProfile').onchange = async () => { if (status && status.state === 'DRIVE_ENABLED') { try { await link.drive(true, +$('drProfile').value); } catch (e) { toast(e.message, 'err'); } } };
// The MIB's drive profiles (MIBconfig.hpp). The trajectory planner scales the normalized stick
// by the active profile's maxima, so the pad gain is a stick deflection, not a speed.
const PROFILE_MAX = [{ v: 0.35, w: 1.0 }, { v: 0.75, w: 2.0 }, { v: 1.0, w: 3.14159 }];
function renderPadScale() {
  const g = +$('padGain').value, p = status ? status.profile : +$('drProfile').value, m = PROFILE_MAX[p] || PROFILE_MAX[1];
  $('padGainV').textContent = Math.round(g * 100) + ' %';
  $('padScale').innerHTML = `Full pad = a real stick at <b>${Math.round(g * 100)} %</b> (stick ±${g.toFixed(2)}): up to <b>${(g * m.v).toFixed(2)} m/s</b> forward and <b>${(g * m.w).toFixed(2)} rad/s</b> turning in ${PROFILE_NAMES[p] || 'NORMAL'} (full stick: ${m.v} m/s, ${m.w.toFixed(1)} rad/s). The MIB keeps stick x and y inside the unit circle.`;
}
const PROFILE_NAMES = ['LOW', 'NORMAL', 'HIGH'];
$('padGain').oninput = renderPadScale; $('drProfile').addEventListener('change', renderPadScale);
renderPadScale();
// Drive tiles: current and speed of each drive wheel, live.
setInterval(() => {
  if (!link || !link.store.count || !$('tab-drive').classList.contains('on')) return;
  const st = link.store;
  for (const [side, seg] of [['L', 'drive_left'], ['R', 'drive_right']]) {
    axSet($('drIq' + side), st.get(`${seg}.iq`), { digits: 2 });
    $('drVel' + side).textContent = `${fmt(st.get(`${seg}.vel`), 2)} rad/s (cmd ${fmt(st.get(`${seg}.cmd_vel`), 2)}) · ${fmt(st.get(`${seg}.torque`), 2)} N·m`;
  }
  renderPadScale();
}, 200);
const padState = { down: false, x: 0, y: 0, timer: null };
function padMove(e) {
  const r = $('pad').getBoundingClientRect(); let x = ((e.clientX - r.left) / r.width) * 2 - 1, y = -(((e.clientY - r.top) / r.height) * 2 - 1);
  const m = Math.hypot(x, y); if (m > 1) { x /= m; y /= m; }
  padState.x = x; padState.y = y; $('knob').style.left = `${(x + 1) * 50}%`; $('knob').style.top = `${(1 - y) * 50}%`;
}
function padSend() { if (!link || !padState.down) return; const g = +$('padGain').value; link.stick(padState.x * g, padState.y * g).catch(() => {}); }
function padRelease() {
  if (!padState.down) return; padState.down = false; clearInterval(padState.timer); $('pad').classList.remove('live');
  $('knob').style.left = '50%'; $('knob').style.top = '50%';
  if (link) link.stick(0, 0).catch(() => {});
}
$('pad').addEventListener('pointerdown', e => {
  if (!link || !status || !['DRIVE_ENABLED', 'AUTONOMOUS'].includes(status.state)) { toast('Enable drive first', 'err'); return; }
  e.preventDefault(); $('pad').setPointerCapture(e.pointerId); padState.down = true; $('pad').classList.add('live'); padMove(e); padSend();
  padState.timer = setInterval(padSend, 50);
});
$('pad').addEventListener('pointermove', e => { if (padState.down) padMove(e); });
['pointerup', 'pointercancel', 'lostpointercapture'].forEach(ev => $('pad').addEventListener(ev, padRelease));

// ---------------------------------------------------------------- seat tab
$('seatGrid').innerHTML = SEAT_AXES.map(s => `<div class="ax-panel" data-seat="${s.id}">
  <h2>${s.label}</h2><div class="ax-cap">${s.min} to ${s.max} ${s.unit}, jog ${s.step} ${s.unit}</div>
  <div class="ax-value" data-rep>–<small>${s.unit}</small></div>
  <div class="row"><input class="ax-range" type="range" min="${s.min}" max="${s.max}" step="${s.step}" value="${Math.max(s.min, 0)}" data-slider></div>
  <div class="row"><button class="ax-btn" data-jog="-1" disabled>− ${s.step}</button><input class="ax-input" type="number" step="${s.step}" value="${Math.max(s.min, 0)}" style="width:90px" data-target>
  <button class="ax-btn" data-jog="1" disabled>+ ${s.step}</button><button class="ax-btn primary" data-send disabled>Move</button></div></div>`).join('');
for (const card of $('seatGrid').children) {
  const s = SEAT_AXES[+card.dataset.seat], tgt = card.querySelector('[data-target]'), sl = card.querySelector('[data-slider]');
  const send = async v => { v = Math.max(s.min, Math.min(s.max, v)); tgt.value = v; sl.value = v; try { await link.seat(s.id, v); } catch (e) { toast(`${s.label}: ${e.message}`, 'err'); } };
  sl.oninput = () => { tgt.value = sl.value; };
  sl.onchange = () => link && send(+sl.value);
  card.querySelector('[data-send]').onclick = () => send(+tgt.value);
  card.querySelectorAll('[data-jog]').forEach(b => b.onclick = () => send(+tgt.value + s.step * +b.dataset.jog));
}
function renderSeatReported(s) {
  for (const card of $('seatGrid').children) { const i = +card.dataset.seat, el = card.querySelector('[data-rep]'); axSet(el, s.seat[i], { digits: 1 }); }
}

// ---------------------------------------------------------------- sensors tab
let channelNames = [], plotted = new Set();
function buildChannelList(names) {
  channelNames = names; plotted = new Set([...plotted].filter(n => names.includes(n)));
  if (!plotted.size && names.includes('sensor.mib_chip_temp')) plotted.add('sensor.mib_chip_temp');
  renderChannelList(); rebuildCustom(); buildSensorTiles(); allCharts.forEach(c => c.rebuild());
}
function renderChannelList() {
  const f = $('chFilter').value.trim().toLowerCase();
  const list = channelNames.filter(n => !f || n.toLowerCase().includes(f));
  $('chList').innerHTML = list.map(n => `<label><input type="checkbox" value="${n}" ${plotted.has(n) ? 'checked' : ''}><span style="flex:1">${n}</span><span class="ax-muted" data-v="${n}">–</span></label>`).join('') || '<div class="empty" style="padding:8px">No channels match.</div>';
  $('chList').querySelectorAll('input').forEach(i => i.onchange = () => {
    if (i.checked && plotted.size >= 6) { i.checked = false; toast('Six channels at most per plot', 'err'); return; }
    i.checked ? plotted.add(i.value) : plotted.delete(i.value); rebuildCustom();
  });
}
$('chFilter').oninput = renderChannelList;
const custom = new LiveChart($('cCustom'), [], 'Value', 20);
// Units of the firmware's channels, by name, for the custom plot's y-axis title.
function unitOf(n) {
  const k = n.split('.').pop();
  if (/^(cmd_)?vel$/.test(k)) return 'rad/s'; if (k === 'pos') return 'rad'; if (k === 'iq') return 'A';
  if (/torque$/.test(k)) return 'N·m'; if (k === 'vbus') return 'V'; if (/temp$/.test(k)) return '°C';
  if (/^imu_/.test(k)) return 'deg'; if (/tilt$/.test(k)) return 'deg'; if (k === 'elevation' || k === 'translation') return 'mm';
  if (/^stick_/.test(k)) return 'normalized'; return 'raw';
}
function rebuildCustom() {
  const cols = SERIES(), units = [...new Set([...plotted].map(unitOf))];
  custom.yLabel = $('cCustomY').value || (units.length === 1 ? `Value, ${units[0]}` : `Value, mixed: ${units.join(' / ')}`);
  custom.rebuild([...plotted].map((n, i) => ({ ch: n, label: n, color: cols[i] })));
  $('cCustomCap').textContent = plotted.size ? `${plotted.size} channel${plotted.size > 1 ? 's' : ''}, last 20 s.` : 'Pick channels on the left.';
}
$('cCustomY').onchange = rebuildCustom;
function buildSensorTiles() {
  const sensors = channelNames.filter(n => n.startsWith('sensor.'));
  $('sensorTiles').innerHTML = sensors.length ? sensors.map(n => { const k = n.slice(7); return `<div class="ax-tile"><div class="ax-label">${k.replace(/_/g, ' ')}</div><div class="ax-value" data-sensor="${n}">–<small>${unitOf(n) === 'raw' ? '' : unitOf(n)}</small></div><div class="ax-sub">${n}</div></div>`; }).join('')
    : '<div class="empty">No sensors reported. A sensor appears here when the MIB\'s firmware adds a <code>sensor.*</code> channel.</div>';
}
setInterval(() => {
  if (!link || !link.store.count) return;
  const st = link.store; $('mRate').textContent = `${st.rate.toFixed(0)} Hz`;
  axSet($('tChip'), st.get('sensor.mib_chip_temp'), { digits: 1 });
  document.querySelectorAll('[data-sensor]').forEach(el => axSet(el, st.get(el.dataset.sensor), { digits: 2 }));
  if ($('tab-sensors').classList.contains('on')) document.querySelectorAll('[data-v]').forEach(el => { el.textContent = fmt(st.get(el.dataset.v), 3); });
}, 250);
$('csvBtn').onclick = () => {
  const st = link.store, names = st.names, w = st.window(names, +$('csvWin').value);
  const lines = ['t_s,' + names.join(',')]; for (let i = 0; i < w[0].length; i++) lines.push([w[0][i].toFixed(4), ...names.map((_, j) => w[j + 1][i] ?? '')].join(','));
  download(`mib-telemetry-${stamp()}.csv`, lines.join('\n'));
};

// ---------------------------------------------------------------- live charts
const c = () => SERIES();
const allCharts = [
  new LiveChart($('cOvVel'), [
    { ch: 'drive_left.vel', label: 'Drive L', color: c()[0] }, { ch: 'drive_left.cmd_vel', label: 'Drive L cmd', color: c()[0], dash: [6, 4] },
    { ch: 'drive_right.vel', label: 'Drive R', color: c()[1] }, { ch: 'drive_right.cmd_vel', label: 'Drive R cmd', color: c()[1], dash: [6, 4] }], 'Wheel speed, rad/s'),
  new LiveChart($('cOvIq'), [{ ch: 'drive_left.iq', label: 'Drive L', color: c()[0] }, { ch: 'drive_right.iq', label: 'Drive R', color: c()[1] }], 'Current i_q, A'),
  new LiveChart($('cDrIq'), [{ ch: 'drive_left.iq', label: 'Drive L', color: c()[0] }, { ch: 'drive_right.iq', label: 'Drive R', color: c()[1] }], 'Current i_q, A'),
  new LiveChart($('cDrStick'), [{ ch: 'mib.stick_x', label: 'x (turn)', color: c()[0] }, { ch: 'mib.stick_y', label: 'y (forward)', color: c()[2] }], 'Stick, normalized'),
  new LiveChart($('cDrVel'), [
    { ch: 'drive_left.vel', label: 'Drive L', color: c()[0] }, { ch: 'drive_left.cmd_vel', label: 'Drive L cmd', color: c()[0], dash: [6, 4] },
    { ch: 'drive_right.vel', label: 'Drive R', color: c()[1] }, { ch: 'drive_right.cmd_vel', label: 'Drive R cmd', color: c()[1], dash: [6, 4] }], 'Wheel speed, rad/s'),
  new LiveChart($('cSeat'), SEAT_AXES.map((s, i) => ({ ch: 'seat.' + s.key, label: `${s.label}, ${s.unit}`, color: c()[i] })), 'Seat target, deg or mm', 30),
];
const tuneCharts = [
  new LiveChart($('cTuVel'), [{ ch: () => `${tuSeg()}.vel`, label: 'Measured', color: c()[0] }, { ch: () => `${tuSeg()}.cmd_vel`, label: 'Command', color: c()[1], dash: [6, 4] }], 'Speed, rad/s'),
  new LiveChart($('cTuIq'), [{ ch: () => `${tuSeg()}.iq`, label: 'i_q', color: c()[2] }], 'Current i_q, A'),
];

// Link choice: remembered; Ethernet by default. Served by eth_bridge.py, the page uses its own origin.
$('transport').value = localStorage.getItem('mib.transport') || 'eth';
$('ethTarget').value = localStorage.getItem('mib.ethTarget') || '192.168.4.1';
$('ethBridge').value = localStorage.getItem('mib.ethBridge') || 'ws://localhost:8765';
const showEth = () => { const on = $('transport').value === 'eth'; $('ethTarget').style.display = $('ethBridge').style.display = on ? '' : 'none'; };
$('transport').onchange = showEth; showEth();
if (/^(localhost|127\.0\.0\.1)$/.test(location.hostname)) fetch('bridge.json').then(r => r.ok ? r.json() : null).then(j => {
  if (!j || !j.bridge) return;
  $('ethBridge').value = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}`; $('ethBridge').style.display = 'none';
  if (!localStorage.getItem('mib.ethTarget')) $('ethTarget').value = j.default_target;
  sys(`served by eth_bridge.py: Ethernet goes through ${location.host}`);
}).catch(() => {});

// ---------------------------------------------------------------- firmware (OTA)
// The PACE RACER dashboard's flow (atos-drive/tools/dashboard.html): BEGIN with the size,
// 4 KiB DATA chunks each acknowledged, FINISH, then the board reboots into the new slot; the
// panel reconnects and marks the new image valid. The target is the MIB (the panel's own
// link) or a PACE RACER (its tcp :5555 through eth_bridge.py, found with "PACE?").
const fw = { picked: null, busy: false, abort: false, rebooting: false, builds: [], racers: new Map() };
const FW_PROJECT = { mib: 'rammp_mib', racer: 'pace_racer_atos_drive' };
const fwCstr = (img, a, b) => new TextDecoder().decode(img.subarray(a, b)).split('\0')[0];
const bridgeServed = () => /^(localhost|127\.0\.0\.1)$/.test(location.hostname);
// The bridge's HTTP base: this origin when it served the page, else the Ethernet bridge field.
const bridgeBase = () => bridgeServed() ? '' : $('ethBridge').value.trim().replace(/^ws/, 'http').replace(/\/$/, '') + '/';
function fwTarget() {
  const v = $('fwTarget').value;
  if (v === 'mib') return { kind: 'mib', label: 'MIB' };
  const r = fw.racers.get(v.slice(6));
  return { kind: 'racer', addr: v.slice(6), label: r && r.axis_name ? `RACER ${r.axis_name}` : `RACER ${v.slice(6)}`, info: r };
}
function fwSource() {
  if (fw.picked) return { name: fw.picked.name, read: async () => new Uint8Array(await fw.picked.arrayBuffer()) };
  const b = fw.builds.find(x => x.id === $('fwBuild').value); if (!b) return null;
  return { name: `${b.id} · ${b.version}`, read: async () => {
    const r = await fetch(`${bridgeBase()}builds/${b.id}/${b.app.file}`, { cache: 'no-store' });
    if (!r.ok) throw new Error(`the bridge could not read ${b.id}: HTTP ${r.status}`);
    return new Uint8Array(await r.arrayBuffer());
  } };
}
function fwRefresh() {
  const t = fwTarget(), src = fwSource(), reach = t.kind === 'mib' ? !!link : true;
  $('fwFlash').textContent = `Flash ${t.label}`;
  $('fwFlash').disabled = fw.busy || !reach || !src; $('fwValid').disabled = fw.busy || !reach; $('fwRollback').disabled = fw.busy || !reach; $('fwRead').disabled = fw.busy || !reach;
  if (!fw.busy) { $('fwStatus').className = 'ax-status'; $('fwStatus').textContent = !src ? 'No image chosen' : !reach ? src.name + ' · connect to flash' : src.name; }
}
function renderBuildList() {
  const want = FW_PROJECT[fwTarget().kind], list = fw.builds.filter(b => b.project === want);
  $('fwBuild').innerHTML = list.length ? list.map(b => `<option value="${b.id}">${b.id} · ${b.version} · ${b.built}</option>`).join('')
    : `<option value="">no ${want} builds from the bridge</option>`;
  fwRefresh();
}
function renderTargets() {
  const keep = $('fwTarget').value;
  $('fwTarget').innerHTML = '<option value="mib">MIB (the connected link)</option>' + [...fw.racers.entries()].map(([addr, r]) =>
    `<option value="racer:${addr}">PACE RACER ${r.axis_name || '(axis ?)'} · ${addr}${r.version ? ' · ' + r.version : ''}</option>`).join('');
  if ([...$('fwTarget').options].some(o => o.value === keep)) $('fwTarget').value = keep;
}
// Info for the "Running image" table: the MIB from its link, a RACER over a short-lived one.
const racerLink = async addr => { const l = new FrameLink(new WsTransport($('ethBridge').value.trim(), addr.includes(':') ? addr : `${addr}:5555`)); await l.connect(); return l; };
async function renderFwInfo() {
  const t = fwTarget();
  if (t.kind === 'mib' && !link) { $('fwInfo').innerHTML = '<tr><td class="empty">Not connected.</td></tr>'; return; }
  let j;
  try {
    if (t.kind === 'mib') { j = info = await link.info(); $('mFw').textContent = `fw ${j.version} (${j.built})`; }
    else { const l = await racerLink(t.addr); try { j = await l.info(); } finally { await l.close(); } }
  } catch (e) { $('fwInfo').innerHTML = `<tr><td class="empty">No info from ${t.label}: ${e.message}</td></tr>`; return; }
  const rows = [['Board', t.kind === 'mib' ? 'MIB' : `PACE RACER ${j.axis_name || ''} (axis ${j.axis ?? '?'})`], ['Version', j.version], ['Built', j.built], ['Slot', j.partition || '–'],
    ['State', j.pending_verify ? '<span class="ax-status warning">Pending verify</span>' : '<span class="ax-status good">Confirmed</span>'],
    ['Reset reason', j.reset_reason || '–'], ['Up', j.uptime_s !== undefined ? `${j.uptime_s} s` : '–']];
  if (j.motor) rows.push(['Motor profile', j.motor]);
  if (j.mac) rows.push(['MAC', j.mac]);
  $('fwInfo').innerHTML = rows.map(([k, v]) => `<tr><td class="ax-label">${k}</td><td class="ax-n">${v ?? '–'}</td></tr>`).join('');
}
async function loadBuilds() {
  try { const r = await fetch(bridgeBase() + 'builds', { cache: 'no-store' }); if (r.ok) fw.builds = await r.json(); } catch (_) {}
  renderBuildList();
}
async function findRacers() {
  axBusy($('fwFind'), true, 'Asking');
  try {
    const r = await fetch(bridgeBase() + 'discover', { cache: 'no-store' }); if (!r.ok) throw new Error('HTTP ' + r.status);
    const boards = await r.json();
    for (const b of boards) fw.racers.set(b.ip, b);
    renderTargets(); sys(`PACE RACERs answering: ${boards.length ? boards.map(b => `${b.axis_name || '?'} ${b.ip} ${b.version || ''}`).join(', ') : 'none'}`);
    toast(boards.length ? `Found ${boards.length} PACE RACER${boards.length > 1 ? 's' : ''}` : 'No PACE RACER answered', boards.length ? 'ok' : '');
  } catch (e) { toast('Find RACERs needs eth_bridge.py: ' + e.message, 'err'); }
  finally { axBusy($('fwFind'), false); }
}
$('fwFind').onclick = findRacers;
$('fwAddRacer').onclick = () => { const a = $('fwRacerIp').value.trim(); if (!a) return; fw.racers.set(a, fw.racers.get(a) || {}); renderTargets(); $('fwTarget').value = 'racer:' + a; $('fwTarget').onchange(); };
$('fwTarget').onchange = () => { fw.picked = null; $('fwFile').value = ''; renderBuildList(); renderFwInfo(); };
$('fwRead').onclick = renderFwInfo;
$('fwFile').onchange = () => { fw.picked = $('fwFile').files[0] || null; fwRefresh(); };
$('fwBuild').onchange = () => { fw.picked = null; $('fwFile').value = ''; fwRefresh(); };
$('tab-firmware').addEventListener('dragover', e => e.preventDefault());
$('tab-firmware').addEventListener('drop', e => { e.preventDefault(); const f = e.dataTransfer.files[0]; if (f) { fw.picked = f; fwRefresh(); } });
$('fwAbort').onclick = () => { fw.abort = true; };
const u32le = n => { const p = new Uint8Array(4); new DataView(p.buffer).setUint32(0, n >>> 0, true); return p; };
// After the MIB reboots, reach it again. Web Serial: the port outlives the reboot and the
// link renegotiates its rate by itself, so just wait for answers. Otherwise the link dropped
// (TCP closed, USB re-enumerated): open a new one of the same kind.
async function reconnectAfterReboot(old, timeoutMs = 90000) {
  try { return await reconnectLoop(old, timeoutMs); } finally { fw.rebooting = false; }
}
async function reconnectLoop(old, timeoutMs) {
  const kind = old ? old.kind : $('transport').value, deadline = Date.now() + timeoutMs;
  $('mLink').className = 'ax-status warning'; $('mLink').textContent = 'Rebooting';
  await sleep(2500);
  if (kind === 'serial' && link && link.t === old) {
    while (Date.now() < deadline) { try { await link.info(); startPolling(250); renderFwInfo(); setConnected(true); return true; } catch (e) { await sleep(1000); } }
    return false;
  }
  if (link) { const l = link; link = null; clearInterval(pollTimer); try { await l.close(); } catch (_) {} }
  setConnected(false, 'Rebooting');
  while (Date.now() < deadline) {
    try {
      let t = makeTransport(kind, old);
      if (kind === 'usb' && !t.device && navigator.usb) t = new UsbTransport((await navigator.usb.getDevices()).find(d => d.vendorId === MIB_USB.vendorId));
      await openLink(t); return true;
    } catch (e) { await sleep(2000); }
  }
  return false;
}
// A RACER after its reboot: open a fresh link until it answers.
async function racerAfterReboot(addr, timeoutMs = 90000) {
  const deadline = Date.now() + timeoutMs; await sleep(3000);
  while (Date.now() < deadline) { try { return await racerLink(addr); } catch (e) { await sleep(2000); } }
  return null;
}
// BEGIN, DATA..., FINISH on `l` (a MibLink or a FrameLink).
async function otaWrite(l, img, say, bar) {
  say('BEGIN: the board erases its update slot');
  await l.ota(OTA.BEGIN, u32le(img.length), 60000);
  for (let sent = 0; sent < img.length;) {
    if (fw.abort) throw Object.assign(new Error('aborted'), { userAbort: true });
    const part = img.subarray(sent, Math.min(sent + SF_MAX_PAYLOAD, img.length));
    await l.ota(OTA.DATA, part, 5000); sent += part.length;
    bar.firstElementChild.style.width = (100 * sent / img.length).toFixed(1) + '%';
    say(`${(sent / 1024).toFixed(0)} / ${(img.length / 1024).toFixed(0)} KiB`);
  }
  say('FINISH: the board verifies the image');
  await l.ota(OTA.FINISH, null, 60000);
}
$('fwFlash').onclick = async () => {
  const t = fwTarget(), src = fwSource(); if (!src || (t.kind === 'mib' && !link)) return fwRefresh();
  if (t.kind === 'racer' && status && status.state === 'DRIVE_ENABLED') { toast('Disable drive on the MIB before flashing a wheel', 'err'); return; }
  if (!confirm(`Flash ${src.name} to the ${t.label}? It stops its motor${t.kind === 'mib' ? 's (drive disabled, every motor disarmed)' : ''} and reboots.`)) return;
  fw.busy = true; fw.abort = false; axBusy($('fwFlash'), true, 'Flashing'); $('fwAbort').disabled = false;
  const bar = $('fwBar'); bar.className = 'bar'; bar.firstElementChild.style.width = '0';
  const say = (s, cls = 'warning') => { $('fwStatus').className = 'ax-status ' + cls; $('fwStatus').textContent = s; };
  const t0 = performance.now(); let l = null;
  try {
    say('Reading ' + src.name);
    const img = await src.read();
    if (img[0] !== 0xE9) throw new Error('not an ESP app image (first byte is not 0xE9)');
    const project = fwCstr(img, 80, 112), version = fwCstr(img, 48, 80), want = FW_PROJECT[t.kind];
    if (project !== want && !confirm(`This image is "${project || 'unknown'}", not ${want}: the wrong firmware for the ${t.label}. Flash it anyway?`)) throw Object.assign(new Error('wrong image'), { userAbort: true });
    sys(`flashing ${t.label}: ${project} ${version}, ${(img.length / 1024).toFixed(0)} KiB`);
    let back;
    if (t.kind === 'mib') {
      const oldT = link.t; oldT.lastPort = oldT.port; oldT.lastDevice = oldT.device; fw.rebooting = true;
      await otaWrite(link, img, say, bar);
      fw.busy = false; say('Written; the MIB is rebooting');
      back = (await reconnectAfterReboot(oldT)) ? link : null;
    } else {
      say(`Connecting to ${t.label}`); l = await racerLink(t.addr);
      await otaWrite(l, img, say, bar);
      await l.close(); l = null; say(`Written; the ${t.label} is rebooting`);
      back = l = await racerAfterReboot(t.addr);
    }
    bar.className = 'bar done'; const dt = (performance.now() - t0) / 1000;
    if (back) {
      const st = await back.otaStatus();
      if (t.kind === 'racer' && fw.racers.has(t.addr)) { fw.racers.get(t.addr).version = st.version; renderTargets(); }
      if (st.pending) { await back.ota(OTA.MARK_VALID); say(`${t.label} runs ${st.version}, marked valid (${dt.toFixed(0)} s)`, 'good'); toast(`${t.label} updated to ${st.version}`, 'ok'); }
      else say(`${t.label} runs ${st.version}${st.version === version ? '' : ` (not ${version}: it rolled back?)`}`, st.version === version ? 'good' : 'critical');
    } else say(`The ${t.label} did not come back. A power cycle boots its previous image.`, 'critical');
  } catch (e) {
    bar.className = 'bar failed'; say(e.userAbort ? 'Aborted' : 'Failed: ' + e.message, 'critical');
    sys(`OTA ${t.label} ` + (e.userAbort ? 'aborted' : 'failed: ' + e.message)); fw.rebooting = false;
    const a = t.kind === 'mib' ? link : l; if (a) { try { await a.ota(OTA.ABORT); } catch (_) {} }
  } finally {
    if (l) await l.close();
    fw.busy = false; axBusy($('fwFlash'), false); $('fwAbort').disabled = true;
    const s = $('fwStatus').textContent, c = $('fwStatus').className; fwRefresh(); $('fwStatus').textContent = s; $('fwStatus').className = c;
    renderFwInfo();
  }
};
// Mark valid / roll back on the target.
async function fwOnTarget(fn) {
  const t = fwTarget();
  if (t.kind === 'mib') return fn(link, t);
  const l = await racerLink(t.addr); try { return await fn(l, t); } finally { await l.close(); }
}
$('fwValid').onclick = async () => { try { await fwOnTarget(l => l.ota(OTA.MARK_VALID)); toast('Running image marked valid', 'ok'); renderFwInfo(); } catch (e) { toast('Mark valid: ' + e.message, 'err'); } };
$('fwRollback').onclick = async () => {
  const t = fwTarget();
  if (!confirm(`Roll the ${t.label} back to its previous image and reboot?`)) return;
  try {
    if (t.kind === 'mib') { const oldT = link.t; oldT.lastPort = oldT.port; oldT.lastDevice = oldT.device; fw.rebooting = true; await link.ota(OTA.MARK_INVALID); sys('rolling back; the MIB is rebooting'); await reconnectAfterReboot(oldT); }
    else { await fwOnTarget(l => l.ota(OTA.MARK_INVALID)); sys(`rolling back; the ${t.label} is rebooting`); const l = await racerAfterReboot(t.addr); if (l) await l.close(); }
    renderFwInfo();
  } catch (e) { toast('Roll back: ' + e.message, 'err'); }
};
// Served by the bridge: list its builds now. Elsewhere (GitHub Pages) ask the bridge only
// when the Firmware tab opens, so the page does not reach for localhost on every visit.
if (bridgeServed()) loadBuilds();
else document.querySelector('[data-tab=firmware]').addEventListener('click', () => { if (!fw.builds.length) loadBuilds(); });

setConnected(false);
chartLoop();
sys('panel ready. Ethernet: run tools/mib_panel/eth_bridge.py and plug into the chair network. Web Serial: the MIB\'s USB-C. Or try the simulator.');
