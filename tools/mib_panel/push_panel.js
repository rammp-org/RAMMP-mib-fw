// push_panel.js: the Push assist tab. The board's link and simulator are in push_link.js; the
// MIB's gains go over the MIB link (panel.js `link`). Loaded after panel.js, whose helpers
// ($, makePlot, LiveChart, toast, sys, bridgeBase) it uses.
'use strict';

const pa = {
  link: null, target: 'sim', boards: new Map(), info: null,
  mibParams: null, mibLoaded: null, boardParams: null, gains: { ...PUSH_DEFAULT_GAINS },
};

// ---------------------------------------------------------------- params editor
// Renders a params list (the MIB's PARAMS reply or the board's GET_PARAMS) as grouped
// slider + number rows. `set(key, value)` returns the param after the device applied it.
function renderParams(el, list, set, onChange) {
  el.innerHTML = ''; let group = null;
  for (const p of list) {
    if (p.group !== group) { group = p.group; const h = document.createElement('div'); h.className = 'param-group'; h.textContent = group.replace(/^Push assist: /, ''); el.appendChild(h); }
    const row = document.createElement('div'); row.className = 'field prm';
    const lab = document.createElement('span'); lab.className = 'ax-label nc'; lab.textContent = p.label; lab.title = `${p.key} · default ${p.default}${p.unit ? ' ' + p.unit : ''}`;
    row.appendChild(lab);
    const mark = () => lab.classList.toggle('changed', p.value !== p.default);
    const apply = async v => {
      try { const r = await set(p.key, v); Object.assign(p, r); } catch (e) { toast(`${p.label}: ${e.message}`, 'err'); }
      show(); mark(); onChange && onChange();
    };
    let show;
    if (p.type === 'bool') {
      const c = document.createElement('label'); c.className = 'chk'; c.style.margin = '0';
      const box = document.createElement('input'); box.type = 'checkbox'; c.appendChild(box); c.appendChild(document.createTextNode(' on'));
      box.onchange = () => apply(box.checked ? 1 : 0); show = () => { box.checked = !!p.value; };
      row.appendChild(c);
    } else {
      const range = document.createElement('input'); range.type = 'range'; range.className = 'ax-range';
      Object.assign(range, { min: p.min, max: p.max, step: p.step });
      const right = document.createElement('span'); right.className = 'unit';
      const num = document.createElement('input'); num.type = 'number'; num.className = 'ax-input num'; Object.assign(num, { min: p.min, max: p.max, step: p.step });
      right.appendChild(num); right.appendChild(document.createTextNode(' ' + (p.unit || '')));
      let timer = null;
      range.oninput = () => { num.value = range.value; clearTimeout(timer); timer = setTimeout(() => apply(+range.value), 150); };
      num.onchange = () => apply(+num.value);
      show = () => { range.value = p.value; num.value = +(+p.value).toFixed(4); };
      row.appendChild(right); row.appendChild(range); // the slider runs full width under the label
    }
    show(); mark(); el.appendChild(row);
  }
}
function paramsDirty(list, saved) { return list.some(p => saved[p.key] !== p.value); }
const snapshot = list => Object.fromEntries(list.map(p => [p.key, p.value]));

// ---------------------------------------------------------------- MIB gains
function setGainsFrom(list) { pa.gains = { ...PUSH_DEFAULT_GAINS, ...snapshot(list) }; drawMap(); }
function mibParamsState() {
  const on = !!(link && pa.mibParams);
  for (const id of ['paMibSave', 'paMibDefaults', 'paMibReload']) $(id).disabled = !on;
  const st = $('paMibState');
  if (!on) { st.className = 'ax-status'; st.textContent = ''; return; }
  const dirty = paramsDirty(pa.mibParams, pa.mibLoaded);
  st.className = 'ax-status ' + (dirty ? 'warning' : 'good'); st.textContent = dirty ? 'Unsaved: live until a reboot' : 'Saved';
}
async function loadMibParams(fn = 'params') {
  if (!link) return;
  try {
    const j = await link[fn]();
    pa.mibParams = j.params.filter(p => p.group.startsWith('Push assist'));
    if (fn === 'params') pa.mibLoaded = j.saved ? snapshot(pa.mibParams) : Object.fromEntries(pa.mibParams.map(p => [p.key, NaN]));
    renderParams($('paMibParams'), pa.mibParams, (k, v) => link.setParam(k, v), () => { setGainsFrom(pa.mibParams); mibParamsState(); });
    setGainsFrom(pa.mibParams);
  } catch (e) {
    pa.mibParams = null;
    $('paMibParams').innerHTML = `<div class="empty">The MIB did not list its params (${e.message}). Its firmware may predate push-assist tuning: flash this branch's build.</div>`;
  }
  mibParamsState();
}
$('paMibReload').onclick = () => loadMibParams();
$('paMibDefaults').onclick = async () => { if (confirm('Put the push-assist gains back to their defaults on the MIB? They stay unsaved until you press Save.')) await loadMibParams('defaultParams'); };
$('paMibSave').onclick = async () => {
  try { await link.saveParams(); pa.mibLoaded = snapshot(pa.mibParams); toast('Push-assist gains saved on the MIB', 'ok'); } catch (e) { toast('Save: ' + e.message, 'err'); }
  mibParamsState();
};
// Follow the MIB link: load the gains when it comes up, go back to defaults when it goes.
let paSeenLink = null;
setInterval(() => {
  if (link === paSeenLink) return; paSeenLink = link;
  if (link) { setTimeout(() => loadMibParams(), 600); return; }
  pa.mibParams = null; pa.gains = { ...PUSH_DEFAULT_GAINS }; drawMap(); mibParamsState();
  $('paMibParams').innerHTML = '<div class="empty">Connect to the MIB (top of the page) to tune its gains. The preview uses the defaults until then.</div>';
}, 500);

// ---------------------------------------------------------------- board link
function paTargets() {
  const cur = $('paTarget').value;
  $('paTarget').innerHTML = '<option value="sim">Simulated board</option>' + [...pa.boards.entries()].map(([addr, b]) =>
    `<option value="${addr}">Push assist · ${addr}${b.firmware ? ' · ' + b.firmware : ''}</option>`).join('');
  if ([...$('paTarget').options].some(o => o.value === cur)) $('paTarget').value = cur;
}
// Boards found by the bridge's /discover: the push-assist board answers PUSH? with product push_assist.
function notePushBoards(boards) { for (const b of boards) if (b.product === 'push_assist') pa.boards.set(b.ip, b); paTargets(); renderTargets(); }
$('paFind').onclick = async () => {
  try {
    const r = await fetch(bridgeBase() + 'discover', { cache: 'no-store' }); if (!r.ok) throw new Error('HTTP ' + r.status);
    const found = (await r.json()).filter(b => b.product === 'push_assist'); notePushBoards(found);
    toast(found.length ? `Found ${found.length} push-assist board${found.length > 1 ? 's' : ''}` : 'No push-assist board answered', found.length ? 'ok' : '');
    if (found.length) $('paTarget').value = found[0].ip;
  } catch (e) { toast('Find needs eth_bridge.py: ' + e.message, 'err'); }
};
$('paAdd').onclick = () => { const a = $('paIp').value.trim(); if (!a) return; pa.boards.set(a, pa.boards.get(a) || {}); paTargets(); renderTargets(); $('paTarget').value = a; };
const paTransport = addr => addr === 'sim' ? new SimPushTransport() : new WsTransport($('ethBridge').value.trim(), addr.includes(':') ? addr : `${addr}:5555`);

function paSetConnected(on, label) {
  $('paConnect').textContent = on ? 'Disconnect' : 'Connect';
  $('paLink').className = 'ax-status ' + (on ? 'live' : label === 'Link lost' ? 'critical' : '');
  $('paLink').textContent = label || (on ? 'Live' : 'Disconnected');
  $('paTare').disabled = !on; $('paTarget').disabled = on;
  for (const id of ['paBoardSave', 'paBoardDefaults', 'paBoardReload']) $(id).disabled = !(on && pa.boardParams);
}
async function paConnect(addr = $('paTarget').value) {
  const l = new PushLink(paTransport(addr), {
    onClose: unexpected => { if (pa.link === l) { pa.link = null; paSetConnected(false, unexpected ? 'Link lost' : null); } },
  });
  await l.connect(); pa.link = l; pa.target = addr; paSetConnected(true);
  try { pa.info = await l.info(); } catch (e) { pa.info = null; }
  renderPaInfo(); await loadBoardParams();
  return l;
}
async function paDisconnect() { const l = pa.link; pa.link = null; if (l) await l.close(); paSetConnected(false); }
$('paConnect').onclick = async () => {
  if (pa.link) return paDisconnect();
  try { await paConnect(); sys(`push-assist board connected (${pa.target})`); } catch (e) { toast('Push-assist board: ' + e.message, 'err'); paSetConnected(false); }
};
$('paTare').onclick = async () => {
  if (!confirm('Tare both handles? Take your hands off them first.')) return;
  $('paTare').disabled = true; const t0 = $('paTare').textContent; $('paTare').textContent = 'Taring…';
  try { const r = await pa.link.tare(); toast(r.left && r.right ? 'Both handles tared' : `Tare: left ${r.left ? 'ok' : 'failed'}, right ${r.right ? 'ok' : 'failed'}`, r.left && r.right ? 'ok' : 'err'); }
  catch (e) { toast('Tare: ' + e.message, 'err'); }
  $('paTare').textContent = t0; $('paTare').disabled = !pa.link;
};
function renderPaInfo() {
  const j = pa.info;
  if (!j) { $('paInfo').innerHTML = `<tr><td class="empty">${pa.link ? 'The board did not answer GET_INFO.' : 'Not connected.'}</td></tr>`; return; }
  const amp = ok => ok ? '<span class="ax-status good">answering</span>' : '<span class="ax-status critical">no amp</span>';
  const rows = [['Firmware', j.firmware || j.version || '–'], ['Built', j.built || '–'], ['Slot', j.partition ? `${j.partition}${j.pending_verify ? ' (pending verify)' : ''}` : '–'],
    ['Address', j.ip || pa.target], ['Left amp', amp(j.left)], ['Right amp', amp(j.right)], ['Rate', j.sps ? `${j.sps} S/s` : '–'],
    ['Scale', j.counts_per_N ? `${(+j.counts_per_N).toFixed(0)} counts/N (${j.full_scale_kg} kg, ${j.rated_uV_per_V} µV/V)` : '–'],
    ['Pins', j.pins ? `L dat ${j.pins.left_dat} clk ${j.pins.left_clk} · R dat ${j.pins.right_dat} clk ${j.pins.right_clk}` : '–'],
    ['Zero tracking', j.zero_tracking ? 'on' : 'off']];
  $('paInfo').innerHTML = rows.map(([k, v]) => `<tr><td class="ax-label">${k}</td><td>${v}</td></tr>`).join('');
}

// ---------------------------------------------------------------- board settings
function boardParamsState() {
  const on = !!(pa.link && pa.boardParams);
  for (const id of ['paBoardSave', 'paBoardDefaults', 'paBoardReload']) $(id).disabled = !on;
  const st = $('paBoardState');
  if (!on) { st.className = 'ax-status'; st.textContent = ''; return; }
  const dirty = paramsDirty(pa.boardParams, pa.boardLoaded);
  st.className = 'ax-status ' + (dirty ? 'warning' : 'good'); st.textContent = dirty ? 'Unsaved: live until a reboot' : 'Saved';
}
async function loadBoardParams(fn = 'params') {
  pa.boardParams = null;
  try {
    const j = await pa.link[fn]();
    pa.boardParams = j.params; if (fn === 'params') pa.boardLoaded = snapshot(j.params);
    renderParams($('paBoardParams'), pa.boardParams, (k, v) => pa.link.setParam(k, v), boardParamsState);
  } catch (e) {
    // Today's board firmware: Kconfig only (INTERFACE.md, "Parameters").
    $('paBoardParams').innerHTML = '<div class="empty">This board\'s firmware has no runtime settings yet: they are compile-time (<code>idf.py menuconfig</code> → Push assist). The params requests are agreed in its <code>firmware/INTERFACE.md</code>; once a build serves them, they show up here.</div>';
  }
  boardParamsState();
}
$('paBoardReload').onclick = () => loadBoardParams();
$('paBoardDefaults').onclick = async () => { if (confirm('Put the board\'s settings back to their defaults? They stay unsaved until you press Save.')) await loadBoardParams('defaultParams'); };
$('paBoardSave').onclick = async () => {
  try { await pa.link.saveParams(); pa.boardLoaded = snapshot(pa.boardParams); toast('Settings saved on the push-assist board', 'ok'); } catch (e) { toast('Save: ' + e.message, 'err'); }
  boardParamsState();
};

// ---------------------------------------------------------------- live view
// A store adapter that adds the preview (push_to_twist with the current gains) as channels.
const paStore = {
  window(names, seconds) {
    const real = names.map(n => n.startsWith('pa.') ? 'left_N' : n);
    const w = pa.link.store.window([...real, 'left_N', 'right_N'], seconds), n = names.length;
    const L = w[n + 1], R = w[n + 2];
    names.forEach((name, i) => {
      if (!name.startsWith('pa.')) return;
      w[i + 1] = L.map((l, k) => { if (l === null || R[k] === null) return null; const t = pushToTwist(l, R[k], pa.gains); return name === 'pa.linear' ? t.linear : t.angular; });
    });
    return w.slice(0, n + 1);
  },
};
const paCharts = [];
function paChart(el, series, yLabel) { const c = new LiveChart(el, series, yLabel, 20); charts.splice(charts.indexOf(c), 1); paCharts.push(c); return c; } // not on the MIB's store
{
  const c = SERIES();
  paChart($('cPaForce'), [{ ch: 'left_N', label: 'Left', color: c[0] }, { ch: 'right_N', label: 'Right', color: c[1] }], 'Force, N');
  paChart($('cPaTwist'), [{ ch: 'pa.linear', label: 'Linear', color: c[0] }, { ch: 'pa.angular', label: 'Angular', color: c[2] }], 'Normalized, −1 to 1');
  paChart($('cPaZero'), [{ ch: 'left_zero_N', label: 'Left', color: c[0] }, { ch: 'right_zero_N', label: 'Right', color: c[1] }], 'Correction, N');
}

const TRACK = ['idle', 'tracking', 'at cap: tare'];
function paTiles() {
  const s = pa.link && pa.link.store, g = s && s.count ? n => s.get(n) : () => NaN;
  const l = g('left_N'), r = g('right_N'), t = pushToTwist(l, r, pa.gains);
  const tile = (id, v, d = 1) => { $(id).firstChild.textContent = fmt(v, d); };
  tile('paL', l); tile('paR', r); tile('paSum', g('sum_N')); tile('paDiff', g('diff_N'));
  const track = k => { const v = g(k); return Number.isFinite(v) ? TRACK[v] || '–' : '–'; };
  $('paLSub').innerHTML = Number.isFinite(l) || !s ? `zero ${track('left_tracking')}` : '<span class="ax-status critical">no amp</span>';
  $('paRSub').innerHTML = Number.isFinite(r) || !s ? `zero ${track('right_tracking')}` : '<span class="ax-status critical">no amp</span>';
  const ok = Number.isFinite(l) && Number.isFinite(r);
  tile('paLin', ok ? t.linear * pa.gains.max_lin_mps : NaN, 2); tile('paAng', ok ? t.angular * pa.gains.max_ang_rps : NaN, 2);
  const src = link && pa.mibParams ? 'MIB gains' : 'default gains';
  $('paLinSub').textContent = ok ? `${Math.round(t.linear * 100)} % · ${src}` : '–';
  $('paAngSub').textContent = ok ? `${Math.round(t.angular * 100)} % · ${t.angular > 0.01 ? 'left' : t.angular < -0.01 ? 'right' : 'straight'}` : '–';
}

// ---------------------------------------------------------------- handle map
// Left force across, right force up. The dead band is a square; full speed / full turn is
// where push_to_twist() first scales its output down.
const MAP = { n: 100, size: 400, pad: 30 };
function drawMap(trail = []) {
  const { n, size, pad } = MAP, g = pa.gains, k = (size - 2 * pad) / (2 * n), X = f => size / 2 + f * k, Y = f => size / 2 - f * k;
  // The saturation outline: along each ray from the origin, the force where scale first hits 1.
  const sat = (l, r) => {
    const b = f => Math.abs(f) <= g.dead_band_n ? 0 : f - Math.sign(f) * g.dead_band_n, L = b(l), R = b(r);
    return Math.max(Math.abs(Math.max((L + R) / g.forward_full_n, -g.reverse_max)), Math.abs(L - R) / g.turn_full_n);
  };
  const pts = [];
  for (let i = 0; i < 180; i++) {
    const a = (i / 180) * 2 * Math.PI, c = Math.cos(a), s = Math.sin(a); let lo = 0, hi = n * 1.5;
    if (sat(hi * c, hi * s) < 1) { pts.push(null); continue; }
    for (let it = 0; it < 30; it++) { const m = (lo + hi) / 2; if (sat(m * c, m * s) >= 1) hi = m; else lo = m; }
    pts.push([X(hi * c), Y(hi * s)]);
  }
  let path = ''; pts.forEach((p, i) => { if (!p) return; path += (i && pts[i - 1] ? 'L' : 'M') + p[0].toFixed(1) + ' ' + p[1].toFixed(1); });
  const db = g.dead_band_n, ticks = [-n, -n / 2, 0, n / 2, n];
  const trailPath = trail.length ? 'M' + trail.map(([l, r]) => `${X(Math.max(-n, Math.min(n, l))).toFixed(1)} ${Y(Math.max(-n, Math.min(n, r))).toFixed(1)}`).join('L') : '';
  const last = trail[trail.length - 1];
  $('paMap').innerHTML = `<svg class="pmap" viewBox="0 0 ${size} ${size}">
    ${ticks.map(t => `<line x1="${X(t)}" y1="${pad}" x2="${X(t)}" y2="${size - pad}" stroke="${css('grid')}" stroke-dasharray="2 4"/><line x1="${pad}" y1="${Y(t)}" x2="${size - pad}" y2="${Y(t)}" stroke="${css('grid')}" stroke-dasharray="2 4"/>
      <text x="${X(t)}" y="${size - pad + 14}" text-anchor="middle">${t}</text><text x="${pad - 6}" y="${Y(t) + 3}" text-anchor="end">${t}</text>`).join('')}
    <line x1="${X(-n)}" y1="${Y(-n)}" x2="${X(n)}" y2="${Y(n)}" stroke="${css('axis')}" stroke-dasharray="4 4"/>
    <rect x="${X(-db)}" y="${Y(db)}" width="${2 * db * k}" height="${2 * db * k}" fill="${css('off')}" fill-opacity="0.35"/>
    <path d="${path}" fill="none" stroke="${css('accent')}" stroke-width="1.5"/>
    ${trailPath ? `<path d="${trailPath}" fill="none" stroke="${css('s1')}" stroke-width="1.5" stroke-opacity="0.6"/>` : ''}
    ${last ? `<circle cx="${X(Math.max(-n, Math.min(n, last[0])))}" cy="${Y(Math.max(-n, Math.min(n, last[1])))}" r="5" fill="${css('s1')}"/>` : ''}
    <text x="${size / 2}" y="${size - 4}" text-anchor="middle">left handle, N (push +)</text>
    <text x="10" y="${size / 2}" text-anchor="middle" transform="rotate(-90 10 ${size / 2})">right handle, N (push +)</text>
    <text x="${X(n) - 4}" y="${Y(n) + 14}" text-anchor="end">even push</text>
  </svg>`;
}
drawMap();

function paLoop() {
  if (document.getElementById('tab-push').classList.contains('on')) {
    paTiles();
    if (pa.link && pa.link.store.count) {
      for (const c of paCharts) c.update(paStore);
      const w = pa.link.store.window(['left_N', 'right_N'], 5);
      drawMap(w[0].map((_, i) => [w[1][i], w[2][i]]).filter(([l, r]) => l !== null && r !== null));
    }
  }
  setTimeout(() => requestAnimationFrame(paLoop), 100);
}
paLoop();
renderTargets(); // the simulated push-assist board is a Firmware tab target from the start
