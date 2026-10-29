// mib_link.js: the RAMMP MIB service link from the browser.
//
// espp stream_frame codec, three transports (WebUSB on the MIB's J3 HS port, Web Serial
// on its USB-C console, and the in-page simulator), and MibLink: telemetry decoding,
// request/reply, and the MIB control module. Numbers mirror
// components/MibSystem/include/mib_panel_protocol.hpp and components/ServiceLink.
'use strict';

// ---------------------------------------------------------------- stream_frame codec
// From the PACE RACER dashboard (pace-racer-fw atos-drive/tools/dashboard.html).
const SF_HEADER = 9, SF_CRC = 4, SF_CORR = 2, SF_MAX_PAYLOAD = 4096;
const SF_REQUEST = 0x10, SF_REPLY_BIT = 0x01, SF_CORR_BIT = 0x02;
const CRC_T = (() => { const t = new Uint32Array(256); for (let i = 0; i < 256; i++) { let c = i; for (let k = 0; k < 8; k++) c = (c & 1) ? ((c >>> 1) ^ 0xEDB88320) : (c >>> 1); t[i] = c >>> 0; } return t; })();
const crc32 = b => { let c = 0xFFFFFFFF; for (let i = 0; i < b.length; i++) c = CRC_T[(c ^ b[i]) & 0xFF] ^ (c >>> 8); return (~c) >>> 0; };

function sfBuild(module, type, payload, reply = false) {
  payload = payload || new Uint8Array(0);
  const f = new Uint8Array(SF_HEADER + payload.length + SF_CRC), v = new DataView(f.buffer);
  v.setUint16(0, 0x4F54, true); f[2] = SF_REQUEST | (reply ? SF_REPLY_BIT : 0); f[3] = module; f[4] = type; v.setUint32(5, payload.length, true);
  f.set(payload, SF_HEADER); v.setUint32(SF_HEADER + payload.length, crc32(f.subarray(0, SF_HEADER + payload.length)), true);
  return f;
}

// Splits a byte stream into frames. With `text` set, bytes that are not part of a valid
// frame come back as text: on the UART the MIB's log lines and frames share one wire.
class SfParser {
  constructor(text = false) { this.text = text; this.buf = new Uint8Array(0); }
  feed(chunk) {
    const m = new Uint8Array(this.buf.length + chunk.length); m.set(this.buf, 0); m.set(chunk, this.buf.length);
    const frames = [], textParts = []; let pos = 0, textStart = 0;
    const flushText = upto => { if (this.text && upto > textStart) textParts.push(m.subarray(textStart, upto)); };
    for (;;) {
      while (pos < m.length && !(m[pos] === 0x54 && (pos + 1 >= m.length || m[pos + 1] === 0x4F))) pos++;
      if (m.length - pos < 3) break;
      const flags = m[pos + 2];
      if ((flags & 0xF0) !== SF_REQUEST) { pos++; continue; }
      const ext = (flags & SF_CORR_BIT) ? SF_CORR : 0, hs = SF_HEADER + ext;
      if (m.length - pos < hs) break;
      const view = new DataView(m.buffer, m.byteOffset + pos), len = view.getUint32(5 + ext, true);
      if (len > SF_MAX_PAYLOAD) { pos++; continue; }
      const total = hs + len + SF_CRC; if (m.length - pos < total) break;
      if (crc32(m.subarray(pos, pos + hs + len)) !== view.getUint32(hs + len, true)) { pos++; continue; }
      flushText(pos);
      frames.push({ module: m[pos + 3], type: m[pos + 4], reply: (flags & SF_REPLY_BIT) !== 0, payload: m.slice(pos + hs, pos + hs + len) });
      pos += total; textStart = pos;
    }
    // Text up to a possible frame start is final; keep the rest for the next chunk.
    let keep = pos;
    if (this.text) {
      if (keep >= m.length) { flushText(m.length); keep = m.length; }
      else { flushText(keep); }
      textStart = keep;
    }
    this.buf = m.slice(keep);
    // A false frame start in plain text holds the text back until it resolves; cap it.
    if (this.text && this.buf.length > SF_MAX_PAYLOAD + 64) { textParts.push(this.buf.subarray(0, 1)); this.buf = this.buf.slice(1); }
    return { frames, text: textParts };
  }
}

// ---------------------------------------------------------------- transports
const MIB_USB = { vendorId: 0x1209, productId: 0x0d42 };

class UsbTransport {
  constructor(device) { this.device = device || null; this.kind = 'usb'; this.reading = false; }
  async open() {
    if (!navigator.usb) throw new Error('WebUSB is not available in this browser (use Chrome or Edge; on a phone, Chrome for Android)');
    if (!this.device) this.device = await navigator.usb.requestDevice({ filters: [MIB_USB, { vendorId: MIB_USB.vendorId }] });
    await this.device.open();
    if (this.device.configuration === null) await this.device.selectConfiguration(1);
    let found = null;
    for (const itf of this.device.configuration.interfaces) for (const alt of itf.alternates) {
      if (alt.interfaceClass !== 0xFF) continue;
      let inEp = null, outEp = null;
      for (const ep of alt.endpoints) { if (ep.type !== 'bulk') continue; if (ep.direction === 'in' && !inEp) inEp = ep; else if (ep.direction === 'out' && !outEp) outEp = ep; }
      if (inEp && outEp) { found = { itf: itf.interfaceNumber, alt: alt.alternateSetting, inEp, outEp }; break; }
    }
    if (!found) throw new Error('no vendor interface with a bulk IN+OUT pair');
    await this.device.claimInterface(found.itf);
    if (found.alt !== 0) await this.device.selectAlternateInterface(found.itf, found.alt);
    this.iface = found.itf; this.epIn = found.inEp.endpointNumber; this.epOut = found.outEp.endpointNumber;
  }
  name() { return `WebUSB ${this.device.productName || ''} ${this.device.serialNumber || ''}`.trim(); }
  async send(bytes) {
    let sent = 0;
    while (sent < bytes.length) {
      const out = await this.device.transferOut(this.epOut, sent === 0 ? bytes : bytes.subarray(sent));
      if (out.status !== 'ok') throw new Error('USB OUT ' + out.status);
      const n = out.bytesWritten || 0; if (!n) throw new Error('USB OUT made no progress'); sent += n;
    }
  }
  async readLoop(onData) {
    this.reading = true;
    while (this.reading && this.device) {
      let r;
      try { r = await this.device.transferIn(this.epIn, 16384); } catch (e) { if (this.reading) throw e; return; }
      if (!this.reading) return;
      if (r.status === 'stall') { try { await this.device.clearHalt('in', this.epIn); } catch (e) {} continue; }
      if (r.data && r.data.byteLength) onData(new Uint8Array(r.data.buffer, r.data.byteOffset, r.data.byteLength));
    }
  }
  async close() { this.reading = false; if (this.device) { try { await this.device.releaseInterface(this.iface); } catch (e) {} try { await this.device.close(); } catch (e) {} } this.device = null; }
}

// The MIB's USB-C is a CH343 USB-UART bridge on the console UART. Its DTR/RTS drive the
// board's EN / BOOT auto-reset circuit, so both are held deasserted while connected.
class SerialTransport {
  constructor(port) { this.port = port || null; this.kind = 'serial'; this.reader = null; this.writer = null; this.baud = 115200; this.reopening = null; }
  async open(baud = 115200) {
    if (!navigator.serial) throw new Error('Web Serial is not available in this browser (use desktop Chrome or Edge)');
    if (!this.port) this.port = await navigator.serial.requestPort({ filters: [{ usbVendorId: 0x1a86 }] });
    await this.port.open({ baudRate: baud, bufferSize: 65536 }); this.baud = baud;
    try { await this.port.setSignals({ dataTerminalReady: false, requestToSend: false }); } catch (e) {}
    this.writer = this.port.writable.getWriter();
  }
  name() { return `Web Serial (USB-C console, ${this.baud} baud)`; }
  async send(bytes) { if (this.reopening) await this.reopening; if (!this.writer) throw new Error('link closed'); await this.writer.write(bytes); }
  async readLoop(onData) {
    for (;;) {
      if (this.reopening) await this.reopening;
      if (!this.port || !this.port.readable) return;
      this.reader = this.port.readable.getReader();
      try {
        for (;;) { const { value, done } = await this.reader.read(); if (done) break; if (value) onData(value); }
      } catch (e) { if (!this.reopening) throw e; }
      finally { try { this.reader.releaseLock(); } catch (e) {} this.reader = null; }
      if (!this.reopening) return;
    }
  }
  // Close and reopen at another rate; the read loop waits and carries on.
  async setBaud(baud) {
    if (baud === this.baud || !this.port) return;
    let done; this.reopening = new Promise(r => { done = r; });
    try {
      try { if (this.reader) await this.reader.cancel(); } catch (e) {}
      while (this.reader) await new Promise(r => setTimeout(r, 5));
      try { this.writer.releaseLock(); } catch (e) {} this.writer = null;
      await this.port.close();
      await this.open(baud);
    } finally { this.reopening = null; done(); }
  }
  // A hardware reset through the auto-reset circuit, the way idf.py monitor does it.
  async hardReset() {
    await this.port.setSignals({ dataTerminalReady: false, requestToSend: true });
    await new Promise(r => setTimeout(r, 120));
    await this.port.setSignals({ dataTerminalReady: false, requestToSend: false });
  }
  async close() {
    const p = this.port; this.port = null;
    try { if (this.reader) await this.reader.cancel(); } catch (e) {}
    try { if (this.writer) { this.writer.releaseLock(); } } catch (e) {}
    try { if (p) await p.close(); } catch (e) {}
  }
}

// Ethernet: the MIB's tcp :5555 service stream through tools/mib_panel/eth_bridge.py, which
// relays a WebSocket (a page cannot open TCP). `bridge` is the bridge's ws:// base.
class WsTransport {
  constructor(bridge, target) { this.bridge = bridge; this.target = target; this.kind = 'eth'; this.ws = null; this.onData = null; this.onClose = null; }
  async open() {
    const url = `${this.bridge.replace(/\/$/, '')}/ws?target=${encodeURIComponent(this.target)}`;
    await new Promise((res, rej) => {
      const ws = new WebSocket(url); ws.binaryType = 'arraybuffer';
      ws.onopen = () => { this.ws = ws; res(); };
      ws.onerror = () => rej(new Error(`no MIB at ${this.target} through the bridge at ${this.bridge}. Is tools/mib_panel/eth_bridge.py running, and is this laptop on the chair's network?`));
      ws.onmessage = ev => this.onData && this.onData(new Uint8Array(ev.data));
      ws.onclose = () => { const was = this.ws; this.ws = null; if (was && this.onClose) this.onClose(); };
    });
  }
  name() { return `Ethernet ${this.target} (bridge ${this.bridge.replace(/^ws:\/\//, '')})`; }
  async send(bytes) { if (!this.ws) throw new Error('link closed'); this.ws.send(bytes); }
  async readLoop(onData) { this.onData = onData; await new Promise(res => { this.onClose = res; if (!this.ws) res(); }); }
  async close() { const ws = this.ws; this.ws = null; if (ws) ws.close(); if (this.onClose) this.onClose(); }
}

// In-page MIB (mib_sim.js): the same frames, no hardware.
class SimTransport {
  constructor() { this.kind = 'sim'; this.sim = null; this.onData = null; }
  async open() {
    this.sim = new SimMib(bytes => this.onData && this.onData(bytes));
    this.sim.onReboot = () => this.close(); // the link drops, as it does on hardware
    this.sim.start();
  }
  name() { return 'Simulated MIB'; }
  async send(bytes) { if (!this.sim) throw new Error('link closed'); this.sim.receive(bytes); }
  async readLoop(onData) { this.onData = onData; await new Promise(res => { this.closed = res; }); }
  async close() { if (this.sim) this.sim.stop(); this.sim = null; if (this.closed) this.closed(); }
}

// ---------------------------------------------------------------- protocol numbers
const M_OTA = 0, M_TM = 3, M_CON = 0x10, M_SYS = 0x20, M_MIB = 0x40;
// espp OtaService (module 0), as the PACE RACER dashboard speaks it.
const OTA = { BEGIN: 1, DATA: 2, FINISH: 3, ABORT: 4, OK: 5, ERROR: 6, PROGRESS: 7, GET_STATUS: 8, MARK_VALID: 9, MARK_INVALID: 10, STATUS: 11 };
const TM = { GET_SCHEMA: 0x01, SET_STREAM: 0x02, SCHEMA: 0x81, SAMPLE: 0x82, OK: 0x83, ERROR: 0x84 };
const SYS = { RESET: 0x01, INFO: 0x02, SET_BAUD: 0x03, OK: 0x81, INFO_REPLY: 0x82 };
const SERIAL_FAST_BAUD = 921600;
const MIBP = {
  OVERRIDE: 0x01, RELEASE: 0x02, TUNING: 0x03, CLEAR_FAULT: 0x04, DRIVE: 0x05, STICK: 0x06,
  SEAT: 0x07, ESTOP: 0x08, STATUS: 0x09, OK: 0x81, STATUS_REPLY: 0x89, ERROR: 0x8F, ALL: 0xFF,
};
const REQ_STATE = { DISARMED: 0, ARMED: 1, SAFE_STOP: 2, ESTOP: 3 };
const MODE = { COAST: 0, TORQUE: 1, VELOCITY: 2, POSITION: 3, HOLD: 4 };
const MIB_STATE = ['INIT', 'IDLE', 'CALIBRATE', 'DRIVE_ENABLED', 'ERROR'];
const BOARD_STATE = ['DISARMED', 'ARMED', 'SAFE_STOPPING', 'HOLDING', 'FAULT'];
const PROFILES = ['LOW', 'NORMAL', 'HIGH'];
// rammp-rtps RAMMP_AXIS_TABLE and RAMMP_SEAT_AXIS_TABLE (raw units are 10^-decimals unit).
const AXES = [
  { id: 0, segment: 'front_caster_left', label: 'Front caster L' },
  { id: 1, segment: 'front_caster_right', label: 'Front caster R' },
  { id: 2, segment: 'drive_left', label: 'Drive L' },
  { id: 3, segment: 'drive_right', label: 'Drive R' },
];
const SEAT_AXES = [
  { id: 0, key: 'fb_tilt', label: 'FB tilt', min: -45, max: 90, step: 2.5, unit: 'deg' },
  { id: 1, key: 'lat_tilt', label: 'Side tilt', min: -30, max: 30, step: 2.5, unit: 'deg' },
  { id: 2, key: 'elevation', label: 'Elevation', min: 0, max: 250, step: 5, unit: 'mm' },
  { id: 3, key: 'translation', label: 'Translation', min: 0, max: 50, step: 2.5, unit: 'mm' },
];

const f32s = (...vals) => { const b = new Uint8Array(vals.length * 4), v = new DataView(b.buffer); vals.forEach((x, i) => v.setFloat32(i * 4, x, true)); return b; };
const cat = (...parts) => { const n = parts.reduce((a, p) => a + p.length, 0), o = new Uint8Array(n); let k = 0; for (const p of parts) { o.set(p, k); k += p.length; } return o; };
const u8 = (...vals) => Uint8Array.from(vals);

// ---------------------------------------------------------------- telemetry store
// A ring of samples per channel. Time is the device clock (u32 microseconds, unwrapped).
class TelemetryStore {
  constructor(capacity = 6000) { this.capacity = capacity; this.reset([]); }
  reset(names) {
    this.names = names; this.index = new Map(names.map((n, i) => [n, i]));
    this.t = new Float64Array(this.capacity); this.cols = names.map(() => new Float32Array(this.capacity));
    this.head = 0; this.count = 0; this.lastRaw = null; this.wrap = 0; this.latest = names.map(() => NaN); this.rate = 0; this._rateT = [];
  }
  push(tsUs, values) {
    if (this.lastRaw !== null && tsUs < this.lastRaw && this.lastRaw - tsUs > 2 ** 31) this.wrap += 2 ** 32;
    this.lastRaw = tsUs; const t = (tsUs + this.wrap) / 1e6;
    const i = this.head; this.t[i] = t; for (let c = 0; c < this.cols.length; c++) { this.cols[c][i] = values[c]; this.latest[c] = values[c]; }
    this.head = (i + 1) % this.capacity; this.count = Math.min(this.count + 1, this.capacity);
    const now = performance.now(); this._rateT.push(now); while (this._rateT.length && now - this._rateT[0] > 2000) this._rateT.shift(); this.rate = this._rateT.length / 2;
  }
  get(name) { const i = this.index.get(name); return i === undefined ? NaN : this.latest[i]; }
  has(name) { return this.index.has(name); }
  now() { return this.count ? this.t[(this.head - 1 + this.capacity) % this.capacity] : 0; }
  // Columns over the last `seconds`, oldest first: [t, ...cols]
  window(names, seconds) {
    const out = [[], ...names.map(() => [])]; if (!this.count) return out;
    const tEnd = this.now(), idx = names.map(n => this.index.get(n));
    for (let k = this.count - 1; k >= 0; k--) {
      const i = (this.head - this.count + k + this.capacity) % this.capacity;
      if (tEnd - this.t[i] > seconds) { break; }
      out[0].push(this.t[i] - tEnd);
      idx.forEach((ci, j) => out[j + 1].push(ci === undefined ? null : (Number.isFinite(this.cols[ci][i]) ? this.cols[ci][i] : null)));
    }
    out.forEach(a => a.reverse()); return out;
  }
  // Raw samples since device time t0 (seconds), for test analysis.
  since(t0, names) {
    const rows = []; const idx = names.map(n => this.index.get(n));
    for (let k = 0; k < this.count; k++) {
      const i = (this.head - this.count + k + this.capacity) % this.capacity;
      if (this.t[i] < t0) continue;
      rows.push([this.t[i], ...idx.map(ci => ci === undefined ? NaN : this.cols[ci][i])]);
    }
    return rows;
  }
}

// ---------------------------------------------------------------- MibLink
class MibLink {
  constructor(transport, handlers = {}) {
    this.t = transport; this.h = handlers; this.parser = new SfParser(transport.kind === 'serial');
    this.waiters = []; this.store = new TelemetryStore(); this.dec = new TextDecoder(); this.open = false;
    this.fast = false; this.lastFrame = 0;
  }
  async connect() {
    await this.t.open(); this.open = true;
    this.t.readLoop(d => this.onBytes(d))
      .catch(e => this.h.onLog && this.h.onLog(`link read error: ${e.message}\n`, 'err'))
      .finally(() => { if (this.open) { this.open = false; clearInterval(this.watch); this.failAll('link closed'); this.h.onClose && this.h.onClose(true); } });
    if (this.t.kind === 'serial') {
      await this.negotiateBaud();
      // A MIB that rebooted is back at 115200 and cannot read us: drop back and ask again.
      this.watch = setInterval(async () => {
        if (!this.open || this.busy) return;
        // Still slow but the MIB answers (it was booting when we asked): ask again.
        if (!this.fast && performance.now() - this.lastFrame < 1000 && performance.now() - this.lastNeg > 5000) {
          this.busy = true; await this.negotiateBaud(); this.busy = false; return;
        }
        if (!this.fast || performance.now() - this.lastFrame < 2500) return;
        this.busy = true; this.fast = false;
        this.h.onLog && this.h.onLog('[panel] no frames for 2.5 s: back to 115200 baud\n', 'sys');
        try { await this.t.setBaud(115200); this.parser = new SfParser(true); await new Promise(r => setTimeout(r, 300)); await this.negotiateBaud(); await this.send(M_TM, TM.GET_SCHEMA); } catch (e) {}
        this.busy = false;
      }, 500);
    }
    await this.send(M_TM, TM.GET_SCHEMA);
    const p = new Uint8Array(3); p[0] = 1; new DataView(p.buffer).setUint16(1, 20, true);
    await this.send(M_TM, TM.SET_STREAM, p);
  }
  // Ask the MIB to run its console at 921600 while the panel is connected: at 115200 the
  // telemetry has to be decimated. The MIB drops back to 115200 3 s after the panel goes quiet.
  async negotiateBaud() {
    this.lastNeg = performance.now();
    try {
      const p = new Uint8Array(4); new DataView(p.buffer).setUint32(0, SERIAL_FAST_BAUD, true);
      await this.request(M_SYS, SYS.SET_BAUD, p, [SYS.OK], 1000);
      await this.t.setBaud(SERIAL_FAST_BAUD); this.parser = new SfParser(true); this.fast = true; this.lastFrame = performance.now();
      this.h.onLog && this.h.onLog(`[panel] console at ${SERIAL_FAST_BAUD} baud while connected\n`, 'sys');
    } catch (e) { this.h.onLog && this.h.onLog(`[panel] staying at 115200 baud, telemetry decimated (${e.message})\n`, 'sys'); }
  }
  async close() {
    if (!this.open) return; this.open = false; clearInterval(this.watch);
    try { const p = new Uint8Array(3); await this.send(M_TM, TM.SET_STREAM, p, true); } catch (e) {}
    if (this.fast) { try { const p = new Uint8Array(4); new DataView(p.buffer).setUint32(0, 115200, true); await this.send(M_SYS, SYS.SET_BAUD, p, true); await new Promise(r => setTimeout(r, 150)); } catch (e) {} }
    this.failAll('disconnected'); try { await this.t.close(); } catch (e) {}
    this.h.onClose && this.h.onClose(false);
  }
  failAll(why) { for (const w of this.waiters) { clearTimeout(w.timer); w.reject(new Error(why)); } this.waiters = []; }
  onBytes(d) {
    const { frames, text } = this.parser.feed(d);
    for (const t of text) this.h.onLog && this.h.onLog(this.dec.decode(t, { stream: true }));
    for (const f of frames) this.onFrame(f);
  }
  onFrame(f) {
    this.lastFrame = performance.now();
    if (f.module === M_TM && f.type === TM.SAMPLE) return this.onSamples(f.payload);
    if (f.module === M_TM && f.type === TM.SCHEMA) return this.onSchema(f.payload);
    if (f.module === M_CON && f.type === 0x82) return this.h.onLog && this.h.onLog(this.dec.decode(f.payload));
    for (const w of [...this.waiters]) if (w.module === f.module && w.types.includes(f.type)) {
      this.waiters.splice(this.waiters.indexOf(w), 1); clearTimeout(w.timer); return w.resolve(f);
    }
    if (f.module === M_MIB && f.type === MIBP.ERROR) {
      this.h.onError && this.h.onError(f.payload[0], this.dec.decode(f.payload.subarray(1)));
    }
  }
  onSchema(p) {
    const n = p[2], names = []; let o = 3;
    for (let i = 0; i < n; i++) { const len = p[o + 1]; names.push(this.dec.decode(p.subarray(o + 2, o + 2 + len))); o += 2 + len; }
    this.store.reset(names); this.h.onSchema && this.h.onSchema(names);
  }
  onSamples(p) {
    const n = this.store.names.length; if (!n) return;
    const rec = 4 + 4 * n, v = new DataView(p.buffer, p.byteOffset, p.byteLength); const vals = new Array(n);
    for (let o = 0; o + rec <= p.length; o += rec) {
      const ts = v.getUint32(o, true); for (let i = 0; i < n; i++) vals[i] = v.getFloat32(o + 4 + 4 * i, true);
      this.store.push(ts, vals);
    }
    this.h.onSample && this.h.onSample();
  }
  async send(module, type, payload, force) { if (!this.open && !force) throw new Error('not connected'); await this.t.send(sfBuild(module, type, payload)); }
  request(module, type, payload, types, timeoutMs = 1500) {
    return new Promise(async (resolve, reject) => {
      const w = { module, types, resolve, reject, timer: setTimeout(() => { this.waiters.splice(this.waiters.indexOf(w), 1); reject(new Error('no reply')); }, timeoutMs) };
      this.waiters.push(w);
      try { await this.send(module, type, payload); } catch (e) { this.waiters.splice(this.waiters.indexOf(w), 1); clearTimeout(w.timer); reject(e); }
    });
  }
  // A MIB control request that answers OK or ERROR.
  async mib(type, payload) {
    const f = await this.request(M_MIB, type, payload, [MIBP.OK, MIBP.ERROR]);
    if (f.type === MIBP.ERROR) throw new Error(this.dec.decode(f.payload.subarray(1)));
    return f.payload[1];
  }
  // One OTA request; PROGRESS frames are not waited for.
  async ota(type, payload, timeoutMs = 3000) {
    const f = await this.request(M_OTA, type, payload, [OTA.OK, OTA.ERROR, OTA.STATUS], timeoutMs);
    if (f.type === OTA.ERROR) { const v = new DataView(f.payload.buffer, f.payload.byteOffset); throw new Error(`MIB OTA error ${v.getUint32(0, true)}: ${this.dec.decode(f.payload.subarray(4))}`); }
    return f;
  }
  async otaStatus() {
    const p = (await this.ota(OTA.GET_STATUS)).payload; let i = 1;
    const rs = () => { if (i >= p.length) return ''; const n = p[i++]; const s = this.dec.decode(p.subarray(i, i + n)); i += n; return s; };
    return { pending: !!(p[0] & 1), rollback: !!(p[0] & 2), version: rs(), project: rs() };
  }
  async info() { const f = await this.request(M_SYS, SYS.INFO, null, [SYS.INFO_REPLY]); return JSON.parse(this.dec.decode(f.payload)); }
  async status() { const f = await this.request(M_MIB, MIBP.STATUS, null, [MIBP.STATUS_REPLY]); return JSON.parse(this.dec.decode(f.payload)); }
  async reset() { return this.request(M_SYS, SYS.RESET, null, [SYS.OK]); }

  // Fire-and-forget streams: refused ones come back through onError.
  override(axis, { state = REQ_STATE.ARMED, mode = MODE.VELOCITY, position = 0, velocity = 0, torque = 0, torqueLimit = 0, velLimit = 0, accelLimit = 0 }) {
    return this.send(M_MIB, MIBP.OVERRIDE, cat(u8(axis, state, mode, 0), f32s(position, velocity, torque, torqueLimit, velLimit, accelLimit)));
  }
  stick(x, y) { return this.send(M_MIB, MIBP.STICK, f32s(x, y)); }
  release(axis = MIBP.ALL) { return this.mib(MIBP.RELEASE, u8(axis)); }
  tune(axis, kp, ki, pll, persist) { return this.mib(MIBP.TUNING, cat(u8(axis, persist ? 1 : 0), f32s(kp, ki, pll))); }
  clearFault(axis) { return this.mib(MIBP.CLEAR_FAULT, u8(axis)); }
  drive(enable, profile) { return this.mib(MIBP.DRIVE, u8(enable ? 1 : 0, profile)); }
  seat(axis, target) { return this.mib(MIBP.SEAT, cat(u8(axis), f32s(target))); }
  estop(engage) { return this.mib(MIBP.ESTOP, u8(engage ? 1 : 2)); }
}
