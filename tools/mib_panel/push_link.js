// push_link.js: the push-assist board (rammp-org/space-bar-push-assist) as the panel sees it.
// Protocol: that repo's firmware/INTERFACE.md. The board serves espp stream_frame on tcp :5555
// (one client at a time: a new one replaces the old), answers "PUSH?" on udp :5556, and has
// Telemetry (3) and a push_assist module (0x30). OTA (0) and the params requests (0x03-0x06)
// are agreed there but not built yet; the panel says so when the board does not answer them.
'use strict';

const M_PUSH = 0x30;
const PUSHP = {
  TARE: 0x01, GET_INFO: 0x02, GET_PARAMS: 0x03, SET_PARAM: 0x04, SAVE_PARAMS: 0x05, DEFAULT_PARAMS: 0x06,
  TARED: 0x81, INFO: 0x82, PARAMS: 0x83, PARAM: 0x84, SAVED: 0x85, DEFAULTS: 0x86, ERROR: 0x8F,
};

// The MIB's push-assist gains at their defaults: components/PushAssist/include/push_assist.hpp
// kDefaultGains. Used for the preview while the MIB is not connected.
const PUSH_DEFAULT_GAINS = {
  dead_band_n: 10, forward_full_n: 60, turn_full_n: 30, reverse_max: 0.3,
  max_lin_mps: 0.35, max_ang_rps: 1.0, lin_acc_mps2: 0.5, ang_acc_rps2: 1.0,
};

// Handle forces (N, push +) to {linear, angular} in [-1, 1], angular counter-clockwise +.
// A port of push_to_twist() in components/PushAssist/include/push_assist.hpp; keep the two
// the same (tools/mib_panel/test/push_to_twist.test.js runs the C++ host test's cases).
function pushToTwist(left, right, g) {
  if (!Number.isFinite(left) || !Number.isFinite(right)) return { linear: 0, angular: 0 };
  const band = f => Math.abs(f) <= g.dead_band_n ? 0 : f - Math.sign(f) * g.dead_band_n;
  const l = band(left), r = band(right);
  let linear = (l + r) / g.forward_full_n;
  const angular = -(l - r) / g.turn_full_n;
  linear = Math.max(linear, -g.reverse_max);
  const scale = Math.max(1, Math.abs(linear), Math.abs(angular));
  return { linear: linear / scale, angular: angular / scale };
}

// A live link to the board: its telemetry into a store, plus tare, info and params.
class PushLink extends MibLink {
  async info() { const f = await this.request(M_PUSH, PUSHP.GET_INFO, null, [PUSHP.INFO]); return JSON.parse(this.dec.decode(f.payload)); }
  async tare() {
    const f = await this.request(M_PUSH, PUSHP.TARE, null, [PUSHP.TARED, PUSHP.ERROR], 8000);
    if (f.type === PUSHP.ERROR) throw new Error(this.dec.decode(f.payload.subarray(1)));
    return { left: !!f.payload[0], right: !!f.payload[1] };
  }
  async params() { return paramsCall(this, M_PUSH, PUSHP.GET_PARAMS, null, PUSHP.PARAMS, PUSHP.ERROR); }
  async setParam(key, value) { return paramsCall(this, M_PUSH, PUSHP.SET_PARAM, paramPayload(key, value), PUSHP.PARAM, PUSHP.ERROR); }
  async saveParams() { return paramsCall(this, M_PUSH, PUSHP.SAVE_PARAMS, null, PUSHP.SAVED, PUSHP.ERROR); }
  async defaultParams() { return paramsCall(this, M_PUSH, PUSHP.DEFAULT_PARAMS, null, PUSHP.DEFAULTS, PUSHP.ERROR); }
}

// A short-lived link for the Firmware tab: OTA, and info from GET_INFO (the board has no
// system module).
class PushFrameLink extends FrameLink {
  async info() { const f = await this.request(M_PUSH, PUSHP.GET_INFO, null, [PUSHP.INFO]); return JSON.parse(this.dec.decode(f.payload)); }
}

// ---------------------------------------------------------------- simulated board
// What the simulated board's flash holds, kept across its (simulated) reboots.
const SIM_PUSH_FLASH = { version: 'sim-0.2', partition: 'ota_0', pending: false, previous: null, params: null };

// The board as INTERFACE.md describes it, built and agreed parts both, so the tab can be
// tried without hardware. The handle forces are a scripted attendant: rest, an even push, a
// right-hand turn, a pull, with tremor and a slow zero drift on the left cell.
class SimPushBoard {
  constructor(out) {
    this.out = out; this.parser = new SfParser(false); this.t0 = performance.now(); this.streaming = false;
    this.channels = ['left_N', 'right_N', 'sum_N', 'diff_N', 'left_counts', 'right_counts', 'left_zero_N', 'right_zero_N', 'left_tracking', 'right_tracking', 'board_C'];
    this.defs = [
      { key: 'invert_left', label: 'Invert left cell', unit: '', group: 'Cells', type: 'bool', default: 0, min: 0, max: 1, step: 1 },
      { key: 'invert_right', label: 'Invert right cell', unit: '', group: 'Cells', type: 'bool', default: 0, min: 0, max: 1, step: 1 },
      { key: 'full_scale_kg', label: 'Cell full scale', unit: 'kg', group: 'Cells', default: 20, min: 1, max: 500, step: 1 },
      { key: 'rated_uv_per_v', label: 'Cell rated output', unit: 'µV/V', group: 'Cells', default: 1000, min: 100, max: 5000, step: 10 },
      { key: 'tare_samples', label: 'Tare samples', unit: '', group: 'Cells', default: 16, min: 1, max: 255, step: 1 },
      { key: 'zero_tracking', label: 'Zero tracking', unit: '', group: 'Zero tracking', type: 'bool', default: 1, min: 0, max: 1, step: 1 },
      { key: 'zt_band_n', label: 'Track below', unit: 'N', group: 'Zero tracking', default: 2, min: 0, max: 20, step: 0.1 },
      { key: 'zt_still_std_n', label: 'Still if std under', unit: 'N', group: 'Zero tracking', default: 0.05, min: 0.001, max: 1, step: 0.005 },
      { key: 'zt_rate_n_per_s', label: 'Max zero rate', unit: 'N/s', group: 'Zero tracking', default: 0.005, min: 0, max: 0.1, step: 0.001 },
      { key: 'zt_max_n', label: 'Max correction', unit: 'N', group: 'Zero tracking', default: 10, min: 0, max: 50, step: 0.5 },
    ];
    this.values = SIM_PUSH_FLASH.params ? { ...SIM_PUSH_FLASH.params } : Object.fromEntries(this.defs.map(d => [d.key, d.default]));
    this.drift = 0; this.zero = [0, 0];
  }
  now() { return (performance.now() - this.t0) / 1000; }
  start() { this.timer = setInterval(() => this.emit(), 100); } // 10 S/s, as the HX711s with RATE low
  stop() { clearInterval(this.timer); }
  send(module, type, payload) { this.out(sfBuild(module, type, payload, true)); }
  json(module, type, obj) { this.send(module, type, new TextEncoder().encode(JSON.stringify(obj))); }
  paramJson(d) { return { ...d, type: d.type || 'float', value: this.values[d.key] }; }
  info() {
    return { product: 'push_assist', project: 'push_assist', firmware: SIM_PUSH_FLASH.version, built: new Date().toISOString().slice(0, 16).replace('T', ' '),
      partition: SIM_PUSH_FLASH.partition, pending_verify: SIM_PUSH_FLASH.pending, ip: '127.0.0.1 (simulated)', left: true, right: true,
      counts_per_N: 10949.12, full_scale_kg: this.values.full_scale_kg, rated_uV_per_V: this.values.rated_uv_per_v, sps: 10,
      pins: { left_dat: 20, left_clk: 21, right_dat: 4, right_clk: 5 }, invert: { left: !!this.values.invert_left, right: !!this.values.invert_right }, zero_tracking: !!this.values.zero_tracking };
  }
  receive(bytes) { for (const f of this.parser.feed(bytes).frames) this.handle(f); }
  handle(f) {
    const p = f.payload, dv = new DataView(p.buffer, p.byteOffset, p.byteLength);
    if (f.module === M_TM) {
      if (f.type === TM.GET_SCHEMA) this.sendSchema();
      if (f.type === TM.SET_STREAM) { this.streaming = !!p[0]; if (this.streaming) this.sendSchema(); }
      return;
    }
    if (f.module === M_OTA) return this.handleOta(f, p, dv);
    if (f.module !== M_PUSH) return;
    const err = why => this.send(M_PUSH, PUSHP.ERROR, cat(u8(f.type), new TextEncoder().encode(why)));
    switch (f.type) {
      case PUSHP.TARE: setTimeout(() => { this.drift = 0; this.zero = [0, 0]; this.send(M_PUSH, PUSHP.TARED, u8(1, 1)); }, 1600); return;
      case PUSHP.GET_INFO: return this.json(M_PUSH, PUSHP.INFO, this.info());
      case PUSHP.GET_PARAMS: return this.json(M_PUSH, PUSHP.PARAMS, { params: this.defs.map(d => this.paramJson(d)) });
      case PUSHP.SET_PARAM: {
        const n = p[0], key = new TextDecoder().decode(p.subarray(1, 1 + n)), v = dv.getFloat32(1 + n, true), d = this.defs.find(x => x.key === key);
        if (!d) return err('unknown param');
        this.values[key] = d.type === 'bool' ? (v ? 1 : 0) : Math.min(d.max, Math.max(d.min, v));
        return this.json(M_PUSH, PUSHP.PARAM, this.paramJson(d));
      }
      case PUSHP.SAVE_PARAMS: SIM_PUSH_FLASH.params = { ...this.values }; return this.send(M_PUSH, PUSHP.SAVED, u8(1));
      case PUSHP.DEFAULT_PARAMS: for (const d of this.defs) this.values[d.key] = d.default; return this.json(M_PUSH, PUSHP.DEFAULTS, { params: this.defs.map(d => this.paramJson(d)) });
      default: return err('unknown request');
    }
  }
  // OTA as espp OtaService answers it, enough for the Firmware tab: the board reboots into
  // the other slot pending verify, and the link drops.
  handleOta(f, p, dv) {
    const ok = () => this.send(M_OTA, OTA.OK, new Uint8Array(0));
    const err = (code, msg) => this.send(M_OTA, OTA.ERROR, cat(u8(code, 0, 0, 0), new TextEncoder().encode(msg)));
    const reboot = () => setTimeout(() => { this.stop(); this.onReboot && this.onReboot(); }, 750);
    switch (f.type) {
      case OTA.BEGIN: this.ota = { size: dv.getUint32(0, true), got: 0 }; return setTimeout(ok, 200);
      case OTA.DATA: if (!this.ota) return err(1, 'no session'); this.ota.got += p.length; if (this.ota.got === p.length) this.ota.head = p.slice(0, 160); return ok();
      case OTA.FINISH: {
        if (!this.ota || this.ota.got !== this.ota.size) return err(2, `got ${this.ota ? this.ota.got : 0} of ${this.ota ? this.ota.size : 0} bytes`);
        const version = new TextDecoder().decode(this.ota.head.subarray(48, 80)).split('\0')[0];
        SIM_PUSH_FLASH.previous = { version: SIM_PUSH_FLASH.version, partition: SIM_PUSH_FLASH.partition };
        SIM_PUSH_FLASH.version = version || 'sim-new'; SIM_PUSH_FLASH.partition = SIM_PUSH_FLASH.partition === 'ota_0' ? 'ota_1' : 'ota_0'; SIM_PUSH_FLASH.pending = true;
        this.ota = null; ok(); return reboot();
      }
      case OTA.ABORT: this.ota = null; return ok();
      case OTA.GET_STATUS: { const e = new TextEncoder(), v = e.encode(SIM_PUSH_FLASH.version), pr = e.encode('push_assist');
        return this.send(M_OTA, OTA.STATUS, cat(u8((SIM_PUSH_FLASH.pending ? 1 : 0) | (SIM_PUSH_FLASH.previous ? 2 : 0), v.length), v, u8(pr.length), pr)); }
      case OTA.MARK_VALID: SIM_PUSH_FLASH.pending = false; return ok();
      case OTA.MARK_INVALID: if (SIM_PUSH_FLASH.previous) Object.assign(SIM_PUSH_FLASH, SIM_PUSH_FLASH.previous, { pending: false, previous: null }); ok(); return reboot();
    }
  }
  sendSchema() {
    const enc = new TextEncoder(), parts = [u8(1, 0, this.channels.length)];
    for (const n of this.channels) { const b = enc.encode(n); parts.push(u8(0, b.length), b); }
    this.send(M_TM, TM.SCHEMA, cat(...parts));
  }
  // The attendant: a 16 s loop.
  hands(t) {
    const k = t % 16, ramp = (a, b) => Math.min(1, Math.max(0, (k - a) / (b - a)));
    let l = 0, r = 0;
    if (k >= 2 && k < 7) { const s = ramp(2, 3.5) * (1 - ramp(6, 7)); l = r = 45 * s; }       // even push
    else if (k >= 8 && k < 11) { const s = ramp(8, 9) * (1 - ramp(10, 11)); l = 20 * s; r = 55 * s; } // right harder: left turn
    else if (k >= 12 && k < 14) { const s = ramp(12, 12.5) * (1 - ramp(13.5, 14)); l = r = -30 * s; } // pull
    const tremor = () => (Math.random() - 0.5) * 0.6;
    return [l + (l ? tremor() : 0), r + (r ? tremor() : 0)];
  }
  emit() {
    if (!this.streaming) return;
    const t = this.now(); this.drift += 0.0016 * 0.1; // N per sample: about 0.1 N/min on the left cell
    let [l, r] = this.hands(t);
    l += this.drift; const zt = this.values.zero_tracking && Math.abs(l) < this.values.zt_band_n;
    if (zt) this.zero[0] = Math.min(this.values.zt_max_n, this.zero[0] + Math.min(this.values.zt_rate_n_per_s * 0.1, this.drift - this.zero[0]));
    l -= this.zero[0];
    if (this.values.invert_left) l = -l; if (this.values.invert_right) r = -r;
    const noise = () => (Math.random() - 0.5) * 0.01;
    l += noise(); r += noise();
    const capped = this.zero[0] >= this.values.zt_max_n;
    const v = [l, r, l + r, l - r, l * 10949, r * 10949, this.zero[0], this.zero[1], capped ? 2 : zt ? 1 : 0, 0, 34 + 0.5 * Math.sin(t / 60)];
    const p = new Uint8Array(4 + 4 * v.length), dv = new DataView(p.buffer);
    dv.setUint32(0, Math.round(t * 1e6) >>> 0, true); v.forEach((x, i) => dv.setFloat32(4 + 4 * i, x, true));
    this.send(M_TM, TM.SAMPLE, p);
  }
}

class SimPushTransport {
  constructor() { this.kind = 'sim'; this.sim = null; this.onData = null; }
  async open() {
    this.sim = new SimPushBoard(bytes => this.onData && this.onData(bytes));
    this.sim.onReboot = () => this.close();
    this.sim.start();
  }
  name() { return 'Simulated push-assist board'; }
  async send(bytes) { if (!this.sim) throw new Error('link closed'); this.sim.receive(bytes); }
  async readLoop(onData) { this.onData = onData; await new Promise(res => { this.closed = res; }); }
  async close() { if (this.sim) this.sim.stop(); this.sim = null; if (this.closed) this.closed(); }
}
