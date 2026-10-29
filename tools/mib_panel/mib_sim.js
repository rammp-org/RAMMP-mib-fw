// mib_sim.js: a simulated MIB with two PACE RACER drive wheels, for trying the panel
// without hardware. It speaks the same frames as the firmware (components/ServiceLink,
// components/MibSystem/src/mib_panel.cpp) and copies its rules: overrides only in IDLE
// with the e-stop clear, 300 ms override lapse, tuning by req id, clear-fault counters.
//
// The wheel model is a guess, not a measurement: a hub motor on blocks with
// J = 0.06 kg m^2, Kt = 0.9 N m/A, 0.35 N m Coulomb drag, a hall speed estimate behind a
// tracking loop of hall_pll_hz, and the board's PI speed loop at 1 kHz.
'use strict';

// What the simulated MIB's flash holds, kept across its (simulated) reboots.
const SIM_FLASH = { version: 'sim-1.0', partition: 'ota_0', pending: false, previous: null };

class SimMib {
  constructor(out) {
    this.out = out; this.parser = new SfParser(false); this.t0 = performance.now();
    this.channels = [
      'mib.state', 'mib.estop', 'mib.profile', 'mib.stick_x', 'mib.stick_y', 'sensor.mib_chip_temp',
      'seat.fb_tilt', 'seat.lat_tilt', 'seat.elevation', 'seat.translation',
      ...AXES.flatMap(a => ['cmd_vel', 'cmd_torque', 'vel', 'pos', 'iq', 'torque', 'vbus', 'temp', 'state', 'online'].map(n => `${a.segment}.${n}`)),
      // Not in the firmware yet: shows how a plugged-in sensor appears on the panel.
      'sensor.imu_pitch', 'sensor.imu_roll',
    ];
    this.streaming = false; this.baud = 115200; this.state = 0; this.profile = 1; this.estop = { latched: true, clearing: false, cause: 'boot', releases: 0 };
    this.seat = [0, 0, 0, 0]; this.stick = { x: 0, y: 0, until: 0 }; this.hmi = { x: 0, y: 0 };
    this.plan = { lin: 0, ang: 0 }; this.tuningReq = 0; this.cmdSeq = 0;
    this.axes = AXES.map(a => ({
      ...a, online: a.id >= 2, override: null, overrideUntil: 0, clearReq: 0, tuning: null, tuningUntil: 0, lastCmd: { state: 0, mode: 2, vel: 0, torque: 0 },
      board: { state: 0, mode: 0, fault: 'NONE', clearAck: 0, tuningAck: 0, lastCmdT: 0, kp: 0.6, ki: 3.0, pll: 8, pos: 0, w: 0, wMeas: 0, iq: 0, iqCmd: 0, integ: 0, ref: 0, temp: 28, holdPos: 0, cmd: null },
      reported: null, reportedT: -1,
    }));
  }
  now() { return (performance.now() - this.t0) / 1000; }
  start() {
    this.physics = setInterval(() => this.step(), 5);
    this.motorTimer = setInterval(() => this.publishMotorCommands(), 50);
    this.stateTimer = setInterval(() => this.reportStates(), 50);
    this.tmTimer = setInterval(() => this.emitTelemetry(), 20);
    setTimeout(() => { this.state = 1; this.log('I (1520) RAMMP_MIB: System state: INIT done, IDLE'); }, 1200);
    this.log('I (312) ServiceLink: Service link up: (simulated) | fw rammp_mib sim');
  }
  stop() { [this.physics, this.motorTimer, this.stateTimer, this.tmTimer].forEach(clearInterval); }
  send(module, type, payload) { this.out(sfBuild(module, type, payload, true)); }
  log(line) { if (this.streaming) this.send(M_CON, 0x82, new TextEncoder().encode(line + '\n')); }
  reply(type, payload) { this.send(M_MIB, type, payload); }
  ok(req, v = 0) { this.reply(MIBP.OK, u8(req, v)); }
  err(req, why) { this.reply(MIBP.ERROR, cat(u8(req), new TextEncoder().encode(why))); }

  receive(bytes) { this.lastRx = this.now(); for (const f of this.parser.feed(bytes).frames) this.handle(f); }
  handle(f) {
    const p = f.payload, dv = new DataView(p.buffer, p.byteOffset, p.byteLength), F = o => dv.getFloat32(o, true);
    if (f.module === M_TM) {
      if (f.type === TM.GET_SCHEMA) this.sendSchema();
      if (f.type === TM.SET_STREAM) { this.streaming = !!p[0]; this.send(M_TM, TM.OK, u8(TM.SET_STREAM)); if (this.streaming) this.sendSchema(); }
      return;
    }
    if (f.module === M_OTA) return this.handleOta(f, p, dv);
    if (f.module === M_SYS) {
      if (f.type === SYS.INFO) this.send(M_SYS, SYS.INFO_REPLY, new TextEncoder().encode(JSON.stringify({ product: 'RAMMP MIB (simulated)', project: 'rammp_mib', version: SIM_FLASH.version, partition: SIM_FLASH.partition, pending_verify: SIM_FLASH.pending, built: new Date().toISOString().slice(0, 16), idf: 'v6.0.1', reset_reason: 'power-on', uptime_s: Math.round(this.now()), usb: true, uart: true, ip: '192.168.4.1' })));
      // The UART's rate, for a test harness that models the serial link (SimTransport ignores it).
      if (f.type === SYS.SET_BAUD) { this.send(M_SYS, SYS.OK, u8(SYS.SET_BAUD)); this.baud = dv.getUint32(0, true); this.lastRx = this.now(); return; }
      if (f.type === SYS.RESET) { this.send(M_SYS, SYS.OK, u8(SYS.RESET)); this.state = 0; this.estop.latched = true; this.estop.cause = 'boot'; setTimeout(() => { this.state = 1; }, 1200); }
      return;
    }
    if (f.module !== M_MIB) return;
    const now = this.now(), idle = this.state === 1 && !this.estop.latched;
    switch (f.type) {
      case MIBP.OVERRIDE: {
        const ax = this.axes[p[0]]; if (!ax) return this.err(f.type, 'no such axis');
        if (!idle) return this.err(f.type, this.estop.latched ? 'e-stop is latched' : 'MIB is not IDLE');
        const cap = (v, c) => (v <= 0 || !Number.isFinite(v)) ? c : Math.min(v, c);
        const c = { state: p[1], mode: p[2], position: F(4), velocity: F(8), torque: F(12), torqueLimit: cap(F(16), 20), velLimit: cap(F(20), 8), accelLimit: Math.max(0, F(24)) };
        c.velocity = Math.max(-c.velLimit, Math.min(c.velLimit, c.velocity)); c.torque = Math.max(-c.torqueLimit, Math.min(c.torqueLimit, c.torque));
        ax.override = c; ax.overrideUntil = now + 0.3; return;
      }
      case MIBP.RELEASE: this.axes.forEach(a => { if (p[0] === 0xFF || p[0] === a.id) a.overrideUntil = 0; }); return this.ok(f.type);
      case MIBP.TUNING: {
        const ax = this.axes[p[0]]; if (!ax) return this.err(f.type, 'no such axis');
        this.tuningReq = (this.tuningReq + 1) & 0xFF || 1;
        ax.tuning = { req: this.tuningReq, persist: p[1], kp: F(2), ki: F(6), pll: F(10) }; ax.tuningUntil = now + 2;
        this.log(`I RAMMP_MIB: Panel tuning ${ax.segment}: kp=${F(2).toFixed(4)} ki=${F(6).toFixed(4)} pll=${F(10)} (req ${this.tuningReq})`);
        return this.ok(f.type, this.tuningReq);
      }
      case MIBP.CLEAR_FAULT: { const ax = this.axes[p[0]]; ax.clearReq = (ax.clearReq + 1) & 0xFF || 1; return this.ok(f.type, ax.clearReq); }
      case MIBP.DRIVE: {
        if (p[0] === 1) {
          if (this.state !== 1 && this.state !== 3) return this.err(f.type, 'refused: see the log');
          if (this.axes.some(a => now < a.overrideUntil)) return this.err(f.type, 'refused: see the log');
          this.profile = p[1]; this.state = 3; this.log(`I RAMMP_MIB: Drive enabled with profile=${p[1]}`);
        } else { if (this.state !== 3) return this.err(f.type, 'refused: see the log'); this.state = 1; this.log('I RAMMP_MIB: Drive disabled'); }
        return this.ok(f.type);
      }
      case MIBP.STICK:
        if (this.state !== 3) return this.err(f.type, 'drive is not enabled');
        this.stick = { x: Math.max(-1, Math.min(1, F(0))), y: Math.max(-1, Math.min(1, F(4))), until: now + 0.3 }; return;
      case MIBP.SEAT:
        if (this.state !== 1) return this.err(f.type, 'refused: seat moves only in IDLE');
        this.seat[p[0]] = F(1); this.log(`I RAMMP_MIB: Seat control command applied: axis=${p[0]} target=${F(1)} - placeholder`); return this.ok(f.type);
      case MIBP.ESTOP:
        if (p[0] === 1) { this.estop.latched = true; this.estop.cause = 'panel'; this.log('W RAMMP_MIB: E-stop engaged from the control panel'); }
        else { this.estop.clearing = true; this.log('W RAMMP_MIB: E-stop clear attempt from the control panel'); setTimeout(() => { this.estop.clearing = false; this.estop.latched = false; this.estop.releases++; }, 800); }
        return this.ok(f.type, p[0]);
      case MIBP.STATUS: return this.reply(MIBP.STATUS_REPLY, new TextEncoder().encode(JSON.stringify(this.statusJson())));
      default: return this.err(f.type, 'unknown request');
    }
  }
  // espp OtaService, enough of it to exercise the panel: BEGIN, DATA, FINISH (then a
  // reboot into the new slot, pending verify), GET_STATUS, MARK_VALID, MARK_INVALID, ABORT.
  handleOta(f, p, dv) {
    const ok = () => this.send(M_OTA, OTA.OK, new Uint8Array(0));
    const err = (code, msg) => this.send(M_OTA, OTA.ERROR, cat(f32s(0).map(() => 0), new TextEncoder().encode(msg)).map((b, i) => i < 4 ? (code >> (8 * i)) & 0xFF : b));
    const reboot = () => setTimeout(() => { this.stop(); this.onReboot && this.onReboot(); }, 750);
    switch (f.type) {
      case OTA.BEGIN: this.ota = { size: dv.getUint32(0, true), got: 0 }; if (this.state === 3) this.state = 1; this.axes.forEach(a => { a.overrideUntil = 0; }); return setTimeout(ok, 300);
      case OTA.DATA: if (!this.ota) return err(1, 'no session'); this.ota.got += p.length; if (this.ota.got === p.length) this.ota.head = p.slice(0, 160); return ok();
      case OTA.FINISH: {
        if (!this.ota || this.ota.got !== this.ota.size) return err(2, `got ${this.ota ? this.ota.got : 0} of ${this.ota ? this.ota.size : 0} bytes`);
        const h = this.ota.head, cs = (a, b) => new TextDecoder().decode(h.subarray(a, b)).split('\0')[0];
        SIM_FLASH.previous = { version: SIM_FLASH.version, partition: SIM_FLASH.partition };
        SIM_FLASH.version = cs(48, 80) || 'sim-new'; SIM_FLASH.partition = SIM_FLASH.partition === 'ota_0' ? 'ota_1' : 'ota_0'; SIM_FLASH.pending = true;
        this.ota = null; ok(); return reboot();
      }
      case OTA.ABORT: this.ota = null; return ok();
      case OTA.GET_STATUS: { const e = new TextEncoder(), v = e.encode(SIM_FLASH.version), pr = e.encode('rammp_mib'); return this.send(M_OTA, OTA.STATUS, cat(u8((SIM_FLASH.pending ? 1 : 0) | (SIM_FLASH.previous ? 2 : 0), v.length), v, u8(pr.length), pr)); }
      case OTA.MARK_VALID: SIM_FLASH.pending = false; return ok();
      case OTA.MARK_INVALID: if (SIM_FLASH.previous) { Object.assign(SIM_FLASH, SIM_FLASH.previous, { pending: false, previous: null }); } ok(); return reboot();
    }
  }
  sendSchema() {
    const enc = new TextEncoder(), parts = [u8(1, 0, this.channels.length)];
    for (const n of this.channels) { const b = enc.encode(n); parts.push(u8(0, b.length), b); }
    this.send(M_TM, TM.SCHEMA, cat(...parts));
  }
  statusJson() {
    const now = this.now(), st = this.state === 1 && this.estop.latched ? 4 : this.state;
    return {
      state: MIB_STATE[st], profile: this.profile, ethernet: true, ip: '192.168.4.1', rtps: this.state !== 0,
      estop: { latched: this.estop.latched, clearing: this.estop.clearing, cause: this.estop.cause, releases: this.estop.releases },
      panel_stick: now < this.stick.until, seat: this.seat,
      axes: this.axes.map(a => {
        const s = a.reported, online = a.online && s && now - a.reportedT < 0.5;
        return { axis: a.id, segment: a.segment, label: a.label, online: !!online, age_ms: s ? Math.round((now - a.reportedT) * 1000) : -1,
          api: s ? 3 : 0, fw: s ? '0.4.0' : '0.0.0', state: s ? BOARD_STATE[s.state] : 'DISARMED', mode: s ? s.mode : 0, fault: s ? s.fault : 'NONE', drv_status: 0,
          cmd_age_ms: s ? 12 : 0, uptime_ms: Math.round(now * 1000), vbus: s ? 24.1 : 0, temp_max: s ? s.temp : 0, limits: [20, 8, 0],
          tuning: { ack: s ? s.tuningAck : 0, req: a.tuning ? a.tuning.req : 0, kp: s ? s.kp : 0, ki: s ? s.ki : 0, pll: s ? s.pll : 0 },
          override: now < a.overrideUntil, clear_req: a.clearReq, clear_ack: s ? s.clearAck : 0, cmd: { state: a.lastCmd.state, mode: a.lastCmd.mode, vel: a.lastCmd.vel } };
      }),
    };
  }
  // The MIB's 20 Hz command task.
  publishMotorCommands() {
    const now = this.now(), st = this.state, drive = st === 3 && !this.estop.latched, idle = st === 1 && !this.estop.latched;
    if (st === 1 && this.estop.latched) { /* firmware: ERROR while latched */ }
    // planner: stick -> chassis velocity with the profile's limits
    const prof = [{ v: 0.35, w: 1.0, a: 0.5, aw: 1.0 }, { v: 0.75, w: 2.0, a: 1.0, aw: 2.0 }, { v: 1.0, w: 3.14, a: 2.0, aw: 4.0 }][this.profile];
    const live = now < this.stick.until, sx = live ? this.stick.x : this.hmi.x, sy = live ? this.stick.y : this.hmi.y;
    const tl = drive ? Math.max(-prof.v, Math.min(prof.v, sy)) : 0, ta = drive ? Math.max(-prof.w, Math.min(prof.w, sx)) : 0;
    const dt = 0.05, toward = (c, t, r) => c + Math.max(-r * dt, Math.min(r * dt, t - c));
    this.plan.lin = toward(this.plan.lin, tl, prof.a); this.plan.ang = toward(this.plan.ang, ta, prof.aw);
    const r = 0.127, half = 0.558 / 2, wl = (this.plan.lin - this.plan.ang * half) / r, wr = -(this.plan.lin + this.plan.ang * half) / r;
    for (const a of this.axes) {
      let c;
      if (idle && now < a.overrideUntil) c = { ...a.override };
      else if (drive && (a.id === 2 || a.id === 3)) c = { state: 1, mode: 2, velocity: a.id === 2 ? wl : wr, torque: 0, position: 0, torqueLimit: 0, velLimit: 0, accelLimit: 0 };
      else c = { state: 0, mode: 2, velocity: 0, torque: 0, position: 0, torqueLimit: 0, velLimit: 0, accelLimit: 0 };
      c.clearReq = a.clearReq; a.lastCmd = { state: c.state, mode: c.mode, vel: c.velocity, torque: c.torque };
      if (a.online) this.boardCommand(a, c, now);
      if (a.tuning && now < a.tuningUntil && a.board.tuningAck !== a.tuning.req) this.boardTuning(a, a.tuning);
    }
  }
  boardCommand(a, c, now) {
    const b = a.board; b.lastCmdT = now;
    if (c.clearReq !== b.clearAck) { b.clearAck = c.clearReq; if (b.state === 4) { b.state = 0; b.fault = 'NONE'; } }
    if (b.state === 4) return;
    const prevMode = b.mode;
    if (c.state === 1) { b.state = 1; b.mode = c.mode; } else if (c.state === 2) { b.state = Math.abs(b.w) > 0.05 ? 2 : 3; } else { b.state = 0; b.mode = 0; }
    if (b.mode !== prevMode) { b.integ = 0; b.ref = b.wMeas; b.holdPos = b.pos; }
    b.cmd = c;
  }
  boardTuning(a, t) {
    const b = a.board; b.tuningAck = t.req;
    if (t.kp > 0) b.kp = t.kp; else if (t.kp < 0) b.kp = 0;
    if (t.ki > 0) b.ki = t.ki; else if (t.ki < 0) { b.ki = 0; b.integ = 0; }
    if (t.pll > 0) b.pll = t.pll;
  }
  step() {
    const now = this.now(); if (!this.lastStep) this.lastStep = now; let dtAll = Math.min(0.05, now - this.lastStep); this.lastStep = now;
    const J = 0.06, Kt = 0.9, drag = 0.35, visc = 0.02, h = 0.001;
    if (this.state === 3 && this.estop.latched) this.state = 4;
    if (this.state === 4 && !this.estop.latched) this.state = 1;
    if (this.state === 1 && this.estop.latched) this.state = 4;
    while (dtAll > 1e-6) {
      const dt = Math.min(h, dtAll); dtAll -= dt;
      for (const a of this.axes) {
        if (!a.online) continue; const b = a.board, c = b.cmd;
        if (b.state !== 4 && b.state !== 0 && now - b.lastCmdT > 0.2) { b.state = 4; b.fault = 'WATCHDOG'; }
        let iqCmd = 0;
        const iMax = c ? c.torqueLimit > 0 ? c.torqueLimit / Kt : 20 / Kt : 0;
        const speedLoop = target => {
          const acc = c.accelLimit > 0 ? c.accelLimit : 1e9; b.ref += Math.max(-acc * dt, Math.min(acc * dt, target - b.ref));
          const e = b.ref - b.wMeas; b.integ += e * dt * (b.ki > 0 ? 1 : 0); if (b.ki <= 0) b.integ = 0;
          let i = b.kp * e + b.ki * b.integ;
          if (Math.abs(i) > iMax) { b.integ -= e * dt; i = Math.sign(i) * iMax; }
          return i;
        };
        if (b.state === 1 && c) {
          const vl = c.velLimit > 0 ? c.velLimit : 8;
          if (b.mode === 2) iqCmd = speedLoop(Math.max(-vl, Math.min(vl, c.velocity)));
          else if (b.mode === 1) { iqCmd = c.torque / Kt; const over = Math.abs(b.wMeas) / vl; if (over > 0.8 && Math.sign(iqCmd) === Math.sign(b.wMeas)) iqCmd *= Math.max(0, (1 - over) / 0.2); }
          else if (b.mode === 3) iqCmd = speedLoop(Math.max(-vl, Math.min(vl, 4 * (c.position - b.pos))));
          else if (b.mode === 4) iqCmd = speedLoop(Math.max(-vl, Math.min(vl, 4 * (b.holdPos - b.pos))));
        } else if (b.state === 2 || b.state === 3) iqCmd = speedLoop(0);
        b.iq += (iqCmd - b.iq) * Math.min(1, dt / 0.002);
        const tauDrag = Math.abs(b.w) < 1e-3 && Math.abs(Kt * b.iq) < drag ? Kt * b.iq : drag * Math.sign(b.w || b.iq);
        b.w += (Kt * b.iq - tauDrag - visc * b.w) / J * dt; if (Math.abs(b.w) < 1e-3 && Math.abs(Kt * b.iq) < drag) b.w = 0;
        b.pos += b.w * dt;
        // hall speed estimate: a tracking loop at hall_pll_hz on a 60-step/rev signal
        const quant = Math.round(b.w / (2 * Math.PI / 60 * 2)) * (2 * Math.PI / 60 * 2) * 0.5 + b.w * 0.5;
        b.wMeas += (quant - b.wMeas) * Math.min(1, 2 * Math.PI * b.pll * dt);
        b.temp += (0.002 * b.iq * b.iq - (b.temp - 28) * 0.01) * dt;
      }
    }
  }
  reportStates() {
    const now = this.now();
    for (const a of this.axes) {
      if (!a.online) continue; const b = a.board;
      a.reported = { state: b.state, mode: b.mode, fault: b.fault, clearAck: b.clearAck, tuningAck: b.tuningAck, kp: b.kp, ki: b.ki, pll: b.pll,
        vel: b.wMeas + (Math.random() - 0.5) * 0.02, pos: b.pos, iq: b.iq + (Math.random() - 0.5) * 0.05, torque: 0.9 * b.iq, vbus: 24.1 - 0.05 * Math.abs(b.iq), temp: b.temp };
      a.reportedT = now;
    }
  }
  emitTelemetry() {
    if (this.baud !== 115200 && this.now() - (this.lastRx || 0) > 3) this.baud = 115200;
    if (!this.streaming) return;
    const now = this.now(), st = (this.state === 1 && this.estop.latched) ? 4 : this.state, live = now < this.stick.until;
    const v = [st, this.estop.latched ? 1 : 0, this.profile, live ? this.stick.x : 0, live ? this.stick.y : 0, 41 + Math.sin(now / 30), ...this.seat];
    for (const a of this.axes) {
      const s = a.reported, on = a.online && s && now - a.reportedT < 0.5;
      v.push(a.lastCmd.vel, a.lastCmd.torque, on ? s.vel : NaN, on ? s.pos : NaN, on ? s.iq : NaN, on ? s.torque : NaN, on ? s.vbus : NaN, on ? s.temp : NaN, on ? s.state : NaN, on ? 1 : 0);
    }
    v.push(1.2 + 0.3 * Math.sin(now * 0.7) + 0.05 * (this.axes[2].board.w - this.axes[3].board.w), -0.4 + 0.2 * Math.sin(now * 0.31));
    const p = new Uint8Array(4 + 4 * v.length), dv = new DataView(p.buffer);
    dv.setUint32(0, Math.round(now * 1e6) >>> 0, true); v.forEach((x, i) => dv.setFloat32(4 + 4 * i, x, true));
    this.send(M_TM, TM.SAMPLE, p);
  }
}
