"""A simulated chair that writes the logger's wire format, for tests and dry runs.

Per axis (brushed DC motor + lead screw, an MCP266 channel):
  command  -> CAN latency -> applied duty (what the controller reads back)
  applied  -> mechanical dead time -> drive duty u
  from rest the axis moves only once |u| exceeds the breakaway duty; once moving,
  speed follows  v_ss = K_dir * (|u| - u_kin_dir)  with time constant tau and
  stops when the drive falls under u_kin (stiction with hysteresis). K, u_kin,
  u_break and the load current differ by direction (gravity).
  current  I = +-I_load + G * (applied - u_bemf), u_bemf the duty that sustains v
  encoder  motor-side position, quantised to counts; the load follows through backlash.
Chassis: a rigid plane least-squares fitted through the four supports (drive
wheels left/right, front casters, rear casters), each lifted by its actuator
with a mild nonlinearity. The IMU sits on that plane at a mounting rotation and
reports gravity vector, quaternion, gyro and linear acceleration, late by the
fusion delay. Sensor noise, CAN staleness, empty fields, dropped rows, garbage
bytes, truncated rows, stray and #log lines and a reordered header are injected.

Truth for every parameter an analysis estimates is returned alongside the bytes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict

import numpy as np
from scipy.signal import butter, filtfilt

from .attitude import rpy_to_R

G0 = 9.80665
DT = 0.001


@dataclass
class Axis:
    name: str
    node: int
    ch: int
    # EHA: + duty raises the corner (pump against the load); the load helps the - direction
    gain_pos: float = 5.0  # counts/s per % duty above u_kin, + duty (up, against gravity)
    gain_neg: float = 5.8
    ukin_pos: float = 6.2  # % duty under which a moving axis stops
    ukin_neg: float = 4.6
    ubrk_pos: float = 9.4  # % duty needed to start from rest
    ubrk_neg: float = 7.3
    tau: float = 0.12  # s, pressure build-up lag
    dead_mech: float = 0.040  # s, applied duty to first motion
    iload_pos: float = 1.6  # A while moving up at steady state
    iload_neg: float = 0.5
    g_elec: float = 0.10  # A per % of duty not balanced by back-EMF
    i_limit: float = 12.0
    backlash: float = 0.0  # counts, encoder to chassis
    enc_min: float = 412.0
    enc_max: float = 3710.0
    enc_start: float | None = None
    mm_per_count: float = 0.0364
    nonlin: float = 0.03
    support: tuple | None = None  # (x, y) m in the chassis frame, None = not a base support
    creep: float = 0.0  # counts/s while at rest (an EHA leaking down under load)

    @property
    def prefix(self):
        return f"n{self.node}c{self.ch}"

    @property
    def mid(self):
        return 0.5 * (self.enc_min + self.enc_max)


def default_axes() -> list[Axis]:
    # nodes as the firmware defaults them: 1 = casters, 2 = carriages, 3 = main legs
    return [
        Axis("FC", 1, 1, 5.0, 5.8, 6.2, 4.6, 9.4, 7.3, 0.10, 0.035, 1.6, 0.5, backlash=6, support=(0.45, 0.0),
             creep=-0.010),
        Axis("RC", 1, 2, 4.6, 5.4, 6.6, 5.1, 10.3, 7.7, 0.12, 0.040, 1.8, 0.6, backlash=4, support=(-0.40, 0.0),
             creep=-0.015),
        Axis("LCarr", 2, 1, 6.0, 6.0, 4.0, 4.0, 6.0, 6.0, 0.08, 0.030, 0.6, 0.6),
        Axis("RCarr", 2, 2, 6.0, 6.0, 4.0, 4.0, 6.0, 6.0, 0.08, 0.030, 0.6, 0.6),
        Axis("ML", 3, 1, 4.2, 5.2, 7.4, 5.6, 11.3, 8.2, 0.14, 0.045, 2.6, 0.8, backlash=8, support=(0.0, 0.28),
             creep=-0.030),
        Axis("MR", 3, 2, 4.3, 5.1, 7.1, 5.4, 10.8, 8.6, 0.13, 0.045, 2.5, 0.8, backlash=8, support=(0.0, -0.28),
             creep=-0.025),
    ]


@dataclass
class Sim:
    axes: list[Axis] = field(default_factory=default_axes)
    row_hz: float = 100.0
    can_hz: float = 50.0
    can_latency: float = 0.008
    imu_delay: float = 0.015
    mount_rpy: tuple = (1.2, -2.0, 0.0)
    quat_yaw: float = 37.0
    imu_pos: tuple = (0.05, 0.0, 0.45)  # m, chassis frame; the seat
    noise: bool = True
    seed: int = 1
    t_us0: int = 8_123_456
    jitter_us: float = 150.0
    drop_prob: float = 0.002
    gap_at_s: float | None = 0.37  # fraction of the run where a 0.25 s USB stall drops rows
    empty_prob: float = 0.002
    missed_read_prob: float = 0.01
    stray_per_min: float = 4.0
    log_per_min: float = 4.0
    garbage_per_min: float = 2.0
    truncate_per_min: float = 1.0
    header_every_s: float = 10.0
    reorder_header: bool = True
    max_field_age_s: float = 0.5
    vbat: float = 25.2
    sd_grav: float = 0.01  # m/s^2 (the BNO055's fused gravity output, not raw accel)
    sd_gyro: float = 0.003  # rad/s
    sd_la: float = 0.02  # m/s^2
    sd_quat: float = 0.0008
    sd_cur_ma: float = 40.0
    sd_spd: float = 1.5
    sd_enc: float = 0.25  # counts before rounding
    gyro_bias: tuple = (0.002, -0.0015, 0.001)
    external_drive: bool = False  # excitation from the controller's own USB: _cmd stays 0, no step events
    kp: float = 0.15  # controller position loop: % duty per count of error
    ki: float = 0.0  # % duty per count*s (0 = P only, as the controllers are set today)
    loop_duty_cap: float = 40.0
    creep_scale: float = 1.0  # 0 disables creep at rest
    duty_scale: float = 1.0  # multiply scenario duties (above 1 needs the firmware's duty cap raised)


@dataclass
class Cmd:
    prefix: str
    t: float
    profile: list  # [(duty %, duration s), ...] contiguous, then 0
    kind: str = "step"


@dataclass
class Mark:
    t: float
    text: str


# --------------------------------------------------------------------------- scenarios

BASE = ("FC", "RC", "ML", "MR")


def _axis(sim, name):
    return next(a for a in sim.axes if a.name == name)


def scenario(name: str, sim: Sim) -> tuple[list, float]:
    """(schedule, duration s). Duties stay within the firmware's 20 % cap, steps within 1.5 s,
    unless sim.duty_scale raises them."""
    sch, dur = _scenario(name, sim)
    # commands arrive asynchronously to the firmware's CAN poll: walk their phase
    k = 0
    for c in sch:
        if isinstance(c, Cmd):
            c.t = round(c.t + (k * 0.0071) % 0.02, 3)
            k += 1
    if sim.duty_scale != 1.0:
        for c in sch:
            if isinstance(c, Cmd) and c.kind not in ("target", "pstep"):
                c.profile = [(d * sim.duty_scale, t) for d, t in c.profile]
    return sch, dur


def _scenario(name: str, sim: Sim) -> tuple[list, float]:
    sch: list = []
    t = 2.0
    if name in ("passive", "level"):
        sch.append(Mark(1.0, name))
        return sch, 20.0
    if name == "steps":  # open loop (legacy: needs a duty command the firmware does not have)
        for ax in BASE:
            p = _axis(sim, ax).prefix
            sch.append(Mark(t - 0.5, f"steps {ax}"))
            for d in (12, -12, 15, -15, 18, -18, 20, -20):
                sch.append(Cmd(p, t, [(d, 1.5)]))
                t += 3.5
        return sch, t + 1.0
    if name == "deadband":  # open loop (legacy)
        for ax in BASE:
            p = _axis(sim, ax).prefix
            sch.append(Mark(t - 0.5, f"deadband {ax}"))
            for m in range(3, 15):
                for s in (1, -1):
                    sch.append(Cmd(p, t, [(s * m, 1.0)]))
                    t += 1.8
            for s in (1, -1):
                prof = [(s * 20, 0.8)] + [(s * d, 1.0) for d in (14, 12, 10, 9, 8, 7, 6, 5, 4)]
                sch.append(Cmd(p, t, prof, "staircase"))
                t += sum(d for _, d in prof) + 1.5
        return sch, t + 1.0
    if name == "kinematics":
        for ax in BASE:
            p = _axis(sim, ax).prefix
            sch.append(Mark(t - 0.5, f"kinematics {ax}"))
            for d in [150] * 3 + [-150] * 6 + [150] * 3:
                sch.append(Cmd(p, t, [(d, 2.5)], "pstep"))
                t += 3.5
            t += 1.0
        return sch, t + 1.0
    if name in ("pstep", "latency"):
        for ax in BASE:
            p = _axis(sim, ax).prefix
            sch.append(Mark(t - 0.5, f"pstep {ax}"))
            for d in (200, -200, 120, -120, 80, -80, 65, -65, 50, -50, 30, -30):
                sch.append(Cmd(p, t, [(d, 3.5)], "pstep"))
                t += 4.5
        return sch, t + 1.0
    if name == "rest":
        sch.append(Mark(1.0, "rest: drive released, nobody on the chair"))
        return sch, 240.0
    if name in ("profile_fast", "profile_slow"):
        # the same moves under two controller tunings (scenario sets the loop gains)
        sim.kp, sim.loop_duty_cap = (0.4, 40.0) if name == "profile_fast" else (0.1, 15.0)
        p = _axis(sim, "FC").prefix
        sch.append(Mark(1.5, f"{name} kp={sim.kp} cap={sim.loop_duty_cap}"))
        for _ in range(3):
            for d in (250, -250):
                sch.append(Cmd(p, t, [(d, 5.0)], "pstep"))
                t += 6.0
        return sch, t + 1.0
    raise ValueError(f"unknown scenario {name}")


SCENARIOS = ("passive", "rest", "pstep", "kinematics", "profile_fast", "profile_slow", "steps", "deadband")


# --------------------------------------------------------------------------- simulation


def _target_arrays(sim: Sim, schedule, n):
    tg = {a.prefix: np.full(n, np.nan) for a in sim.axes}
    for c in schedule:
        if isinstance(c, Cmd) and c.kind == "target":
            t = c.t
            for target, dur in c.profile:
                tg[c.prefix][int(round(t / DT)):int(round((t + dur) / DT))] = target
                t += dur
    return tg


def _pstep_arrays(sim: Sim, schedule, n):
    start = {a.prefix: np.full(n, np.nan) for a in sim.axes}
    active = {a.prefix: np.zeros(n, dtype=bool) for a in sim.axes}
    for c in schedule:
        if isinstance(c, Cmd) and c.kind == "pstep":
            delta, dur = c.profile[0]
            i0, i1 = int(round(c.t / DT)), int(round((c.t + dur) / DT))
            start[c.prefix][i0] = delta
            active[c.prefix][i0:i1] = True
    return {p: (start[p], active[p]) for p in start}


def _cmd_arrays(sim: Sim, schedule, n):
    cmd = {a.prefix: np.zeros(n) for a in sim.axes}
    for c in schedule:
        if not isinstance(c, Cmd) or c.kind in ("target", "pstep"):
            continue
        t = c.t
        for duty, dur in c.profile:
            i0, i1 = int(round(t / DT)), int(round((t + dur) / DT))
            cmd[c.prefix][i0:i1] = duty
            t += dur
    return cmd


def _shift(a, k):
    if k <= 0:
        return a.copy()
    out = np.zeros_like(a)
    out[k:] = a[:-k]
    return out


def _run_axis(ax: Axis, cmd: np.ndarray, can_latency: float, target=None, kp=0.0, cap=20.0,
              pstep=None, ki=0.0, creep_scale=1.0):
    """One EHA axis at 1 kHz.

    Open loop: applied duty = command delayed by CAN latency.
    Closed loop (the controller's own position loop, P or PI):
      target   absolute target counts per ms (old 'target' scenarios), or
      pstep    (start, active): at a start index the target becomes round(x) + delta and
               holds while active; the controller sees it CAN latency after the firmware.
    Hydraulics: duty (after the mechanical dead time) builds 'pressure' q with lag tau;
    flow starts when |q| exceeds breakaway, stops when it falls under u_kin, and is
    K_dir * (|q| - u_kin) in between. Not back-drivable: no motion at zero duty except
    an optional slow creep at rest (ax.creep, counts/s).
    """
    n = len(cmd)
    nl = int(round(can_latency / DT))
    applied = _shift(cmd, nl)
    nm = int(round(ax.dead_mech / DT))
    closed_abs = target is not None and np.isfinite(target).any()
    closed_rel = pstep is not None and np.isfinite(pstep[0]).any()
    x0 = ax.mid if ax.enc_start is None else ax.enc_start
    creep = ax.creep * creep_scale
    tgt_log = np.full(n, np.nan)
    if not np.any(cmd) and not closed_abs and not closed_rel and creep == 0.0:
        z = np.zeros(n)
        return {"x": np.full(n, x0), "v": z, "y": np.full(n, x0), "I": z.copy(), "applied": applied, "lim": z.copy(),
                "moving": z.astype(bool), "target": tgt_log}
    x, v, y, q, moving = x0, 0.0, x0, 0.0, False
    a = DT / ax.tau
    Kp, Kn, ukp, ukn, ubp, ubn = ax.gain_pos, ax.gain_neg, ax.ukin_pos, ax.ukin_neg, ax.ubrk_pos, ax.ubrk_neg
    hb = 0.5 * ax.backlash
    g, ilp, iln, ilim = ax.g_elec, ax.iload_pos, ax.iload_neg, ax.i_limit
    xmin, xmax = ax.enc_min, ax.enc_max
    al = applied.tolist()
    tl = target.tolist() if closed_abs else None
    if closed_rel:
        ps_start = pstep[0].tolist()
        ps_act = pstep[1].tolist()
    ctrl_tgt = None
    fw_tgt = None
    integ = 0.0
    xo, vo, yo, Io, lo, mo, to = ([0.0] * n for _ in range(7))
    for k in range(n):
        if closed_rel:
            d = ps_start[k]
            if d == d:  # the firmware commands: target from its encoder reading
                fw_tgt = round(x) + d
            if not ps_act[k]:
                fw_tgt = None
            to[k] = fw_tgt if fw_tgt is not None else float("nan")
            kc = k - nl
            if kc >= 0:
                d2 = ps_start[kc]
                if d2 == d2:
                    ctrl_tgt = to[kc]
                    integ = 0.0
                if not ps_act[kc]:
                    ctrl_tgt = None
            if ctrl_tgt is not None and ctrl_tgt == ctrl_tgt:
                e = ctrl_tgt - round(x)
                u = kp * e + ki * integ
                if -cap < u < cap:
                    integ += e * DT
                al[k] = max(-cap, min(cap, u))
            else:
                al[k] = 0.0
        elif closed_abs and tl[k] == tl[k]:
            e = tl[k] - round(x)
            al[k] = max(-cap, min(cap, kp * e))
            to[k] = tl[k]
        else:
            to[k] = float("nan")
        u = al[k - nm] if k >= nm else 0.0
        q += (u - q) * a
        if not moving and (q > ubp or q < -ubn):
            moving = True
        if moving:
            if q > ukp:
                v = Kp * (q - ukp)
            elif q < -ukn:
                v = -Kn * (-q - ukn)
            else:
                v = 0.0
                moving = False
        else:
            v = creep
        x += v * DT
        at = 0.0
        if x > xmax:
            x, v, moving, at = xmax, 0.0, False, 1.0
        elif x < xmin:
            x, v, moving, at = xmin, 0.0, False, 1.0
        if x - y > hb:
            y = x - hb
        elif y - x > hb:
            y = x + hb
        ua = al[k]
        if moving and v > 0:
            I = ilp + g * (ua - (v / Kp + ukp))
        elif moving and v < 0:
            I = -iln + g * (ua - (v / Kn - ukn))
        else:
            I = g * ua
        I = max(-ilim, min(ilim, I))
        xo[k], vo[k], yo[k], Io[k], lo[k], mo[k] = x, v, y, I, at, moving
    return {"x": np.array(xo), "v": np.array(vo), "y": np.array(yo), "I": np.array(Io), "applied": np.array(al),
            "lim": np.array(lo), "moving": np.array(mo, dtype=bool), "target": np.array(to, dtype=float)}


def _heights(ax: Axis, y):
    d = y - ax.mid
    half = 0.5 * (ax.enc_max - ax.enc_min)
    return ax.mm_per_count * 1e-3 * (d + ax.nonlin * d * d / half)


def chassis_pose(sim: Sim, ys: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(a, pitch rad, roll rad) arrays from load positions of the base axes."""
    sup = [a for a in sim.axes if a.support is not None]
    A = np.array([[1.0, a.support[0], a.support[1]] for a in sup])
    P = np.linalg.pinv(A)
    H = np.column_stack([_heights(a, ys[a.prefix]) for a in sup])
    coef = H @ P.T
    a0, b, c = coef[:, 0], coef[:, 1], coef[:, 2]
    N = np.sqrt(1 + b * b + c * c)
    pitch = -np.arctan(b)
    roll = np.arcsin(c / N)
    return a0, pitch, roll


def _R_wc(pitch, roll):
    cp, sp, cr, sr = np.cos(pitch), np.sin(pitch), np.cos(roll), np.sin(roll)
    R = np.zeros((len(pitch), 3, 3))
    # Ry(pitch) @ Rx(roll)
    R[:, 0, 0] = cp
    R[:, 0, 1] = sp * sr
    R[:, 0, 2] = sp * cr
    R[:, 1, 1] = cr
    R[:, 1, 2] = -sr
    R[:, 2, 0] = -sp
    R[:, 2, 1] = cp * sr
    R[:, 2, 2] = cp * cr
    return R


def _quat(R):
    w = 0.5 * np.sqrt(np.clip(1 + R[:, 0, 0] + R[:, 1, 1] + R[:, 2, 2], 1e-12, None))
    x = (R[:, 2, 1] - R[:, 1, 2]) / (4 * w)
    y = (R[:, 0, 2] - R[:, 2, 0]) / (4 * w)
    z = (R[:, 1, 0] - R[:, 0, 1]) / (4 * w)
    return np.column_stack([w, x, y, z])


def true_jacobian(sim: Sim) -> dict:
    """d(pitch, roll)/d(count) in degrees at mid stroke for each base axis."""
    out = {}
    for ax in sim.axes:
        if ax.support is None:
            continue
        res = []
        for dy in (-1.0, 1.0):
            ys = {a.prefix: np.array([a.mid + (dy if a is ax else 0.0)]) for a in sim.axes}
            _, p, r = chassis_pose(sim, ys)
            res.append((math.degrees(p[0]), math.degrees(r[0])))
        out[ax.prefix] = {"pitch": (res[1][0] - res[0][0]) / 2, "roll": (res[1][1] - res[0][1]) / 2}
    return out


def simulate(scenario_name: str, sim: Sim | None = None, schedule=None, duration=None):
    """Returns (raw bytes, truth dict, hires dict)."""
    sim = sim or Sim()
    rng = np.random.default_rng(sim.seed)
    if schedule is None:
        schedule, duration = scenario(scenario_name, sim)
    n = int(round(duration / DT))
    tt = np.arange(n) * DT
    cmds = _cmd_arrays(sim, schedule, n)
    tgts = _target_arrays(sim, schedule, n)
    pst = _pstep_arrays(sim, schedule, n)
    st = {a.prefix: _run_axis(a, cmds[a.prefix], sim.can_latency, tgts[a.prefix], sim.kp, sim.loop_duty_cap,
                              pst[a.prefix], sim.ki, sim.creep_scale)
          for a in sim.axes}

    # chassis and IMU at 1 kHz
    a0, pitch, roll = chassis_pose(sim, {p: s["y"] for p, s in st.items()})
    Rwc = _R_wc(pitch, roll)
    Rcb = rpy_to_R(*sim.mount_rpy)
    Rwb = Rwc @ Rcb
    p_imu = np.array(sim.imu_pos)
    pw = Rwc @ p_imu + np.column_stack([np.zeros(n), np.zeros(n), a0])
    acc_w = np.gradient(np.gradient(pw, DT, axis=0), DT, axis=0)
    la_b = np.einsum("nji,nj->ni", Rwb, acc_w)
    la_c = np.einsum("nji,nj->ni", Rwc, acc_w)
    g_b = G0 * Rwb[:, 2, :]
    droll = np.gradient(roll, DT)
    dpitch = np.gradient(pitch, DT)
    w_c = np.column_stack([droll, dpitch * np.cos(roll), -dpitch * np.sin(roll)])
    w_b = w_c @ Rcb  # R_cb^T w_c, row-wise
    # the BNO055 filters internally: no 1 kHz finite-difference spikes aliased into 100 Hz samples
    baa, aaa = butter(4, 40.0 / (0.5 / DT))
    la_b = filtfilt(baa, aaa, la_b, axis=0)
    w_b = filtfilt(baa, aaa, w_b, axis=0)
    Ryaw = rpy_to_R(0, 0, sim.quat_yaw)
    q_b = _quat(Ryaw @ Rwb)

    # rows
    nrow = int(duration * sim.row_hz)
    tr = np.arange(nrow) / sim.row_hz + 0.004
    jit = rng.normal(0, sim.jitter_us * 1e-6, nrow) if sim.noise else np.zeros(nrow)
    tr = np.clip(tr + jit, 0, duration - 2 * DT)
    tr.sort()
    t_us = (sim.t_us0 + tr * 1e6).astype(np.int64)

    def at(arr, times):
        idx = np.clip(np.floor(times / DT + 1e-9).astype(int), 0, n - 1)
        return arr[idx]

    ti = tr - sim.imu_delay
    nz = (lambda sd, shape: rng.normal(0, sd, shape)) if sim.noise else (lambda sd, shape: np.zeros(shape))
    imu = {}
    q = at(q_b, ti) + nz(sim.sd_quat, (nrow, 4))
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    gyro = at(w_b, ti) + nz(sim.sd_gyro, (nrow, 3)) + (np.array(sim.gyro_bias) if sim.noise else 0)
    la = at(la_b, ti) + nz(sim.sd_la, (nrow, 3))
    gr = at(g_b, ti) + nz(sim.sd_grav, (nrow, 3))
    for j, a in enumerate("wxyz"):
        imu[f"imu_q{a}"] = (q[:, j], 5)
    for j, a in enumerate("xyz"):
        imu[f"imu_g{a}"] = (gyro[:, j], 4)
    for j, a in enumerate("xyz"):
        imu[f"imu_la{a}"] = (la[:, j], 3)
    for j, a in enumerate("xyz"):
        imu[f"imu_gr{a}"] = (gr[:, j], 3)
    imu["imu_cal"] = (np.where(tr < 1.0, 63, 255).astype(float), 0)

    cols = ["t_us"] + list(imu)
    data = dict(imu)
    nodes = sorted({a.node for a in sim.axes})
    for ax in sim.axes:
        s = st[ax.prefix]
        p = ax.prefix
        period = 1.0 / sim.can_hz
        phase = rng.uniform(0, period)
        tc = np.arange(phase, duration, period)
        if sim.noise:
            tc = tc + rng.normal(0, 0.0005, len(tc))
            tc = tc[rng.random(len(tc)) > sim.missed_read_prob]
        tc = np.clip(np.sort(tc), 0, duration - DT)
        k = np.searchsorted(tc, tr, side="right") - 1
        valid = k >= 0
        kk = np.clip(k, 0, len(tc) - 1)
        tread = tc[kk]
        age = np.where(valid, (tr - tread) * 1e6, np.nan)
        valid &= age <= sim.max_field_age_s * 1e6
        enc = np.round(at(s["x"], tread) + nz(sim.sd_enc, nrow))
        spd = np.round(at(s["v"], tread) + nz(sim.sd_spd, nrow))
        cur = np.round(at(s["I"], tread) * 1000 + nz(sim.sd_cur_ma, nrow))
        duty = np.round(at(s["applied"], tread), 1)
        lim = at(s["lim"], tread)
        cmdv = np.round(at(cmds[p], tr), 1) * (0.0 if sim.external_drive else 1.0)
        tgtv = np.round(at(s["target"], tr))
        for name, arr, dec in (("enc", enc, 0), ("spd", spd, 0), ("cur_ma", cur, 0), ("duty", duty, 1),
                               ("cmd", cmdv, 1), ("tgt", tgtv, 0), ("status", np.zeros(nrow), 0), ("lim", lim, 0),
                               ("age_us", np.round(age), 0)):
            vals = arr.astype(float).copy()
            if name not in ("cmd", "tgt"):
                vals[~valid] = np.nan
            data[f"{p}_{name}"] = (vals, dec)
            cols.append(f"{p}_{name}")
    for nd in nodes:
        itot = sum(np.abs(at(st[a.prefix]["I"], tr)) for a in sim.axes if a.node == nd)
        data[f"n{nd}_vbat_mv"] = (np.round(sim.vbat * 1000 - 60 * itot + nz(15, nrow)), 0)
        data[f"n{nd}_temp_dc"] = (np.round(310 + 4 * nd + tr / 60.0), 0)
        cols += [f"n{nd}_vbat_mv", f"n{nd}_temp_dc"]

    # empty fields (never t_us)
    n_empty = 0
    if sim.noise and sim.empty_prob > 0:
        for c in cols[1:]:
            m = rng.random(nrow) < sim.empty_prob
            vals = data[c][0].copy()
            n_empty += int(np.sum(m & np.isfinite(vals)))
            vals[m] = np.nan
            data[c] = (vals, data[c][1])

    # format rows in both column orders
    alt_cols = ["t_us"] + cols[1:][::-1]
    t_switch = duration * 0.55 if (sim.reorder_header and sim.noise) else math.inf

    def fmt(vals, dec):
        if dec == 0:
            return ["" if not np.isfinite(v) else str(int(v)) for v in vals]
        f = f"{{:.{dec}f}}"
        return ["" if not np.isfinite(v) else f.format(v) for v in vals]

    strs = {c: fmt(*data[c]) for c in cols[1:]}
    strs["t_us"] = [str(int(v)) for v in t_us]
    keep = np.ones(nrow, dtype=bool)
    if sim.noise:
        keep &= rng.random(nrow) > sim.drop_prob
        if sim.gap_at_s is not None:
            g0 = sim.gap_at_s * duration
            keep &= ~((tr >= g0) & (tr < g0 + 0.25))
    lines: list[tuple[float, int, str]] = []  # (time, order, text)
    n_garbled = 0
    garble = set()
    truncate = set()
    if sim.noise:
        idx = np.flatnonzero(keep)
        ng = int(rng.poisson(sim.garbage_per_min * duration / 60))
        nt = int(rng.poisson(sim.truncate_per_min * duration / 60))
        pick = rng.choice(idx[100:], size=min(ng + nt, len(idx) - 100), replace=False)
        garble, truncate = set(pick[:ng].tolist()), set(pick[ng:].tolist())
    for i in range(nrow):
        if not keep[i]:
            continue
        order = alt_cols if tr[i] >= t_switch else cols
        body = "D," + ",".join(strs[c][i] for c in order)
        if i in garble:
            cut = rng.integers(8, len(body) - 4)
            junk = bytes(rng.integers(0x80, 0x100, rng.integers(3, 12)).tolist()).decode("latin-1")
            body = body[:cut] + junk + body[cut:]
            n_garbled += 1
        elif i in truncate:
            body = body[: len(body) // 2]
            n_garbled += 1
        lines.append((tr[i], 1, body))
    rows_written = int(keep.sum()) - n_garbled
    ok_rows = np.flatnonzero(keep)
    ok_rows = [i for i in ok_rows if i not in garble and i not in truncate]
    t0 = float(tr[ok_rows[0]])

    def hdr(t, order):
        lines.append((t, 0, "#cols:" + ",".join(order)))

    meta = [("fw", "sysid-sim 0.1"), ("sample_rate_hz", f"{sim.row_hz:g}"),
            ("nodes", ",".join(str(nd) for nd in nodes)), ("units", "t_us=us gyro=rad/s la=m/s2 gr=m/s2 enc=counts spd=counts/s cur=mA duty=% cmd=% age=us vbat=mV temp=0.1C")]
    for k, (key, val) in enumerate(meta):
        lines.append((-1e-3 + k * 1e-5, 0, f"#meta:{key}={val}"))
    lines.append((-1e-4, 0, "I (312) boot: ESP-IDF v6.0.1 2nd stage bootloader"))
    # a `dump` reply: read-only controller config
    for nd in nodes:
        for k, (name, idx, sub, val) in enumerate((("pos_kp", "0x2101", 1, int(sim.kp * 1000)),
                                                   ("pos_ki", "0x2101", 2, int(sim.ki * 1000)),
                                                   ("max_duty", "0x2102", 0, int(sim.loop_duty_cap)))):
            lines.append((0.2 + 1e-4 * k, 2, f"#cfg,{nd},{name},{idx},{sub},{val}"))
    hdr(-5e-5, cols)
    th = sim.header_every_s
    while th < duration:
        hdr(th - 1e-5, alt_cols if th >= t_switch else cols)
        th += sim.header_every_s
    if math.isfinite(t_switch):
        hdr(t_switch - 2e-5, alt_cols)

    # events
    def ev(t, text):
        lines.append((t, 2, f"#evt,{int(sim.t_us0 + t * 1e6)},{text}"))

    ev(0.5, "armed")
    steps_truth = []
    pos_truth = []
    for c in schedule:
        if isinstance(c, Mark):
            ev(c.t, f"mark {c.text}")
            continue
        ax = next(a for a in sim.axes if a.prefix == c.prefix)
        if c.kind == "pstep":
            delta, dur = c.profile[0]
            ev(c.t, f"pstep {ax.node} {ax.ch} {int(delta):+d}")
            ev(c.t + dur, f"pstep end n{ax.node}c{ax.ch}")
            pos_truth.append(_pstep_truth(sim, ax, c, st[c.prefix], t0))
            continue
        if c.kind == "target":
            for target, dur in c.profile:
                ev(c.t, f"target {ax.node} {ax.ch} {int(round(target))}")
            if c is [q for q in schedule if isinstance(q, Cmd) and q.prefix == c.prefix][-1]:
                ev(c.t + dur, f"stop n{ax.node}c{ax.ch} (position loop released)")
            pos_truth.append(_position_truth(sim, ax, c, st[c.prefix], t0))
            continue
        t = c.t
        if sim.external_drive:
            steps_truth += _step_truth(sim, ax, c, st[c.prefix], cmds[c.prefix], n, t0)
            continue
        for duty, dur in c.profile:
            ev(t, f"step {ax.node} {ax.ch} {duty:.1f} {int(round(dur * 1000))}" if c.kind != "profile"
               else f"profile {ax.node} {ax.ch} {duty:.2f}")
            t += dur
        ev(t, f"step end n{ax.node}c{ax.ch}")
        steps_truth += _step_truth(sim, ax, c, st[c.prefix], cmds[c.prefix], n, t0)
    ev(duration - 0.3, "stop")

    n_stray = n_log = 0
    if sim.noise:
        for _ in range(int(rng.poisson(sim.stray_per_min * duration / 60))):
            lines.append((rng.uniform(0, duration), 3, f"E ({rng.integers(1e5, 1e7)}) mcp266: SDO timeout node {rng.integers(1, 4)}"))
            n_stray += 1
        for _ in range(int(rng.poisson(sim.log_per_min * duration / 60))):
            lines.append((rng.uniform(0, duration), 3, "#log,W (1234) twai: bus error count 3"))
            n_log += 1
    lines.sort(key=lambda r: (r[0], r[1]))
    raw = ("\n".join(l for _, _, l in lines) + "\n").encode("latin-1")

    jac = true_jacobian(sim)
    truth = {
        "scenario": scenario_name,
        "sim": {k: v for k, v in asdict(sim).items() if k != "axes"},
        "axes": {a.prefix: asdict(a) for a in sim.axes},
        "axis_names": {a.prefix: a.name for a in sim.axes},
        "dead_total": {a.prefix: sim.can_latency + a.dead_mech for a in sim.axes},
        "jacobian_deg_per_count": jac,
        "counts": {"rows_total": nrow, "rows_written": rows_written, "rows_dropped": int(nrow - keep.sum()),
                   "rows_garbled": n_garbled, "stray_lines": n_stray + 1, "log_lines": n_log,
                   "empty_fields": n_empty, "header_changes": 1 if math.isfinite(t_switch) else 0},
        "steps": steps_truth,
        "position_steps": pos_truth,
        "rest": _rest_truth(sim, st, pitch, roll, tt, schedule),
        "t0_s": t0,
    }
    truth["smoothness"] = _smooth_truth(sim, schedule, la_c, w_c, tt, t0)
    hires = {"t": tt, "pitch": np.degrees(pitch), "roll": np.degrees(roll), "la_c": la_c, "w_c": w_c,
             "states": st, "cmds": cmds, "t0_s": t0}
    return raw, truth, hires


def _step_truth(sim, ax, c: Cmd, s, cmd, n, t0):
    out = []
    t = c.t
    prev = 0.0
    for duty, dur in c.profile:
        i0, i1 = int(round(t / DT)), int(round((t + dur) / DT))
        sgn = 1 if duty > 0 else -1
        K = ax.gain_pos if sgn > 0 else ax.gain_neg
        uk = ax.ukin_pos if sgn > 0 else ax.ukin_neg
        ub = ax.ubrk_pos if sgn > 0 else ax.ubrk_neg
        from_rest = prev == 0.0
        moves = abs(duty) > (ub if from_rest else uk)
        if not from_rest and abs(s["v"][max(i0 - 1, 0)]) == 0.0:
            moves = abs(duty) > ub
        v_ss = sgn * K * max(abs(duty) - uk, 0.0) if moves else 0.0
        iss = (ax.iload_pos if sgn > 0 else -ax.iload_neg) if moves and v_ss != 0 else ax.g_elec * duty
        last = i1 == int(round((c.t + sum(d for _, d in c.profile)) / DT))
        rec = {"prefix": ax.prefix, "t_on": t - t0, "t_off": t + dur - t0, "duty": duty, "from_rest": from_rest,
               "moves": bool(moves), "v_ss": v_ss, "i_ss_A": iss,
               "i_peak_A": float(np.max(np.abs(s["I"][i0:min(i1 + 50, n)]))) if i1 > i0 else 0.0}
        if last:
            j = i1
            while j < n - 1 and (s["moving"][j] or j < i1 + int((sim.can_latency + ax.dead_mech) / DT) + 5):
                j += 1
            rec["coast_counts"] = float(s["x"][j] - s["x"][i1])
        out.append(rec)
        prev = duty
        t += dur
    return out


def _position_truth(sim, ax, c, s, t0):
    from .stepmetrics import position_step

    target, dur = c.profile[0]
    i0, i1 = int(round(c.t / DT)), int(round((c.t + dur) / DT))
    t = np.arange(i0, i1) * DT
    x = np.round(s["x"][i0:i1])
    m = position_step(t, x, c.t, float(x[0]), target)
    tail = slice(i1 - 520, i1 - 20)  # last 0.5 s, clear of the next target's (de-phased) start
    m.update({"prefix": ax.prefix, "t_on": c.t - t0, "pwm_rest_pct": float(np.median(s["applied"][tail])),
              "i_rest_A": float(np.median(s["I"][tail])), "kp": sim.kp})
    return m


def _pstep_truth(sim, ax, c, s, t0):
    from .stepmetrics import position_step

    delta, dur = c.profile[0]
    i0, i1 = int(round(c.t / DT)), int(round((c.t + dur) / DT))
    tgt = float(s["target"][i0 + 1])
    t = np.arange(i0, i1) * DT
    x = np.round(s["x"][i0:i1])
    m = position_step(t, x, c.t, float(x[0]), tgt)
    tail = slice(i1 - 520, i1 - 20)
    mov = s["moving"][i0:i1]
    m.update({"prefix": ax.prefix, "t_on": c.t - t0, "delta": delta, "pwm_rest_pct": float(np.median(s["applied"][tail])),
              "i_rest_A": float(np.median(s["I"][tail])), "kp": sim.kp, "ki": sim.ki,
              "ever_moved": bool(mov.any())})
    return m


def _rest_truth(sim, st, pitch, roll, tt, schedule):
    """Creep and attitude drift per minute over the stretches where nothing was commanded."""
    busy = np.zeros(len(tt), dtype=bool)
    for c in schedule:
        if isinstance(c, Cmd):
            dur = sum(d for _, d in c.profile)
            busy |= (tt >= c.t) & (tt <= c.t + dur + 1.0)
    m = ~busy
    if m.sum() < 1000:
        return {}
    out = {"creep_counts_per_min": {}, "pitch_deg_per_min": None, "roll_deg_per_min": None}
    for a in sim.axes:
        out["creep_counts_per_min"][a.prefix] = float(np.polyfit(tt[m], st[a.prefix]["x"][m], 1)[0] * 60)
    out["pitch_deg_per_min"] = float(np.polyfit(tt[m], np.degrees(pitch[m]), 1)[0] * 60)
    out["roll_deg_per_min"] = float(np.polyfit(tt[m], np.degrees(roll[m]), 1)[0] * 60)
    return out


def _smooth_truth(sim, schedule, la_c, w_c, tt, t0, fc=5.0, settle=1.0):
    """RMS and peak jerk and angular acceleration per move from the noise-free 1 kHz signals."""
    b, a = butter(4, fc / (0.5 / DT))
    la = filtfilt(b, a, la_c, axis=0)
    w = filtfilt(b, a, w_c, axis=0)
    jerk = np.linalg.norm(np.gradient(la, DT, axis=0), axis=1)
    aacc = np.degrees(np.linalg.norm(np.gradient(w, DT, axis=0), axis=1))
    out = []
    for c in schedule:
        if not isinstance(c, Cmd):
            continue
        t1 = c.t + sum(d for _, d in c.profile)
        m = (tt >= c.t) & (tt <= t1 + settle)
        out.append({"t_on": c.t - t0, "t_end": t1 + settle - t0, "rms_jerk": float(np.sqrt(np.mean(jerk[m] ** 2))),
                    "peak_jerk": float(jerk[m].max()), "rms_ang_acc": float(np.sqrt(np.mean(aacc[m] ** 2))),
                    "peak_ang_acc": float(aacc[m].max())})
    return out
