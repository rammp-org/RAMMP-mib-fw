"""overview: every signal against time, events marked, plus the passive sanity checks.

Checks (each becomes a flag with a word, never just a color):
  gravity magnitude ~9.81 m/s^2, quaternion norm ~1, quaternion and gravity agree,
  BNO055 calibration bits, gyro bias at rest, CAN staleness, empty fields,
  speed readback sign vs encoder motion, duty sign vs encoder motion, battery range,
  limit flags.
"""

from __future__ import annotations

import numpy as np

from .. import attitude, plotstyle as ps, signals, timebase
from .common import header, meta_line


def compute(run, opts) -> dict:
    res = header("overview", run, opts)
    flags = res["flags"]
    tb = timebase.check(run)
    res["timebase"] = tb
    if not tb.get("ok", False):
        flags.append(f"warning: time base: {tb.get('dropped_rows_est')} rows dropped ({tb.get('dropped_pct')} %), "
                     f"jitter p99 {tb.get('jitter_p99_us')} us, {tb.get('non_monotonic')} non-monotonic")
    if tb.get("clock_resets"):
        flags.append(f"note: device clock restarted {tb['clock_resets']} time(s) (reset or replug); segments stitched")
    imu = {}
    g = attitude.imu_vec(run, "gr")
    moving = _moving_mask(run)
    if g is not None:
        gm = np.linalg.norm(g, axis=1)
        imu["gravity_norm_mean"] = float(np.nanmean(gm))
        if abs(imu["gravity_norm_mean"] - 9.81) > 0.3:
            flags.append(f"critical: gravity vector magnitude {imu['gravity_norm_mean']:.2f}, expected 9.81 m/s^2 (units?)")
    q = np.column_stack([run.col(f"imu_q{a}") for a in "wxyz"]) if run.col("imu_qw") is not None else None
    if q is not None:
        qn = np.linalg.norm(q, axis=1)
        imu["quat_norm_mean"] = float(np.nanmean(qn))
        if abs(imu["quat_norm_mean"] - 1) > 0.02:
            flags.append(f"critical: quaternion norm {imu['quat_norm_mean']:.3f}, expected 1 (scaling?)")
    conv = attitude.quat_convention(run)
    imu["quat_convention"] = conv
    if conv.get("agreement_deg") is not None and conv["agreement_deg"] > 3:
        flags.append(f"warning: quaternion and gravity vector disagree by {conv['agreement_deg']:.1f} deg")
    cal = run.col("imu_cal")
    if cal is not None and np.isfinite(cal).any():
        c = int(cal[np.isfinite(cal)][-1])
        imu["cal_last"] = c
        imu["cal_bits"] = {"sys": (c >> 6) & 3, "gyro": (c >> 4) & 3, "accel": (c >> 2) & 3, "mag": c & 3}
        if imu["cal_bits"]["gyro"] < 3 or imu["cal_bits"]["accel"] < 3:
            flags.append(f"warning: BNO055 calibration {imu['cal_bits']} (gyro/accel below 3)")
    gy = attitude.imu_vec(run, "g")
    if gy is not None:
        rest = ~moving & np.all(np.isfinite(gy), axis=1)
        if rest.sum() > 20:
            bias = np.median(gy[rest], axis=0)
            imu["gyro_bias_rad_s"] = bias.tolist()
            if np.max(np.abs(bias)) > 0.01:
                flags.append(f"warning: gyro bias at rest {np.max(np.abs(bias)):.3f} rad/s")
    att = attitude.attitude(run, opts.mount)
    if "pitch_g" in att:
        imu["pitch_mean_deg"] = float(np.nanmean(att["pitch_g"]))
        imu["roll_mean_deg"] = float(np.nanmean(att["roll_g"]))
    res["imu"] = imu

    ch = {}
    for p in run.channels():
        d = {"name": run.short(p)}
        for f in ("enc", "spd", "cur_ma", "duty", "cmd", "age_us"):
            c = run.col(f"{p}_{f}")
            if c is not None:
                d[f"{f}_empty_pct"] = float(100 * np.mean(~np.isfinite(c)))
        age = run.col(f"{p}_age_us")
        if age is not None and np.isfinite(age).any():
            a = age[np.isfinite(age)] / 1000
            d["age_ms"] = {"p50": float(np.percentile(a, 50)), "p95": float(np.percentile(a, 95)), "max": float(a.max())}
            if d["age_ms"]["p95"] > 100:
                flags.append(f"warning: {run.label(p)} CAN data p95 {d['age_ms']['p95']:.0f} ms old")
        if d.get("enc_empty_pct", 0) > 50:
            flags.append(f"warning: {run.label(p)} encoder empty in {d['enc_empty_pct']:.0f} % of rows (node offline?)")
        ts, x = signals.fresh(run, p, "enc")
        if len(ts) > 10:
            d["enc_first"], d["enc_last"] = float(x[0]), float(x[-1])
            d["enc_range"] = float(np.ptp(x))
        if len(ts) > 10 and d["enc_range"] >= 20:  # sign checks need real travel, not encoder flicker
            tg, xg, vg = signals.velocity(ts, x, win_s=0.2)
            tv, sv = signals.fresh(run, p, "spd")
            if len(tv) > 10 and np.nanmax(np.abs(vg)) > 5:
                vv = np.interp(tv, tg, vg)
                m = np.abs(vv) > 5
                if m.sum() > 5:
                    agree = float(np.mean(np.sign(vv[m]) == np.sign(sv[m])))
                    d["spd_sign_agrees_pct"] = 100 * agree
                    if agree < 0.5:
                        flags.append(f"warning: {run.label(p)} speed readback has the opposite sign to encoder motion")
            td, du = signals.fresh(run, p, "duty")
            if len(td) > 10 and np.nanmax(np.abs(vg)) > 5:
                vv = np.interp(td, tg, vg)
                m = (np.abs(vv) > 5) & (np.abs(du) > 1)
                if m.sum() > 5:
                    agree = float(np.mean(np.sign(vv[m]) == np.sign(du[m])))
                    d["pos_duty_raises_counts"] = bool(agree >= 0.5)
                    d["duty_sign_agrees_pct"] = 100 * agree
        lim = run.col(f"{p}_lim")
        if lim is not None and np.nanmax(np.nan_to_num(lim), initial=0) > 0:
            d["limit_rows"] = int(np.nansum(lim > 0))
            flags.append(f"note: {run.label(p)} reported a limit in {d['limit_rows']} rows")
        ch[p] = d
    res["channels"] = ch
    nodes = {}
    for n in run.nodes():
        vb = run.col(f"{n}_vbat_mv")
        if vb is not None and np.isfinite(vb).any():
            v = vb[np.isfinite(vb)] / 1000
            nodes[n] = {"vbat_min_V": float(v.min()), "vbat_max_V": float(v.max())}
            if v.min() < 21 or v.max() > 30:
                flags.append(f"warning: {n} battery {v.min():.1f}..{v.max():.1f} V outside 21..30 V")
        tc = run.col(f"{n}_temp_dc")
        if tc is not None and np.isfinite(tc).any():
            nodes.setdefault(n, {})["temp_max_C"] = float(np.nanmax(tc) / 10)
    res["nodes"] = nodes
    res["events"] = [e.to_dict() for e in run.events[:500]]
    return res


def _moving_mask(run):
    m = np.zeros(len(run.t), dtype=bool)
    for a, b, _ in signals.moves(run, settle_s=1.0):
        m |= (run.t >= a) & (run.t <= b)
    return m


def _lp(y, t, fc):
    """Display-only low-pass of a row signal (NaNs held)."""
    if len(t) < 30:
        return y
    fs = 1.0 / float(np.median(np.diff(t)))
    return signals.lowpass(signals.hold(y), fs, fc)


def _rollmax(y, n=50):
    y = np.nan_to_num(y)
    m = len(y) // n * n
    if m == 0:
        return y
    blk = y[:m].reshape(-1, n).max(axis=1)
    return np.repeat(blk, n)[: len(y)] if m == len(y) else np.concatenate([np.repeat(blk, n), y[m:]])


def _dec(t, *ys, n=6000):
    k = max(1, len(t) // n)
    return (t[::k],) + tuple(y[::k] for y in ys)


def plot(run, res, path, opts):
    att = attitude.attitude(run, opts.mount)
    t = run.t
    chans = run.channels()
    panels = ["att", "gyro", "la", "enc", "duty", "cur", "age", "vbat"]
    fig, axs = ps.figure(len(panels), 1, w=12, h_per=2.0, sharex=True)
    axs = axs[:, 0]
    p = ps.colors()

    a = axs[0]
    for i, (k, lab) in enumerate((("pitch_g", "pitch (gravity)"), ("roll_g", "roll (gravity)"),
                                  ("pitch_q", "pitch (quat)"), ("roll_q", "roll (quat)"))):
        if k in att:
            tt, y = _dec(t, _lp(att[k], t, 2.0))
            a.plot(tt, y, color=ps.series(i), lw=1.6 if i < 2 else 1.0,
                   linestyle="-" if i < 2 else (0, (3, 2)), label=lab)
    a.set_ylabel("Angle, deg")
    ps.title(a, "Chassis attitude", "pitch + = nose down, roll + = left side up; gravity vector (solid), "
             "quaternion (dashed); 2 Hz low-pass for display")
    ps.legend_above(a, [("pitch", ps.series(0)), ("roll", ps.series(1)), ("pitch quat", ps.series(2)),
                        ("roll quat", ps.series(3))])

    for a, key, unit, ttl, scale in ((axs[1], "gyro_c", "Rate, deg/s", "Angular rate", 57.29578),
                                     (axs[2], "la_c", "Accel, m/s²", "Linear acceleration", 1.0)):
        if key in att:
            for j, n in enumerate("xyz"):
                tt, y = _dec(t, _lp(att[key][:, j] * scale, t, 3.0))
                a.plot(tt, y, color=ps.series(j), lw=1.0, label=n)
            ps.legend_above(a, [(n, ps.series(j)) for j, n in enumerate("xyz")])
        a.set_ylabel(unit)
        ps.title(a, ttl, "chassis frame (after the mounting rotation); 3 Hz low-pass for display")

    def per_channel(a, field, fn, ylabel, ttl, sub, dashed=None):
        items = []
        for i, pre in enumerate(chans):
            ts, v = signals.fresh(run, pre, field)
            if not len(ts):
                continue
            tt, y = _dec(ts, fn(v, pre))
            a.plot(tt, y, color=ps.series(i), lw=1.3, drawstyle="steps-post" if field in ("duty",) else "default")
            items.append((run.short(pre), ps.series(i)))
            if dashed:
                c = run.col(f"{pre}_{dashed}")
                if c is not None and np.nanmax(np.abs(np.nan_to_num(c)), initial=0) > 0:
                    tt, y = _dec(t, c)
                    a.plot(tt, y, color=ps.series(i), lw=0.8, linestyle=(0, (3, 2)), drawstyle="steps-post")
        a.set_ylabel(ylabel)
        ps.title(a, ttl, sub)
        if items:
            ps.legend_above(a, items[:6])
        else:
            ps.empty(a, "No channel data in this run.")

    first = {}

    def rel(v, pre):
        first.setdefault(pre, v[0])
        return v - first[pre]

    per_channel(axs[3], "enc", rel, "Encoder change, counts", "Encoders", "change from the first reading, by CAN read time")
    per_channel(axs[4], "duty", lambda v, _: v, "Duty, %", "Applied duty", "controller readback (solid), firmware command (dashed)", dashed="cmd")
    per_channel(axs[5], "cur_ma", lambda v, _: v / 1000, "Current, A", "Motor current", "controller readback")
    a = axs[6]
    items = []
    for i, pre in enumerate(chans):
        c = run.col(f"{pre}_age_us")
        if c is not None:
            tt, y = _dec(t, _rollmax(c / 1000))
            a.plot(tt, y, color=ps.series(i), lw=1.0)
            items.append((run.short(pre), ps.series(i)))
    a.set_ylabel("Age, ms")
    ps.title(a, "CAN staleness", "oldest read per 0.5 s, each channel")
    if items:
        ps.legend_above(a, items[:6])
    a = axs[7]
    for i, n in enumerate(run.nodes()):
        c = run.col(f"{n}_vbat_mv")
        if c is not None:
            tt, y = _dec(t, _lp(c / 1000, t, 1.0))
            a.plot(tt, y, color=ps.series(i), lw=1.2, label=n)
    if a.get_lines():
        ps.legend_above(a, [(n, ps.series(i)) for i, n in enumerate(run.nodes())])
    a.set_ylabel("Battery, V")
    for a in axs:
        a.set_xlabel("Time, s")
    ps.title(a, "Controller supply", "per node, 1 Hz low-pass for display")
    ev = [e for e in run.events if e.source == "fw" or e.kind in ("note", "cmd", "sys")]
    ps.mark_events(axs, ev)
    loud = [e for e in ev if not e.text.lower().startswith(("step", "profile", "target", "cmd"))]
    ps.mark_events([], loud, label_axis=axs[0])
    ps.page_title(fig, "Run overview", meta_line(run))
    return ps.save(fig, path)


def summary_lines(res):
    tb = res["timebase"]
    out = [f"rows {tb['rows']}  rate {tb.get('measured_rate_hz')} Hz  dropped {tb.get('dropped_rows_est')} "
           f"({tb.get('dropped_pct')} %)  jitter p99 {tb.get('jitter_p99_us')} us  garbled {tb.get('garbled_lines')}  "
           f"stray {tb.get('stray_lines')}"]
    imu = res["imu"]
    out.append(f"imu |g| {imu.get('gravity_norm_mean', float('nan')):.2f} m/s2  pitch {imu.get('pitch_mean_deg', float('nan')):+.2f}  "
               f"roll {imu.get('roll_mean_deg', float('nan')):+.2f} deg  quat {imu['quat_convention'].get('convention')} "
               f"({imu['quat_convention'].get('agreement_deg')} deg)")
    for p, d in res["channels"].items():
        age = d.get("age_ms", {})
        out.append(f"{d['name']:>6} ({p})  enc range {d.get('enc_range', 0):6.0f}  age p95 {age.get('p95', float('nan')):5.1f} ms"
                   f"  +duty raises counts: {d.get('pos_duty_raises_counts', '?')}")
    return out
