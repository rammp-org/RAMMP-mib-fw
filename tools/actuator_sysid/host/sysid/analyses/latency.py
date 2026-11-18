"""latency: split command-to-chassis delay into its parts.

  CAN / command   firmware command edge -> the controller reports the new duty.
                  Each step brackets it: the last CAN read still showing the old duty
                  and the first showing the new. Intersecting the brackets of many
                  steps (read phases differ) narrows it to a few ms.
  mechanical      applied duty -> encoder motion: FOPDT dead time minus the CAN part.
  encoder -> IMU  the lag that best explains chassis gyro from encoder velocities
                  (a least-squares cross-correlation over lags); includes the BNO055
                  fusion delay and any mechanical compliance.
  staleness       statistics of each channel's _age_us.

With excitation from outside the MIB (no _cmd), the CAN part cannot be measured;
the dead time from the duty readback edge is then mechanical only.
"""

from __future__ import annotations

import numpy as np

from .. import attitude, plotstyle as ps, signals
from .common import header, median_or_none, meta_line
from .step import step_fits


def compute(run, opts) -> dict:
    res = header("latency", run, opts)
    ages = {}
    for p in run.channels():
        a = run.col(f"{p}_age_us")
        if a is not None and np.isfinite(a).any():
            v = a[np.isfinite(a)] / 1000
            ts, _ = signals.fresh(run, p, "enc")
            ages[p] = {"name": run.short(p), "mean_ms": float(v.mean()), "p50_ms": float(np.percentile(v, 50)),
                       "p95_ms": float(np.percentile(v, 95)), "max_ms": float(v.max()),
                       "read_rate_hz": float(len(ts) / max(np.ptp(ts), 1e-9)) if len(ts) > 2 else None}
    res["age"] = ages

    chans = {}
    all_dead = []
    stop_edges = []
    for p in run.channels():
        src, steps = step_fits(run, opts, p)
        steps = [s for s in steps if s.get("L") is not None]
        if not steps:
            d = _closed_loop(run, p, ages)
            if d:
                all_dead.append(d["dead_time_s"])
                stop_edges += d.pop("_edges")
                chans[p] = d
            continue
        d = {"name": run.short(p), "source": src, "n_steps": len(steps)}
        d["dead_time_s"] = median_or_none([s["L"] for s in steps])
        d["tau_s"] = median_or_none([s["tau"] for s in steps])
        if src == "cmd":
            td, du = signals.fresh(run, p, "duty")
            lows, highs = [], []
            for s in steps:
                pre = td < s["t_on"]
                old = du[pre][-1] if pre.any() else 0.0
                half = 0.5 * (old + s["duty"])
                after = td >= s["t_on"] - 0.05
                idx = np.flatnonzero(after)
                new_idx = [i for i in idx if (du[i] - half) * np.sign(s["duty"] - old) > 0]
                if not new_idx:
                    continue
                i = new_idx[0]
                highs.append(td[i] - s["t_on"])
                lows.append(max(td[i - 1] - s["t_on"], 0.0) if i > 0 else 0.0)
            if highs:
                lo, hi = max(lows), min(highs)
                d["can_bracket_s"] = [lo, hi]
                d["can_latency_s"] = 0.5 * (lo + hi) if hi >= lo else float(np.median(highs) - 0.5 / max(
                    ages.get(p, {}).get("read_rate_hz") or 50, 1))
                d["can_bracket_consistent"] = bool(hi >= lo)
                d["mechanical_dead_time_s"] = d["dead_time_s"] - d["can_latency_s"]
        else:
            d["can_latency_s"] = None
            d["mechanical_dead_time_s"] = d["dead_time_s"]
            d["note"] = f"edges from {src}: CAN latency not measurable, dead time is from the applied duty"
        all_dead.append(d["dead_time_s"])
        stop_edges += [s["t_off"] for s in steps]
        chans[p] = d
    res["channels"] = chans

    # encoder -> IMU lag
    att = attitude.attitude(run, opts.mount)
    imu = {}
    if "gyro_c" in att and chans:
        grid_fs = 200.0
        t0, t1 = float(run.t[0]), float(run.t[-1])
        tg = np.arange(t0, t1, 1 / grid_fs)
        V = []
        for p in run.channels():
            ts, x = signals.fresh(run, p, "enc")
            if len(ts) < 20 or np.ptp(x) < 10:
                continue
            tt, xx, vv = signals.velocity(ts, x, fs=grid_fs, win_s=0.06)
            vv = signals.lowpass(np.nan_to_num(vv), grid_fs, 8.0)
            V.append(np.interp(tg, tt, vv, left=0, right=0))
        # Only around the ends of moves: at a start the chassis waits for the backlash to be
        # taken up, which would read as IMU lag; at a stop the load follows the motor directly.
        near = np.zeros(len(run.t), dtype=bool)
        for te in stop_edges:
            near |= (run.t >= te - 0.3) & (run.t <= te + 0.6)
        for d in chans.values():
            for a, b in d.pop("_spans", []):
                near |= (run.t >= a) & (run.t <= b)
        if V and near.sum() > 50:
            X = np.column_stack(V)
            lags = np.arange(-0.06, 0.12, 0.002)
            row_fs = 1.0 / float(np.median(np.diff(run.t)))
            Y = np.column_stack([signals.lowpass(signals.hold(att["gyro_c"][:, j]), row_fs, 8.0) for j in (0, 1)])
            r2, r2x, r2y = _joint_scan(tg, X, run.t[near], Y[near], lags)
            k = int(np.nanargmax(r2))
            lag, se = _peak(lags, r2, k, n_eff=near.sum() * min(1.0, 2 * 8.0 / row_fs))
            imu = {"enc_to_imu_lag_s": lag, "enc_to_imu_lag_se_s": se, "lags_s": lags.tolist(), "r2": r2.tolist(),
                   "r2_x": r2x.tolist(), "r2_y": r2y.tolist(), "r2_max": float(np.nanmax(r2)),
                   "chassis_rate_p95_dps": float(np.degrees(np.nanpercentile(np.linalg.norm(Y[near], axis=1), 95)))}
            if se is None or se > 0.008:
                res["flags"].append(
                    f"warning: encoder->IMU lag uncertain ({'?' if se is None else f'+-{1000 * se:.0f}'} ms): the chassis "
                    f"turned at only {imu['chassis_rate_p95_dps']:.2f} deg/s against gyro noise; use larger, faster moves")
    res["imu"] = imu
    res["summary"] = {
        "can_latency_s": median_or_none([d.get("can_latency_s") for d in chans.values()]),
        "dead_time_s": median_or_none(all_dead),
        "mechanical_dead_time_s": median_or_none([d.get("mechanical_dead_time_s") for d in chans.values()]),
        "tau_s": median_or_none([d.get("tau_s") for d in chans.values()]),
        "enc_to_imu_lag_s": imu.get("enc_to_imu_lag_s"),
        "enc_to_imu_lag_se_s": imu.get("enc_to_imu_lag_se_s"),
        "age_p95_ms_max": max([a["p95_ms"] for a in ages.values()], default=None),
    }
    return res


def _closed_loop(run, p, ages):
    """pstep runs: CAN = target edge -> first nonzero PWM readback; mechanical = the plant fit's dead time."""
    from .plantid import fit_channel
    from .pstep import windows

    wins = windows(run).get(p)
    if not wins:
        return None
    td, du = signals.fresh(run, p, "duty")
    lows, highs = [], []
    for t0, t1, *_ in wins:
        pre = (td < t0) & (td > t0 - 0.5)
        if pre.any() and np.max(np.abs(du[pre])) > 0.5:
            continue  # PWM already applied: not a clean edge
        idx = np.flatnonzero((td >= t0 - 0.05) & (np.abs(du) > 0.5))
        if not len(idx):
            continue
        i = idx[0]
        highs.append(td[i] - t0)
        lows.append(max(td[i - 1] - t0, 0.0) if i > 0 else 0.0)
    f = fit_channel(run, p, wins)
    if f is None:
        return None
    d = {"name": run.short(p), "source": "tgt", "n_steps": len(wins), "tau_s": f["tau_s"]}
    if highs:
        lo, hi = max(lows), min(highs)
        d["can_bracket_s"] = [lo, hi]
        d["can_bracket_consistent"] = bool(hi >= lo)
        d["can_latency_s"] = 0.5 * (lo + hi) if hi >= lo else float(np.median(highs))
    else:
        d["can_latency_s"] = None
    d["mechanical_dead_time_s"] = f["dead_time_s"]
    d["dead_time_s"] = f["dead_time_s"] + (d["can_latency_s"] or 0.0)
    d["note"] = "closed loop: CAN from target edge to PWM readback; mechanical from the plant fit (PWM in)"
    # motion ends gently in a P loop: use the whole move after the backlash is taken up
    d["_edges"] = []
    d["_spans"] = [(t0 + 0.3, t1) for t0, t1, *_ in wins]
    return d


def _joint_scan(tg, X, ty, Y, lags):
    """R^2 of both gyro axes fitted from encoder speeds X(t - lag), pooled; and per axis."""
    r2, r2x, r2y = (np.full(len(lags), np.nan) for _ in range(3))
    sst = np.nansum((Y - np.nanmean(Y, axis=0)) ** 2, axis=0)
    for k, lag in enumerate(lags):
        tq = ty - lag
        m = (tq >= tg[0]) & (tq <= tg[-1]) & np.all(np.isfinite(Y), axis=1)
        if m.sum() < 20:
            continue
        A = np.column_stack([np.interp(tq[m], tg, X[:, j]) for j in range(X.shape[1])] + [np.ones(m.sum())])
        coef, *_ = np.linalg.lstsq(A, Y[m], rcond=None)
        sse = np.sum((Y[m] - A @ coef) ** 2, axis=0)
        r2x[k], r2y[k] = 1 - sse / np.where(sst > 0, sst, np.nan)
        r2[k] = 1 - sse.sum() / sst.sum()
    return r2, r2x, r2y


def _peak(lags, r2, k, n_eff):
    """Parabolic peak of R^2(lag) and a standard error from its curvature:
    SSE(lag) ~ SSE_min + c (lag - lag*)^2, se = sqrt(sigma^2 / c), sigma^2 = SSE_min / n_eff."""
    lo, hi = max(k - 5, 0), min(k + 6, len(lags))
    ok = np.isfinite(r2[lo:hi])
    if ok.sum() < 5:
        return float(lags[k]), None
    x, y = lags[lo:hi][ok], 1 - r2[lo:hi][ok]  # SSE / SST
    c2, c1, c0 = np.polyfit(x, y, 2)
    if c2 <= 0:
        return float(lags[k]), None
    lag = float(np.clip(-c1 / (2 * c2), lags[0], lags[-1]))
    sse_min = max(c0 - c1 * c1 / (4 * c2), 1e-12)
    return lag, float(np.sqrt(sse_min / max(n_eff, 1) / c2))


def plot(run, res, path, opts):
    fig, axs = ps.figure(2, 2, w=12, h_per=3.4)
    s = res["summary"]
    a = axs[0, 0]
    labels, vals = [], []
    se = (res.get("imu") or {}).get("enc_to_imu_lag_se_s")
    shaky = se is None or se > 0.008
    for lab, k in (("CAN / command", "can_latency_s"), ("Mechanical dead", "mechanical_dead_time_s"),
                   ("Time constant", "tau_s"), ("Encoder to IMU", "enc_to_imu_lag_s")):
        if s.get(k) is not None:
            labels.append(lab)
            vals.append(round(1000 * s[k], 1))
    if vals:
        ps.bars(a, labels, vals, unit=" ms", xlabel="Delay, ms", ylabel="Component",
                highlight="Encoder to IMU" if shaky else None)
    else:
        ps.empty(a, "No steps to measure.")
        a.set_xlabel("Delay, ms")
        a.set_ylabel("Component")
    ps.title(a, "Delay budget", "medians over channels and steps" + ("; red = uncertain, see flags" if shaky and
                                                                        "Encoder to IMU" in labels else ""))

    a = axs[0, 1]
    imu = res.get("imu") or {}
    if imu.get("lags_s"):
        lg = np.array(imu["lags_s"]) * 1000
        a.plot(lg, imu["r2"], color=ps.series(0), lw=2.0)
        a.plot(lg, imu["r2_y"], color=ps.series(1), lw=1.0)
        a.plot(lg, imu["r2_x"], color=ps.series(2), lw=1.0)
        se = imu.get("enc_to_imu_lag_se_s")
        ok = se is not None and se <= 0.008
        ps.ref_line(a, 1000 * imu["enc_to_imu_lag_s"], f"{1000 * imu['enc_to_imu_lag_s']:.1f}"
                    + (f" +-{1000 * se:.0f}" if se is not None else "") + " ms", "good" if ok else "warning", vertical=True)
        ps.legend_above(a, [("both", ps.series(0)), ("gyro y (pitch)", ps.series(1)), ("gyro x (roll)", ps.series(2))])
    else:
        ps.empty(a, "No IMU or no encoder motion.")
    a.set_xlabel("IMU lag behind encoder, ms")
    a.set_ylabel("Fit R², fraction")
    ps.title(a, "Encoder to IMU", "R² by lag, around move ends")

    a = axs[1, 0]
    ages = res.get("age") or {}
    if ages:
        ps.bars(a, [f"{v['name']} p95" for v in ages.values()], [round(v["p95_ms"], 1) for v in ages.values()],
                unit=" ms", xlabel="Age, ms", ylabel="Channel")
    else:
        ps.empty(a, "No _age_us columns.")
        a.set_xlabel("Age, ms")
        a.set_ylabel("Channel")
    ps.title(a, "CAN staleness", "95th percentile of _age_us")

    a = axs[1, 1]
    _example(run, res, a, opts)
    ps.page_title(fig, "Latency", meta_line(run))
    return ps.save(fig, path)


def _example(run, res, a, opts):
    chans = res["channels"]
    if not chans:
        ps.empty(a, "No steps.")
        a.set_xlabel("Time from step, s")
        a.set_ylabel("Normalized response, fraction")
        return
    p = next(iter(chans))
    src, steps = step_fits(run, opts, p)
    steps = [s for s in steps if s.get("L") is not None]
    td, du = signals.fresh(run, p, "duty")
    if steps:
        s = max(steps, key=lambda s: abs(s["duty"]))
        t0, scale = s["t_on"], s["duty"]
        label = f"{s['duty']:+.0f} % duty"
    else:  # closed loop: the largest pstep; PWM scaled to its peak
        from .pstep import windows

        wins = windows(run).get(p, [])
        if not wins:
            ps.empty(a, "No steps.")
            a.set_xlabel("Time from step, s")
            a.set_ylabel("Normalized response, fraction")
            return
        ts_, x_ = signals.fresh(run, p, "enc")

        def size(w):
            pre = ts_ < w[0]
            return abs(w[2] - (x_[pre][-1] if pre.any() else x_[0]))

        w = max(wins, key=size)
        t0 = w[0]
        mm = (td >= t0) & (td <= t0 + 0.6)
        scale = du[mm][np.argmax(np.abs(du[mm]))] if mm.any() else 1.0
        s = {"v_ss": None}
        label = "largest pstep, PWM scaled to its peak"
    win = (-0.1, 0.6)
    tt = np.linspace(*win, 300)
    a.plot(tt, (tt >= 0).astype(float), color=ps.series(0), lw=1.4, label="command")
    m = (td >= t0 + win[0]) & (td <= t0 + win[1])
    a.plot(td[m] - t0, du[m] / (scale or 1), color=ps.series(1), lw=1.4, drawstyle="steps-post", label="duty readback")
    ts, x = signals.fresh(run, p, "enc")
    tg, xg, vg = signals.velocity(ts, x, win_s=0.08)
    m = (tg >= t0 + win[0]) & (tg <= t0 + win[1])
    if m.any():
        vs = s["v_ss"] or (np.nanmax(np.abs(vg[m])) * np.sign(scale)) or 1
        a.plot(tg[m] - t0, vg[m] / vs, color=ps.series(2), lw=1.4, label="encoder speed")
    att = attitude.attitude(run, opts.mount)
    if "gyro_c" in att:
        g = np.linalg.norm(att["gyro_c"][:, :2], axis=1)
        m = (run.t >= t0 + win[0]) & (run.t <= t0 + win[1])
        if m.any():
            gm = signals.lowpass(signals.hold(g), 100.0, 8.0)[m]
            base = np.median(gm[:8]) if m.sum() > 8 else 0
            top = np.percentile(gm, 90) - base
            a.plot(run.t[m] - t0, (gm - base) / (top or 1), color=ps.series(3), lw=1.2, label="gyro")
    ps.label_lines(a)
    a.set_xlabel("Time from step, s")
    a.set_ylabel("Normalized response, fraction")
    ps.title(a, f"{run.short(p)} example step", f"{label}, each signal scaled to its final or peak value")


def summary_lines(res):
    s = res["summary"]
    f = lambda x: "-" if x is None else f"{1000 * x:.1f} ms"  # noqa: E731
    se = (res.get("imu") or {}).get("enc_to_imu_lag_se_s")
    out = [f"CAN/command {f(s['can_latency_s'])}  dead time {f(s['dead_time_s'])}  mechanical "
           f"{f(s['mechanical_dead_time_s'])}  tau {f(s['tau_s'])}  encoder->IMU {f(s['enc_to_imu_lag_s'])}"
           + (f" +-{1000 * se:.1f} ms" if se is not None else "")]
    for p, d in res["channels"].items():
        out.append(f"{d['name']:>6}: CAN bracket {['%.1f ms' % (1000 * v) for v in d.get('can_bracket_s', [])]}  "
                   f"dead {f(d['dead_time_s'])}  mech {f(d.get('mechanical_dead_time_s'))}  (edges: {d['source']})")
    for p, a in res["age"].items():
        out.append(f"{a['name']:>6} age p50 {a['p50_ms']:.1f}  p95 {a['p95_ms']:.1f}  max {a['max_ms']:.1f} ms  "
                   f"reads {a['read_rate_hz'] or 0:.1f} Hz")
    return out
