"""plant: identify each actuator from closed-loop data, PWM in, encoder out.

Input is the applied PWM the controller reports (_duty, at its CAN read times,
linearly interpolated); output is the encoder. It does not matter who closed the
loop (the controller's PID during a pstep, or an operator in Motion Studio):
only what the motor was actually given and what the joint did.

Model (an electro-hydraulic actuator, not back-drivable):
    q' = (u(t - L) - q) / tau                      dead time L, pressure build-up lag tau
    v  = K+ * max(q - d+, 0) - K- * max(-q - d-, 0)  gain and deadband per direction
fitted by least squares on encoder position, window by window (each move from
0.2 s before the target to 0.5 s after it is dropped).

Deadband, three ways (they should agree to a percent or so):
  d+/d-             the fitted deadband (kinetic: below it a moving joint stops)
  stall PWM         PWM held at rest after a step stopped short (from pstep)
  kp x stall error  the same number predicted by the loop gain (equal for a P-only loop)
Breakaway (to start from rest, usually above d): a bracket between the largest
PWM held without moving and the lag-compensated PWM at which motion began.
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import lfilter

from .. import plotstyle as ps, signals
from .common import header, meta_line
from .pstep import loop_gain, windows

DT = 0.005


def _u_of(td, du):
    def f(t):
        return np.interp(t, td, du, left=0.0, right=0.0) if len(td) else np.zeros_like(t)

    return f


def _gate(q, brk, kin):
    """1 from the first time q passes breakaway until it next falls to the kinetic level (one move)."""
    g = np.zeros(len(q))
    above = q > brk
    if not above.any():
        return g
    i0 = int(np.argmax(above))
    below = q[i0:] <= kin
    i1 = i0 + int(np.argmax(below)) if below.any() else len(q)
    g[i0:i1] = 1.0
    return g


def _predict(p, segs, uf, with_v=False):
    Kp, Kn, dp, dn, tau, L, hp, hn = p
    a = np.exp(-DT / max(tau, 1e-3))
    out = []
    for tg, x0 in segs:
        u = uf(tg - L)
        q = lfilter([1 - a], [1, -a], u)
        v = (Kp * np.maximum(q - dp, 0) * _gate(q, dp + hp, dp)
             - Kn * np.maximum(-q - dn, 0) * _gate(-q, dn + hn, dn))
        xp = x0 + np.concatenate([[0.0], np.cumsum(v[:-1]) * DT])
        out.append((xp, v) if with_v else xp)
    return out


def fit_channel(run, p, wins):
    ts, x = signals.fresh(run, p, "enc")
    td, du = signals.fresh(run, p, "duty")
    if len(ts) < 30 or len(td) < 30 or np.nanmax(np.abs(du)) < 1:
        return None
    uf = _u_of(td, du)
    tv, sv = signals.fresh(run, p, "spd")
    segs, obs, vobs = [], [], []
    for t0, t1, *_ in wins:
        a, b = t0 - 0.2, t1 + 0.5
        m = (ts >= a) & (ts <= b)
        if m.sum() < 10:
            continue
        tg = np.arange(a, b, DT)
        pre = ts < t0
        x0 = float(np.median(x[pre & (ts > t0 - 0.5)])) if (pre & (ts > t0 - 0.5)).any() else float(x[m][0])
        segs.append((tg, x0))
        obs.append((ts[m], x[m]))
        mv = (tv >= a) & (tv <= b)
        vobs.append((tv[mv], sv[mv]))
    if not segs:
        return None
    # initial guess from a lag-free static map
    tg_all = np.arange(ts[0], ts[-1], DT)
    _, _, vg = signals.velocity(ts, x, fs=1 / DT, win_s=0.15)
    ug = uf(tg_all[: len(vg)] - 0.1)
    mv = np.abs(vg) > 3
    Kp0 = Kn0 = 5.0
    dp0 = dn0 = 5.0
    for sgn in (1, -1):
        m = mv & (np.sign(vg) == sgn) & (np.sign(ug) == sgn)
        if m.sum() > 20:
            k, c = np.polyfit(np.abs(ug[m]), np.abs(vg[m]), 1)
            if k > 0:
                if sgn > 0:
                    Kp0, dp0 = k, max(-c / k, 0.5)
                else:
                    Kn0, dn0 = k, max(-c / k, 0.5)

    use_v = sum(len(a) for a, _ in vobs) > 50
    sd_v = 2.0
    if use_v:
        rest_v = np.concatenate([b[np.abs(b) < 5] for _, b in vobs if len(b)]) if vobs else np.array([])
        sd_v = max(float(np.std(rest_v)) if len(rest_v) > 20 else 2.0, 0.5)

    def resid(pv, split=False):
        pred = _predict(pv, segs, uf, with_v=True)
        rx = np.concatenate([np.interp(to, tg, xp) - xo for (to, xo), (tg, _), (xp, _) in zip(obs, segs, pred)]) / 0.6
        if not use_v:
            return rx
        rv = np.concatenate([np.interp(tvv, tg, vp) - vv for (tvv, vv), (tg, _), (_, vp) in zip(vobs, segs, pred)]) / sd_v
        return (rx, rv) if split else np.concatenate([rx, rv])

    best = None
    for tau0 in (0.05, 0.15, 0.35):
        for L0 in (0.02, 0.06):
            p0 = [Kp0, Kn0, min(dp0, 30), min(dn0, 30), tau0, L0, 2.0, 2.0]
            try:
                r = least_squares(resid, p0, bounds=([0.01, 0.01, 0, 0, 0.005, 0, 0, 0],
                                                     [500, 500, 60, 60, 2.0, 0.4, 40, 40]),
                                  loss="soft_l1", f_scale=3.0, x_scale=[1, 1, 1, 1, 0.05, 0.02, 1, 1],
                                  diff_step=[1e-3, 1e-3, 1e-2, 1e-2, 1e-2, 1e-2, 2e-2, 2e-2])
            except ValueError:
                continue
            if best is None or r.cost < best.cost:
                best = r
    if best is None:
        return None
    rr = (resid(best.x, split=True)[0] if use_v else resid(best.x)) * 0.6
    try:
        J = best.jac
        rall = resid(best.x)
        cov = np.linalg.pinv(J.T @ J) * (np.sum(rall ** 2) / max(len(rall) - 8, 1))
        se = np.sqrt(np.clip(np.diag(cov), 0, None))
    except np.linalg.LinAlgError:
        se = np.full(8, np.nan)
    Kp, Kn, dp, dn, tau, L, hp, hn = best.x
    return {"gain_pos": Kp, "gain_neg": Kn, "deadband_pos_pct": dp, "deadband_neg_pct": dn, "tau_s": tau,
            "dead_time_s": L, "breakaway_pos_pct": dp + hp, "breakaway_neg_pct": dn + hn,
            "se": dict(zip(("gain_pos", "gain_neg", "deadband_pos_pct", "deadband_neg_pct",
                            "tau_s", "dead_time_s", "hysteresis_pos_pct", "hysteresis_neg_pct"), se.tolist())),
            "rms_counts": float(np.sqrt(np.mean(rr ** 2))), "n_windows": len(segs), "n_samples": int(len(rr)),
            "speed_readback_used": bool(use_v),
            "_segs": segs, "_obs": obs, "_x": best.x.tolist()}


def breakaway(run, p, wins, fit):
    """Bracket per direction: [largest PWM held without moving, lag-compensated PWM at motion onset]."""
    ts, x = signals.fresh(run, p, "enc")
    td, du = signals.fresh(run, p, "duty")
    uf = _u_of(td, du)
    tau, L = fit["tau_s"], fit["dead_time_s"]
    a = np.exp(-DT / tau)
    held = {1: [], -1: []}
    onset = {1: [], -1: []}
    for t0, t1, *_ in wins:
        m = (ts >= t0) & (ts <= t1)
        if m.sum() < 5:
            continue
        pre = (ts < t0) & (ts > t0 - 0.5)
        x0 = float(np.median(x[pre])) if pre.any() else float(x[m][0])
        tg = np.arange(t0 - 0.5, t1, DT)
        q = lfilter([1 - a], [1, -a], uf(tg - L))
        moved_idx = np.flatnonzero(np.abs(x[m] - x0) >= 2.5)
        uw = uf(tg)
        sgn = int(np.sign(np.median(uw[tg > t0 + 0.05]))) if (tg > t0 + 0.05).any() else 0
        if sgn == 0:
            continue
        if not len(moved_idx):
            held[sgn].append(float(np.max(np.abs(q[tg > t0]))))
        else:
            t_on = ts[m][moved_idx[0]]
            # back off the time the first 2.5 counts took at the early speed
            q_on = float(np.abs(np.interp(t_on - 0.03, tg, q)))
            onset[sgn].append(q_on)
    out = {}
    for sgn, dn in ((1, "pos"), (-1, "neg")):
        lo = max(held[sgn]) if held[sgn] else None
        hi = min(onset[sgn]) if onset[sgn] else None
        out[dn] = {"breakaway_bracket_pct": [lo, hi], "n_held": len(held[sgn]), "n_moved": len(onset[sgn])}
    return out


def compute(run, opts) -> dict:
    res = header("plant", run, opts)
    W = windows(run)
    if not W:
        # open-loop or externally driven: windows from the moves
        for a, b, ps_ in signals.moves(run, settle_s=0.0, source=opts.source):
            for p in ps_:
                W.setdefault(p, []).append((a, b, None, "duty"))
    chans = {}
    for p, wins in W.items():
        f = fit_channel(run, p, wins)
        if f is None:
            continue
        lg = loop_gain(run, p, [w for w in wins if w[2] is not None]) if any(w[2] is not None for w in wins) else None
        br = breakaway(run, p, wins, f)
        d = {"name": run.short(p), "tau_s": f["tau_s"], "dead_time_s": f["dead_time_s"], "fit_rms_counts": f["rms_counts"],
             "n_windows": f["n_windows"], "se": f["se"], "loop": lg, "_fit": f}
        stall = _stalls(run, p, wins)
        for sgn, dn in ((1, "pos"), (-1, "neg")):
            e = {"gain": f[f"gain_{dn}"], "deadband_pct": f[f"deadband_{dn}_pct"],
                 "kinetic_pct": f[f"deadband_{dn}_pct"], **br[dn]}
            e["breakaway_pct"] = f[f"breakaway_{dn}_pct"]
            lo, hi = e["breakaway_bracket_pct"]
            if lo is not None and e["breakaway_pct"] < lo - 0.5 or hi is not None and e["breakaway_pct"] > hi + 0.5:
                res["flags"].append(f"note: {run.label(p)} {dn}: fitted breakaway {e['breakaway_pct']:.1f} % outside "
                                    f"the bracket {e['breakaway_bracket_pct']} from step onsets")
            st = stall.get(sgn)
            if st:
                e["stall_pwm_pct"] = st["pwm"]
                e["stall_error_counts"] = st["err"]
                if lg:
                    e["kp_x_stall_error_pct"] = lg["kp_pct_per_count"] * st["err"]
            d[dn] = e
        kp_, kn_ = d["pos"]["gain"], d["neg"]["gain"]
        d["gain_ratio_pos_over_neg"] = kp_ / kn_ if kn_ else None
        if d["gain_ratio_pos_over_neg"] and abs(d["gain_ratio_pos_over_neg"] - 1) > 0.10:
            res["flags"].append(f"note: {run.label(p)}: gain + {kp_:.2f} vs - {kn_:.2f} counts/s per % "
                                f"(the {'- ' if kp_ < kn_ else '+'}direction is helped by the load)")
        if f["rms_counts"] > 3:
            res["flags"].append(f"warning: {run.label(p)}: model misses the encoder by {f['rms_counts']:.1f} counts RMS")
        for dn in ("pos", "neg"):
            e = d[dn]
            if e.get("stall_pwm_pct") is not None and abs(e["stall_pwm_pct"] - e["deadband_pct"]) > 2:
                res["flags"].append(f"note: {run.label(p)} {dn}: stall PWM {e['stall_pwm_pct']:.1f} % vs fitted "
                                    f"deadband {e['deadband_pct']:.1f} % (stick-slip, or steps cut short)")
        chans[p] = d
    res["channels"] = chans
    if not chans:
        res["flags"].append("warning: no moves with PWM readback to identify from")
    return res


def _stalls(run, p, wins):
    ts, x = signals.fresh(run, p, "enc")
    td, du = signals.fresh(run, p, "duty")
    out = {1: [], -1: []}
    for t0, t1, tgt, _ in wins:
        if tgt is None:
            continue
        tail_x = (ts >= t1 - 0.5) & (ts <= t1)
        tail_u = (td >= t1 - 0.5) & (td <= t1)
        pre = (ts < t0) & (ts > t0 - 0.5)
        if tail_x.sum() < 3 or tail_u.sum() < 2 or not pre.any():
            continue
        x0, xf = float(np.median(x[pre])), float(np.median(x[tail_x]))
        moving = np.ptp(x[tail_x]) > 2
        err = tgt - xf
        if abs(xf - x0) < 3 or moving or abs(err) < 3:
            continue  # never moved, still moving, or reached the target
        out[int(np.sign(err))].append((abs(float(np.median(du[tail_u]))), abs(err)))
    return {s: {"pwm": float(np.median([a for a, _ in v])), "err": float(np.median([b for _, b in v]))}
            for s, v in out.items() if v}


def plot(run, res, path, opts):
    chans = res["channels"]
    if not chans:
        fig, axs = ps.figure(1, 1)
        ps.empty(axs[0, 0], "No moves with PWM readback.")
        axs[0, 0].set_xlabel("PWM, %")
        axs[0, 0].set_ylabel("Speed, counts/s")
        return ps.save(fig, path)
    fig, axs = ps.figure(len(chans), 2, w=13, h_per=3.2, gridspec_kw={"width_ratios": [1, 1.5]})
    col = ps.colors()
    for r, (p, d) in enumerate(chans.items()):
        f = d["_fit"]
        a, b = axs[r]
        ts, x = signals.fresh(run, p, "enc")
        td, du = signals.fresh(run, p, "duty")
        uf = _u_of(td, du)
        tg, xg, vg = signals.velocity(ts, x, fs=1 / DT, win_s=0.12)
        al = np.exp(-DT / f["tau_s"])
        q = lfilter([1 - al], [1, -al], uf(tg - f["dead_time_s"]))
        inwin = np.zeros(len(tg), dtype=bool)
        for seg, _ in f["_segs"]:
            inwin |= (tg >= seg[0]) & (tg <= seg[-1])
        k = max(1, inwin.sum() // 4000)
        a.plot(q[inwin][::k], vg[inwin][::k], ".", color=ps.series(0), ms=1.5, alpha=0.5)
        qq = np.linspace(min(q[inwin].min(), -1), max(q[inwin].max(), 1), 200)
        Kp, Kn, dp, dn = f["_x"][:4]
        a.plot(qq, Kp * np.maximum(qq - dp, 0) - Kn * np.maximum(-qq - dn, 0), color=col["ink"], lw=1.2)
        ps.ref_line(a, dp, f"+{dp:.1f}", "good", vertical=True, bottom=True)
        ps.ref_line(a, -dn, f"-{dn:.1f}", "good", vertical=True, bottom=True)
        for dn_, sg in (("pos", 1), ("neg", -1)):
            bk = d[dn_].get("breakaway_pct")
            if bk is not None:
                ps.ref_line(a, sg * bk, f"brk {bk:.1f}", "warning", vertical=True)
        a.set_xlabel("PWM, lag-compensated, %")
        a.set_ylabel("Speed, counts/s")
        ps.title(a, f"{run.short(p)} static map", f"gain +{Kp:.2f} / -{Kn:.2f} counts/s per %, deadband +{dp:.1f} / -{dn:.1f} %")
        # model vs measured, all windows end to end
        off = 0.0
        for (seg, x0), (to, xo), xp in zip(f["_segs"], f["_obs"], _predict(f["_x"], f["_segs"], uf)):
            b.plot(to - seg[0] + off, xo - x0, color=ps.series(0), lw=1.4)
            b.plot(seg - seg[0] + off, xp - x0, color=col["ink"], lw=0.8, linestyle=(0, (3, 2)))
            off += seg[-1] - seg[0] + 0.3
        b.set_xlabel("Time within moves, end to end, s")
        b.set_ylabel("Position change, counts")
        ps.title(b, "Model against encoder", f"tau {1000 * f['tau_s']:.0f} ms, dead time {1000 * f['dead_time_s']:.0f} ms, "
                 f"residual {f['rms_counts']:.1f} counts RMS")
        ps.legend_above(b, [("encoder", ps.series(0)), ("model", col["ink"])])
    ps.page_title(fig, "Plant identification", meta_line(run))
    return ps.save(fig, path)


def summary_lines(res):
    out = []
    f = lambda v, d=1: "-" if v is None else f"{v:.{d}f}"  # noqa: E731
    for p, d in res["channels"].items():
        out.append(f"{d['name']:>6}: tau {1000 * d['tau_s']:.0f} ms  dead {1000 * d['dead_time_s']:.0f} ms  fit "
                   f"{d['fit_rms_counts']:.1f} counts RMS  ({d['n_windows']} moves)")
        for dn in ("pos", "neg"):
            e = d[dn]
            out.append(f"        {dn}: gain {e['gain']:.2f} counts/s/%  deadband {e['deadband_pct']:.1f} %  stall PWM "
                       f"{f(e.get('stall_pwm_pct'))} %  kp x stall err {f(e.get('kp_x_stall_error_pct'))} %  breakaway "
                       f"{f(e.get('breakaway_pct'))} % (step onsets bracket "
                       f"{[None if v is None else round(v, 1) for v in e['breakaway_bracket_pct']]})")
    return out
