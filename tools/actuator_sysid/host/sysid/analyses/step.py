"""step: per open-loop duty step from rest, a first-order-plus-dead-time velocity fit.

Per step: steady speed, time constant, dead time, current at steady state and peak,
coast-down distance after the excitation ends. Per channel and direction: gain
(slope of steady speed against duty) and the duty offset where that line meets
zero speed (the kinetic deadband), and the up/down asymmetry gravity causes.

Dead time is measured from the excitation edge. With the firmware's _cmd column
that is the command; with the _duty readback (excitation from outside the MIB)
it is the applied duty, so CAN latency is not in it.

On an electro-hydraulic actuator starting from rest, the fitted dead time also
holds the pressure build-up to breakaway, and the speed jumps at breakaway rather
than rising as a first-order lag: read 'lag_total' (dead time + time constant) as
"how long until it is really moving", and use 'analyze.py plant' for the plant's
own dead time, pressure lag and breakaway.
"""

from __future__ import annotations

import numpy as np

from .. import fopdt, plotstyle as ps, signals
from .common import dir_name, header, median_or_none, meta_line, slope


def step_fits(run, opts, p, min_dur=0.3):
    """FOPDT fits for every from-rest plateau of one channel."""
    src = signals.excitation_source(run, p, opts.source)
    pls = signals.plateaus(run, p, src, min_dur=min_dur)
    ts, x = signals.fresh(run, p, "enc")
    tv, sv = signals.fresh(run, p, "spd")
    tc, cur = signals.fresh(run, p, "cur_ma")
    out = []
    if len(ts) < 10:
        return src, out
    for k, pl in enumerate(pls):
        if not pl.from_rest:
            continue
        nxt = min([q.t_on for q in pls[k + 1:]] + [pl.t_off + 2.5, float(run.t[-1])])
        w = (ts >= pl.t_on) & (ts <= min(pl.t_off + 0.3, nxt))
        moved = bool(w.sum() > 2 and np.ptp(x[w]) >= opts.move_thresh)
        rec = {"t_on": pl.t_on, "t_off": pl.t_off, "duty": pl.duty, "dir": dir_name(pl.sign), "moved": moved,
               "edge": pl.edge_on, "dur": pl.dur}
        mc = (tc >= pl.t_on) & (tc <= pl.t_off)
        if mc.sum() > 2:
            ac = np.abs(cur[mc]) / 1000
            tail = tc[mc] >= pl.t_on + 0.6 * pl.dur
            rec["i_peak_A"] = float(ac.max())
            rec["i_ss_A"] = float(np.median(ac[tail])) if tail.sum() else None
        if moved:
            f = fopdt.fit(ts, x, pl.t_on, pl.t_off, tv=tv if len(tv) else None, v=sv if len(sv) else None)
            fe = fopdt.fit(ts, x, pl.t_on, pl.t_off)
            if f:
                rec.update({k2: f[k2] for k2 in ("v_ss", "tau", "L", "lag_total", "se_tau", "se_L", "rms_counts",
                                                 "joint_speed")})
                rec["fit"] = f
            if fe:
                rec["lag_total_enc_only"] = fe["lag_total"]
                if f and abs(fe["lag_total"] - f["lag_total"]) > 0.02:
                    rec["speed_readback_lag_suspect"] = True
            # coast-down after the excitation ends
            x_off = float(np.interp(pl.t_off, ts, x))
            rest = (ts >= nxt - 0.3) & (ts <= nxt)
            if rest.sum() >= 2 and nxt - pl.t_off > 0.5:
                final = float(np.median(x[rest]))
                rec["coast_counts"] = final - x_off
                after = (ts >= pl.t_off) & (ts <= nxt)
                still = np.abs(x[after] - final) <= 1.0
                ta = ts[after]
                if len(ta):
                    moving_idx = np.flatnonzero(~still)
                    rec["coast_s"] = float(ta[moving_idx[-1]] - pl.t_off) if len(moving_idx) else 0.0
        out.append(rec)
    return src, out


def compute(run, opts) -> dict:
    res = header("step", run, opts)
    chans = {}
    for p in run.channels():
        src, steps = step_fits(run, opts, p)
        if not steps:
            continue
        d = {"name": run.short(p), "source": src, "steps": steps}
        if src == "duty":
            d["dead_time_note"] = "edges from applied-duty readback: dead time is mechanical only, +-half a CAN read"
        elif any(s["edge"] == "mark" for s in steps):
            d["dead_time_note"] = "edges from operator marks: dead time and time constant are not meaningful"
        for sgn in (1, -1):
            dn = dir_name(sgn)
            ss = [s for s in steps if s["dir"] == dn and s.get("v_ss") is not None]
            if not ss:
                continue
            u = np.array([abs(s["duty"]) for s in ss])
            v = np.array([abs(s["v_ss"]) for s in ss])
            e = {"n": len(ss), "tau_s": median_or_none([s["tau"] for s in ss]),
                 "dead_time_s": median_or_none([s["L"] for s in ss]),
                 "lag_total_s": median_or_none([s["lag_total"] for s in ss]),
                 "i_ss_A": median_or_none([s.get("i_ss_A") for s in ss]),
                 "i_peak_A": max([s.get("i_peak_A") or 0 for s in ss]),
                 "coast_per_speed_s": median_or_none([abs(s["coast_counts"]) / abs(s["v_ss"]) for s in ss
                                                      if s.get("coast_counts") is not None and abs(s["v_ss"]) > 1]),
                 "apparent_gain": median_or_none(list(v / u))}
            if len(np.unique(u)) >= 2:
                K, b = np.polyfit(u, v, 1)
                e["gain"] = float(K)
                e["duty_offset_pct"] = float(-b / K) if K else None
                pred = K * u + b
                e["gain_fit_rms"] = float(np.sqrt(np.mean((v - pred) ** 2)))
            else:
                e["gain"] = e["apparent_gain"]
                e["duty_offset_pct"] = None
            d[dn] = e
        if "pos" in d and "neg" in d:
            kp, kn = d["pos"]["gain"], d["neg"]["gain"]
            asym = {"gain_ratio_pos_over_neg": kp / kn if kn else None}
            if d["pos"]["i_ss_A"] and d["neg"]["i_ss_A"]:
                asym["i_ss_ratio_pos_over_neg"] = d["pos"]["i_ss_A"] / d["neg"]["i_ss_A"]
            if d["pos"].get("duty_offset_pct") is not None and d["neg"].get("duty_offset_pct") is not None:
                asym["offset_diff_pct"] = d["pos"]["duty_offset_pct"] - d["neg"]["duty_offset_pct"]
            heavy = None
            if asym.get("i_ss_ratio_pos_over_neg"):
                heavy = "pos" if asym["i_ss_ratio_pos_over_neg"] > 1 else "neg"
            asym["loaded_direction"] = heavy
            asym["flag"] = bool((asym["gain_ratio_pos_over_neg"] and abs(asym["gain_ratio_pos_over_neg"] - 1) > 0.10)
                                or (asym.get("i_ss_ratio_pos_over_neg") and
                                    max(asym["i_ss_ratio_pos_over_neg"], 1 / asym["i_ss_ratio_pos_over_neg"]) > 1.5))
            d["asymmetry"] = asym
            if asym["flag"]:
                res["flags"].append(f"note: {run.label(p)} up/down asymmetry: gain ratio + / - "
                                    f"{asym['gain_ratio_pos_over_neg']:.2f}, current ratio "
                                    f"{asym.get('i_ss_ratio_pos_over_neg') or float('nan'):.2f} "
                                    f"(the {heavy} direction lifts the load)")
        if any(s.get("speed_readback_lag_suspect") for s in steps):
            res["flags"].append(f"warning: {run.label(p)} speed readback disagrees with encoder timing; trust lag_total")
        chans[p] = d
    res["channels"] = chans
    if not chans:
        from ..signals import has_tgt

        res["flags"].append("note: closed-loop (pstep) run: open-loop step fits do not apply; use 'analyze.py plant' "
                            "and 'analyze.py pstep'" if has_tgt(run) else
                            "warning: no steps from rest found (no excitation in _cmd, _duty or events)")
    return res


def plot(run, res, path, opts):
    chans = list(res["channels"])
    if not chans:
        fig, axs = ps.figure(1, 1)
        ps.empty(axs[0, 0], "No steps from rest in this run.")
        axs[0, 0].set_xlabel("Time, s")
        axs[0, 0].set_ylabel("Speed, counts/s")
        return ps.save(fig, path)
    fig, axs = ps.figure(len(chans), 3, w=13, h_per=2.7, gridspec_kw={"width_ratios": [1.6, 1, 1]})
    p = ps.colors()
    for r, pre in enumerate(chans):
        d = res["channels"][pre]
        ts, x = signals.fresh(run, pre, "enc")
        tg, xg, vg = signals.velocity(ts, x, win_s=opts.vel_win_s)
        a = axs[r, 0]
        for s in d["steps"]:
            if not s.get("fit"):
                continue
            c = ps.series(0 if s["dir"] == "pos" else 1)
            m = (tg >= s["t_on"] - 0.2) & (tg <= s["t_off"] + 0.8)
            a.plot(tg[m] - s["t_on"], vg[m], color=c, lw=1.2)
            tt = np.linspace(-0.2, s["dur"], 200)
            f = s["fit"]
            a.plot(tt, fopdt.model_v(tt + s["t_on"], s["t_on"], f["v_ss"], f["tau"], f["L"]), color=p["ink"], lw=0.8,
                   linestyle=(0, (3, 2)))
        a.set_xlabel("Time from step, s")
        a.set_ylabel("Speed, counts/s")
        pos, neg = d.get("pos", {}), d.get("neg", {})
        sub = (f"tau {_ms(pos.get('tau_s'))}/{_ms(neg.get('tau_s'))} ms, dead {_ms(pos.get('dead_time_s'))}/"
               f"{_ms(neg.get('dead_time_s'))} ms (+/-), edges from {d['source']}")
        ps.title(a, f"{run.short(pre)} velocity response", sub)
        ps.legend_above(a, [("+duty", ps.series(0)), ("-duty", ps.series(1)), ("FOPDT fit", p["ink"])])

        a = axs[r, 1]
        for sgn, dn in ((1, "pos"), (-1, "neg")):
            ss = [s for s in d["steps"] if s["dir"] == dn and s.get("v_ss") is not None]
            if not ss:
                continue
            c = ps.series(0 if dn == "pos" else 1)
            u = np.array([abs(s["duty"]) for s in ss])
            v = np.array([abs(s["v_ss"]) for s in ss])
            a.plot(u, v, "o", color=c, ms=5)
            e = d.get(dn, {})
            if e.get("duty_offset_pct") is not None:
                uu = np.linspace(e["duty_offset_pct"], u.max(), 20)
                a.plot(uu, e["gain"] * (uu - e["duty_offset_pct"]), color=c, lw=1.2)
        a.set_xlabel("Duty magnitude, %")
        a.set_ylabel("Steady speed, counts/s")
        ps.title(a, "Gain", f"+ {_f(pos.get('gain'))}, - {_f(neg.get('gain'))} counts/s per %")
        ps.legend_above(a, [("+duty", ps.series(0)), ("-duty", ps.series(1))])

        a = axs[r, 2]
        for dn in ("pos", "neg"):
            ss = [s for s in d["steps"] if s["dir"] == dn and s.get("i_ss_A") is not None]
            c = ps.series(0 if dn == "pos" else 1)
            if ss:
                a.plot([abs(s["duty"]) for s in ss], [s["i_ss_A"] for s in ss], "o", color=c, ms=5)
                a.plot([abs(s["duty"]) for s in ss], [s["i_peak_A"] for s in ss], "o", mfc="none", mec=c, ms=5)
        a.set_xlabel("Duty magnitude, %")
        a.set_ylabel("Current, A")
        ps.title(a, "Current", "filled steady state, hollow peak")
    ps.page_title(fig, "Step response", meta_line(run))
    return ps.save(fig, path)


def _ms(x):
    return "-" if x is None else f"{1000 * x:.0f}"


def _f(x):
    return "-" if x is None else f"{x:.2f}"


def summary_lines(res):
    out = []
    for p, d in res["channels"].items():
        for dn in ("pos", "neg"):
            e = d.get(dn)
            if not e:
                continue
            off = e.get("duty_offset_pct")
            out.append(f"{d['name']:>6} {dn}: gain {e['gain']:.2f} counts/s/%  offset {off if off is None else round(off, 1)} %  "
                       f"tau {_ms(e['tau_s'])} ms  dead {_ms(e['dead_time_s'])} ms  I_ss {e['i_ss_A'] or 0:.2f} A  "
                       f"I_peak {e['i_peak_A']:.2f} A  coast/speed {_ms(e['coast_per_speed_s'])} ms  (edges: {d['source']})")
    return out
