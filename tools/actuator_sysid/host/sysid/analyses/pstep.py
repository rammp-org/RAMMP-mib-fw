"""pstep: closed-loop position steps run by the controller's own PID (firmware `pstep <node> <ch> <delta>`).

A step is a span where n<node>c<ch>_tgt holds one value (the firmware logs the
target while the move is active and leaves it empty otherwise). Without that
column, steps come from events or operator notes: 'target <node> <ch> <counts>'
(absolute) or 'pstep <node> <ch> <delta>' (relative to the encoder at that time).

Per step (definitions in sysid/stepmetrics.py): rise time, overshoot, settling,
steady-state error in counts, and the PWM (duty readback) and current at rest
(last 0.5 s of the step). Per channel the loop gain is estimated from the data
(PWM against error while unsaturated): with a P-only loop PWM at rest equals
kp x stall error, and a PI loop shows up as a residual that grows with time.
A P-only loop against an EHA's deadband stops where kp x error falls to the
kinetic threshold: 'stalled' steps report that PWM; compare it with the
deadband from 'analyze.py plant'.
"""

from __future__ import annotations

import json
import re
from pathlib import Path

import numpy as np

from .. import plotstyle as ps, signals
from ..stepmetrics import position_step
from .common import header, meta_line

TARGET_RE = re.compile(r"\b(?:target|goto)\s+n?(\d+)[\s,]*c?(\d+)\s+(-?\d+(?:\.\d+)?)", re.I)
PSTEP_RE = re.compile(r"\bpstep\s+n?(\d+)[\s,]*c?(\d+)\s+([+-]?\d+(?:\.\d+)?)", re.I)
STOP_RE = re.compile(r"\b(stop|abort|end|released|done)\b", re.I)


def windows(run):
    """{prefix: [(t_on, t_off, target, edge)]}."""
    out = {}
    for p in run.channels():
        sp = signals.tgt_spans(run, p)
        if sp:
            out[p] = [(s.t_on, s.t_off, s.target, s.edge_on) for s in sp]
    if out:
        return out
    # no _tgt column: events and notes
    seen = []
    starts = {}
    for e in run.events:
        m = TARGET_RE.search(e.text) or PSTEP_RE.search(e.text)
        if not m:
            continue
        key = (int(m.group(1)), int(m.group(2)), m.group(3))
        if any(k == key and abs(t - e.t) < 0.25 for k, t in seen):
            continue  # the host command echoed by the firmware
        seen.append((key, e.t))
        p = f"n{key[0]}c{key[1]}"
        val = float(m.group(3))
        if m.re is PSTEP_RE:
            ts, x = signals.fresh(run, p, "enc")
            if not len(ts):
                continue
            pre = ts <= e.t
            val += float(x[pre][-1] if pre.any() else x[0])
        starts.setdefault(p, []).append((e.t, val, "event" if e.source == "fw" else "mark"))
    stops = [e.t for e in run.events if STOP_RE.search(e.text) and not PSTEP_RE.search(e.text)]
    for p, lst in starts.items():
        lst.sort()
        w = []
        for k, (t0, tgt, src) in enumerate(lst):
            t1 = min([lst[k + 1][0] if k + 1 < len(lst) else float(run.t[-1])] + [s for s in stops if s > t0])
            w.append((t0, t1, tgt, src))
        out[p] = w
    return out


def loop_gain(run, p, wins, sat_margin=0.9):
    """Loop gains from the PWM the controller reports against the error it saw.

    Per window, only samples after the last one within 10 % of the duty cap are used:
    near the cap an anti-windup integrator runs in slivers too short for the CAN reads
    to see. From there, PWM = kp * e + ki * integral(e) + c_window, with the integral
    restarted at that point (trapezoids between reads) and a free constant per window
    holding whatever the integrator had built up. 'PI' when ki is clearly nonzero.
    """
    td, du = signals.fresh(run, p, "duty")
    ts, x = signals.fresh(run, p, "enc")
    if len(td) < 20 or len(ts) < 20:
        return None
    cap = float(np.nanmax(np.abs(du))) if len(du) else 0.0
    E, I, U, W = [], [], [], []
    for w, (t0, t1, tgt, _) in enumerate(wins):
        m = (td > t0 + 0.05) & (td < t1)
        if m.sum() < 8:
            continue
        tt, u = td[m], du[m]
        near = np.flatnonzero(np.abs(u) >= sat_margin * cap)
        k0 = near[-1] + 1 if len(near) else 0
        tt, u = tt[k0:], u[k0:]
        if len(tt) < 8:
            continue
        e = tgt - np.interp(tt, ts, x)
        dt = np.diff(tt, prepend=tt[0])
        e_prev = np.concatenate([[e[0]], e[:-1]])
        E += list(e)
        I += list(np.cumsum(0.5 * (e + e_prev) * dt))
        U += list(u)
        W += [w] * len(e)
    if len(E) < 10 or np.ptp(E) <= 0:
        return None
    E, I, U, W = map(np.array, (E, I, U, W))
    ws = np.unique(W)
    D = (W[:, None] == ws[None, :]).astype(float)

    def solve(cols):
        A = np.column_stack(cols + [D])
        coef, *_ = np.linalg.lstsq(A, U, rcond=None)
        r = U - A @ coef
        cov = np.linalg.pinv(A.T @ A) * (r @ r / max(len(r) - A.shape[1], 1))
        return coef, r, np.sqrt(np.clip(np.diag(cov), 0, None))

    coef, r, se = solve([E, I])
    ki, ki_se = float(coef[1]), float(se[1])
    pi = ki > 3 * ki_se and abs(ki) * float(np.std(I)) > 0.3
    if not pi:
        coef, r, se = solve([E])
    return {"kp_pct_per_count": float(coef[0]), "kp_se": float(se[0]), "ki_pct_per_count_s": ki if pi else 0.0,
            "ki_fit": ki, "ki_se": ki_se, "offset_pct": float(np.median(coef[2 if pi else 1:])),
            "resid_rms_pct": float(np.std(r)), "duty_cap_seen_pct": cap, "n": int(len(E)),
            "looks_like": "PI" if pi else "P"}


def _deadband(run, opts):
    cands = [Path(opts.deadband_json)] if opts.deadband_json else [run.path / "plant.json", run.path / "deadband.json"]
    for c in cands:
        if c.exists():
            try:
                return json.loads(c.read_text()), str(c)
            except ValueError:
                pass
    return None, None


def compute(run, opts) -> dict:
    res = header("pstep", run, opts)
    db, db_src = _deadband(run, opts)
    res["deadband_source"] = db_src
    chans = {}
    for p, wins in windows(run).items():
        ts, x = signals.fresh(run, p, "enc")
        td, du = signals.fresh(run, p, "duty")
        tc, cu = signals.fresh(run, p, "cur_ma")
        if len(ts) < 10:
            continue
        lg = loop_gain(run, p, wins)
        steps = []
        for t0, t1, target, src in wins:
            pre = ts < t0
            x0 = float(x[pre][-1]) if pre.any() else float(x[0])
            m = (ts >= t0) & (ts <= t1)
            if m.sum() < 5:
                continue
            r = position_step(ts[m], x[m], t0, x0, target, opts.tol_counts)
            tail = lambda tt: (tt >= t1 - 0.5) & (tt <= t1)  # noqa: E731
            r["pwm_rest_pct"] = float(np.median(du[tail(td)])) if tail(td).sum() else None
            r["i_rest_A"] = float(np.median(cu[tail(tc)]) / 1000) if tail(tc).sum() else None
            r["t_on"], r["t_end"], r["edge"] = t0, t1, src
            if lg and r["pwm_rest_pct"] is not None:
                r["kp_x_error_pct"] = lg["kp_pct_per_count"] * r["ss_error_counts"]
            if r["stalled"] and r["pwm_rest_pct"] is not None:
                dn = "pos" if r["ss_error_counts"] > 0 else "neg"
                e = (((db or {}).get("channels", {}) or {}).get(p, {}) or {}).get(dn, {})
                kin = e.get("kinetic_pct") or e.get("deadband_pct")
                brk = e.get("breakaway_pct")
                if kin is not None:
                    r["plant_kinetic_pct"] = kin
                if brk is not None:
                    r["plant_breakaway_pct"] = brk
            steps.append(r)
        if not steps:
            continue
        moved = [s for s in steps if s["moved"]]
        stalled = [s for s in steps if s["stalled"] and s["moved"]]
        summ = {"n": len(steps), "n_moved": len(moved), "n_stalled": len(stalled),
                "n_still_moving": sum(s["moving_at_end"] for s in steps),
                "median_abs_ss_error_counts": float(np.median([abs(s["ss_error_counts"]) for s in steps])),
                "max_overshoot_counts": float(max(s["overshoot_counts"] for s in steps)),
                "median_rise_actual_s": _med([s["rise_actual_s"] for s in moved]),
                "median_settle_rest_s": _med([s["settle_rest_s"] for s in moved])}
        for sgn, dn in ((1, "pos"), (-1, "neg")):
            st = [s for s in stalled if np.sign(s["ss_error_counts"]) == sgn and s["pwm_rest_pct"] is not None]
            if st:
                summ[f"stall_pwm_{dn}_pct"] = float(np.median([abs(s["pwm_rest_pct"]) for s in st]))
                summ[f"stall_error_{dn}_counts"] = float(np.median([abs(s["ss_error_counts"]) for s in st]))
            nm = [abs(s["pwm_rest_pct"]) for s in steps if not s["moved"] and s["pwm_rest_pct"] is not None
                  and np.sign(s["step_counts"]) == sgn]
            mv = [s for s in moved if np.sign(s["step_counts"]) == sgn]
            if nm:
                summ[f"held_without_moving_max_pwm_{dn}_pct"] = float(max(nm))
        chans[p] = {"name": run.short(p), "loop": lg, "steps": steps, "summary": summ}
        if stalled:
            s_ = summ
            res["flags"].append(
                f"note: {run.label(p)}: {len(stalled)} of {len(steps)} steps stopped short (median "
                f"{s_['median_abs_ss_error_counts']:.0f} counts) holding PWM +{s_.get('stall_pwm_pos_pct', float('nan')):.1f}"
                f" / -{s_.get('stall_pwm_neg_pct', float('nan')):.1f} %"
                + (f"; loop kp {lg['kp_pct_per_count']:.3f} %/count ({lg['looks_like']})" if lg else ""))
        if summ["n_still_moving"]:
            res["flags"].append(f"note: {run.label(p)}: {summ['n_still_moving']} step(s) still moving when the "
                                "target was dropped; hold longer to see where they stop")
    res["channels"] = chans
    if not chans:
        res["flags"].append("warning: no closed-loop steps: no _tgt column and no 'pstep'/'target' events or notes")
    else:
        p = next(iter(chans))
        s0 = chans[p]["steps"][0]
        ts, x = signals.fresh(run, p, "enc")
        m = (ts >= s0["t_on"]) & (ts <= s0["t_end"])
        res["trace"] = {"t": (ts[m] - s0["t_on"]).tolist(), "y": (x[m] - s0["x0"]).tolist(),
                        "ylabel": "Position change, counts", "xlabel": "Time from target, s"}
    return res


def _med(xs):
    xs = [v for v in xs if v is not None]
    return float(np.median(xs)) if xs else None


def plot(run, res, path, opts):
    chans = res["channels"]
    if not chans:
        fig, axs = ps.figure(1, 1)
        ps.empty(axs[0, 0], "No closed-loop steps in this run.")
        axs[0, 0].set_xlabel("Time, s")
        axs[0, 0].set_ylabel("Position, counts")
        return ps.save(fig, path)
    n = len(chans)
    fig, axs = ps.figure(n, 3, w=14, h_per=3.0, gridspec_kw={"width_ratios": [1.5, 1, 1]})
    col = ps.colors()
    for r, (p, d) in enumerate(chans.items()):
        ts, x = signals.fresh(run, p, "enc")
        td, du = signals.fresh(run, p, "duty")
        a, b, c = axs[r]
        for s in d["steps"]:
            m = (ts >= s["t_on"] - 0.2) & (ts <= s["t_end"])
            step = s["step_counts"] or 1
            colr = ps.series(0 if step > 0 else 1)
            a.plot(ts[m] - s["t_on"], (x[m] - s["x0"]) / step, color=colr, lw=1.2,
                   alpha=1.0 if s["moved"] else 0.45)
            md = (td >= s["t_on"] - 0.2) & (td <= s["t_end"])
            b.plot(td[md] - s["t_on"], du[md], color=colr, lw=1.0, drawstyle="steps-post",
                   alpha=1.0 if s["moved"] else 0.45)
        a.axhline(1.0, color=col["ink"], lw=0.8, linestyle=(0, (3, 2)))
        a.set_xlabel("Time from target, s")
        a.set_ylabel("Fraction of step, ratio")
        sm = d["summary"]
        ps.title(a, f"{run.short(p)} position steps",
                 f"{sm['n']} steps, {sm['n_stalled']} stopped short, median error {sm['median_abs_ss_error_counts']:.0f} counts")
        ps.legend_above(a, [("step +", ps.series(0)), ("step -", ps.series(1)), ("target", col["ink"])])
        b.set_xlabel("Time from target, s")
        b.set_ylabel("PWM, %")
        lg = d.get("loop") or {}
        ps.title(b, "Applied PWM", f"loop kp {lg.get('kp_pct_per_count', float('nan')):.3f} %/count, "
                 f"{lg.get('looks_like', '?')}; faded = never moved")
        # stall map: PWM at rest against error at rest
        st = [s for s in d["steps"] if s["pwm_rest_pct"] is not None]
        for s in st:
            cc = ps.series(0 if s["step_counts"] > 0 else 1)
            c.plot(s["ss_error_counts"], s["pwm_rest_pct"], "o", color=cc, ms=5,
                   mfc=cc if s["moved"] else "none", mec=cc)
        if lg.get("kp_pct_per_count") is not None and st:
            ee = np.linspace(min(s["ss_error_counts"] for s in st), max(s["ss_error_counts"] for s in st), 10)
            c.plot(ee, lg["kp_pct_per_count"] * ee + lg["offset_pct"], color=col["ink"], lw=1.0)
        c.set_xlabel("Error at rest, counts")
        c.set_ylabel("PWM at rest, %")
        ps.title(c, "Where it stops", "hollow = never moved; line = kp x error")
    ps.page_title(fig, "Closed-loop position steps", meta_line(run))
    return ps.save(fig, path)


def _s(x):
    return "-" if x is None else f"{x:.2f}"


def summary_lines(res):
    out = []
    for p, d in res["channels"].items():
        lg = d.get("loop") or {}
        if lg:
            out.append(f"{d['name']:>6} loop: kp {lg['kp_pct_per_count']:.3f} %/count, ki {lg['ki_pct_per_count_s']:.3f} "
                       f"%/(count s) ({lg['looks_like']}), duty cap seen "
                       f"{lg['duty_cap_seen_pct']:.1f} %")
        for s in d["steps"]:
            tag = "  STALLED" if s["stalled"] and s["moved"] else ("  NO MOTION" if not s["moved"] else (
                "  STILL MOVING" if s["moving_at_end"] else ""))
            out.append(f"{d['name']:>6} {s['step_counts']:+5.0f}: error {s['ss_error_counts']:+4.0f}  overshoot "
                       f"{s['overshoot_counts']:.0f}  rise {_s(s['rise_actual_s'])} s  settle {_s(s['settle_rest_s'])} s  "
                       f"PWM at rest {s['pwm_rest_pct'] if s['pwm_rest_pct'] is None else round(s['pwm_rest_pct'], 1)} % "
                       f"(kp x err {_s(s.get('kp_x_error_pct'))})  I {_s(s['i_rest_A'])} A{tag}")
    return out
