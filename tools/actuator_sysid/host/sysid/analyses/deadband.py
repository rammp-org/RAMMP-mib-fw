"""deadband: the duty that starts motion from rest (breakaway), the duty under which a
moving axis stops (kinetic), per direction, and the hysteresis between them.

Breakaway comes from small steps from rest: a bracket [largest duty that did not
move, smallest that did]; the estimate is its midpoint. Kinetic comes from every
plateau where the axis was moving: steady speed v = K * (|duty| - u_kin) is fitted
and u_kin is where it meets zero, plus a bracket from a descending staircase
(a step sent while the previous one still runs).
"""

from __future__ import annotations

import numpy as np

from .. import plotstyle as ps, signals
from .common import dir_name, header, meta_line, slope


def compute(run, opts) -> dict:
    res = header("deadband", run, opts)
    chans = {}
    for p in run.channels():
        src = signals.excitation_source(run, p, opts.source)
        pls = signals.plateaus(run, p, src, min_dur=0.3)
        ts, x = signals.fresh(run, p, "enc")
        tc, cur = signals.fresh(run, p, "cur_ma")
        if not pls or len(ts) < 10:
            continue
        recs = []
        moving_before = False
        for k, pl in enumerate(pls):
            nxt = pls[k + 1].t_on if k + 1 < len(pls) else float(run.t[-1])
            contiguous = (not pl.from_rest) and k > 0 and abs(pls[k - 1].t_off - pl.t_on) < 0.05
            start_moving = contiguous and moving_before and np.sign(pls[k - 1].duty) == pl.sign
            half = (ts >= pl.t_on + 0.5 * pl.dur) & (ts <= pl.t_off)
            v = slope(ts[half], x[half]) if half.sum() >= 3 else np.nan
            disp_half = float(np.ptp(x[half])) if half.sum() else 0.0
            w = (ts >= pl.t_on) & (ts <= min(pl.t_off + 0.3, nxt))
            disp = float(np.ptp(x[w])) if w.sum() else 0.0
            if start_moving:
                moved = bool(disp_half >= 2 and abs(v) >= 1.0 and np.sign(v) == pl.sign)
            else:
                moved = bool(disp >= opts.move_thresh)
            mc = (tc >= pl.t_on + 0.5 * pl.dur) & (tc <= pl.t_off)
            rec = {"t_on": pl.t_on, "duty": pl.duty, "dir": dir_name(pl.sign), "start": "moving" if start_moving else "rest",
                   "moved": moved, "v": float(v) if np.isfinite(v) else None, "disp": disp,
                   "i_A": float(np.median(np.abs(cur[mc])) / 1000) if mc.sum() else None}
            recs.append(rec)
            moving_before = moved and abs(v) > 0.5 if np.isfinite(v) else moved
        d = {"name": run.short(p), "source": src, "plateaus": recs}
        for sgn in (1, -1):
            dn = dir_name(sgn)
            rest = [r for r in recs if r["dir"] == dn and r["start"] == "rest"]
            e = {}
            if rest:
                mv = sorted(abs(r["duty"]) for r in rest if r["moved"])
                st = sorted(abs(r["duty"]) for r in rest if not r["moved"])
                lo = max([u for u in st if not mv or u < mv[0]], default=None)
                hi = mv[0] if mv else None
                e["breakaway_bracket_pct"] = [lo, hi]
                e["breakaway_pct"] = (0.5 * (lo + hi) if lo is not None and hi is not None else hi)
                e["breakaway_inconsistent"] = int(sum(1 for u in st if mv and u > mv[0]))
                stall = [r for r in rest if not r["moved"] and r["i_A"] is not None and abs(r["duty"]) > 0]
                if len(stall) >= 2:
                    e["stall_current_A_per_pct"] = float(np.polyfit([abs(r["duty"]) for r in stall],
                                                                    [r["i_A"] for r in stall], 1)[0])
            movers = [r for r in recs if r["dir"] == dn and r["moved"] and r["v"] is not None and abs(r["v"]) >= 3]
            if len({abs(r["duty"]) for r in movers}) >= 2:
                u = np.array([abs(r["duty"]) for r in movers])
                vv = np.array([abs(r["v"]) for r in movers])
                K, b = np.polyfit(u, vv, 1)
                e["gain"] = float(K)
                e["kinetic_pct"] = float(-b / K) if K > 0 else None
            stair = [r for r in recs if r["dir"] == dn and r["start"] == "moving"]
            if stair:
                mv = [abs(r["duty"]) for r in stair if r["moved"]]
                stp = [abs(r["duty"]) for r in stair if not r["moved"]]
                e["kinetic_bracket_pct"] = [max([u for u in stp if not mv or u < min(mv)], default=None),
                                            min(mv) if mv else None]
            if e.get("breakaway_pct") is not None and e.get("kinetic_pct") is not None:
                e["hysteresis_pct"] = e["breakaway_pct"] - e["kinetic_pct"]
            if e:
                d[dn] = e
            if e.get("breakaway_inconsistent"):
                res["flags"].append(f"warning: {run.label(p)} {dn}: {e['breakaway_inconsistent']} step(s) above the "
                                    "breakaway duty did not move (stiction varies along the stroke?)")
            if rest and e.get("breakaway_bracket_pct", [None, None])[1] is None:
                res["flags"].append(f"note: {run.label(p)} {dn}: no step from rest moved; breakaway is above "
                                    f"{max(abs(r['duty']) for r in rest)} %")
        chans[p] = d
    res["channels"] = chans
    if not chans:
        from ..signals import has_tgt

        res["flags"].append("note: closed-loop (pstep) run: use 'analyze.py plant' for the deadband"
                            if has_tgt(run) else "warning: no open-loop duty plateaus found")
    return res


def plot(run, res, path, opts):
    chans = list(res["channels"])
    n = max(1, len(chans))
    cols = 2 if n > 1 else 1
    rows = (n + cols - 1) // cols
    fig, axs = ps.figure(rows, cols, w=12, h_per=3.2)
    for k in range(rows * cols):
        a = axs[k // cols, k % cols]
        if k >= len(chans):
            a.set_visible(False)
            continue
        p = chans[k]
        d = res["channels"][p]
        for start, c, mk in (("rest", ps.series(0), "o"), ("moving", ps.series(2), "D")):
            rs = [r for r in d["plateaus"] if r["start"] == start]
            mv = [r for r in rs if r["moved"]]
            st = [r for r in rs if not r["moved"]]
            a.plot([r["duty"] for r in mv], [r["v"] or 0 for r in mv], mk, color=c, ms=5)
            a.plot([r["duty"] for r in st], [r["v"] or 0 for r in st], mk, mfc="none", mec=c, ms=5)
        for sgn, dn in ((1, "pos"), (-1, "neg")):
            e = d.get(dn, {})
            if e.get("kinetic_pct") is not None:
                uu = np.linspace(e["kinetic_pct"], max(abs(r["duty"]) for r in d["plateaus"]), 20)
                a.plot(sgn * uu, sgn * e["gain"] * (uu - e["kinetic_pct"]), color=ps.colors()["ink"], lw=1.0)
                ps.ref_line(a, sgn * e["kinetic_pct"], f"kin {e['kinetic_pct']:.1f}", "good", vertical=True, bottom=True)
            if e.get("breakaway_pct") is not None:
                ps.ref_line(a, sgn * e["breakaway_pct"], f"brk {e['breakaway_pct']:.1f}", "warning", vertical=True)
        a.set_xlabel("Duty, %")
        a.set_ylabel("Steady speed, counts/s")
        pos, neg = d.get("pos", {}), d.get("neg", {})
        ps.title(a, f"{run.short(p)} deadband",
                 f"breakaway +{_f(pos.get('breakaway_pct'))} / -{_f(neg.get('breakaway_pct'))} %, kinetic "
                 f"+{_f(pos.get('kinetic_pct'))} / -{_f(neg.get('kinetic_pct'))} %")
        if k == 0:
            ps.legend_above(a, [("from rest", ps.series(0)), ("while moving", ps.series(2))])
    ps.page_title(fig, "Deadband and stiction", meta_line(run) + "  |  hollow = did not move")
    return ps.save(fig, path)


def _f(x):
    return "-" if x is None else f"{x:.1f}"


def summary_lines(res):
    out = []
    for p, d in res["channels"].items():
        for dn in ("pos", "neg"):
            e = d.get(dn)
            if e:
                out.append(f"{d['name']:>6} {dn}: breakaway {_f(e.get('breakaway_pct'))} % {e.get('breakaway_bracket_pct')}  "
                           f"kinetic {_f(e.get('kinetic_pct'))} % {e.get('kinetic_bracket_pct', '')}  "
                           f"hysteresis {_f(e.get('hysteresis_pct'))} %")
    return out
