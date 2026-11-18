"""smoothness: what the rider feels during each move.

At the IMU (on the seat), in the chassis frame: jerk = d/dt of linear acceleration
and angular acceleration = d/dt of angular rate, both after a zero-phase
low-pass at fc_hz (default 5 Hz; differentiation amplifies sensor noise above the
motion's own bandwidth). Per move (excitation start to end + settle_s): RMS and
peak of the jerk magnitude and the angular acceleration magnitude. The noise
floor (the same RMS over the run's still periods) is removed in quadrature in the
*_corrected numbers, which are the ones to compare between runs.
"""

from __future__ import annotations

import numpy as np

from .. import attitude, plotstyle as ps, signals
from .common import header, meta_line


def _signals(run, opts):
    att = attitude.attitude(run, opts.mount)
    if "la_c" not in att or "gyro_c" not in att:
        return None
    tg, la, fs = signals.uniform(run.t, att["la_c"])
    _, w, _ = signals.uniform(run.t, att["gyro_c"], fs)
    la = signals.lowpass(la, fs, opts.fc_hz)
    w = signals.lowpass(w, fs, opts.fc_hz)
    jerk = np.linalg.norm(np.gradient(la, 1 / fs, axis=0), axis=1)
    jz = np.abs(np.gradient(la[:, 2], 1 / fs))
    aacc = np.degrees(np.linalg.norm(np.gradient(w, 1 / fs, axis=0), axis=1))
    rate = np.degrees(np.linalg.norm(w[:, :2], axis=1))
    return {"t": tg, "jerk": jerk, "jerk_z": jz, "aacc": aacc, "la": np.linalg.norm(la, axis=1), "rate": rate, "fs": fs}


def compute(run, opts) -> dict:
    res = header("smoothness", run, opts, {"fc_hz": opts.fc_hz, "settle_s": opts.settle_s})
    s = _signals(run, opts)
    mv = signals.moves(run, settle_s=opts.settle_s, source=opts.source)
    res["moves"] = []
    if s is None or not mv:
        res["flags"].append("warning: no IMU linear acceleration / gyro, or no moves")
        res["summary"] = {}
        return res
    t = s["t"]
    busy = np.zeros(len(t), dtype=bool)
    for a, b, _ in mv:
        busy |= (t >= a - 0.5) & (t <= b + 0.5)
    quiet = ~busy
    edge = int(s["fs"])  # ignore filter edge effects
    quiet[:edge] = quiet[-edge:] = False
    floor = {k: float(np.sqrt(np.mean(s[k][quiet] ** 2))) if quiet.sum() > 50 else 0.0 for k in ("jerk", "aacc", "jerk_z")}
    res["noise_floor"] = {"rms_jerk": floor["jerk"], "rms_ang_acc": floor["aacc"], "rms_jerk_z": floor["jerk_z"],
                          "quiet_s": float(quiet.sum() / s["fs"])}
    for a, b, ps_ in mv:
        m = (t >= a) & (t <= b)
        if m.sum() < 5:
            continue
        rec = {"t_on": a, "t_end": b, "axes": ps_, "names": [run.short(p) for p in ps_]}
        for k, key in (("jerk", "jerk"), ("aacc", "ang_acc"), ("jerk_z", "jerk_z")):
            rms = float(np.sqrt(np.mean(s[k][m] ** 2)))
            rec[f"rms_{key}"] = rms
            rec[f"rms_{key}_corrected"] = float(np.sqrt(max(rms ** 2 - floor[k] ** 2, 0.0)))
            rec[f"peak_{key}"] = float(s[k][m].max())
            rec[f"snr_{key}"] = rms / floor[k] if floor[k] > 0 else None
        rec["peak_accel"] = float(s["la"][m].max())
        rec["peak_rate_dps"] = float(s["rate"][m].max())
        res["moves"].append(rec)
    mvs = res["moves"]
    res["summary"] = {k: float(np.mean([r[k] for r in mvs])) for k in
                      ("rms_jerk", "rms_jerk_corrected", "peak_jerk", "rms_ang_acc", "rms_ang_acc_corrected",
                       "peak_ang_acc", "peak_accel", "peak_rate_dps")} if mvs else {}
    low = [r for r in mvs if r["snr_jerk"] is not None and r["snr_jerk"] < 1.5]
    if low:
        res["flags"].append(f"note: {len(low)} of {len(mvs)} moves have jerk within 1.5x the noise floor; "
                            "compare the corrected numbers, or lower fc_hz")
    # a compact trace of the first move for compare overlays
    if mvs:
        a, b = mvs[0]["t_on"], mvs[0]["t_end"]
        m = (t >= a - 0.3) & (t <= b)
        res["trace"] = {"t": (t[m] - a)[::2].tolist(), "y": s["jerk"][m][::2].tolist(), "ylabel": "Jerk, m/s³",
                        "xlabel": "Time from move start, s"}
    return res


def plot(run, res, path, opts):
    s = _signals(run, opts)
    fig, axs = ps.figure(2, 2, w=13, h_per=3.2, gridspec_kw={"width_ratios": [2.2, 1]})
    mvs = res.get("moves", [])
    for r, (k, unit, ttl, fl) in enumerate((("jerk", "Jerk, m/s³", "Seat jerk", "rms_jerk"),
                                            ("aacc", "Angular accel, deg/s²", "Angular acceleration", "rms_ang_acc"))):
        a = axs[r, 0]
        if s is not None:
            k_ = max(1, len(s["t"]) // 8000)
            a.plot(s["t"][::k_], s[k][::k_], color=ps.series(0), lw=1.0)
            for mv in mvs:
                a.axvspan(mv["t_on"], mv["t_end"], color=ps.colors()["grid"], alpha=0.6, lw=0, zorder=0)
            nf = res.get("noise_floor", {}).get(fl)
            if nf:
                ps.ref_line(a, nf, f"noise floor {nf:.2f}", "off")
        a.set_xlabel("Time, s")
        a.set_ylabel(unit)
        ps.title(a, ttl, f"magnitude, chassis frame, low-pass {opts.fc_hz:g} Hz; shaded = moves")
        b = axs[r, 1]
        key = "rms_jerk_corrected" if k == "jerk" else "rms_ang_acc_corrected"
        if mvs:
            labels = [f"{i + 1} {'+'.join(m['names'])}" for i, m in enumerate(mvs)][:24]
            vals = [round(m[key], 2) for m in mvs][:24]
            ps.bars(b, labels, vals, xlabel=unit.replace(",", " RMS,"), ylabel="Move")
        else:
            ps.empty(b, "No moves in this run.")
            b.set_xlabel(unit)
            b.set_ylabel("Move")
        ps.title(b, "Per move", "RMS, noise floor removed")
    ps.page_title(fig, "Ride smoothness", meta_line(run))
    return ps.save(fig, path)


def summary_lines(res):
    s = res.get("summary") or {}
    if not s:
        return ["no moves"]
    nf = res["noise_floor"]
    return [f"{len(res['moves'])} moves  RMS jerk {s['rms_jerk_corrected']:.3f} m/s3 (raw {s['rms_jerk']:.3f}, floor "
            f"{nf['rms_jerk']:.3f})  peak {s['peak_jerk']:.3f}  RMS ang acc {s['rms_ang_acc_corrected']:.2f} deg/s2 "
            f"(floor {nf['rms_ang_acc']:.2f})  peak {s['peak_ang_acc']:.2f}  peak rate {s['peak_rate_dps']:.2f} deg/s"]
